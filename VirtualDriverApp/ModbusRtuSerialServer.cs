using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Threading;

internal sealed class ModbusRtuSerialServer : IDisposable
{
    private const int ReadTimeoutMilliseconds = 250;
    private const int WriteTimeoutMilliseconds = 1000;
    private const int InitialReconnectDelayMilliseconds = 500;
    private const int MaximumReconnectDelayMilliseconds = 10000;
    private const int MaximumReceiveBufferSize = 4096;
    private const int ErrorLogIntervalSeconds = 30;

    private readonly object lifecycleLock = new object();
    private readonly Func<byte[], byte[]> requestHandler;
    private readonly List<byte> receiveBuffer = new List<byte>(256);
    private readonly ManualResetEventSlim stopSignal =
        new ManualResetEventSlim(false);

    private SerialPortConfiguration configuration;
    private SerialPort serialPort;
    private Thread receiveThread;
    private volatile bool stopRequested;
    private DateTime lastErrorLogUtc = DateTime.MinValue;
    private int suppressedErrorCount;

    public ModbusRtuSerialServer(Func<byte[], byte[]> requestHandler)
    {
        if (requestHandler == null)
        {
            throw new ArgumentNullException(nameof(requestHandler));
        }

        this.requestHandler = requestHandler;
    }

    public void ConfigureAndOpen(
        string portName,
        int baudRate,
        Parity parity,
        int dataBits,
        StopBits stopBits)
    {
        SerialPortConfiguration newConfiguration =
            new SerialPortConfiguration(
                portName,
                baudRate,
                parity,
                dataBits,
                stopBits);

        bool restartAfterConfiguration;
        lock (lifecycleLock)
        {
            restartAfterConfiguration =
                receiveThread != null &&
                receiveThread.IsAlive;

            if (configuration != null &&
                configuration.Equals(newConfiguration) &&
                restartAfterConfiguration)
            {
                return;
            }
        }

        if (restartAfterConfiguration)
        {
            Stop();
        }

        lock (lifecycleLock)
        {
            configuration = newConfiguration;
            ClosePortUnsafe();
            OpenPortUnsafe();
        }

        if (restartAfterConfiguration)
        {
            Start();
        }
    }

    public void Start()
    {
        lock (lifecycleLock)
        {
            if (receiveThread != null && receiveThread.IsAlive)
            {
                return;
            }

            if (configuration == null)
            {
                throw new InvalidOperationException(
                    "必须先设置串口参数，再启动 Modbus RTU 从站。");
            }

            if (serialPort == null || !serialPort.IsOpen)
            {
                ClosePortUnsafe();
                OpenPortUnsafe();
            }

            receiveBuffer.Clear();
            stopRequested = false;
            stopSignal.Reset();

            receiveThread = new Thread(ReceiveLoop)
            {
                IsBackground = true,
                Name = "ModbusRtuSerialReceive"
            };
            receiveThread.Start();
        }
    }

    public void Stop()
    {
        Thread threadToJoin;
        lock (lifecycleLock)
        {
            stopRequested = true;
            stopSignal.Set();
            threadToJoin = receiveThread;

            // 关闭串口可以立即解除正在等待的阻塞读取。
            ClosePortUnsafe();
        }

        bool threadStopped =
            threadToJoin == null ||
            threadToJoin == Thread.CurrentThread ||
            threadToJoin.Join(3000);
        if (!threadStopped)
        {
            LogHelper.Logger.Warn(
                "Modbus RTU 串口接收线程未能在 3 秒内退出。");
        }

        lock (lifecycleLock)
        {
            bool receiveThreadStopped =
                threadToJoin == null ||
                !threadToJoin.IsAlive;
            if (receiveThread == threadToJoin &&
                receiveThreadStopped)
            {
                receiveThread = null;
            }

            if (receiveThreadStopped)
            {
                receiveBuffer.Clear();
            }
        }
    }

    public void Dispose()
    {
        Stop();
        lock (lifecycleLock)
        {
            if (receiveThread == null)
            {
                stopSignal.Dispose();
            }
        }
    }

    private void ReceiveLoop()
    {
        byte[] readBuffer = new byte[256];
        int reconnectDelayMilliseconds =
            InitialReconnectDelayMilliseconds;

        while (!stopRequested)
        {
            SerialPort port = null;
            try
            {
                port = GetOrOpenPort();
                int requestedByteCount = Math.Max(
                    1,
                    Math.Min(
                        readBuffer.Length,
                        port.BytesToRead));
                int receivedByteCount = port.Read(
                    readBuffer,
                    0,
                    requestedByteCount);

                if (receivedByteCount <= 0)
                {
                    continue;
                }

                reconnectDelayMilliseconds =
                    InitialReconnectDelayMilliseconds;
                ProcessReceivedBytes(
                    port,
                    readBuffer,
                    receivedByteCount);
            }
            catch (TimeoutException)
            {
                // 读超时是空闲串口的正常状态，用于定期检查退出信号。
            }
            catch (Exception ex)
            {
                if (stopRequested)
                {
                    break;
                }

                LogCommunicationError(ex);
                InvalidatePort(port);
                receiveBuffer.Clear();

                if (stopSignal.Wait(reconnectDelayMilliseconds))
                {
                    break;
                }

                reconnectDelayMilliseconds = Math.Min(
                    MaximumReconnectDelayMilliseconds,
                    reconnectDelayMilliseconds * 2);
            }
        }
    }

    private SerialPort GetOrOpenPort()
    {
        lock (lifecycleLock)
        {
            if (stopRequested)
            {
                throw new OperationCanceledException();
            }

            if (serialPort == null || !serialPort.IsOpen)
            {
                ClosePortUnsafe();
                OpenPortUnsafe();
                LogHelper.Logger.Info(
                    "Modbus RTU 串口已恢复连接：{0}",
                    configuration.PortName);
            }

            return serialPort;
        }
    }

    private void ProcessReceivedBytes(
        SerialPort port,
        byte[] bytes,
        int count)
    {
        if (receiveBuffer.Count + count >
            MaximumReceiveBufferSize)
        {
            int bytesToDiscard = receiveBuffer.Count + count -
                MaximumReceiveBufferSize;
            if (bytesToDiscard >= receiveBuffer.Count)
            {
                receiveBuffer.Clear();
            }
            else
            {
                receiveBuffer.RemoveRange(0, bytesToDiscard);
            }

            LogHelper.Logger.Warn(
                "Modbus RTU 接收缓冲区超过上限，已丢弃最旧数据。");
        }

        for (int index = 0; index < count; index++)
        {
            receiveBuffer.Add(bytes[index]);
        }

        byte[] request;
        while (ModbusRtuProtocol.TryExtractRequestFrame(
            receiveBuffer,
            out request))
        {
            byte[] response;
            try
            {
                response = requestHandler(request);
            }
            catch (Exception ex)
            {
                LogHelper.Logger.Error(
                    ex,
                    "处理 Modbus RTU 请求时发生未预期异常，请求：{0}",
                    BitConverter.ToString(request));
                continue;
            }

            if (response == null || response.Length == 0)
            {
                continue;
            }

            try
            {
                port.Write(response, 0, response.Length);
            }
            catch (Exception ex)
            {
                throw new IOException(
                    "写入 Modbus RTU 响应失败。",
                    ex);
            }
        }
    }

    private void OpenPortUnsafe()
    {
        if (configuration == null)
        {
            throw new InvalidOperationException(
                "尚未配置串口参数。");
        }

        SerialPort newPort = new SerialPort(
            configuration.PortName,
            configuration.BaudRate,
            configuration.Parity,
            configuration.DataBits,
            configuration.StopBits)
        {
            Handshake = Handshake.None,
            DtrEnable = false,
            RtsEnable = false,
            ReadBufferSize = 4096,
            WriteBufferSize = 2048,
            ReadTimeout = ReadTimeoutMilliseconds,
            WriteTimeout = WriteTimeoutMilliseconds,
            ReceivedBytesThreshold = 1
        };

        try
        {
            newPort.Open();
            serialPort = newPort;
            LogHelper.Logger.Info(
                "Modbus RTU 串口已打开：{0}，{1}，{2}，{3}，{4}",
                configuration.PortName,
                configuration.BaudRate,
                configuration.Parity,
                configuration.DataBits,
                configuration.StopBits);
        }
        catch
        {
            newPort.Dispose();
            throw;
        }
    }

    private void InvalidatePort(SerialPort failedPort)
    {
        lock (lifecycleLock)
        {
            if (failedPort == null ||
                ReferenceEquals(serialPort, failedPort))
            {
                ClosePortUnsafe();
            }
        }
    }

    private void ClosePortUnsafe()
    {
        SerialPort portToClose = serialPort;
        serialPort = null;

        if (portToClose == null)
        {
            return;
        }

        try
        {
            if (portToClose.IsOpen)
            {
                portToClose.Close();
            }
        }
        catch (Exception ex)
        {
            LogHelper.Logger.Debug(
                ex,
                "关闭 Modbus RTU 串口时出现异常。");
        }
        finally
        {
            try
            {
                portToClose.Dispose();
            }
            catch (Exception ex)
            {
                LogHelper.Logger.Debug(
                    ex,
                    "释放 Modbus RTU 串口时出现异常。");
            }
        }
    }

    private void LogCommunicationError(Exception ex)
    {
        DateTime nowUtc = DateTime.UtcNow;
        if ((nowUtc - lastErrorLogUtc).TotalSeconds <
            ErrorLogIntervalSeconds)
        {
            suppressedErrorCount++;
            return;
        }

        if (suppressedErrorCount > 0)
        {
            LogHelper.Logger.Warn(
                "Modbus RTU 串口通信异常仍在持续，期间已合并 {0} 条重复日志。",
                suppressedErrorCount);
        }

        LogHelper.Logger.Warn(
            ex,
            "Modbus RTU 串口通信异常，将自动重连。");
        lastErrorLogUtc = nowUtc;
        suppressedErrorCount = 0;
    }

    private sealed class SerialPortConfiguration
    {
        public SerialPortConfiguration(
            string portName,
            int baudRate,
            Parity parity,
            int dataBits,
            StopBits stopBits)
        {
            if (string.IsNullOrWhiteSpace(portName))
            {
                throw new ArgumentException(
                    "串口名称不能为空。",
                    nameof(portName));
            }

            if (baudRate <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(baudRate));
            }

            if (dataBits < 5 || dataBits > 8)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(dataBits));
            }

            if (stopBits == StopBits.None)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(stopBits));
            }

            PortName = portName.Trim();
            BaudRate = baudRate;
            Parity = parity;
            DataBits = dataBits;
            StopBits = stopBits;
        }

        public string PortName { get; }

        public int BaudRate { get; }

        public Parity Parity { get; }

        public int DataBits { get; }

        public StopBits StopBits { get; }

        public override bool Equals(object obj)
        {
            SerialPortConfiguration other =
                obj as SerialPortConfiguration;
            return other != null &&
                string.Equals(
                    PortName,
                    other.PortName,
                    StringComparison.OrdinalIgnoreCase) &&
                BaudRate == other.BaudRate &&
                Parity == other.Parity &&
                DataBits == other.DataBits &&
                StopBits == other.StopBits;
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hashCode =
                    StringComparer.OrdinalIgnoreCase.GetHashCode(
                        PortName);
                hashCode = (hashCode * 397) ^ BaudRate;
                hashCode = (hashCode * 397) ^ (int)Parity;
                hashCode = (hashCode * 397) ^ DataBits;
                hashCode = (hashCode * 397) ^ (int)StopBits;
                return hashCode;
            }
        }
    }
}

internal static class ModbusRtuProtocol
{
    public const int RequestFrameLength = 8;

    public static bool TryExtractRequestFrame(
        List<byte> receiveBuffer,
        out byte[] frame)
    {
        if (receiveBuffer == null)
        {
            throw new ArgumentNullException(nameof(receiveBuffer));
        }

        frame = null;
        if (receiveBuffer.Count < RequestFrameLength)
        {
            return false;
        }

        int latestStart =
            receiveBuffer.Count - RequestFrameLength;
        for (int start = 0; start <= latestStart; start++)
        {
            byte address = receiveBuffer[start];
            if (address == 0 || address > 247)
            {
                continue;
            }

            if (!HasValidCrc(
                receiveBuffer,
                start,
                RequestFrameLength))
            {
                continue;
            }

            if (start > 0)
            {
                receiveBuffer.RemoveRange(0, start);
            }

            frame = receiveBuffer
                .GetRange(0, RequestFrameLength)
                .ToArray();
            receiveBuffer.RemoveRange(
                0,
                RequestFrameLength);
            return true;
        }

        // 最多保留 7 个尾部字节，便于下一次读取后拼成完整请求。
        int bytesToDiscard =
            receiveBuffer.Count - (RequestFrameLength - 1);
        receiveBuffer.RemoveRange(0, bytesToDiscard);
        return false;
    }

    public static bool HasValidCrc(byte[] frame)
    {
        return frame != null &&
            frame.Length >= 4 &&
            HasValidCrc(frame, 0, frame.Length);
    }

    public static byte[] AppendCrc(byte[] payload)
    {
        if (payload == null)
        {
            throw new ArgumentNullException(nameof(payload));
        }

        byte[] frame = new byte[payload.Length + 2];
        Buffer.BlockCopy(
            payload,
            0,
            frame,
            0,
            payload.Length);

        ushort crc = CalculateCrc(
            payload,
            0,
            payload.Length);
        frame[frame.Length - 2] = (byte)(crc & 0xFF);
        frame[frame.Length - 1] = (byte)(crc >> 8);
        return frame;
    }

    private static bool HasValidCrc(
        IList<byte> bytes,
        int offset,
        int count)
    {
        ushort calculatedCrc = CalculateCrc(
            bytes,
            offset,
            count - 2);
        return bytes[offset + count - 2] ==
                (byte)(calculatedCrc & 0xFF) &&
            bytes[offset + count - 1] ==
                (byte)(calculatedCrc >> 8);
    }

    private static ushort CalculateCrc(
        IList<byte> bytes,
        int offset,
        int count)
    {
        ushort crc = 0xFFFF;
        int end = offset + count;

        for (int byteIndex = offset;
            byteIndex < end;
            byteIndex++)
        {
            crc ^= bytes[byteIndex];

            for (int bitIndex = 0;
                bitIndex < 8;
                bitIndex++)
            {
                bool leastSignificantBitSet =
                    (crc & 0x0001) != 0;
                crc >>= 1;
                if (leastSignificantBitSet)
                {
                    crc ^= 0xA001;
                }
            }
        }

        return crc;
    }
}
