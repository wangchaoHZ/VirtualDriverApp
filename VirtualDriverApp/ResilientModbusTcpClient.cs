using FluentModbus;
using System;
using System.IO;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;

public enum ModbusTcpConnectionState
{
    Disconnected,
    Connecting,
    Connected,
    Faulted,
    Stopped
}

public sealed class ModbusTcpHealthSnapshot
{
    public string DeviceName { get; internal set; }

    public string Endpoint { get; internal set; }

    public ModbusTcpConnectionState State { get; internal set; }

    public int ConsecutiveFailures { get; internal set; }

    public int ReconnectCount { get; internal set; }

    public DateTime? LastSuccessUtc { get; internal set; }

    public DateTime? LastFailureUtc { get; internal set; }

    public string LastError { get; internal set; }
}

public sealed class ModbusTcpCommunicationException : Exception
{
    public ModbusTcpCommunicationException(
        string deviceName,
        string operationName,
        Exception innerException)
        : base(
            string.Format(
                "{0} 执行“{1}”失败。",
                deviceName,
                operationName),
            innerException)
    {
    }
}

public sealed class ResilientModbusTcpClient : IDisposable
{
    private const int MaximumAttempts = 3;
    private const int ConnectTimeoutMilliseconds = 1500;
    private const int ReadTimeoutMilliseconds = 1500;
    private const int WriteTimeoutMilliseconds = 1500;
    private const int FailureLogIntervalSeconds = 30;

    private static readonly int[] RetryDelaysMilliseconds =
    {
        200,
        500
    };

    private readonly object healthLock = new object();
    private readonly SemaphoreSlim operationGate =
        new SemaphoreSlim(1, 1);
    private readonly CancellationTokenSource stopSource =
        new CancellationTokenSource();
    private readonly string deviceName;
    private readonly string endpoint;
    private readonly ModbusEndianness endianness;

    private ModbusTcpClient client;
    private volatile bool disposeRequested;
    private ModbusTcpConnectionState state =
        ModbusTcpConnectionState.Disconnected;
    private int consecutiveFailures;
    private int reconnectCount;
    private bool hasConnectedBefore;
    private DateTime? lastSuccessUtc;
    private DateTime? lastFailureUtc;
    private DateTime lastFailureLogUtc = DateTime.MinValue;
    private string lastError;

    public ResilientModbusTcpClient(
        string deviceName,
        string endpoint,
        ModbusEndianness endianness)
    {
        if (string.IsNullOrWhiteSpace(deviceName))
        {
            throw new ArgumentException(
                "设备名称不能为空。",
                nameof(deviceName));
        }

        if (string.IsNullOrWhiteSpace(endpoint))
        {
            throw new ArgumentException(
                "设备地址不能为空。",
                nameof(endpoint));
        }

        this.deviceName = deviceName.Trim();
        this.endpoint = endpoint.Trim();
        this.endianness = endianness;
    }

    public Task ConnectAsync(CancellationToken cancellationToken)
    {
        return ExecuteAsync(
            "建立连接",
            activeClient => { },
            cancellationToken);
    }

    public async Task ExecuteAsync(
        string operationName,
        Action<ModbusTcpClient> operation,
        CancellationToken cancellationToken)
    {
        if (operation == null)
        {
            throw new ArgumentNullException(nameof(operation));
        }

        await ExecuteAsync(
            operationName,
            activeClient =>
            {
                operation(activeClient);
                return true;
            },
            cancellationToken).ConfigureAwait(false);
    }

    public async Task<T> ExecuteAsync<T>(
        string operationName,
        Func<ModbusTcpClient, T> operation,
        CancellationToken cancellationToken)
    {
        if (string.IsNullOrWhiteSpace(operationName))
        {
            throw new ArgumentException(
                "操作名称不能为空。",
                nameof(operationName));
        }

        if (operation == null)
        {
            throw new ArgumentNullException(nameof(operation));
        }

        ThrowIfDisposed();

        using (CancellationTokenSource linkedSource =
            CancellationTokenSource.CreateLinkedTokenSource(
                cancellationToken,
                stopSource.Token))
        {
            return await Task.Run(
                () => ExecuteWithRetry(
                    operationName,
                    operation,
                    linkedSource.Token),
                linkedSource.Token).ConfigureAwait(false);
        }
    }

    public ModbusTcpHealthSnapshot GetHealthSnapshot()
    {
        lock (healthLock)
        {
            return new ModbusTcpHealthSnapshot
            {
                DeviceName = deviceName,
                Endpoint = endpoint,
                State = state,
                ConsecutiveFailures = consecutiveFailures,
                ReconnectCount = reconnectCount,
                LastSuccessUtc = lastSuccessUtc,
                LastFailureUtc = lastFailureUtc,
                LastError = lastError
            };
        }
    }

    public void Disconnect()
    {
        if (disposeRequested)
        {
            return;
        }

        if (!operationGate.Wait(5000))
        {
            LogHelper.Logger.Warn(
                "{0} 未能在 5 秒内获得断开连接锁。",
                deviceName);
            return;
        }

        try
        {
            ResetClientUnsafe();
            SetState(ModbusTcpConnectionState.Disconnected);
        }
        finally
        {
            operationGate.Release();
        }
    }

    public void Dispose()
    {
        if (disposeRequested)
        {
            return;
        }

        disposeRequested = true;
        stopSource.Cancel();

        if (!operationGate.Wait(5000))
        {
            LogHelper.Logger.Warn(
                "{0} 未能在 5 秒内完成停止；当前通信完成后将自动释放。",
                deviceName);
            return;
        }

        try
        {
            ResetClientUnsafe();
            SetState(ModbusTcpConnectionState.Stopped);
        }
        finally
        {
            operationGate.Release();
        }
    }

    private T ExecuteWithRetry<T>(
        string operationName,
        Func<ModbusTcpClient, T> operation,
        CancellationToken cancellationToken)
    {
        bool gateEntered = false;
        try
        {
            operationGate.Wait(cancellationToken);
            gateEntered = true;
            ThrowIfDisposed();

            Exception lastException = null;
            for (int attempt = 1;
                attempt <= MaximumAttempts;
                attempt++)
            {
                cancellationToken.ThrowIfCancellationRequested();

                try
                {
                    ModbusTcpClient activeClient =
                        EnsureConnectedUnsafe();
                    T result = operation(activeClient);
                    RecordSuccess();
                    return result;
                }
                catch (Exception ex)
                {
                    if (disposeRequested ||
                        cancellationToken.IsCancellationRequested)
                    {
                        throw new OperationCanceledException(
                            cancellationToken);
                    }

                    if (!IsCommunicationException(ex))
                    {
                        throw;
                    }

                    lastException = ex;
                    ResetClientUnsafe();
                    RecordFailure(
                        operationName,
                        attempt,
                        ex);

                    if (attempt >= MaximumAttempts)
                    {
                        break;
                    }

                    WaitForRetryDelay(
                        RetryDelaysMilliseconds[attempt - 1],
                        cancellationToken);
                }
            }

            throw new ModbusTcpCommunicationException(
                deviceName,
                operationName,
                lastException);
        }
        finally
        {
            if (gateEntered)
            {
                if (disposeRequested)
                {
                    ResetClientUnsafe();
                    SetState(ModbusTcpConnectionState.Stopped);
                }

                operationGate.Release();
            }
        }
    }

    private ModbusTcpClient EnsureConnectedUnsafe()
    {
        if (client != null && client.IsConnected)
        {
            return client;
        }

        ResetClientUnsafe();
        SetState(ModbusTcpConnectionState.Connecting);

        ModbusTcpClient newClient = new ModbusTcpClient
        {
            ConnectTimeout = ConnectTimeoutMilliseconds,
            ReadTimeout = ReadTimeoutMilliseconds,
            WriteTimeout = WriteTimeoutMilliseconds
        };

        try
        {
            newClient.Connect(endpoint, endianness);
            client = newClient;

            if (hasConnectedBefore)
            {
                lock (healthLock)
                {
                    reconnectCount++;
                }
            }
            else
            {
                hasConnectedBefore = true;
            }

            SetState(ModbusTcpConnectionState.Connected);
            return client;
        }
        catch (Exception ex)
        {
            newClient.Dispose();
            SetState(ModbusTcpConnectionState.Faulted);
            throw new IOException(
                string.Format(
                    "{0} 无法连接到 {1}。",
                    deviceName,
                    endpoint),
                ex);
        }
    }

    private void ResetClientUnsafe()
    {
        ModbusTcpClient clientToDispose = client;
        client = null;

        if (clientToDispose == null)
        {
            return;
        }

        try
        {
            if (clientToDispose.IsConnected)
            {
                clientToDispose.Disconnect();
            }
        }
        catch (Exception ex)
        {
            LogHelper.Logger.Debug(
                ex,
                "{0} 断开连接时出现异常。",
                deviceName);
        }
        finally
        {
            try
            {
                clientToDispose.Dispose();
            }
            catch (Exception ex)
            {
                LogHelper.Logger.Debug(
                    ex,
                    "{0} 释放客户端时出现异常。",
                    deviceName);
            }
        }
    }

    private void RecordSuccess()
    {
        int previousFailureCount;
        lock (healthLock)
        {
            previousFailureCount = consecutiveFailures;
            consecutiveFailures = 0;
            lastSuccessUtc = DateTime.UtcNow;
            lastError = null;
            state = ModbusTcpConnectionState.Connected;
        }

        if (previousFailureCount > 0)
        {
            LogHelper.Logger.Info(
                "{0} 通信已恢复，地址：{1}。",
                deviceName,
                endpoint);
        }
    }

    private void RecordFailure(
        string operationName,
        int attempt,
        Exception ex)
    {
        DateTime nowUtc = DateTime.UtcNow;
        bool shouldLog;
        lock (healthLock)
        {
            consecutiveFailures++;
            lastFailureUtc = nowUtc;
            lastError = ex.Message;
            state = ModbusTcpConnectionState.Faulted;
            shouldLog =
                consecutiveFailures == 1 ||
                (nowUtc - lastFailureLogUtc).TotalSeconds >=
                    FailureLogIntervalSeconds;

            if (shouldLog)
            {
                lastFailureLogUtc = nowUtc;
            }
        }

        if (shouldLog)
        {
            LogHelper.Logger.Warn(
                ex,
                "{0} 执行“{1}”失败，第 {2}/{3} 次尝试；将自动重连。地址：{4}",
                deviceName,
                operationName,
                attempt,
                MaximumAttempts,
                endpoint);
        }
    }

    private void SetState(ModbusTcpConnectionState newState)
    {
        lock (healthLock)
        {
            state = newState;
        }
    }

    private void ThrowIfDisposed()
    {
        if (disposeRequested)
        {
            throw new ObjectDisposedException(deviceName);
        }
    }

    private static void WaitForRetryDelay(
        int delayMilliseconds,
        CancellationToken cancellationToken)
    {
        if (cancellationToken.WaitHandle.WaitOne(
            delayMilliseconds))
        {
            cancellationToken.ThrowIfCancellationRequested();
        }
    }

    private static bool IsCommunicationException(Exception ex)
    {
        return ex is ModbusException ||
            ex is IOException ||
            ex is SocketException ||
            ex is TimeoutException ||
            ex is InvalidOperationException ||
            ex is ObjectDisposedException;
    }
}
