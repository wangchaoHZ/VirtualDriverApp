using FluentModbus;
using System;
using System.Net;

internal sealed class ModbusRtuServerAdapter : IModbusServer
{
    private readonly ModbusRtuServerConfiguration configuration;
    private readonly ModbusRtuSerialServer serialServer;
    private bool disposed;

    public ModbusRtuServerAdapter(
        ModbusRtuServerConfiguration configuration,
        IModbusDataStore dataStore)
    {
        this.configuration = configuration ??
            throw new ArgumentNullException(nameof(configuration));
        serialServer = new ModbusRtuSerialServer(
            new ModbusRtuRequestProcessor(dataStore)
                .ProcessRequestFrame);
    }

    public ModbusTransportMode TransportMode =>
        ModbusTransportMode.Rtu;

    public bool IsRunning { get; private set; }

    public string EndpointDescription => string.Format(
        "RTU {0} / {1}-{2}-{3}-{4}",
        configuration.PortName,
        configuration.BaudRate,
        configuration.DataBits,
        configuration.Parity,
        configuration.StopBits);

    public void Start()
    {
        ThrowIfDisposed();
        if (IsRunning)
        {
            return;
        }

        serialServer.ConfigureAndOpen(
            configuration.PortName,
            configuration.BaudRate,
            configuration.Parity,
            configuration.DataBits,
            configuration.StopBits);
        serialServer.Start();
        IsRunning = true;

        LogHelper.Logger.Info(
            "Modbus RTU 模拟服务端已启动：{0}。",
            EndpointDescription);
    }

    public void Stop()
    {
        if (!IsRunning)
        {
            return;
        }

        serialServer.Stop();
        IsRunning = false;
        LogHelper.Logger.Info(
            "Modbus RTU 模拟服务端已停止。");
    }

    public void Dispose()
    {
        if (disposed)
        {
            return;
        }

        disposed = true;
        try
        {
            Stop();
        }
        finally
        {
            serialServer.Dispose();
        }
    }

    private void ThrowIfDisposed()
    {
        if (disposed)
        {
            throw new ObjectDisposedException(
                nameof(ModbusRtuServerAdapter));
        }
    }
}

internal sealed class FluentModbusTcpServerAdapter : IModbusServer
{
    private const ushort MaximumReadHoldingRegisterQuantity = 125;

    private readonly object lifecycleLock = new object();
    private readonly ModbusTcpServerConfiguration configuration;
    private readonly IModbusDataStore dataStore;

    private ModbusTcpServer server;
    private bool disposed;

    public FluentModbusTcpServerAdapter(
        ModbusTcpServerConfiguration configuration,
        IModbusDataStore dataStore)
    {
        this.configuration = configuration ??
            throw new ArgumentNullException(nameof(configuration));
        this.dataStore = dataStore ??
            throw new ArgumentNullException(nameof(dataStore));
    }

    public ModbusTransportMode TransportMode =>
        ModbusTransportMode.Tcp;

    public bool IsRunning { get; private set; }

    public string EndpointDescription => string.Format(
        "TCP {0}:{1}",
        configuration.ListenAddress,
        configuration.Port);

    public void Start()
    {
        lock (lifecycleLock)
        {
            ThrowIfDisposed();
            if (IsRunning)
            {
                return;
            }

            ModbusTcpServer newServer = new ModbusTcpServer();
            newServer.RemoveUnit(0);
            foreach (byte unitIdentifier in
                dataStore.UnitIdentifiers)
            {
                newServer.AddUnit(unitIdentifier);
            }

            newServer.EnableRaisingEvents = true;
            newServer.RequestValidator = ValidateRequest;
            newServer.RegistersChanged += OnRegistersChanged;
            server = newServer;

            try
            {
                newServer.Start(configuration.EndPoint);
                IsRunning = true;
            }
            catch
            {
                server = null;
                newServer.RegistersChanged -= OnRegistersChanged;
                newServer.Dispose();
                throw;
            }
        }

        LogHelper.Logger.Info(
            "Modbus TCP 模拟服务端已通过 FluentModbus 启动：{0}，" +
            "Unit ID={1}。",
            EndpointDescription,
            string.Join(",", dataStore.UnitIdentifiers));
    }

    public void Stop()
    {
        ModbusTcpServer serverToStop;
        lock (lifecycleLock)
        {
            serverToStop = server;
            server = null;
            IsRunning = false;
        }

        if (serverToStop == null)
        {
            return;
        }

        serverToStop.RegistersChanged -= OnRegistersChanged;
        try
        {
            serverToStop.Stop();
        }
        finally
        {
            serverToStop.Dispose();
        }

        LogHelper.Logger.Info(
            "Modbus TCP 模拟服务端已停止。");
    }

    public void Dispose()
    {
        lock (lifecycleLock)
        {
            if (disposed)
            {
                return;
            }

            disposed = true;
        }

        Stop();
    }

    private ModbusExceptionCode ValidateRequest(
        byte unitIdentifier,
        ModbusFunctionCode functionCode,
        ushort address,
        ushort quantityOfRegisters)
    {
        if (!dataStore.ContainsUnit(unitIdentifier))
        {
            return ModbusExceptionCode.IllegalDataAddress;
        }

        if (functionCode ==
            ModbusFunctionCode.ReadHoldingRegisters)
        {
            if (quantityOfRegisters == 0 ||
                quantityOfRegisters >
                    MaximumReadHoldingRegisterQuantity)
            {
                return ModbusExceptionCode.IllegalDataValue;
            }

            if (!dataStore.IsValidHoldingRegisterRange(
                unitIdentifier,
                address,
                quantityOfRegisters))
            {
                return ModbusExceptionCode.IllegalDataAddress;
            }

            SynchronizeDataStoreToServer(
                unitIdentifier,
                address,
                quantityOfRegisters);
            return ModbusExceptionCode.OK;
        }

        if (functionCode ==
            ModbusFunctionCode.WriteSingleRegister)
        {
            return dataStore.IsValidHoldingRegisterRange(
                unitIdentifier,
                address,
                1)
                    ? ModbusExceptionCode.OK
                    : ModbusExceptionCode.IllegalDataAddress;
        }

        // 与原 RTU 模拟器保持一致：仅开放 FC03 和 FC06。
        return ModbusExceptionCode.IllegalFunction;
    }

    private void OnRegistersChanged(
        object sender,
        RegistersChangedEventArgs eventArgs)
    {
        ModbusTcpServer activeServer =
            sender as ModbusTcpServer;
        if (activeServer == null ||
            !dataStore.ContainsUnit(eventArgs.UnitIdentifier))
        {
            return;
        }

        lock (activeServer.Lock)
        {
            Span<byte> registerBuffer =
                activeServer.GetHoldingRegisterBuffer(
                    eventArgs.UnitIdentifier);

            foreach (int changedAddress in eventArgs.Registers)
            {
                if (changedAddress < 0 ||
                    changedAddress > ushort.MaxValue ||
                    !dataStore.IsValidHoldingRegisterRange(
                        eventArgs.UnitIdentifier,
                        (ushort)changedAddress,
                        1))
                {
                    continue;
                }

                int byteOffset = changedAddress * 2;
                ushort registerValue = (ushort)(
                    (registerBuffer[byteOffset] << 8) |
                    registerBuffer[byteOffset + 1]);

                dataStore.WriteSingleHoldingRegister(
                    eventArgs.UnitIdentifier,
                    (ushort)changedAddress,
                    registerValue);

                // 业务层可能对写入值进行限幅，立即把最终值回写到
                // FluentModbus 缓冲区，保证下一次读取一致。
                SynchronizeDataStoreToServerUnsafe(
                    activeServer,
                    eventArgs.UnitIdentifier,
                    (ushort)changedAddress,
                    1);
            }
        }
    }

    private void SynchronizeDataStoreToServer(
        byte unitIdentifier,
        ushort startingAddress,
        ushort quantity)
    {
        ModbusTcpServer activeServer = server;
        if (activeServer == null)
        {
            return;
        }

        lock (activeServer.Lock)
        {
            SynchronizeDataStoreToServerUnsafe(
                activeServer,
                unitIdentifier,
                startingAddress,
                quantity);
        }
    }

    private void SynchronizeDataStoreToServerUnsafe(
        ModbusTcpServer activeServer,
        byte unitIdentifier,
        ushort startingAddress,
        ushort quantity)
    {
        ushort[] values = dataStore.ReadHoldingRegisters(
            unitIdentifier,
            startingAddress,
            quantity);
        Span<byte> registerBuffer =
            activeServer.GetHoldingRegisterBuffer(unitIdentifier);

        for (int index = 0; index < values.Length; index++)
        {
            int byteOffset = (startingAddress + index) * 2;
            ushort value = values[index];
            registerBuffer[byteOffset] = (byte)(value >> 8);
            registerBuffer[byteOffset + 1] =
                (byte)(value & 0xFF);
        }
    }

    private void ThrowIfDisposed()
    {
        if (disposed)
        {
            throw new ObjectDisposedException(
                nameof(FluentModbusTcpServerAdapter));
        }
    }
}

internal sealed class ModbusRtuRequestProcessor
{
    private readonly IModbusDataStore dataStore;

    public ModbusRtuRequestProcessor(IModbusDataStore dataStore)
    {
        this.dataStore = dataStore ??
            throw new ArgumentNullException(nameof(dataStore));
    }

    public byte[] ProcessRequestFrame(byte[] request)
    {
        if (request == null ||
            request.Length != ModbusRtuProtocol.RequestFrameLength ||
            !ModbusRtuProtocol.HasValidCrc(request))
        {
            return null;
        }

        byte unitIdentifier = request[0];
        byte functionCode = request[1];

        // Modbus RTU 对未配置的从站地址不应返回任何数据。
        if (!dataStore.ContainsUnit(unitIdentifier))
        {
            return null;
        }

        try
        {
            if (functionCode == 0x03)
            {
                ushort startingAddress = (ushort)(
                    (request[2] << 8) | request[3]);
                ushort quantity = (ushort)(
                    (request[4] << 8) | request[5]);
                return HandleReadHoldingRegisters(
                    unitIdentifier,
                    startingAddress,
                    quantity);
            }

            if (functionCode == 0x06)
            {
                ushort registerAddress = (ushort)(
                    (request[2] << 8) | request[3]);
                ushort registerValue = (ushort)(
                    (request[4] << 8) | request[5]);
                return HandleWriteSingleRegister(
                    unitIdentifier,
                    registerAddress,
                    registerValue);
            }

            return CreateExceptionResponse(
                unitIdentifier,
                functionCode,
                0x01);
        }
        catch (Exception ex)
        {
            LogHelper.Logger.Error(
                ex,
                "处理 Modbus RTU 请求失败，从站：{0}，功能码：0x{1:X2}",
                unitIdentifier,
                functionCode);
            return CreateExceptionResponse(
                unitIdentifier,
                functionCode,
                0x04);
        }
    }

    private byte[] HandleReadHoldingRegisters(
        byte unitIdentifier,
        ushort startingAddress,
        ushort quantity)
    {
        if (quantity == 0 || quantity > 125)
        {
            return CreateExceptionResponse(
                unitIdentifier,
                0x03,
                0x03);
        }

        if (!dataStore.IsValidHoldingRegisterRange(
            unitIdentifier,
            startingAddress,
            quantity))
        {
            return CreateExceptionResponse(
                unitIdentifier,
                0x03,
                0x02);
        }

        ushort[] values = dataStore.ReadHoldingRegisters(
            unitIdentifier,
            startingAddress,
            quantity);
        byte[] payload = new byte[3 + 2 * quantity];
        payload[0] = unitIdentifier;
        payload[1] = 0x03;
        payload[2] = (byte)(2 * quantity);

        for (int index = 0; index < values.Length; index++)
        {
            payload[3 + 2 * index] =
                (byte)(values[index] >> 8);
            payload[4 + 2 * index] =
                (byte)(values[index] & 0xFF);
        }

        return ModbusRtuProtocol.AppendCrc(payload);
    }

    private byte[] HandleWriteSingleRegister(
        byte unitIdentifier,
        ushort registerAddress,
        ushort registerValue)
    {
        if (!dataStore.IsValidHoldingRegisterRange(
            unitIdentifier,
            registerAddress,
            1))
        {
            return CreateExceptionResponse(
                unitIdentifier,
                0x06,
                0x02);
        }

        dataStore.WriteSingleHoldingRegister(
            unitIdentifier,
            registerAddress,
            registerValue);

        byte[] payload = new byte[6];
        payload[0] = unitIdentifier;
        payload[1] = 0x06;
        payload[2] = (byte)(registerAddress >> 8);
        payload[3] = (byte)(registerAddress & 0xFF);
        payload[4] = (byte)(registerValue >> 8);
        payload[5] = (byte)(registerValue & 0xFF);
        return ModbusRtuProtocol.AppendCrc(payload);
    }

    private static byte[] CreateExceptionResponse(
        byte unitIdentifier,
        byte functionCode,
        byte exceptionCode)
    {
        byte[] payload =
        {
            unitIdentifier,
            (byte)(functionCode | 0x80),
            exceptionCode
        };
        return ModbusRtuProtocol.AppendCrc(payload);
    }
}
