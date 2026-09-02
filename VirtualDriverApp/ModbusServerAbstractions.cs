using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Net;

public enum ModbusTransportMode
{
    Rtu,
    Tcp
}

public interface IModbusServer : IDisposable
{
    ModbusTransportMode TransportMode { get; }

    bool IsRunning { get; }

    string EndpointDescription { get; }

    void Start();

    void Stop();
}

public abstract class ModbusServerConfiguration
{
    protected ModbusServerConfiguration(ModbusTransportMode transportMode)
    {
        TransportMode = transportMode;
    }

    public ModbusTransportMode TransportMode { get; }
}

public sealed class ModbusRtuServerConfiguration : ModbusServerConfiguration
{
    public ModbusRtuServerConfiguration(
        string portName,
        int baudRate,
        Parity parity,
        int dataBits,
        StopBits stopBits)
        : base(ModbusTransportMode.Rtu)
    {
        if (string.IsNullOrWhiteSpace(portName))
        {
            throw new ArgumentException(
                "串口名称不能为空。",
                nameof(portName));
        }

        if (baudRate <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(baudRate));
        }

        if (dataBits < 5 || dataBits > 8)
        {
            throw new ArgumentOutOfRangeException(nameof(dataBits));
        }

        if (stopBits == StopBits.None)
        {
            throw new ArgumentOutOfRangeException(nameof(stopBits));
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
}

public sealed class ModbusTcpServerConfiguration : ModbusServerConfiguration
{
    public ModbusTcpServerConfiguration(IPAddress listenAddress, int port)
        : base(ModbusTransportMode.Tcp)
    {
        if (listenAddress == null)
        {
            throw new ArgumentNullException(nameof(listenAddress));
        }

        if (port <= IPEndPoint.MinPort ||
            port > IPEndPoint.MaxPort)
        {
            throw new ArgumentOutOfRangeException(nameof(port));
        }

        ListenAddress = listenAddress;
        Port = port;
    }

    public IPAddress ListenAddress { get; }

    public int Port { get; }

    public IPEndPoint EndPoint => new IPEndPoint(ListenAddress, Port);
}

internal interface IModbusDataStore
{
    IReadOnlyList<byte> UnitIdentifiers { get; }

    bool ContainsUnit(byte unitIdentifier);

    bool IsValidInputRegisterRange(
        byte unitIdentifier,
        ushort startingAddress,
        ushort quantity);

    ushort[] ReadInputRegisters(
        byte unitIdentifier,
        ushort startingAddress,
        ushort quantity);

    bool IsValidHoldingRegisterRange(
        byte unitIdentifier,
        ushort startingAddress,
        ushort quantity);

    ushort[] ReadHoldingRegisters(
        byte unitIdentifier,
        ushort startingAddress,
        ushort quantity);

    void WriteSingleHoldingRegister(
        byte unitIdentifier,
        ushort registerAddress,
        ushort registerValue);
}

internal sealed class VirtualModbusDataStore : IModbusDataStore
{
    private readonly Dictionary<byte, ModbusRtuSlave> units;
    private readonly byte[] unitIdentifiers;
    private readonly ModbusRtuSlave[] energyUnits;

    public VirtualModbusDataStore(
        IEnumerable<ModbusRtuSlave> modbusUnits)
    {
        if (modbusUnits == null)
        {
            throw new ArgumentNullException(nameof(modbusUnits));
        }

        ModbusRtuSlave[] configuredUnits = modbusUnits.ToArray();
        units = configuredUnits.ToDictionary(
            unit => unit.SlaveAddress,
            unit => unit);

        if (units.Count == 0)
        {
            throw new ArgumentException(
                "至少需要配置一个 Modbus 从站。",
                nameof(modbusUnits));
        }

        unitIdentifiers = units.Keys
            .OrderBy(unitIdentifier => unitIdentifier)
            .ToArray();

        // Form1 passes the pumps in the fixed P1, N1, P2, N2 order.
        // Preserve that order for the aggregate FC04 input-register map.
        energyUnits = configuredUnits
            .Take(PumpEnergyInputRegisterMap.PumpCount)
            .ToArray();
    }

    public IReadOnlyList<byte> UnitIdentifiers => unitIdentifiers;

    public bool ContainsUnit(byte unitIdentifier)
    {
        return units.ContainsKey(unitIdentifier);
    }

    public bool IsValidInputRegisterRange(
        byte unitIdentifier,
        ushort startingAddress,
        ushort quantity)
    {
        if (quantity == 0 || !ContainsUnit(unitIdentifier))
        {
            return false;
        }

        int endAddressExclusive = startingAddress + quantity;
        return startingAddress <
                PumpEnergyInputRegisterMap.RegisterCount &&
            endAddressExclusive <=
                PumpEnergyInputRegisterMap.RegisterCount;
    }

    public ushort[] ReadInputRegisters(
        byte unitIdentifier,
        ushort startingAddress,
        ushort quantity)
    {
        if (!IsValidInputRegisterRange(
            unitIdentifier,
            startingAddress,
            quantity))
        {
            throw new ArgumentOutOfRangeException(
                nameof(startingAddress));
        }

        ushort[] allRegisters = PumpEnergyInputRegisterMap.Encode(
            GetEnergyKwh(0),
            GetEnergyKwh(1),
            GetEnergyKwh(2),
            GetEnergyKwh(3));
        ushort[] values = new ushort[quantity];
        Array.Copy(
            allRegisters,
            startingAddress,
            values,
            0,
            quantity);
        return values;
    }

    public bool IsValidHoldingRegisterRange(
        byte unitIdentifier,
        ushort startingAddress,
        ushort quantity)
    {
        ModbusRtuSlave unit;
        if (quantity == 0 ||
            !units.TryGetValue(unitIdentifier, out unit))
        {
            return false;
        }

        int endAddressExclusive = startingAddress + quantity;
        return startingAddress < unit.HoldingRegisterCount &&
            endAddressExclusive <= unit.HoldingRegisterCount;
    }

    public ushort[] ReadHoldingRegisters(
        byte unitIdentifier,
        ushort startingAddress,
        ushort quantity)
    {
        ModbusRtuSlave unit = GetUnit(unitIdentifier);
        return unit.ReadHoldingRegisters(
            startingAddress,
            quantity);
    }

    public void WriteSingleHoldingRegister(
        byte unitIdentifier,
        ushort registerAddress,
        ushort registerValue)
    {
        ModbusRtuSlave unit = GetUnit(unitIdentifier);
        unit.SetHoldingRegister(
            registerAddress,
            registerValue);
    }

    private ModbusRtuSlave GetUnit(byte unitIdentifier)
    {
        ModbusRtuSlave unit;
        if (!units.TryGetValue(unitIdentifier, out unit))
        {
            throw new ArgumentOutOfRangeException(
                nameof(unitIdentifier),
                "未配置指定的 Modbus Unit ID。");
        }

        return unit;
    }

    private double GetEnergyKwh(int pumpIndex)
    {
        return pumpIndex < energyUnits.Length
            ? energyUnits[pumpIndex].GetAccumulatedEnergyKwh()
            : 0.0;
    }
}

internal sealed class SwitchableModbusServer : IDisposable
{
    private readonly object lifecycleLock = new object();
    private readonly IModbusDataStore dataStore;

    private IModbusServer activeServer;
    private bool disposed;

    public SwitchableModbusServer(IModbusDataStore dataStore)
    {
        this.dataStore = dataStore ??
            throw new ArgumentNullException(nameof(dataStore));
    }

    public bool IsRunning
    {
        get
        {
            lock (lifecycleLock)
            {
                return activeServer != null &&
                    activeServer.IsRunning;
            }
        }
    }

    public ModbusTransportMode? ActiveTransportMode
    {
        get
        {
            lock (lifecycleLock)
            {
                return activeServer?.TransportMode;
            }
        }
    }

    public string EndpointDescription
    {
        get
        {
            lock (lifecycleLock)
            {
                return activeServer?.EndpointDescription ??
                    "未启动";
            }
        }
    }

    public void Start(ModbusServerConfiguration configuration)
    {
        if (configuration == null)
        {
            throw new ArgumentNullException(nameof(configuration));
        }

        IModbusServer previousServer;
        lock (lifecycleLock)
        {
            ThrowIfDisposed();
            previousServer = activeServer;
            activeServer = null;
        }

        previousServer?.Dispose();

        IModbusServer newServer = CreateServer(configuration);
        try
        {
            newServer.Start();
        }
        catch
        {
            newServer.Dispose();
            throw;
        }

        lock (lifecycleLock)
        {
            if (disposed)
            {
                newServer.Dispose();
                throw new ObjectDisposedException(
                    nameof(SwitchableModbusServer));
            }

            activeServer = newServer;
        }
    }

    public void Stop()
    {
        IModbusServer serverToStop;
        lock (lifecycleLock)
        {
            serverToStop = activeServer;
            activeServer = null;
        }

        serverToStop?.Dispose();
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

    private IModbusServer CreateServer(
        ModbusServerConfiguration configuration)
    {
        ModbusRtuServerConfiguration rtuConfiguration =
            configuration as ModbusRtuServerConfiguration;
        if (rtuConfiguration != null)
        {
            return new ModbusRtuServerAdapter(
                rtuConfiguration,
                dataStore);
        }

        ModbusTcpServerConfiguration tcpConfiguration =
            configuration as ModbusTcpServerConfiguration;
        if (tcpConfiguration != null)
        {
            return new FluentModbusTcpServerAdapter(
                tcpConfiguration,
                dataStore);
        }

        throw new NotSupportedException(
            "不支持的 Modbus 服务端配置类型。");
    }

    private void ThrowIfDisposed()
    {
        if (disposed)
        {
            throw new ObjectDisposedException(
                nameof(SwitchableModbusServer));
        }
    }
}
