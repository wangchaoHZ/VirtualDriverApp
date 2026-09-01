using System;
using System.Net;
using System.Net.Sockets;

internal sealed class HydraulicOutputConfiguration
{
    public HydraulicOutputConfiguration(
        IPAddress targetAddress,
        int port,
        byte unitIdentifier)
    {
        if (targetAddress == null)
        {
            throw new ArgumentNullException(nameof(targetAddress));
        }

        if (targetAddress.AddressFamily !=
            AddressFamily.InterNetwork)
        {
            throw new ArgumentException(
                "压力/流量直发目标仅支持 IPv4 地址。",
                nameof(targetAddress));
        }

        if (port < 1 || port > 65535)
        {
            throw new ArgumentOutOfRangeException(nameof(port));
        }

        if (unitIdentifier < 1 || unitIdentifier > 247)
        {
            throw new ArgumentOutOfRangeException(
                nameof(unitIdentifier),
                "Modbus Unit ID 必须在 1 到 247 之间。");
        }

        TargetAddress = targetAddress;
        Port = port;
        UnitIdentifier = unitIdentifier;
    }

    public IPAddress TargetAddress { get; }

    public int Port { get; }

    public byte UnitIdentifier { get; }

    public string Endpoint => string.Format(
        "{0}:{1}",
        TargetAddress,
        Port);
}

internal static class HydraulicOutputRegisterMap
{
    public const ushort StartingAddress = 0;
    public const int RegisterCount = 8;

    // 每个量使用一个 16 位保持寄存器。
    // 压力分辨率：0.0001 MPa；流量分辨率：0.01 m³/h。
    public const double PressureScale = 10000.0;
    public const double FlowScale = 100.0;

    public const ushort P1PressureAddress = 0;
    public const ushort P1FlowAddress = 1;
    public const ushort N1PressureAddress = 2;
    public const ushort N1FlowAddress = 3;
    public const ushort P2PressureAddress = 4;
    public const ushort P2FlowAddress = 5;
    public const ushort N2PressureAddress = 6;
    public const ushort N2FlowAddress = 7;

    public static ushort[] Encode(
        double p1PressureMpa,
        double p1FlowM3PerHour,
        double n1PressureMpa,
        double n1FlowM3PerHour,
        double p2PressureMpa,
        double p2FlowM3PerHour,
        double n2PressureMpa,
        double n2FlowM3PerHour)
    {
        ushort[] registers = new ushort[RegisterCount];
        registers[P1PressureAddress] = ToRegisterValue(
            p1PressureMpa,
            PressureScale);
        registers[P1FlowAddress] = ToRegisterValue(
            p1FlowM3PerHour,
            FlowScale);
        registers[N1PressureAddress] = ToRegisterValue(
            n1PressureMpa,
            PressureScale);
        registers[N1FlowAddress] = ToRegisterValue(
            n1FlowM3PerHour,
            FlowScale);
        registers[P2PressureAddress] = ToRegisterValue(
            p2PressureMpa,
            PressureScale);
        registers[P2FlowAddress] = ToRegisterValue(
            p2FlowM3PerHour,
            FlowScale);
        registers[N2PressureAddress] = ToRegisterValue(
            n2PressureMpa,
            PressureScale);
        registers[N2FlowAddress] = ToRegisterValue(
            n2FlowM3PerHour,
            FlowScale);
        return registers;
    }

    private static ushort ToRegisterValue(
        double engineeringValue,
        double scale)
    {
        if (double.IsNaN(engineeringValue) ||
            double.IsInfinity(engineeringValue))
        {
            throw new ArgumentOutOfRangeException(
                nameof(engineeringValue));
        }

        double scaledValue = Math.Round(
            engineeringValue * scale,
            MidpointRounding.AwayFromZero);
        return (ushort)Math.Max(
            ushort.MinValue,
            Math.Min(ushort.MaxValue, scaledValue));
    }
}
