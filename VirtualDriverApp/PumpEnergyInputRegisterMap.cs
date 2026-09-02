using System;

internal static class PumpEnergyInputRegisterMap
{
    public const ushort StartingAddress = 0;
    public const int RegisterCount = 8;
    public const int PumpCount = 4;

    // The source value is kWh with three decimal places in the existing UI.
    // Encode it as Wh so UInt32 keeps the existing 0.001 kWh resolution.
    public const double EnergyScale = 1000.0;

    public const ushort P1HighWordAddress = 0;
    public const ushort P1LowWordAddress = 1;
    public const ushort N1HighWordAddress = 2;
    public const ushort N1LowWordAddress = 3;
    public const ushort P2HighWordAddress = 4;
    public const ushort P2LowWordAddress = 5;
    public const ushort N2HighWordAddress = 6;
    public const ushort N2LowWordAddress = 7;

    public static ushort[] Encode(
        double p1EnergyKwh,
        double n1EnergyKwh,
        double p2EnergyKwh,
        double n2EnergyKwh)
    {
        ushort[] registers = new ushort[RegisterCount];
        WriteUInt32(
            registers,
            P1HighWordAddress,
            ToRegisterValue(p1EnergyKwh));
        WriteUInt32(
            registers,
            N1HighWordAddress,
            ToRegisterValue(n1EnergyKwh));
        WriteUInt32(
            registers,
            P2HighWordAddress,
            ToRegisterValue(p2EnergyKwh));
        WriteUInt32(
            registers,
            N2HighWordAddress,
            ToRegisterValue(n2EnergyKwh));
        return registers;
    }

    private static uint ToRegisterValue(double energyKwh)
    {
        if (double.IsNaN(energyKwh) ||
            double.IsInfinity(energyKwh))
        {
            throw new ArgumentOutOfRangeException(nameof(energyKwh));
        }

        double scaledValue = Math.Round(
            energyKwh * EnergyScale,
            MidpointRounding.AwayFromZero);
        if (scaledValue <= uint.MinValue)
        {
            return uint.MinValue;
        }

        if (scaledValue >= uint.MaxValue)
        {
            return uint.MaxValue;
        }

        return (uint)scaledValue;
    }

    private static void WriteUInt32(
        ushort[] registers,
        ushort highWordAddress,
        uint value)
    {
        // Modbus byte order and UInt32 word order are both big-endian:
        // high 16-bit register first, then low 16-bit register.
        registers[highWordAddress] = (ushort)(value >> 16);
        registers[highWordAddress + 1] =
            (ushort)(value & ushort.MaxValue);
    }
}
