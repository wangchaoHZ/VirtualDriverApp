using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;

internal static class EnergyHistoryStore
{
    private const string FileName = "history_energy.txt";
    private const string BackupFileName = "history_energy.bak";
    private const string TemporaryFileName = "history_energy.tmp";

    private static readonly object fileLock = new object();

    public static string FilePath { get; } = Path.Combine(
        Path.GetDirectoryName(typeof(EnergyHistoryStore).Assembly.Location),
        FileName);

    private static string BackupFilePath => Path.Combine(
        Path.GetDirectoryName(FilePath),
        BackupFileName);

    private static string TemporaryFilePath => Path.Combine(
        Path.GetDirectoryName(FilePath),
        TemporaryFileName);

    public static Dictionary<byte, double> Load()
    {
        lock (fileLock)
        {
            Dictionary<byte, double> energyByAddress;
            if (TryLoadFile(FilePath, out energyByAddress))
            {
                return energyByAddress;
            }

            if (TryLoadFile(BackupFilePath, out energyByAddress))
            {
                LogHelper.Logger.Warn(
                    "累计功耗主历史文件不可用，已从备份文件恢复：{0}",
                    BackupFilePath);
                return energyByAddress;
            }

            return new Dictionary<byte, double>();
        }
    }

    public static void Save(
        IEnumerable<KeyValuePair<byte, double>> energyByAddress)
    {
        string[] lines = new[]
        {
            "# VirtualDriverApp 泵累计功耗，单位：kWh",
            "# 格式：Modbus从站地址=累计功耗；程序运行时请勿修改"
        }
        .Concat(
            energyByAddress
                .OrderBy(item => item.Key)
                .Select(item => string.Format(
                    CultureInfo.InvariantCulture,
                    "{0}={1:F9}",
                    item.Key,
                    Math.Max(0.0, item.Value))))
        .ToArray();

        lock (fileLock)
        {
            string directory = Path.GetDirectoryName(FilePath);
            Directory.CreateDirectory(directory);

            try
            {
                File.WriteAllLines(
                    TemporaryFilePath,
                    lines,
                    new UTF8Encoding(true));

                if (File.Exists(FilePath))
                {
                    File.Replace(
                        TemporaryFilePath,
                        FilePath,
                        BackupFilePath,
                        true);
                }
                else
                {
                    File.Move(TemporaryFilePath, FilePath);
                }
            }
            finally
            {
                if (File.Exists(TemporaryFilePath))
                {
                    File.Delete(TemporaryFilePath);
                }
            }
        }
    }

    private static bool TryLoadFile(
        string path,
        out Dictionary<byte, double> energyByAddress)
    {
        energyByAddress = new Dictionary<byte, double>();

        if (!File.Exists(path))
        {
            return false;
        }

        try
        {
            foreach (string rawLine in File.ReadAllLines(path))
            {
                string line = rawLine.Trim();
                if (line.Length == 0 || line.StartsWith("#"))
                {
                    continue;
                }

                string[] parts = line.Split(new[] { '=' }, 2);
                byte address;
                double energyKwh;
                if (parts.Length != 2 ||
                    !byte.TryParse(
                        parts[0].Trim(),
                        NumberStyles.Integer,
                        CultureInfo.InvariantCulture,
                        out address) ||
                    !double.TryParse(
                        parts[1].Trim(),
                        NumberStyles.Float,
                        CultureInfo.InvariantCulture,
                        out energyKwh) ||
                    double.IsNaN(energyKwh) ||
                    double.IsInfinity(energyKwh) ||
                    energyKwh < 0.0)
                {
                    LogHelper.Logger.Warn(
                        "忽略累计功耗历史文件中的无效行：{0}",
                        rawLine);
                    continue;
                }

                energyByAddress[address] = energyKwh;
            }

            return energyByAddress.Count > 0;
        }
        catch (Exception ex)
        {
            LogHelper.Logger.Warn(
                ex,
                "读取累计功耗历史文件失败：{0}",
                path);
            energyByAddress.Clear();
            return false;
        }
    }
}
