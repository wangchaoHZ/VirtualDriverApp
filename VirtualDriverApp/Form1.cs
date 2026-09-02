using FluentModbus;
using NLog;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Net;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace VirtualDriverApp
{
    public partial class Form1 : Form
    {
        private const float MinimumScaledFontSize = 6.0f;

        private readonly Dictionary<Control, ResponsiveControlState> _responsiveControls =
            new Dictionary<Control, ResponsiveControlState>();
        private readonly Dictionary<Control, Font> _responsiveFonts =
            new Dictionary<Control, Font>();
        private readonly Size _responsiveBaseClientSize;

        private bool _isApplyingResponsiveLayout;

        public Form1()
        {
            slave1 = new ModbusRtuSlave(11);
            slave2 = new ModbusRtuSlave(22);
            slave3 = new ModbusRtuSlave(33);
            slave4 = new ModbusRtuSlave(44);
            _modbusDataStore = new VirtualModbusDataStore(
                new[] { slave1, slave2, slave3, slave4 });
            _modbusServer = new SwitchableModbusServer(
                _modbusDataStore);

            InitializeComponent();

            // 设置窗体启动时自动居中
            this.StartPosition = FormStartPosition.CenterScreen;

            _responsiveBaseClientSize = ClientSize;
            CaptureResponsiveLayout(this);

            DoubleBuffered = true;
            Resize += Form1_Resize;
            Disposed += Form1_Disposed;
        }

        private sealed class ResponsiveControlState
        {
            public Rectangle Bounds { get; set; }

            public string FontFamilyName { get; set; }

            public float FontSize { get; set; }

            public FontStyle FontStyle { get; set; }

            public GraphicsUnit FontUnit { get; set; }

            public byte GdiCharSet { get; set; }

            public bool GdiVerticalFont { get; set; }
        }

        private void CaptureResponsiveLayout(Control parent)
        {
            foreach (Control control in parent.Controls)
            {
                Font font = control.Font;
                _responsiveControls[control] = new ResponsiveControlState
                {
                    Bounds = control.Bounds,
                    FontFamilyName = font.FontFamily.Name,
                    FontSize = font.Size,
                    FontStyle = font.Style,
                    FontUnit = font.Unit,
                    GdiCharSet = font.GdiCharSet,
                    GdiVerticalFont = font.GdiVerticalFont
                };

                if (control.HasChildren)
                {
                    CaptureResponsiveLayout(control);
                }
            }
        }

        private void Form1_Resize(object sender, EventArgs e)
        {
            ApplyResponsiveLayout();
        }

        private void ApplyResponsiveLayout()
        {
            if (_isApplyingResponsiveLayout ||
                _responsiveBaseClientSize.Width <= 0 ||
                _responsiveBaseClientSize.Height <= 0 ||
                ClientSize.Width <= 0 ||
                ClientSize.Height <= 0)
            {
                return;
            }

            float widthScale = (float)ClientSize.Width / _responsiveBaseClientSize.Width;
            float heightScale = (float)ClientSize.Height / _responsiveBaseClientSize.Height;
            float scale = Math.Min(widthScale, heightScale);

            int scaledWidth = (int)Math.Round(_responsiveBaseClientSize.Width * scale);
            int scaledHeight = (int)Math.Round(_responsiveBaseClientSize.Height * scale);
            int offsetX = (ClientSize.Width - scaledWidth) / 2;
            int offsetY = (ClientSize.Height - scaledHeight) / 2;

            _isApplyingResponsiveLayout = true;
            SuspendLayout();

            try
            {
                ScaleResponsiveControls(this, scale, offsetX, offsetY);
            }
            finally
            {
                ResumeLayout(true);
                _isApplyingResponsiveLayout = false;
            }
        }

        private void ScaleResponsiveControls(
            Control parent,
            float scale,
            int parentOffsetX,
            int parentOffsetY)
        {
            foreach (Control control in parent.Controls)
            {
                ResponsiveControlState state;
                if (!_responsiveControls.TryGetValue(control, out state))
                {
                    continue;
                }

                SetScaledFont(control, state, scale);

                control.Bounds = new Rectangle(
                    parentOffsetX + (int)Math.Round(state.Bounds.X * scale),
                    parentOffsetY + (int)Math.Round(state.Bounds.Y * scale),
                    Math.Max(1, (int)Math.Round(state.Bounds.Width * scale)),
                    Math.Max(1, (int)Math.Round(state.Bounds.Height * scale)));

                KeepControlInsideParent(control, parent);

                if (control.HasChildren)
                {
                    ScaleResponsiveControls(control, scale, 0, 0);
                }
            }
        }

        private static void KeepControlInsideParent(Control control, Control parent)
        {
            int maximumLeft = Math.Max(0, parent.ClientSize.Width - control.Width);
            int maximumTop = Math.Max(0, parent.ClientSize.Height - control.Height);

            control.Left = Math.Min(Math.Max(0, control.Left), maximumLeft);
            control.Top = Math.Min(Math.Max(0, control.Top), maximumTop);
        }

        private void SetScaledFont(
            Control control,
            ResponsiveControlState state,
            float scale)
        {
            float fontSize = Math.Max(MinimumScaledFontSize, state.FontSize * scale);
            if (Math.Abs(control.Font.Size - fontSize) < 0.05f)
            {
                return;
            }

            Font scaledFont = new Font(
                state.FontFamilyName,
                fontSize,
                state.FontStyle,
                state.FontUnit,
                state.GdiCharSet,
                state.GdiVerticalFont);

            Font previousScaledFont;
            _responsiveFonts.TryGetValue(control, out previousScaledFont);

            control.Font = scaledFont;
            _responsiveFonts[control] = scaledFont;

            if (previousScaledFont != null)
            {
                previousScaledFont.Dispose();
            }
        }

        private void Form1_Disposed(object sender, EventArgs e)
        {
            _isShuttingDown = true;
            timer1.Enabled = false;
            timer2.Enabled = false;
            timer3.Enabled = false;
            _lifetimeSource.Cancel();

            HydraulicOutputClientSession outputSession =
                DetachHydraulicOutputSession();
            if (outputSession != null)
            {
                outputSession.Dispose();
            }

            _doClient.Dispose();
            _diClient.Dispose();
            try
            {
                _modbusServer.Dispose();
            }
            catch (Exception ex)
            {
                LogHelper.Logger.Warn(
                    ex,
                    "关闭 Modbus 模拟服务端时出现异常。");
            }

            foreach (Font font in _responsiveFonts.Values)
            {
                font.Dispose();
            }

            _responsiveFonts.Clear();
        }

        private const byte DO_ModbusClient_ID = 2;
        private const byte DI_ModbusClient_ID = 3;
        private const int DiDataMaximumAgeSeconds = 5;
        private const string ModbusRtuModeDisplayName = "Modbus RTU";
        private const string ModbusTcpModeDisplayName = "Modbus TCP";

        private readonly ResilientModbusTcpClient _doClient =
            new ResilientModbusTcpClient(
                "DO",
                "192.168.1.131",
                ModbusEndianness.BigEndian);
        private readonly ResilientModbusTcpClient _diClient =
            new ResilientModbusTcpClient(
                "DI",
                "192.168.1.132",
                ModbusEndianness.BigEndian);
        private readonly CancellationTokenSource _lifetimeSource =
            new CancellationTokenSource();
        private readonly object _diStateLock = new object();
        private readonly object _hydraulicOutputSessionLock =
            new object();

        private HydraulicOutputClientSession
            _hydraulicOutputSession;

        private sealed class HydraulicOutputClientSession : IDisposable
        {
            private int _disposeStarted;

            public HydraulicOutputClientSession(
                HydraulicOutputConfiguration configuration,
                CancellationToken applicationCancellationToken)
            {
                Configuration = configuration ??
                    throw new ArgumentNullException(
                        nameof(configuration));
                CancellationSource =
                    CancellationTokenSource.CreateLinkedTokenSource(
                        applicationCancellationToken);
                Client = new ResilientModbusTcpClient(
                    "压力/流量直发",
                    configuration.Endpoint,
                    ModbusEndianness.BigEndian);
            }

            public HydraulicOutputConfiguration Configuration { get; }

            public ResilientModbusTcpClient Client { get; }

            public CancellationTokenSource CancellationSource { get; }

            public void Dispose()
            {
                if (Interlocked.Exchange(
                    ref _disposeStarted,
                    1) != 0)
                {
                    return;
                }

                CancellationSource.Cancel();
                Client.Dispose();
                CancellationSource.Dispose();
            }
        }

        // 四个模拟从站共用同一数据层，由 RTU/TCP 服务端适配器访问。
        private readonly ModbusRtuSlave slave1;
        private readonly ModbusRtuSlave slave2;
        private readonly ModbusRtuSlave slave3;
        private readonly ModbusRtuSlave slave4;
        private readonly VirtualModbusDataStore _modbusDataStore;
        private readonly SwitchableModbusServer _modbusServer;
        private ModbusServerConfiguration
            _activeModbusServerConfiguration;

        private double PN1_PRESS_DIFF;
        private double PN2_PRESS_DIFF;
        private double PN1_FLOW_DIFF;
        private double PN2_FLOW_DIFF;

        private readonly ushort[] DI_InputRegisters = new ushort[32];
        private bool _diDataValid;
        private DateTime _diLastSuccessUtc = DateTime.MinValue;
        private DateTime _lastDiInvalidLogUtc = DateTime.MinValue;
        private bool _isShuttingDown;
        private bool _systemStarted;

        // 新增：防止 Timer 重入（定时器回调在 UI 线程，网络慢时容易叠加）
        private volatile bool _timer2Busy = false;
        private volatile bool _timer3Busy = false;

        private void Form1_Load(object sender, EventArgs e)
        {
            label10.ForeColor = Color.Black;
            label10.Text = "未启动";
            checkBox17.Checked = false;
            LogHelper.Logger.Info("Application started at " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"));

            PN1_PRESS_DIFF = 0.0;
            PN2_PRESS_DIFF = 0.0;
            PN1_FLOW_DIFF = 0.0;
            PN2_FLOW_DIFF = 0.0;

            textBox11.Text = PN1_PRESS_DIFF.ToString("F3");
            textBox12.Text = PN1_FLOW_DIFF.ToString("F2");

            textBox17.Text = PN2_PRESS_DIFF.ToString("F3");
            textBox20.Text = PN2_FLOW_DIFF.ToString("F2");

            // 获取所有可用的串口名称
            string[] portNames = SerialPort.GetPortNames();

            // 将串口名称添加到 ComboBox 中
            comboBox1.Items.Clear();  // 清除原有的项
            foreach (string port in portNames)
            {
                comboBox1.Items.Add(port);  // 添加串口名称到 ComboBox
            }

            string savedSerialPort =
                Properties.Settings.Default.ModbusSerialPort;
            if (!string.IsNullOrWhiteSpace(savedSerialPort) &&
                comboBox1.Items.Contains(savedSerialPort))
            {
                comboBox1.SelectedItem = savedSerialPort;
            }
            else if (comboBox1.Items.Count > 0)
            {
                comboBox1.SelectedIndex = 0;
            }

            comboBoxModbusMode.Items.Clear();
            comboBoxModbusMode.Items.Add(ModbusRtuModeDisplayName);
            comboBoxModbusMode.Items.Add(ModbusTcpModeDisplayName);
            comboBoxModbusMode.SelectedIndex = string.Equals(
                Properties.Settings.Default.ModbusTransportMode,
                ModbusTransportMode.Tcp.ToString(),
                StringComparison.OrdinalIgnoreCase)
                    ? 1
                    : 0;
            textBoxTcpListenAddress.Text = string.IsNullOrWhiteSpace(
                Properties.Settings.Default.ModbusTcpListenAddress)
                    ? IPAddress.Any.ToString()
                    : Properties.Settings.Default
                        .ModbusTcpListenAddress;

            int savedTcpPort =
                Properties.Settings.Default.ModbusTcpPort;
            numericUpDownTcpPort.Value = Math.Max(
                numericUpDownTcpPort.Minimum,
                Math.Min(
                    numericUpDownTcpPort.Maximum,
                    savedTcpPort));

            textBoxHydraulicTargetIp.Text =
                string.IsNullOrWhiteSpace(
                    Properties.Settings.Default
                        .HydraulicOutputTargetIp)
                    ? IPAddress.Loopback.ToString()
                    : Properties.Settings.Default
                        .HydraulicOutputTargetIp;
            numericUpDownHydraulicPort.Value = Math.Max(
                numericUpDownHydraulicPort.Minimum,
                Math.Min(
                    numericUpDownHydraulicPort.Maximum,
                    Properties.Settings.Default
                        .HydraulicOutputPort));
            numericUpDownHydraulicUnitId.Value = Math.Max(
                numericUpDownHydraulicUnitId.Minimum,
                Math.Min(
                    numericUpDownHydraulicUnitId.Maximum,
                    Properties.Settings.Default
                        .HydraulicOutputUnitId));
            SetHydraulicOutputConfigurationEnabled(true);
            UpdateModbusConfigurationControls();

            // 为每个从站设置保持寄存器的初始值
            slave1.SetHoldingRegister(0, 0);  // 设置从站11的寄存器0初始值
            slave2.SetHoldingRegister(0, 0);  // 设置从站22的寄存器0初始值
            slave3.SetHoldingRegister(0, 0);  // 设置从站33的寄存器0初始值
            slave4.SetHoldingRegister(0, 0);  // 设置从站44的寄存器0初始值
        }

        // 修改为异步，避免在 UI 线程同步阻塞 Connect 和 Sleep
        private async void button1_Click(object sender, EventArgs e)
        {
            if (_isShuttingDown)
            {
                return;
            }

            if (_systemStarted)
            {
                await SwitchModbusServerAsync();
                return;
            }

            button1.Enabled = false;
            label10.ForeColor = Color.Black;
            label10.Text = "正在连接...";

            try
            {
                ModbusServerConfiguration serverConfiguration =
                    BuildModbusServerConfiguration();
                CancellationToken cancellationToken =
                    _lifetimeSource.Token;

                // DO、DI 是基础设备；压力/流量 FC16 直发由 checkBox17 动态启用。
                await Task.WhenAll(
                    _doClient.ConnectAsync(cancellationToken),
                    _diClient.ConnectAsync(cancellationToken));

                // 保留原有的四次 DO 初始化写入行为。
                for (int writeIndex = 0;
                    writeIndex < 4;
                    writeIndex++)
                {
                    await _doClient.ExecuteAsync(
                        "初始化 DO 寄存器 4",
                        activeClient =>
                            activeClient.WriteSingleRegister(
                                DO_ModbusClient_ID,
                                4,
                                (short)1),
                        cancellationToken);

                    if (writeIndex < 3)
                    {
                        await Task.Delay(
                            250,
                            cancellationToken);
                    }
                }

                await Task.Run(
                    () => _modbusServer.Start(
                        serverConfiguration),
                    cancellationToken);
                _activeModbusServerConfiguration =
                    serverConfiguration;
                _systemStarted = true;
                SaveModbusServerConfiguration(
                    serverConfiguration);

                if (checkBox17.Checked)
                {
                    await EnableHydraulicOutputAsync();
                }

                button1.ForeColor = Color.Green;
                button1.Text = "应用通信配置";
                label10.ForeColor = Color.Green;
                label10.Text = GetRunningStatusText("运行中");

                // 直接启用定时器（不要再用 Thread.Sleep 阻塞 UI）
                timer1.Enabled = true;
                timer2.Enabled = true;
                timer3.Enabled = true;
            }
            catch (OperationCanceledException)
                when (_isShuttingDown)
            {
                // 程序退出时的正常取消，不显示错误弹窗。
            }
            catch (Exception ex)
            {
                try
                {
                    _modbusServer.Stop();
                }
                catch (Exception stopException)
                {
                    LogHelper.Logger.Warn(
                        stopException,
                        "启动失败后关闭 Modbus 模拟服务端时出现异常。");
                }

                DisconnectAllTcpClients();
                _activeModbusServerConfiguration = null;
                SetModbusConfigurationEnabled(true);
                label10.ForeColor = Color.Red;
                label10.Text = "连接失败";
                LogHelper.Logger.Error(ex, "启动连接失败");
                MessageBox.Show($"连接设备失败：{ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                button1.Enabled = !_isShuttingDown;
            }
        }

        private async Task SwitchModbusServerAsync()
        {
            button1.Enabled = false;
            SetModbusConfigurationEnabled(false);

            ModbusServerConfiguration previousConfiguration =
                _activeModbusServerConfiguration;

            try
            {
                ModbusServerConfiguration newConfiguration =
                    BuildModbusServerConfiguration();
                if (_modbusServer.IsRunning &&
                    AreEquivalentModbusConfigurations(
                        previousConfiguration,
                        newConfiguration))
                {
                    label10.ForeColor = Color.Green;
                    label10.Text = GetRunningStatusText("运行中");
                    return;
                }

                label10.ForeColor = Color.DarkOrange;
                label10.Text = "正在切换 Modbus 服务端...";

                await Task.Run(
                    () => _modbusServer.Start(newConfiguration),
                    _lifetimeSource.Token);

                _activeModbusServerConfiguration =
                    newConfiguration;
                SaveModbusServerConfiguration(newConfiguration);
                label10.ForeColor = Color.Green;
                label10.Text = GetRunningStatusText("运行中");
                LogHelper.Logger.Info(
                    "Modbus 模拟服务端已切换：{0}。",
                    _modbusServer.EndpointDescription);
            }
            catch (OperationCanceledException)
                when (_isShuttingDown)
            {
                // 程序退出时的正常取消。
            }
            catch (Exception switchException)
            {
                bool rollbackSucceeded = false;
                Exception rollbackException = null;

                if (previousConfiguration != null &&
                    !_isShuttingDown)
                {
                    try
                    {
                        await Task.Run(
                            () => _modbusServer.Start(
                                previousConfiguration),
                            _lifetimeSource.Token);
                        _activeModbusServerConfiguration =
                            previousConfiguration;
                        ApplyModbusConfigurationToControls(
                            previousConfiguration);
                        rollbackSucceeded = true;
                    }
                    catch (Exception ex)
                    {
                        rollbackException = ex;
                    }
                }

                LogHelper.Logger.Error(
                    switchException,
                    "切换 Modbus 模拟服务端失败。回退结果：{0}",
                    rollbackSucceeded ? "成功" : "失败");
                if (rollbackException != null)
                {
                    LogHelper.Logger.Error(
                        rollbackException,
                        "回退到上一个 Modbus 服务端配置失败。");
                }

                if (!rollbackSucceeded)
                {
                    _activeModbusServerConfiguration = null;
                }

                label10.ForeColor = rollbackSucceeded
                    ? Color.DarkOrange
                    : Color.Red;
                label10.Text = rollbackSucceeded
                    ? GetRunningStatusText("切换失败，已回退")
                    : "Modbus 服务端切换及回退均失败";
                MessageBox.Show(
                    rollbackSucceeded
                        ? string.Format(
                            "切换通信模式失败，已恢复原配置：{0}",
                            switchException.Message)
                        : string.Format(
                            "切换通信模式失败，且无法恢复原配置：{0}",
                            switchException.Message),
                    "Modbus 模式切换失败",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error);
            }
            finally
            {
                SetModbusConfigurationEnabled(true);
                button1.Enabled = !_isShuttingDown;
            }
        }

        private ModbusServerConfiguration
            BuildModbusServerConfiguration()
        {
            if (SelectedModbusTransportMode ==
                ModbusTransportMode.Rtu)
            {
                string selectedPort =
                    comboBox1.SelectedItem?.ToString();
                return new ModbusRtuServerConfiguration(
                    selectedPort,
                    9600,
                    Parity.None,
                    8,
                    StopBits.One);
            }

            IPAddress listenAddress;
            if (!IPAddress.TryParse(
                textBoxTcpListenAddress.Text.Trim(),
                out listenAddress))
            {
                throw new ArgumentException(
                    "TCP 监听 IP 地址格式无效。");
            }

            return new ModbusTcpServerConfiguration(
                listenAddress,
                Decimal.ToInt32(numericUpDownTcpPort.Value));
        }

        private ModbusTransportMode SelectedModbusTransportMode =>
            comboBoxModbusMode.SelectedIndex == 1
                ? ModbusTransportMode.Tcp
                : ModbusTransportMode.Rtu;

        private void SaveModbusServerConfiguration(
            ModbusServerConfiguration configuration)
        {
            Properties.Settings.Default.ModbusTransportMode =
                configuration.TransportMode.ToString();

            ModbusRtuServerConfiguration rtuConfiguration =
                configuration as ModbusRtuServerConfiguration;
            if (rtuConfiguration != null)
            {
                Properties.Settings.Default.ModbusSerialPort =
                    rtuConfiguration.PortName;
            }

            ModbusTcpServerConfiguration tcpConfiguration =
                configuration as ModbusTcpServerConfiguration;
            if (tcpConfiguration != null)
            {
                Properties.Settings.Default.ModbusTcpListenAddress =
                    tcpConfiguration.ListenAddress.ToString();
                Properties.Settings.Default.ModbusTcpPort =
                    tcpConfiguration.Port;
            }

            Properties.Settings.Default.Save();
        }

        private void ApplyModbusConfigurationToControls(
            ModbusServerConfiguration configuration)
        {
            ModbusRtuServerConfiguration rtuConfiguration =
                configuration as ModbusRtuServerConfiguration;
            if (rtuConfiguration != null)
            {
                comboBoxModbusMode.SelectedIndex = 0;
                if (!comboBox1.Items.Contains(
                    rtuConfiguration.PortName))
                {
                    comboBox1.Items.Add(
                        rtuConfiguration.PortName);
                }

                comboBox1.SelectedItem =
                    rtuConfiguration.PortName;
                return;
            }

            ModbusTcpServerConfiguration tcpConfiguration =
                configuration as ModbusTcpServerConfiguration;
            if (tcpConfiguration != null)
            {
                comboBoxModbusMode.SelectedIndex = 1;
                textBoxTcpListenAddress.Text =
                    tcpConfiguration.ListenAddress.ToString();
                numericUpDownTcpPort.Value =
                    tcpConfiguration.Port;
            }
        }

        private static bool AreEquivalentModbusConfigurations(
            ModbusServerConfiguration first,
            ModbusServerConfiguration second)
        {
            if (ReferenceEquals(first, second))
            {
                return true;
            }

            if (first == null ||
                second == null ||
                first.TransportMode != second.TransportMode)
            {
                return false;
            }

            ModbusRtuServerConfiguration firstRtu =
                first as ModbusRtuServerConfiguration;
            ModbusRtuServerConfiguration secondRtu =
                second as ModbusRtuServerConfiguration;
            if (firstRtu != null && secondRtu != null)
            {
                return string.Equals(
                        firstRtu.PortName,
                        secondRtu.PortName,
                        StringComparison.OrdinalIgnoreCase) &&
                    firstRtu.BaudRate == secondRtu.BaudRate &&
                    firstRtu.Parity == secondRtu.Parity &&
                    firstRtu.DataBits == secondRtu.DataBits &&
                    firstRtu.StopBits == secondRtu.StopBits;
            }

            ModbusTcpServerConfiguration firstTcp =
                first as ModbusTcpServerConfiguration;
            ModbusTcpServerConfiguration secondTcp =
                second as ModbusTcpServerConfiguration;
            return firstTcp != null &&
                secondTcp != null &&
                firstTcp.ListenAddress.Equals(
                    secondTcp.ListenAddress) &&
                firstTcp.Port == secondTcp.Port;
        }

        private void comboBoxModbusMode_SelectedIndexChanged(
            object sender,
            EventArgs e)
        {
            UpdateModbusConfigurationControls();
        }

        private void UpdateModbusConfigurationControls()
        {
            //bool isRtu = SelectedModbusTransportMode ==
            //    ModbusTransportMode.Rtu;

            ////label11.Visible = isRtu;
            ////comboBox1.Visible = isRtu;
            ////labelTcpListenAddress.Visible = !isRtu;
            ////textBoxTcpListenAddress.Visible = !isRtu;
            //////labelTcpPort.Visible = !isRtu;
            ////numericUpDownTcpPort.Visible = !isRtu;
        }

        private void SetModbusConfigurationEnabled(bool enabled)
        {
            comboBoxModbusMode.Enabled = enabled;
            comboBox1.Enabled = enabled;
            textBoxTcpListenAddress.Enabled = enabled;
            numericUpDownTcpPort.Enabled = enabled;
        }

        private string GetRunningStatusText(string prefix)
        {
            return string.Format(
                "{0} [{1}]",
                prefix,
                _modbusServer.EndpointDescription);
        }

        private void DisconnectAllTcpClients()
        {
            HydraulicOutputClientSession outputSession =
                DetachHydraulicOutputSession();
            if (outputSession != null)
            {
                outputSession.Dispose();
            }

            _doClient.Disconnect();
            _diClient.Disconnect();
        }

        private HydraulicOutputClientSession
            GetHydraulicOutputSession()
        {
            lock (_hydraulicOutputSessionLock)
            {
                return _hydraulicOutputSession;
            }
        }

        private HydraulicOutputClientSession
            GetOrCreateHydraulicOutputSession()
        {
            HydraulicOutputConfiguration configuration =
                BuildHydraulicOutputConfiguration();

            lock (_hydraulicOutputSessionLock)
            {
                if (_hydraulicOutputSession == null)
                {
                    _hydraulicOutputSession =
                        new HydraulicOutputClientSession(
                            configuration,
                            _lifetimeSource.Token);
                    SaveHydraulicOutputConfiguration(
                        configuration);
                    SetHydraulicOutputConfigurationEnabled(false);
                    LogHelper.Logger.Info(
                        "压力/流量 Modbus TCP 主站已创建：{0}，Unit ID={1}。",
                        configuration.Endpoint,
                        configuration.UnitIdentifier);
                }

                return _hydraulicOutputSession;
            }
        }

        private HydraulicOutputClientSession
            DetachHydraulicOutputSession()
        {
            lock (_hydraulicOutputSessionLock)
            {
                HydraulicOutputClientSession outputSession =
                    _hydraulicOutputSession;
                _hydraulicOutputSession = null;
                return outputSession;
            }
        }

        private HydraulicOutputConfiguration
            BuildHydraulicOutputConfiguration()
        {
            IPAddress targetAddress;
            if (!IPAddress.TryParse(
                textBoxHydraulicTargetIp.Text.Trim(),
                out targetAddress))
            {
                throw new ArgumentException(
                    "压力/流量直发目标 IP 地址格式无效。");
            }

            return new HydraulicOutputConfiguration(
                targetAddress,
                Decimal.ToInt32(
                    numericUpDownHydraulicPort.Value),
                Decimal.ToByte(
                    numericUpDownHydraulicUnitId.Value));
        }

        private void SaveHydraulicOutputConfiguration(
            HydraulicOutputConfiguration configuration)
        {
            Properties.Settings.Default.HydraulicOutputTargetIp =
                configuration.TargetAddress.ToString();
            Properties.Settings.Default.HydraulicOutputPort =
                configuration.Port;
            Properties.Settings.Default.HydraulicOutputUnitId =
                configuration.UnitIdentifier;
            Properties.Settings.Default.Save();
        }

        private void SetHydraulicOutputConfigurationEnabled(
            bool enabled)
        {
            textBoxHydraulicTargetIp.Enabled = enabled;
            numericUpDownHydraulicPort.Enabled = enabled;
            numericUpDownHydraulicUnitId.Enabled = enabled;
        }

        private async Task EnableHydraulicOutputAsync()
        {
            if (!_systemStarted ||
                _isShuttingDown ||
                !checkBox17.Checked)
            {
                return;
            }

            HydraulicOutputClientSession outputSession = null;

            try
            {
                outputSession =
                    GetOrCreateHydraulicOutputSession();
                label10.ForeColor = Color.DarkOrange;
                label10.Text = "压力/流量目标连接中...";

                CancellationToken cancellationToken =
                    outputSession.CancellationSource.Token;
                await outputSession.Client.ConnectAsync(
                    cancellationToken);

                if (ReferenceEquals(
                    GetHydraulicOutputSession(),
                    outputSession) &&
                    checkBox17.Checked)
                {
                    label10.ForeColor = Color.Green;
                    label10.Text = GetRunningStatusText("运行中");
                    LogHelper.Logger.Info(
                        "压力/流量 Modbus TCP 目标连接成功：{0}。",
                        outputSession.Configuration.Endpoint);
                }
            }
            catch (OperationCanceledException)
                when (outputSession != null &&
                    outputSession.CancellationSource
                        .IsCancellationRequested)
            {
                // 取消直发或程序退出时的正常结束。
            }
            catch (Exception ex)
            {
                LogHelper.Logger.Warn(
                    ex,
                    "压力/流量目标连接失败；后续 FC16 发送时将自动重连。");

                if (outputSession == null)
                {
                    checkBox17.Checked = false;
                    SetHydraulicOutputConfigurationEnabled(true);
                    MessageBox.Show(
                        ex.Message,
                        "压力/流量直发配置无效",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Error);
                }
                else if (ReferenceEquals(
                    GetHydraulicOutputSession(),
                    outputSession) &&
                    checkBox17.Checked)
                {
                    label10.ForeColor = Color.DarkOrange;
                    label10.Text = "压力/流量通信恢复中...";
                }
            }
        }

        private async Task DisableHydraulicOutputAsync()
        {
            HydraulicOutputClientSession outputSession =
                DetachHydraulicOutputSession();
            if (outputSession != null)
            {
                outputSession.CancellationSource.Cancel();
                await Task.Run(() => outputSession.Dispose());
                LogHelper.Logger.Info(
                    "压力/流量 Modbus TCP 直发已关闭，连接已释放。");
            }

            SetHydraulicOutputConfigurationEnabled(true);
        }

        private async void checkBox17_CheckedChanged(
            object sender,
            EventArgs e)
        {
            if (!_systemStarted || _isShuttingDown)
            {
                return;
            }

            if (checkBox17.Checked)
            {
                await EnableHydraulicOutputAsync();
            }
            else
            {
                await DisableHydraulicOutputAsync();
                if (!_isShuttingDown)
                {
                    label10.ForeColor = Color.Green;
                    label10.Text = GetRunningStatusText(
                        "运行中（压力/流量直发关闭）");
                }
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            ApplyCurrentFault(slave1, checkBox1.Checked, checkBox2.Checked);
            ApplyCurrentFault(slave2, checkBox4.Checked, checkBox3.Checked);
            ApplyCurrentFault(slave3, checkBox6.Checked, checkBox5.Checked);
            ApplyCurrentFault(slave4, checkBox8.Checked, checkBox7.Checked);

            SetPumpFaultState(slave1, checkBox13.Checked);
            SetPumpFaultState(slave2, checkBox14.Checked);
            SetPumpFaultState(slave3, checkBox15.Checked);
            SetPumpFaultState(slave4, checkBox16.Checked);

            UpdatePumpDisplay(slave1, textBox1, textBox2, textBox15);
            UpdatePumpDisplay(slave2, textBox4, textBox3, textBox22);
            UpdatePumpDisplay(slave3, textBox8, textBox7, textBox23);
            UpdatePumpDisplay(slave4, textBox6, textBox5, textBox24);

            ModbusRtuSlave.SaveEnergyHistoryIfDue();
            UpdateTcpConnectionStatus();
        }

        private void UpdateTcpConnectionStatus()
        {
            if (!_systemStarted || _isShuttingDown)
            {
                return;
            }

            List<ModbusTcpHealthSnapshot> snapshots =
                new List<ModbusTcpHealthSnapshot>
            {
                _doClient.GetHealthSnapshot(),
                _diClient.GetHealthSnapshot()
            };

            bool outputRequested = checkBox17.Checked;
            HydraulicOutputClientSession outputSession =
                GetHydraulicOutputSession();
            if (outputRequested && outputSession != null)
            {
                snapshots.Add(
                    outputSession.Client.GetHealthSnapshot());
            }

            bool isRecovering = snapshots.Any(
                snapshot =>
                    snapshot.State !=
                    ModbusTcpConnectionState.Connected) ||
                (outputRequested && outputSession == null);

            label10.ForeColor =
                isRecovering
                    ? Color.DarkOrange
                    : Color.Green;
            label10.Text =
                isRecovering
                    ? GetRunningStatusText("通信恢复中...")
                    : outputRequested
                        ? GetRunningStatusText("运行中")
                        : GetRunningStatusText(
                            "运行中（压力/流量直发关闭）");
        }

        private static void ApplyCurrentFault(
            ModbusRtuSlave slave,
            bool highCurrentFault,
            bool lowCurrentFault)
        {
            double? currentOverride = null;
            if (highCurrentFault)
            {
                currentOverride = 30.0;
            }
            else if (lowCurrentFault)
            {
                currentOverride = 0.3;
            }

            slave.SetCurrentOverride(currentOverride);
        }

        private static void SetPumpFaultState(ModbusRtuSlave slave, bool hasFault)
        {
            slave.SetHoldingRegister(0x2100, hasFault ? (ushort)4 : (ushort)1);
        }

        private static void UpdatePumpDisplay(
            ModbusRtuSlave slave,
            TextBox frequencyTextBox,
            TextBox currentTextBox,
            TextBox energyTextBox)
        {
            ModbusRtuSlave.PumpSnapshot snapshot = slave.GetSnapshot();
            frequencyTextBox.Text = snapshot.FrequencyHz.ToString("F2") + " HZ";
            currentTextBox.Text = snapshot.CurrentA.ToString("F2") + " A";
            energyTextBox.Text = snapshot.AccumulatedEnergyKwh.ToString("F3") + " kWh";
        }

        private static double Clamp(
            double value,
            double minimum,
            double maximum)
        {
            return Math.Max(minimum, Math.Min(maximum, value));
        }

        private void button2_Click(object sender, EventArgs e)
        {
            Application.Exit();
        }

        // 注意：该 Tick 由 UI 线程调用。将“所有网络 I/O”移到后台，并加防重入。
        private async void timer2_Tick(object sender, EventArgs e)
        {
            if (_timer2Busy || _isShuttingDown) return;
            _timer2Busy = true;
            try
            {
                const double flowSensorMaximum = 90.0;
                const double pressureSensorMaximum = 0.20;

                ModbusRtuSlave.PumpSnapshot p1Snapshot =
                    slave1.GetSnapshot();
                ModbusRtuSlave.PumpSnapshot n1Snapshot =
                    slave2.GetSnapshot();
                ModbusRtuSlave.PumpSnapshot p2Snapshot =
                    slave3.GetSnapshot();
                ModbusRtuSlave.PumpSnapshot n2Snapshot =
                    slave4.GetSnapshot();

                double P1_PV_Show = Clamp(
                    p1Snapshot.PressureMpa,
                    0.0,
                    pressureSensorMaximum);
                double N1_PV_Show = Clamp(
                    n1Snapshot.PressureMpa + PN1_PRESS_DIFF,
                    0.0,
                    pressureSensorMaximum);
                double P2_PV_Show = Clamp(
                    p2Snapshot.PressureMpa,
                    0.0,
                    pressureSensorMaximum);
                double N2_PV_Show = Clamp(
                    n2Snapshot.PressureMpa + PN2_PRESS_DIFF,
                    0.0,
                    pressureSensorMaximum);

                double P1_FV_Show = Clamp(
                    p1Snapshot.FlowM3PerHour,
                    0.0,
                    flowSensorMaximum);
                double N1_FV_Show = Clamp(
                    n1Snapshot.FlowM3PerHour + PN1_FLOW_DIFF,
                    0.0,
                    flowSensorMaximum);
                double P2_FV_Show = Clamp(
                    p2Snapshot.FlowM3PerHour,
                    0.0,
                    flowSensorMaximum);
                double N2_FV_Show = Clamp(
                    n2Snapshot.FlowM3PerHour + PN2_FLOW_DIFF,
                    0.0,
                    flowSensorMaximum);

                textBox10.Text = P1_PV_Show.ToString("F3") + "Mpa";
                textBox9.Text = N1_PV_Show.ToString("F3") + "Mpa";

                textBox14.Text = P1_FV_Show.ToString("F2") + "m³/h";
                textBox13.Text = N1_FV_Show.ToString("F2") + "m³/h";

                textBox21.Text = P2_PV_Show.ToString("F3") + "Mpa";
                textBox19.Text = N2_PV_Show.ToString("F3") + "Mpa";

                textBox16.Text = P2_FV_Show.ToString("F2") + "m³/h";
                textBox18.Text = N2_FV_Show.ToString("F2") + "m³/h";

                LogHelper.Logger.Debug(
                    "泵水力模拟：P1={0:F3}MPa/{1:F2}m³/h，" +
                    "N1={2:F3}MPa/{3:F2}m³/h，" +
                    "P2={4:F3}MPa/{5:F2}m³/h，" +
                    "N2={6:F3}MPa/{7:F2}m³/h",
                    P1_PV_Show,
                    P1_FV_Show,
                    N1_PV_Show,
                    N1_FV_Show,
                    P2_PV_Show,
                    P2_FV_Show,
                    N2_PV_Show,
                    N2_FV_Show);

                byte[] diWordArray = null;
                try
                {
                    diWordArray = await _diClient.ExecuteAsync(
                        "读取 DI 输入寄存器",
                        activeClient =>
                            activeClient.ReadInputRegisters(
                                DI_ModbusClient_ID,
                                0,
                                32).ToArray(),
                        _lifetimeSource.Token);
                }
                catch (OperationCanceledException)
                    when (_isShuttingDown)
                {
                    return;
                }
                catch (ModbusTcpCommunicationException)
                {
                    MarkDiDataInvalid();
                }
                catch (Exception ex)
                {
                    MarkDiDataInvalid();
                    LogHelper.Logger.Error(
                        ex,
                        "处理 DI 输入寄存器时发生异常。");
                }

                if (diWordArray != null && diWordArray.Length >= 64)
                {
                    lock (_diStateLock)
                    {
                        for (int i = 0; i < 32; i++)
                        {
                            DI_InputRegisters[i] = (ushort)(
                                (diWordArray[i * 2] << 8) |
                                diWordArray[i * 2 + 1]);
                        }

                        _diLastSuccessUtc = DateTime.UtcNow;
                        _diDataValid = true;
                    }
                }
                else if (diWordArray != null)
                {
                    MarkDiDataInvalid();
                    LogHelper.Logger.Error(
                        "DI 返回数据长度不足：期望至少 64 字节，实际 {0} 字节。",
                        diWordArray.Length);
                }


                ushort[] hydraulicRegisters =
                    HydraulicOutputRegisterMap.Encode(
                        P1_PV_Show,
                        P1_FV_Show,
                        N1_PV_Show,
                        N1_FV_Show,
                        P2_PV_Show,
                        P2_FV_Show,
                        N2_PV_Show,
                        N2_FV_Show);

                HydraulicOutputClientSession outputSession =
                    GetHydraulicOutputSession();
                if (checkBox17.Checked &&
                    outputSession != null)
                {
                    try
                    {
                        CancellationToken outputCancellationToken =
                            outputSession.CancellationSource.Token;
                        await outputSession.Client.ExecuteAsync(
                            "FC16 写入压力/流量寄存器 0-7",
                            activeClient =>
                                activeClient.WriteMultipleRegisters(
                                    outputSession.Configuration
                                        .UnitIdentifier,
                                    HydraulicOutputRegisterMap
                                        .StartingAddress,
                                    hydraulicRegisters),
                            outputCancellationToken);
                    }
                    catch (OperationCanceledException)
                        when (outputSession.CancellationSource
                            .IsCancellationRequested)
                    {
                        // 动态关闭直发或退出程序时的正常取消。
                    }
                    catch (ModbusTcpCommunicationException)
                    {
                        // 连接管理器已记录错误，下一轮发送会自动重连。
                    }
                }
            }
            catch (Exception exOuter)
            {
                LogHelper.Logger.Error(exOuter, "timer2_Tick 执行异常");
            }
            finally
            {
                _timer2Busy = false;
            }
        }

        private void textBox11_TextChanged(object sender, EventArgs e)
        {
            try
            {
                //if (checkBox9.Checked)
                //{
                //    PN1_PRESS_DIFF = -Convert.ToDouble(textBox11.Text);
                //}
                //else
                //{
                //    PN1_PRESS_DIFF = Convert.ToDouble(textBox11.Text);
                //}

                //Console.WriteLine("转换成功: " + PN1_PRESS_DIFF);
            }
            catch (FormatException)
            {
                // 捕获转换失败的异常
                Console.WriteLine("无效的数字格式");
            }
            catch (Exception ex)
            {
                // 其他异常
                Console.WriteLine("发生错误: " + ex.Message);
            }
        }

        private void textBox12_TextChanged(object sender, EventArgs e)
        {
            try
            {
                //if (checkBox10.Checked)
                //{
                //    PN1_FLOW_DIFF = -Convert.ToDouble(textBox12.Text);
                //}
                //else
                //{
                //    PN1_FLOW_DIFF = Convert.ToDouble(textBox12.Text);
                //}
                //Console.WriteLine("转换成功: " + PN1_FLOW_DIFF);
            }
            catch (FormatException)
            {
                // 捕获转换失败的异常
                Console.WriteLine("无效的数字格式");
            }
            catch (Exception ex)
            {
                // 其他异常
                Console.WriteLine("发生错误: " + ex.Message);
            }
        }

        private void textBox17_TextChanged(object sender, EventArgs e)
        {
            try
            {
                //if (checkBox11.Checked)
                //{
                //    PN2_PRESS_DIFF = -Convert.ToDouble(textBox17.Text);
                //}
                //else
                //{
                //    PN2_PRESS_DIFF = Convert.ToDouble(textBox17.Text);
                //}
                //Console.WriteLine("转换成功: " + PN2_PRESS_DIFF);
            }
            catch (FormatException)
            {
                // 捕获转换失败的异常
                Console.WriteLine("无效的数字格式");
            }
            catch (Exception ex)
            {
                // 其他异常
                Console.WriteLine("发生错误: " + ex.Message);
            }
        }

        private void textBox20_TextChanged(object sender, EventArgs e)
        {
            try
            {
                //if (checkBox12.Checked)
                //{
                //    PN2_FLOW_DIFF = -Convert.ToDouble(textBox20.Text);
                //}
                //else
                //{
                //    PN2_FLOW_DIFF = Convert.ToDouble(textBox20.Text);
                //}
                //Console.WriteLine("转换成功: " + PN2_FLOW_DIFF);
            }
            catch (FormatException)
            {
                // 捕获转换失败的异常
                Console.WriteLine("无效的数字格式");
            }
            catch (Exception ex)
            {
                // 其他异常
                Console.WriteLine("发生错误: " + ex.Message);
            }
        }

        private void checkBox9_CheckedChanged(object sender, EventArgs e)
        {
            PN1_PRESS_DIFF = -PN1_PRESS_DIFF;
        }

        private void checkBox10_CheckedChanged(object sender, EventArgs e)
        {
            PN1_FLOW_DIFF = -PN1_FLOW_DIFF;
        }

        private void MarkDiDataInvalid()
        {
            lock (_diStateLock)
            {
                _diDataValid = false;
            }
        }

        private bool TryGetFreshDiInputs(
            out ushort input2,
            out ushort input3)
        {
            lock (_diStateLock)
            {
                bool isFresh =
                    _diDataValid &&
                    (DateTime.UtcNow - _diLastSuccessUtc)
                        .TotalSeconds <=
                    DiDataMaximumAgeSeconds;

                if (!isFresh)
                {
                    input2 = 0;
                    input3 = 0;
                    return false;
                }

                input2 = DI_InputRegisters[2];
                input3 = DI_InputRegisters[3];
                return true;
            }
        }

        private void LogInvalidDiStateIfDue()
        {
            DateTime nowUtc = DateTime.UtcNow;
            if ((nowUtc - _lastDiInvalidLogUtc).TotalSeconds <
                30)
            {
                return;
            }

            _lastDiInvalidLogUtc = nowUtc;
            LogHelper.Logger.Warn(
                "DI 数据无效或已超过 {0} 秒未更新，暂停 DO 输出更新。",
                DiDataMaximumAgeSeconds);
        }

        private static ushort[] BuildDoOutputValues(
            ushort input2,
            ushort input3)
        {
            bool firstInputIsActive = input2 == 1;
            bool secondInputIsActive = input3 == 1;

            return new[]
            {
                firstInputIsActive ? (ushort)1 : (ushort)0,
                firstInputIsActive ? (ushort)0 : (ushort)1,
                secondInputIsActive ? (ushort)1 : (ushort)0,
                secondInputIsActive ? (ushort)0 : (ushort)1
            };
        }

        // DO 仅执行写入，不进行状态回读或写后确认。
        private async void timer3_Tick(object sender, EventArgs e)
        {
            if (_timer3Busy || _isShuttingDown) return;
            _timer3Busy = true;
            try
            {
                ushort input2;
                ushort input3;
                if (!TryGetFreshDiInputs(
                    out input2,
                    out input3))
                {
                    LogInvalidDiStateIfDue();
                    return;
                }

                ushort[] outputValues =
                    BuildDoOutputValues(input2, input3);

                try
                {
                    await _doClient.ExecuteAsync(
                        "写入 DO 输出",
                        activeClient =>
                            activeClient.WriteMultipleRegisters(
                                DO_ModbusClient_ID,
                                5,
                                outputValues),
                        _lifetimeSource.Token);
                }
                catch (OperationCanceledException)
                    when (_isShuttingDown)
                {
                    return;
                }
                catch (ModbusTcpCommunicationException)
                {
                    // 连接管理器已经限频记录，并将在下一次操作时自动重连。
                }
            }
            catch (Exception exOuter)
            {
                LogHelper.Logger.Error(exOuter, "timer3_Tick 执行异常");
            }
            finally
            {
                _timer3Busy = false;
            }
        }

        private void labelHydraulicScale_Click(object sender, EventArgs e)
        {

        }

        private void labelHydraulicRegisterMap_Click(object sender, EventArgs e)
        {

        }
    }
}

public class ModbusRtuSlave
{
    private const double RegisterScale = 100.0;
    // 现场 7.5 kW 泵由变频器在 0~50 Hz 范围内运行。
    private const double MaximumFrequencyHz = 50.0;
    private const double RampRateHzPerSecond = 2.5;
    private const double PowerKilowattsPerAmp = 0.42500269603871194;
    private const double RunningFrequencyThresholdHz = 0.5;
    private const double EnergyHistorySaveIntervalSeconds = 10.0;
    private const double HydraulicNoiseTimeConstantSeconds = 4.0;
    private const double FlowResponseTimeConstantSeconds = 1.2;
    private const double PressureResponseTimeConstantSeconds = 0.8;
    private const double MaximumSimulatedFlowM3PerHour = 90.0;
    private const double MaximumSimulatedPressureMpa = 0.20;

    private static readonly Dictionary<byte, ModbusRtuSlave> slaveInstances =
        new Dictionary<byte, ModbusRtuSlave>();
    private static readonly object energyHistorySaveLock = new object();
    private static readonly Stopwatch energyHistorySaveClock =
        Stopwatch.StartNew();
    private static readonly Dictionary<byte, double> initialEnergyByAddress;

    private static double lastEnergyHistorySaveSeconds;

    private readonly object stateLock = new object();
    private readonly Stopwatch simulationClock = Stopwatch.StartNew();
    private readonly Random random;
    private readonly byte slaveAddress;
    private readonly ushort[] holdingRegisters;

    private readonly double currentQuadraticCoefficient;
    private readonly double currentLinearCoefficient;
    private readonly double currentConstantCoefficient;
    private readonly double pressureLinearCoefficient;
    private readonly double pressureQuadraticCoefficient;
    private readonly double flowLinearCoefficient;
    private readonly double flowQuadraticCoefficient;
    private readonly double pressureNoiseStandardDeviation;
    private readonly double flowNoiseStandardDeviation;

    private double lastSimulationSeconds;
    private double currentFrequencyHz;
    private double targetFrequencyHz;
    private double simulatedCurrentA;
    private double currentNoiseA;
    private double instantaneousPowerKw;
    private double accumulatedEnergyKwh;
    private double pressureNoiseMpa;
    private double flowNoiseM3PerHour;
    private double simulatedPressureMpa;
    private double simulatedFlowM3PerHour;
    private double? currentOverrideA;

    public sealed class PumpSnapshot
    {
        public double FrequencyHz { get; internal set; }

        public double TargetFrequencyHz { get; internal set; }

        public double CurrentA { get; internal set; }

        public double InstantaneousPowerKw { get; internal set; }

        public double AccumulatedEnergyKwh { get; internal set; }

        public double PressureMpa { get; internal set; }

        public double FlowM3PerHour { get; internal set; }
    }

    // 静态构造函数只负责与通信传输无关的持久化生命周期。
    static ModbusRtuSlave()
    {
        initialEnergyByAddress = EnergyHistoryStore.Load();
        Application.ApplicationExit += OnApplicationExit;
    }

    // 构造函数，初始化每个从站
    public ModbusRtuSlave(byte slaveAddress)
    {
        this.slaveAddress = slaveAddress;
        holdingRegisters = new ushort[0x3010];
        random = new Random(Environment.TickCount ^ (slaveAddress << 16));

        /*
         * 电流曲线来源：
         * 冰山嘉德 BMS 1# 系统 2026-03-01 至 2026-03-25 实测数据，
         * 共 414,603 行。四个地址依次对应子1正极、子1负极、
         * 子2正极、子2负极泵。运行点和启停爬坡样本经异常值过滤后，
         * 分别拟合 I = a*f² + b*f + c；停机时电流单独置零。
         */
        switch (slaveAddress)
        {
            case 11:
                currentQuadraticCoefficient = 0.005063652281299397;
                currentLinearCoefficient = -0.05049377264325327;
                currentConstantCoefficient = 5.72586487089944;
                pressureLinearCoefficient = -0.0017519264267753013;
                pressureQuadraticCoefficient = 0.000054289938274595505;
                flowLinearCoefficient = 1.0470850992100933;
                flowQuadraticCoefficient = 0.001936579593091914;
                pressureNoiseStandardDeviation = 0.0008;
                flowNoiseStandardDeviation = 0.45;
                break;
            case 22:
                currentQuadraticCoefficient = 0.004814660602610448;
                currentLinearCoefficient = -0.030112118746925126;
                currentConstantCoefficient = 5.922344660623048;
                pressureLinearCoefficient = -0.0014111359146488102;
                pressureQuadraticCoefficient = 0.00004808691434077446;
                flowLinearCoefficient = 0.9488193515208853;
                flowQuadraticCoefficient = 0.002600661293946133;
                pressureNoiseStandardDeviation = 0.0008;
                flowNoiseStandardDeviation = 0.45;
                break;
            case 33:
                currentQuadraticCoefficient = 0.005129481141858832;
                currentLinearCoefficient = -0.047392561760144405;
                currentConstantCoefficient = 5.9362718966768675;
                pressureLinearCoefficient = -0.001822022767125942;
                pressureQuadraticCoefficient = 0.000054534776543527415;
                flowLinearCoefficient = 1.1770360145564065;
                flowQuadraticCoefficient = 0.0;
                pressureNoiseStandardDeviation = 0.0007;
                flowNoiseStandardDeviation = 0.65;
                break;
            case 44:
                currentQuadraticCoefficient = 0.004907373916841373;
                currentLinearCoefficient = -0.03910113450718124;
                currentConstantCoefficient = 6.053938877833055;
                pressureLinearCoefficient = -0.0014474703331022654;
                pressureQuadraticCoefficient = 0.00004910647037848411;
                flowLinearCoefficient = 1.1151560232888347;
                flowQuadraticCoefficient = -0.00015769372956885515;
                pressureNoiseStandardDeviation = 0.0008;
                flowNoiseStandardDeviation = 0.40;
                break;
            default:
                currentQuadraticCoefficient = 0.004978792;
                currentLinearCoefficient = -0.041774;
                currentConstantCoefficient = 5.909355;
                pressureLinearCoefficient = -0.0016081388604130798;
                pressureQuadraticCoefficient = 0.00005150452488434537;
                flowLinearCoefficient = 1.072024122144055;
                flowQuadraticCoefficient = 0.001094886789367298;
                pressureNoiseStandardDeviation = 0.0008;
                flowNoiseStandardDeviation = 0.50;
                break;
        }

        holdingRegisters[0x2100] = 1;
        double initialEnergyKwh;
        if (initialEnergyByAddress.TryGetValue(
            slaveAddress,
            out initialEnergyKwh))
        {
            accumulatedEnergyKwh = initialEnergyKwh;
        }

        lastSimulationSeconds = simulationClock.Elapsed.TotalSeconds;

        lock (slaveInstances)
        {
            slaveInstances[slaveAddress] = this;
        }
    }

    internal byte SlaveAddress => slaveAddress;

    internal int HoldingRegisterCount => holdingRegisters.Length;

    public static string HistoryEnergyFilePath =>
        EnergyHistoryStore.FilePath;

    public static void SaveEnergyHistoryIfDue()
    {
        lock (energyHistorySaveLock)
        {
            double nowSeconds =
                energyHistorySaveClock.Elapsed.TotalSeconds;
            if (nowSeconds - lastEnergyHistorySaveSeconds <
                EnergyHistorySaveIntervalSeconds)
            {
                return;
            }

            SaveEnergyHistoryUnsafe();
        }
    }

    public static void SaveEnergyHistory()
    {
        lock (energyHistorySaveLock)
        {
            SaveEnergyHistoryUnsafe();
        }
    }

    // 频率命令寄存器使用 0.01 Hz，模拟输出寄存器使用 0.01 Hz / 0.01 A。
    public void SetHoldingRegister(ushort address, ushort value)
    {
        if (address >= holdingRegisters.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(address));
        }

        UpdateSimulation();

        lock (stateLock)
        {
            if (address == 0x2001)
            {
                targetFrequencyHz = Math.Max(
                    0.0,
                    Math.Min(MaximumFrequencyHz, value / RegisterScale));
                holdingRegisters[address] = ToRegisterValue(
                    targetFrequencyHz,
                    RegisterScale);
            }
            else
            {
                holdingRegisters[address] = value;
            }
        }
    }

    public ushort GetHoldingRegister(ushort address)
    {
        if (address >= holdingRegisters.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(address));
        }

        UpdateSimulation();
        lock (stateLock)
        {
            return holdingRegisters[address];
        }
    }

    internal ushort[] ReadHoldingRegisters(
        ushort startingAddress,
        ushort quantity)
    {
        int endAddressExclusive = startingAddress + quantity;
        if (quantity == 0 ||
            startingAddress >= holdingRegisters.Length ||
            endAddressExclusive > holdingRegisters.Length)
        {
            throw new ArgumentOutOfRangeException(
                nameof(startingAddress));
        }

        UpdateSimulation();
        lock (stateLock)
        {
            ushort[] values = new ushort[quantity];
            Array.Copy(
                holdingRegisters,
                startingAddress,
                values,
                0,
                quantity);
            return values;
        }
    }

    public void SetCurrentOverride(double? currentA)
    {
        UpdateSimulation();
        lock (stateLock)
        {
            currentOverrideA = currentA.HasValue
                ? Math.Max(0.0, Math.Min(100.0, currentA.Value))
                : (double?)null;
            UpdateOutputRegistersUnsafe();
        }
    }

    public PumpSnapshot GetSnapshot()
    {
        UpdateSimulation();
        lock (stateLock)
        {
            return new PumpSnapshot
            {
                FrequencyHz = currentFrequencyHz,
                TargetFrequencyHz = targetFrequencyHz,
                CurrentA = simulatedCurrentA,
                InstantaneousPowerKw = instantaneousPowerKw,
                AccumulatedEnergyKwh = accumulatedEnergyKwh,
                PressureMpa = simulatedPressureMpa,
                FlowM3PerHour = simulatedFlowM3PerHour
            };
        }
    }

    public float GetCurrent()
    {
        return (float)GetSnapshot().CurrentA;
    }

    public ushort GetFinalCurrent()
    {
        return ToRegisterValue(GetSnapshot().CurrentA, RegisterScale);
    }

    public double GetAccumulatedEnergyKwh()
    {
        return GetSnapshot().AccumulatedEnergyKwh;
    }

    private void UpdateSimulation()
    {
        double nowSeconds = simulationClock.Elapsed.TotalSeconds;

        lock (stateLock)
        {
            double elapsedSeconds = nowSeconds - lastSimulationSeconds;
            if (elapsedSeconds <= 0)
            {
                return;
            }

            double frequencyDifference = targetFrequencyHz - currentFrequencyHz;
            double rampDurationSeconds = Math.Min(
                elapsedSeconds,
                Math.Abs(frequencyDifference) / RampRateHzPerSecond);
            double rampDirection = Math.Sign(frequencyDifference);
            double startFrequencyHz = currentFrequencyHz;
            double endFrequencyHz = startFrequencyHz +
                rampDirection * RampRateHzPerSecond * rampDurationSeconds;

            if (Math.Abs(targetFrequencyHz - endFrequencyHz) < 0.0001)
            {
                endFrequencyHz = targetFrequencyHz;
            }

            double endPowerKw = CalculateBasePowerKw(endFrequencyHz);
            double rampEnergyKwh = CalculateRampEnergyKwh(
                startFrequencyHz,
                endFrequencyHz);
            double steadyEnergyKwh =
                endPowerKw *
                (elapsedSeconds - rampDurationSeconds) / 3600.0;

            accumulatedEnergyKwh += rampEnergyKwh + steadyEnergyKwh;
            currentFrequencyHz = endFrequencyHz;
            instantaneousPowerKw = endPowerKw;
            lastSimulationSeconds = nowSeconds;

            if (currentFrequencyHz >= RunningFrequencyThresholdHz)
            {
                double newNoiseA = (random.NextDouble() * 2.0 - 1.0) * 0.18;
                double smoothingFactor =
                    1.0 - Math.Exp(-elapsedSeconds / 1.5);
                currentNoiseA =
                    currentNoiseA * (1.0 - smoothingFactor) +
                    newNoiseA * smoothingFactor;
            }
            else
            {
                currentNoiseA = 0.0;
            }

            UpdateHydraulicStateUnsafe(elapsedSeconds);
            UpdateOutputRegistersUnsafe();
        }
    }

    private void UpdateHydraulicStateUnsafe(double elapsedSeconds)
    {
        double basePressureMpa =
            CalculateBasePressureMpa(currentFrequencyHz);
        double baseFlowM3PerHour =
            CalculateBaseFlowM3PerHour(currentFrequencyHz);

        double noiseSmoothingFactor =
            1.0 - Math.Exp(
                -elapsedSeconds /
                HydraulicNoiseTimeConstantSeconds);

        if (currentFrequencyHz >= RunningFrequencyThresholdHz)
        {
            double pressureNoiseTarget =
                basePressureMpa > 0.0
                    ? NextGaussianLike() *
                        pressureNoiseStandardDeviation
                    : 0.0;
            double flowNoiseTarget =
                NextGaussianLike() *
                flowNoiseStandardDeviation;

            pressureNoiseMpa +=
                (pressureNoiseTarget - pressureNoiseMpa) *
                noiseSmoothingFactor;
            flowNoiseM3PerHour +=
                (flowNoiseTarget - flowNoiseM3PerHour) *
                noiseSmoothingFactor;
        }
        else
        {
            pressureNoiseMpa = 0.0;
            flowNoiseM3PerHour = 0.0;
        }

        double targetPressureMpa = Math.Max(
            0.0,
            basePressureMpa + pressureNoiseMpa);
        double targetFlowM3PerHour = Math.Max(
            0.0,
            baseFlowM3PerHour + flowNoiseM3PerHour);

        double pressureResponseFactor =
            1.0 - Math.Exp(
                -elapsedSeconds /
                PressureResponseTimeConstantSeconds);
        double flowResponseFactor =
            1.0 - Math.Exp(
                -elapsedSeconds /
                FlowResponseTimeConstantSeconds);

        simulatedPressureMpa +=
            (targetPressureMpa - simulatedPressureMpa) *
            pressureResponseFactor;
        simulatedFlowM3PerHour +=
            (targetFlowM3PerHour - simulatedFlowM3PerHour) *
            flowResponseFactor;

        simulatedPressureMpa = Math.Max(
            0.0,
            Math.Min(
                MaximumSimulatedPressureMpa,
                simulatedPressureMpa));
        simulatedFlowM3PerHour = Math.Max(
            0.0,
            Math.Min(
                MaximumSimulatedFlowM3PerHour,
                simulatedFlowM3PerHour));
    }

    private void UpdateOutputRegistersUnsafe()
    {
        double baseCurrentA = CalculateBaseCurrentA(currentFrequencyHz);
        double measuredCurrentA = baseCurrentA + currentNoiseA;

        if (currentFrequencyHz < RunningFrequencyThresholdHz)
        {
            measuredCurrentA = 0.0;
        }
        else if (currentOverrideA.HasValue)
        {
            measuredCurrentA = currentOverrideA.Value;
        }

        simulatedCurrentA = Math.Max(0.0, measuredCurrentA);
        holdingRegisters[0x3000] = ToRegisterValue(
            currentFrequencyHz,
            RegisterScale);
        holdingRegisters[0x3004] = ToRegisterValue(
            simulatedCurrentA,
            RegisterScale);
    }

    private double CalculateBaseCurrentA(double frequencyHz)
    {
        if (frequencyHz < RunningFrequencyThresholdHz)
        {
            return 0.0;
        }

        double boundedFrequencyHz = Math.Min(
            MaximumFrequencyHz,
            frequencyHz);
        double currentA =
            currentQuadraticCoefficient *
                boundedFrequencyHz *
                boundedFrequencyHz +
            currentLinearCoefficient * boundedFrequencyHz +
            currentConstantCoefficient;

        return Math.Max(0.0, Math.Min(30.0, currentA));
    }

    private double CalculateBasePressureMpa(double frequencyHz)
    {
        if (frequencyHz < RunningFrequencyThresholdHz)
        {
            return 0.0;
        }

        double pressureMpa =
            pressureLinearCoefficient * frequencyHz +
            pressureQuadraticCoefficient *
                frequencyHz * frequencyHz;
        return Math.Max(
            0.0,
            Math.Min(
                MaximumSimulatedPressureMpa,
                pressureMpa));
    }

    private double CalculateBaseFlowM3PerHour(double frequencyHz)
    {
        if (frequencyHz < RunningFrequencyThresholdHz)
        {
            return 0.0;
        }

        double flowM3PerHour =
            flowLinearCoefficient * frequencyHz +
            flowQuadraticCoefficient *
                frequencyHz * frequencyHz;
        return Math.Max(
            0.0,
            Math.Min(
                MaximumSimulatedFlowM3PerHour,
                flowM3PerHour));
    }

    private double NextGaussianLike()
    {
        double sum = 0.0;
        for (int index = 0; index < 6; index++)
        {
            sum += random.NextDouble();
        }

        return (sum - 3.0) * 1.4142135623730951;
    }

    private double CalculateBasePowerKw(double frequencyHz)
    {
        return CalculateBaseCurrentA(frequencyHz) * PowerKilowattsPerAmp;
    }

    private double CalculateRampEnergyKwh(
        double startFrequencyHz,
        double endFrequencyHz)
    {
        double lowerFrequencyHz = Math.Max(
            RunningFrequencyThresholdHz,
            Math.Min(startFrequencyHz, endFrequencyHz));
        double upperFrequencyHz = Math.Max(
            RunningFrequencyThresholdHz,
            Math.Max(startFrequencyHz, endFrequencyHz));

        if (upperFrequencyHz <= lowerFrequencyHz)
        {
            return 0.0;
        }

        double integratedCurrentAmpHz =
            currentQuadraticCoefficient / 3.0 *
                (Math.Pow(upperFrequencyHz, 3) - Math.Pow(lowerFrequencyHz, 3)) +
            currentLinearCoefficient / 2.0 *
                (Math.Pow(upperFrequencyHz, 2) - Math.Pow(lowerFrequencyHz, 2)) +
            currentConstantCoefficient *
                (upperFrequencyHz - lowerFrequencyHz);

        return integratedCurrentAmpHz * PowerKilowattsPerAmp /
            RampRateHzPerSecond / 3600.0;
    }

    private static ushort ToRegisterValue(double value, double scale)
    {
        double scaledValue = Math.Round(value * scale);
        return (ushort)Math.Max(
            ushort.MinValue,
            Math.Min(ushort.MaxValue, scaledValue));
    }

    private static void SaveEnergyHistoryUnsafe()
    {
        List<ModbusRtuSlave> slaves;
        lock (slaveInstances)
        {
            slaves = slaveInstances.Values.ToList();
        }

        if (slaves.Count == 0)
        {
            return;
        }

        KeyValuePair<byte, double>[] energyByAddress = slaves
            .Select(slave => new KeyValuePair<byte, double>(
                slave.slaveAddress,
                slave.GetAccumulatedEnergyKwh()))
            .ToArray();

        try
        {
            EnergyHistoryStore.Save(energyByAddress);
            lastEnergyHistorySaveSeconds =
                energyHistorySaveClock.Elapsed.TotalSeconds;
        }
        catch (Exception ex)
        {
            LogHelper.Logger.Error(
                ex,
                "保存累计功耗历史文件失败：{0}",
                EnergyHistoryStore.FilePath);
        }
    }

    private static void OnApplicationExit(
        object sender,
        EventArgs e)
    {
        SaveEnergyHistory();
    }
}

public static class LogHelper
{
    // 创建全局静态的 Logger 实例
    private static readonly Logger logger = LogManager.GetCurrentClassLogger();

    public static Logger Logger => logger;
}
