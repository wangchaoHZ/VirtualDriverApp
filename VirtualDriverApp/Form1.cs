using FluentModbus;
using NLog;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Linq;
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
            foreach (Font font in _responsiveFonts.Values)
            {
                font.Dispose();
            }

            _responsiveFonts.Clear();
        }

        private ModbusTcpClient AI01_ModbusClient;
        private byte AI01_ModbusClient_ID = 5;

        private ModbusTcpClient AI02_ModbusClient;

        private ModbusTcpClient DO_ModbusClient;
        private byte DO_ModbusClient_ID = 2;

        private ModbusTcpClient DI_ModbusClient;
        private byte DI_ModbusClient_ID = 3;

        // 创建四个从站实例，分别为地址11、22、33、44
        ModbusRtuSlave slave1 = new ModbusRtuSlave(11);
        ModbusRtuSlave slave2 = new ModbusRtuSlave(22);
        ModbusRtuSlave slave3 = new ModbusRtuSlave(33);
        ModbusRtuSlave slave4 = new ModbusRtuSlave(44);

        private double P1_Cur;
        private double N1_Cur;
        private double P2_Cur;
        private double N2_Cur;

        private double PN1_PRESS_DIFF;
        private double PN2_PRESS_DIFF;
        private double PN1_FLOW_DIFF;
        private double PN2_FLOW_DIFF;

        private ushort[] DI_InputRegisters = new ushort[32];

        // 新增：用于串行化各 Modbus 客户端的访问，避免并发与潜在死锁
        private readonly SemaphoreSlim _ai01Lock = new SemaphoreSlim(1, 1);
        private readonly SemaphoreSlim _diLock = new SemaphoreSlim(1, 1);
        private readonly SemaphoreSlim _doLock = new SemaphoreSlim(1, 1);

        // 新增：防止 Timer 重入（定时器回调在 UI 线程，网络慢时容易叠加）
        private volatile bool _timer2Busy = false;
        private volatile bool _timer3Busy = false;

        private void Form1_Load(object sender, EventArgs e)
        {
            label10.ForeColor = Color.Black;
            label10.Text = "未启动";
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

            // 如果有可用串口，默认选择第一个串口
            if (comboBox1.Items.Count > 0)
            {
                comboBox1.SelectedIndex = 0;
            }

            // 为每个从站设置保持寄存器的初始值
            slave1.SetHoldingRegister(0, 0);  // 设置从站11的寄存器0初始值
            slave2.SetHoldingRegister(0, 0);  // 设置从站22的寄存器0初始值
            slave3.SetHoldingRegister(0, 0);  // 设置从站33的寄存器0初始值
            slave4.SetHoldingRegister(0, 0);  // 设置从站44的寄存器0初始值
        }


        // 简单写寄存器带重试方法
        private void WriteRegisterWithRetry(FluentModbus.ModbusClient client, int clientId, int register, short value, int retryCount = 5)
        {
            for (int i = 0; i < retryCount; i++)
            {
                try
                {
                    client.WriteSingleRegister(clientId, register, value);
                    break; // 成功写入跳出循环
                }
                catch (FluentModbus.ModbusException ex)
                {
                    // 可记录日志或显示提示
                    Console.WriteLine($"Modbus写入寄存器异常: {register}, 第{i + 1}次尝试: {ex.Message}");
                    System.Threading.Thread.Sleep(180); // 失败时稍微延时再重试
                                                        // 最后一次失败可做告警或忽略
                }
                catch (Exception ex)
                {
                    // 其他异常也捕获
                    Console.WriteLine($"其他异常: {ex.Message}");
                    System.Threading.Thread.Sleep(200);
                }
            }
        }

        // 修改为异步，避免在 UI 线程同步阻塞 Connect 和 Sleep
        private async void button1_Click(object sender, EventArgs e)
        {
            button1.Enabled = false;
            label10.ForeColor = Color.Black;
            label10.Text = "正在连接...";

            try
            {
                // ModbusTcpClient实例化
                AI01_ModbusClient = new ModbusTcpClient();
                AI02_ModbusClient = new ModbusTcpClient();
                DO_ModbusClient = new ModbusTcpClient();
                DI_ModbusClient = new ModbusTcpClient();

                string selectedPort = comboBox1.SelectedItem?.ToString();

                // 在后台线程执行所有可能阻塞的 I/O（网络连接、串口打开）
                await Task.Run(() =>
                {
                    // 依次连接"192.168.1.133", "192.168.1.134" "192.168.1.131", "192.168.1.132" 
                    AI01_ModbusClient.Connect("192.168.1.133", ModbusEndianness.BigEndian);
                    AI02_ModbusClient.Connect("192.168.1.134", ModbusEndianness.BigEndian);

                    DO_ModbusClient.Connect("192.168.1.131", ModbusEndianness.BigEndian);
                    DI_ModbusClient.Connect("192.168.1.132", ModbusEndianness.BigEndian);

                    // comboBox1为变频器端口选择窗
                    // 设置串口配置（打开串口）
                    ModbusRtuSlave.SetSerialPortSettings(selectedPort, 9600, Parity.None, 8, StopBits.One);

                    // 稍作延时给串口/设备稳定（后台线程里，不阻塞 UI）
                    Thread.Sleep(250);
                    WriteRegisterWithRetry(DO_ModbusClient, DO_ModbusClient_ID, 4, 1);
                    Thread.Sleep(250);
                    WriteRegisterWithRetry(DO_ModbusClient, DO_ModbusClient_ID, 4, 1);
                    Thread.Sleep(250);
                    WriteRegisterWithRetry(DO_ModbusClient, DO_ModbusClient_ID, 4, 1);
                    Thread.Sleep(250);
                    WriteRegisterWithRetry(DO_ModbusClient, DO_ModbusClient_ID, 4, 1);

                    // 启动共享的 Modbus 线程
                    ModbusRtuSlave.Start();
                });

                button1.ForeColor = Color.Green;
                label10.ForeColor = Color.Green;
                label10.Text = "运行中";

                // 直接启用定时器（不要再用 Thread.Sleep 阻塞 UI）
                timer1.Enabled = true;
                timer2.Enabled = true;
                timer3.Enabled = true;
            }
            catch (Exception ex)
            {
                label10.ForeColor = Color.Red;
                label10.Text = "连接失败";
                LogHelper.Logger.Error(ex, "启动连接失败");
                MessageBox.Show($"连接设备失败：{ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                button1.Enabled = true;
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

            P1_Cur = UpdatePumpDisplay(slave1, textBox1, textBox2, textBox15);
            N1_Cur = UpdatePumpDisplay(slave2, textBox4, textBox3, textBox22);
            P2_Cur = UpdatePumpDisplay(slave3, textBox8, textBox7, textBox23);
            N2_Cur = UpdatePumpDisplay(slave4, textBox6, textBox5, textBox24);
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

        private static double UpdatePumpDisplay(
            ModbusRtuSlave slave,
            TextBox frequencyTextBox,
            TextBox currentTextBox,
            TextBox energyTextBox)
        {
            ModbusRtuSlave.PumpSnapshot snapshot = slave.GetSnapshot();
            frequencyTextBox.Text = snapshot.FrequencyHz.ToString("F2") + " HZ";
            currentTextBox.Text = snapshot.CurrentA.ToString("F2") + " A";
            energyTextBox.Text = snapshot.AccumulatedEnergyKwh.ToString("F3") + " kWh";
            return snapshot.CurrentA;
        }
        private void button2_Click(object sender, EventArgs e)
        {
            Application.Exit();
        }

        // 注意：该 Tick 由 UI 线程调用。将“所有网络 I/O”移到后台，并加防重入。
        private async void timer2_Tick(object sender, EventArgs e)
        {
            if (_timer2Busy) return;
            _timer2Busy = true;
            try
            {
                double flow_max = 90.0;
                double press_max = 0.20;

                double Cal_Base = 15.0; // 基准值

                double P1_PV_Show = ((P1_Cur / Cal_Base) * (press_max));
                double N1_PV_Show = ((N1_Cur / Cal_Base) * (press_max)) + PN1_PRESS_DIFF;
                double P2_PV_Show = ((P2_Cur / Cal_Base) * (press_max));
                double N2_PV_Show = ((N2_Cur / Cal_Base) * (press_max)) + PN2_PRESS_DIFF;

                double P1_FV_Show = ((P1_Cur / Cal_Base) * (flow_max));
                double N1_FV_Show = ((N1_Cur / Cal_Base) * (flow_max)) + PN1_FLOW_DIFF;
                double P2_FV_Show = ((P2_Cur / Cal_Base) * (flow_max));
                double N2_FV_Show = ((N2_Cur / Cal_Base) * (flow_max)) + PN2_FLOW_DIFF;

                textBox10.Text = P1_PV_Show.ToString("F3") + "Mpa";
                textBox9.Text = N1_PV_Show.ToString("F3") + "Mpa";

                LogHelper.Logger.Info("---------------------------------------------");
                LogHelper.Logger.Info("P1_PV_Show:" + textBox10.Text + " N1_PV_Show:" + textBox9.Text);

                textBox14.Text = P1_FV_Show.ToString("F2") + "m³/h";
                textBox13.Text = N1_FV_Show.ToString("F2") + "m³/h";

                LogHelper.Logger.Info("P1_FV_Show:" + textBox14.Text + " N1_FV_Show:" + textBox13.Text);

                textBox21.Text = P2_PV_Show.ToString("F3") + "Mpa";
                textBox19.Text = N2_PV_Show.ToString("F3") + "Mpa";

                LogHelper.Logger.Info("P2_PV_Show:" + textBox21.Text + " N2_PV_Show:" + textBox19.Text);

                textBox16.Text = P2_FV_Show.ToString("F2") + "m³/h";
                textBox18.Text = N2_FV_Show.ToString("F2") + "m³/h";

                LogHelper.Logger.Info("P2_FV_Show:" + textBox16.Text + " N2_FV_Show:" + textBox18.Text);
                LogHelper.Logger.Info("---------------------------------------------");


                double flow_sensor_max = 90.0;
                double press_sensor_max = 0.20;

                ushort P1_PV_SET = (ushort)((P1_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
                ushort P2_PV_SET = (ushort)((P2_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
                ushort N1_PV_SET = (ushort)((N1_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
                ushort N2_PV_SET = (ushort)((N2_PV_Show / press_sensor_max) * 16000.0 + 4000.0);

                ushort P1_FV_SET = (ushort)((P1_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
                ushort P2_FV_SET = (ushort)((P2_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
                ushort N1_FV_SET = (ushort)((N1_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
                ushort N2_FV_SET = (ushort)((N2_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);

                ushort S1_CUR_SET = 0;
                ushort S2_CUR_SET = 0;


                const ushort H2_SET = 4120;

                Console.WriteLine("S1 Cur Set:" + S1_CUR_SET.ToString() + "S2 Cur Set:" + S2_CUR_SET);

                // 把 DI 读取放到后台线程，并串行化访问，避免在 UI 线程阻塞
                byte[] diWordArray = null;
                try
                {
                    diWordArray = await Task.Run(async () =>
                    {
                        await _diLock.WaitAsync();
                        try
                        {
                            // 如果设备未连接，直接抛异常让上层捕获
                            return DI_ModbusClient.ReadInputRegisters(DI_ModbusClient_ID, 0, 32).ToArray();
                        }
                        finally
                        {
                            _diLock.Release();
                        }
                    });
                }
                catch (Exception ex)
                {
                    LogHelper.Logger.Error(ex, "读取 DI 输入寄存器失败");
                }

                if (diWordArray != null && diWordArray.Length >= 64)
                {
                    for (int i = 0; i < 32; i++)
                    {
                        DI_InputRegisters[i] = (ushort)((diWordArray[i * 2] << 8) | diWordArray[i * 2 + 1]);
                    }
                }


                // 发送 Modbus 请求到后台线程（只写操作放后台）
                ushort startAddress = 10;

                ushort[] values =
                {
                    P1_PV_SET,
                    N1_PV_SET,

                    P1_FV_SET,
                    N1_FV_SET,

                    P2_PV_SET,
                    N2_PV_SET,

                    P2_FV_SET,
                    N2_FV_SET,

                    S1_CUR_SET,
                    H2_SET,
                    S2_CUR_SET,
                    H2_SET,
                };

                try
                {
                    await Task.Run(async () =>
                    {
                        await _ai01Lock.WaitAsync();
                        try
                        {
                            AI01_ModbusClient.WriteMultipleRegisters(AI01_ModbusClient_ID, startAddress, values);
                        }
                        finally
                        {
                            _ai01Lock.Release();
                        }
                    });
                }
                catch (FluentModbus.ModbusException ex)
                {
                    // 捕获 Modbus 异常
                    Console.WriteLine($"Modbus 写入异常: {ex.Message}");
                }
                catch (Exception ex)
                {
                    // 捕获其他异常
                    Console.WriteLine($"其他异常: {ex.Message}");
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
                if (checkBox9.Checked)
                {
                    PN1_PRESS_DIFF = -Convert.ToDouble(textBox11.Text);
                }
                else
                {
                    PN1_PRESS_DIFF = Convert.ToDouble(textBox11.Text);
                }

                Console.WriteLine("转换成功: " + PN1_PRESS_DIFF);
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
                if (checkBox10.Checked)
                {
                    PN1_FLOW_DIFF = -Convert.ToDouble(textBox12.Text);
                }
                else
                {
                    PN1_FLOW_DIFF = Convert.ToDouble(textBox12.Text);
                }
                Console.WriteLine("转换成功: " + PN1_FLOW_DIFF);
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
                if (checkBox11.Checked)
                {
                    PN2_PRESS_DIFF = -Convert.ToDouble(textBox17.Text);
                }
                else
                {
                    PN2_PRESS_DIFF = Convert.ToDouble(textBox17.Text);
                }
                Console.WriteLine("转换成功: " + PN2_PRESS_DIFF);
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
                if (checkBox12.Checked)
                {
                    PN2_FLOW_DIFF = -Convert.ToDouble(textBox20.Text);
                }
                else
                {
                    PN2_FLOW_DIFF = Convert.ToDouble(textBox20.Text);
                }
                Console.WriteLine("转换成功: " + PN2_FLOW_DIFF);
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

        // 修改为异步，避免在 UI 线程内同步网络写阻塞
        private async void timer3_Tick(object sender, EventArgs e)
        {
            if (_timer3Busy) return;
            _timer3Busy = true;
            try
            {
                if (DI_InputRegisters[2] == 1 && DI_InputRegisters[3] == 1)
                {
                    ushort[] open_values =
                    {
                        1,
                        0,
                        1,
                        0
                    };

                    try
                    {
                        await Task.Run(async () =>
                        {
                            await _doLock.WaitAsync();
                            try
                            {
                                DO_ModbusClient.WriteMultipleRegisters(DO_ModbusClient_ID, 5, open_values);
                            }
                            finally
                            {
                                _doLock.Release();
                            }
                        });
                    }
                    catch (FluentModbus.ModbusException ex)
                    {
                        // 捕获 Modbus 异常
                        Console.WriteLine($"Modbus 写入异常: {ex.Message}");
                    }
                    catch (Exception ex)
                    {
                        // 捕获其他异常
                        Console.WriteLine($"其他异常: {ex.Message}");
                    }
                }
                else if (DI_InputRegisters[2] == 1 && DI_InputRegisters[3] == 0)
                {
                    ushort[] open_values =
                    {
                        1,
                        0,
                        0,
                        1
                    };

                    try
                    {
                        await Task.Run(async () =>
                        {
                            await _doLock.WaitAsync();
                            try
                            {
                                DO_ModbusClient.WriteMultipleRegisters(DO_ModbusClient_ID, 5, open_values);
                            }
                            finally
                            {
                                _doLock.Release();
                            }
                        });
                    }
                    catch (FluentModbus.ModbusException ex)
                    {
                        // 捕获 Modbus 异常
                        Console.WriteLine($"Modbus 写入异常: {ex.Message}");
                    }
                    catch (Exception ex)
                    {
                        // 捕获其他异常
                        Console.WriteLine($"其他异常: {ex.Message}");
                    }
                }
                else if (DI_InputRegisters[2] == 0 && DI_InputRegisters[3] == 1)
                {
                    ushort[] open_values =
                    {
                        0,
                        1,
                        1,
                        0
                    };

                    try
                    {
                        await Task.Run(async () =>
                        {
                            await _doLock.WaitAsync();
                            try
                            {
                                DO_ModbusClient.WriteMultipleRegisters(DO_ModbusClient_ID, 5, open_values);
                            }
                            finally
                            {
                                _doLock.Release();
                            }
                        });
                    }
                    catch (FluentModbus.ModbusException ex)
                    {
                        // 捕获 Modbus 异常
                        Console.WriteLine($"Modbus 写入异常: {ex.Message}");
                    }
                    catch (Exception ex)
                    {
                        // 捕获其他异常
                        Console.WriteLine($"其他异常: {ex.Message}");
                    }
                }
                else
                {
                    ushort[] open_values =
                    {
                        0,
                        1,
                        0,
                        1
                    };

                    try
                    {
                        await Task.Run(async () =>
                        {
                            await _doLock.WaitAsync();
                            try
                            {
                                DO_ModbusClient.WriteMultipleRegisters(DO_ModbusClient_ID, 5, open_values);
                            }
                            finally
                            {
                                _doLock.Release();
                            }
                        });
                    }
                    catch (FluentModbus.ModbusException ex)
                    {
                        // 捕获 Modbus 异常
                        Console.WriteLine($"Modbus 写入异常: {ex.Message}");
                    }
                    catch (Exception ex)
                    {
                        // 捕获其他异常
                        Console.WriteLine($"其他异常: {ex.Message}");
                    }
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

    }
}

public class ModbusRtuSlave
{
    private const double RegisterScale = 100.0;
    private const double MaximumFrequencyHz = 60.0;
    private const double RampRateHzPerSecond = 2.5;
    private const double PowerKilowattsPerAmp = 0.42500269603871194;
    private const double RunningFrequencyThresholdHz = 0.5;

    private static bool isPortOpen;
    private static SerialPort serialPort;
    private static readonly Dictionary<byte, ModbusRtuSlave> slaveInstances =
        new Dictionary<byte, ModbusRtuSlave>();

    private readonly object stateLock = new object();
    private readonly Stopwatch simulationClock = Stopwatch.StartNew();
    private readonly Random random;
    private readonly byte slaveAddress;
    private readonly ushort[] holdingRegisters;

    private readonly double currentQuadraticCoefficient;
    private readonly double currentLinearCoefficient;
    private readonly double currentConstantCoefficient;

    private double lastSimulationSeconds;
    private double currentFrequencyHz;
    private double targetFrequencyHz;
    private double simulatedCurrentA;
    private double currentNoiseA;
    private double instantaneousPowerKw;
    private double accumulatedEnergyKwh;
    private double? currentOverrideA;

    public sealed class PumpSnapshot
    {
        public double FrequencyHz { get; internal set; }

        public double TargetFrequencyHz { get; internal set; }

        public double CurrentA { get; internal set; }

        public double InstantaneousPowerKw { get; internal set; }

        public double AccumulatedEnergyKwh { get; internal set; }
    }

    // 静态构造函数，初始化串口
    static ModbusRtuSlave()
    {
        // 串口未打开时初始化
        serialPort = new SerialPort();
        serialPort.DataReceived += SerialPort_DataReceived; // 注册 DataReceived 事件
    }

    // 设置串口参数的接口，外部调用此方法设置串口
    public static void SetSerialPortSettings(string portName = "COM3", int baudRate = 9600, Parity parity = Parity.None, int dataBits = 8, StopBits stopBits = StopBits.One)
    {
        if (!isPortOpen)
        {
            serialPort.PortName = portName;
            serialPort.BaudRate = baudRate;
            serialPort.Parity = parity;
            serialPort.DataBits = dataBits;
            serialPort.StopBits = stopBits;
            serialPort.Open();
            isPortOpen = true;  // 标记串口已打开
        }
        else
        {
            Console.WriteLine("串口已经打开，无法更改设置。");
        }
    }

    // 构造函数，初始化每个从站
    public ModbusRtuSlave(byte slaveAddress)
    {
        this.slaveAddress = slaveAddress;
        holdingRegisters = new ushort[0x3010];
        random = new Random(Environment.TickCount ^ (slaveAddress << 16));

        switch (slaveAddress)
        {
            case 11:
                currentQuadraticCoefficient = 0.005063652281299397;
                currentLinearCoefficient = -0.05049377264325327;
                currentConstantCoefficient = 5.72586487089944;
                break;
            case 22:
                currentQuadraticCoefficient = 0.004814660602610448;
                currentLinearCoefficient = -0.030112118746925126;
                currentConstantCoefficient = 5.922344660623048;
                break;
            case 33:
                currentQuadraticCoefficient = 0.005129481141858832;
                currentLinearCoefficient = -0.047392561760144405;
                currentConstantCoefficient = 5.9362718966768675;
                break;
            case 44:
                currentQuadraticCoefficient = 0.004907373916841373;
                currentLinearCoefficient = -0.03910113450718124;
                currentConstantCoefficient = 6.053938877833055;
                break;
            default:
                currentQuadraticCoefficient = 0.004978792;
                currentLinearCoefficient = -0.041774;
                currentConstantCoefficient = 5.909355;
                break;
        }

        holdingRegisters[0x2100] = 1;
        lastSimulationSeconds = simulationClock.Elapsed.TotalSeconds;

        lock (slaveInstances)
        {
            slaveInstances[slaveAddress] = this;
        }
    }

    // 启动 Modbus RTU 从站（只启动一个串口线程）
    public static void Start()
    {
        // 启动 Modbus 处理线程（注意，这里没有使用 Thread.Sleep）
        Console.WriteLine("Modbus RTU Slave started...");
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
                AccumulatedEnergyKwh = accumulatedEnergyKwh
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

            UpdateOutputRegistersUnsafe();
        }
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

        double currentA =
            currentQuadraticCoefficient * frequencyHz * frequencyHz +
            currentLinearCoefficient * frequencyHz +
            currentConstantCoefficient;

        return Math.Max(0.0, Math.Min(30.0, currentA));
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

    // DataReceived 事件处理方法
    private static void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
    {
        try
        {
            // 检查串口是否已打开
            if (!serialPort.IsOpen)
            {
                Console.WriteLine("Serial port is not open.");
                return;
            }

            // 确保串口缓冲区有足够的数据
            int bytesToRead = serialPort.BytesToRead;
            if (bytesToRead < 8)
            {
                Console.WriteLine($"Not enough data available. Expected 8 bytes, got {bytesToRead} bytes.");
                return;
            }

            byte[] request = new byte[8];
            serialPort.Read(request, 0, 8);  // 从串口读取8字节数据

            // 打印接收到的字节数据
            Console.WriteLine("Received Data: " + BitConverter.ToString(request));

            byte slaveAddress = request[0];  // 获取请求中的从站地址

            ModbusRtuSlave slave;
            lock (slaveInstances)
            {
                slaveInstances.TryGetValue(slaveAddress, out slave);
            }

            if (slave != null)
            {
                byte functionCode = request[1];  // 功能码
                byte[] response = null;

                // 处理 Modbus 读保持寄存器请求（功能码 0x03）
                if (functionCode == 0x03)
                {
                    ushort startingAddress = (ushort)((request[2] << 8) + request[3]);
                    ushort quantity = (ushort)((request[4] << 8) + request[5]);

                    // 处理该从站的保持寄存器读取请求
                    response = slave.HandleReadHoldingRegisters(startingAddress, quantity);
                }
                // 处理 Modbus 写单个寄存器请求（功能码 0x06）
                else if (functionCode == 0x06)
                {
                    ushort registerAddress = (ushort)((request[2] << 8) + request[3]);
                    ushort registerValue = (ushort)((request[4] << 8) + request[5]);

                    // 处理写单个寄存器
                    response = slave.HandleWriteSingleRegister(registerAddress, registerValue);
                }

                if (response != null)
                {
                    serialPort.Write(response, 0, response.Length);
                    Console.WriteLine("Sent response: " + BitConverter.ToString(response));
                }
            }
        }
        catch (IOException ioEx)
        {
            Console.WriteLine("IOException: " + ioEx.Message);
        }
        catch (Exception ex)
        {
            Console.WriteLine("Exception: " + ex.Message);
        }
    }

    // 处理读保持寄存器（功能码 0x03）
    private byte[] HandleReadHoldingRegisters(ushort startingAddress, ushort quantity)
    {
        UpdateSimulation();

        lock (stateLock)
        {
            if (startingAddress >= holdingRegisters.Length || startingAddress + quantity > holdingRegisters.Length)
            {
                Console.WriteLine("Invalid register range.");
                return null;  // 无效的寄存器地址范围
            }

            byte[] response = new byte[5 + 2 * quantity]; // 响应长度：功能码 + 字节数 + 寄存器数据

            response[0] = slaveAddress;  // 从站地址
            response[1] = 0x03;  // 功能码（0x03：读取保持寄存器）
            response[2] = (byte)(2 * quantity); // 数据字节数（每个寄存器 2 字节）

            // 写入寄存器值
            for (int i = 0; i < quantity; i++)
            {
                ushort registerValue = holdingRegisters[startingAddress + (ushort)i];
                response[3 + 2 * i] = (byte)(registerValue >> 8);  // 高字节
                response[4 + 2 * i] = (byte)(registerValue & 0xFF);  // 低字节
            }

            // 校验（CRC 检查）
            byte[] crc = CalculateCRC(response.Take(response.Length - 2).ToArray()); // CRC 检查
            response[response.Length - 2] = crc[0];
            response[response.Length - 1] = crc[1];

            // 比较计算出的 CRC 和接收到的 CRC 是否一致
            if (!ValidateCRC(response))
            {
                Console.WriteLine("CRC mismatch!");
                return null;  // CRC 不匹配时返回 null
            }

            return response;
        }
    }

    // 处理写单个保持寄存器（功能码 0x06）
    private byte[] HandleWriteSingleRegister(ushort registerAddress, ushort registerValue)
    {
        if (registerAddress >= holdingRegisters.Length)
        {
            Console.WriteLine("Invalid register address.");
            return null;  // 无效的寄存器地址
        }

        byte[] response = new byte[8]; // 响应长度：从站地址 + 功能码 + 寄存器地址 + 寄存器值 + CRC

        response[0] = slaveAddress;  // 从站地址
        response[1] = 0x06;  // 功能码（0x06：写单个寄存器）
        response[2] = (byte)(registerAddress >> 8);  // 寄存器地址高字节
        response[3] = (byte)(registerAddress & 0xFF);  // 寄存器地址低字节
        Console.WriteLine(">>registerValue:" + registerValue.ToString());
        response[4] = (byte)(registerValue >> 8);  // 寄存器值高字节
        Console.WriteLine(">>response[4]:" + response[4].ToString());
        response[5] = (byte)(registerValue & 0xFF);  // 寄存器值低字节
        Console.WriteLine(">>response[5]:" + response[5].ToString());

        // 校验（CRC 检查）
        byte[] crc = CalculateCRC(response.Take(response.Length - 2).ToArray()); // CRC 检查
        response[response.Length - 2] = crc[0];
        response[response.Length - 1] = crc[1];

        // 比较计算出的 CRC 和接收到的 CRC 是否一致
        if (!ValidateCRC(response))
        {
            Console.WriteLine("CRC mismatch!");
            return null;  // CRC 不匹配时返回 null
        }

        SetHoldingRegister(registerAddress, registerValue);

        return response;
    }

    // CRC 校验方法（假设使用的是 Modbus CRC16）
    private byte[] CalculateCRC(byte[] data)
    {
        ushort crc = 0xFFFF;

        foreach (byte byteData in data)
        {
            crc ^= byteData;

            for (int i = 0; i < 8; i++)
            {
                if ((crc & 0x0001) != 0)
                {
                    crc >>= 1;
                    crc ^= 0xA001;
                }
                else
                {
                    crc >>= 1;
                }
            }
        }

        return new byte[] { (byte)(crc & 0xFF), (byte)((crc >> 8) & 0xFF) };
    }

    // 校验计算出来的 CRC 是否和响应中的 CRC 一致
    private bool ValidateCRC(byte[] response)
    {
        // 提取响应中存储的 CRC 值
        byte[] receivedCRC = new byte[] { response[response.Length - 2], response[response.Length - 1] };

        // 计算实际的 CRC 值
        byte[] calculatedCRC = CalculateCRC(response.Take(response.Length - 2).ToArray());

        // 比较计算出的 CRC 和响应中的 CRC
        return receivedCRC.SequenceEqual(calculatedCRC);
    }
}

public static class LogHelper
{
    // 创建全局静态的 Logger 实例
    private static readonly Logger logger = LogManager.GetCurrentClassLogger();

    public static Logger Logger => logger;
}
