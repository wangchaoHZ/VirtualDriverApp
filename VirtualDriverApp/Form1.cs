using FluentModbus;
using Modbus;
using NLog;
using System;
using System.Collections.Generic;
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
        public Form1()
        {
            InitializeComponent();

            // 设置窗体启动时自动居中
            this.StartPosition = FormStartPosition.CenterScreen;
        }

        private ModbusTcpClient AI01_ModbusClient;
        private byte AI01_ModbusClient_ID = 5;

        private ModbusTcpClient AI02_ModbusClient;
        private byte AI02_ModbusClient_ID = 6;

        private ModbusTcpClient DO_ModbusClient;
        private byte DO_ModbusClient_ID = 2;

        private ModbusTcpClient DI_ModbusClient;
        private byte DI_ModbusClient_ID = 3;

        private ModbusTcpClient M160T_AI_ModbusClient;
        private byte M106T_AI_ModbusClient_ID = 1;

        // 创建四个从站实例，分别为地址11、22、33、44
        ModbusRtuSlave slave1 = new ModbusRtuSlave(11);
        ModbusRtuSlave slave2 = new ModbusRtuSlave(22);
        ModbusRtuSlave slave3 = new ModbusRtuSlave(33);
        ModbusRtuSlave slave4 = new ModbusRtuSlave(44);

        private double CAL_RATIO;

        private double P1_Cur;
        private double N1_Cur;
        private double P2_Cur;
        private double N2_Cur;

        private double PN1_PRESS_DIFF;
        private double PN2_PRESS_DIFF;
        private double PN1_FLOW_DIFF;
        private double PN2_FLOW_DIFF;

        private double S1_AP_Cur;
        private int S1_H2;
        private double S2_AP_Cur;
        private int S2_H2;

        private ushort[] DI_InputRegisters = new ushort[32];

        private static Random random = new Random();  // 随机数生成器

        // 新增：用于串行化各 Modbus 客户端的访问，避免并发与潜在死锁
        private readonly SemaphoreSlim _ai01Lock = new SemaphoreSlim(1, 1);
        private readonly SemaphoreSlim _diLock = new SemaphoreSlim(1, 1);
        private readonly SemaphoreSlim _doLock = new SemaphoreSlim(1, 1);

        // 新增：防止 Timer 重入（定时器回调在 UI 线程，网络慢时容易叠加）
        private volatile bool _timer2Busy = false;
        private volatile bool _timer3Busy = false;

        public double CalculateCurrent(double frequency)
        {
            if (frequency < 0)
            {
                return 0.0;
            }

            if (frequency > 6000)
            {
                return 15.0;
            }

            // 根据频率和最大电流估算电流
            double current = 1500 * (frequency / 6000);

            // 加入随机数，范围是正负0.5
            double randomAdjustment = random.NextDouble() * 1.0 - 0.5; // 生成 -0.5 到 +0.5 之间的随机数
            current += randomAdjustment;

            // 保留一位小数
            return Math.Round(current, 1);
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            label10.ForeColor = Color.Black;
            label10.Text = "未启动";
            //textBox9.Clear();

            // LogHelper.Logger.Info("Application started at " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"));
            //Console.WriteLine("CP")

            PN1_PRESS_DIFF = 0.0;
            PN2_PRESS_DIFF = 0.0;
            PN1_FLOW_DIFF = 0.0;
            PN2_FLOW_DIFF = 0.0;

            CAL_RATIO = 0.82;

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
                //
                M160T_AI_ModbusClient = new ModbusTcpClient();

                string selectedPort = comboBox1.SelectedItem?.ToString();

                // 在后台线程执行所有可能阻塞的 I/O（网络连接、串口打开）
                await Task.Run(() =>
                {
                    // 依次连接"192.168.1.133", "192.168.1.134" "192.168.1.131", "192.168.1.132" 
                    //AI01_ModbusClient.Connect("192.168.1.133", ModbusEndianness.BigEndian);
                    //AI02_ModbusClient.Connect("192.168.1.134", ModbusEndianness.BigEndian);

                    DO_ModbusClient.Connect("192.168.1.131", ModbusEndianness.BigEndian);
                    DI_ModbusClient.Connect("192.168.1.132", ModbusEndianness.BigEndian);
                    // "192.168.1.100:1502"
                    M160T_AI_ModbusClient.Connect("192.168.1.196:5021", ModbusEndianness.BigEndian);
                    // 
                    // comboBox1为变频器端口选择窗
                    // 设置串口配置（打开串口）
                    ModbusRtuSlave.SetSerialPortSettings(selectedPort, 9600, Parity.None, 8, StopBits.One);

                    if (true)
                    {
                        // 稍作延时给串口/设备稳定（后台线程里，不阻塞 UI）
                        Thread.Sleep(250);
                        WriteRegisterWithRetry(DO_ModbusClient, DO_ModbusClient_ID, 4, 1);
                        Thread.Sleep(250);
                        WriteRegisterWithRetry(DO_ModbusClient, DO_ModbusClient_ID, 4, 1);
                        Thread.Sleep(250);
                        WriteRegisterWithRetry(DO_ModbusClient, DO_ModbusClient_ID, 4, 1);
                        Thread.Sleep(250);
                        WriteRegisterWithRetry(DO_ModbusClient, DO_ModbusClient_ID, 4, 1);
                    }

                    // 启动共享的 Modbus 线程
                    ModbusRtuSlave.Start();
                });

                //pictureBox2.Visible = true;

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
                // LogHelper.Logger.Error(ex, "启动连接失败");
                MessageBox.Show($"连接设备失败：{ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                button1.Enabled = true;
            }
        }

        private int slave1_last_randomv = 0;
        private int slave2_last_randomv = 0;
        private int slave3_last_randomv = 0;
        private int slave4_last_randomv = 0;
        private void timer1_Tick(object sender, EventArgs e)
        {
            //
            if (slave1.GetFinalCurrent() > 100)
            {
                int rdv = random.Next(0, 11);
                slave1_last_randomv = (slave1_last_randomv > 0) ? -Math.Sign(rdv) * rdv : Math.Sign(rdv) * rdv;
                ushort rv = (ushort)(slave1.GetFinalCurrent() + random.Next(0, 11));

                if (checkBox1.Checked || checkBox2.Checked)
                {
                    if (checkBox1.Checked)
                    {
                        slave1.SetHoldingRegister(0x3004, 3000);
                        //slave1.SetHoldingRegister(0x2100, 4);
                    }
                    else
                    {
                        slave1.SetHoldingRegister(0x3004, 30);
                        //slave1.SetHoldingRegister(0x2100, 4);
                    }
                }
                else
                {
                    slave1.SetHoldingRegister(0x3004, rv);
                    //slave1.SetHoldingRegister(0x2100, 1);
                }
            }
            else
            {
                slave1.SetHoldingRegister(0x2100, 3);
            }

            if (checkBox13.Checked)
            {
                // 泵故障
                slave1.SetHoldingRegister(0x2100, 4);
            }
            else
            {
                // 泵正常
                slave1.SetHoldingRegister(0x2100, 1);
            }

            if (slave2.GetFinalCurrent() > 100)
            {
                int rdv = random.Next(0, 11);
                slave2_last_randomv = (slave2_last_randomv > 0) ? -Math.Sign(rdv) * rdv : Math.Sign(rdv) * rdv;
                ushort rv = (ushort)(slave2.GetFinalCurrent() + random.Next(0, 11));

                if (checkBox4.Checked || checkBox3.Checked)
                {
                    if (checkBox4.Checked)
                    {
                        slave2.SetHoldingRegister(0x3004, 3000);

                    }
                    else
                    {
                        slave2.SetHoldingRegister(0x3004, 30);
                    }
                    //slave2.SetHoldingRegister(0x2100, 4);
                }
                else
                {
                    slave2.SetHoldingRegister(0x3004, rv);
                    //slave2.SetHoldingRegister(0x2100, 1);
                }
            }
            else
            {
                //slave2.SetHoldingRegister(0x2100, 3);
            }

            if (checkBox14.Checked)
            {
                // 泵故障
                slave2.SetHoldingRegister(0x2100, 4);
            }
            else
            {
                // 泵正常
                slave2.SetHoldingRegister(0x2100, 1);
            }

            if (slave3.GetFinalCurrent() > 100)
            {
                int rdv = random.Next(0, 11);
                slave3_last_randomv = (slave3_last_randomv > 0) ? -Math.Sign(rdv) * rdv : Math.Sign(rdv) * rdv;
                ushort rv = (ushort)(slave3.GetFinalCurrent() + random.Next(0, 11));

                if (checkBox6.Checked || checkBox5.Checked)
                {
                    if (checkBox6.Checked)
                    {
                        slave3.SetHoldingRegister(0x3004, 3000);
                    }
                    else
                    {
                        slave3.SetHoldingRegister(0x3004, 30);
                    }
                    //slave3.SetHoldingRegister(0x2100, 4);
                }
                else
                {
                    slave3.SetHoldingRegister(0x3004, rv);
                    //slave3.SetHoldingRegister(0x2100, 1);
                }
            }
            else
            {
                //slave3.SetHoldingRegister(0x2100, 3);
            }

            if (checkBox15.Checked)
            {
                // 泵故障
                slave3.SetHoldingRegister(0x2100, 4);
            }
            else
            {
                // 泵正常
                slave3.SetHoldingRegister(0x2100, 1);
            }

            if (slave4.GetFinalCurrent() > 100)
            {
                int rdv = random.Next(0, 11);
                slave4_last_randomv = (slave4_last_randomv > 0) ? -Math.Sign(rdv) * rdv : Math.Sign(rdv) * rdv;
                ushort rv = (ushort)(slave4.GetFinalCurrent() + random.Next(10, 16));


                if (checkBox8.Checked || checkBox7.Checked)
                {
                    if (checkBox8.Checked)
                    {
                        slave4.SetHoldingRegister(0x3004, 3000);
                    }
                    else
                    {
                        slave4.SetHoldingRegister(0x3004, 30);
                    }
                    //slave4.SetHoldingRegister(0x2100, 4);
                }
                else
                {
                    slave4.SetHoldingRegister(0x3004, rv);
                    //slave4.SetHoldingRegister(0x2100, 1);
                }
            }
            else
            {
                //slave4.SetHoldingRegister(0x2100, 3);
            }

            if (checkBox16.Checked)
            {
                // 泵故障
                slave4.SetHoldingRegister(0x2100, 4);
            }
            else
            {
                // 泵正常
                slave4.SetHoldingRegister(0x2100, 1);
            }

            textBox1.Text = ((double)slave1.GetHoldingRegister(0x3000) / 100.0).ToString("F2") + " HZ";
            textBox2.Text = ((double)slave1.GetHoldingRegister(0x3004) / 100.0).ToString("F2") + " A";
            P1_Cur = (double)slave1.GetHoldingRegister(0x3004) / 100.0;

            textBox4.Text = ((double)slave2.GetHoldingRegister(0x3000) / 100.0).ToString("F2") + " HZ";
            textBox3.Text = ((double)slave2.GetHoldingRegister(0x3004) / 100.0).ToString("F2") + " A";
            N1_Cur = (double)slave2.GetHoldingRegister(0x3004) / 100.0;

            S1_AP_Cur = (P1_Cur + N1_Cur) * 0.5 * 0.093;

            textBox8.Text = ((double)slave3.GetHoldingRegister(0x3000) / 100.0).ToString("F2") + " HZ";
            textBox7.Text = ((double)slave3.GetHoldingRegister(0x3004) / 100.0).ToString("F2") + " A";
            P2_Cur = (double)slave3.GetHoldingRegister(0x3004) / 100.0;

            textBox6.Text = ((double)slave4.GetHoldingRegister(0x3000) / 100.0).ToString("F2") + " HZ";
            textBox5.Text = ((double)slave4.GetHoldingRegister(0x3004) / 100.0).ToString("F2") + " A";
            N2_Cur = (double)slave4.GetHoldingRegister(0x3004) / 100.0;

            S2_AP_Cur = (P2_Cur + N2_Cur) * 0.5 * 0.091;

        }
        private void button2_Click(object sender, EventArgs e)
        {
            Application.Exit();
        }

        private void label9_Click(object sender, EventArgs e)
        {

        }

        private void textBox14_TextChanged(object sender, EventArgs e)
        {

        }

        // 注意：该 Tick 由 UI 线程调用。将“所有网络 I/O”移到后台，并加防重入。
        private async void timer2_Tick(object sender, EventArgs e)
        {
            if (_timer2Busy) return;
            _timer2Busy = true;
            try
            {
                double flow_max = 90.0; double flow_min = 0.0;
                //设置压力上限
                double press_max = 300.0; double press_min = 0.0;

                double Cal_Base = 15.0; // 基准值

                double P1_PV_Show = ((P1_Cur / Cal_Base) * (press_max));
                double N1_PV_Show = ((N1_Cur / Cal_Base) * (press_max)) + PN1_PRESS_DIFF;
                double P2_PV_Show = ((P2_Cur / Cal_Base) * (press_max));
                double N2_PV_Show = ((N2_Cur / Cal_Base) * (press_max)) + PN2_PRESS_DIFF;

                double P1_FV_Show = ((P1_Cur / Cal_Base) * (flow_max));
                double N1_FV_Show = ((N1_Cur / Cal_Base) * (flow_max)) + PN1_FLOW_DIFF;
                double P2_FV_Show = ((P2_Cur / Cal_Base) * (flow_max));
                double N2_FV_Show = ((N2_Cur / Cal_Base) * (flow_max)) + PN2_FLOW_DIFF;


                // 做转换
                P1_PV_Show = P1_PV_Show * CAL_RATIO;
                N1_PV_Show = N1_PV_Show * CAL_RATIO;
                P2_PV_Show = P2_PV_Show * CAL_RATIO;
                N2_PV_Show = N2_PV_Show * CAL_RATIO;

                P1_FV_Show = 404;
                N1_FV_Show = P1_FV_Show;
                P2_FV_Show = P1_FV_Show;
                N2_FV_Show = P1_FV_Show;

                textBox10.Text = P1_PV_Show.ToString("F1") + "kpa";
                textBox9.Text = N1_PV_Show.ToString("F1") + "kpa";

                // LogHelper.Logger.Info("---------------------------------------------");
                // LogHelper.Logger.Info("P1_PV_Show:" + textBox10.Text + " N1_PV_Show:" + textBox9.Text);

                textBox14.Text = P1_FV_Show.ToString("F2") + "m³/h";
                textBox13.Text = N1_FV_Show.ToString("F2") + "m³/h";

                //// LogHelper.Logger.Info("P1_FV_Show:" + textBox14.Text + " N1_FV_Show:" + textBox13.Text);

                textBox21.Text = P2_PV_Show.ToString("F1") + "kpa";
                textBox19.Text = N2_PV_Show.ToString("F1") + "kpa";

                //// LogHelper.Logger.Info("P2_PV_Show:" + textBox21.Text + " N2_PV_Show:" + textBox19.Text);

                textBox16.Text = P2_FV_Show.ToString("F2") + "m³/h";
                textBox18.Text = N2_FV_Show.ToString("F2") + "m³/h";

                // LogHelper.Logger.Info("P2_FV_Show:" + textBox16.Text + " N2_FV_Show:" + textBox18.Text);
                // LogHelper.Logger.Info("---------------------------------------------");

                // 放大10倍传输出去
                ushort P1_PV_GIVEN = (ushort)(P1_PV_Show * 10);
                ushort N1_PV_GIVEN = (ushort)(N1_PV_Show * 10);
                ushort P2_PV_GIVEN = (ushort)(P2_PV_Show * 10);
                ushort N2_PV_GIVEN = (ushort)(N2_PV_Show * 10);
                //

                double air_pump_sensor_max = 3.5;
                double flow_sensor_max = 90.0;
                double press_sensor_max = 300.0;
#if false
                ushort P1_PV_SET = (ushort)((P1_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
                ushort P2_PV_SET = (ushort)((P2_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
                ushort N1_PV_SET = (ushort)((N1_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
                ushort N2_PV_SET = (ushort)((N2_PV_Show / press_sensor_max) * 16000.0 + 4000.0);

                ushort P1_FV_SET = (ushort)((P1_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
                ushort P2_FV_SET = (ushort)((P2_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
                ushort N1_FV_SET = (ushort)((N1_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
                ushort N2_FV_SET = (ushort)((N2_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
#endif
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
                    // LogHelper.Logger.Error(ex, "读取 DI 输入寄存器失败");
                }

                if (diWordArray != null && diWordArray.Length >= 64)
                {
                    for (int i = 0; i < 32; i++)
                    {
                        DI_InputRegisters[i] = (ushort)((diWordArray[i * 2] << 8) | diWordArray[i * 2 + 1]);
                    }
                }

                if (DI_InputRegisters[7] == 1)
                {
                    label24.Text = "PCS连锁连接";
                    label24.ForeColor = Color.Lime;
                }
                else
                {
                    label24.Text = "PCS连锁断开";
                    label24.ForeColor = Color.Red;
                }

                if (DI_InputRegisters[6] == 1)
                {
                    label25.Text = "BAU连锁连接";
                    label25.ForeColor = Color.Lime;
                }
                else
                {
                    label25.Text = "BAU连锁断开";
                    label25.ForeColor = Color.Red;
                }

                if (DI_InputRegisters[0] == 1)
                {
                    label26.Text = "A容量箱风机开";
                    label26.ForeColor = Color.Lime;
                }
                else
                {
                    label26.Text = "A容量箱风机关";
                    label26.ForeColor = Color.Red;
                }

                if (DI_InputRegisters[1] == 1)
                {
                    label27.Text = "B容量箱风机开";
                    label27.ForeColor = Color.Lime;
                }
                else
                {
                    label27.Text = "B容量箱风机关";
                    label27.ForeColor = Color.Red;
                }


                if (DI_InputRegisters[2] == 1)
                {
                    label28.Text = "A侧轴流风机1开";
                    label28.ForeColor = Color.Lime;
                }
                else
                {
                    label28.Text = "A侧轴流风机1关闭";
                    label28.ForeColor = Color.Red;
                }

                if (DI_InputRegisters[3] == 1)
                {
                    label29.Text = "A侧轴流风机2开";
                    label29.ForeColor = Color.Lime;
                }
                else
                {
                    label29.Text = "A侧轴流风机2关闭";
                    label29.ForeColor = Color.Red;
                }

                if (DI_InputRegisters[4] == 1)
                {
                    label30.Text = "B侧轴流风机1开";
                    label30.ForeColor = Color.Lime;
                }
                else
                {
                    label30.Text = "B侧轴流风机1关闭";
                    label30.ForeColor = Color.Red;
                }

                if (DI_InputRegisters[5] == 1)
                {
                    label31.Text = "B侧轴流风机2开";
                    label31.ForeColor = Color.Lime;
                }
                else
                {
                    label31.Text = "B侧轴流风机2关闭";
                    label31.ForeColor = Color.Red;
                }


                // 发送 Modbus 请求到后台线程（只写操作放后台）
                ushort vstartAddress = 100;

                ushort[] give_m160t_values =
                {
                    P1_PV_GIVEN,
                    N1_PV_GIVEN,
                    P2_PV_GIVEN,
                    N2_PV_GIVEN,
                };

                try
                {
                    await Task.Run(async () =>
                    {
                        await _ai01Lock.WaitAsync();
                        try
                        {
                            M160T_AI_ModbusClient.WriteMultipleRegisters(M106T_AI_ModbusClient_ID, vstartAddress, give_m160t_values);
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
                // LogHelper.Logger.Error(exOuter, "timer2_Tick 执行异常");
            }
            finally
            {
                _timer2Busy = false;
            }
        }

        // 封装发送 Modbus 请求的代码
        private void SendModbusRequest(string portName, ushort registerAddress, ushort value)
        {
            // 构建 Modbus RTU 写单个寄存器请求帧
            byte[] requestFrame = ModbusRTUSender.BuildWriteSingleRegisterRequest(0x01, registerAddress, value);

            // 发送数据帧
            ModbusRTUSender.SendModbusFrame(portName, requestFrame);
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

        private void checkBo11_CheckedChanged(object sender, EventArgs e)
        {
            PN2_PRESS_DIFF = -PN2_PRESS_DIFF;
        }

        private void checkBox12_CheckedChanged(object sender, EventArgs e)
        {
            PN2_FLOW_DIFF = -PN2_FLOW_DIFF;
        }

        private void button5_Click(object sender, EventArgs e)
        {
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
                // LogHelper.Logger.Error(exOuter, "timer3_Tick 执行异常");
            }
            finally
            {
                _timer3Busy = false;
            }
        }

        private void trackBar4_Scroll(object sender, EventArgs e)
        {
        }

        private void trackBar1_Scroll(object sender, EventArgs e)
        {
        }

        private void trackBar1_Scroll_1(object sender, EventArgs e)
        {
        }

        private void button3_Click(object sender, EventArgs e)
        {
        }

        private void button6_Click(object sender, EventArgs e)
        {
        }

        int MAIN_DOOR = 0;
        int POWER_DOOR = 0;

        private void button4_Click(object sender, EventArgs e)
        {
        }

        private void button7_Click(object sender, EventArgs e)
        {
        }

        private void pictureBox2_Click(object sender, EventArgs e)
        {

        }

        private void textBox22_TextChanged(object sender, EventArgs e)
        {
            double value;

            if (double.TryParse(textBox22.Text, out value))
            {
                CAL_RATIO = value;
            }
            else
            {
                CAL_RATIO = 0.9;
            }
        }
    }
}

public class ModbusRtuSlave
{
    private static bool isPortOpen = false;                 // 是否已打开串口
    private static readonly object serialLock = new object(); // 串口读写锁，避免并发读写串口
    private static readonly object rxLock = new object();     // 接收缓存锁
    private static SerialPort serialPort;                   // 串口实例
    private static readonly Dictionary<byte, ModbusRtuSlave> slaveInstances = new Dictionary<byte, ModbusRtuSlave>();
    private static readonly List<byte> rxBuffer = new List<byte>(256); // RTU接收缓存，用于解决半包/粘包/错位
    private const int FixedRequestFrameLength = 8;           // 当前只支持03/06，主站请求固定8字节
    private const int MaxRxBufferLength = 512;               // 防止异常数据无限堆积

    private static readonly object randomLock = new object();
    private static readonly Random sharedRandom = new Random();

    private readonly object dataLock = new object();         // 寄存器/频率/电流数据锁
    private readonly byte slaveAddress;
    private readonly ushort[] holdingRegisters;

    private float currentFrequency = 0;                      // 当前频率，原程序按寄存器值保存：6000代表60.00Hz
    private const float maxFrequency = 6000.0f;              // 最大频率寄存器值
    private const float maxCurrent = 1500.0f;                // 最大电流寄存器值，1500代表15.00A
    private float targetFrequency = 0;                       // 目标频率寄存器值
    private bool frequencyTaskRunning = false;               // 每个泵只允许一个频率渐变任务运行

    private ushort final_current = 0;
    private string response_log = "";

    // 静态构造函数，初始化串口
    static ModbusRtuSlave()
    {
        serialPort = new SerialPort();
        serialPort.DataReceived += SerialPort_DataReceived;
    }

    // 设置串口参数的接口，外部调用此方法设置串口
    public static void SetSerialPortSettings(string portName = "COM3", int baudRate = 9600, Parity parity = Parity.None, int dataBits = 8, StopBits stopBits = StopBits.One)
    {
        lock (serialLock)
        {
            if (!isPortOpen)
            {
                if (string.IsNullOrWhiteSpace(portName))
                {
                    throw new ArgumentException("串口名称为空，请先选择有效串口。", nameof(portName));
                }

                serialPort.PortName = portName;
                serialPort.BaudRate = baudRate;
                serialPort.Parity = parity;
                serialPort.DataBits = dataBits;
                serialPort.StopBits = stopBits;
                serialPort.ReadTimeout = 200;
                serialPort.WriteTimeout = 200;
                serialPort.ReceivedBytesThreshold = 1;
                serialPort.ReadBufferSize = 4096;
                serialPort.WriteBufferSize = 4096;

                serialPort.Open();
                serialPort.DiscardInBuffer();
                serialPort.DiscardOutBuffer();

                lock (rxLock)
                {
                    rxBuffer.Clear();
                }

                isPortOpen = true;
            }
            else
            {
                Console.WriteLine("串口已经打开，无法更改设置。");
            }
        }
    }

    // 构造函数，初始化每个从站
    public ModbusRtuSlave(byte slaveAddress)
    {
        this.slaveAddress = slaveAddress;
        this.holdingRegisters = new ushort[0x3010];
        this.currentFrequency = 0;

        lock (slaveInstances)
        {
            slaveInstances[slaveAddress] = this;
        }
    }

    // 启动 Modbus RTU 从站
    public static void Start()
    {
        Console.WriteLine("Modbus RTU Slave started...");
    }

    // 设置保持寄存器值（包括频率目标值）
    public void SetHoldingRegister(ushort address, ushort value)
    {
        bool needStartFrequencyTask = false;

        lock (dataLock)
        {
            if (address >= holdingRegisters.Length)
            {
                Console.WriteLine($"SetHoldingRegister invalid address: 0x{address:X4}");
                return;
            }

            holdingRegisters[address] = value;

            if (address == 0x2001) // 设置频率值
            {
                targetFrequency = value;

                if (!frequencyTaskRunning)
                {
                    frequencyTaskRunning = true;
                    needStartFrequencyTask = true;
                }
            }
        }

        if (address == 0x2001)
        {
            Console.WriteLine("Setting Target Frequency:" + value.ToString());
        }

        if (needStartFrequencyTask)
        {
            Task.Run(() => FrequencyWorkerLoop());
        }
    }

    public ushort GetHoldingRegister(ushort address)
    {
        lock (dataLock)
        {
            if (address >= holdingRegisters.Length)
            {
                return 0;
            }

            return holdingRegisters[address];
        }
    }

    public string GetCurentLogString()
    {
        lock (dataLock)
        {
            return response_log;
        }
    }

    // 单任务频率渐变：主站频繁写0x2001时，只更新targetFrequency，不重复创建任务
    private async Task FrequencyWorkerLoop()
    {
        try
        {
            while (true)
            {
                float cur;
                float target;

                lock (dataLock)
                {
                    cur = currentFrequency;
                    target = targetFrequency;
                }

                float diff = target - cur;

                if (Math.Abs(diff) <= 1.0f)
                {
                    lock (dataLock)
                    {
                        currentFrequency = targetFrequency;
                        UpdateFrequencyAndCurrentRegistersLocked();

                        // 如果锁内确认已经到达最新目标，则结束任务
                        if (Math.Abs(targetFrequency - currentFrequency) <= 1.0f)
                        {
                            frequencyTaskRunning = false;
                            return;
                        }
                    }
                }
                else
                {
                    // 每100ms向目标靠近一部分，避免一次跳变；同时不会因为频繁写目标而启动多个任务
                    float step = diff * 0.25f;

                    // 防止极小步进导致长时间不收敛
                    if (Math.Abs(step) < 1.0f)
                    {
                        step = Math.Sign(diff) * 1.0f;
                    }

                    lock (dataLock)
                    {
                        currentFrequency += step;

                        if (currentFrequency < 0)
                        {
                            currentFrequency = 0;
                        }
                        else if (currentFrequency > maxFrequency)
                        {
                            currentFrequency = maxFrequency;
                        }

                        UpdateFrequencyAndCurrentRegistersLocked();
                    }
                }

                await Task.Delay(100);
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine("FrequencyWorkerLoop exception: " + ex.Message);
            lock (dataLock)
            {
                frequencyTaskRunning = false;
            }
        }
    }

    private void UpdateFrequencyAndCurrentRegistersLocked()
    {
        ushort freq = (ushort)Math.Max(0, Math.Min(maxFrequency, currentFrequency));
        ushort cur = (ushort)Math.Max(0, Math.Min(ushort.MaxValue, GetCurrentLocked()));

        holdingRegisters[0x3000] = freq;
        holdingRegisters[0x3004] = cur;
        final_current = cur;
    }

    // 获取当前频率
    public float GetFrequency()
    {
        lock (dataLock)
        {
            return currentFrequency;
        }
    }

    // 获取当前电流
    public float GetCurrent()
    {
        lock (dataLock)
        {
            return GetCurrentLocked();
        }
    }

    private float GetCurrentLocked()
    {
        int randomNumber;
        lock (randomLock)
        {
            randomNumber = sharedRandom.Next(1, 6);
        }

        return (currentFrequency / maxFrequency) * maxCurrent + randomNumber;
    }

    public ushort GetFinalCurrent()
    {
        lock (dataLock)
        {
            return final_current;
        }
    }

    // DataReceived事件：只负责读取当前可用字节，放入缓存，再从缓存中解析完整RTU帧
    private static void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
    {
        try
        {
            if (serialPort == null || !serialPort.IsOpen)
            {
                Console.WriteLine("Serial port is not open.");
                return;
            }

            byte[] incoming;

            lock (serialLock)
            {
                int bytesToRead = serialPort.BytesToRead;
                if (bytesToRead <= 0)
                {
                    return;
                }

                incoming = new byte[bytesToRead];
                int readLen = serialPort.Read(incoming, 0, incoming.Length);

                if (readLen <= 0)
                {
                    return;
                }

                if (readLen != incoming.Length)
                {
                    Array.Resize(ref incoming, readLen);
                }
            }

            List<byte[]> framesToProcess;

            lock (rxLock)
            {
                rxBuffer.AddRange(incoming);

                if (rxBuffer.Count > MaxRxBufferLength)
                {
                    // 异常情况下防止缓存无限增长，保留最后7字节用于半包续接
                    int keep = Math.Min(FixedRequestFrameLength - 1, rxBuffer.Count);
                    byte[] tail = rxBuffer.Skip(rxBuffer.Count - keep).ToArray();
                    rxBuffer.Clear();
                    rxBuffer.AddRange(tail);
                    Console.WriteLine("RTU rxBuffer overflow, buffer trimmed.");
                }

                framesToProcess = ExtractCompleteFramesLocked();
            }

            foreach (byte[] request in framesToProcess)
            {
                ProcessRequestFrame(request);
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

    // 从接收缓存中提取完整03/06请求帧，解决半包、粘包、错位问题
    private static List<byte[]> ExtractCompleteFramesLocked()
    {
        List<byte[]> frames = new List<byte[]>();

        while (rxBuffer.Count >= FixedRequestFrameLength)
        {
            byte[] first8 = rxBuffer.Take(FixedRequestFrameLength).ToArray();

            if (IsSupportedFixedRequestFrame(first8))
            {
                frames.Add(first8);
                rxBuffer.RemoveRange(0, FixedRequestFrameLength);
                continue;
            }

            // 当前头不对，尝试在缓存中寻找下一个合法帧头，避免因为残留字节导致永久错位
            int nextStart = FindNextSupportedFrameStartLocked();

            if (nextStart > 0)
            {
                rxBuffer.RemoveRange(0, nextStart);
                continue;
            }

            if (nextStart == 0)
            {
                // 理论上不会走到这里，因为first8前面已经判断过
                continue;
            }

            // 当前缓存里找不到完整合法帧。保留最后7字节，等待后续字节拼成完整帧。
            if (rxBuffer.Count > FixedRequestFrameLength - 1)
            {
                int keep = FixedRequestFrameLength - 1;
                byte[] tail = rxBuffer.Skip(rxBuffer.Count - keep).ToArray();
                rxBuffer.Clear();
                rxBuffer.AddRange(tail);
            }

            break;
        }

        return frames;
    }

    private static int FindNextSupportedFrameStartLocked()
    {
        for (int i = 1; i <= rxBuffer.Count - FixedRequestFrameLength; i++)
        {
            byte[] candidate = rxBuffer.Skip(i).Take(FixedRequestFrameLength).ToArray();
            if (IsSupportedFixedRequestFrame(candidate))
            {
                return i;
            }
        }

        return -1;
    }

    private static bool IsSupportedFixedRequestFrame(byte[] frame)
    {
        if (frame == null || frame.Length != FixedRequestFrameLength)
        {
            return false;
        }

        byte functionCode = frame[1];

        // 本程序按你的要求只支持03和06，不增加0x10
        if (functionCode != 0x03 && functionCode != 0x06)
        {
            return false;
        }

        return ValidateCRC(frame);
    }

    private static void ProcessRequestFrame(byte[] request)
    {
        Console.WriteLine("Received Data: " + BitConverter.ToString(request));

        if (!ValidateCRC(request))
        {
            Console.WriteLine("Request CRC mismatch, frame dropped.");
            return;
        }

        byte addr = request[0];
        byte functionCode = request[1];

        ModbusRtuSlave slave;
        lock (slaveInstances)
        {
            if (!slaveInstances.TryGetValue(addr, out slave))
            {
                // 不是本模拟器地址，不响应
                Console.WriteLine($"Ignore frame for slave address: {addr}");
                return;
            }
        }

        byte[] response = null;

        if (functionCode == 0x03)
        {
            ushort startingAddress = (ushort)((request[2] << 8) | request[3]);
            ushort quantity = (ushort)((request[4] << 8) | request[5]);
            response = slave.HandleReadHoldingRegisters(startingAddress, quantity);
        }
        else if (functionCode == 0x06)
        {
            ushort registerAddress = (ushort)((request[2] << 8) | request[3]);
            ushort registerValue = (ushort)((request[4] << 8) | request[5]);
            response = slave.HandleWriteSingleRegister(registerAddress, registerValue);
        }

        if (response != null)
        {
            lock (serialLock)
            {
                if (serialPort != null && serialPort.IsOpen)
                {
                    serialPort.Write(response, 0, response.Length);
                }
            }

            Console.WriteLine("Sent response: " + BitConverter.ToString(response));
        }
    }

    // 处理读保持寄存器（功能码0x03）
    private byte[] HandleReadHoldingRegisters(ushort startingAddress, ushort quantity)
    {
        if (quantity == 0 || quantity > 125)
        {
            Console.WriteLine("Invalid register quantity.");
            SetResponseLog("Read Holding Registers invalid quantity!");
            return BuildExceptionResponse(0x03, 0x03); // Illegal Data Value
        }

        int start = startingAddress;
        int end = start + quantity;

        if (start < 0 || end > holdingRegisters.Length)
        {
            Console.WriteLine("Invalid register range.");
            SetResponseLog("Read Holding Registers invalid range!");
            return BuildExceptionResponse(0x03, 0x02); // Illegal Data Address
        }

        byte[] response = new byte[5 + 2 * quantity];

        response[0] = slaveAddress;
        response[1] = 0x03;
        response[2] = (byte)(2 * quantity);

        lock (dataLock)
        {
            for (int i = 0; i < quantity; i++)
            {
                ushort registerValue = holdingRegisters[startingAddress + i];
                response[3 + 2 * i] = (byte)(registerValue >> 8);
                response[4 + 2 * i] = (byte)(registerValue & 0xFF);
            }
        }

        AppendCRC(response);
        return response;
    }

    // 处理写单个保持寄存器（功能码0x06）
    private byte[] HandleWriteSingleRegister(ushort registerAddress, ushort registerValue)
    {
        if (registerAddress >= holdingRegisters.Length)
        {
            Console.WriteLine("Invalid register address.");
            SetResponseLog("Write Single Register invalid address!");
            return BuildExceptionResponse(0x06, 0x02); // Illegal Data Address
        }

        SetHoldingRegister(registerAddress, registerValue);

        byte[] response = new byte[8];

        response[0] = slaveAddress;
        response[1] = 0x06;
        response[2] = (byte)(registerAddress >> 8);
        response[3] = (byte)(registerAddress & 0xFF);
        response[4] = (byte)(registerValue >> 8);
        response[5] = (byte)(registerValue & 0xFF);

        AppendCRC(response);
        return response;
    }

    private byte[] BuildExceptionResponse(byte functionCode, byte exceptionCode)
    {
        byte[] response = new byte[5];
        response[0] = slaveAddress;
        response[1] = (byte)(functionCode | 0x80);
        response[2] = exceptionCode;
        AppendCRC(response);
        return response;
    }

    private void SetResponseLog(string log)
    {
        lock (dataLock)
        {
            response_log = log;
        }
    }

    private static void AppendCRC(byte[] frame)
    {
        byte[] crc = CalculateCRC(frame.Take(frame.Length - 2).ToArray());
        frame[frame.Length - 2] = crc[0];
        frame[frame.Length - 1] = crc[1];
    }

    // CRC校验方法，Modbus CRC16，返回低字节在前
    private static byte[] CalculateCRC(byte[] data)
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

    // 校验整帧CRC，frame最后2字节应为CRC低字节、高字节
    private static bool ValidateCRC(byte[] frame)
    {
        if (frame == null || frame.Length < 4)
        {
            return false;
        }

        byte[] receivedCRC = new byte[] { frame[frame.Length - 2], frame[frame.Length - 1] };
        byte[] calculatedCRC = CalculateCRC(frame.Take(frame.Length - 2).ToArray());
        return receivedCRC.SequenceEqual(calculatedCRC);
    }
}


namespace Modbus
{
    public class ModbusRTUSender
    {
        // 构建 Modbus RTU 写多个寄存器的请求帧（功能码 0x10）
        public static byte[] BuildWriteMultipleRegistersRequest(byte slaveAddress, ushort startAddress, ushort[] data)
        {
            byte[] frame = new byte[5 + data.Length * 2 + 2];

            frame[0] = slaveAddress; // 从站地址
            frame[1] = 0x10; // 功能码：16 (Write Multiple Registers)
            frame[2] = (byte)(startAddress >> 8); // 起始地址高字节
            frame[3] = (byte)(startAddress & 0xFF); // 起始地址低字节
            frame[4] = (byte)(data.Length >> 8); // 寄存器数量高字节
            frame[5] = (byte)(data.Length & 0xFF); // 寄存器数量低字节
            frame[6] = (byte)(data.Length * 2); // 字节数量，2个字节表示1个寄存器

            int index = 7;
            foreach (var value in data)
            {
                frame[index++] = (byte)(value >> 8); // 数据高字节
                frame[index++] = (byte)(value & 0xFF); // 数据低字节
            }

            // 计算CRC
            ushort crc = CalculateCRC(frame, frame.Length - 2);
            frame[frame.Length - 2] = (byte)(crc & 0xFF); // CRC低字节
            frame[frame.Length - 1] = (byte)(crc >> 8); // CRC高字节

            return frame;
        }

        // 构建 Modbus RTU 写单个寄存器的请求帧（功能码 0x06）
        public static byte[] BuildWriteSingleRegisterRequest(byte slaveAddress, ushort registerAddress, ushort value)
        {
            byte[] frame = new byte[8];

            frame[0] = slaveAddress; // 从站地址
            frame[1] = 0x06; // 功能码：6 (Write Single Register)
            frame[2] = (byte)(registerAddress >> 8); // 寄存器地址高字节
            frame[3] = (byte)(registerAddress & 0xFF); // 寄存器地址低字节
            frame[4] = (byte)(value >> 8); // 数据高字节
            frame[5] = (byte)(value & 0xFF); // 数据低字节

            // 计算CRC
            ushort crc = CalculateCRC(frame, frame.Length - 2);
            frame[frame.Length - 2] = (byte)(crc & 0xFF); // CRC低字节
            frame[frame.Length - 1] = (byte)(crc >> 8); // CRC高字节

            return frame;
        }

        // CRC16计算函数
        public static ushort CalculateCRC(byte[] data, int length)
        {
            ushort crc = 0xFFFF;
            for (int i = 0; i < length; i++)
            {
                crc ^= data[i];
                for (int j = 8; j > 0; j--)
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
            return crc;
        }

        // 串口参数设置
        public static SerialPort SetupSerialPort(string portName)
        {
            SerialPort serialPort = new SerialPort
            {
                PortName = portName,        // 设置串口号
                BaudRate = 9600,            // 设置波特率
                Parity = Parity.None,       // 设置奇偶校验
                DataBits = 8,               // 设置数据位
                StopBits = StopBits.One     // 设置停止位
            };

            return serialPort;
        }

        // 使用串口发送Modbus RTU帧
        public static void SendModbusFrame(string portName, byte[] frame)
        {
            try
            {
                // 创建串口对象并设置参数
                using (SerialPort serialPort = SetupSerialPort(portName))
                {
                    serialPort.Open(); // 打开串口

                    // 发送数据
                    serialPort.Write(frame, 0, frame.Length);
                    Console.WriteLine("Modbus RTU 帧已发送：");
                    foreach (var b in frame)
                    {
                        Console.Write($"{b:X2} ");
                    }
                    Console.WriteLine();

                    serialPort.Close(); // 关闭串口
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine("串口通信出错: " + ex.Message);
            }
        }
    }
}
public static class LogHelper
{
    // 创建全局静态的 Logger 实例
    private static readonly Logger logger = LogManager.GetCurrentClassLogger();

    public static Logger Logger => logger;
}
