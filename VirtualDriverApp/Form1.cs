using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Modbus;
using NLog;

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

        private static Random random = new Random();  // 随机数生成器
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
            pictureBox2.Visible = false;


            LogHelper.Logger.Info("Application started at " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"));
            //Console.WriteLine("CP")

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

            comboBox2.Items.Clear();  // 清除原有的项
            foreach (string port in portNames)
            {
                comboBox2.Items.Add(port);  // 添加串口名称到 ComboBox
            }

            // 如果有可用串口，默认选择第一个串口
            if (comboBox1.Items.Count > 0)
            {
                comboBox1.SelectedIndex = 0;
            }
            // 如果有可用串口，默认选择第一个串口
            if (comboBox2.Items.Count > 0)
            {
                comboBox2.SelectedIndex = 0;
            }

            // 为每个从站设置保持寄存器的初始值
            slave1.SetHoldingRegister(0, 0);  // 设置从站11的寄存器0初始值
            slave2.SetHoldingRegister(0, 0);  // 设置从站22的寄存器0初始值
            slave3.SetHoldingRegister(0, 0);  // 设置从站33的寄存器0初始值
            slave4.SetHoldingRegister(0, 0);  // 设置从站44的寄存器0初始值


        }

        private void button1_Click(object sender, EventArgs e)
        {
            string selectedPort = comboBox1.SelectedItem?.ToString();
            // 设置串口配置
            ModbusRtuSlave.SetSerialPortSettings(selectedPort, 9600, Parity.None, 8, StopBits.One);
            Thread.Sleep(200);
            // 启动共享的 Modbus 线程
            ModbusRtuSlave.Start();

            pictureBox2.Visible = true;

            button1.ForeColor = Color.Green;
            label10.ForeColor = Color.Green;
            label10.Text = "运行中";

            timer1.Enabled = true;
            Thread.Sleep(250);
            timer2.Enabled = true;
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
                        slave1.SetHoldingRegister(0x2100, 4);
                    }
                    else
                    {
                        slave1.SetHoldingRegister(0x3004, 30);
                        slave1.SetHoldingRegister(0x2100, 4);
                    }
                }
                else
                {
                    slave1.SetHoldingRegister(0x3004, rv);
                    slave1.SetHoldingRegister(0x2100, 1);
                }
            }
            else
            {
                slave1.SetHoldingRegister(0x2100, 3);
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
                    slave2.SetHoldingRegister(0x2100, 4);
                }
                else
                {
                    slave2.SetHoldingRegister(0x3004, rv);
                    slave2.SetHoldingRegister(0x2100, 1);
                }
            }
            else
            {
                slave2.SetHoldingRegister(0x2100, 3);
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
                    slave3.SetHoldingRegister(0x2100, 4);
                }
                else
                {
                    slave3.SetHoldingRegister(0x3004, rv);
                    slave3.SetHoldingRegister(0x2100, 1);
                }
            }
            else
            {
                slave3.SetHoldingRegister(0x2100, 3);
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
                    slave4.SetHoldingRegister(0x2100, 4);
                }
                else
                {
                    slave4.SetHoldingRegister(0x3004, rv);
                    slave4.SetHoldingRegister(0x2100, 1);
                }
            }
            else
            {
                slave4.SetHoldingRegister(0x2100, 3);
            }

            textBox1.Text = ((double)slave1.GetHoldingRegister(0x3000) / 100.0).ToString("F2") + " HZ";
            textBox2.Text = ((double)slave1.GetHoldingRegister(0x3004) / 100.0).ToString("F2") + " A";
            P1_Cur = (double)slave1.GetHoldingRegister(0x3004) / 100.0;


            textBox4.Text = ((double)slave2.GetHoldingRegister(0x3000) / 100.0).ToString("F2") + " HZ";
            textBox3.Text = ((double)slave2.GetHoldingRegister(0x3004) / 100.0).ToString("F2") + " A";
            N1_Cur = (double)slave2.GetHoldingRegister(0x3004) / 100.0;

            textBox8.Text = ((double)slave3.GetHoldingRegister(0x3000) / 100.0).ToString("F2") + " HZ";
            textBox7.Text = ((double)slave3.GetHoldingRegister(0x3004) / 100.0).ToString("F2") + " A";
            P2_Cur = (double)slave3.GetHoldingRegister(0x3004) / 100.0;

            textBox6.Text = ((double)slave4.GetHoldingRegister(0x3000) / 100.0).ToString("F2") + " HZ";
            textBox5.Text = ((double)slave4.GetHoldingRegister(0x3004) / 100.0).ToString("F2") + " A";
            N2_Cur = (double)slave4.GetHoldingRegister(0x3004) / 100.0;

            textBox22.Text = textBox22.Text + "P1 Current:" + textBox2.Text + "\r\n";
            textBox22.Text = textBox22.Text + "N1 Current:" + textBox3.Text + "\r\n";
            textBox22.Text = textBox22.Text + "P2 Current:" + textBox7.Text + "\r\n";
            textBox22.Text = textBox22.Text + "N2 Current:" + textBox5.Text + "\r\n";

            if (textBox22.Lines.Length > 40)
            {
                textBox22.Clear();
            }
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

        private async void timer2_Tick(object sender, EventArgs e)
        {
            double flow_max = 60.0; double flow_min = 0.0;
            double press_max = 0.28; double press_min = 0.0;

            // 设置从站地址、寄存器地址和要写入的值
            //string portName = "COM16";  // 根据实际情况修改串口号
            //string portName = comboBox2.SelectedItem?.ToString();
            //byte slaveAddress = 1;  // 从站地址
            //ushort registerAddress = 10;  // 寄存器地址

            double P1_PV_Show = ((P1_Cur / 15.0) * (press_max));
            double N1_PV_Show = ((N1_Cur / 15.0) * (press_max)) + PN1_PRESS_DIFF;
            double P2_PV_Show = ((P2_Cur / 15.0) * (press_max));
            double N2_PV_Show = ((N2_Cur / 15.0) * (press_max)) + PN2_PRESS_DIFF;

            double P1_FV_Show = ((P1_Cur / 15.0) * (flow_max));
            double N1_FV_Show = ((N1_Cur / 15.0) * (flow_max)) + PN1_FLOW_DIFF;
            double P2_FV_Show = ((P2_Cur / 15.0) * (flow_max));
            double N2_FV_Show = ((N2_Cur / 15.0) * (flow_max)) + PN2_FLOW_DIFF;

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

            double flow_sensor_max = 70.0;
            double press_sensor_max = 0.34;

            ushort P1_PV_SET = (ushort)((P1_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
            ushort P2_PV_SET = (ushort)((P2_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
            ushort N1_PV_SET = (ushort)((N1_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
            ushort N2_PV_SET = (ushort)((N2_PV_Show / press_sensor_max) * 16000.0 + 4000.0);

            ushort P1_FV_SET = (ushort)((P1_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
            ushort P2_FV_SET = (ushort)((P2_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
            ushort N1_FV_SET = (ushort)((N1_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
            ushort N2_FV_SET = (ushort)((N2_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);

            // 发送 Modbus 请求到一个新线程
            //await Task.Run(() =>
            //{
            //    SendModbusRequest(portName, 10, P1_PV_SET);
            //    SendModbusRequest(portName, 11, N1_PV_SET);
            //    SendModbusRequest(portName, 14, P2_PV_SET);
            //    SendModbusRequest(portName, 15, N2_PV_SET);
            //    SendModbusRequest(portName, 12, P1_FV_SET);
            //    SendModbusRequest(portName, 13, N1_FV_SET);
            //    SendModbusRequest(portName, 16, P2_FV_SET);
            //    SendModbusRequest(portName, 17, N2_FV_SET);
            //});
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





        //private void timer2_Tick(object sender, EventArgs e)
        //{
        //    double flow_max = 60.0; double flow_min = 0.0;
        //    double press_max = 0.28; double press_min = 0.0;

        //    // 设置从站地址、寄存器地址和要写入的值
        //    string portName = "COM16";  // 根据实际情况修改串口号
        //    byte slaveAddress = 1;  // 从站地址
        //    ushort registerAddress = 10;  // 寄存器地址
        //    //ushort value = 1234;  // 要写入的值

        //    double P1_PV_Show = ((P1_Cur / 15.0) * (press_max));
        //    //
        //    double N1_PV_Show = ((N1_Cur / 15.0) * (press_max));
        //    //
        //    double P2_PV_Show = ((P2_Cur / 15.0) * (press_max));
        //    //
        //    double N2_PV_Show = ((N2_Cur / 15.0) * (press_max));

        //    double P1_FV_Show = ((P1_Cur / 15.0) * (flow_max));
        //    //
        //    double N1_FV_Show = ((N1_Cur / 15.0) * (flow_max));
        //    //
        //    double P2_FV_Show = ((P2_Cur / 15.0) * (flow_max));
        //    //
        //    double N2_FV_Show = ((N2_Cur / 15.0) * (flow_max));

        //    textBox10.Text = P1_PV_Show.ToString("F3") + "Mpa";
        //    textBox9.Text = N1_PV_Show.ToString("F3") + "Mpa";

        //    textBox14.Text = P1_FV_Show.ToString("F2") + "m³/h";
        //    textBox13.Text = N1_FV_Show.ToString("F2") + "m³/h";

        //    textBox21.Text = P2_PV_Show.ToString("F3") + "Mpa";
        //    textBox19.Text = N2_PV_Show.ToString("F3") + "Mpa";

        //    textBox16.Text = P2_FV_Show.ToString("F2") + "m³/h";
        //    textBox18.Text = N2_FV_Show.ToString("F2") + "m³/h";

        //    double flow_sensor_max = 70.0;
        //    double press_sensor_max = 0.34;

        //    ushort P1_PV_SET = (ushort)((P1_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
        //    ushort P2_PV_SET = (ushort)((P2_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
        //    ushort N1_PV_SET = (ushort)((N1_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
        //    ushort N2_PV_SET = (ushort)((N2_PV_Show / press_sensor_max) * 16000.0 + 4000.0);

        //    ushort P1_FV_SET = (ushort)((P1_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
        //    ushort P2_FV_SET = (ushort)((P2_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
        //    ushort N1_FV_SET = (ushort)((N1_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
        //    ushort N2_FV_SET = (ushort)((N2_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);


        //    // 构建 Modbus RTU 写单个寄存器请求帧
        //    byte[] requestFrame = ModbusRTUSender.BuildWriteSingleRegisterRequest(1, 10, P1_PV_SET);
        //    // 发送数据帧
        //    ModbusRTUSender.SendModbusFrame(portName, requestFrame);


        //    // 构建 Modbus RTU 写单个寄存器请求帧
        //    requestFrame = ModbusRTUSender.BuildWriteSingleRegisterRequest(1, 11, N1_PV_SET);
        //    // 发送数据帧
        //    ModbusRTUSender.SendModbusFrame(portName, requestFrame);

        //    // 构建 Modbus RTU 写单个寄存器请求帧
        //    requestFrame = ModbusRTUSender.BuildWriteSingleRegisterRequest(1, 14, P2_PV_SET);
        //    // 发送数据帧
        //    ModbusRTUSender.SendModbusFrame(portName, requestFrame);

        //    // 构建 Modbus RTU 写单个寄存器请求帧
        //    requestFrame = ModbusRTUSender.BuildWriteSingleRegisterRequest(1, 15, N2_PV_SET);
        //    // 发送数据帧
        //    ModbusRTUSender.SendModbusFrame(portName, requestFrame);


        //    requestFrame = ModbusRTUSender.BuildWriteSingleRegisterRequest(1, 12, P1_FV_SET);
        //    // 发送数据帧
        //    ModbusRTUSender.SendModbusFrame(portName, requestFrame);

        //    // 构建 Modbus RTU 写单个寄存器请求帧
        //    requestFrame = ModbusRTUSender.BuildWriteSingleRegisterRequest(1, 13, N1_FV_SET);
        //    // 发送数据帧
        //    ModbusRTUSender.SendModbusFrame(portName, requestFrame);

        //    // 构建 Modbus RTU 写单个寄存器请求帧
        //    requestFrame = ModbusRTUSender.BuildWriteSingleRegisterRequest(1, 16, P2_FV_SET);
        //    // 发送数据帧
        //    ModbusRTUSender.SendModbusFrame(portName, requestFrame);

        //    // 构建 Modbus RTU 写单个寄存器请求帧
        //    requestFrame = ModbusRTUSender.BuildWriteSingleRegisterRequest(1, 17, N2_FV_SET);
        //    // 发送数据帧
        //    ModbusRTUSender.SendModbusFrame(portName, requestFrame);
        //}
    }
}

public class ModbusRtuSlave
{
    private static bool isPortOpen = false;  // 是否已打开串口
    private static SerialPort serialPort;    // 串口实例
    private static Dictionary<byte, ModbusRtuSlave> slaveInstances = new Dictionary<byte, ModbusRtuSlave>();  // 存储所有从站实例

    private byte slaveAddress;
    private ushort[] holdingRegisters;

    private float currentFrequency = 0;  // 当前频率，单位Hz
    private const float maxFrequency = 6000.0f;  // 最大频率，单位Hz
    private const float maxCurrent = 1500.0f;  // 最大电流，单位A
    private float targetFrequency = 0;  // 目标频率，单位Hz

    private ushort final_current = 0;

    private string response_log = "";

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
        this.holdingRegisters = new ushort[0x3010];  // 默认有 100 个寄存器
        this.currentFrequency = 0;  // 初始化频率为 0Hz
        slaveInstances[slaveAddress] = this;
    }

    // 启动 Modbus RTU 从站（只启动一个串口线程）
    public static void Start()
    {
        // 启动 Modbus 处理线程（注意，这里没有使用 Thread.Sleep）
        Console.WriteLine("Modbus RTU Slave started...");
    }

    // 设置保持寄存器值（包括频率的目标值）
    public void SetHoldingRegister(ushort address, ushort value)
    {
        Console.WriteLine("Setting Addr:" + address.ToString());
        if (address == 0x2001)  // 设置频率值
        {
            Console.WriteLine("Setting Target Frequency:" + value.ToString());
            this.targetFrequency = value;  // 设置目标频率
            Task.Run(() => GradualFrequencyChange());  // 启动频率逐步变化的任务
        }
        else
        {
            // 设置其他寄存器值的逻辑
            holdingRegisters[address] = value;
        }
    }

    public ushort GetHoldingRegister(ushort address)
    {
        return holdingRegisters[address];
    }

    public string GetCurentLogString()
    {
        return response_log;
    }

    // 逐步变化频率（模拟真实场景）
    private void GradualFrequencyChange()
    {
        float startFrequency = currentFrequency;
        float changeDuration = 1.0f;

        //if (Math.Abs(targetFrequency - startFrequency) > 20 * 100)
        //{
        //    changeDuration = 3.5f;
        //}
        //else if (Math.Abs(targetFrequency - startFrequency) > 10 * 100 && Math.Abs(targetFrequency - startFrequency) < 20 * 100)
        //{
        //    changeDuration = 2.5f;
        //}
        //else if (Math.Abs(targetFrequency - startFrequency) > 5 * 100 && Math.Abs(targetFrequency - startFrequency) < 10 * 100)
        //{
        //    changeDuration = 1.5f;
        //}
        //else if (Math.Abs(targetFrequency - startFrequency) > 250 && Math.Abs(targetFrequency - startFrequency) < 500)
        //{
        //    changeDuration = 1.2f;
        //}
        //else
        //{
        //    changeDuration = 1.0f;
        //}

        float frequencyChangeRate = (targetFrequency - startFrequency) / changeDuration; // 每秒变化频率

        // 逐步变化频率
        for (float t = 0; t < changeDuration; t += 0.2f) // 每 0.1 秒变化一次
        {
            currentFrequency = startFrequency + frequencyChangeRate * t;
            Console.WriteLine($"Current Frequency: {currentFrequency:F2} Hz");

            // 在 Modbus 保持寄存器地址 0x3000 返回频率值（模拟返回）
            holdingRegisters[0x3000] = (ushort)currentFrequency;
            holdingRegisters[0x3004] = (ushort)this.GetCurrent();

            // 模拟每 0.1 秒的时间间隔
            Task.Delay(88).Wait();
        }

        // 确保最终频率与目标频率一致
        currentFrequency = targetFrequency;

        holdingRegisters[0x3000] = (ushort)currentFrequency;
        holdingRegisters[0x3004] = (ushort)this.GetCurrent();

        final_current = (ushort)this.GetCurrent();

        Console.WriteLine($"Final Frequency: {currentFrequency:F2} Hz");
        Console.WriteLine($"Final Current: {currentFrequency:F2} A");
    }

    // 获取当前频率
    public float GetFrequency()
    {
        return currentFrequency;
    }

    // 获取当前电流
    public float GetCurrent()
    {
        Random random = new Random();
        // 生成一个在 -50 到 50 之间的随机数
        int randomNumber = random.Next(1, 6);
        // 计算电流，最大频率 60Hz 对应最大电流 15A
        return (currentFrequency / maxFrequency) * maxCurrent + (float)(randomNumber);
    }

    public ushort GetFinalCurrent()
    {
        return final_current;
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

            if (slaveInstances.ContainsKey(slaveAddress))  // 如果字典中有该从站实例
            {
                var slave = slaveInstances[slaveAddress];  // 获取对应的从站实例
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
            this.response_log = "Read Holding Registers CRC mismatch!";
            return null;  // CRC 不匹配时返回 null
        }

        return response;
    }

    // 处理写单个保持寄存器（功能码 0x06）
    private byte[] HandleWriteSingleRegister(ushort registerAddress, ushort registerValue)
    {
        if (registerAddress >= holdingRegisters.Length)
        {
            Console.WriteLine("Invalid register address.");
            return null;  // 无效的寄存器地址
        }

        // 更新寄存器值
        holdingRegisters[registerAddress] = registerValue;

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
            this.response_log = "Write Single Register CRC mismatch!";
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
