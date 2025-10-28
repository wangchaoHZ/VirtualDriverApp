using FluentModbus;
using Modbus;
using Newtonsoft.Json;
using Serilog;
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
        private ModbusTcpClient AI01_ModbusClient;
        private ModbusTcpClient AI02_ModbusClient;
        private ModbusTcpClient DO01_ModbusClient;
        private ModbusTcpClient DO02_ModbusClient;
        private ModbusTcpClient DI_ModbusClient;

        private string AI01_ModuleIP = string.Empty;
        private string AI02_ModuleIP = string.Empty;
        private string DO01_ModuleIP = string.Empty;
        private string DO02_ModuleIP = string.Empty;
        private string DI_ModuleIP = string.Empty;

        private string VFD_COM_PORT = string.Empty;
        private string VBT_COM_PORT = string.Empty;

        public Form1()
        {
            InitializeComponent();
            // 设置窗体启动时自动居中
            this.StartPosition = FormStartPosition.CenterScreen;
        }

        public class ConfigLoader
        {
            public static AppConfig LoadConfig(string filePath)
            {
                if (!File.Exists(filePath))
                    throw new FileNotFoundException("配置文件未找到", filePath);

                string json = File.ReadAllText(filePath);

                Console.WriteLine(json);  // 打印原始JSON，确认实际加载的内容

                return JsonConvert.DeserializeObject<AppConfig>(json);
            }
        }

        //创建四个变频器从站实例，分别为地址11、22、33、44
        private readonly ModbusRtuSlave slave1 = new ModbusRtuSlave(11);
        private readonly ModbusRtuSlave slave2 = new ModbusRtuSlave(22);
        private readonly ModbusRtuSlave slave3 = new ModbusRtuSlave(33);
        private readonly ModbusRtuSlave slave4 = new ModbusRtuSlave(44);
        //创建两个电压板从站实例，分别为地址1、2
        private readonly ModbusRtuSlaveVBT Volt_A_Slave = new ModbusRtuSlaveVBT(1);
        private readonly ModbusRtuSlaveVBT Volt_B_Slave = new ModbusRtuSlaveVBT(2);

        // AI
        private readonly ushort[] DI_RESULT = new ushort[32];

        // 泵运行电流
        private double P1_Cur = 0.0;
        private double N1_Cur = 0.0; 
        private double P2_Cur = 0.0;
        private double N2_Cur = 0.0;

        // 压力 & 流量
        private double PN1_PRESS_DIFF = 0.0;
        private double PN2_PRESS_DIFF = 0.0;
        private double PN1_FLOW_DIFF  = 0.0;
        private double PN2_FLOW_DIFF  = 0.0;

        // AB侧电堆电压基准
        private double A_ES_VOLT_BASE = 12.12;
        private double B_ES_VOLT_BASE = 12.15;
        // AB侧OCV电压基准
        private double A_OCV_VOLT_BASE = 1.2583;
        private double B_OCV_VOLT_BASE = 1.2587;

        // AB侧电堆电压 & OCV
        private readonly double[] A_ES_VOLT = new double[6];
        private double A_OCV;
        private readonly double[] B_ES_VOLT = new double[6];
        private double B_OCV;

        // 电流转换后的电压值
        private ushort A_CURRENT_VOLT;
        private ushort B_CURRENT_VOLT;

        // 总电压 & 总功率
        private float Total_Volt = 0;
        private float Total_Power = 0;

        // PCS工作模式
        //0:空转 1:充电 2:放电 
        private ushort PCS_WORK_MODE = 0;

        private int ConvertToCurrentVolt(float aCurrent)
        {
            double scaleFactor = 4000.0 / 1000.0;
            double result = aCurrent * scaleFactor;
            return (int)result;
        }

        private void Update_RTU_Regs()
        {
            ushort A_Max_Total_V = (ushort)Math.Max((A_ES_VOLT[0] + A_ES_VOLT[1] + A_ES_VOLT[2]), (A_ES_VOLT[3] + A_ES_VOLT[4] + A_ES_VOLT[5]));

            Volt_A_Slave.SetHoldingRegister(36, (ushort)(A_Max_Total_V * 10));//0
            Volt_A_Slave.SetHoldingRegister(37, (ushort)(A_ES_VOLT[0] * 10)); //1
            Volt_A_Slave.SetHoldingRegister(38, (ushort)(A_ES_VOLT[1] * 10)); //1
            Volt_A_Slave.SetHoldingRegister(39, (ushort)(A_ES_VOLT[2] * 10)); //1
            Volt_A_Slave.SetHoldingRegister(40, (ushort)(A_ES_VOLT[3] * 10)); //1
            Volt_A_Slave.SetHoldingRegister(41, (ushort)(A_ES_VOLT[4] * 10)); //1
            Volt_A_Slave.SetHoldingRegister(42, (ushort)(A_ES_VOLT[5] * 10)); //1
            Volt_A_Slave.SetHoldingRegister(43, 0);//7
            Volt_A_Slave.SetHoldingRegister(44, 0);//8
            Volt_A_Slave.SetHoldingRegister(45, A_CURRENT_VOLT);//9电流通道
            Volt_A_Slave.SetHoldingRegister(46, (ushort)(A_OCV * 10000));//10
            Volt_A_Slave.SetHoldingRegister(47, A_CURRENT_VOLT);//12
            Volt_A_Slave.SetHoldingRegister(48, A_CURRENT_VOLT);//13

            ushort B_Max_Total_V = (ushort)Math.Max((B_ES_VOLT[0] + B_ES_VOLT[1] + B_ES_VOLT[2]), (B_ES_VOLT[3] + B_ES_VOLT[4] + B_ES_VOLT[5]));

            Volt_B_Slave.SetHoldingRegister(36, (ushort)(B_Max_Total_V * 10));
            Volt_B_Slave.SetHoldingRegister(37, (ushort)(B_ES_VOLT[0] * 10)); //1
            Volt_B_Slave.SetHoldingRegister(38, (ushort)(B_ES_VOLT[1] * 10)); //1
            Volt_B_Slave.SetHoldingRegister(39, (ushort)(B_ES_VOLT[2] * 10)); //1
            Volt_B_Slave.SetHoldingRegister(40, (ushort)(B_ES_VOLT[3] * 10)); //1
            Volt_B_Slave.SetHoldingRegister(41, (ushort)(B_ES_VOLT[4] * 10)); //1
            Volt_B_Slave.SetHoldingRegister(42, (ushort)(B_ES_VOLT[5] * 10)); //1
            Volt_B_Slave.SetHoldingRegister(43, 0);
            Volt_B_Slave.SetHoldingRegister(44, 0);
            Volt_B_Slave.SetHoldingRegister(45, B_CURRENT_VOLT);
            Volt_B_Slave.SetHoldingRegister(46, (ushort)(B_OCV * 10000));
            Volt_B_Slave.SetHoldingRegister(47, B_CURRENT_VOLT);
            Volt_B_Slave.SetHoldingRegister(48, B_CURRENT_VOLT);

            Total_Volt = (float)(A_Max_Total_V + B_Max_Total_V);

            LogHelper.Logger.Information("UpDate Total Volt:" + Total_Volt.ToString());
        }

        private ushort GenerateRandomNumber()
        {
            int randomNumber = random.Next(0, 5);
            int finalNumber = randomNumber;  
            return (ushort)Math.Abs(finalNumber); 
        }

        private ushort GenerateRandomNumber03()
        {
            int randomNumber = random.Next(0, 2);
            int finalNumber = randomNumber - 1;  
            return (ushort)Math.Abs(finalNumber); 
        }

        private float RandomFloatGenerator()
        {
            Random rand = new Random();
            float value = (float)(rand.NextDouble() * 0.345f); 
            return value;
        }

        public void EstackAndOcvVoltInit()
        {
            // 电池电堆初始电压 
            for (int i = 0; i < A_ES_VOLT.Length; i++)
            {
                A_ES_VOLT[i] = A_ES_VOLT_BASE + GenerateRandomNumber03() + RandomFloatGenerator();
                B_ES_VOLT[i] = B_ES_VOLT_BASE + GenerateRandomNumber03() + RandomFloatGenerator();
            }
            // OCV初始电压
            A_OCV = A_OCV_VOLT_BASE;
            B_OCV = B_OCV_VOLT_BASE;

            // 界面显示
            textBox24.Text = A_ES_VOLT[0].ToString("F1") + "V";
            textBox25.Text = A_ES_VOLT[1].ToString("F1") + "V";
            textBox27.Text = A_ES_VOLT[2].ToString("F1") + "V";
            textBox15.Text = A_ES_VOLT[3].ToString("F1") + "V";
            textBox23.Text = A_ES_VOLT[4].ToString("F1") + "V";
            textBox26.Text = A_ES_VOLT[5].ToString("F1") + "V";
            textBox34.Text = A_OCV.ToString("F4") + "V";
            // 界面显示
            textBox35.Text = B_ES_VOLT[0].ToString("F1") + "V";
            textBox33.Text = B_ES_VOLT[1].ToString("F1") + "V";
            textBox31.Text = B_ES_VOLT[2].ToString("F1") + "V";
            textBox32.Text = B_ES_VOLT[3].ToString("F1") + "V";
            textBox30.Text = B_ES_VOLT[4].ToString("F1") + "V";
            textBox29.Text = B_ES_VOLT[5].ToString("F1") + "V";
            textBox28.Text = B_OCV.ToString("F4") + "V";
        }

        public void EstackAndOcvVoltUpdateShow()
        {
            // 电池电堆初始电压
            if (true)
            {
                for (int i = 0; i < A_ES_VOLT.Length; i++)
                {
                    A_ES_VOLT[i] = A_ES_VOLT_BASE + RandomFloatGenerator();
                }

                for (int i = 0; i < A_ES_VOLT.Length; i++)
                {
                    B_ES_VOLT[i] = B_ES_VOLT_BASE + +RandomFloatGenerator();
                }
            }
            // OCV初始电压
            A_OCV = A_OCV_VOLT_BASE + RandomFloatGenerator() * 0.000245;
            B_OCV = B_OCV_VOLT_BASE + RandomFloatGenerator() * 0.000245;

            // 界面显示
            textBox24.Text = A_ES_VOLT[0].ToString("F1") + "V";
            textBox25.Text = A_ES_VOLT[1].ToString("F1") + "V";
            textBox27.Text = A_ES_VOLT[2].ToString("F1") + "V";
            textBox15.Text = A_ES_VOLT[3].ToString("F1") + "V";
            textBox23.Text = A_ES_VOLT[4].ToString("F1") + "V";
            textBox26.Text = A_ES_VOLT[5].ToString("F1") + "V";
            textBox34.Text = A_OCV.ToString("F4") + "V";
            // 界面显示
            textBox35.Text = B_ES_VOLT[0].ToString("F1") + "V";
            textBox33.Text = B_ES_VOLT[1].ToString("F1") + "V";
            textBox31.Text = B_ES_VOLT[2].ToString("F1") + "V";
            textBox32.Text = B_ES_VOLT[3].ToString("F1") + "V";
            textBox30.Text = B_ES_VOLT[4].ToString("F1") + "V";
            textBox29.Text = B_ES_VOLT[5].ToString("F1") + "V";
            textBox28.Text = B_OCV.ToString("F4") + "V";
        }

        private float Branch_Cur1;
        private float Branch_Cur2;
        private float Branch_Cur1_BASE = 2.5F;
        private float Branch_Cur2_BASE = 3.1F;

        public void Branch_Cur_Init()
        {
            Branch_Cur1 = (float)2.7;
            Branch_Cur2 = (float)2.5;

            hslGauge1.Value = Branch_Cur1;
            hslGauge2.Value = Branch_Cur2;
        }

        public void Branch_Cur_UpdateShow()
        {
            Branch_Cur1 = (float)(Branch_Cur1_BASE + (GenerateRandomNumber() * 0.712) );
            Branch_Cur2 = (float)(Branch_Cur2_BASE + (GenerateRandomNumber() * 0.712) );

            A_CURRENT_VOLT = (ushort)ConvertToCurrentVolt(Branch_Cur1);
            B_CURRENT_VOLT = (ushort)ConvertToCurrentVolt(Branch_Cur2);

            LogHelper.Logger.Information("---------------------------------------------");
            LogHelper.Logger.Information("支路-1(A):" + Branch_Cur1 + " 支路-2(A):" + Branch_Cur2);
            LogHelper.Logger.Information("支路-1(SetV):" + (short)A_CURRENT_VOLT + " 支路-2(SetV):" + (short)B_CURRENT_VOLT);
            LogHelper.Logger.Information("---------------------------------------------");

            float Branch_Cur1_Show = Math.Abs(Branch_Cur1); 
            float Branch_Cur2_Show = Math.Abs(Branch_Cur2); 

            hslGauge1.Value = Branch_Cur1_Show;
            hslGauge2.Value = Branch_Cur2_Show;

            uiDigitalLabel2.Value = Branch_Cur1 + Branch_Cur2;
        }

        // 电解液温度基值
        private int P1_ELECTP_TEMP_BASE = 191;
        private int N1_ELECTP_TEMP_BASE = 192;
        private int P2_ELECTP_TEMP_BASE = 192;
        private int N2_ELECTP_TEMP_BASE = 193;

        // 电解液储液罐温度基值
        private int P1_CYG_TEMP_BASE = 183;
        private int N1_CYG_TEMP_BASE = 183;
        private int P2_CYG_TEMP_BASE = 183;
        private int N2_CYG_TEMP_BASE = 183;

        public void AllTempInit()
        {
            // 电解液温度基值
            trackBar8.Value = P1_ELECTP_TEMP_BASE;
            //
            trackBar7.Value = N1_ELECTP_TEMP_BASE;
            //
            trackBar5.Value = P2_ELECTP_TEMP_BASE;
            //
            trackBar6.Value = N2_ELECTP_TEMP_BASE;

            trackBar12.Value = P1_CYG_TEMP_BASE;
            //
            trackBar11.Value = N1_CYG_TEMP_BASE;
            //
            trackBar10.Value = P2_CYG_TEMP_BASE;
            //
            trackBar9.Value = N2_CYG_TEMP_BASE;

            // 界面显示
            textBox38.Text = (trackBar8.Value / 10.0).ToString("F1");
            textBox39.Text = (trackBar7.Value / 10.0).ToString("F1");
            textBox41.Text = (trackBar12.Value / 10.0).ToString("F1");
            textBox42.Text = (trackBar11.Value / 10.0).ToString("F1");
            // 界面显示
            textBox36.Text = (trackBar5.Value / 10.0).ToString("F1");
            textBox22.Text = (trackBar10.Value / 10.0).ToString("F1");
            textBox37.Text = (trackBar6.Value / 10.0).ToString("F1");
            textBox40.Text = (trackBar9.Value / 10.0).ToString("F1");
        }

        public void AllTempUpdateShow()
        {
            trackBar8.Value = P1_ELECTP_TEMP_BASE + (int)((GenerateRandomNumber() * 0.256));
            //
            trackBar7.Value = N1_ELECTP_TEMP_BASE + (int)((GenerateRandomNumber() * 0.526));
            //
            trackBar5.Value = P2_ELECTP_TEMP_BASE + (int)((GenerateRandomNumber() * 0.256));
            //
            trackBar6.Value = N2_ELECTP_TEMP_BASE + (int)((GenerateRandomNumber() * 0.526));

            trackBar12.Value = P1_CYG_TEMP_BASE + (int)((GenerateRandomNumber() * 0.556));
            //
            trackBar11.Value = N1_CYG_TEMP_BASE + (int)((GenerateRandomNumber() * 0.556));

            trackBar10.Value = P2_CYG_TEMP_BASE + (int)((GenerateRandomNumber() * 0.556));
            //
            trackBar9.Value = N2_CYG_TEMP_BASE + (int)((GenerateRandomNumber() * 0.556));

            textBox38.Text = (trackBar8.Value / 10.0).ToString("F1");
            textBox39.Text = (trackBar7.Value / 10.0).ToString("F1");
            textBox41.Text = (trackBar12.Value / 10.0).ToString("F1");
            textBox42.Text = (trackBar11.Value / 10.0).ToString("F1");

            textBox36.Text = (trackBar5.Value / 10.0).ToString("F1");
            textBox22.Text = (trackBar10.Value / 10.0).ToString("F1");
            textBox37.Text = (trackBar6.Value / 10.0).ToString("F1");
            textBox40.Text = (trackBar9.Value / 10.0).ToString("F1");
        }
        // 随机数生成器
        private readonly Random random = new Random();  
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

        static double CalculateSocValue(double ocv)
        {
            Console.WriteLine("OCV Voltage Input: " + ocv + " V");

            double soc;

            if (ocv <= 1.25)
            {
                soc = 0.0;
            }
            else if (ocv < 1.48)
            {
                double f_ocv = ocv;
                soc = -63021.58807 * Math.Pow(f_ocv, 5)
                    + 418096.88999 * Math.Pow(f_ocv, 4)
                    - 1113069.82600 * Math.Pow(f_ocv, 3)
                    + 1487920.93576 * Math.Pow(f_ocv, 2)
                    - 999276.10223 * f_ocv
                    + 269765.43313;

                soc = soc / 10.0; // 原本是 0~1000 的整数，这里换算为百分比
            }
            else
            {
                soc = 100.0;
            }

            return soc;
        }



        private void Form1_Load(object sender, EventArgs e)
        {
            hslTitle1.TextLeft = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");

            // W:1940
            // H:1080
            //this.FormBorderStyle = FormBorderStyle.None;
            // 程序启动时（如Main函数中）添加
            // 记录启动信息
            LogHelper.Logger.Information("APP程序启动时间点 " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"));

            // 配置文件路径和加载
            string configPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "AppConfig.json");
            AppConfig config = ConfigLoader.LoadConfig(configPath);

            // 配置完整性检查
            if (config == null || config.VFBDevice == null || config.VoltDevice == null)
            {
                MessageBox.Show("AppConfig配置文件异常或缺失关键节点！", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            // 提取串口
            VFD_COM_PORT = config.VFBDevice.PortName;
            VBT_COM_PORT = config.VoltDevice.PortName;
            // IP配置提取
            AI01_ModuleIP = config.AIModule1.IPAddress;
            AI02_ModuleIP = config.AIModule2.IPAddress;
            DI_ModuleIP = config.DIModule.IPAddress;
            DO01_ModuleIP = config.DOModule1.IPAddress;
            DO02_ModuleIP = config.DOModule2.IPAddress;

            LogHelper.Logger.Information("VFD PORT : " + VFD_COM_PORT);
            LogHelper.Logger.Information("VBT PORT : " + VBT_COM_PORT);
            LogHelper.Logger.Information("AI-01 IP : " + AI01_ModuleIP.ToString());
            LogHelper.Logger.Information("AI-02 IP : " + AI02_ModuleIP.ToString());
            LogHelper.Logger.Information("DO-01 IP : " + DO01_ModuleIP.ToString());
            LogHelper.Logger.Information("DO-02 IP : " + DO02_ModuleIP.ToString());
            LogHelper.Logger.Information("DI-01 IP : " + DI_ModuleIP.ToString());

            // 压力流量差值初始化
            PN1_PRESS_DIFF = 0.0;
            PN2_PRESS_DIFF = 0.0;
            PN1_FLOW_DIFF  = 0.0;
            PN2_FLOW_DIFF  = 0.0;

            textBox11.Text = PN1_PRESS_DIFF.ToString("F3");
            textBox12.Text = PN1_FLOW_DIFF.ToString("F2");
            textBox17.Text = PN2_PRESS_DIFF.ToString("F3");
            textBox20.Text = PN2_FLOW_DIFF.ToString("F2");

            // 参数初始化
            EstackAndOcvVoltInit();
            Branch_Cur_Init();
            AllTempInit();

            // 为每个从站设置保持寄存器的初始值
            slave1.SetHoldingRegister(0, 0);  // - 设置从站11的寄存器0初始值
            slave2.SetHoldingRegister(0, 0);  // - 设置从站22的寄存器0初始值
            slave3.SetHoldingRegister(0, 0);  // - 设置从站33的寄存器0初始值
            slave4.SetHoldingRegister(0, 0);  // - 设置从站44的寄存器0初始值
        }

        private int slave1_last_randomv = 0;
        private int slave2_last_randomv = 0;
        private int slave3_last_randomv = 0;
        private int slave4_last_randomv = 0;
        private void timer1_Tick(object sender, EventArgs e)
        {
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
        }
        private async void timer2_Tick(object sender, EventArgs e)
        {
            double flow_max = 60.0; double flow_min = 0.0;
            double press_max = 0.28; double press_min = 0.0;

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

            textBox21.Text = P2_PV_Show.ToString("F3") + "Mpa";
            textBox19.Text = N2_PV_Show.ToString("F3") + "Mpa";

            textBox14.Text = P1_FV_Show.ToString("F3") + "m³/h";
            textBox13.Text = N1_FV_Show.ToString("F3") + "m³/h";

            textBox16.Text = P2_FV_Show.ToString("F3") + "m³/h";
            textBox18.Text = N2_FV_Show.ToString("F3") + "m³/h";

            double flow_sensor_max = 99.9;
            double press_sensor_max = 0.34;

            ushort P1_PV_SET = (ushort)((P1_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
            ushort P2_PV_SET = (ushort)((P2_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
            ushort N1_PV_SET = (ushort)((N1_PV_Show / press_sensor_max) * 16000.0 + 4000.0);
            ushort N2_PV_SET = (ushort)((N2_PV_Show / press_sensor_max) * 16000.0 + 4000.0);

            ushort P1_FV_SET = (ushort)((P1_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
            ushort P2_FV_SET = (ushort)((P2_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
            ushort N1_FV_SET = (ushort)((N1_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);
            ushort N2_FV_SET = (ushort)((N2_FV_Show / flow_sensor_max) * 16000.0 + 4000.0);

            LogHelper.Logger.Information("---------------------------------------------");
            LogHelper.Logger.Information("P1压力给定(mA):" + P1_PV_SET + " N1压力给定(mA):" + N1_PV_SET);
            LogHelper.Logger.Information("P2压力给定(mA):" + P2_PV_SET + " N2压力给定(mA):" + N2_PV_SET);
            LogHelper.Logger.Information("---------------------------------------------");

            LogHelper.Logger.Information("---------------------------------------------");
            LogHelper.Logger.Information("P1流量给定(mA):" + P1_FV_SET + " N1流量给定(mA):" + N1_FV_SET);
            LogHelper.Logger.Information("P2流量给定(mA):" + P2_FV_SET + " N2流量给定(mA):" + N2_FV_SET);
            LogHelper.Logger.Information("---------------------------------------------");

            double temp_sensor_range = 120.0;
            double temp_sensor_min = -20.0;
            //
            ushort P1_DJY_TEMP = (ushort)(((trackBar8.Value / 10.0 - temp_sensor_min) / temp_sensor_range) * 16000.0 + 4000.0);
            ushort N1_DJY_TEMP = (ushort)(((trackBar7.Value / 10.0 - temp_sensor_min) / temp_sensor_range) * 16000.0 + 4000.0);
            ushort P2_DJY_TEMP = (ushort)(((trackBar5.Value / 10.0 - temp_sensor_min) / temp_sensor_range) * 16000.0 + 4000.0);
            ushort N2_DJY_TEMP = (ushort)(((trackBar6.Value / 10.0 - temp_sensor_min) / temp_sensor_range) * 16000.0 + 4000.0);
            LogHelper.Logger.Information("---------------------------------------------");
            LogHelper.Logger.Information("P1电解液温度(mA):" + P1_DJY_TEMP + " N1电解液温度(mA):" + N1_DJY_TEMP);
            LogHelper.Logger.Information("P2电解液温度(mA):" + P2_DJY_TEMP + " N2电解液温度(mA):" + N2_DJY_TEMP);
            LogHelper.Logger.Information("---------------------------------------------");

            double cyg_temp_sensor_range = 100.0;
            double cyg_temp_sensor_min = 0.0;
            //
            ushort P1_CYG_TEMP = (ushort)(((trackBar12.Value / 10.0 - cyg_temp_sensor_min) / cyg_temp_sensor_range) * 16000.0 + 4000.0);
            ushort N1_CYG_TEMP = (ushort)(((trackBar11.Value / 10.0 - cyg_temp_sensor_min) / cyg_temp_sensor_range) * 16000.0 + 4000.0);
            ushort P2_CYG_TEMP = (ushort)(((trackBar10.Value / 10.0 - cyg_temp_sensor_min) / cyg_temp_sensor_range) * 16000.0 + 4000.0);
            ushort N2_CYG_TEMP = (ushort)(((trackBar9.Value / 10.0 - cyg_temp_sensor_min) / cyg_temp_sensor_range) * 16000.0 + 4000.0);
            //发送 Modbus 请求到一个新线程
            await Task.Run(() =>
            {
                ushort StartWriteAddr = 10;
                // AI2 设定值
                ushort[] AI2_SetVal =
                {
                    P2_DJY_TEMP,
                    N2_DJY_TEMP,
                    P2_FV_SET,
                    N2_FV_SET,
                    N1_FV_SET,
                    P1_FV_SET,      
                    N1_DJY_TEMP,
                    P1_DJY_TEMP,    
                    N1_CYG_TEMP,    //P1储罐温度AI21
                    P1_CYG_TEMP,    //P1储罐温度AI22
                    N1_PV_SET,      //P1负极压力AI23
                    P1_PV_SET       //P1正极压力AI24
                };
                AI02_ModbusClient.WriteMultipleRegisters(1, StartWriteAddr, AI2_SetVal);

                // AI1 设定值
                ushort[] AI1_SetVal =
                {
                    0,//1
                    0,//2
                    7888,//3
                    7888,//4
                    P1_CYG_TEMP,//5
                    6000,//6
                    0,//7
                    0,//8
                    P2_PV_SET,//9
                    N2_PV_SET,//10
                    N2_CYG_TEMP,//11
                    P2_CYG_TEMP//12
                };

                AI01_ModbusClient.WriteMultipleRegisters(1, StartWriteAddr, AI1_SetVal);
            });
        }

        private void textBox11_TextChanged(object sender, EventArgs e)
        {
            try
            {
                PN1_PRESS_DIFF = Convert.ToDouble(textBox11.Text);
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
                PN1_FLOW_DIFF = Convert.ToDouble(textBox12.Text);
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
                PN2_PRESS_DIFF = Convert.ToDouble(textBox17.Text);
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
                PN2_FLOW_DIFF = Convert.ToDouble(textBox20.Text);
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

        private void hslButton1_Click(object sender, EventArgs e)
        {
            PCS_WORK_MODE = 1;
            hslTitle1.TextRight = "充电运行中";
            hslTitle1.RightTextColor = Color.Lime;
            Branch_Cur1_BASE = (float)383.7;
            Branch_Cur2_BASE = (float)382.5;
            hslButton1.OriginalColor = Color.Lime;
            hslButton2.OriginalColor = Color.DimGray;
            hslButton3.OriginalColor = Color.DimGray;
            timer4.Start();
        }

        private void timer3_Tick(object sender, EventArgs e)
        {
            hslTitle1.TextLeft = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");

            Total_Power = (float)((Total_Volt) * (Branch_Cur1 + Branch_Cur2) / 1000.0);

            uiDigitalLabel1.Value = Total_Power;

            double A_SOC = CalculateSocValue(A_OCV);
            double B_SOC = CalculateSocValue(B_OCV);

            hslProgressColorful1.Value = (int)(A_SOC * 100);
            hslProgressColorful2.Value = (int)(B_SOC * 100);

            Console.WriteLine("@A SOC:" + A_SOC + "," + "@B SOC:" + B_SOC);

            Branch_Cur_UpdateShow();

            AllTempUpdateShow();

            EstackAndOcvVoltUpdateShow();

            Update_RTU_Regs();

            string result = string.Join(", ", DI_RESULT);
            Console.WriteLine("Input Registers: " + result);

            LogHelper.Logger.Information("DI模块采集:" + result);

            //if (DI_RESULT[1] == 1)
            //{
            //    hslMoveText1.Text = "PCS连锁已建立";
            //    hslMoveText1.ForeColor = Color.Lime;
            //}
            //else
            //{
            //    hslMoveText1.Text = "PCS连锁未建立";
            //    hslMoveText1.ForeColor = Color.Red;
            //}
        }

        private void checkBox14_CheckedChanged(object sender, EventArgs e)
        {
            // 支路1电流故障
            if (checkBox14.Checked)
            {
                Branch_Cur1_BASE = 1000;
            }
            else
            {
                Branch_Cur1_BASE = 0;
            }
        }

        private void hslButton8_Click(object sender, EventArgs e)
        {
            if (hslButton8.Text == "启动模拟器")
            {
                // 设置串口配置
                ModbusRtuSlave.SetSerialPortSettings(VFD_COM_PORT, 9600, Parity.None, 8, StopBits.One);
                Thread.Sleep(20);
                // 启动共享的 Modbus 线程
                ModbusRtuSlave.Start();
                Thread.Sleep(50);

                // 设置串口配置
                ModbusRtuSlaveVBT.SetSerialPortSettings(VBT_COM_PORT, 9600, Parity.None, 8, StopBits.One);
                //ModbusRtuSlaveVBT.SetSerialPortSettings("COM6", 9600, Parity.None, 8, StopBits.One);
                Thread.Sleep(20);
                // 启动共享的 Modbus 线程
                ModbusRtuSlaveVBT.Start();
                Thread.Sleep(50);

                // ModbusTcpClient实例化
                AI01_ModbusClient = new ModbusTcpClient();
                AI02_ModbusClient = new ModbusTcpClient();
                DO01_ModbusClient = new ModbusTcpClient();
                DO02_ModbusClient = new ModbusTcpClient();
                DI_ModbusClient = new ModbusTcpClient();

                // 依次连接
                AI01_ModbusClient.Connect(AI01_ModuleIP, ModbusEndianness.BigEndian);
                Thread.Sleep(50);
                AI02_ModbusClient.Connect(AI02_ModuleIP, ModbusEndianness.BigEndian);
                Thread.Sleep(50);
                DO01_ModbusClient.Connect(DO01_ModuleIP, ModbusEndianness.BigEndian);
                Thread.Sleep(50);
                DO01_ModbusClient.Connect(DO02_ModuleIP, ModbusEndianness.BigEndian);
                Thread.Sleep(50);
                DI_ModbusClient.Connect(DI_ModuleIP, ModbusEndianness.BigEndian);
                Thread.Sleep(50);

                timer1.Enabled = true;
                timer2.Enabled = true;
                timer3.Enabled = true;

                timer1.Start();
                timer2.Start();
                timer3.Start();

                hslButton8.OriginalColor = Color.Lime;
                hslButton8.Text = "关闭模拟器";
            }
            else if (hslButton8.Text == "关闭模拟器")
            {
                Application.Exit();
                Console.WriteLine("应用程序已退出。");
            }
        }

        public class SerialDeviceConfig
        {
            public string PortName { get; set; }
            public int BaudRate { get; set; }
            public int DataBits { get; set; }
            public int StopBits { get; set; }
            public string Parity { get; set; }
        }

        public class NetworkDeviceConfig
        {
            public string IPAddress { get; set; }
            public int Port { get; set; }
        }

        public class AppConfig
        {
            public SerialDeviceConfig VFBDevice { get; set; }
            public SerialDeviceConfig VoltDevice { get; set; }
            public NetworkDeviceConfig AIModule1 { get; set; }
            public NetworkDeviceConfig AIModule2 { get; set; }
            public NetworkDeviceConfig DIModule { get; set; }
            public NetworkDeviceConfig DOModule1 { get; set; }
            public NetworkDeviceConfig DOModule2 { get; set; }
        }

        private void hslButton2_Click(object sender, EventArgs e)
        {
            PCS_WORK_MODE = 2;
            Branch_Cur1_BASE = (float)0.0 - (float)318.7;
            Branch_Cur2_BASE = (float)0.0 - (float)316.5;
            hslButton1.OriginalColor = Color.DimGray;
            hslButton2.OriginalColor = Color.Lime;
            hslButton3.OriginalColor = Color.DimGray;

            hslTitle1.TextRight = "放电运行中";
            hslTitle1.RightTextColor = Color.Cyan;
            timer4.Start();
        }

        private void hslButton3_Click(object sender, EventArgs e)
        {
            PCS_WORK_MODE = 0;
            Branch_Cur1_BASE = (float)2.5;
            Branch_Cur2_BASE = (float)2.5;
            hslButton1.OriginalColor = Color.DimGray;
            hslButton2.OriginalColor = Color.DimGray;
            hslButton3.OriginalColor = Color.Lime;
            hslTitle1.TextRight = "系统空转";
            timer4.Stop();
        }

        private void trackBar8_Scroll(object sender, EventArgs e)
        {
            P1_ELECTP_TEMP_BASE = trackBar8.Value;
            textBox38.Text = (P1_ELECTP_TEMP_BASE / 10.0).ToString("F1");
        }

        private void trackBar7_Scroll(object sender, EventArgs e)
        {
            N1_ELECTP_TEMP_BASE = trackBar7.Value;
            textBox39.Text = (N1_ELECTP_TEMP_BASE / 10.0).ToString("F1");
        }

        private void trackBar12_Scroll(object sender, EventArgs e)
        {
            P1_CYG_TEMP_BASE = trackBar12.Value;
            textBox41.Text = (P1_CYG_TEMP_BASE / 10.0).ToString("F1");
        }

        private void trackBar11_Scroll(object sender, EventArgs e)
        {
            N1_CYG_TEMP_BASE = trackBar11.Value;
            textBox42.Text = (N1_CYG_TEMP_BASE / 10.0).ToString("F1");
        }

        private void trackBar5_Scroll(object sender, EventArgs e)
        {
            P2_ELECTP_TEMP_BASE = trackBar5.Value;
            textBox36.Text = (P2_ELECTP_TEMP_BASE / 10.0).ToString("F1");
        }

        private void trackBar10_Scroll(object sender, EventArgs e)
        {
            P2_CYG_TEMP_BASE = trackBar10.Value;
            textBox22.Text = (P2_CYG_TEMP_BASE / 10.0).ToString("F1");
        }

        private void trackBar6_Scroll(object sender, EventArgs e)
        {
            N2_ELECTP_TEMP_BASE = trackBar6.Value;
            textBox37.Text = (N2_ELECTP_TEMP_BASE / 10.0).ToString("F1");
        }

        private void trackBar9_Scroll(object sender, EventArgs e)
        {
            N2_CYG_TEMP_BASE = trackBar9.Value;
            textBox40.Text = (N2_CYG_TEMP_BASE / 10.0).ToString("F1");
        }

        private void trackBar1_Scroll(object sender, EventArgs e)
        {
            PN1_PRESS_DIFF = trackBar1.Value / 1000.0;
            textBox11.Text = PN1_PRESS_DIFF.ToString("F3");
        }

        private void trackBar2_Scroll(object sender, EventArgs e)
        {
            PN1_FLOW_DIFF = trackBar2.Value / 1000.0;
            textBox12.Text = PN1_FLOW_DIFF.ToString("F3");
        }

        private void trackBar3_Scroll(object sender, EventArgs e)
        {
            PN2_PRESS_DIFF = trackBar3.Value / 1000.0;
            textBox17.Text = PN2_PRESS_DIFF.ToString("F3");
        }

        private void trackBar4_Scroll(object sender, EventArgs e)
        {
            PN2_FLOW_DIFF = trackBar4.Value / 1000.0;
            textBox20.Text = PN2_FLOW_DIFF.ToString("F3");
        }

        private void checkBox15_CheckedChanged(object sender, EventArgs e)
        {
            // 支路2电流故障
            if (checkBox15.Checked)
            {
                Branch_Cur2_BASE = 1000;
            }
            else
            {
                Branch_Cur2_BASE = 0;
            }
        }

        double A_ES_ADJUST_STEP = 0.0;
        double B_ES_ADJUST_STEP = 0.0;

        double A_ES_MAX_VOLT = 80.5;
        double B_ES_MAX_VOLT = 80.6;

        double A_ES_MIN_VOLT = 11.6;
        double B_ES_MIN_VOLT = 11.9;

        double A_OCV_ADJUST_STEP = 0.0;
        double B_OCV_ADJUST_STEP = 0.0;

        double A_OCV_MAX_VOLT = 1.485;
        double B_OCV_MAX_VOLT = 1.485;

        double A_OCV_MIN_VOLT = 1.239;
        double B_OCV_MIN_VOLT = 1.239;

        int CHARGE_TICK_CNT = 0;
        int DISCHARGE_TICK_CNT = 0;

        int ChargeAddCount = 0;
        int DischargeAddCount = 0;
        private void timer4_Tick(object sender, EventArgs e)
        {

            float time_add_freq = 300.0F;

            if (checkBox13.Checked)
            {
                label10.Text = "循环：" + ChargeAddCount.ToString() + " " + DischargeAddCount.ToString();
            }
            else
            {
                label10.Text = string.Empty;
            }

            if (PCS_WORK_MODE == 1)
            {

                //if (DI_RESULT[1] == 1)
                {
                    hslMoveText1.Text = "PCS连锁已建立";
                    hslMoveText1.ForeColor = Color.Lime;
                }
                //else
                //{
                //    hslMoveText1.Text = "PCS连锁未建立";
                //    hslMoveText1.ForeColor = Color.Red;
                //}

                if (A_ES_VOLT_BASE > A_ES_MAX_VOLT)
                {
                    A_ES_VOLT_BASE = A_ES_MAX_VOLT;
                }

                if (B_ES_VOLT_BASE > B_ES_MAX_VOLT)
                {
                    B_ES_VOLT_BASE = B_ES_MAX_VOLT;
                }

                if (A_ES_VOLT_BASE == A_ES_MAX_VOLT && B_ES_VOLT_BASE == B_ES_MAX_VOLT)
                {
                    A_ES_ADJUST_STEP = 0.0;
                    B_ES_ADJUST_STEP = 0.0;

                    A_OCV_ADJUST_STEP = 0.0;
                    B_OCV_ADJUST_STEP = 0.0;

                    hslTitle1.TextRight = "充电完成";
                    hslTitle1.RightTextColor = Color.Lime;
                    CHARGE_TICK_CNT = 0;

                    hslProgressColorful1.Value = 1000;

                    if (checkBox13.Checked)
                    {
                        Thread.Sleep(5000);
                        {
                            PCS_WORK_MODE = 2;
                            Branch_Cur1_BASE = -(float)388.7;
                            Branch_Cur2_BASE = -(float)386.5;
                            hslButton1.OriginalColor = Color.DimGray;
                            hslButton2.OriginalColor = Color.Lime;
                            hslButton3.OriginalColor = Color.DimGray;
                            ChargeAddCount++;
                            hslTitle1.TextRight = "放电运行中";
                            hslTitle1.RightTextColor = Color.Cyan;
                        }
                    }
                    else
                    {
                        timer4.Stop();
                    }
                }
                else
                {
                    CHARGE_TICK_CNT++;

                    LogHelper.Logger.Information("充电操作步数:" + CHARGE_TICK_CNT);

                    if (A_ES_ADJUST_STEP == 0.0)
                    {
                        A_ES_ADJUST_STEP = (A_ES_MAX_VOLT - A_ES_VOLT_BASE) / time_add_freq;
                    }

                    if (B_ES_ADJUST_STEP == 0.0)
                    {
                        B_ES_ADJUST_STEP = (B_ES_MAX_VOLT - B_ES_VOLT_BASE) / time_add_freq;
                    }

                    if (A_OCV_ADJUST_STEP == 0.0)
                    {
                        A_OCV_ADJUST_STEP = (A_OCV_MAX_VOLT - A_OCV_VOLT_BASE) / time_add_freq;
                    }

                    if (B_OCV_ADJUST_STEP == 0.0)
                    {
                        B_OCV_ADJUST_STEP = (B_OCV_MAX_VOLT - B_OCV_VOLT_BASE) / time_add_freq;
                    }

                    A_OCV_VOLT_BASE = A_OCV_VOLT_BASE + A_OCV_ADJUST_STEP;
                    B_OCV_VOLT_BASE = B_OCV_VOLT_BASE + B_OCV_ADJUST_STEP;

                    A_ES_VOLT_BASE = A_ES_VOLT_BASE + A_ES_ADJUST_STEP;
                    B_ES_VOLT_BASE = B_ES_VOLT_BASE + B_ES_ADJUST_STEP;
                }
            }

            if (PCS_WORK_MODE == 2)
            {

                {
                    hslMoveText1.Text = "PCS连锁已建立";
                    hslMoveText1.ForeColor = Color.Lime;
                }

                if (A_ES_VOLT_BASE < A_ES_MIN_VOLT)
                {
                    A_ES_VOLT_BASE = A_ES_MIN_VOLT;
                }

                if (B_ES_VOLT_BASE < B_ES_MIN_VOLT)
                {
                    B_ES_VOLT_BASE = B_ES_MIN_VOLT;
                }

                if (A_ES_VOLT_BASE == A_ES_MIN_VOLT && B_ES_VOLT_BASE == B_ES_MIN_VOLT)
                {
                    A_ES_ADJUST_STEP = 0.0;
                    B_ES_ADJUST_STEP = 0.0;

                    A_OCV_ADJUST_STEP = 0.0;
                    B_OCV_ADJUST_STEP = 0.0;

                    hslTitle1.TextRight = "放电完成";
                    hslTitle1.RightTextColor = Color.Cyan;
                    DISCHARGE_TICK_CNT = 0;

                    hslProgressColorful1.Value = 0;

                    if (checkBox13.Checked)
                    {
                        Thread.Sleep(5000);

                        PCS_WORK_MODE = 1;
                        hslTitle1.TextRight = "充电运行中";
                        hslTitle1.RightTextColor = Color.Lime;
                        Branch_Cur1_BASE = (float)372.7;
                        Branch_Cur2_BASE = (float)376.5;
                        hslButton1.OriginalColor = Color.Lime;
                        hslButton2.OriginalColor = Color.DimGray;
                        hslButton3.OriginalColor = Color.DimGray;
                        DischargeAddCount++;
                    }
                    else
                    {
                        timer4.Stop();
                    }
                }
                else
                {
                    DISCHARGE_TICK_CNT++;

                    LogHelper.Logger.Information("放电操作步数:" + DISCHARGE_TICK_CNT);

                    if (A_ES_ADJUST_STEP == 0.0)
                    {
                        A_ES_ADJUST_STEP = (A_ES_MAX_VOLT - A_ES_MIN_VOLT) / time_add_freq;
                    }
                    if (B_ES_ADJUST_STEP == 0.0)
                    {
                        B_ES_ADJUST_STEP = (B_ES_MAX_VOLT - B_ES_MIN_VOLT) / time_add_freq;
                    }
                    if (A_OCV_ADJUST_STEP == 0.0)
                    {
                        A_OCV_ADJUST_STEP = (A_OCV_MAX_VOLT - A_OCV_MIN_VOLT) / time_add_freq;
                    }
                    if (B_OCV_ADJUST_STEP == 0.0)
                    {
                        B_OCV_ADJUST_STEP = (B_OCV_MAX_VOLT - B_OCV_MIN_VOLT) / time_add_freq;
                    }

                    A_OCV_VOLT_BASE = A_OCV_VOLT_BASE - A_OCV_ADJUST_STEP;
                    B_OCV_VOLT_BASE = B_OCV_VOLT_BASE - B_OCV_ADJUST_STEP;

                    A_ES_VOLT_BASE = A_ES_VOLT_BASE - A_ES_ADJUST_STEP;
                    B_ES_VOLT_BASE = B_ES_VOLT_BASE - B_ES_ADJUST_STEP;
                }
            }

            if(PCS_WORK_MODE == 0)
            {
                {
                    hslMoveText1.Text = "PCS连锁未建立";
                    hslMoveText1.ForeColor = Color.Red;
                }
            }

            EstackAndOcvVoltUpdateShow();
            Update_RTU_Regs();
        }
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
        //Console.WriteLine("Setting Addr:" + address.ToString());
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
    private static ILogger _logger;

    /// <summary>
    /// 初始化Serilog日志，按天分割，保留最近3天
    /// </summary>
    public static void Init()
    {
        string logDir = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "logs");
        System.IO.Directory.CreateDirectory(logDir);
        string logPath = System.IO.Path.Combine(logDir, "LOG.log");  // 基础名

        _logger = new LoggerConfiguration()
            .MinimumLevel.Information()
            .WriteTo.File(
                logPath,
                rollingInterval: RollingInterval.Day,
                retainedFileCountLimit: 3,
                outputTemplate: "{Timestamp:yyyy-MM-dd HH:mm:ss.fff} [{Level}] {Message}{NewLine}{Exception}",
                shared: false
            )
            .CreateLogger();
    }

    public static ILogger Logger
    {
        get
        {
            if (_logger == null)
            {
                Init();
            }
            return _logger;
        }
    }
}
public class ModbusRtuSlaveVBT
{
    private static bool isPortOpen = false;  // 是否已打开串口
    private static SerialPort serialPort;    // 串口实例
    private static Dictionary<byte, ModbusRtuSlaveVBT> slaveInstances = new Dictionary<byte, ModbusRtuSlaveVBT>();  // 存储所有从站实例

    private byte slaveAddress;
    private ushort[] holdingRegisters;

    private float currentFrequency = 0;  // 当前频率，单位Hz
    private const float maxFrequency = 6000.0f;  // 最大频率，单位Hz
    private const float maxCurrent = 1500.0f;  // 最大电流，单位A
    private float targetFrequency = 0;  // 目标频率，单位Hz

    private ushort final_current = 0;

    private string response_log = "";

    private bool comm_state = false;

    // 静态构造函数，初始化串口
    static ModbusRtuSlaveVBT()
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
    public ModbusRtuSlaveVBT(byte slaveAddress)
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
        //Console.WriteLine("Setting Addr:" + address.ToString() + " VALUE:" + value.ToString());
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

    public bool GetCommState()
    {
        return comm_state;
    }

    // 逐步变化频率（模拟真实场景）
    private void GradualFrequencyChange()
    {
        float startFrequency = currentFrequency;
        float changeDuration = 1.2f;

        if (Math.Abs(targetFrequency - startFrequency) > 20 * 100)
        {
            changeDuration = 6.5f;
        }
        else if (Math.Abs(targetFrequency - startFrequency) > 10 * 100 && Math.Abs(targetFrequency - startFrequency) < 20 * 100)
        {
            changeDuration = 4.5f;
        }
        else if (Math.Abs(targetFrequency - startFrequency) > 5 * 100 && Math.Abs(targetFrequency - startFrequency) < 10 * 100)
        {
            changeDuration = 3.3f;
        }
        else if (Math.Abs(targetFrequency - startFrequency) > 250 && Math.Abs(targetFrequency - startFrequency) < 500)
        {
            changeDuration = 2.5f;
        }
        else
        {
            changeDuration = 1.3f;
        }

        float frequencyChangeRate = (targetFrequency - startFrequency) / changeDuration; // 每秒变化频率

        // 逐步变化频率
        for (float t = 0; t < changeDuration; t += 0.1f) // 每 0.1 秒变化一次
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
        int randomNumber = random.Next(-3, 4);
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
        // 每次有数据到达时触发
        int bytesToRead = serialPort.BytesToRead;
        byte[] request = new byte[bytesToRead];
        serialPort.Read(request, 0, bytesToRead);


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
                slave.comm_state = !slave.comm_state;

                serialPort.Write(response, 0, response.Length);
                Console.WriteLine("Sent response: " + BitConverter.ToString(response));
                slave.response_log = "Sent response: " + BitConverter.ToString(response);
            }
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

        byte[] response = new byte[6]; // 响应长度：从站地址 + 功能码 + 寄存器地址 + 寄存器值 + CRC

        response[0] = slaveAddress;  // 从站地址
        response[1] = 0x06;  // 功能码（0x06：写单个寄存器）
        response[2] = (byte)(registerAddress >> 8);  // 寄存器地址高字节
        response[3] = (byte)(registerAddress & 0xFF);  // 寄存器地址低字节
        response[4] = (byte)(registerValue >> 8);  // 寄存器值高字节
        response[5] = (byte)(registerValue & 0xFF);  // 寄存器值低字节

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
