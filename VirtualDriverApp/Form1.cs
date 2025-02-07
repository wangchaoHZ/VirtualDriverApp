using System;
using System.Collections.Generic;
using System.Drawing;
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
        }


        // 创建四个从站实例，分别为地址11、22、33、44
        ModbusRtuSlave slave1 = new ModbusRtuSlave(11);
        ModbusRtuSlave slave2 = new ModbusRtuSlave(22);
        ModbusRtuSlave slave3 = new ModbusRtuSlave(33);
        ModbusRtuSlave slave4 = new ModbusRtuSlave(44);

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
            textBox9.Clear();
            pictureBox2.Visible = false;

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
            timer1.Enabled = true;
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
        }

        private int slave1_last_randomv = 0;
        private int slave2_last_randomv = 0;
        private int slave3_last_randomv = 0;
        private int slave4_last_randomv = 0;
        private void timer1_Tick(object sender, EventArgs e)
        {

            if (slave1.GetFinalCurrent() > 100)
            {
                int rdv = random.Next(-10, 11);
                slave1_last_randomv = (slave1_last_randomv > 0) ? -Math.Sign(rdv) * rdv : Math.Sign(rdv) * rdv;
                ushort rv = (ushort)(slave1.GetFinalCurrent() + slave1_last_randomv);

                if (checkBox1.Checked || checkBox2.Checked)
                {
                    if (checkBox1.Checked)
                    {
                        slave1.SetHoldingRegister(0x3004, 3000);
                    }
                    else
                    {
                        slave1.SetHoldingRegister(0x3004, 30);
                    }
                }
                else
                {
                    slave1.SetHoldingRegister(0x3004, rv);
                }
            }

            if (slave2.GetFinalCurrent() > 100)
            {
                int rdv = random.Next(-10, 11);
                slave2_last_randomv = (slave2_last_randomv > 0) ? -Math.Sign(rdv) * rdv : Math.Sign(rdv) * rdv;
                ushort rv = (ushort)(slave2.GetFinalCurrent() + slave2_last_randomv);

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
                }
                else
                {
                    slave2.SetHoldingRegister(0x3004, rv);
                }
            }

            if (slave3.GetFinalCurrent() > 100)
            {
                int rdv = random.Next(-10, 11);
                slave3_last_randomv = (slave3_last_randomv > 0) ? -Math.Sign(rdv) * rdv : Math.Sign(rdv) * rdv;
                ushort rv = (ushort)(slave3.GetFinalCurrent() + slave3_last_randomv);

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
                }
                else
                {
                    slave3.SetHoldingRegister(0x3004, rv);
                }
            }

            if (slave4.GetFinalCurrent() > 100)
            {
                int rdv = random.Next(-10, 11);
                slave4_last_randomv = (slave4_last_randomv > 0) ? -Math.Sign(rdv) * rdv : Math.Sign(rdv) * rdv;
                ushort rv = (ushort)(slave4.GetFinalCurrent() + slave4_last_randomv);

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
                }
                else
                {
                    slave4.SetHoldingRegister(0x3004, rv);
                }
            }

            textBox1.Text = ((double)slave1.GetHoldingRegister(0x3000) / 100.0).ToString("F2") + " HZ";
            textBox2.Text = ((double)slave1.GetHoldingRegister(0x3004) / 100.0).ToString("F2") + " A";

            textBox4.Text = ((double)slave2.GetHoldingRegister(0x3000) / 100.0).ToString("F2") + " HZ";
            textBox3.Text = ((double)slave2.GetHoldingRegister(0x3004) / 100.0).ToString("F2") + " A";

            textBox8.Text = ((double)slave3.GetHoldingRegister(0x3000) / 100.0).ToString("F2") + " HZ";
            textBox7.Text = ((double)slave3.GetHoldingRegister(0x3004) / 100.0).ToString("F2") + " A";

            textBox6.Text = ((double)slave4.GetHoldingRegister(0x3000) / 100.0).ToString("F2") + " HZ";
            textBox5.Text = ((double)slave4.GetHoldingRegister(0x3004) / 100.0).ToString("F2") + " A";

            if (slave1.GetCurentLogString().Length > 0)
            {
                textBox9.Text += "P1:" + slave1.GetCurentLogString() + "\r\n";
            }
            if (slave2.GetCurentLogString().Length > 0)
            {
                textBox9.Text += "N1:" + slave2.GetCurentLogString() + "\r\n";
            }
            if (slave3.GetCurentLogString().Length > 0)
            {
                textBox9.Text += "P2:" + slave3.GetCurentLogString() + "\r\n";
            }
            if (slave4.GetCurentLogString().Length > 0)
            {
                textBox9.Text += "N2:" + slave4.GetCurentLogString() + "\r\n";
            }

            if (textBox9.Lines.Length > 9)
            {
                textBox9.Clear();
            }
        }
        private void button2_Click(object sender, EventArgs e)
        {
            Application.Exit();
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
