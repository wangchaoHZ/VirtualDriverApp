using System;
using System.Drawing;
using System.Windows.Forms;

namespace VirtualDriverApp
{
    public partial class Form1
    {
        private Panel _contentViewport;
        private TableLayoutPanel _contentLayout;
        private TableLayoutPanel _monitorLayout;
        private TableLayoutPanel _pumpGrid;
        private TableLayoutPanel _sensorGrid;
        private Control[] _pumpCards;
        private Control[] _sensorCards;
        private TableLayoutPanel _rtuEndpointLayout;
        private TableLayoutPanel _tcpEndpointLayout;
        private Panel _headerStatusHost;
        private Label _headerStatusLabel;
        private Label[] _pumpStatusLabels;
        private CheckBox[] _pumpFaultCheckBoxes;
        private ToolTip _modernToolTip;

        private void InitializeModernLayout()
        {
            SuspendLayout();
            Controls.Clear();

            AutoScroll = false;
            BackColor = UiTheme.PageBackground;
            ClientSize = new Size(1440, 900);
            Font = UiTheme.CreateFont(10.0f);
            MinimumSize = new Size(1100, 700);
            Text = "BMS 变频器模拟终端";
            WindowState = FormWindowState.Maximized;

            _modernToolTip = new ToolTip(components)
            {
                AutoPopDelay = 8000,
                InitialDelay = 400,
                ReshowDelay = 100,
                ShowAlways = true
            };

            Panel header = BuildHeader();
            _contentViewport = new Panel
            {
                AutoScroll = true,
                BackColor = UiTheme.PageBackground,
                Dock = DockStyle.Fill
            };

            _contentLayout = BuildContentLayout();
            _contentViewport.Controls.Add(_contentLayout);
            _contentViewport.Resize += ContentViewport_Resize;
            DpiChanged += Form1_DpiChanged;

            Controls.Add(_contentViewport);
            Controls.Add(header);

            label10.TextChanged += StatusLabel_Changed;
            label10.ForeColorChanged += StatusLabel_Changed;

            _modernToolTip.SetToolTip(
                label28,
                "四台模拟变频器共用的频率给定保持寄存器地址。 ");
            _modernToolTip.SetToolTip(
                checkBox17,
                "使用 Modbus TCP 功能码 16，将八项压力/流量数据连续写入保持寄存器 0～7。 ");
            _modernToolTip.SetToolTip(
                numericUpDownHydraulicUnitId,
                "接收压力/流量数据的 Modbus TCP 从站地址。 ");
            _modernToolTip.SetToolTip(
                textBoxTcpListenAddress,
                "0.0.0.0 表示监听本机所有 IPv4 网络接口。 ");
            _modernToolTip.SetToolTip(
                textBoxHydraulicTargetIp,
                "压力/流量数据接收设备的 IPv4 地址。 ");

            foreach (CheckBox checkBox in _pumpFaultCheckBoxes)
            {
                checkBox.CheckedChanged += PumpFault_CheckedChanged;
            }

            ResumeLayout(true);
            UpdateModernContentSize();
            UpdateModernUiState();
        }

        private Panel BuildHeader()
        {
            var header = new Panel
            {
                BackColor = UiTheme.CardBackground,
                Dock = DockStyle.Top,
                Height = 84,
                Padding = new Padding(24, 10, 24, 10)
            };

            var layout = new TableLayoutPanel
            {
                BackColor = UiTheme.CardBackground,
                ColumnCount = 3,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 1
            };
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Absolute, 220F));
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Absolute, 270F));
            layout.RowStyles.Add(
                new RowStyle(SizeType.Percent, 100F));

            pictureBox1.Dock = DockStyle.Fill;
            pictureBox1.Margin = new Padding(0, 4, 20, 4);
            pictureBox1.SizeMode = PictureBoxSizeMode.Zoom;

            var titleLayout = new TableLayoutPanel
            {
                BackColor = UiTheme.CardBackground,
                ColumnCount = 2,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 2
            };
            titleLayout.ColumnStyles.Add(
                new ColumnStyle(SizeType.AutoSize));
            titleLayout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            titleLayout.RowStyles.Add(
                new RowStyle(SizeType.Percent, 64F));
            titleLayout.RowStyles.Add(
                new RowStyle(SizeType.Percent, 36F));

            label9.AutoSize = true;
            label9.Dock = DockStyle.Fill;
            label9.Font = UiTheme.CreateFont(22.0f, FontStyle.Bold);
            label9.ForeColor = UiTheme.Heading;
            label9.Margin = new Padding(0);
            label9.Text = "储能控制系统变频器模拟终端";
            label9.TextAlign = ContentAlignment.BottomLeft;

            var badge = new Label
            {
                AutoSize = true,
                BackColor = UiTheme.PrimaryLight,
                Dock = DockStyle.Left,
                Font = UiTheme.CreateFont(10.0f, FontStyle.Bold),
                ForeColor = UiTheme.Primary,
                Margin = new Padding(0, 4, 0, 0),
                Padding = new Padding(8, 2, 8, 2),
                Text = "MPC 联调验证",
                TextAlign = ContentAlignment.MiddleCenter
            };
            titleLayout.Controls.Add(label9, 0, 0);
            titleLayout.SetColumnSpan(label9, 2);
            titleLayout.Controls.Add(badge, 0, 1);

            _headerStatusHost = new Panel
            {
                BackColor = UiTheme.PageBackground,
                Dock = DockStyle.Fill,
                Margin = new Padding(12, 10, 0, 10),
                Padding = new Padding(12, 4, 12, 4)
            };
            _headerStatusLabel = new Label
            {
                AutoEllipsis = true,
                Dock = DockStyle.Fill,
                Font = UiTheme.CreateFont(10.0f, FontStyle.Bold),
                ForeColor = UiTheme.MutedText,
                Text = "未启动",
                TextAlign = ContentAlignment.MiddleCenter
            };
            _headerStatusHost.Controls.Add(_headerStatusLabel);

            layout.Controls.Add(pictureBox1, 0, 0);
            layout.Controls.Add(titleLayout, 1, 0);
            layout.Controls.Add(_headerStatusHost, 2, 0);
            header.Controls.Add(layout);
            return header;
        }

        private TableLayoutPanel BuildContentLayout()
        {
            var layout = new TableLayoutPanel
            {
                BackColor = UiTheme.PageBackground,
                ColumnCount = 2,
                Height = 820,
                Margin = new Padding(0),
                RowCount = 1
            };
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Absolute, 350F));
            layout.RowStyles.Add(
                new RowStyle(SizeType.Percent, 100F));

            TableLayoutPanel monitor = BuildMonitorArea();
            monitor.Margin = new Padding(0, 0, 8, 0);
            TableLayoutPanel controls = BuildControlArea();
            controls.Margin = new Padding(8, 0, 0, 0);
            layout.Controls.Add(monitor, 0, 0);
            layout.Controls.Add(controls, 1, 0);
            return layout;
        }

        private TableLayoutPanel BuildMonitorArea()
        {
            _monitorLayout = new TableLayoutPanel
            {
                BackColor = UiTheme.PageBackground,
                ColumnCount = 1,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 5
            };
            TableLayoutPanel layout = _monitorLayout;
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 36F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 390F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 16F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 36F));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

            layout.Controls.Add(CreateSectionTitle("变频器监控"), 0, 0);

            _pumpGrid = new TableLayoutPanel
            {
                AutoScroll = true,
                BackColor = UiTheme.PageBackground,
                ColumnCount = 4,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 1
            };
            TableLayoutPanel pumpGrid = _pumpGrid;
            for (int column = 0; column < 4; column++)
            {
                pumpGrid.ColumnStyles.Add(
                    new ColumnStyle(SizeType.Percent, 25F));
            }
            pumpGrid.RowStyles.Add(
                new RowStyle(SizeType.Percent, 100F));

            _pumpStatusLabels = new Label[4];
            _pumpFaultCheckBoxes = new[]
            {
                checkBox13,
                checkBox14,
                checkBox15,
                checkBox16
            };

            pumpGrid.Controls.Add(
                CreatePumpCard(
                    0,
                    "P1 正极泵",
                    11,
                    textBox1,
                    textBox2,
                    textBox15,
                    checkBox1,
                    checkBox2,
                    checkBox13),
                0,
                0);
            pumpGrid.Controls.Add(
                CreatePumpCard(
                    1,
                    "N1 负极泵",
                    22,
                    textBox4,
                    textBox3,
                    textBox22,
                    checkBox4,
                    checkBox3,
                    checkBox14),
                1,
                0);
            pumpGrid.Controls.Add(
                CreatePumpCard(
                    2,
                    "P2 正极泵",
                    33,
                    textBox8,
                    textBox7,
                    textBox23,
                    checkBox6,
                    checkBox5,
                    checkBox15),
                2,
                0);
            pumpGrid.Controls.Add(
                CreatePumpCard(
                    3,
                    "N2 负极泵",
                    44,
                    textBox6,
                    textBox5,
                    textBox24,
                    checkBox8,
                    checkBox7,
                    checkBox16),
                3,
                0);
            _pumpCards = new[]
            {
                pumpGrid.GetControlFromPosition(0, 0),
                pumpGrid.GetControlFromPosition(1, 0),
                pumpGrid.GetControlFromPosition(2, 0),
                pumpGrid.GetControlFromPosition(3, 0)
            };
            layout.Controls.Add(pumpGrid, 0, 1);

            layout.Controls.Add(CreateSectionTitle("核心传感器"), 0, 3);
            layout.Controls.Add(BuildSensorGrid(), 0, 4);
            return layout;
        }

        private Control BuildSensorGrid()
        {
            _sensorGrid = new TableLayoutPanel
            {
                AutoScroll = true,
                BackColor = UiTheme.PageBackground,
                ColumnCount = 4,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 2
            };
            TableLayoutPanel grid = _sensorGrid;
            for (int column = 0; column < 4; column++)
            {
                grid.ColumnStyles.Add(
                    new ColumnStyle(SizeType.Percent, 25F));
            }
            grid.RowStyles.Add(new RowStyle(SizeType.Percent, 50F));
            grid.RowStyles.Add(new RowStyle(SizeType.Percent, 50F));

            grid.Controls.Add(
                CreateSensorCard("P1 正极压力", textBox10, "MPa"),
                0,
                0);
            grid.Controls.Add(
                CreateSensorCard("P1 正极流量", textBox14, "m³/h"),
                1,
                0);
            grid.Controls.Add(
                CreateSensorCard("P2 正极压力", textBox21, "MPa"),
                2,
                0);
            grid.Controls.Add(
                CreateSensorCard("P2 正极流量", textBox16, "m³/h"),
                3,
                0);
            grid.Controls.Add(
                CreateSensorCard("N1 负极压力", textBox9, "MPa"),
                0,
                1);
            grid.Controls.Add(
                CreateSensorCard("N1 负极流量", textBox13, "m³/h"),
                1,
                1);
            grid.Controls.Add(
                CreateSensorCard("N2 负极压力", textBox19, "MPa"),
                2,
                1);
            grid.Controls.Add(
                CreateSensorCard("N2 负极流量", textBox18, "m³/h"),
                3,
                1);
            _sensorCards = new[]
            {
                grid.GetControlFromPosition(0, 0),
                grid.GetControlFromPosition(1, 0),
                grid.GetControlFromPosition(2, 0),
                grid.GetControlFromPosition(3, 0),
                grid.GetControlFromPosition(0, 1),
                grid.GetControlFromPosition(1, 1),
                grid.GetControlFromPosition(2, 1),
                grid.GetControlFromPosition(3, 1)
            };
            return grid;
        }

        private CardPanel CreatePumpCard(
            int statusIndex,
            string name,
            int address,
            TextBox frequency,
            TextBox current,
            TextBox energy,
            CheckBox maximum,
            CheckBox minimum,
            CheckBox fault)
        {
            var card = new CardPanel
            {
                Dock = DockStyle.Fill,
                Margin = new Padding(8, 0, 8, 0),
                MinimumSize = new Size(200, 0)
            };
            var layout = new TableLayoutPanel
            {
                BackColor = UiTheme.CardBackground,
                ColumnCount = 1,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 5
            };
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 54F));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 33.33F));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 33.33F));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 33.34F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 42F));

            var header = new TableLayoutPanel
            {
                BackColor = UiTheme.CardBackground,
                ColumnCount = 2,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 2
            };
            header.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            header.ColumnStyles.Add(
                new ColumnStyle(SizeType.Absolute, 68F));
            header.RowStyles.Add(
                new RowStyle(SizeType.Percent, 58F));
            header.RowStyles.Add(
                new RowStyle(SizeType.Percent, 42F));

            var title = new Label
            {
                AutoSize = true,
                Dock = DockStyle.Fill,
                Font = UiTheme.CreateFont(13.0f, FontStyle.Bold),
                ForeColor = UiTheme.Heading,
                Margin = new Padding(0),
                Text = name,
                TextAlign = ContentAlignment.MiddleLeft
            };
            var addressLabel = new Label
            {
                AutoSize = true,
                Dock = DockStyle.Fill,
                Font = UiTheme.CreateFont(9.0f),
                ForeColor = UiTheme.MutedText,
                Margin = new Padding(0),
                Text = "Modbus 地址：" + address,
                TextAlign = ContentAlignment.TopLeft
            };
            var status = new Label
            {
                AutoSize = false,
                BackColor = UiTheme.PageBackground,
                Dock = DockStyle.Fill,
                Font = UiTheme.CreateFont(9.0f, FontStyle.Bold),
                ForeColor = UiTheme.MutedText,
                Margin = new Padding(4, 5, 0, 7),
                Text = "未启动",
                TextAlign = ContentAlignment.MiddleCenter
            };
            _pumpStatusLabels[statusIndex] = status;
            header.Controls.Add(title, 0, 0);
            header.Controls.Add(addressLabel, 0, 1);
            header.Controls.Add(status, 1, 0);
            header.SetRowSpan(status, 2);

            layout.Controls.Add(header, 0, 0);
            layout.Controls.Add(
                CreateMetricPanel("实时频率", frequency, "Hz", 20.0f),
                0,
                1);
            layout.Controls.Add(
                CreateMetricPanel("泵电流", current, "A", 20.0f),
                0,
                2);
            layout.Controls.Add(
                CreateMetricPanel("累计功耗", energy, "kWh", 18.0f),
                0,
                3);

            var flags = new TableLayoutPanel
            {
                BackColor = UiTheme.CardBackground,
                ColumnCount = 3,
                Dock = DockStyle.Fill,
                Margin = new Padding(0, 7, 0, 0),
                RowCount = 1
            };
            flags.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 33.33F));
            flags.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 33.33F));
            flags.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 33.34F));
            flags.RowStyles.Add(
                new RowStyle(SizeType.Percent, 100F));
            StyleSimulationCheckBox(maximum, "最大值");
            StyleSimulationCheckBox(minimum, "最小值");
            StyleSimulationCheckBox(fault, "故障");
            fault.ForeColor = UiTheme.Danger;
            flags.Controls.Add(maximum, 0, 0);
            flags.Controls.Add(minimum, 1, 0);
            flags.Controls.Add(fault, 2, 0);
            layout.Controls.Add(flags, 0, 4);

            _modernToolTip.SetToolTip(
                maximum,
                "保持原有最大电流模拟行为。 ");
            _modernToolTip.SetToolTip(
                minimum,
                "保持原有最小电流模拟行为。 ");
            _modernToolTip.SetToolTip(
                fault,
                "保持原有设备故障寄存器模拟行为。 ");

            card.Controls.Add(layout);
            return card;
        }

        private Control CreateMetricPanel(
            string title,
            TextBox value,
            string unit,
            float valueFontSize)
        {
            var panel = new Panel
            {
                BackColor = UiTheme.ReadoutBackground,
                Dock = DockStyle.Fill,
                Margin = new Padding(0, 5, 0, 5),
                Padding = new Padding(10, 5, 10, 4)
            };
            var layout = new TableLayoutPanel
            {
                BackColor = UiTheme.ReadoutBackground,
                ColumnCount = 2,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 2
            };
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Absolute, 48F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 22F));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

            var titleLabel = new Label
            {
                AutoSize = true,
                Dock = DockStyle.Fill,
                Font = UiTheme.CreateFont(10.5f, FontStyle.Bold),
                ForeColor = UiTheme.Text,
                Margin = new Padding(0),
                Text = title,
                TextAlign = ContentAlignment.MiddleLeft
            };
            UiTheme.StyleReadout(value, valueFontSize);
            value.Dock = DockStyle.Fill;
            value.Margin = new Padding(0, 2, 4, 0);

            var unitLabel = new Label
            {
                AutoSize = false,
                Dock = DockStyle.Fill,
                Font = UiTheme.CreateNumberFont(9.0f, FontStyle.Regular),
                ForeColor = UiTheme.MutedText,
                Margin = new Padding(0),
                Text = unit,
                TextAlign = ContentAlignment.BottomLeft
            };
            layout.Controls.Add(titleLabel, 0, 0);
            layout.SetColumnSpan(titleLabel, 2);
            layout.Controls.Add(value, 0, 1);
            layout.Controls.Add(unitLabel, 1, 1);
            panel.Controls.Add(layout);
            return panel;
        }

        private CardPanel CreateSensorCard(
            string title,
            TextBox value,
            string unit)
        {
            var card = new CardPanel
            {
                Dock = DockStyle.Fill,
                Margin = new Padding(8),
                MinimumSize = new Size(180, 0),
                Padding = new Padding(14, 10, 14, 10)
            };
            var layout = new TableLayoutPanel
            {
                BackColor = UiTheme.CardBackground,
                ColumnCount = 2,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 2
            };
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Absolute, 52F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 30F));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

            var titleLabel = new Label
            {
                AutoSize = true,
                Dock = DockStyle.Fill,
                Font = UiTheme.CreateFont(10.5f, FontStyle.Bold),
                ForeColor = UiTheme.Text,
                Margin = new Padding(0),
                Text = title,
                TextAlign = ContentAlignment.MiddleLeft
            };
            UiTheme.StyleReadout(value, 18.0f);
            value.BackColor = UiTheme.CardBackground;
            value.Dock = DockStyle.Fill;
            value.Margin = new Padding(0, 10, 6, 0);

            var unitLabel = new Label
            {
                AutoSize = false,
                Dock = DockStyle.Fill,
                Font = UiTheme.CreateNumberFont(9.0f, FontStyle.Regular),
                ForeColor = UiTheme.MutedText,
                Margin = new Padding(0),
                Text = unit,
                TextAlign = ContentAlignment.BottomLeft
            };
            layout.Controls.Add(titleLabel, 0, 0);
            layout.SetColumnSpan(titleLabel, 2);
            layout.Controls.Add(value, 0, 1);
            layout.Controls.Add(unitLabel, 1, 1);
            card.Controls.Add(layout);
            return card;
        }

        private TableLayoutPanel BuildControlArea()
        {
            var layout = new TableLayoutPanel
            {
                BackColor = UiTheme.PageBackground,
                ColumnCount = 1,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 6
            };
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 226F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 14F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 348F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 14F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 218F));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
            layout.Controls.Add(
                CreateConfigurationCard(
                    "模拟变频器服务",
                    BuildServerConfigurationBody()),
                0,
                0);
            layout.Controls.Add(
                CreateConfigurationCard(
                    "压力/流量通信",
                    BuildHydraulicConfigurationBody()),
                0,
                2);
            layout.Controls.Add(
                CreateConfigurationCard(
                    "运行控制",
                    BuildRunControlBody()),
                0,
                4);
            return layout;
        }

        private CardPanel CreateConfigurationCard(
            string title,
            Control body)
        {
            var card = new CardPanel
            {
                Dock = DockStyle.Fill,
                Margin = new Padding(0)
            };
            var layout = new TableLayoutPanel
            {
                BackColor = UiTheme.CardBackground,
                ColumnCount = 1,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 2
            };
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 34F));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
            var titleLabel = new Label
            {
                AutoSize = true,
                Dock = DockStyle.Fill,
                Font = UiTheme.CreateFont(14.0f, FontStyle.Bold),
                ForeColor = UiTheme.Heading,
                Margin = new Padding(0),
                Text = title,
                TextAlign = ContentAlignment.TopLeft
            };
            body.Dock = DockStyle.Fill;
            body.Margin = new Padding(0);
            layout.Controls.Add(titleLabel, 0, 0);
            layout.Controls.Add(body, 0, 1);
            card.Controls.Add(layout);
            return card;
        }

        private Control BuildServerConfigurationBody()
        {
            var layout = new TableLayoutPanel
            {
                BackColor = UiTheme.CardBackground,
                ColumnCount = 1,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 3
            };
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 44F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 44F));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

            var modeLayout = CreateFieldRow(78F);
            ConfigureFieldLabel(labelModbusMode, "通信模式");
            UiTheme.StyleInput(comboBoxModbusMode);
            comboBoxModbusMode.Dock = DockStyle.Fill;
            modeLayout.Controls.Add(labelModbusMode, 0, 0);
            modeLayout.Controls.Add(comboBoxModbusMode, 1, 0);

            var endpointHost = new Panel
            {
                BackColor = UiTheme.CardBackground,
                Dock = DockStyle.Fill,
                Margin = new Padding(0)
            };
            _rtuEndpointLayout = CreateFieldRow(78F);
            ConfigureFieldLabel(label11, "串行端口");
            UiTheme.StyleInput(comboBox1);
            comboBox1.Dock = DockStyle.Fill;
            _rtuEndpointLayout.Controls.Add(label11, 0, 0);
            _rtuEndpointLayout.Controls.Add(comboBox1, 1, 0);

            _tcpEndpointLayout = new TableLayoutPanel
            {
                BackColor = UiTheme.CardBackground,
                ColumnCount = 4,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 1
            };
            _tcpEndpointLayout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Absolute, 72F));
            _tcpEndpointLayout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            _tcpEndpointLayout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Absolute, 46F));
            _tcpEndpointLayout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Absolute, 70F));
            ConfigureFieldLabel(labelTcpListenAddress, "监听地址");
            ConfigureFieldLabel(labelTcpPort, "端口");
            UiTheme.StyleInput(textBoxTcpListenAddress);
            UiTheme.StyleInput(numericUpDownTcpPort);
            textBoxTcpListenAddress.Dock = DockStyle.Fill;
            numericUpDownTcpPort.Dock = DockStyle.Fill;
            labelTcpPort.TextAlign = ContentAlignment.MiddleRight;
            _tcpEndpointLayout.Controls.Add(labelTcpListenAddress, 0, 0);
            _tcpEndpointLayout.Controls.Add(textBoxTcpListenAddress, 1, 0);
            _tcpEndpointLayout.Controls.Add(labelTcpPort, 2, 0);
            _tcpEndpointLayout.Controls.Add(numericUpDownTcpPort, 3, 0);

            endpointHost.Controls.Add(_tcpEndpointLayout);
            endpointHost.Controls.Add(_rtuEndpointLayout);

            label28.AutoEllipsis = true;
            label28.Dock = DockStyle.Fill;
            label28.Font = UiTheme.CreateFont(9.0f);
            label28.ForeColor = UiTheme.MutedText;
            label28.Margin = new Padding(0, 8, 0, 0);
            label28.Text = "频率给定寄存器：0x2001";
            label28.TextAlign = ContentAlignment.TopLeft;

            layout.Controls.Add(modeLayout, 0, 0);
            layout.Controls.Add(endpointHost, 0, 1);
            layout.Controls.Add(label28, 0, 2);
            return layout;
        }

        private Control BuildHydraulicConfigurationBody()
        {
            var layout = new TableLayoutPanel
            {
                BackColor = UiTheme.CardBackground,
                ColumnCount = 1,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 5
            };
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 42F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 42F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 42F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 75F));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

            checkBox17.AutoSize = true;
            checkBox17.Dock = DockStyle.Fill;
            checkBox17.Font = UiTheme.CreateFont(10.0f, FontStyle.Bold);
            checkBox17.ForeColor = UiTheme.Text;
            checkBox17.Margin = new Padding(0, 3, 0, 3);
            checkBox17.Text = "启用压力/流量数据转发（FC16）";

            var targetRow = CreateFieldRow(78F);
            ConfigureFieldLabel(labelHydraulicTargetIp, "目标地址");
            UiTheme.StyleInput(textBoxHydraulicTargetIp);
            textBoxHydraulicTargetIp.Dock = DockStyle.Fill;
            targetRow.Controls.Add(labelHydraulicTargetIp, 0, 0);
            targetRow.Controls.Add(textBoxHydraulicTargetIp, 1, 0);

            var endpointRow = new TableLayoutPanel
            {
                BackColor = UiTheme.CardBackground,
                ColumnCount = 4,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 1
            };
            endpointRow.ColumnStyles.Add(
                new ColumnStyle(SizeType.Absolute, 78F));
            endpointRow.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 48F));
            endpointRow.ColumnStyles.Add(
                new ColumnStyle(SizeType.Absolute, 78F));
            endpointRow.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 52F));
            ConfigureFieldLabel(labelHydraulicPort, "目标端口");
            ConfigureFieldLabel(labelHydraulicUnitId, "从站地址");
            UiTheme.StyleInput(numericUpDownHydraulicPort);
            UiTheme.StyleInput(numericUpDownHydraulicUnitId);
            numericUpDownHydraulicPort.Dock = DockStyle.Fill;
            numericUpDownHydraulicUnitId.Dock = DockStyle.Fill;
            endpointRow.Controls.Add(labelHydraulicPort, 0, 0);
            endpointRow.Controls.Add(numericUpDownHydraulicPort, 1, 0);
            endpointRow.Controls.Add(labelHydraulicUnitId, 2, 0);
            endpointRow.Controls.Add(numericUpDownHydraulicUnitId, 3, 0);

            labelHydraulicRegisterMap.AutoEllipsis = true;
            labelHydraulicRegisterMap.Dock = DockStyle.Fill;
            labelHydraulicRegisterMap.Font = UiTheme.CreateFont(8.5f);
            labelHydraulicRegisterMap.ForeColor = UiTheme.MutedText;
            labelHydraulicRegisterMap.Margin = new Padding(0, 7, 0, 0);
            labelHydraulicRegisterMap.Text =
                "保持寄存器 0～7\r\n" +
                "0/1=P1 压力/流量；2/3=N1 压力/流量\r\n" +
                "4/5=P2 压力/流量；6/7=N2 压力/流量";

            labelHydraulicScale.AutoEllipsis = true;
            labelHydraulicScale.Dock = DockStyle.Fill;
            labelHydraulicScale.Font = UiTheme.CreateFont(8.5f);
            labelHydraulicScale.ForeColor = UiTheme.MutedText;
            labelHydraulicScale.Margin = new Padding(0, 4, 0, 0);
            labelHydraulicScale.Text =
                "编码：压力 × 10000（0.0001 MPa）\r\n" +
                "　　　流量 × 100（0.01 m³/h）";

            layout.Controls.Add(checkBox17, 0, 0);
            layout.Controls.Add(targetRow, 0, 1);
            layout.Controls.Add(endpointRow, 0, 2);
            layout.Controls.Add(labelHydraulicRegisterMap, 0, 3);
            layout.Controls.Add(labelHydraulicScale, 0, 4);
            return layout;
        }

        private Control BuildRunControlBody()
        {
            var layout = new TableLayoutPanel
            {
                BackColor = UiTheme.CardBackground,
                ColumnCount = 1,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 3
            };
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 52F));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 52F));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

            UiTheme.StylePrimaryButton(button1);
            button1.Dock = DockStyle.Fill;
            button1.Margin = new Padding(0, 2, 0, 5);
            button1.Text = "启动模拟器";

            UiTheme.StyleDangerButton(button2);
            button2.Dock = DockStyle.Fill;
            button2.Margin = new Padding(0, 5, 0, 2);
            button2.Text = "停止模拟器";

            label10.AutoEllipsis = true;
            label10.BackColor = UiTheme.PageBackground;
            label10.Dock = DockStyle.Fill;
            label10.Font = UiTheme.CreateFont(10.0f, FontStyle.Bold);
            label10.ForeColor = UiTheme.MutedText;
            label10.Margin = new Padding(0, 10, 0, 0);
            label10.Padding = new Padding(10, 4, 10, 4);
            label10.Text = "未启动";
            label10.TextAlign = ContentAlignment.MiddleCenter;

            layout.Controls.Add(button1, 0, 0);
            layout.Controls.Add(button2, 0, 1);
            layout.Controls.Add(label10, 0, 2);
            return layout;
        }

        private static TableLayoutPanel CreateFieldRow(
            float labelWidth)
        {
            var layout = new TableLayoutPanel
            {
                BackColor = UiTheme.CardBackground,
                ColumnCount = 2,
                Dock = DockStyle.Fill,
                Margin = new Padding(0),
                RowCount = 1
            };
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Absolute, labelWidth));
            layout.ColumnStyles.Add(
                new ColumnStyle(SizeType.Percent, 100F));
            return layout;
        }

        private static void ConfigureFieldLabel(
            Label label,
            string text)
        {
            label.AutoSize = false;
            label.Dock = DockStyle.Fill;
            label.Font = UiTheme.CreateFont(10.0f);
            label.ForeColor = UiTheme.Text;
            label.Margin = new Padding(0);
            label.Text = text;
            label.TextAlign = ContentAlignment.MiddleLeft;
        }

        private static void StyleSimulationCheckBox(
            CheckBox checkBox,
            string text)
        {
            checkBox.AutoSize = false;
            checkBox.Dock = DockStyle.Fill;
            checkBox.Font = UiTheme.CreateFont(9.0f);
            checkBox.ForeColor = UiTheme.Text;
            checkBox.Margin = new Padding(0);
            checkBox.Text = text;
            checkBox.UseVisualStyleBackColor = true;
        }

        private static Label CreateSectionTitle(string text)
        {
            return new Label
            {
                AutoSize = false,
                Dock = DockStyle.Fill,
                Font = UiTheme.CreateFont(14.0f, FontStyle.Bold),
                ForeColor = UiTheme.Heading,
                Margin = new Padding(8, 0, 0, 0),
                Text = text,
                TextAlign = ContentAlignment.MiddleLeft
            };
        }

        private void ContentViewport_Resize(object sender, EventArgs e)
        {
            UpdateModernContentSize();
        }

        private void Form1_DpiChanged(
            object sender,
            DpiChangedEventArgs e)
        {
            if (!IsHandleCreated || IsDisposed)
            {
                return;
            }

            BeginInvoke((MethodInvoker)ApplyModernDpiLayout);
        }

        private void ApplyModernDpiLayout()
        {
            UpdateModernContentSize();
            PerformLayout();
            ReflowModernGrids();
        }

        private void ReflowModernGrids()
        {
            ReflowModernGridsForDpi(DeviceDpi);
        }

        private void ReflowModernGridsForDpi(int dpi)
        {
            float dpiScale = dpi / 96.0f;
            bool useCompactColumns = dpi >= 144;
            int columnCount = useCompactColumns ? 2 : 4;

            ClearLayoutMaximumSizes(_contentLayout);
            _contentLayout.Height = (int)Math.Round(
                (useCompactColumns ? 1640.0 : 820.0) *
                dpiScale);
            _monitorLayout.RowStyles[0].Height =
                (float)Math.Round(36.0 * dpiScale);
            _monitorLayout.RowStyles[1].Height =
                (float)Math.Round(
                    (useCompactColumns ? 780.0 : 390.0) *
                    dpiScale);
            _monitorLayout.RowStyles[2].Height =
                (float)Math.Round(16.0 * dpiScale);
            _monitorLayout.RowStyles[3].Height =
                (float)Math.Round(36.0 * dpiScale);
            _contentLayout.PerformLayout();
            _monitorLayout.PerformLayout();

            ConfigureResponsiveGrid(
                _pumpGrid,
                _pumpCards,
                columnCount);
            ConfigureResponsiveGrid(
                _sensorGrid,
                _sensorCards,
                columnCount);
            ConstrainNestedTableLayouts(_contentLayout);
        }

        private static void ConfigureResponsiveGrid(
            TableLayoutPanel layout,
            Control[] controls,
            int columnCount)
        {
            if (layout == null || controls == null)
            {
                return;
            }

            layout.SuspendLayout();
            layout.MaximumSize = Size.Empty;
            layout.ColumnCount = columnCount;
            layout.RowCount = (int)Math.Ceiling(
                controls.Length / (double)columnCount);
            layout.ColumnStyles.Clear();
            layout.RowStyles.Clear();
            for (int column = 0;
                column < columnCount;
                column++)
            {
                layout.ColumnStyles.Add(
                    new ColumnStyle(
                        SizeType.Percent,
                        100.0f / columnCount));
            }

            for (int row = 0;
                row < layout.RowCount;
                row++)
            {
                layout.RowStyles.Add(
                    new RowStyle(
                        SizeType.Percent,
                        100.0f / layout.RowCount));
            }

            for (int index = 0;
                index < controls.Length;
                index++)
            {
                layout.SetCellPosition(
                    controls[index],
                    new TableLayoutPanelCellPosition(
                        index % columnCount,
                        index / columnCount));
            }

            layout.ResumeLayout(true);
            if (layout.Parent != null)
            {
                int desiredHeight = Math.Max(1, layout.Height);
                TableLayoutPanel parentLayout =
                    layout.Parent as TableLayoutPanel;
                if (parentLayout != null)
                {
                    TableLayoutPanelCellPosition position =
                        parentLayout.GetPositionFromControl(layout);
                    int[] rowHeights =
                        parentLayout.GetRowHeights();
                    if (position.Row >= 0 &&
                        position.Row < rowHeights.Length)
                    {
                        desiredHeight = Math.Max(
                            1,
                            rowHeights[position.Row] -
                                layout.Margin.Vertical);
                    }
                }

                layout.Dock = DockStyle.None;
                layout.Anchor = AnchorStyles.Top |
                    AnchorStyles.Left |
                    AnchorStyles.Right;
                layout.Left = 0;
                layout.MaximumSize = new Size(
                    layout.Parent.ClientSize.Width,
                    100000);
                layout.Width = Math.Max(
                    1,
                    layout.Parent.ClientSize.Width -
                        layout.Margin.Horizontal);
                layout.Height = desiredHeight;
            }

            layout.PerformLayout();
        }

        private static void ConstrainNestedTableLayouts(
            Control parent)
        {
            if (parent == null)
            {
                return;
            }

            foreach (Control child in parent.Controls)
            {
                TableLayoutPanel table =
                    child as TableLayoutPanel;
                if (table != null)
                {
                    int availableWidth = Math.Max(
                        1,
                        parent.DisplayRectangle.Width -
                            table.Margin.Horizontal);
                    table.MaximumSize = new Size(
                        availableWidth,
                        100000);
                    table.Width = availableWidth;
                }

                ConstrainNestedTableLayouts(child);
                table?.PerformLayout();
            }
        }

        private static void ClearLayoutMaximumSizes(Control parent)
        {
            if (parent == null)
            {
                return;
            }

            foreach (Control child in parent.Controls)
            {
                TableLayoutPanel table =
                    child as TableLayoutPanel;
                if (table != null)
                {
                    table.MaximumSize = Size.Empty;
                }

                ClearLayoutMaximumSizes(child);
            }
        }

        private void UpdateModernContentSize()
        {
            if (_contentViewport == null || _contentLayout == null)
            {
                return;
            }

            float dpiScale = DeviceDpi / 96.0f;
            int pagePadding = (int)Math.Round(
                UiTheme.PagePadding * dpiScale);
            int minimumContentWidth = (int)Math.Round(
                UiTheme.MinimumContentWidth * dpiScale);
            int availableWidth =
                _contentViewport.ClientSize.Width -
                pagePadding * 2 -
                SystemInformation.VerticalScrollBarWidth;
            _contentLayout.Width = Math.Max(
                minimumContentWidth,
                availableWidth);
            _contentLayout.Location = new Point(
                pagePadding,
                pagePadding);
        }

        private void StatusLabel_Changed(object sender, EventArgs e)
        {
            SyncHeaderStatus();
        }

        private void SyncHeaderStatus()
        {
            if (_headerStatusLabel == null ||
                _headerStatusHost == null)
            {
                return;
            }

            _headerStatusLabel.Text = label10.Text;
            if (label10.ForeColor == Color.Green ||
                label10.ForeColor == UiTheme.Success)
            {
                _headerStatusHost.BackColor = UiTheme.SuccessLight;
                _headerStatusLabel.ForeColor = UiTheme.Success;
            }
            else if (label10.ForeColor == Color.Red ||
                label10.ForeColor == UiTheme.Danger)
            {
                _headerStatusHost.BackColor = UiTheme.DangerLight;
                _headerStatusLabel.ForeColor = UiTheme.Danger;
            }
            else if (label10.ForeColor == Color.DarkOrange ||
                label10.ForeColor == UiTheme.Warning)
            {
                _headerStatusHost.BackColor = UiTheme.WarningLight;
                _headerStatusLabel.ForeColor = UiTheme.Warning;
            }
            else
            {
                _headerStatusHost.BackColor = UiTheme.PageBackground;
                _headerStatusLabel.ForeColor = UiTheme.MutedText;
            }
        }

        private void PumpFault_CheckedChanged(object sender, EventArgs e)
        {
            UpdateModernUiState();
        }

        private void UpdateModernUiState()
        {
            if (_pumpStatusLabels == null ||
                _pumpFaultCheckBoxes == null)
            {
                return;
            }

            for (int index = 0;
                index < _pumpStatusLabels.Length;
                index++)
            {
                Label status = _pumpStatusLabels[index];
                bool hasFault =
                    _pumpFaultCheckBoxes[index].Checked;

                if (!_systemStarted)
                {
                    status.BackColor = UiTheme.PageBackground;
                    status.ForeColor = UiTheme.MutedText;
                    status.Text = "未启动";
                }
                else if (hasFault)
                {
                    status.BackColor = UiTheme.DangerLight;
                    status.ForeColor = UiTheme.Danger;
                    status.Text = "故障";
                }
                else
                {
                    status.BackColor = UiTheme.SuccessLight;
                    status.ForeColor = UiTheme.Success;
                    status.Text = "运行中";
                }
            }

            SyncHeaderStatus();
        }
    }
}
