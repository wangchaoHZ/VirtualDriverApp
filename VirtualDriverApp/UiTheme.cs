using System.Drawing;
using System.Windows.Forms;

namespace VirtualDriverApp
{
    internal static class UiTheme
    {
        public static readonly Color PageBackground =
            Color.FromArgb(244, 247, 251);
        public static readonly Color CardBackground = Color.White;
        public static readonly Color Primary =
            Color.FromArgb(37, 99, 235);
        public static readonly Color PrimaryHover =
            Color.FromArgb(29, 78, 216);
        public static readonly Color PrimaryLight =
            Color.FromArgb(239, 246, 255);
        public static readonly Color Heading =
            Color.FromArgb(15, 23, 42);
        public static readonly Color Text =
            Color.FromArgb(51, 65, 85);
        public static readonly Color MutedText =
            Color.FromArgb(100, 116, 139);
        public static readonly Color DisabledText =
            Color.FromArgb(148, 163, 184);
        public static readonly Color Border =
            Color.FromArgb(226, 232, 240);
        public static readonly Color InputBorder =
            Color.FromArgb(203, 213, 225);
        public static readonly Color ReadoutBackground =
            Color.FromArgb(248, 250, 252);
        public static readonly Color Success =
            Color.FromArgb(22, 163, 74);
        public static readonly Color SuccessLight =
            Color.FromArgb(240, 253, 244);
        public static readonly Color Warning =
            Color.FromArgb(217, 119, 6);
        public static readonly Color WarningLight =
            Color.FromArgb(255, 251, 235);
        public static readonly Color Danger =
            Color.FromArgb(220, 38, 38);
        public static readonly Color DangerLight =
            Color.FromArgb(254, 242, 242);

        public const int PagePadding = 20;
        public const int CardGap = 16;
        public const int CardPadding = 16;
        public const int MinimumContentWidth = 1240;

        public static Font CreateFont(
            float size,
            FontStyle style = FontStyle.Regular)
        {
            return new Font(
                "Microsoft YaHei UI",
                size,
                style,
                GraphicsUnit.Point);
        }

        public static Font CreateNumberFont(
            float size,
            FontStyle style = FontStyle.Bold)
        {
            return new Font(
                "Segoe UI",
                size,
                style,
                GraphicsUnit.Point);
        }

        public static void StyleReadout(
            TextBox textBox,
            float fontSize)
        {
            textBox.BackColor = ReadoutBackground;
            textBox.BorderStyle = BorderStyle.None;
            textBox.Font = CreateNumberFont(fontSize);
            textBox.ForeColor = Heading;
            textBox.Multiline = false;
            textBox.ReadOnly = true;
            textBox.TabStop = false;
            textBox.TextAlign = HorizontalAlignment.Right;
        }

        public static void StyleInput(Control control)
        {
            control.BackColor = CardBackground;
            control.Font = CreateFont(10.0f);
            control.ForeColor = Text;
            control.Margin = new Padding(0);
        }

        public static void StylePrimaryButton(Button button)
        {
            button.BackColor = Primary;
            button.FlatAppearance.BorderColor = Primary;
            button.FlatAppearance.BorderSize = 1;
            button.FlatAppearance.MouseDownBackColor = PrimaryHover;
            button.FlatAppearance.MouseOverBackColor = PrimaryHover;
            button.FlatStyle = FlatStyle.Flat;
            button.Font = CreateFont(11.0f, FontStyle.Bold);
            button.ForeColor = Color.White;
            button.UseVisualStyleBackColor = false;
        }

        public static void StyleDangerButton(Button button)
        {
            button.BackColor = DangerLight;
            button.FlatAppearance.BorderColor = Danger;
            button.FlatAppearance.BorderSize = 1;
            button.FlatAppearance.MouseDownBackColor =
                Color.FromArgb(254, 226, 226);
            button.FlatAppearance.MouseOverBackColor =
                Color.FromArgb(254, 226, 226);
            button.FlatStyle = FlatStyle.Flat;
            button.Font = CreateFont(11.0f, FontStyle.Bold);
            button.ForeColor = Danger;
            button.UseVisualStyleBackColor = false;
        }
    }

    internal sealed class CardPanel : Panel
    {
        public CardPanel()
        {
            BackColor = UiTheme.CardBackground;
            DoubleBuffered = true;
            Padding = new Padding(UiTheme.CardPadding);
            ResizeRedraw = true;
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            using (var pen = new Pen(UiTheme.Border))
            {
                Rectangle border = ClientRectangle;
                border.Width -= 1;
                border.Height -= 1;
                e.Graphics.DrawRectangle(pen, border);
            }
        }
    }
}
