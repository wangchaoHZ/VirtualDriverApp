using System;
using System.Windows.Forms;

namespace VirtualDriverApp
{
    internal static class Program
    {
        /// <summary>
        /// 应用程序的主入口点。
        /// </summary>
        [STAThread]
        static void Main()
        {
            LogHelper.Init();
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new Form1());
            Serilog.Log.CloseAndFlush();
        }
    }
}
