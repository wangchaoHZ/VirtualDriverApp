using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;
using NLog;

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
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            try
            {
                Application.Run(new Form1());
            }
            finally
            {
                // 等待异步日志队列写完并关闭文件句柄。
                LogManager.Shutdown();
            }
        }
    }
}
