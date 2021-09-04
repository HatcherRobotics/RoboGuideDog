using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace DogImgClient
{
    class Program
    {
        static MainForm mainForm = new MainForm();

        static void Main(string[] args)
        {
            mainForm.ShowDialog();
        }
    }
}
