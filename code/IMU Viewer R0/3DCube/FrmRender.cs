/* Viewer for IMU's
 * R0
*/
using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Text;
using System.Windows.Forms;
using System.IO;
using System.IO.Ports;

namespace _DCubeNoGimbalLock
{
    public partial class FrmRender : Form
    {
        [System.Runtime.InteropServices.DllImportAttribute("gdi32.dll")]
        private static extern bool BitBlt(IntPtr hdcDest, int nXDest, int nYDest, int nWidth, int nHeight, IntPtr hdcSrc, int nXSrc, int nYSrc, System.Int32 dwRop);

        [System.Runtime.InteropServices.DllImportAttribute("user32.dll")]
        public static extern IntPtr GetDC(IntPtr hwnd);

        [System.Runtime.InteropServices.DllImportAttribute("user32.dll")]
        public static extern IntPtr ReleaseDC(IntPtr hwnd, IntPtr hdc);

        //added
        SerialPort Arduino = new SerialPort();

        int X_IMU = 0;
        int Y_IMU = 0;
        int Z_IMU = 0;
        string SerialData;

        public FrmRender()
        {
            InitializeComponent();

            //initialize com ports
            string[] ports = SerialPort.GetPortNames();

            foreach (string port in ports)
            {
                PortcomboBox.Items.Add(port);
            }

            Arduino.BaudRate = 38400;
            
        }

        Math3D.Cube mainCube;
        Point drawOrigin;

        private void FrmRender_Load(object sender, EventArgs e)
        {
            mainCube = new Math3D.Cube(200, 75, 100);
            drawOrigin = new Point(pictureBox1.Width / 2, pictureBox1.Height / 2);

            mainCube.FillFront = true;
            mainCube.FillBack = true;
            mainCube.FillLeft = true;
            mainCube.FillRight = true;
            mainCube.FillTop = true;
            mainCube.FillBottom = true;
        }

        private void Render()
        {
            mainCube.RotateX = -(float)X_IMU;
            mainCube.RotateY = -(float)Y_IMU;
            mainCube.RotateZ = -(float)Z_IMU;

            pictureBox1.Image = mainCube.DrawCube(drawOrigin);

            textBoxX.Text = X_IMU.ToString();
            textBoxY.Text = Y_IMU.ToString();
            textBoxZ.Text = Z_IMU.ToString();
        }

        private void Form1_Paint(object sender, PaintEventArgs e)
        {
            Render();
        }

        /// <summary>
        /// This functions gets the ADC value from the arduino
        /// </summary>
        private void Arduino_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            SerialData = Arduino.ReadLine();

            try
            {
                var intList = new List<int>(Array.ConvertAll(SerialData.Split(','), Convert.ToInt32));
                X_IMU = intList[0];
                Y_IMU = intList[1];
                Z_IMU = intList[2];
            }
            catch
            {
                X_IMU = 0;
                Y_IMU = 0;
                Z_IMU = 0;
            }
        }

        private void PortcomboBox_SelectedIndexChanged_1(object sender, EventArgs e)
        {
            Arduino.Close();
            string port = PortcomboBox.SelectedItem.ToString();
            Arduino.PortName = port;
            Arduino.Open();
            this.Arduino.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(this.Arduino_DataReceived);
            PortcomboBox.Enabled = false;
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            Render();
        }
    }
}