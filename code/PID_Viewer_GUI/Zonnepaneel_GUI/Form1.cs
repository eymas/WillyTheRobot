using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

using System.IO;
using System.IO.Ports;

using System.Timers;

namespace Zonnepaneel_GUI
{
    public partial class Form1 : Form
    {
        bool Started = false;           //Flag to indicate the program is started     

        int SamplesVisible = 200;      //number of visible samples [n]

        float Drive_Input = 0;             
        float Turn_Input = 0;              
        
        float Drive_Speed = 0;
        float Turn_Speed = 0;

        float Drive_Output = 0;
        float Turn_Output = 0;

        string s;

        SerialPort SerialCom = new SerialPort();

        public Form1()
        {
            InitializeComponent();

            //initialize com ports
            string[] ports = SerialPort.GetPortNames();

            foreach (string port in ports)
            {
                PortcomboBox.Items.Add(port);
            }
        }

        /// <summary>
        /// This function updates the interface components.
        /// </summary>
        private void UpdateInterface()
        {
            Turn_Input_textBox.Text = Turn_Input.ToString();
            Drive_Input_textBox.Text = Drive_Input.ToString();

            Turn_Speed_textBox.Text = Turn_Speed.ToString();
            Drive_Speed_textBox.Text = Drive_Speed.ToString();

            Turn_Output_textBox.Text = Turn_Output.ToString();
            Drive_Output_textBox.Text = Drive_Output.ToString();

            Incoming_Serial_textBox.Text = s;

            TurnChart.Series["Turn Input"].Points.AddY(Turn_Input);
            TurnChart.Series["Turn Speed"].Points.AddY(Turn_Speed);
            DriveChart.Series["Drive Input"].Points.AddY(Drive_Input);
            DriveChart.Series["Drive Speed"].Points.AddY(Drive_Speed);

            if (SamplesVisible == 0)
            {
                TurnChart.Series[0].Points.RemoveAt(0);
                TurnChart.Series[1].Points.RemoveAt(0);
                DriveChart.Series[0].Points.RemoveAt(0);
                DriveChart.Series[1].Points.RemoveAt(0);
            }
            else
            {
                SamplesVisible--;
            }
        }

        private void Serial_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            try
            {
                s = SerialCom.ReadLine();
                var intList = new List<float>(Array.ConvertAll(s.Split(';'), Convert.ToSingle));
                Turn_Input = intList[0] / 100000;
                Drive_Input = intList[1] / 100000;

                Turn_Speed = intList[2] / 100000;
                Drive_Speed = intList[3] / 100000;

                Turn_Output = intList[4] / 100000;
                Drive_Output = intList[5] / 100000;
            }
            catch
            {
                Turn_Input = 0;
                Drive_Input = 0;

                Turn_Speed = 0;
                Drive_Speed = 0;

                Turn_Output = 0;
                Drive_Output = 0;
            }
        }

        private void StartStop()
        {
            if (Started)
            {
                //Stop sequence
                SerialCom.Close();
                PortcomboBox.Enabled = true;
                BaudcomboBox.Enabled = true;
                this.SerialCom.DataReceived -= new System.IO.Ports.SerialDataReceivedEventHandler(this.Serial_DataReceived);
                Started = false;
                UpdateInterface_Timer.Enabled = false;
                Start_Button.Text = "Start";
            }
            else
            {
                //Start sequence
                try
                {
                    SerialCom.Close();
                    SerialCom.PortName = PortcomboBox.SelectedItem.ToString();
                    SerialCom.BaudRate = int.Parse(BaudcomboBox.SelectedItem.ToString());
                    SerialCom.Open();

                    PortcomboBox.Enabled = false;
                    BaudcomboBox.Enabled = false;

                    this.SerialCom.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(this.Serial_DataReceived);
                    Started = true;
                    UpdateInterface_Timer.Enabled = true;
                    Start_Button.Text = " Stop";
                }
                catch
                {
                    MessageBox.Show("Error: Serial could not be opend!");
                }
            }
        }

        private void Start_Button_Click(object sender, EventArgs e)
        {
            StartStop();
        }

        private void UpdateInterface_Timer_Tick(object sender, EventArgs e)
        {
            UpdateInterface();
        }
    }
}
