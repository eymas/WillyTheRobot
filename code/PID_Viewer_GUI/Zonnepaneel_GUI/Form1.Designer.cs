namespace Zonnepaneel_GUI
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.Windows.Forms.Label label2;
            System.Windows.Forms.Label label8;
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea1 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend1 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series1 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series2 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea2 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend2 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series3 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Series series4 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Form1));
            this.UpdateInterface_Timer = new System.Windows.Forms.Timer(this.components);
            this.Turn_Input_textBox = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.Turn_Speed_textBox = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.Drive_Input_textBox = new System.Windows.Forms.TextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.Start_Button = new System.Windows.Forms.Button();
            this.PortcomboBox = new System.Windows.Forms.ComboBox();
            this.Incoming_Serial_textBox = new System.Windows.Forms.TextBox();
            this.label5 = new System.Windows.Forms.Label();
            this.Drive_Speed_textBox = new System.Windows.Forms.TextBox();
            this.label6 = new System.Windows.Forms.Label();
            this.Drive_Output_textBox = new System.Windows.Forms.TextBox();
            this.label7 = new System.Windows.Forms.Label();
            this.Turn_Output_textBox = new System.Windows.Forms.TextBox();
            this.TurnChart = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.DriveChart = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.BaudcomboBox = new System.Windows.Forms.ComboBox();
            this.label9 = new System.Windows.Forms.Label();
            label2 = new System.Windows.Forms.Label();
            label8 = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.TurnChart)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.DriveChart)).BeginInit();
            this.SuspendLayout();
            // 
            // label2
            // 
            label2.AutoSize = true;
            label2.Location = new System.Drawing.Point(324, 14);
            label2.Name = "label2";
            label2.Size = new System.Drawing.Size(87, 17);
            label2.TabIndex = 5;
            label2.Text = "Turn_Speed";
            // 
            // label8
            // 
            label8.AutoSize = true;
            label8.Location = new System.Drawing.Point(469, 14);
            label8.Name = "label8";
            label8.Size = new System.Drawing.Size(89, 17);
            label8.TabIndex = 18;
            label8.Text = "Turn_Output";
            // 
            // UpdateInterface_Timer
            // 
            this.UpdateInterface_Timer.Tick += new System.EventHandler(this.UpdateInterface_Timer_Tick);
            // 
            // Turn_Input_textBox
            // 
            this.Turn_Input_textBox.Location = new System.Drawing.Point(182, 35);
            this.Turn_Input_textBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.Turn_Input_textBox.Name = "Turn_Input_textBox";
            this.Turn_Input_textBox.Size = new System.Drawing.Size(127, 22);
            this.Turn_Input_textBox.TabIndex = 6;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(179, 14);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(77, 17);
            this.label3.TabIndex = 7;
            this.label3.Text = "Turn_Input";
            // 
            // Turn_Speed_textBox
            // 
            this.Turn_Speed_textBox.Location = new System.Drawing.Point(327, 35);
            this.Turn_Speed_textBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.Turn_Speed_textBox.Name = "Turn_Speed_textBox";
            this.Turn_Speed_textBox.Size = new System.Drawing.Size(127, 22);
            this.Turn_Speed_textBox.TabIndex = 4;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(179, 456);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(80, 17);
            this.label1.TabIndex = 3;
            this.label1.Text = "Drive_Input";
            // 
            // Drive_Input_textBox
            // 
            this.Drive_Input_textBox.Location = new System.Drawing.Point(180, 477);
            this.Drive_Input_textBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.Drive_Input_textBox.Name = "Drive_Input_textBox";
            this.Drive_Input_textBox.Size = new System.Drawing.Size(127, 22);
            this.Drive_Input_textBox.TabIndex = 2;
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(10, 14);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(114, 17);
            this.label4.TabIndex = 11;
            this.label4.Text = "Select serial port";
            // 
            // Start_Button
            // 
            this.Start_Button.Location = new System.Drawing.Point(13, 141);
            this.Start_Button.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.Start_Button.Name = "Start_Button";
            this.Start_Button.Size = new System.Drawing.Size(128, 34);
            this.Start_Button.TabIndex = 1;
            this.Start_Button.Text = "Start";
            this.Start_Button.UseVisualStyleBackColor = true;
            this.Start_Button.Click += new System.EventHandler(this.Start_Button_Click);
            // 
            // PortcomboBox
            // 
            this.PortcomboBox.FormattingEnabled = true;
            this.PortcomboBox.Location = new System.Drawing.Point(13, 35);
            this.PortcomboBox.Margin = new System.Windows.Forms.Padding(4);
            this.PortcomboBox.Name = "PortcomboBox";
            this.PortcomboBox.Size = new System.Drawing.Size(124, 24);
            this.PortcomboBox.TabIndex = 12;
            // 
            // Incoming_Serial_textBox
            // 
            this.Incoming_Serial_textBox.Location = new System.Drawing.Point(182, 907);
            this.Incoming_Serial_textBox.Name = "Incoming_Serial_textBox";
            this.Incoming_Serial_textBox.Size = new System.Drawing.Size(1033, 22);
            this.Incoming_Serial_textBox.TabIndex = 13;
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(10, 910);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(80, 17);
            this.label5.TabIndex = 14;
            this.label5.Text = "Serial data:";
            // 
            // Drive_Speed_textBox
            // 
            this.Drive_Speed_textBox.Location = new System.Drawing.Point(325, 477);
            this.Drive_Speed_textBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.Drive_Speed_textBox.Name = "Drive_Speed_textBox";
            this.Drive_Speed_textBox.Size = new System.Drawing.Size(127, 22);
            this.Drive_Speed_textBox.TabIndex = 15;
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(324, 456);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(90, 17);
            this.label6.TabIndex = 16;
            this.label6.Text = "Drive_Speed";
            // 
            // Drive_Output_textBox
            // 
            this.Drive_Output_textBox.Location = new System.Drawing.Point(470, 477);
            this.Drive_Output_textBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.Drive_Output_textBox.Name = "Drive_Output_textBox";
            this.Drive_Output_textBox.Size = new System.Drawing.Size(127, 22);
            this.Drive_Output_textBox.TabIndex = 19;
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(469, 456);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(92, 17);
            this.label7.TabIndex = 20;
            this.label7.Text = "Drive_Output";
            // 
            // Turn_Output_textBox
            // 
            this.Turn_Output_textBox.Location = new System.Drawing.Point(472, 35);
            this.Turn_Output_textBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.Turn_Output_textBox.Name = "Turn_Output_textBox";
            this.Turn_Output_textBox.Size = new System.Drawing.Size(127, 22);
            this.Turn_Output_textBox.TabIndex = 17;
            // 
            // TurnChart
            // 
            chartArea1.AxisX.Title = "Samples [n]";
            chartArea1.AxisY.Maximum = 2D;
            chartArea1.AxisY.Minimum = -2D;
            chartArea1.AxisY.Title = "Rad/s";
            chartArea1.Name = "ChartArea1";
            this.TurnChart.ChartAreas.Add(chartArea1);
            legend1.Name = "Legend1";
            this.TurnChart.Legends.Add(legend1);
            this.TurnChart.Location = new System.Drawing.Point(182, 65);
            this.TurnChart.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.TurnChart.Name = "TurnChart";
            series1.BorderWidth = 2;
            series1.ChartArea = "ChartArea1";
            series1.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series1.Legend = "Legend1";
            series1.Name = "Turn Input";
            series2.BorderWidth = 2;
            series2.ChartArea = "ChartArea1";
            series2.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series2.Legend = "Legend1";
            series2.Name = "Turn Speed";
            this.TurnChart.Series.Add(series1);
            this.TurnChart.Series.Add(series2);
            this.TurnChart.Size = new System.Drawing.Size(1622, 375);
            this.TurnChart.TabIndex = 21;
            this.TurnChart.Text = "chart1";
            // 
            // DriveChart
            // 
            chartArea2.AxisX.Title = "Samples [n]";
            chartArea2.AxisY.Maximum = 1D;
            chartArea2.AxisY.Minimum = -1D;
            chartArea2.AxisY.Title = "m/s";
            chartArea2.Name = "ChartArea1";
            this.DriveChart.ChartAreas.Add(chartArea2);
            legend2.Name = "Legend1";
            this.DriveChart.Legends.Add(legend2);
            this.DriveChart.Location = new System.Drawing.Point(182, 511);
            this.DriveChart.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.DriveChart.Name = "DriveChart";
            series3.BorderWidth = 2;
            series3.ChartArea = "ChartArea1";
            series3.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series3.Legend = "Legend1";
            series3.Name = "Drive Input";
            series4.BorderWidth = 2;
            series4.ChartArea = "ChartArea1";
            series4.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series4.Legend = "Legend1";
            series4.Name = "Drive Speed";
            this.DriveChart.Series.Add(series3);
            this.DriveChart.Series.Add(series4);
            this.DriveChart.Size = new System.Drawing.Size(1622, 375);
            this.DriveChart.TabIndex = 22;
            this.DriveChart.Text = "chart1";
            // 
            // BaudcomboBox
            // 
            this.BaudcomboBox.FormattingEnabled = true;
            this.BaudcomboBox.Items.AddRange(new object[] {
            "300",
            "1200",
            "2400",
            "4800",
            "9600",
            "19200",
            "38400",
            "57600",
            "74880",
            "115200",
            "230400",
            "250000",
            "500000",
            "1000000",
            "2000000"});
            this.BaudcomboBox.Location = new System.Drawing.Point(13, 101);
            this.BaudcomboBox.Margin = new System.Windows.Forms.Padding(4);
            this.BaudcomboBox.Name = "BaudcomboBox";
            this.BaudcomboBox.Size = new System.Drawing.Size(124, 24);
            this.BaudcomboBox.TabIndex = 23;
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(10, 80);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(108, 17);
            this.label9.TabIndex = 24;
            this.label9.Text = "Select baudrate";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1816, 941);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.BaudcomboBox);
            this.Controls.Add(this.DriveChart);
            this.Controls.Add(this.TurnChart);
            this.Controls.Add(this.Drive_Output_textBox);
            this.Controls.Add(this.label7);
            this.Controls.Add(label8);
            this.Controls.Add(this.Turn_Output_textBox);
            this.Controls.Add(this.Drive_Speed_textBox);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.Incoming_Serial_textBox);
            this.Controls.Add(this.PortcomboBox);
            this.Controls.Add(this.Start_Button);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.Turn_Input_textBox);
            this.Controls.Add(this.Drive_Input_textBox);
            this.Controls.Add(label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.Turn_Speed_textBox);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.Name = "Form1";
            this.Text = "PID Viewer";
            ((System.ComponentModel.ISupportInitialize)(this.TurnChart)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.DriveChart)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion
        private System.Windows.Forms.Timer UpdateInterface_Timer;
        private System.Windows.Forms.TextBox Turn_Input_textBox;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox Turn_Speed_textBox;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox Drive_Input_textBox;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Button Start_Button;
        private System.Windows.Forms.ComboBox PortcomboBox;
        private System.Windows.Forms.TextBox Incoming_Serial_textBox;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TextBox Drive_Speed_textBox;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox Drive_Output_textBox;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox Turn_Output_textBox;
        private System.Windows.Forms.DataVisualization.Charting.Chart TurnChart;
        private System.Windows.Forms.DataVisualization.Charting.Chart DriveChart;
        private System.Windows.Forms.ComboBox BaudcomboBox;
        private System.Windows.Forms.Label label9;
    }
}

