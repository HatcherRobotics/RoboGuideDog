namespace DogImgClient
{
    partial class MainForm
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
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.pictureBox2 = new System.Windows.Forms.PictureBox();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.button4 = new System.Windows.Forms.Button();
            this.button3 = new System.Windows.Forms.Button();
            this.button2 = new System.Windows.Forms.Button();
            this.button1 = new System.Windows.Forms.Button();
            this.label9 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.rollTextBox = new System.Windows.Forms.TextBox();
            this.pitchTextBox = new System.Windows.Forms.TextBox();
            this.yawTextBox = new System.Windows.Forms.TextBox();
            this.footRaiseHeightTextBox = new System.Windows.Forms.TextBox();
            this.bodyHeightTextBox = new System.Windows.Forms.TextBox();
            this.rotateSpeedTextBox = new System.Windows.Forms.TextBox();
            this.sideSpeedTextBox = new System.Windows.Forms.TextBox();
            this.forwardSpeedTextBox = new System.Windows.Forms.TextBox();
            this.modeTextBox = new System.Windows.Forms.TextBox();
            this.stopTimer = new System.Windows.Forms.Timer(this.components);
            this.safityTestTimer = new System.Windows.Forms.Timer(this.components);
            this.button5 = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            // 
            // pictureBox1
            // 
            this.pictureBox1.BackgroundImageLayout = System.Windows.Forms.ImageLayout.None;
            this.pictureBox1.Location = new System.Drawing.Point(12, 12);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(1436, 732);
            this.pictureBox1.SizeMode = System.Windows.Forms.PictureBoxSizeMode.StretchImage;
            this.pictureBox1.TabIndex = 0;
            this.pictureBox1.TabStop = false;
            // 
            // pictureBox2
            // 
            this.pictureBox2.Location = new System.Drawing.Point(894, 424);
            this.pictureBox2.Name = "pictureBox2";
            this.pictureBox2.Size = new System.Drawing.Size(554, 320);
            this.pictureBox2.SizeMode = System.Windows.Forms.PictureBoxSizeMode.StretchImage;
            this.pictureBox2.TabIndex = 1;
            this.pictureBox2.TabStop = false;
            // 
            // timer1
            // 
            this.timer1.Interval = 2000;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.button5);
            this.groupBox1.Controls.Add(this.button4);
            this.groupBox1.Controls.Add(this.button3);
            this.groupBox1.Controls.Add(this.button2);
            this.groupBox1.Controls.Add(this.button1);
            this.groupBox1.Controls.Add(this.label9);
            this.groupBox1.Controls.Add(this.label8);
            this.groupBox1.Controls.Add(this.label7);
            this.groupBox1.Controls.Add(this.label6);
            this.groupBox1.Controls.Add(this.label5);
            this.groupBox1.Controls.Add(this.label4);
            this.groupBox1.Controls.Add(this.label3);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.rollTextBox);
            this.groupBox1.Controls.Add(this.pitchTextBox);
            this.groupBox1.Controls.Add(this.yawTextBox);
            this.groupBox1.Controls.Add(this.footRaiseHeightTextBox);
            this.groupBox1.Controls.Add(this.bodyHeightTextBox);
            this.groupBox1.Controls.Add(this.rotateSpeedTextBox);
            this.groupBox1.Controls.Add(this.sideSpeedTextBox);
            this.groupBox1.Controls.Add(this.forwardSpeedTextBox);
            this.groupBox1.Controls.Add(this.modeTextBox);
            this.groupBox1.Location = new System.Drawing.Point(1118, 12);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(330, 406);
            this.groupBox1.TabIndex = 2;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "调试";
            // 
            // button4
            // 
            this.button4.Location = new System.Drawing.Point(168, 303);
            this.button4.Name = "button4";
            this.button4.Size = new System.Drawing.Size(75, 23);
            this.button4.TabIndex = 21;
            this.button4.Text = "切换状态";
            this.button4.UseVisualStyleBackColor = true;
            this.button4.Click += new System.EventHandler(this.button4_Click);
            // 
            // button3
            // 
            this.button3.Location = new System.Drawing.Point(249, 361);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(75, 23);
            this.button3.TabIndex = 20;
            this.button3.Text = "视觉识别";
            this.button3.UseVisualStyleBackColor = true;
            this.button3.Click += new System.EventHandler(this.button3_Click);
            // 
            // button2
            // 
            this.button2.Location = new System.Drawing.Point(249, 332);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(75, 23);
            this.button2.TabIndex = 19;
            this.button2.Text = "强制停止";
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.button2_Click);
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(249, 303);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 18;
            this.button1.Text = "发送";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.cmdTextChanged);
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(6, 272);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(39, 15);
            this.label9.TabIndex = 17;
            this.label9.Text = "roll";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(6, 241);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(47, 15);
            this.label8.TabIndex = 16;
            this.label8.Text = "pitch";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(6, 210);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(31, 15);
            this.label7.TabIndex = 15;
            this.label7.Text = "yaw";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(6, 179);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(127, 15);
            this.label6.TabIndex = 14;
            this.label6.Text = "footRaiseHeight";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(6, 148);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(87, 15);
            this.label5.TabIndex = 13;
            this.label5.Text = "bodyHeight";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(6, 117);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(95, 15);
            this.label4.TabIndex = 12;
            this.label4.Text = "rotateSpeed";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(6, 86);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(79, 15);
            this.label3.TabIndex = 11;
            this.label3.Text = "sideSpeed";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(6, 55);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(103, 15);
            this.label2.TabIndex = 10;
            this.label2.Text = "forwardSpeed";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(6, 24);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(39, 15);
            this.label1.TabIndex = 9;
            this.label1.Text = "mode";
            // 
            // rollTextBox
            // 
            this.rollTextBox.Location = new System.Drawing.Point(138, 272);
            this.rollTextBox.Name = "rollTextBox";
            this.rollTextBox.Size = new System.Drawing.Size(186, 25);
            this.rollTextBox.TabIndex = 8;
            this.rollTextBox.TextChanged += new System.EventHandler(this.cmdTextChanged);
            // 
            // pitchTextBox
            // 
            this.pitchTextBox.Location = new System.Drawing.Point(138, 241);
            this.pitchTextBox.Name = "pitchTextBox";
            this.pitchTextBox.Size = new System.Drawing.Size(186, 25);
            this.pitchTextBox.TabIndex = 7;
            this.pitchTextBox.TextChanged += new System.EventHandler(this.cmdTextChanged);
            // 
            // yawTextBox
            // 
            this.yawTextBox.Location = new System.Drawing.Point(138, 210);
            this.yawTextBox.Name = "yawTextBox";
            this.yawTextBox.Size = new System.Drawing.Size(186, 25);
            this.yawTextBox.TabIndex = 6;
            this.yawTextBox.TextChanged += new System.EventHandler(this.cmdTextChanged);
            // 
            // footRaiseHeightTextBox
            // 
            this.footRaiseHeightTextBox.Location = new System.Drawing.Point(138, 179);
            this.footRaiseHeightTextBox.Name = "footRaiseHeightTextBox";
            this.footRaiseHeightTextBox.Size = new System.Drawing.Size(186, 25);
            this.footRaiseHeightTextBox.TabIndex = 5;
            this.footRaiseHeightTextBox.TextChanged += new System.EventHandler(this.cmdTextChanged);
            // 
            // bodyHeightTextBox
            // 
            this.bodyHeightTextBox.Location = new System.Drawing.Point(138, 148);
            this.bodyHeightTextBox.Name = "bodyHeightTextBox";
            this.bodyHeightTextBox.Size = new System.Drawing.Size(186, 25);
            this.bodyHeightTextBox.TabIndex = 4;
            this.bodyHeightTextBox.TextChanged += new System.EventHandler(this.cmdTextChanged);
            // 
            // rotateSpeedTextBox
            // 
            this.rotateSpeedTextBox.Location = new System.Drawing.Point(138, 117);
            this.rotateSpeedTextBox.Name = "rotateSpeedTextBox";
            this.rotateSpeedTextBox.Size = new System.Drawing.Size(186, 25);
            this.rotateSpeedTextBox.TabIndex = 3;
            this.rotateSpeedTextBox.TextChanged += new System.EventHandler(this.cmdTextChanged);
            // 
            // sideSpeedTextBox
            // 
            this.sideSpeedTextBox.Location = new System.Drawing.Point(138, 86);
            this.sideSpeedTextBox.Name = "sideSpeedTextBox";
            this.sideSpeedTextBox.Size = new System.Drawing.Size(186, 25);
            this.sideSpeedTextBox.TabIndex = 2;
            this.sideSpeedTextBox.TextChanged += new System.EventHandler(this.cmdTextChanged);
            // 
            // forwardSpeedTextBox
            // 
            this.forwardSpeedTextBox.Location = new System.Drawing.Point(138, 55);
            this.forwardSpeedTextBox.Name = "forwardSpeedTextBox";
            this.forwardSpeedTextBox.Size = new System.Drawing.Size(186, 25);
            this.forwardSpeedTextBox.TabIndex = 1;
            this.forwardSpeedTextBox.TextChanged += new System.EventHandler(this.cmdTextChanged);
            // 
            // modeTextBox
            // 
            this.modeTextBox.Location = new System.Drawing.Point(138, 24);
            this.modeTextBox.Name = "modeTextBox";
            this.modeTextBox.Size = new System.Drawing.Size(186, 25);
            this.modeTextBox.TabIndex = 0;
            this.modeTextBox.TextChanged += new System.EventHandler(this.cmdTextChanged);
            // 
            // stopTimer
            // 
            this.stopTimer.Enabled = true;
            this.stopTimer.Tick += new System.EventHandler(this.stopTimer_Tick);
            // 
            // safityTestTimer
            // 
            this.safityTestTimer.Enabled = true;
            this.safityTestTimer.Interval = 250;
            this.safityTestTimer.Tick += new System.EventHandler(this.safityTestTimer_Tick);
            // 
            // button5
            // 
            this.button5.Location = new System.Drawing.Point(168, 332);
            this.button5.Name = "button5";
            this.button5.Size = new System.Drawing.Size(75, 23);
            this.button5.TabIndex = 22;
            this.button5.Text = "button5";
            this.button5.UseVisualStyleBackColor = true;
            this.button5.Click += new System.EventHandler(this.button5_Click_1);
            // 
            // MainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 15F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1460, 756);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.pictureBox2);
            this.Controls.Add(this.pictureBox1);
            this.Name = "MainForm";
            this.Text = "MainForm";
            this.Load += new System.EventHandler(this.MainForm_Load);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.PictureBox pictureBox2;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.TextBox rollTextBox;
        private System.Windows.Forms.TextBox pitchTextBox;
        private System.Windows.Forms.TextBox yawTextBox;
        private System.Windows.Forms.TextBox footRaiseHeightTextBox;
        private System.Windows.Forms.TextBox bodyHeightTextBox;
        private System.Windows.Forms.TextBox rotateSpeedTextBox;
        private System.Windows.Forms.TextBox sideSpeedTextBox;
        private System.Windows.Forms.TextBox forwardSpeedTextBox;
        private System.Windows.Forms.TextBox modeTextBox;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Timer stopTimer;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.Button button3;
        private System.Windows.Forms.Timer safityTestTimer;
        private System.Windows.Forms.Button button4;
        private System.Windows.Forms.Button button5;
    }
}