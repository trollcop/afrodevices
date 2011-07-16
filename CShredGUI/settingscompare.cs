using System.Windows.Forms;
using System;
using System.IO;
using System.ComponentModel;
using System.Drawing;

public class settingscompare : Form
{
    // Fields
    private Button button1;
    private Button button2;
    private IContainer components;
    private OpenFileDialog openFileDialog1;
    private Panel panel1;
    private Panel panel2;
    private RichTextBox richTextBox1;
    private RichTextBox richTextBox2;
    private string[] settingsone = new string[0x22];
    private string[] varnames = new string[0x22];
    private int zahler;

    // Methods
    public settingscompare()
    {
        this.InitializeComponent();
    }

    public void Button1Click(object sender, EventArgs e)
    {
        this.openFileDialog1.InitialDirectory = Environment.CurrentDirectory;
        this.openFileDialog1.ShowDialog();
        string loadfrom = this.openFileDialog1.FileName;
        string Pfad = loadfrom;
        string Name = Path.GetFileName(Pfad);
        if (loadfrom != "")
        {
            string[] readtext = File.ReadAllLines(loadfrom);
            int VBt_i4L0 = readtext.Length;
            this.zahler = 1;
            while (this.zahler <= VBt_i4L0)
            {
                this.settingsone[this.zahler] = readtext[this.zahler - 1];
                this.zahler++;
            }
            this.richTextBox1.Clear();
            this.richTextBox1.Text = "Filename: " + Name + "\r\n";
            this.varnames[1] = "enable motors: ";
            this.varnames[2] = "roll gyro dir: ";
            this.varnames[3] = "nick gyro dir: ";
            this.varnames[4] = "yaw gyro dir: ";
            this.varnames[5] = "x acc dir: ";
            this.varnames[6] = "y acc dir: ";
            this.varnames[7] = "acro P: ";
            this.varnames[8] = "acro I: ";
            this.varnames[9] = "hover P: ";
            this.varnames[10] = "hover I: ";
            this.varnames[11] = "hover D: ";
            this.varnames[12] = "yaw P: ";
            this.varnames[13] = "yaw I: ";
            this.varnames[14] = "acc influence: ";
            this.varnames[15] = "x acc scale: ";
            this.varnames[0x10] = "y acc scale: ";
            this.varnames[0x11] = "acro stick sens: ";
            this.varnames[0x12] = "hover stick sens: ";
            this.varnames[0x13] = "yaw stick sens: ";
            this.varnames[20] = "expon. agility boost: ";
            this.varnames[0x15] = "min throttle: ";
            this.varnames[0x16] = "low voltage warning: ";
            this.varnames[0x17] = "x acc offset: ";
            this.varnames[0x18] = "y acc offset: ";
            this.varnames[0x19] = "throttle channel: ";
            this.varnames[0x1a] = "nick channel: ";
            this.varnames[0x1b] = "roll channel: ";
            this.varnames[0x1c] = "yaw channel: ";
            this.varnames[0x1d] = "acro D: ";
            this.varnames[30] = "hover D\x00b2: ";
            this.varnames[0x1f] = "not used";
            this.varnames[0x20] = "not used";
            this.varnames[0x21] = "not used";
            this.zahler = 1;
            do
            {
                this.richTextBox1.AppendText(this.varnames[this.zahler] + this.settingsone[this.zahler] + "\r\n");
                this.zahler++;
            }
            while (this.zahler <= 0x21);
        }
    }

    public void Button2Click(object sender, EventArgs e)
    {
        this.openFileDialog1.InitialDirectory = Environment.CurrentDirectory;
        this.openFileDialog1.ShowDialog();
        string loadfrom = this.openFileDialog1.FileName;
        string Pfad = loadfrom;
        string Name = Path.GetFileName(Pfad);
        if (loadfrom != "")
        {
            string[] readtext = File.ReadAllLines(loadfrom);
            int VBt_i4L0 = readtext.Length;
            this.zahler = 1;
            while (this.zahler <= VBt_i4L0)
            {
                this.settingsone[this.zahler] = readtext[this.zahler - 1];
                this.zahler++;
            }
            this.richTextBox2.Clear();
            this.richTextBox2.Text = "Filename: " + Name + "\r\n";
            this.varnames[1] = "enable motors: ";
            this.varnames[2] = "roll gyro dir: ";
            this.varnames[3] = "nick gyro dir: ";
            this.varnames[4] = "yaw gyro dir: ";
            this.varnames[5] = "x acc dir: ";
            this.varnames[6] = "y acc dir: ";
            this.varnames[7] = "acro P: ";
            this.varnames[8] = "acro I: ";
            this.varnames[9] = "hover P: ";
            this.varnames[10] = "hover I: ";
            this.varnames[11] = "hover D: ";
            this.varnames[12] = "yaw P: ";
            this.varnames[13] = "yaw I: ";
            this.varnames[14] = "acc influence: ";
            this.varnames[15] = "x acc scale: ";
            this.varnames[0x10] = "y acc scale: ";
            this.varnames[0x11] = "acro stick sens: ";
            this.varnames[0x12] = "hover stick sens: ";
            this.varnames[0x13] = "yaw stick sens: ";
            this.varnames[20] = "expon. agility boost: ";
            this.varnames[0x15] = "min throttle: ";
            this.varnames[0x16] = "low voltage warning: ";
            this.varnames[0x17] = "x acc offset: ";
            this.varnames[0x18] = "y acc offset: ";
            this.varnames[0x19] = "throttle channel: ";
            this.varnames[0x1a] = "nick channel: ";
            this.varnames[0x1b] = "roll channel: ";
            this.varnames[0x1c] = "yaw channel: ";
            this.varnames[0x1d] = "acro D: ";
            this.varnames[30] = "hover D\x00b2: ";
            this.varnames[0x1f] = "not used";
            this.varnames[0x20] = "not used";
            this.varnames[0x21] = "not used";
            this.zahler = 1;
            do
            {
                this.richTextBox2.AppendText(this.varnames[this.zahler] + this.settingsone[this.zahler] + "\r\n");
                this.zahler++;
            }
            while (this.zahler <= 0x21);
        }
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing && (this.components != null))
        {
            this.components.Dispose();
        }
        base.Dispose(disposing);
    }

    private void InitializeComponent()
    {
        this.button1 = new Button();
        this.openFileDialog1 = new OpenFileDialog();
        this.button2 = new Button();
        this.panel1 = new Panel();
        this.richTextBox1 = new RichTextBox();
        this.panel2 = new Panel();
        this.richTextBox2 = new RichTextBox();
        this.panel1.SuspendLayout();
        this.panel2.SuspendLayout();
        this.SuspendLayout();
        this.button1.Location = new Point(3, 0x1cd);
        this.button1.Name = "button1";
        this.button1.Size = new Size(0x62, 0x17);
        this.button1.TabIndex = 0;
        this.button1.Text = "Load Settings-1";
        this.button1.UseVisualStyleBackColor = true;
        this.button1.Click += new EventHandler(this.Button1Click);
        this.openFileDialog1.FileName = "openFileDialog1";
        this.button2.Location = new Point(5, 0x1cd);
        this.button2.Name = "button2";
        this.button2.Size = new Size(0x62, 0x17);
        this.button2.TabIndex = 1;
        this.button2.Text = "Load Settings-2";
        this.button2.UseVisualStyleBackColor = true;
        this.button2.Click += new EventHandler(this.Button2Click);
        this.panel1.BorderStyle = BorderStyle.Fixed3D;
        this.panel1.Controls.Add(this.richTextBox1);
        this.panel1.Controls.Add(this.button1);
        this.panel1.Location = new Point(12, 13);
        this.panel1.Name = "panel1";
        this.panel1.Size = new Size(0xc5, 0x1eb);
        this.panel1.TabIndex = 3;
        this.richTextBox1.Location = new Point(3, 3);
        this.richTextBox1.Name = "richTextBox1";
        this.richTextBox1.ReadOnly = true;
        // this.richTextBox1.RightToLeft = RightToLeft.Yes;
        this.richTextBox1.Size = new Size(0xb3, 0x1c4);
        this.richTextBox1.TabIndex = 1;
        this.richTextBox1.Text = "";
        this.richTextBox1.TextChanged += new EventHandler(this.RichTextBox1TextChanged);
        this.panel2.BorderStyle = BorderStyle.Fixed3D;
        this.panel2.Controls.Add(this.richTextBox2);
        this.panel2.Controls.Add(this.button2);
        this.panel2.Location = new Point(0xd7, 13);
        this.panel2.Name = "panel2";
        this.panel2.Size = new Size(0xc4, 0x1eb);
        this.panel2.TabIndex = 4;
        this.richTextBox2.Location = new Point(5, 3);
        this.richTextBox2.Name = "richTextBox2";
        this.richTextBox2.ReadOnly = true;
        // this.richTextBox2.RightToLeft = RightToLeft.Yes;
        this.richTextBox2.Size = new Size(0xb3, 0x1c4);
        this.richTextBox2.TabIndex = 2;
        this.richTextBox2.Text = "";
        this.richTextBox2.TextChanged += new EventHandler(this.RichTextBox2TextChanged);
        SizeF VBt_structS2 = new SizeF(6f, 13f);
        this.AutoScaleDimensions = VBt_structS2;
        // this.AutoScaleMode = AutoScaleMode.Font;
        this.ClientSize = new Size(0x1a2, 0x204);
        this.Controls.Add(this.panel2);
        this.Controls.Add(this.panel1);
        this.Name = "settingscompare";
        this.Text = "settingscompare";
        this.panel1.ResumeLayout(false);
        this.panel2.ResumeLayout(false);
        this.ResumeLayout(false);
    }

    public void Panel1Paint(object sender, PaintEventArgs e)
    {
    }

    public void RichTextBox1TextChanged(object sender, EventArgs e)
    {
    }

    public void RichTextBox2TextChanged(object sender, EventArgs e)
    {
    }
}
