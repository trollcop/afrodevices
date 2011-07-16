using System;
using System.Windows.Forms;
using System.ComponentModel;
using System.Diagnostics;
using System.IO.Ports;
using XComponent.SliderBar;
using System.Collections;
using System.Drawing;
using System.Runtime.CompilerServices;
using Microsoft.VisualBasic;
using Microsoft.VisualBasic.CompilerServices;
using System.IO;
using System.Threading;

public struct Settings
{
    public bool MotorsEnabled;  // [0] Motors are enabled or not


}

public class MainForm : Form
{
    private Settings MainSettings;

    // Fields
    private int accinfluence;
    private int acrod;
    private int acroi;
    private int acrop;
    private int bytesRead;
    private TextBox avrdudeout;
    private GroupBox Battery;
    private Button btnRefresh;
    private Label bytecounter;
    private CheckBox checkBox1;
    private CheckBox checkBox2;
    private CheckBox checkBox3;
    private CheckBox checkBoxB;
    private CheckBox checkBoxB2;
    private CheckBox checkBoxC;
    private CheckBox checkBoxC2;
    private CheckBox checkBoxD;
    private CheckBox checkBoxD2;
    private CheckBox checkBoxe;
    private CheckBox checkBoxe2;
    private CheckBox checkBoxf;
    private CheckBox checkBoxf2;
    private CheckBox checkBoxT;
    private CheckBox checkBoxT2;
    private Button clearRxD;
    private Button Compare_Settings;
    private IContainer components;
    private ComboBox comport;
    private Button connect;
    private CheckBox deactivatemotors;
    private int errorcounter;
    private bool errorintx;
    private Label errorlabel;
    private Button flash;
    private bool flashok;
    private string foundstring;
    private GroupBox groupBox1;
    private GroupBox groupBox10;
    private GroupBox groupBox11;
    private GroupBox groupBox12;
    private GroupBox groupBox13;
    private GroupBox groupBox2;
    private GroupBox groupBox3;
    private GroupBox groupBox4;
    private GroupBox groupBox5;
    private GroupBox groupBox6;
    private GroupBox groupBox7;
    private GroupBox groupBox8;
    private GroupBox groupBox9;
    private int hoverd;
    private int hoverdd;
    private int hoveri;
    private int hoverp;
    private int IsAbyte;
    private bool isconnected;
    private int k;
    private Label label1;
    private Label label10;
    private Label label11;
    private Label label12;
    private Label label13;
    private Label label14;
    private Label label15;
    private Label label16;
    private Label label17;
    private Label label18;
    private Label label19;
    private Label label2;
    private Label label20;
    private Label label21;
    private Label label22;
    private Label label23;
    private Label label24;
    private Label label25;
    private Label label26;
    private Label label3;
    private Label label4;
    private Label label5;
    private Label label6;
    private Label label7;
    private Label label8;
    private Label label9;
    private Label labelG;
    private Label labelH;
    private Label labeli;
    private Label labelj;
    private Label labelk;
    private Label labell;
    private Label labelm;
    private Label labeln;
    private Label labelo;
    private Label labelok;
    private Label labelp;
    private Label labelq;
    private Label labelr;
    private Label labels;
    private Label labelu;
    private Label labelv;
    private Label labelvolts;
    private Label labelw;
    private Label labelx;
    private Label labely;
    private Label labelz;
    private int lfacro;
    private int lfboost;
    private int lfhover;
    private int lfyaw;
    private LinkLabel linkLabel1;
    private LinkLabel linkLabel2;
    private int minthrottle;
    private ComboBox Nickbox;
    private byte nickchannel;
    private int nickgyrodir;
    private int nrtextboxes;
    private bool okay;
    private OpenFileDialog openFileDialog1;
    private byte[] outvar = new byte[0x22];
    private Process p = new Process();
    private Panel panel1;
    private Panel panel2;
    private Panel panelRollCenter;
    private Panel panel4;
    private PictureBox pictureBox1;
    private PictureBox pictureBox2;
    private System.Windows.Forms.Timer pingshrediquette;
    private ProgressBar progressBar1;
    private ProgressBar progressBar2;
    private ProgressBar pbVoltage;
    private ProgressBar progressBar4;
    private bool readsens;
    private Button readsettings;
    private string readvar;
    private Button resetmC;
    private ComboBox rollbox;
    private byte rollchannel;
    private int rollgyrodir;
    private GroupBox RxD;
    private string s3;
    private SaveFileDialog saveFileDialog1;
    private Label searchlabel;
    private byte[] sensors = new byte[14];
    private byte[] rcdata = new byte[24];
    private SerialPort serialPort;
    private CheckBox showavrout;
    private CheckBox showdebug;
    private Label staic59;
    private System.Windows.Forms.Timer startupdelay;
    private Label static10;
    private Label static11;
    private Label static12;
    private Label static13;
    private Label static14;
    private Label static15;
    private Label static16;
    private Label static17;
    private Label static18;
    private Label static19;
    private Label static2;
    private Label static20;
    private Label static21;
    private Label static22;
    private Label static23;
    private Label static24;
    private Label static25;
    private Label static26;
    private Label static27;
    private Label static28;
    private Label static29;
    private Label static3;
    private Label static30;
    private Label static31;
    private Label static32;
    private Label static33;
    private Label static34;
    private Label static35;
    private Label static36;
    private Label static37;
    private Label static38;
    private Label static39;
    private Label static4;
    private Label static40;
    private Label static41;
    private Label static42;
    private Label static43;
    private Label static44;
    private Label static45;
    private Label static46;
    private Label static47;
    private Label static48;
    private Label static49;
    private Label static5;
    private Label static50;
    private Label static51;
    private Label static52;
    private Label static53;
    private Label static54;
    private Label static55;
    private Label static56;
    private Label static57;
    private Label static58;
    private Label static6;
    private Label static60;
    private Label static61;
    private Label static63;
    private Label static64;
    private Label static65;
    private Label static66;
    private Label static67;
    private Label static68;
    private Label static7;
    private Label static9;
    private Label static99;
    private StatusStrip statusStrip1;
    private TabControl tabControl;
    private TabPage tabPage1;
    private TabPage tabPage2;
    private TabPage tabPage3;
    private TabPage tabPage4;
    private TabPage tabPage5;
    private TabPage tabPage6;
    private TextBox textBoxG;
    private TextBox textBoxH;
    private TextBox textBoxi;
    private TextBox textBoxj;
    private TextBox textBoxk;
    private TextBox textBoxl;
    private TextBox textBoxm;
    private TextBox textBoxn;
    private TextBox textBoxo;
    private TextBox textBoxp;
    private TextBox textBoxQ;
    private TextBox textBoxR;
    private TextBox textBoxS;
    private TextBox textBoxu;
    private TextBox textBoxv;
    private TextBox textBoxw;
    private TextBox textBoxx;
    private TextBox textBoxy;
    private TextBox textBoxz;
    private TextBox textRxD;
    private ComboBox throttlebox;
    private byte throttlechannel;
    private System.Windows.Forms.Timer timer1;
    private System.Windows.Forms.Timer timersens;
    private ToolStripStatusLabel toolStatusLabel;
    private ToolTip toolTip1;
    private MACTrackBar TrackBar1;
    private MACTrackBar TrackBar10;
    private MACTrackBar TrackBar11;
    private MACTrackBar TrackBar12;
    private TrackBar trackBar13;
    private TrackBar trackBar14;
    private TrackBar trackBar15;
    private MACTrackBar TrackBar2;
    private MACTrackBar TrackBar3;
    private MACTrackBar TrackBar4;
    private MACTrackBar TrackBar5;
    private MACTrackBar TrackBar6;
    private MACTrackBar TrackBar7;
    private MACTrackBar TrackBar8;
    private MACTrackBar TrackBar9;
    private byte[] variable = new byte[0x22];
    private int voltage;
    private Label voltagelevel;
    private int voltsfromcopter;
    private Button writeall;
    private Button writesettings;
    private int xacc_offset;
    private int xaccdir;
    private int xaccscale;
    private int yacc_offset;
    private int yaccdir;
    private int yaccscale;
    private ComboBox yawbox;
    private byte yawchannel;
    private byte switchchannel;
    private int yawgyrodir;
    private int yawi;
    private GroupBox groupBox14;
    private ComboBox switchbox;
    private MACTrackBar tbch1;
    private MACTrackBar tbch2;
    private MACTrackBar tbch4;
    private MACTrackBar tbch3;
    private MACTrackBar tbch5;
    private MACTrackBar tbch6;
    private MACTrackBar tbch7;
    private MACTrackBar tbch8;
    private Label label34;
    private Label label33;
    private Label label32;
    private Label label31;
    private Label label30;
    private Label label29;
    private Label label28;
    private Label label27;
    private GroupBox groupBox15;
    private TriGUI_v11.BitMask led1mask;
    private Label label36;
    private Label label35;
    private TriGUI_v11.BitMask led2mask;
    private Label labelled2;
    private Label labelled1;
    private Label label37;
    private Label label38;
    private Label label39;
    private Label label40;
    private TriGUI_v11.BitMask emergmask;
    private Label label41;
    private Label lbch1;
    private Label lbch8;
    private Label lbch7;
    private Label lbch6;
    private Label lbch5;
    private Label lbch4;
    private Label lbch3;
    private Label lbch2;
    private int yawp;

    // Methods
    public MainForm()
    {
        this.InitializeComponent();
    }

    public void Button1Click(object sender, EventArgs e)
    {
        this.startupdelay.Enabled = true;
    }

    public void Button2Click(object sender, EventArgs e)
    {
        // TODO FIX
        Form scom = new settingscompare();
        scom.Show();
        // MyProject.Forms.settingscompare.Show();
    }

    public void checkbeforeupload()
    {
        IEnumerator VBt_refL0 = null;
        IEnumerator VBt_refL1 = null;
        IEnumerator VBt_refL2 = null;
        IEnumerator VBt_refL3 = null;
        IEnumerator VBt_refL4 = null;
        IEnumerator VBt_refL5 = null;
        if (Conversions.ToBoolean(Operators.OrObject(Operators.OrObject(Operators.OrObject(((this.throttlebox.SelectedIndex == this.Nickbox.SelectedIndex) | (this.throttlebox.SelectedIndex == this.rollbox.SelectedIndex)) | (this.throttlebox.SelectedIndex == this.yawbox.SelectedIndex), Operators.CompareObjectEqual(this.Nickbox.SelectedItem, this.rollbox.SelectedItem, false)), Operators.CompareObjectEqual(this.Nickbox.SelectedItem, this.yawbox.SelectedItem, false)), Operators.CompareObjectEqual(this.rollbox.SelectedItem, this.yawbox.SelectedItem, false)))) {
            this.errorintx = true;
            this.throttlebox.BackColor = Color.LightSalmon;
            this.rollbox.BackColor = Color.LightSalmon;
            this.Nickbox.BackColor = Color.LightSalmon;
            this.yawbox.BackColor = Color.LightSalmon;
        } else {
            this.errorintx = false;
            this.throttlebox.BackColor = Color.White;
            this.rollbox.BackColor = Color.White;
            this.Nickbox.BackColor = Color.White;
            this.yawbox.BackColor = Color.White;
        }
        try {
            VBt_refL0 = this.groupBox2.Controls.GetEnumerator();
            while (VBt_refL0.MoveNext()) {
                Control ctl = (Control)VBt_refL0.Current;
                if (ctl is TextBox) {
                    this.nrtextboxes++;
                    if (((TextBox)ctl).BackColor != Color.LightSalmon) {
                        this.IsAbyte++;
                    }
                }
            }
        } finally {
            if (VBt_refL0 is IDisposable) {
                (VBt_refL0 as IDisposable).Dispose();
            }
        }
        try {
            VBt_refL1 = this.groupBox1.Controls.GetEnumerator();
            while (VBt_refL1.MoveNext()) {
                Control ctl = (Control)VBt_refL1.Current;
                if (ctl is TextBox) {
                    this.nrtextboxes++;
                    if (((TextBox)ctl).BackColor != Color.LightSalmon) {
                        this.IsAbyte++;
                    }
                }
            }
        } finally {
            if (VBt_refL1 is IDisposable) {
                (VBt_refL1 as IDisposable).Dispose();
            }
        }
        try {
            VBt_refL2 = this.groupBox3.Controls.GetEnumerator();
            while (VBt_refL2.MoveNext()) {
                Control ctl = (Control)VBt_refL2.Current;
                if (ctl is TextBox) {
                    this.nrtextboxes++;
                    if (((TextBox)ctl).BackColor != Color.LightSalmon) {
                        this.IsAbyte++;
                    }
                }
            }
        } finally {
            if (VBt_refL2 is IDisposable) {
                (VBt_refL2 as IDisposable).Dispose();
            }
        }
        try {
            VBt_refL3 = this.groupBox5.Controls.GetEnumerator();
            while (VBt_refL3.MoveNext()) {
                Control ctl = (Control)VBt_refL3.Current;
                if (ctl is TextBox) {
                    this.nrtextboxes++;
                    if (((TextBox)ctl).BackColor != Color.LightSalmon) {
                        this.IsAbyte++;
                    }
                }
            }
        } finally {
            if (VBt_refL3 is IDisposable) {
                (VBt_refL3 as IDisposable).Dispose();
            }
        }
        try {
            VBt_refL4 = this.groupBox6.Controls.GetEnumerator();
            while (VBt_refL4.MoveNext()) {
                Control ctl = (Control)VBt_refL4.Current;
                if (ctl is TextBox) {
                    this.nrtextboxes++;
                    if (((TextBox)ctl).BackColor != Color.LightSalmon) {
                        this.IsAbyte++;
                    }
                }
            }
        } finally {
            if (VBt_refL4 is IDisposable) {
                (VBt_refL4 as IDisposable).Dispose();
            }
        }
        try {
            VBt_refL5 = this.groupBox7.Controls.GetEnumerator();
            while (VBt_refL5.MoveNext()) {
                Control ctl = (Control)VBt_refL5.Current;
                if (ctl is TextBox) {
                    this.nrtextboxes++;
                    if (((TextBox)ctl).BackColor != Color.LightSalmon) {
                        this.IsAbyte++;
                    }
                }
            }
        } finally {
            if (VBt_refL5 is IDisposable) {
                (VBt_refL5 as IDisposable).Dispose();
            }
        }
    }

    public void CheckBox1CheckedChanged(object sender, EventArgs e)
    {
        if (this.checkBox1.Checked) {
            this.textBoxi.Enabled = false;
            this.textBoxj.Enabled = false;
            this.textBoxk.Enabled = false;
            this.textBoxz.Enabled = false;
            this.trackBar13.Enabled = true;
        } else {
            this.textBoxi.Enabled = true;
            this.textBoxj.Enabled = true;
            this.textBoxk.Enabled = true;
            this.textBoxz.Enabled = true;
            this.trackBar13.Enabled = false;
        }
    }

    public void CheckBox2CheckedChanged(object sender, EventArgs e)
    {
        if (this.checkBox2.Checked) {
            this.textBoxG.Enabled = false;
            this.textBoxH.Enabled = false;
            this.textBoxy.Enabled = false;
            this.trackBar14.Enabled = true;
        } else {
            this.textBoxG.Enabled = true;
            this.textBoxH.Enabled = true;
            this.textBoxy.Enabled = true;
            this.trackBar14.Enabled = false;
        }
    }

    public void CheckBox3CheckedChanged(object sender, EventArgs e)
    {
        if (this.checkBox3.Checked) {
            this.textBoxl.Enabled = false;
            this.textBoxm.Enabled = false;
            this.trackBar15.Enabled = true;
        } else {
            this.textBoxl.Enabled = true;
            this.textBoxm.Enabled = true;
            this.trackBar15.Enabled = false;
        }
    }

    public void checkinput(object sender)
    {
        if (Operators.ConditionalCompareObjectEqual(NewLateBinding.LateGet(sender, null, "text", new object[0], null, null, null), "", false)) {
            NewLateBinding.LateSet(sender, null, "BackColor", new object[] { Color.LightSalmon }, null, null);
        }
        try {
            Convert.ToByte(RuntimeHelpers.GetObjectValue(NewLateBinding.LateGet(sender, null, "Text", new object[0], null, null, null)));
            NewLateBinding.LateSet(sender, null, "BackColor", new object[] { Color.White }, null, null);
        } catch (Exception exception1) {
            ProjectData.SetProjectError(exception1);
            NewLateBinding.LateSet(sender, null, "BackColor", new object[] { Color.LightSalmon }, null, null);
            ProjectData.ClearProjectError();
        }
    }

    public void ClearRxDClick(object sender, EventArgs e)
    {
        this.textRxD.Clear();
    }

    public void ComportSelectedIndexChanged(object sender, EventArgs e)
    {
        this.label13.Text = Conversions.ToString(Operators.ConcatenateObject("Using ", this.comport.SelectedItem));
    }

    public void ConnectClick(object sender, EventArgs e)
    {
        this.isconnected = false;
        this.progressBar2.Value = 0;
        this.labelok.Visible = false;
        try {
            btnRefresh.Enabled = false;
            comport.Enabled = false;
            if (serialPort.IsOpen)
                serialPort.Close();

            serialPort.PortName = comport.SelectedItem.ToString();
            serialPort.Open();
            if (serialPort.IsOpen) {
                errorcounter = 0;
                pingshrediquette.Enabled = true;
                searchlabel.Visible = true;
                timer1.Enabled = true;
                toolStatusLabel.Text = "Connected (" + this.serialPort.PortName + ", @" + serialPort.BaudRate.ToString() + " baud) - Shrediquette not found (yet)";
                resetmC.Enabled = true;
            }
        } catch (Exception) {
            MessageBox.Show(comport.Text + " could not be opened.\nTry a different COM port.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            toolStatusLabel.Text = "Not connected";
            btnRefresh.Enabled = true;
            comport.Enabled = true;
        }
    }

    public void DeactivatemotorsClick(object sender, EventArgs e)
    {
        if (!deactivatemotors.Checked) {
            if (MessageBox.Show("Before enabling the motors, you HAVE to make sure that your transmitter is set up correctly. Check this by looking at 'realtime data'. Also make sure that RX/TX channel assignment is correct.\n\r\n\rThe throttle channel and switch channel are the most important ones.\n\r\n\rIf your settings aren't correct, you will NOT be able to reconnect to the GUI!\n\r\n\rEnable motors?", "Sure?", MessageBoxButtons.YesNo) == DialogResult.Yes) {
                deactivatemotors.Checked = false;
            } else {
                deactivatemotors.Checked = true;
            }
        }
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing && (this.components != null)) {
            this.components.Dispose();
        }
        base.Dispose(disposing);
    }

    public void FlashClick(object sender, EventArgs e)
    {
        if (Interaction.MsgBox("This should only be done if a new, improved firmware was released. All parameter settings might be lost.\n\r\n\rFlash firmware?", MsgBoxStyle.YesNo, "Sure?") == MsgBoxResult.Yes) {
            if (serialPort.IsOpen) {
                if (!this.isconnected) {
                    MessageBox.Show("Shrediquette was not found. This might be the case because there is no firmware on the controller yet. If this is the case, you have to press the reset button on the controller at the same time you click \"Ok\" in this message box.\n\rIf you have an outdated firmware on your tricopter, you don't have to press the reset button.", "Notice", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
                if (!File.Exists(Environment.CurrentDirectory + @"\Tricopter_m328p_11.hex")) {
                    Interaction.MsgBox("A firmware file could not be found in the current directory.\n\r" + Environment.CurrentDirectory + @"\Tricopter_m328p_11.hex not found.", MsgBoxStyle.OkOnly, "Notice");
                } else {
                    this.avrdudeout.Clear();
                    this.progressBar4.Value = this.progressBar4.Minimum;
                    this.progressBar4.PerformStep();
                    this.serialPort.Write("reset!\r");
                    Thread.Sleep(100);
                    this.resetmCClick(RuntimeHelpers.GetObjectValue(sender), e);
                    Thread.Sleep(100);
                    Application.DoEvents();
                    this.flash.Enabled = false;
                    this.flash.Text = "Please wait...";
                    Application.DoEvents();
                    this.p.StartInfo.FileName = "avrdude.exe";
                    this.p.StartInfo.Arguments = Conversions.ToString(Operators.ConcatenateObject(Operators.ConcatenateObject("-e -p m328p -P ", this.comport.SelectedItem), " -U flash:w:Tricopter_m328p_11.hex -c stk500v1 -b 57600"));
                    this.p.StartInfo.UseShellExecute = false;
                    this.p.StartInfo.RedirectStandardOutput = true;
                    this.p.StartInfo.CreateNoWindow = true;
                    this.p.StartInfo.RedirectStandardError = true;
                    this.p.Start();
                    this.avrdudeout.AppendText("Starting avrdude. Please wait...\r\n");
                    do {
                        try {
                            this.s3 = this.p.StandardError.ReadLine();
                            this.avrdudeout.AppendText(this.s3 + "\r\n");
                            if (this.s3.Contains("AVR device initialized")) {
                                this.progressBar4.PerformStep();
                                Application.DoEvents();
                            }
                            if (this.s3.Contains("bytes of flash written")) {
                                this.progressBar4.PerformStep();
                                Application.DoEvents();
                            }
                            if (this.s3.Contains("verifying ...")) {
                                this.progressBar4.PerformStep();
                                Application.DoEvents();
                            }
                            if (this.s3.Contains("Fuses OK")) {
                                this.progressBar4.PerformStep();
                                this.flashok = true;
                                Application.DoEvents();
                            }
                            if (this.s3.Contains("protocol error")) {
                                this.flashok = false;
                            }
                        } catch (Exception exception1) {
                            ProjectData.SetProjectError(exception1);
                            this.s3 = "";
                            ProjectData.ClearProjectError();
                        }
                        if (this.p.HasExited) {
                            this.okay = true;
                        }
                    }
                    while (!this.okay);
                    this.flash.Enabled = true;
                    this.flash.Text = "Flash new firmware";
                    if (this.flashok) {
                        this.progressBar4.Value = this.progressBar4.Maximum;
                        Interaction.MsgBox("New firmware was flashed successfully!\r\n\r\nThe first thing you have to do now is to upload the parameters 'defaults.shr'!", MsgBoxStyle.OkOnly, "Notice");
                    } else if (!this.flashok) {
                        this.progressBar4.Value = this.progressBar4.Minimum;
                        Interaction.MsgBox("There was an error in communication.\r\n\r\nCheck your connection and check the output window.", MsgBoxStyle.OkOnly, "Error");
                    }
                }
            } else {
                Interaction.MsgBox("You have to connect to Shrediquette first. Go to the tab \"Connection\" and press connect.", MsgBoxStyle.OkOnly, "Notice");
            }
        }
        this.flashok = false;
        this.okay = false;
    }

    public void FormShown(object sender, EventArgs e)
    {
        this.startupdelay.Enabled = true;
    }

    public void getparams()
    {
        serialPort.Write("cr!\r");
    }

    private void InitializeComponent()
    {
        this.components = new System.ComponentModel.Container();
        System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainForm));
        System.Windows.Forms.Label label42;
        this.tabControl = new System.Windows.Forms.TabControl();
        this.tabPage1 = new System.Windows.Forms.TabPage();
        this.showdebug = new System.Windows.Forms.CheckBox();
        this.Compare_Settings = new System.Windows.Forms.Button();
        this.RxD = new System.Windows.Forms.GroupBox();
        this.static48 = new System.Windows.Forms.Label();
        this.bytecounter = new System.Windows.Forms.Label();
        this.textRxD = new System.Windows.Forms.TextBox();
        this.clearRxD = new System.Windows.Forms.Button();
        this.linkLabel1 = new System.Windows.Forms.LinkLabel();
        this.btnRefresh = new System.Windows.Forms.Button();
        this.readsettings = new System.Windows.Forms.Button();
        this.writesettings = new System.Windows.Forms.Button();
        this.labelok = new System.Windows.Forms.Label();
        this.writeall = new System.Windows.Forms.Button();
        this.resetmC = new System.Windows.Forms.Button();
        this.comport = new System.Windows.Forms.ComboBox();
        this.connect = new System.Windows.Forms.Button();
        this.searchlabel = new System.Windows.Forms.Label();
        this.progressBar2 = new System.Windows.Forms.ProgressBar();
        this.pictureBox2 = new System.Windows.Forms.PictureBox();
        this.tabPage2 = new System.Windows.Forms.TabPage();
        this.groupBox14 = new System.Windows.Forms.GroupBox();
        this.label34 = new System.Windows.Forms.Label();
        this.label33 = new System.Windows.Forms.Label();
        this.label32 = new System.Windows.Forms.Label();
        this.label31 = new System.Windows.Forms.Label();
        this.label30 = new System.Windows.Forms.Label();
        this.label29 = new System.Windows.Forms.Label();
        this.label28 = new System.Windows.Forms.Label();
        this.label27 = new System.Windows.Forms.Label();
        this.tbch8 = new XComponent.SliderBar.MACTrackBar();
        this.tbch7 = new XComponent.SliderBar.MACTrackBar();
        this.tbch6 = new XComponent.SliderBar.MACTrackBar();
        this.tbch5 = new XComponent.SliderBar.MACTrackBar();
        this.tbch4 = new XComponent.SliderBar.MACTrackBar();
        this.tbch3 = new XComponent.SliderBar.MACTrackBar();
        this.tbch2 = new XComponent.SliderBar.MACTrackBar();
        this.tbch1 = new XComponent.SliderBar.MACTrackBar();
        this.groupBox2 = new System.Windows.Forms.GroupBox();
        this.switchbox = new System.Windows.Forms.ComboBox();
        this.deactivatemotors = new System.Windows.Forms.CheckBox();
        this.static67 = new System.Windows.Forms.Label();
        this.static68 = new System.Windows.Forms.Label();
        this.static66 = new System.Windows.Forms.Label();
        this.static65 = new System.Windows.Forms.Label();
        this.static64 = new System.Windows.Forms.Label();
        this.static63 = new System.Windows.Forms.Label();
        this.yawbox = new System.Windows.Forms.ComboBox();
        this.rollbox = new System.Windows.Forms.ComboBox();
        this.Nickbox = new System.Windows.Forms.ComboBox();
        this.throttlebox = new System.Windows.Forms.ComboBox();
        this.textBoxu = new System.Windows.Forms.TextBox();
        this.labelu = new System.Windows.Forms.Label();
        this.static40 = new System.Windows.Forms.Label();
        this.textBoxS = new System.Windows.Forms.TextBox();
        this.textBoxR = new System.Windows.Forms.TextBox();
        this.textBoxQ = new System.Windows.Forms.TextBox();
        this.checkBoxT = new System.Windows.Forms.CheckBox();
        this.labels = new System.Windows.Forms.Label();
        this.labelr = new System.Windows.Forms.Label();
        this.labelq = new System.Windows.Forms.Label();
        this.static15 = new System.Windows.Forms.Label();
        this.static13 = new System.Windows.Forms.Label();
        this.static14 = new System.Windows.Forms.Label();
        this.static12 = new System.Windows.Forms.Label();
        this.static11 = new System.Windows.Forms.Label();
        this.checkBoxT2 = new System.Windows.Forms.CheckBox();
        this.static9 = new System.Windows.Forms.Label();
        this.static10 = new System.Windows.Forms.Label();
        this.tabPage3 = new System.Windows.Forms.TabPage();
        this.groupBox6 = new System.Windows.Forms.GroupBox();
        this.textBoxx = new System.Windows.Forms.TextBox();
        this.textBoxw = new System.Windows.Forms.TextBox();
        this.labelx = new System.Windows.Forms.Label();
        this.labelw = new System.Windows.Forms.Label();
        this.static46 = new System.Windows.Forms.Label();
        this.static45 = new System.Windows.Forms.Label();
        this.textBoxp = new System.Windows.Forms.TextBox();
        this.textBoxo = new System.Windows.Forms.TextBox();
        this.textBoxn = new System.Windows.Forms.TextBox();
        this.labelp = new System.Windows.Forms.Label();
        this.labelo = new System.Windows.Forms.Label();
        this.labeln = new System.Windows.Forms.Label();
        this.static39 = new System.Windows.Forms.Label();
        this.static38 = new System.Windows.Forms.Label();
        this.static37 = new System.Windows.Forms.Label();
        this.static36 = new System.Windows.Forms.Label();
        this.static35 = new System.Windows.Forms.Label();
        this.static34 = new System.Windows.Forms.Label();
        this.groupBox5 = new System.Windows.Forms.GroupBox();
        this.trackBar15 = new System.Windows.Forms.TrackBar();
        this.checkBox3 = new System.Windows.Forms.CheckBox();
        this.label14 = new System.Windows.Forms.Label();
        this.label25 = new System.Windows.Forms.Label();
        this.label26 = new System.Windows.Forms.Label();
        this.textBoxm = new System.Windows.Forms.TextBox();
        this.textBoxl = new System.Windows.Forms.TextBox();
        this.labelm = new System.Windows.Forms.Label();
        this.labell = new System.Windows.Forms.Label();
        this.static33 = new System.Windows.Forms.Label();
        this.static32 = new System.Windows.Forms.Label();
        this.static29 = new System.Windows.Forms.Label();
        this.static31 = new System.Windows.Forms.Label();
        this.static30 = new System.Windows.Forms.Label();
        this.groupBox4 = new System.Windows.Forms.GroupBox();
        this.checkBoxf2 = new System.Windows.Forms.CheckBox();
        this.checkBoxf = new System.Windows.Forms.CheckBox();
        this.checkBoxe2 = new System.Windows.Forms.CheckBox();
        this.checkBoxe = new System.Windows.Forms.CheckBox();
        this.static28 = new System.Windows.Forms.Label();
        this.static27 = new System.Windows.Forms.Label();
        this.checkBoxD2 = new System.Windows.Forms.CheckBox();
        this.checkBoxB2 = new System.Windows.Forms.CheckBox();
        this.checkBoxC2 = new System.Windows.Forms.CheckBox();
        this.checkBoxD = new System.Windows.Forms.CheckBox();
        this.checkBoxC = new System.Windows.Forms.CheckBox();
        this.checkBoxB = new System.Windows.Forms.CheckBox();
        this.static26 = new System.Windows.Forms.Label();
        this.static25 = new System.Windows.Forms.Label();
        this.static24 = new System.Windows.Forms.Label();
        this.static22 = new System.Windows.Forms.Label();
        this.static23 = new System.Windows.Forms.Label();
        this.static21 = new System.Windows.Forms.Label();
        this.groupBox3 = new System.Windows.Forms.GroupBox();
        this.checkBox2 = new System.Windows.Forms.CheckBox();
        this.trackBar14 = new System.Windows.Forms.TrackBar();
        this.label24 = new System.Windows.Forms.Label();
        this.textBoxy = new System.Windows.Forms.TextBox();
        this.label23 = new System.Windows.Forms.Label();
        this.textBoxH = new System.Windows.Forms.TextBox();
        this.textBoxG = new System.Windows.Forms.TextBox();
        this.labely = new System.Windows.Forms.Label();
        this.labelH = new System.Windows.Forms.Label();
        this.label16 = new System.Windows.Forms.Label();
        this.labelG = new System.Windows.Forms.Label();
        this.static17 = new System.Windows.Forms.Label();
        this.label22 = new System.Windows.Forms.Label();
        this.static16 = new System.Windows.Forms.Label();
        this.static18 = new System.Windows.Forms.Label();
        this.static20 = new System.Windows.Forms.Label();
        this.static19 = new System.Windows.Forms.Label();
        this.groupBox1 = new System.Windows.Forms.GroupBox();
        this.label21 = new System.Windows.Forms.Label();
        this.label20 = new System.Windows.Forms.Label();
        this.checkBox1 = new System.Windows.Forms.CheckBox();
        this.trackBar13 = new System.Windows.Forms.TrackBar();
        this.label18 = new System.Windows.Forms.Label();
        this.static6 = new System.Windows.Forms.Label();
        this.static5 = new System.Windows.Forms.Label();
        this.static4 = new System.Windows.Forms.Label();
        this.textBoxi = new System.Windows.Forms.TextBox();
        this.label19 = new System.Windows.Forms.Label();
        this.static3 = new System.Windows.Forms.Label();
        this.labeli = new System.Windows.Forms.Label();
        this.static7 = new System.Windows.Forms.Label();
        this.static2 = new System.Windows.Forms.Label();
        this.labelj = new System.Windows.Forms.Label();
        this.textBoxz = new System.Windows.Forms.TextBox();
        this.textBoxk = new System.Windows.Forms.TextBox();
        this.labelz = new System.Windows.Forms.Label();
        this.labelk = new System.Windows.Forms.Label();
        this.textBoxj = new System.Windows.Forms.TextBox();
        this.tabPage5 = new System.Windows.Forms.TabPage();
        this.groupBox15 = new System.Windows.Forms.GroupBox();
        this.label41 = new System.Windows.Forms.Label();
        this.label40 = new System.Windows.Forms.Label();
        this.labelled2 = new System.Windows.Forms.Label();
        this.labelled1 = new System.Windows.Forms.Label();
        this.label37 = new System.Windows.Forms.Label();
        this.label38 = new System.Windows.Forms.Label();
        this.label39 = new System.Windows.Forms.Label();
        this.label36 = new System.Windows.Forms.Label();
        this.label35 = new System.Windows.Forms.Label();
        this.groupBox8 = new System.Windows.Forms.GroupBox();
        this.errorlabel = new System.Windows.Forms.Label();
        this.label12 = new System.Windows.Forms.Label();
        this.static47 = new System.Windows.Forms.Label();
        this.groupBox7 = new System.Windows.Forms.GroupBox();
        this.voltagelevel = new System.Windows.Forms.Label();
        this.progressBar1 = new System.Windows.Forms.ProgressBar();
        this.textBoxv = new System.Windows.Forms.TextBox();
        this.labelv = new System.Windows.Forms.Label();
        this.static44 = new System.Windows.Forms.Label();
        this.static41 = new System.Windows.Forms.Label();
        this.static43 = new System.Windows.Forms.Label();
        this.static42 = new System.Windows.Forms.Label();
        this.tabPage4 = new System.Windows.Forms.TabPage();
        this.static99 = new System.Windows.Forms.Label();
        this.Battery = new System.Windows.Forms.GroupBox();
        this.labelvolts = new System.Windows.Forms.Label();
        this.pbVoltage = new System.Windows.Forms.ProgressBar();
        this.groupBox12 = new System.Windows.Forms.GroupBox();
        this.TrackBar7 = new XComponent.SliderBar.MACTrackBar();
        this.static58 = new System.Windows.Forms.Label();
        this.static55 = new System.Windows.Forms.Label();
        this.static61 = new System.Windows.Forms.Label();
        this.groupBox11 = new System.Windows.Forms.GroupBox();
        this.TrackBar12 = new XComponent.SliderBar.MACTrackBar();
        this.TrackBar11 = new XComponent.SliderBar.MACTrackBar();
        this.TrackBar10 = new XComponent.SliderBar.MACTrackBar();
        this.panel2 = new System.Windows.Forms.Panel();
        this.panel4 = new System.Windows.Forms.Panel();
        this.panelRollCenter = new System.Windows.Forms.Panel();
        this.label15 = new System.Windows.Forms.Label();
        this.panel1 = new System.Windows.Forms.Panel();
        this.label10 = new System.Windows.Forms.Label();
        this.label9 = new System.Windows.Forms.Label();
        this.label8 = new System.Windows.Forms.Label();
        this.label6 = new System.Windows.Forms.Label();
        this.label7 = new System.Windows.Forms.Label();
        this.label5 = new System.Windows.Forms.Label();
        this.label4 = new System.Windows.Forms.Label();
        this.label3 = new System.Windows.Forms.Label();
        this.label2 = new System.Windows.Forms.Label();
        this.label1 = new System.Windows.Forms.Label();
        this.TrackBar9 = new XComponent.SliderBar.MACTrackBar();
        this.TrackBar8 = new XComponent.SliderBar.MACTrackBar();
        this.groupBox10 = new System.Windows.Forms.GroupBox();
        this.TrackBar6 = new XComponent.SliderBar.MACTrackBar();
        this.TrackBar5 = new XComponent.SliderBar.MACTrackBar();
        this.TrackBar4 = new XComponent.SliderBar.MACTrackBar();
        this.static54 = new System.Windows.Forms.Label();
        this.static53 = new System.Windows.Forms.Label();
        this.staic59 = new System.Windows.Forms.Label();
        this.static60 = new System.Windows.Forms.Label();
        this.static52 = new System.Windows.Forms.Label();
        this.groupBox9 = new System.Windows.Forms.GroupBox();
        this.TrackBar3 = new XComponent.SliderBar.MACTrackBar();
        this.TrackBar2 = new XComponent.SliderBar.MACTrackBar();
        this.static51 = new System.Windows.Forms.Label();
        this.TrackBar1 = new XComponent.SliderBar.MACTrackBar();
        this.static49 = new System.Windows.Forms.Label();
        this.static57 = new System.Windows.Forms.Label();
        this.static50 = new System.Windows.Forms.Label();
        this.static56 = new System.Windows.Forms.Label();
        this.tabPage6 = new System.Windows.Forms.TabPage();
        this.groupBox13 = new System.Windows.Forms.GroupBox();
        this.label13 = new System.Windows.Forms.Label();
        this.flash = new System.Windows.Forms.Button();
        this.progressBar4 = new System.Windows.Forms.ProgressBar();
        this.showavrout = new System.Windows.Forms.CheckBox();
        this.avrdudeout = new System.Windows.Forms.TextBox();
        this.serialPort = new System.IO.Ports.SerialPort(this.components);
        this.timer1 = new System.Windows.Forms.Timer(this.components);
        this.pictureBox1 = new System.Windows.Forms.PictureBox();
        this.toolTip1 = new System.Windows.Forms.ToolTip(this.components);
        this.statusStrip1 = new System.Windows.Forms.StatusStrip();
        this.toolStatusLabel = new System.Windows.Forms.ToolStripStatusLabel();
        this.timersens = new System.Windows.Forms.Timer(this.components);
        this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
        this.saveFileDialog1 = new System.Windows.Forms.SaveFileDialog();
        this.pingshrediquette = new System.Windows.Forms.Timer(this.components);
        this.label11 = new System.Windows.Forms.Label();
        this.startupdelay = new System.Windows.Forms.Timer(this.components);
        this.linkLabel2 = new System.Windows.Forms.LinkLabel();
        this.label17 = new System.Windows.Forms.Label();
        this.lbch1 = new System.Windows.Forms.Label();
        this.lbch2 = new System.Windows.Forms.Label();
        this.lbch3 = new System.Windows.Forms.Label();
        this.lbch4 = new System.Windows.Forms.Label();
        this.lbch5 = new System.Windows.Forms.Label();
        this.lbch6 = new System.Windows.Forms.Label();
        this.lbch7 = new System.Windows.Forms.Label();
        this.lbch8 = new System.Windows.Forms.Label();
        this.emergmask = new TriGUI_v11.BitMask();
        this.led2mask = new TriGUI_v11.BitMask();
        this.led1mask = new TriGUI_v11.BitMask();
        label42 = new System.Windows.Forms.Label();
        this.tabControl.SuspendLayout();
        this.tabPage1.SuspendLayout();
        this.RxD.SuspendLayout();
        ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
        this.tabPage2.SuspendLayout();
        this.groupBox14.SuspendLayout();
        this.groupBox2.SuspendLayout();
        this.tabPage3.SuspendLayout();
        this.groupBox6.SuspendLayout();
        this.groupBox5.SuspendLayout();
        ((System.ComponentModel.ISupportInitialize)(this.trackBar15)).BeginInit();
        this.groupBox4.SuspendLayout();
        this.groupBox3.SuspendLayout();
        ((System.ComponentModel.ISupportInitialize)(this.trackBar14)).BeginInit();
        this.groupBox1.SuspendLayout();
        ((System.ComponentModel.ISupportInitialize)(this.trackBar13)).BeginInit();
        this.tabPage5.SuspendLayout();
        this.groupBox15.SuspendLayout();
        this.groupBox8.SuspendLayout();
        this.groupBox7.SuspendLayout();
        this.tabPage4.SuspendLayout();
        this.Battery.SuspendLayout();
        this.groupBox12.SuspendLayout();
        this.groupBox11.SuspendLayout();
        this.groupBox10.SuspendLayout();
        this.groupBox9.SuspendLayout();
        this.tabPage6.SuspendLayout();
        this.groupBox13.SuspendLayout();
        ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
        this.statusStrip1.SuspendLayout();
        this.SuspendLayout();
        // 
        // tabControl
        // 
        this.tabControl.Controls.Add(this.tabPage1);
        this.tabControl.Controls.Add(this.tabPage2);
        this.tabControl.Controls.Add(this.tabPage3);
        this.tabControl.Controls.Add(this.tabPage5);
        this.tabControl.Controls.Add(this.tabPage4);
        this.tabControl.Controls.Add(this.tabPage6);
        this.tabControl.Location = new System.Drawing.Point(12, 60);
        this.tabControl.Name = "tabControl";
        this.tabControl.SelectedIndex = 0;
        this.tabControl.Size = new System.Drawing.Size(657, 392);
        this.tabControl.TabIndex = 0;
        this.tabControl.SelectedIndexChanged += new System.EventHandler(this.TabControl1SelectedIndexChanged);
        // 
        // tabPage1
        // 
        this.tabPage1.Controls.Add(this.showdebug);
        this.tabPage1.Controls.Add(this.Compare_Settings);
        this.tabPage1.Controls.Add(this.RxD);
        this.tabPage1.Controls.Add(this.linkLabel1);
        this.tabPage1.Controls.Add(this.btnRefresh);
        this.tabPage1.Controls.Add(this.readsettings);
        this.tabPage1.Controls.Add(this.writesettings);
        this.tabPage1.Controls.Add(this.labelok);
        this.tabPage1.Controls.Add(this.writeall);
        this.tabPage1.Controls.Add(this.resetmC);
        this.tabPage1.Controls.Add(this.comport);
        this.tabPage1.Controls.Add(this.connect);
        this.tabPage1.Controls.Add(this.searchlabel);
        this.tabPage1.Controls.Add(this.progressBar2);
        this.tabPage1.Controls.Add(this.pictureBox2);
        this.tabPage1.Location = new System.Drawing.Point(4, 22);
        this.tabPage1.Name = "tabPage1";
        this.tabPage1.Padding = new System.Windows.Forms.Padding(3);
        this.tabPage1.Size = new System.Drawing.Size(649, 366);
        this.tabPage1.TabIndex = 0;
        this.tabPage1.Text = "Connection";
        this.tabPage1.UseVisualStyleBackColor = true;
        // 
        // showdebug
        // 
        this.showdebug.Location = new System.Drawing.Point(21, 307);
        this.showdebug.Name = "showdebug";
        this.showdebug.Size = new System.Drawing.Size(104, 24);
        this.showdebug.TabIndex = 11;
        this.showdebug.Text = "Show terminal";
        this.toolTip1.SetToolTip(this.showdebug, "Shows a terminal windows showing which things are transferred from the tricopter");
        this.showdebug.UseVisualStyleBackColor = true;
        this.showdebug.CheckedChanged += new System.EventHandler(this.ShowdebugCheckedChanged);
        // 
        // Compare_Settings
        // 
        this.Compare_Settings.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.Compare_Settings.Location = new System.Drawing.Point(21, 278);
        this.Compare_Settings.Name = "Compare_Settings";
        this.Compare_Settings.Size = new System.Drawing.Size(136, 23);
        this.Compare_Settings.TabIndex = 21;
        this.Compare_Settings.Text = "Compare settings (*.shr)";
        this.Compare_Settings.UseVisualStyleBackColor = true;
        this.Compare_Settings.Click += new System.EventHandler(this.Button2Click);
        // 
        // RxD
        // 
        this.RxD.Controls.Add(this.static48);
        this.RxD.Controls.Add(this.bytecounter);
        this.RxD.Controls.Add(this.textRxD);
        this.RxD.Controls.Add(this.clearRxD);
        this.RxD.Location = new System.Drawing.Point(338, 14);
        this.RxD.Name = "RxD";
        this.RxD.Size = new System.Drawing.Size(289, 297);
        this.RxD.TabIndex = 10;
        this.RxD.TabStop = false;
        this.RxD.Text = "RxD";
        this.RxD.Visible = false;
        // 
        // static48
        // 
        this.static48.Location = new System.Drawing.Point(39, 273);
        this.static48.Name = "static48";
        this.static48.Size = new System.Drawing.Size(70, 18);
        this.static48.TabIndex = 16;
        this.static48.Text = "Dataset size:";
        // 
        // bytecounter
        // 
        this.bytecounter.Location = new System.Drawing.Point(114, 273);
        this.bytecounter.Name = "bytecounter";
        this.bytecounter.Size = new System.Drawing.Size(83, 18);
        this.bytecounter.TabIndex = 15;
        this.bytecounter.Text = "---";
        // 
        // textRxD
        // 
        this.textRxD.Location = new System.Drawing.Point(6, 19);
        this.textRxD.Multiline = true;
        this.textRxD.Name = "textRxD";
        this.textRxD.ReadOnly = true;
        this.textRxD.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
        this.textRxD.Size = new System.Drawing.Size(285, 224);
        this.textRxD.TabIndex = 3;
        // 
        // clearRxD
        // 
        this.clearRxD.Location = new System.Drawing.Point(207, 268);
        this.clearRxD.Name = "clearRxD";
        this.clearRxD.Size = new System.Drawing.Size(75, 23);
        this.clearRxD.TabIndex = 5;
        this.clearRxD.Text = "clear";
        this.toolTip1.SetToolTip(this.clearRxD, "Clears the content of the terminal window");
        this.clearRxD.UseVisualStyleBackColor = true;
        this.clearRxD.Click += new System.EventHandler(this.ClearRxDClick);
        // 
        // linkLabel1
        // 
        this.linkLabel1.Font = new System.Drawing.Font("Microsoft Sans Serif", 14F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.linkLabel1.Location = new System.Drawing.Point(366, 285);
        this.linkLabel1.Name = "linkLabel1";
        this.linkLabel1.Size = new System.Drawing.Size(211, 23);
        this.linkLabel1.TabIndex = 20;
        this.linkLabel1.TabStop = true;
        this.linkLabel1.Text = "Watch the tutorial video!";
        this.toolTip1.SetToolTip(this.linkLabel1, "http://www.villalachouette.de/william/krims/tricopter/TriGUI_tut.wmv");
        this.linkLabel1.LinkClicked += new System.Windows.Forms.LinkLabelLinkClickedEventHandler(this.LinkLabel1LinkClicked);
        // 
        // btnRefresh
        // 
        this.btnRefresh.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.btnRefresh.Location = new System.Drawing.Point(163, 62);
        this.btnRefresh.Name = "btnRefresh";
        this.btnRefresh.Size = new System.Drawing.Size(58, 21);
        this.btnRefresh.TabIndex = 19;
        this.btnRefresh.Text = "Refresh";
        this.btnRefresh.UseVisualStyleBackColor = true;
        this.btnRefresh.Click += new System.EventHandler(this.Button1Click);
        // 
        // readsettings
        // 
        this.readsettings.Enabled = false;
        this.readsettings.Location = new System.Drawing.Point(21, 229);
        this.readsettings.Name = "readsettings";
        this.readsettings.Size = new System.Drawing.Size(136, 23);
        this.readsettings.TabIndex = 16;
        this.readsettings.Text = "Load file and write to µC";
        this.toolTip1.SetToolTip(this.readsettings, "Loads all parameters from a text file and writes them to the the µC");
        this.readsettings.UseVisualStyleBackColor = true;
        this.readsettings.Click += new System.EventHandler(this.ReadsettingsClick);
        // 
        // writesettings
        // 
        this.writesettings.Enabled = false;
        this.writesettings.Location = new System.Drawing.Point(21, 200);
        this.writesettings.Name = "writesettings";
        this.writesettings.Size = new System.Drawing.Size(136, 23);
        this.writesettings.TabIndex = 15;
        this.writesettings.Text = "Save all parameters to file";
        this.toolTip1.SetToolTip(this.writesettings, "Saves the all parameters from the GUI to a text file");
        this.writesettings.UseVisualStyleBackColor = true;
        this.writesettings.Click += new System.EventHandler(this.writesettingsClick);
        // 
        // labelok
        // 
        this.labelok.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.labelok.ForeColor = System.Drawing.Color.Lime;
        this.labelok.Location = new System.Drawing.Point(567, 314);
        this.labelok.Name = "labelok";
        this.labelok.Size = new System.Drawing.Size(76, 23);
        this.labelok.TabIndex = 13;
        this.labelok.Text = "OK!";
        this.labelok.TextAlign = System.Drawing.ContentAlignment.BottomRight;
        this.labelok.Visible = false;
        // 
        // writeall
        // 
        this.writeall.Enabled = false;
        this.writeall.Location = new System.Drawing.Point(21, 131);
        this.writeall.Name = "writeall";
        this.writeall.Size = new System.Drawing.Size(136, 50);
        this.writeall.TabIndex = 6;
        this.writeall.Text = "Write all parameters to µC";
        this.toolTip1.SetToolTip(this.writeall, "Writes all the parameters of all tabs and resets the tricopter. The new parameter" +
                "s are subsequently loaded again from the copter");
        this.writeall.UseVisualStyleBackColor = true;
        this.writeall.Click += new System.EventHandler(this.WriteallClick);
        // 
        // resetmC
        // 
        this.resetmC.Enabled = false;
        this.resetmC.Location = new System.Drawing.Point(21, 89);
        this.resetmC.Name = "resetmC";
        this.resetmC.Size = new System.Drawing.Size(136, 23);
        this.resetmC.TabIndex = 2;
        this.resetmC.Text = "Disconnect";
        this.toolTip1.SetToolTip(this.resetmC, "Sends reset command to the tricopter and closes the serialport");
        this.resetmC.UseVisualStyleBackColor = true;
        this.resetmC.Click += new System.EventHandler(this.resetmCClick);
        // 
        // comport
        // 
        this.comport.FormattingEnabled = true;
        this.comport.Items.AddRange(new object[] {
            "COM1",
            "COM2",
            "COM3",
            "COM4",
            "COM5",
            "COM6",
            "COM7",
            "COM8",
            "COM9",
            "COM10",
            "COM11",
            "COM12",
            "COM13",
            "COM14",
            "COM15",
            "COM16",
            "COM17",
            "COM18",
            "COM19",
            "COM20",
            "COM21",
            "COM22",
            "COM23",
            "COM24",
            "COM25",
            "COM26",
            "COM27",
            "COM28",
            "COM29",
            "COM30"});
        this.comport.Location = new System.Drawing.Point(163, 35);
        this.comport.Name = "comport";
        this.comport.Size = new System.Drawing.Size(58, 21);
        this.comport.TabIndex = 1;
        this.comport.Text = "COM1";
        this.toolTip1.SetToolTip(this.comport, "Select the COM port here.\r\nPlease note that avrdude doesn\'t seem to support high " +
                "COM port numbers...");
        this.comport.SelectedIndexChanged += new System.EventHandler(this.ComportSelectedIndexChanged);
        // 
        // connect
        // 
        this.connect.Location = new System.Drawing.Point(21, 33);
        this.connect.Name = "connect";
        this.connect.Size = new System.Drawing.Size(136, 50);
        this.connect.TabIndex = 0;
        this.connect.Text = "Connect + read out";
        this.toolTip1.SetToolTip(this.connect, "Opens a serial connection and reads the current paramters from the tricopter");
        this.connect.UseVisualStyleBackColor = true;
        this.connect.Click += new System.EventHandler(this.ConnectClick);
        // 
        // searchlabel
        // 
        this.searchlabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.searchlabel.ForeColor = System.Drawing.Color.Lime;
        this.searchlabel.Location = new System.Drawing.Point(511, 314);
        this.searchlabel.Name = "searchlabel";
        this.searchlabel.Size = new System.Drawing.Size(132, 23);
        this.searchlabel.TabIndex = 17;
        this.searchlabel.Text = "Searching...";
        this.searchlabel.TextAlign = System.Drawing.ContentAlignment.BottomRight;
        this.searchlabel.Visible = false;
        // 
        // progressBar2
        // 
        this.progressBar2.Location = new System.Drawing.Point(6, 337);
        this.progressBar2.MarqueeAnimationSpeed = 10;
        this.progressBar2.Maximum = 27;
        this.progressBar2.Name = "progressBar2";
        this.progressBar2.Size = new System.Drawing.Size(637, 20);
        this.progressBar2.Step = 1;
        this.progressBar2.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
        this.progressBar2.TabIndex = 12;
        // 
        // pictureBox2
        // 
        this.pictureBox2.Location = new System.Drawing.Point(327, 14);
        this.pictureBox2.Name = "pictureBox2";
        this.pictureBox2.Size = new System.Drawing.Size(300, 300);
        this.pictureBox2.TabIndex = 14;
        this.pictureBox2.TabStop = false;
        // 
        // tabPage2
        // 
        this.tabPage2.Controls.Add(this.groupBox14);
        this.tabPage2.Controls.Add(this.groupBox2);
        this.tabPage2.Location = new System.Drawing.Point(4, 22);
        this.tabPage2.Name = "tabPage2";
        this.tabPage2.Padding = new System.Windows.Forms.Padding(3);
        this.tabPage2.Size = new System.Drawing.Size(649, 366);
        this.tabPage2.TabIndex = 1;
        this.tabPage2.Text = "Radio settings";
        this.tabPage2.UseVisualStyleBackColor = true;
        // 
        // groupBox14
        // 
        this.groupBox14.Controls.Add(label42);
        this.groupBox14.Controls.Add(this.lbch8);
        this.groupBox14.Controls.Add(this.lbch7);
        this.groupBox14.Controls.Add(this.lbch6);
        this.groupBox14.Controls.Add(this.lbch5);
        this.groupBox14.Controls.Add(this.lbch4);
        this.groupBox14.Controls.Add(this.lbch3);
        this.groupBox14.Controls.Add(this.lbch2);
        this.groupBox14.Controls.Add(this.lbch1);
        this.groupBox14.Controls.Add(this.label34);
        this.groupBox14.Controls.Add(this.label33);
        this.groupBox14.Controls.Add(this.label32);
        this.groupBox14.Controls.Add(this.label31);
        this.groupBox14.Controls.Add(this.label30);
        this.groupBox14.Controls.Add(this.label29);
        this.groupBox14.Controls.Add(this.label28);
        this.groupBox14.Controls.Add(this.label27);
        this.groupBox14.Controls.Add(this.tbch8);
        this.groupBox14.Controls.Add(this.tbch7);
        this.groupBox14.Controls.Add(this.tbch6);
        this.groupBox14.Controls.Add(this.tbch5);
        this.groupBox14.Controls.Add(this.tbch4);
        this.groupBox14.Controls.Add(this.tbch3);
        this.groupBox14.Controls.Add(this.tbch2);
        this.groupBox14.Controls.Add(this.tbch1);
        this.groupBox14.Location = new System.Drawing.Point(407, 20);
        this.groupBox14.Name = "groupBox14";
        this.groupBox14.Size = new System.Drawing.Size(223, 329);
        this.groupBox14.TabIndex = 8;
        this.groupBox14.TabStop = false;
        this.groupBox14.Text = "RX Channels";
        // 
        // label34
        // 
        this.label34.AutoSize = true;
        this.label34.Location = new System.Drawing.Point(191, 141);
        this.label34.Name = "label34";
        this.label34.Size = new System.Drawing.Size(13, 13);
        this.label34.TabIndex = 40;
        this.label34.Text = "8";
        // 
        // label33
        // 
        this.label33.AutoSize = true;
        this.label33.Location = new System.Drawing.Point(166, 141);
        this.label33.Name = "label33";
        this.label33.Size = new System.Drawing.Size(13, 13);
        this.label33.TabIndex = 39;
        this.label33.Text = "7";
        // 
        // label32
        // 
        this.label32.AutoSize = true;
        this.label32.Location = new System.Drawing.Point(141, 141);
        this.label32.Name = "label32";
        this.label32.Size = new System.Drawing.Size(13, 13);
        this.label32.TabIndex = 38;
        this.label32.Text = "6";
        // 
        // label31
        // 
        this.label31.AutoSize = true;
        this.label31.Location = new System.Drawing.Point(116, 141);
        this.label31.Name = "label31";
        this.label31.Size = new System.Drawing.Size(13, 13);
        this.label31.TabIndex = 37;
        this.label31.Text = "5";
        // 
        // label30
        // 
        this.label30.AutoSize = true;
        this.label30.Location = new System.Drawing.Point(91, 141);
        this.label30.Name = "label30";
        this.label30.Size = new System.Drawing.Size(13, 13);
        this.label30.TabIndex = 36;
        this.label30.Text = "4";
        // 
        // label29
        // 
        this.label29.AutoSize = true;
        this.label29.Location = new System.Drawing.Point(66, 141);
        this.label29.Name = "label29";
        this.label29.Size = new System.Drawing.Size(13, 13);
        this.label29.TabIndex = 35;
        this.label29.Text = "3";
        // 
        // label28
        // 
        this.label28.AutoSize = true;
        this.label28.Location = new System.Drawing.Point(41, 141);
        this.label28.Name = "label28";
        this.label28.Size = new System.Drawing.Size(13, 13);
        this.label28.TabIndex = 34;
        this.label28.Text = "2";
        // 
        // label27
        // 
        this.label27.AutoSize = true;
        this.label27.Location = new System.Drawing.Point(16, 141);
        this.label27.Name = "label27";
        this.label27.Size = new System.Drawing.Size(13, 13);
        this.label27.TabIndex = 33;
        this.label27.Text = "1";
        // 
        // tbch8
        // 
        this.tbch8.BackColor = System.Drawing.Color.Transparent;
        this.tbch8.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.tbch8.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.tbch8.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.tbch8.IndentHeight = 1;
        this.tbch8.IndentWidth = 1;
        this.tbch8.LargeChange = 100;
        this.tbch8.Location = new System.Drawing.Point(188, 35);
        this.tbch8.Maximum = 255;
        this.tbch8.Minimum = 0;
        this.tbch8.Name = "tbch8";
        this.tbch8.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.tbch8.Size = new System.Drawing.Size(19, 100);
        this.tbch8.SmallChange = 10;
        this.tbch8.TabIndex = 32;
        this.tbch8.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.tbch8.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.tbch8.TickFrequency = 64;
        this.tbch8.TickHeight = 1;
        this.tbch8.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.tbch8.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.tbch8.TrackerSize = new System.Drawing.Size(13, 13);
        this.tbch8.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.tbch8.TrackLineHeight = 9;
        this.tbch8.Value = 9;
        // 
        // tbch7
        // 
        this.tbch7.BackColor = System.Drawing.Color.Transparent;
        this.tbch7.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.tbch7.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.tbch7.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.tbch7.IndentHeight = 1;
        this.tbch7.IndentWidth = 1;
        this.tbch7.LargeChange = 100;
        this.tbch7.Location = new System.Drawing.Point(163, 35);
        this.tbch7.Maximum = 255;
        this.tbch7.Minimum = 0;
        this.tbch7.Name = "tbch7";
        this.tbch7.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.tbch7.Size = new System.Drawing.Size(19, 100);
        this.tbch7.SmallChange = 10;
        this.tbch7.TabIndex = 31;
        this.tbch7.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.tbch7.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.tbch7.TickFrequency = 64;
        this.tbch7.TickHeight = 1;
        this.tbch7.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.tbch7.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.tbch7.TrackerSize = new System.Drawing.Size(13, 13);
        this.tbch7.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.tbch7.TrackLineHeight = 9;
        this.tbch7.Value = 9;
        // 
        // tbch6
        // 
        this.tbch6.BackColor = System.Drawing.Color.Transparent;
        this.tbch6.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.tbch6.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.tbch6.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.tbch6.IndentHeight = 1;
        this.tbch6.IndentWidth = 1;
        this.tbch6.LargeChange = 100;
        this.tbch6.Location = new System.Drawing.Point(138, 35);
        this.tbch6.Maximum = 255;
        this.tbch6.Minimum = 0;
        this.tbch6.Name = "tbch6";
        this.tbch6.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.tbch6.Size = new System.Drawing.Size(19, 100);
        this.tbch6.SmallChange = 10;
        this.tbch6.TabIndex = 30;
        this.tbch6.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.tbch6.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.tbch6.TickFrequency = 64;
        this.tbch6.TickHeight = 1;
        this.tbch6.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.tbch6.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.tbch6.TrackerSize = new System.Drawing.Size(13, 13);
        this.tbch6.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.tbch6.TrackLineHeight = 9;
        this.tbch6.Value = 9;
        // 
        // tbch5
        // 
        this.tbch5.BackColor = System.Drawing.Color.Transparent;
        this.tbch5.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.tbch5.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.tbch5.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.tbch5.IndentHeight = 1;
        this.tbch5.IndentWidth = 1;
        this.tbch5.LargeChange = 100;
        this.tbch5.Location = new System.Drawing.Point(113, 35);
        this.tbch5.Maximum = 255;
        this.tbch5.Minimum = 0;
        this.tbch5.Name = "tbch5";
        this.tbch5.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.tbch5.Size = new System.Drawing.Size(19, 100);
        this.tbch5.SmallChange = 10;
        this.tbch5.TabIndex = 29;
        this.tbch5.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.tbch5.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.tbch5.TickFrequency = 64;
        this.tbch5.TickHeight = 1;
        this.tbch5.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.tbch5.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.tbch5.TrackerSize = new System.Drawing.Size(13, 13);
        this.tbch5.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.tbch5.TrackLineHeight = 9;
        this.tbch5.Value = 9;
        // 
        // tbch4
        // 
        this.tbch4.BackColor = System.Drawing.Color.Transparent;
        this.tbch4.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.tbch4.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.tbch4.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.tbch4.IndentHeight = 1;
        this.tbch4.IndentWidth = 1;
        this.tbch4.LargeChange = 100;
        this.tbch4.Location = new System.Drawing.Point(88, 35);
        this.tbch4.Maximum = 255;
        this.tbch4.Minimum = 0;
        this.tbch4.Name = "tbch4";
        this.tbch4.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.tbch4.Size = new System.Drawing.Size(19, 100);
        this.tbch4.SmallChange = 10;
        this.tbch4.TabIndex = 28;
        this.tbch4.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.tbch4.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.tbch4.TickFrequency = 64;
        this.tbch4.TickHeight = 1;
        this.tbch4.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.tbch4.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.tbch4.TrackerSize = new System.Drawing.Size(13, 13);
        this.tbch4.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.tbch4.TrackLineHeight = 9;
        this.tbch4.Value = 9;
        // 
        // tbch3
        // 
        this.tbch3.BackColor = System.Drawing.Color.Transparent;
        this.tbch3.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.tbch3.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.tbch3.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.tbch3.IndentHeight = 1;
        this.tbch3.IndentWidth = 1;
        this.tbch3.LargeChange = 100;
        this.tbch3.Location = new System.Drawing.Point(63, 35);
        this.tbch3.Maximum = 255;
        this.tbch3.Minimum = 0;
        this.tbch3.Name = "tbch3";
        this.tbch3.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.tbch3.Size = new System.Drawing.Size(19, 100);
        this.tbch3.SmallChange = 10;
        this.tbch3.TabIndex = 27;
        this.tbch3.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.tbch3.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.tbch3.TickFrequency = 64;
        this.tbch3.TickHeight = 1;
        this.tbch3.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.tbch3.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.tbch3.TrackerSize = new System.Drawing.Size(13, 13);
        this.tbch3.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.tbch3.TrackLineHeight = 9;
        this.tbch3.Value = 9;
        // 
        // tbch2
        // 
        this.tbch2.BackColor = System.Drawing.Color.Transparent;
        this.tbch2.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.tbch2.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.tbch2.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.tbch2.IndentHeight = 1;
        this.tbch2.IndentWidth = 1;
        this.tbch2.LargeChange = 100;
        this.tbch2.Location = new System.Drawing.Point(38, 35);
        this.tbch2.Maximum = 255;
        this.tbch2.Minimum = 0;
        this.tbch2.Name = "tbch2";
        this.tbch2.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.tbch2.Size = new System.Drawing.Size(19, 100);
        this.tbch2.SmallChange = 10;
        this.tbch2.TabIndex = 26;
        this.tbch2.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.tbch2.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.tbch2.TickFrequency = 64;
        this.tbch2.TickHeight = 1;
        this.tbch2.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.tbch2.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.tbch2.TrackerSize = new System.Drawing.Size(13, 13);
        this.tbch2.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.tbch2.TrackLineHeight = 9;
        this.tbch2.Value = 9;
        // 
        // tbch1
        // 
        this.tbch1.BackColor = System.Drawing.Color.Transparent;
        this.tbch1.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.tbch1.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.tbch1.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.tbch1.IndentHeight = 1;
        this.tbch1.IndentWidth = 1;
        this.tbch1.LargeChange = 100;
        this.tbch1.Location = new System.Drawing.Point(13, 35);
        this.tbch1.Maximum = 255;
        this.tbch1.Minimum = 0;
        this.tbch1.Name = "tbch1";
        this.tbch1.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.tbch1.Size = new System.Drawing.Size(19, 100);
        this.tbch1.SmallChange = 10;
        this.tbch1.TabIndex = 25;
        this.tbch1.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.tbch1.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.tbch1.TickFrequency = 64;
        this.tbch1.TickHeight = 1;
        this.tbch1.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.tbch1.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.tbch1.TrackerSize = new System.Drawing.Size(13, 13);
        this.tbch1.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.tbch1.TrackLineHeight = 9;
        this.tbch1.Value = 9;
        // 
        // groupBox2
        // 
        this.groupBox2.Controls.Add(this.switchbox);
        this.groupBox2.Controls.Add(this.deactivatemotors);
        this.groupBox2.Controls.Add(this.static67);
        this.groupBox2.Controls.Add(this.static68);
        this.groupBox2.Controls.Add(this.static66);
        this.groupBox2.Controls.Add(this.static65);
        this.groupBox2.Controls.Add(this.static64);
        this.groupBox2.Controls.Add(this.static63);
        this.groupBox2.Controls.Add(this.yawbox);
        this.groupBox2.Controls.Add(this.rollbox);
        this.groupBox2.Controls.Add(this.Nickbox);
        this.groupBox2.Controls.Add(this.throttlebox);
        this.groupBox2.Controls.Add(this.textBoxu);
        this.groupBox2.Controls.Add(this.labelu);
        this.groupBox2.Controls.Add(this.static40);
        this.groupBox2.Controls.Add(this.textBoxS);
        this.groupBox2.Controls.Add(this.textBoxR);
        this.groupBox2.Controls.Add(this.textBoxQ);
        this.groupBox2.Controls.Add(this.checkBoxT);
        this.groupBox2.Controls.Add(this.labels);
        this.groupBox2.Controls.Add(this.labelr);
        this.groupBox2.Controls.Add(this.labelq);
        this.groupBox2.Controls.Add(this.static15);
        this.groupBox2.Controls.Add(this.static13);
        this.groupBox2.Controls.Add(this.static14);
        this.groupBox2.Controls.Add(this.static12);
        this.groupBox2.Controls.Add(this.static11);
        this.groupBox2.Controls.Add(this.checkBoxT2);
        this.groupBox2.Controls.Add(this.static9);
        this.groupBox2.Controls.Add(this.static10);
        this.groupBox2.Location = new System.Drawing.Point(18, 20);
        this.groupBox2.Name = "groupBox2";
        this.groupBox2.Size = new System.Drawing.Size(383, 329);
        this.groupBox2.TabIndex = 7;
        this.groupBox2.TabStop = false;
        this.groupBox2.Text = "Transmitter settings";
        // 
        // switchbox
        // 
        this.switchbox.FormattingEnabled = true;
        this.switchbox.Items.AddRange(new object[] {
            "Ch. 1",
            "Ch. 2",
            "Ch. 3",
            "Ch. 4",
            "Ch. 5",
            "Ch. 6",
            "Ch. 7",
            "Ch. 8",
            "Ch. 9",
            "Ch. 10",
            "Ch. 11",
            "Ch. 12"});
        this.switchbox.Location = new System.Drawing.Point(267, 235);
        this.switchbox.Name = "switchbox";
        this.switchbox.Size = new System.Drawing.Size(57, 21);
        this.switchbox.TabIndex = 28;
        this.toolTip1.SetToolTip(this.switchbox, "Select RC function here");
        // 
        // deactivatemotors
        // 
        this.deactivatemotors.Checked = true;
        this.deactivatemotors.CheckState = System.Windows.Forms.CheckState.Checked;
        this.deactivatemotors.Location = new System.Drawing.Point(15, 286);
        this.deactivatemotors.Name = "deactivatemotors";
        this.deactivatemotors.Size = new System.Drawing.Size(132, 24);
        this.deactivatemotors.TabIndex = 27;
        this.deactivatemotors.Text = "Deactivate motors";
        this.toolTip1.SetToolTip(this.deactivatemotors, "Deactivate motors (for safety)");
        this.deactivatemotors.UseVisualStyleBackColor = true;
        this.deactivatemotors.Click += new System.EventHandler(this.DeactivatemotorsClick);
        // 
        // static67
        // 
        this.static67.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static67.Location = new System.Drawing.Point(15, 202);
        this.static67.Name = "static67";
        this.static67.Size = new System.Drawing.Size(246, 15);
        this.static67.TabIndex = 26;
        this.static67.Text = "Controls (RX/TX channel assignment)";
        // 
        // static68
        // 
        this.static68.Location = new System.Drawing.Point(267, 204);
        this.static68.Name = "static68";
        this.static68.Size = new System.Drawing.Size(87, 30);
        this.static68.TabIndex = 26;
        this.static68.Text = "Motor switch/ \r\nmode selection:";
        // 
        // static66
        // 
        this.static66.Location = new System.Drawing.Point(204, 217);
        this.static66.Name = "static66";
        this.static66.Size = new System.Drawing.Size(57, 15);
        this.static66.TabIndex = 26;
        this.static66.Text = "Yaw:";
        // 
        // static65
        // 
        this.static65.Location = new System.Drawing.Point(141, 217);
        this.static65.Name = "static65";
        this.static65.Size = new System.Drawing.Size(57, 15);
        this.static65.TabIndex = 26;
        this.static65.Text = "Roll:";
        // 
        // static64
        // 
        this.static64.Location = new System.Drawing.Point(78, 217);
        this.static64.Name = "static64";
        this.static64.Size = new System.Drawing.Size(57, 15);
        this.static64.TabIndex = 26;
        this.static64.Text = "Nick:";
        // 
        // static63
        // 
        this.static63.Location = new System.Drawing.Point(15, 217);
        this.static63.Name = "static63";
        this.static63.Size = new System.Drawing.Size(57, 15);
        this.static63.TabIndex = 26;
        this.static63.Text = "Throttle:";
        // 
        // yawbox
        // 
        this.yawbox.FormattingEnabled = true;
        this.yawbox.Items.AddRange(new object[] {
            "Ch. 1",
            "Ch. 2",
            "Ch. 3",
            "Ch. 4",
            "Ch. 5",
            "Ch. 6",
            "Ch. 7",
            "Ch. 8",
            "Ch. 9",
            "Ch. 10",
            "Ch. 11",
            "Ch. 12"});
        this.yawbox.Location = new System.Drawing.Point(204, 235);
        this.yawbox.Name = "yawbox";
        this.yawbox.Size = new System.Drawing.Size(57, 21);
        this.yawbox.TabIndex = 25;
        this.toolTip1.SetToolTip(this.yawbox, "Select RC function here");
        // 
        // rollbox
        // 
        this.rollbox.FormattingEnabled = true;
        this.rollbox.Items.AddRange(new object[] {
            "Ch. 1",
            "Ch. 2",
            "Ch. 3",
            "Ch. 4",
            "Ch. 5",
            "Ch. 6",
            "Ch. 7",
            "Ch. 8",
            "Ch. 9",
            "Ch. 10",
            "Ch. 11",
            "Ch. 12"});
        this.rollbox.Location = new System.Drawing.Point(141, 235);
        this.rollbox.Name = "rollbox";
        this.rollbox.Size = new System.Drawing.Size(57, 21);
        this.rollbox.TabIndex = 24;
        this.toolTip1.SetToolTip(this.rollbox, "Select RC function here");
        // 
        // Nickbox
        // 
        this.Nickbox.FormattingEnabled = true;
        this.Nickbox.Items.AddRange(new object[] {
            "Ch. 1",
            "Ch. 2",
            "Ch. 3",
            "Ch. 4",
            "Ch. 5",
            "Ch. 6",
            "Ch. 7",
            "Ch. 8",
            "Ch. 9",
            "Ch. 10",
            "Ch. 11",
            "Ch. 12"});
        this.Nickbox.Location = new System.Drawing.Point(78, 235);
        this.Nickbox.Name = "Nickbox";
        this.Nickbox.Size = new System.Drawing.Size(57, 21);
        this.Nickbox.TabIndex = 23;
        this.toolTip1.SetToolTip(this.Nickbox, "Select RC function here");
        // 
        // throttlebox
        // 
        this.throttlebox.FormattingEnabled = true;
        this.throttlebox.Items.AddRange(new object[] {
            "Ch. 1",
            "Ch. 2",
            "Ch. 3",
            "Ch. 4",
            "Ch. 5",
            "Ch. 6",
            "Ch. 7",
            "Ch. 8",
            "Ch. 9",
            "Ch. 10",
            "Ch. 11",
            "Ch. 12"});
        this.throttlebox.Location = new System.Drawing.Point(15, 235);
        this.throttlebox.Name = "throttlebox";
        this.throttlebox.Size = new System.Drawing.Size(57, 21);
        this.throttlebox.TabIndex = 22;
        this.toolTip1.SetToolTip(this.throttlebox, "Select RC function here");
        // 
        // textBoxu
        // 
        this.textBoxu.Location = new System.Drawing.Point(290, 128);
        this.textBoxu.Name = "textBoxu";
        this.textBoxu.Size = new System.Drawing.Size(50, 20);
        this.textBoxu.TabIndex = 21;
        this.toolTip1.SetToolTip(this.textBoxu, "Minimum throttle of the motors when they are turned on and the throttle stick is " +
                "at the bottom");
        this.textBoxu.TextChanged += new System.EventHandler(this.TextBoxuTextChanged);
        // 
        // labelu
        // 
        this.labelu.Location = new System.Drawing.Point(225, 131);
        this.labelu.Name = "labelu";
        this.labelu.Size = new System.Drawing.Size(50, 20);
        this.labelu.TabIndex = 20;
        this.labelu.Text = "---";
        // 
        // static40
        // 
        this.static40.Location = new System.Drawing.Point(15, 131);
        this.static40.Name = "static40";
        this.static40.Size = new System.Drawing.Size(186, 23);
        this.static40.TabIndex = 19;
        this.static40.Text = "Minimum throttle (idle up) [15]";
        // 
        // textBoxS
        // 
        this.textBoxS.Location = new System.Drawing.Point(290, 82);
        this.textBoxS.Name = "textBoxS";
        this.textBoxS.Size = new System.Drawing.Size(50, 20);
        this.textBoxS.TabIndex = 17;
        this.toolTip1.SetToolTip(this.textBoxS, "Yaw sensitivity (for both hover and acro mode)");
        this.textBoxS.TextChanged += new System.EventHandler(this.TextBoxSTextChanged);
        // 
        // textBoxR
        // 
        this.textBoxR.Location = new System.Drawing.Point(290, 59);
        this.textBoxR.Name = "textBoxR";
        this.textBoxR.Size = new System.Drawing.Size(50, 20);
        this.textBoxR.TabIndex = 16;
        this.toolTip1.SetToolTip(this.textBoxR, "Agility in hover mode");
        this.textBoxR.TextChanged += new System.EventHandler(this.TextBoxRTextChanged);
        // 
        // textBoxQ
        // 
        this.textBoxQ.Location = new System.Drawing.Point(290, 36);
        this.textBoxQ.Name = "textBoxQ";
        this.textBoxQ.Size = new System.Drawing.Size(50, 20);
        this.textBoxQ.TabIndex = 15;
        this.toolTip1.SetToolTip(this.textBoxQ, "Agility in acro mode");
        this.textBoxQ.TextChanged += new System.EventHandler(this.TextBoxQTextChanged);
        // 
        // checkBoxT
        // 
        this.checkBoxT.Checked = true;
        this.checkBoxT.CheckState = System.Windows.Forms.CheckState.Checked;
        this.checkBoxT.Enabled = false;
        this.checkBoxT.Location = new System.Drawing.Point(225, 105);
        this.checkBoxT.Name = "checkBoxT";
        this.checkBoxT.Size = new System.Drawing.Size(30, 20);
        this.checkBoxT.TabIndex = 14;
        this.checkBoxT.UseVisualStyleBackColor = true;
        // 
        // labels
        // 
        this.labels.Location = new System.Drawing.Point(225, 85);
        this.labels.Name = "labels";
        this.labels.Size = new System.Drawing.Size(50, 20);
        this.labels.TabIndex = 13;
        this.labels.Text = "---";
        // 
        // labelr
        // 
        this.labelr.Location = new System.Drawing.Point(225, 62);
        this.labelr.Name = "labelr";
        this.labelr.Size = new System.Drawing.Size(50, 20);
        this.labelr.TabIndex = 12;
        this.labelr.Text = "---";
        // 
        // labelq
        // 
        this.labelq.Location = new System.Drawing.Point(225, 39);
        this.labelq.Name = "labelq";
        this.labelq.Size = new System.Drawing.Size(50, 20);
        this.labelq.TabIndex = 11;
        this.labelq.Text = "---";
        // 
        // static15
        // 
        this.static15.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static15.Location = new System.Drawing.Point(290, 16);
        this.static15.Name = "static15";
        this.static15.Size = new System.Drawing.Size(84, 23);
        this.static15.TabIndex = 10;
        this.static15.Text = "New values";
        // 
        // static13
        // 
        this.static13.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static13.Location = new System.Drawing.Point(15, 16);
        this.static13.Name = "static13";
        this.static13.Size = new System.Drawing.Size(115, 23);
        this.static13.TabIndex = 9;
        this.static13.Text = "Parameter [default]";
        // 
        // static14
        // 
        this.static14.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static14.Location = new System.Drawing.Point(205, 16);
        this.static14.Name = "static14";
        this.static14.Size = new System.Drawing.Size(79, 23);
        this.static14.TabIndex = 8;
        this.static14.Text = "Values in µC";
        // 
        // static12
        // 
        this.static12.Location = new System.Drawing.Point(15, 108);
        this.static12.Name = "static12";
        this.static12.Size = new System.Drawing.Size(198, 23);
        this.static12.TabIndex = 5;
        this.static12.Text = "ACRO: Exponential agility boost [0]";
        // 
        // static11
        // 
        this.static11.Location = new System.Drawing.Point(15, 85);
        this.static11.Name = "static11";
        this.static11.Size = new System.Drawing.Size(198, 23);
        this.static11.TabIndex = 4;
        this.static11.Text = "Yaw stick sensitivity [130]";
        // 
        // checkBoxT2
        // 
        this.checkBoxT2.Checked = true;
        this.checkBoxT2.CheckState = System.Windows.Forms.CheckState.Checked;
        this.checkBoxT2.Location = new System.Drawing.Point(290, 105);
        this.checkBoxT2.Name = "checkBoxT2";
        this.checkBoxT2.Size = new System.Drawing.Size(50, 20);
        this.checkBoxT2.TabIndex = 6;
        this.checkBoxT2.Text = "on";
        this.toolTip1.SetToolTip(this.checkBoxT2, "Exponential increase in agility when roll or yaw sticks move out of the centre");
        this.checkBoxT2.UseVisualStyleBackColor = true;
        // 
        // static9
        // 
        this.static9.Location = new System.Drawing.Point(15, 39);
        this.static9.Name = "static9";
        this.static9.Size = new System.Drawing.Size(198, 23);
        this.static9.TabIndex = 2;
        this.static9.Text = "ACRO: Stick sensitivity roll + nick [130]";
        // 
        // static10
        // 
        this.static10.Location = new System.Drawing.Point(15, 62);
        this.static10.Name = "static10";
        this.static10.Size = new System.Drawing.Size(200, 23);
        this.static10.TabIndex = 3;
        this.static10.Text = "HOVER: Stick sensitivity roll + nick [70]";
        // 
        // tabPage3
        // 
        this.tabPage3.Controls.Add(this.groupBox6);
        this.tabPage3.Controls.Add(this.groupBox5);
        this.tabPage3.Controls.Add(this.groupBox4);
        this.tabPage3.Controls.Add(this.groupBox3);
        this.tabPage3.Controls.Add(this.groupBox1);
        this.tabPage3.Location = new System.Drawing.Point(4, 22);
        this.tabPage3.Name = "tabPage3";
        this.tabPage3.Size = new System.Drawing.Size(649, 366);
        this.tabPage3.TabIndex = 2;
        this.tabPage3.Text = "PID loop";
        this.tabPage3.UseVisualStyleBackColor = true;
        // 
        // groupBox6
        // 
        this.groupBox6.Controls.Add(this.textBoxx);
        this.groupBox6.Controls.Add(this.textBoxw);
        this.groupBox6.Controls.Add(this.labelx);
        this.groupBox6.Controls.Add(this.labelw);
        this.groupBox6.Controls.Add(this.static46);
        this.groupBox6.Controls.Add(this.static45);
        this.groupBox6.Controls.Add(this.textBoxp);
        this.groupBox6.Controls.Add(this.textBoxo);
        this.groupBox6.Controls.Add(this.textBoxn);
        this.groupBox6.Controls.Add(this.labelp);
        this.groupBox6.Controls.Add(this.labelo);
        this.groupBox6.Controls.Add(this.labeln);
        this.groupBox6.Controls.Add(this.static39);
        this.groupBox6.Controls.Add(this.static38);
        this.groupBox6.Controls.Add(this.static37);
        this.groupBox6.Controls.Add(this.static36);
        this.groupBox6.Controls.Add(this.static35);
        this.groupBox6.Controls.Add(this.static34);
        this.groupBox6.Location = new System.Drawing.Point(3, 180);
        this.groupBox6.Name = "groupBox6";
        this.groupBox6.Size = new System.Drawing.Size(267, 182);
        this.groupBox6.TabIndex = 4;
        this.groupBox6.TabStop = false;
        this.groupBox6.Text = "ACC settings";
        // 
        // textBoxx
        // 
        this.textBoxx.Location = new System.Drawing.Point(193, 130);
        this.textBoxx.Name = "textBoxx";
        this.textBoxx.Size = new System.Drawing.Size(50, 20);
        this.textBoxx.TabIndex = 25;
        this.toolTip1.SetToolTip(this.textBoxx, "Y acc offset determines at which angle the acc thinks it is horizontal. If your c" +
                "opter doesn\'t stay in place, you need to tune this parameter.");
        this.textBoxx.TextChanged += new System.EventHandler(this.TextBoxxTextChanged);
        // 
        // textBoxw
        // 
        this.textBoxw.Location = new System.Drawing.Point(193, 107);
        this.textBoxw.Name = "textBoxw";
        this.textBoxw.Size = new System.Drawing.Size(50, 20);
        this.textBoxw.TabIndex = 24;
        this.toolTip1.SetToolTip(this.textBoxw, "X acc offset determines at which angle the acc thinks it is horizontal. If your c" +
                "opter doesn\'t stay in place, you need to tune this parameter.");
        this.textBoxw.TextChanged += new System.EventHandler(this.TextBoxwTextChanged);
        // 
        // labelx
        // 
        this.labelx.Location = new System.Drawing.Point(126, 133);
        this.labelx.Name = "labelx";
        this.labelx.Size = new System.Drawing.Size(50, 20);
        this.labelx.TabIndex = 23;
        this.labelx.Text = "---";
        // 
        // labelw
        // 
        this.labelw.Location = new System.Drawing.Point(126, 110);
        this.labelw.Name = "labelw";
        this.labelw.Size = new System.Drawing.Size(50, 20);
        this.labelw.TabIndex = 22;
        this.labelw.Text = "---";
        // 
        // static46
        // 
        this.static46.Location = new System.Drawing.Point(8, 133);
        this.static46.Name = "static46";
        this.static46.Size = new System.Drawing.Size(100, 23);
        this.static46.TabIndex = 21;
        this.static46.Text = "Y acc offset [128]";
        // 
        // static45
        // 
        this.static45.Location = new System.Drawing.Point(8, 110);
        this.static45.Name = "static45";
        this.static45.Size = new System.Drawing.Size(100, 23);
        this.static45.TabIndex = 20;
        this.static45.Text = "X acc offset [128]";
        // 
        // textBoxp
        // 
        this.textBoxp.Location = new System.Drawing.Point(193, 84);
        this.textBoxp.Name = "textBoxp";
        this.textBoxp.Size = new System.Drawing.Size(50, 20);
        this.textBoxp.TabIndex = 19;
        this.toolTip1.SetToolTip(this.textBoxp, "Scale factor of the acc, the acc has to have a similar amplitude as the gyro inte" +
                "gral");
        this.textBoxp.TextChanged += new System.EventHandler(this.TextBoxpTextChanged);
        // 
        // textBoxo
        // 
        this.textBoxo.Location = new System.Drawing.Point(193, 61);
        this.textBoxo.Name = "textBoxo";
        this.textBoxo.Size = new System.Drawing.Size(50, 20);
        this.textBoxo.TabIndex = 18;
        this.toolTip1.SetToolTip(this.textBoxo, "Scale factor of the acc, the acc has to have a similar amplitude as the gyro inte" +
                "gral");
        this.textBoxo.TextChanged += new System.EventHandler(this.TextBoxoTextChanged);
        // 
        // textBoxn
        // 
        this.textBoxn.Location = new System.Drawing.Point(193, 38);
        this.textBoxn.Name = "textBoxn";
        this.textBoxn.Size = new System.Drawing.Size(50, 20);
        this.textBoxn.TabIndex = 17;
        this.toolTip1.SetToolTip(this.textBoxn, "Determines how much the gyro integral is influenced by the acc");
        this.textBoxn.TextChanged += new System.EventHandler(this.TextBoxnTextChanged);
        // 
        // labelp
        // 
        this.labelp.Location = new System.Drawing.Point(126, 87);
        this.labelp.Name = "labelp";
        this.labelp.Size = new System.Drawing.Size(50, 20);
        this.labelp.TabIndex = 16;
        this.labelp.Text = "---";
        // 
        // labelo
        // 
        this.labelo.Location = new System.Drawing.Point(126, 64);
        this.labelo.Name = "labelo";
        this.labelo.Size = new System.Drawing.Size(50, 20);
        this.labelo.TabIndex = 15;
        this.labelo.Text = "---";
        // 
        // labeln
        // 
        this.labeln.Location = new System.Drawing.Point(126, 41);
        this.labeln.Name = "labeln";
        this.labeln.Size = new System.Drawing.Size(50, 20);
        this.labeln.TabIndex = 14;
        this.labeln.Text = "---";
        // 
        // static39
        // 
        this.static39.Location = new System.Drawing.Point(8, 87);
        this.static39.Name = "static39";
        this.static39.Size = new System.Drawing.Size(100, 23);
        this.static39.TabIndex = 13;
        this.static39.Text = "Y acc scale [140]";
        // 
        // static38
        // 
        this.static38.Location = new System.Drawing.Point(8, 64);
        this.static38.Name = "static38";
        this.static38.Size = new System.Drawing.Size(100, 23);
        this.static38.TabIndex = 12;
        this.static38.Text = "X acc scale [140]";
        // 
        // static37
        // 
        this.static37.Location = new System.Drawing.Point(8, 41);
        this.static37.Name = "static37";
        this.static37.Size = new System.Drawing.Size(100, 23);
        this.static37.TabIndex = 11;
        this.static37.Text = "ACC influence [10]";
        // 
        // static36
        // 
        this.static36.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static36.Location = new System.Drawing.Point(193, 14);
        this.static36.Name = "static36";
        this.static36.Size = new System.Drawing.Size(74, 23);
        this.static36.TabIndex = 10;
        this.static36.Text = "New values";
        // 
        // static35
        // 
        this.static35.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static35.Location = new System.Drawing.Point(113, 14);
        this.static35.Name = "static35";
        this.static35.Size = new System.Drawing.Size(79, 23);
        this.static35.TabIndex = 8;
        this.static35.Text = "Values in µC";
        // 
        // static34
        // 
        this.static34.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static34.Location = new System.Drawing.Point(8, 14);
        this.static34.Name = "static34";
        this.static34.Size = new System.Drawing.Size(115, 23);
        this.static34.TabIndex = 9;
        this.static34.Text = "Parameter [default]";
        // 
        // groupBox5
        // 
        this.groupBox5.Controls.Add(this.trackBar15);
        this.groupBox5.Controls.Add(this.checkBox3);
        this.groupBox5.Controls.Add(this.label14);
        this.groupBox5.Controls.Add(this.label25);
        this.groupBox5.Controls.Add(this.label26);
        this.groupBox5.Controls.Add(this.textBoxm);
        this.groupBox5.Controls.Add(this.textBoxl);
        this.groupBox5.Controls.Add(this.labelm);
        this.groupBox5.Controls.Add(this.labell);
        this.groupBox5.Controls.Add(this.static33);
        this.groupBox5.Controls.Add(this.static32);
        this.groupBox5.Controls.Add(this.static29);
        this.groupBox5.Controls.Add(this.static31);
        this.groupBox5.Controls.Add(this.static30);
        this.groupBox5.Location = new System.Drawing.Point(276, 267);
        this.groupBox5.Name = "groupBox5";
        this.groupBox5.Size = new System.Drawing.Size(370, 95);
        this.groupBox5.TabIndex = 3;
        this.groupBox5.TabStop = false;
        this.groupBox5.Text = "Yaw PI";
        // 
        // trackBar15
        // 
        this.trackBar15.BackColor = System.Drawing.SystemColors.Window;
        this.trackBar15.LargeChange = 2;
        this.trackBar15.Location = new System.Drawing.Point(325, 30);
        this.trackBar15.Margin = new System.Windows.Forms.Padding(1);
        this.trackBar15.Name = "trackBar15";
        this.trackBar15.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.trackBar15.Size = new System.Drawing.Size(42, 61);
        this.trackBar15.TabIndex = 21;
        this.trackBar15.TickStyle = System.Windows.Forms.TickStyle.TopLeft;
        this.trackBar15.Value = 6;
        this.trackBar15.Scroll += new System.EventHandler(this.TrackBar15Scroll);
        // 
        // checkBox3
        // 
        this.checkBox3.Location = new System.Drawing.Point(335, 10);
        this.checkBox3.Name = "checkBox3";
        this.checkBox3.Size = new System.Drawing.Size(20, 24);
        this.checkBox3.TabIndex = 20;
        this.checkBox3.UseVisualStyleBackColor = true;
        this.checkBox3.CheckedChanged += new System.EventHandler(this.CheckBox3CheckedChanged);
        // 
        // label14
        // 
        this.label14.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.label14.Location = new System.Drawing.Point(259, 34);
        this.label14.Name = "label14";
        this.label14.Size = new System.Drawing.Size(63, 23);
        this.label14.TabIndex = 18;
        this.label14.Text = "high sens";
        this.label14.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
        // 
        // label25
        // 
        this.label25.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.label25.Location = new System.Drawing.Point(259, 69);
        this.label25.Name = "label25";
        this.label25.Size = new System.Drawing.Size(63, 23);
        this.label25.TabIndex = 19;
        this.label25.Text = "low sens";
        this.label25.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
        // 
        // label26
        // 
        this.label26.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.label26.Location = new System.Drawing.Point(266, 16);
        this.label26.Name = "label26";
        this.label26.Size = new System.Drawing.Size(63, 23);
        this.label26.TabIndex = 17;
        this.label26.Text = "Easy mode";
        // 
        // textBoxm
        // 
        this.textBoxm.Location = new System.Drawing.Point(190, 53);
        this.textBoxm.Name = "textBoxm";
        this.textBoxm.Size = new System.Drawing.Size(50, 20);
        this.textBoxm.TabIndex = 16;
        this.toolTip1.SetToolTip(this.textBoxm, "I of yaw: to sum of the errors in angular velocity");
        this.textBoxm.TextChanged += new System.EventHandler(this.TextBoxmTextChanged);
        // 
        // textBoxl
        // 
        this.textBoxl.Location = new System.Drawing.Point(190, 30);
        this.textBoxl.Name = "textBoxl";
        this.textBoxl.Size = new System.Drawing.Size(50, 20);
        this.textBoxl.TabIndex = 15;
        this.toolTip1.SetToolTip(this.textBoxl, "P of yaw: reacts to errors in yaw velocity");
        this.textBoxl.TextChanged += new System.EventHandler(this.TextBoxlTextChanged);
        // 
        // labelm
        // 
        this.labelm.Location = new System.Drawing.Point(113, 56);
        this.labelm.Name = "labelm";
        this.labelm.Size = new System.Drawing.Size(50, 20);
        this.labelm.TabIndex = 14;
        this.labelm.Text = "---";
        // 
        // labell
        // 
        this.labell.Location = new System.Drawing.Point(113, 33);
        this.labell.Name = "labell";
        this.labell.Size = new System.Drawing.Size(50, 20);
        this.labell.TabIndex = 13;
        this.labell.Text = "---";
        // 
        // static33
        // 
        this.static33.Location = new System.Drawing.Point(7, 56);
        this.static33.Name = "static33";
        this.static33.Size = new System.Drawing.Size(100, 23);
        this.static33.TabIndex = 12;
        this.static33.Text = "I [70]";
        // 
        // static32
        // 
        this.static32.Location = new System.Drawing.Point(7, 33);
        this.static32.Name = "static32";
        this.static32.Size = new System.Drawing.Size(100, 23);
        this.static32.TabIndex = 11;
        this.static32.Text = "P [100]";
        // 
        // static29
        // 
        this.static29.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static29.Location = new System.Drawing.Point(6, 16);
        this.static29.Name = "static29";
        this.static29.Size = new System.Drawing.Size(107, 23);
        this.static29.TabIndex = 9;
        this.static29.Text = "Parameter [default]";
        // 
        // static31
        // 
        this.static31.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static31.Location = new System.Drawing.Point(188, 16);
        this.static31.Name = "static31";
        this.static31.Size = new System.Drawing.Size(100, 23);
        this.static31.TabIndex = 10;
        this.static31.Text = "New values";
        // 
        // static30
        // 
        this.static30.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static30.Location = new System.Drawing.Point(113, 16);
        this.static30.Name = "static30";
        this.static30.Size = new System.Drawing.Size(88, 23);
        this.static30.TabIndex = 8;
        this.static30.Text = "Values in µC";
        // 
        // groupBox4
        // 
        this.groupBox4.Controls.Add(this.checkBoxf2);
        this.groupBox4.Controls.Add(this.checkBoxf);
        this.groupBox4.Controls.Add(this.checkBoxe2);
        this.groupBox4.Controls.Add(this.checkBoxe);
        this.groupBox4.Controls.Add(this.static28);
        this.groupBox4.Controls.Add(this.static27);
        this.groupBox4.Controls.Add(this.checkBoxD2);
        this.groupBox4.Controls.Add(this.checkBoxB2);
        this.groupBox4.Controls.Add(this.checkBoxC2);
        this.groupBox4.Controls.Add(this.checkBoxD);
        this.groupBox4.Controls.Add(this.checkBoxC);
        this.groupBox4.Controls.Add(this.checkBoxB);
        this.groupBox4.Controls.Add(this.static26);
        this.groupBox4.Controls.Add(this.static25);
        this.groupBox4.Controls.Add(this.static24);
        this.groupBox4.Controls.Add(this.static22);
        this.groupBox4.Controls.Add(this.static23);
        this.groupBox4.Controls.Add(this.static21);
        this.groupBox4.Location = new System.Drawing.Point(3, 3);
        this.groupBox4.Name = "groupBox4";
        this.groupBox4.Size = new System.Drawing.Size(267, 171);
        this.groupBox4.TabIndex = 2;
        this.groupBox4.TabStop = false;
        this.groupBox4.Text = "Directions";
        // 
        // checkBoxf2
        // 
        this.checkBoxf2.Location = new System.Drawing.Point(199, 136);
        this.checkBoxf2.Name = "checkBoxf2";
        this.checkBoxf2.Size = new System.Drawing.Size(50, 20);
        this.checkBoxf2.TabIndex = 21;
        this.toolTip1.SetToolTip(this.checkBoxf2, "Reverse Y acc");
        this.checkBoxf2.UseVisualStyleBackColor = true;
        // 
        // checkBoxf
        // 
        this.checkBoxf.Enabled = false;
        this.checkBoxf.Location = new System.Drawing.Point(142, 136);
        this.checkBoxf.Name = "checkBoxf";
        this.checkBoxf.Size = new System.Drawing.Size(20, 20);
        this.checkBoxf.TabIndex = 20;
        this.checkBoxf.UseVisualStyleBackColor = true;
        // 
        // checkBoxe2
        // 
        this.checkBoxe2.Location = new System.Drawing.Point(199, 109);
        this.checkBoxe2.Name = "checkBoxe2";
        this.checkBoxe2.Size = new System.Drawing.Size(50, 20);
        this.checkBoxe2.TabIndex = 19;
        this.toolTip1.SetToolTip(this.checkBoxe2, "Reverse X acc");
        this.checkBoxe2.UseVisualStyleBackColor = true;
        // 
        // checkBoxe
        // 
        this.checkBoxe.Enabled = false;
        this.checkBoxe.Location = new System.Drawing.Point(142, 109);
        this.checkBoxe.Name = "checkBoxe";
        this.checkBoxe.Size = new System.Drawing.Size(20, 20);
        this.checkBoxe.TabIndex = 18;
        this.checkBoxe.UseVisualStyleBackColor = true;
        // 
        // static28
        // 
        this.static28.Location = new System.Drawing.Point(7, 139);
        this.static28.Name = "static28";
        this.static28.Size = new System.Drawing.Size(100, 23);
        this.static28.TabIndex = 17;
        this.static28.Text = "Y acc reverse [0]";
        // 
        // static27
        // 
        this.static27.Location = new System.Drawing.Point(7, 112);
        this.static27.Name = "static27";
        this.static27.Size = new System.Drawing.Size(100, 23);
        this.static27.TabIndex = 16;
        this.static27.Text = "X acc reverse [0]";
        // 
        // checkBoxD2
        // 
        this.checkBoxD2.Location = new System.Drawing.Point(199, 82);
        this.checkBoxD2.Name = "checkBoxD2";
        this.checkBoxD2.Size = new System.Drawing.Size(50, 20);
        this.checkBoxD2.TabIndex = 15;
        this.toolTip1.SetToolTip(this.checkBoxD2, "Reverse yaw gyro");
        this.checkBoxD2.UseVisualStyleBackColor = true;
        // 
        // checkBoxB2
        // 
        this.checkBoxB2.Location = new System.Drawing.Point(199, 33);
        this.checkBoxB2.Name = "checkBoxB2";
        this.checkBoxB2.Size = new System.Drawing.Size(50, 20);
        this.checkBoxB2.TabIndex = 14;
        this.toolTip1.SetToolTip(this.checkBoxB2, "Reverse roll gyro");
        this.checkBoxB2.UseVisualStyleBackColor = true;
        // 
        // checkBoxC2
        // 
        this.checkBoxC2.Location = new System.Drawing.Point(199, 59);
        this.checkBoxC2.Name = "checkBoxC2";
        this.checkBoxC2.Size = new System.Drawing.Size(50, 20);
        this.checkBoxC2.TabIndex = 13;
        this.toolTip1.SetToolTip(this.checkBoxC2, "Reverse nick gyro");
        this.checkBoxC2.UseVisualStyleBackColor = true;
        // 
        // checkBoxD
        // 
        this.checkBoxD.Enabled = false;
        this.checkBoxD.Location = new System.Drawing.Point(142, 83);
        this.checkBoxD.Name = "checkBoxD";
        this.checkBoxD.Size = new System.Drawing.Size(20, 20);
        this.checkBoxD.TabIndex = 11;
        this.checkBoxD.UseVisualStyleBackColor = true;
        // 
        // checkBoxC
        // 
        this.checkBoxC.Enabled = false;
        this.checkBoxC.Location = new System.Drawing.Point(142, 59);
        this.checkBoxC.Name = "checkBoxC";
        this.checkBoxC.Size = new System.Drawing.Size(20, 20);
        this.checkBoxC.TabIndex = 10;
        this.checkBoxC.UseVisualStyleBackColor = true;
        // 
        // checkBoxB
        // 
        this.checkBoxB.Enabled = false;
        this.checkBoxB.Location = new System.Drawing.Point(142, 33);
        this.checkBoxB.Name = "checkBoxB";
        this.checkBoxB.Size = new System.Drawing.Size(20, 20);
        this.checkBoxB.TabIndex = 9;
        this.checkBoxB.UseVisualStyleBackColor = true;
        // 
        // static26
        // 
        this.static26.Location = new System.Drawing.Point(7, 85);
        this.static26.Name = "static26";
        this.static26.Size = new System.Drawing.Size(114, 23);
        this.static26.TabIndex = 8;
        this.static26.Text = "Yaw gyro reverse [0]";
        // 
        // static25
        // 
        this.static25.Location = new System.Drawing.Point(7, 61);
        this.static25.Name = "static25";
        this.static25.Size = new System.Drawing.Size(114, 23);
        this.static25.TabIndex = 8;
        this.static25.Text = "Nick gyro reverse [0]";
        // 
        // static24
        // 
        this.static24.Location = new System.Drawing.Point(7, 36);
        this.static24.Name = "static24";
        this.static24.Size = new System.Drawing.Size(114, 23);
        this.static24.TabIndex = 8;
        this.static24.Text = "Roll gyro reverse [0]";
        // 
        // static22
        // 
        this.static22.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static22.Location = new System.Drawing.Point(117, 16);
        this.static22.Name = "static22";
        this.static22.Size = new System.Drawing.Size(75, 23);
        this.static22.TabIndex = 6;
        this.static22.Text = "Values in µC";
        // 
        // static23
        // 
        this.static23.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static23.Location = new System.Drawing.Point(193, 16);
        this.static23.Name = "static23";
        this.static23.Size = new System.Drawing.Size(74, 23);
        this.static23.TabIndex = 7;
        this.static23.Text = "New values";
        // 
        // static21
        // 
        this.static21.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static21.Location = new System.Drawing.Point(6, 16);
        this.static21.Name = "static21";
        this.static21.Size = new System.Drawing.Size(115, 23);
        this.static21.TabIndex = 6;
        this.static21.Text = "Parameter [default]";
        // 
        // groupBox3
        // 
        this.groupBox3.Controls.Add(this.checkBox2);
        this.groupBox3.Controls.Add(this.trackBar14);
        this.groupBox3.Controls.Add(this.label24);
        this.groupBox3.Controls.Add(this.textBoxy);
        this.groupBox3.Controls.Add(this.label23);
        this.groupBox3.Controls.Add(this.textBoxH);
        this.groupBox3.Controls.Add(this.textBoxG);
        this.groupBox3.Controls.Add(this.labely);
        this.groupBox3.Controls.Add(this.labelH);
        this.groupBox3.Controls.Add(this.label16);
        this.groupBox3.Controls.Add(this.labelG);
        this.groupBox3.Controls.Add(this.static17);
        this.groupBox3.Controls.Add(this.label22);
        this.groupBox3.Controls.Add(this.static16);
        this.groupBox3.Controls.Add(this.static18);
        this.groupBox3.Controls.Add(this.static20);
        this.groupBox3.Controls.Add(this.static19);
        this.groupBox3.Location = new System.Drawing.Point(276, 145);
        this.groupBox3.Name = "groupBox3";
        this.groupBox3.Size = new System.Drawing.Size(370, 116);
        this.groupBox3.TabIndex = 1;
        this.groupBox3.TabStop = false;
        this.groupBox3.Text = "ACRO Mode PID";
        // 
        // checkBox2
        // 
        this.checkBox2.Location = new System.Drawing.Point(335, 10);
        this.checkBox2.Name = "checkBox2";
        this.checkBox2.Size = new System.Drawing.Size(20, 24);
        this.checkBox2.TabIndex = 12;
        this.checkBox2.UseVisualStyleBackColor = true;
        this.checkBox2.CheckedChanged += new System.EventHandler(this.CheckBox2CheckedChanged);
        // 
        // trackBar14
        // 
        this.trackBar14.BackColor = System.Drawing.SystemColors.Window;
        this.trackBar14.LargeChange = 2;
        this.trackBar14.Location = new System.Drawing.Point(325, 36);
        this.trackBar14.Margin = new System.Windows.Forms.Padding(0);
        this.trackBar14.Name = "trackBar14";
        this.trackBar14.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.trackBar14.Size = new System.Drawing.Size(42, 77);
        this.trackBar14.TabIndex = 11;
        this.trackBar14.TickStyle = System.Windows.Forms.TickStyle.TopLeft;
        this.trackBar14.Value = 5;
        this.trackBar14.Scroll += new System.EventHandler(this.TrackBar14Scroll);
        // 
        // label24
        // 
        this.label24.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.label24.Location = new System.Drawing.Point(259, 39);
        this.label24.Name = "label24";
        this.label24.Size = new System.Drawing.Size(63, 23);
        this.label24.TabIndex = 10;
        this.label24.Text = "high sens";
        this.label24.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
        // 
        // textBoxy
        // 
        this.textBoxy.Location = new System.Drawing.Point(190, 83);
        this.textBoxy.Name = "textBoxy";
        this.textBoxy.Size = new System.Drawing.Size(50, 20);
        this.textBoxy.TabIndex = 10;
        this.toolTip1.SetToolTip(this.textBoxy, "D in acro mode: reacts to changes in angular acceleration");
        this.textBoxy.TextChanged += new System.EventHandler(this.TextBoxHTextChanged);
        // 
        // label23
        // 
        this.label23.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.label23.Location = new System.Drawing.Point(259, 89);
        this.label23.Name = "label23";
        this.label23.Size = new System.Drawing.Size(63, 23);
        this.label23.TabIndex = 10;
        this.label23.Text = "low sens";
        this.label23.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
        // 
        // textBoxH
        // 
        this.textBoxH.Location = new System.Drawing.Point(190, 59);
        this.textBoxH.Name = "textBoxH";
        this.textBoxH.Size = new System.Drawing.Size(50, 20);
        this.textBoxH.TabIndex = 10;
        this.toolTip1.SetToolTip(this.textBoxH, "I in acro mode: The sum of the errors in angular velocity");
        this.textBoxH.TextChanged += new System.EventHandler(this.TextBoxHTextChanged);
        // 
        // textBoxG
        // 
        this.textBoxG.Location = new System.Drawing.Point(190, 36);
        this.textBoxG.Name = "textBoxG";
        this.textBoxG.Size = new System.Drawing.Size(50, 20);
        this.textBoxG.TabIndex = 9;
        this.toolTip1.SetToolTip(this.textBoxG, "P in acro mode: Reacts to errors in angular velocity");
        this.textBoxG.TextChanged += new System.EventHandler(this.TextBoxGTextChanged);
        // 
        // labely
        // 
        this.labely.Location = new System.Drawing.Point(113, 86);
        this.labely.Name = "labely";
        this.labely.Size = new System.Drawing.Size(50, 20);
        this.labely.TabIndex = 8;
        this.labely.Text = "---";
        // 
        // labelH
        // 
        this.labelH.Location = new System.Drawing.Point(113, 62);
        this.labelH.Name = "labelH";
        this.labelH.Size = new System.Drawing.Size(50, 20);
        this.labelH.TabIndex = 8;
        this.labelH.Text = "---";
        // 
        // label16
        // 
        this.label16.Location = new System.Drawing.Point(8, 86);
        this.label16.Name = "label16";
        this.label16.Size = new System.Drawing.Size(51, 23);
        this.label16.TabIndex = 1;
        this.label16.Text = "D [15]";
        // 
        // labelG
        // 
        this.labelG.Location = new System.Drawing.Point(113, 39);
        this.labelG.Name = "labelG";
        this.labelG.Size = new System.Drawing.Size(50, 20);
        this.labelG.TabIndex = 2;
        this.labelG.Text = "---";
        // 
        // static17
        // 
        this.static17.Location = new System.Drawing.Point(8, 62);
        this.static17.Name = "static17";
        this.static17.Size = new System.Drawing.Size(51, 23);
        this.static17.TabIndex = 1;
        this.static17.Text = "I [30]";
        // 
        // label22
        // 
        this.label22.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.label22.Location = new System.Drawing.Point(266, 16);
        this.label22.Name = "label22";
        this.label22.Size = new System.Drawing.Size(63, 23);
        this.label22.TabIndex = 7;
        this.label22.Text = "Easy mode";
        // 
        // static16
        // 
        this.static16.Location = new System.Drawing.Point(8, 39);
        this.static16.Name = "static16";
        this.static16.Size = new System.Drawing.Size(51, 23);
        this.static16.TabIndex = 0;
        this.static16.Text = "P [130]";
        // 
        // static18
        // 
        this.static18.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static18.Location = new System.Drawing.Point(8, 16);
        this.static18.Name = "static18";
        this.static18.Size = new System.Drawing.Size(107, 23);
        this.static18.TabIndex = 6;
        this.static18.Text = "Parameter [default]";
        // 
        // static20
        // 
        this.static20.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static20.Location = new System.Drawing.Point(190, 16);
        this.static20.Name = "static20";
        this.static20.Size = new System.Drawing.Size(74, 23);
        this.static20.TabIndex = 7;
        this.static20.Text = "New values";
        // 
        // static19
        // 
        this.static19.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static19.Location = new System.Drawing.Point(113, 16);
        this.static19.Name = "static19";
        this.static19.Size = new System.Drawing.Size(79, 23);
        this.static19.TabIndex = 6;
        this.static19.Text = "Values in µC";
        // 
        // groupBox1
        // 
        this.groupBox1.BackColor = System.Drawing.Color.Transparent;
        this.groupBox1.Controls.Add(this.label21);
        this.groupBox1.Controls.Add(this.label20);
        this.groupBox1.Controls.Add(this.checkBox1);
        this.groupBox1.Controls.Add(this.trackBar13);
        this.groupBox1.Controls.Add(this.label18);
        this.groupBox1.Controls.Add(this.static6);
        this.groupBox1.Controls.Add(this.static5);
        this.groupBox1.Controls.Add(this.static4);
        this.groupBox1.Controls.Add(this.textBoxi);
        this.groupBox1.Controls.Add(this.label19);
        this.groupBox1.Controls.Add(this.static3);
        this.groupBox1.Controls.Add(this.labeli);
        this.groupBox1.Controls.Add(this.static7);
        this.groupBox1.Controls.Add(this.static2);
        this.groupBox1.Controls.Add(this.labelj);
        this.groupBox1.Controls.Add(this.textBoxz);
        this.groupBox1.Controls.Add(this.textBoxk);
        this.groupBox1.Controls.Add(this.labelz);
        this.groupBox1.Controls.Add(this.labelk);
        this.groupBox1.Controls.Add(this.textBoxj);
        this.groupBox1.Location = new System.Drawing.Point(276, 3);
        this.groupBox1.Name = "groupBox1";
        this.groupBox1.Size = new System.Drawing.Size(370, 136);
        this.groupBox1.TabIndex = 0;
        this.groupBox1.TabStop = false;
        this.groupBox1.Text = "HOVER Mode PIDD²";
        // 
        // label21
        // 
        this.label21.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.label21.Location = new System.Drawing.Point(259, 38);
        this.label21.Name = "label21";
        this.label21.Size = new System.Drawing.Size(63, 23);
        this.label21.TabIndex = 10;
        this.label21.Text = "high sens";
        this.label21.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
        // 
        // label20
        // 
        this.label20.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.label20.Location = new System.Drawing.Point(259, 105);
        this.label20.Name = "label20";
        this.label20.Size = new System.Drawing.Size(63, 23);
        this.label20.TabIndex = 10;
        this.label20.Text = "low sens";
        this.label20.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
        // 
        // checkBox1
        // 
        this.checkBox1.Location = new System.Drawing.Point(335, 12);
        this.checkBox1.Name = "checkBox1";
        this.checkBox1.Size = new System.Drawing.Size(20, 20);
        this.checkBox1.TabIndex = 9;
        this.toolTip1.SetToolTip(this.checkBox1, "Easy mode gives you less options in changing the parameters.");
        this.checkBox1.UseVisualStyleBackColor = true;
        this.checkBox1.CheckedChanged += new System.EventHandler(this.CheckBox1CheckedChanged);
        // 
        // trackBar13
        // 
        this.trackBar13.BackColor = System.Drawing.SystemColors.Window;
        this.trackBar13.LargeChange = 2;
        this.trackBar13.Location = new System.Drawing.Point(325, 38);
        this.trackBar13.Margin = new System.Windows.Forms.Padding(0);
        this.trackBar13.Name = "trackBar13";
        this.trackBar13.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.trackBar13.Size = new System.Drawing.Size(42, 90);
        this.trackBar13.TabIndex = 9;
        this.trackBar13.TickStyle = System.Windows.Forms.TickStyle.TopLeft;
        this.trackBar13.Value = 6;
        this.trackBar13.Scroll += new System.EventHandler(this.TrackBar13Scroll);
        // 
        // label18
        // 
        this.label18.Location = new System.Drawing.Point(8, 109);
        this.label18.Name = "label18";
        this.label18.Size = new System.Drawing.Size(51, 23);
        this.label18.TabIndex = 8;
        this.label18.Text = "D² [10]";
        // 
        // static6
        // 
        this.static6.Location = new System.Drawing.Point(8, 85);
        this.static6.Name = "static6";
        this.static6.Size = new System.Drawing.Size(51, 23);
        this.static6.TabIndex = 8;
        this.static6.Text = "D [80]";
        // 
        // static5
        // 
        this.static5.Location = new System.Drawing.Point(8, 62);
        this.static5.Name = "static5";
        this.static5.Size = new System.Drawing.Size(51, 23);
        this.static5.TabIndex = 8;
        this.static5.Text = "I [190]";
        // 
        // static4
        // 
        this.static4.Location = new System.Drawing.Point(8, 39);
        this.static4.Name = "static4";
        this.static4.Size = new System.Drawing.Size(51, 23);
        this.static4.TabIndex = 8;
        this.static4.Text = "P [130]";
        // 
        // textBoxi
        // 
        this.textBoxi.Location = new System.Drawing.Point(190, 36);
        this.textBoxi.Name = "textBoxi";
        this.textBoxi.Size = new System.Drawing.Size(50, 20);
        this.textBoxi.TabIndex = 3;
        this.toolTip1.SetToolTip(this.textBoxi, "P in hover mode: reacts to errors in angle");
        this.textBoxi.TextChanged += new System.EventHandler(this.TextBoxiTextChanged);
        // 
        // label19
        // 
        this.label19.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.label19.Location = new System.Drawing.Point(266, 16);
        this.label19.Name = "label19";
        this.label19.Size = new System.Drawing.Size(63, 23);
        this.label19.TabIndex = 7;
        this.label19.Text = "Easy mode";
        // 
        // static3
        // 
        this.static3.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static3.Location = new System.Drawing.Point(190, 16);
        this.static3.Name = "static3";
        this.static3.Size = new System.Drawing.Size(74, 23);
        this.static3.TabIndex = 7;
        this.static3.Text = "New values";
        // 
        // labeli
        // 
        this.labeli.Location = new System.Drawing.Point(113, 39);
        this.labeli.Name = "labeli";
        this.labeli.Size = new System.Drawing.Size(50, 20);
        this.labeli.TabIndex = 0;
        this.labeli.Text = "---";
        // 
        // static7
        // 
        this.static7.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static7.Location = new System.Drawing.Point(8, 16);
        this.static7.Name = "static7";
        this.static7.Size = new System.Drawing.Size(107, 23);
        this.static7.TabIndex = 6;
        this.static7.Text = "Parameter [default]";
        // 
        // static2
        // 
        this.static2.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static2.Location = new System.Drawing.Point(113, 16);
        this.static2.Name = "static2";
        this.static2.Size = new System.Drawing.Size(79, 23);
        this.static2.TabIndex = 6;
        this.static2.Text = "Values in µC";
        // 
        // labelj
        // 
        this.labelj.Location = new System.Drawing.Point(113, 62);
        this.labelj.Name = "labelj";
        this.labelj.Size = new System.Drawing.Size(50, 20);
        this.labelj.TabIndex = 1;
        this.labelj.Text = "---";
        // 
        // textBoxz
        // 
        this.textBoxz.Location = new System.Drawing.Point(190, 106);
        this.textBoxz.Name = "textBoxz";
        this.textBoxz.Size = new System.Drawing.Size(50, 20);
        this.textBoxz.TabIndex = 5;
        this.toolTip1.SetToolTip(this.textBoxz, "D² in hover mode: reacts to errors in angular acceleration");
        this.textBoxz.TextChanged += new System.EventHandler(this.TextBoxkTextChanged);
        // 
        // textBoxk
        // 
        this.textBoxk.Location = new System.Drawing.Point(190, 82);
        this.textBoxk.Name = "textBoxk";
        this.textBoxk.Size = new System.Drawing.Size(50, 20);
        this.textBoxk.TabIndex = 5;
        this.toolTip1.SetToolTip(this.textBoxk, "D in hover mode: reacts to errors in angular velocity");
        this.textBoxk.TextChanged += new System.EventHandler(this.TextBoxkTextChanged);
        // 
        // labelz
        // 
        this.labelz.Location = new System.Drawing.Point(113, 109);
        this.labelz.Name = "labelz";
        this.labelz.Size = new System.Drawing.Size(50, 20);
        this.labelz.TabIndex = 2;
        this.labelz.Text = "---";
        // 
        // labelk
        // 
        this.labelk.Location = new System.Drawing.Point(113, 85);
        this.labelk.Name = "labelk";
        this.labelk.Size = new System.Drawing.Size(50, 20);
        this.labelk.TabIndex = 2;
        this.labelk.Text = "---";
        // 
        // textBoxj
        // 
        this.textBoxj.Location = new System.Drawing.Point(190, 59);
        this.textBoxj.Name = "textBoxj";
        this.textBoxj.Size = new System.Drawing.Size(50, 20);
        this.textBoxj.TabIndex = 4;
        this.toolTip1.SetToolTip(this.textBoxj, "I in hover mode: the sum of the error of the angle");
        this.textBoxj.TextChanged += new System.EventHandler(this.TextBoxjTextChanged);
        // 
        // tabPage5
        // 
        this.tabPage5.Controls.Add(this.groupBox15);
        this.tabPage5.Controls.Add(this.groupBox8);
        this.tabPage5.Controls.Add(this.groupBox7);
        this.tabPage5.Location = new System.Drawing.Point(4, 22);
        this.tabPage5.Name = "tabPage5";
        this.tabPage5.Size = new System.Drawing.Size(649, 366);
        this.tabPage5.TabIndex = 4;
        this.tabPage5.Text = "Miscellaneous";
        this.tabPage5.UseVisualStyleBackColor = true;
        // 
        // groupBox15
        // 
        this.groupBox15.Controls.Add(this.emergmask);
        this.groupBox15.Controls.Add(this.label41);
        this.groupBox15.Controls.Add(this.label40);
        this.groupBox15.Controls.Add(this.labelled2);
        this.groupBox15.Controls.Add(this.labelled1);
        this.groupBox15.Controls.Add(this.label37);
        this.groupBox15.Controls.Add(this.label38);
        this.groupBox15.Controls.Add(this.label39);
        this.groupBox15.Controls.Add(this.label36);
        this.groupBox15.Controls.Add(this.label35);
        this.groupBox15.Controls.Add(this.led2mask);
        this.groupBox15.Controls.Add(this.led1mask);
        this.groupBox15.Location = new System.Drawing.Point(15, 106);
        this.groupBox15.Name = "groupBox15";
        this.groupBox15.Size = new System.Drawing.Size(531, 188);
        this.groupBox15.TabIndex = 2;
        this.groupBox15.TabStop = false;
        this.groupBox15.Text = "Output LED Timing";
        // 
        // label41
        // 
        this.label41.Location = new System.Drawing.Point(152, 100);
        this.label41.Name = "label41";
        this.label41.Size = new System.Drawing.Size(50, 20);
        this.label41.TabIndex = 20;
        this.label41.Text = "---";
        // 
        // label40
        // 
        this.label40.AutoSize = true;
        this.label40.Location = new System.Drawing.Point(17, 100);
        this.label40.Name = "label40";
        this.label40.Size = new System.Drawing.Size(124, 13);
        this.label40.TabIndex = 19;
        this.label40.Text = "Emergency Pattern [170]";
        // 
        // labelled2
        // 
        this.labelled2.Location = new System.Drawing.Point(152, 74);
        this.labelled2.Name = "labelled2";
        this.labelled2.Size = new System.Drawing.Size(50, 20);
        this.labelled2.TabIndex = 18;
        this.labelled2.Text = "---";
        // 
        // labelled1
        // 
        this.labelled1.Location = new System.Drawing.Point(152, 48);
        this.labelled1.Name = "labelled1";
        this.labelled1.Size = new System.Drawing.Size(50, 20);
        this.labelled1.TabIndex = 17;
        this.labelled1.Text = "---";
        // 
        // label37
        // 
        this.label37.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.label37.Location = new System.Drawing.Point(17, 25);
        this.label37.Name = "label37";
        this.label37.Size = new System.Drawing.Size(115, 13);
        this.label37.TabIndex = 15;
        this.label37.Text = "Parameter [default]";
        // 
        // label38
        // 
        this.label38.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.label38.Location = new System.Drawing.Point(226, 25);
        this.label38.Name = "label38";
        this.label38.Size = new System.Drawing.Size(82, 13);
        this.label38.TabIndex = 16;
        this.label38.Text = "New values";
        // 
        // label39
        // 
        this.label39.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.label39.Location = new System.Drawing.Point(138, 25);
        this.label39.Name = "label39";
        this.label39.Size = new System.Drawing.Size(82, 13);
        this.label39.TabIndex = 14;
        this.label39.Text = "Values in µC";
        // 
        // label36
        // 
        this.label36.AutoSize = true;
        this.label36.Location = new System.Drawing.Point(17, 74);
        this.label36.Name = "label36";
        this.label36.Size = new System.Drawing.Size(98, 13);
        this.label36.TabIndex = 3;
        this.label36.Text = "LED2 Pattern [207]";
        // 
        // label35
        // 
        this.label35.AutoSize = true;
        this.label35.Location = new System.Drawing.Point(17, 48);
        this.label35.Name = "label35";
        this.label35.Size = new System.Drawing.Size(98, 13);
        this.label35.TabIndex = 2;
        this.label35.Text = "LED1 Pattern [243]";
        // 
        // groupBox8
        // 
        this.groupBox8.Controls.Add(this.errorlabel);
        this.groupBox8.Controls.Add(this.label12);
        this.groupBox8.Controls.Add(this.static47);
        this.groupBox8.Location = new System.Drawing.Point(15, 300);
        this.groupBox8.Name = "groupBox8";
        this.groupBox8.Size = new System.Drawing.Size(531, 50);
        this.groupBox8.TabIndex = 1;
        this.groupBox8.TabStop = false;
        this.groupBox8.Text = "RC transmission";
        // 
        // errorlabel
        // 
        this.errorlabel.Location = new System.Drawing.Point(138, 20);
        this.errorlabel.Name = "errorlabel";
        this.errorlabel.Size = new System.Drawing.Size(47, 23);
        this.errorlabel.TabIndex = 1;
        this.errorlabel.Text = "0";
        // 
        // label12
        // 
        this.label12.ForeColor = System.Drawing.SystemColors.ControlDark;
        this.label12.Location = new System.Drawing.Point(191, 16);
        this.label12.Name = "label12";
        this.label12.Size = new System.Drawing.Size(340, 27);
        this.label12.TabIndex = 0;
        this.label12.Text = "Every tiny RX jitter will appear here. \r\nNo need to worry if the number stays bel" +
            "ow 50.";
        // 
        // static47
        // 
        this.static47.Location = new System.Drawing.Point(7, 20);
        this.static47.Name = "static47";
        this.static47.Size = new System.Drawing.Size(125, 23);
        this.static47.TabIndex = 0;
        this.static47.Text = "Nr. of receiver problems:";
        // 
        // groupBox7
        // 
        this.groupBox7.Controls.Add(this.voltagelevel);
        this.groupBox7.Controls.Add(this.progressBar1);
        this.groupBox7.Controls.Add(this.textBoxv);
        this.groupBox7.Controls.Add(this.labelv);
        this.groupBox7.Controls.Add(this.static44);
        this.groupBox7.Controls.Add(this.static41);
        this.groupBox7.Controls.Add(this.static43);
        this.groupBox7.Controls.Add(this.static42);
        this.groupBox7.Location = new System.Drawing.Point(15, 18);
        this.groupBox7.Name = "groupBox7";
        this.groupBox7.Size = new System.Drawing.Size(531, 82);
        this.groupBox7.TabIndex = 0;
        this.groupBox7.TabStop = false;
        this.groupBox7.Text = "Per-cell Voltage Warning";
        // 
        // voltagelevel
        // 
        this.voltagelevel.Location = new System.Drawing.Point(282, 52);
        this.voltagelevel.Name = "voltagelevel";
        this.voltagelevel.Size = new System.Drawing.Size(79, 20);
        this.voltagelevel.TabIndex = 18;
        this.voltagelevel.Text = "= 3.3 Volts";
        this.toolTip1.SetToolTip(this.voltagelevel, "Voltage in real units");
        // 
        // progressBar1
        // 
        this.progressBar1.Location = new System.Drawing.Point(367, 49);
        this.progressBar1.Maximum = 44;
        this.progressBar1.Name = "progressBar1";
        this.progressBar1.Size = new System.Drawing.Size(102, 20);
        this.progressBar1.Step = 1;
        this.progressBar1.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
        this.progressBar1.TabIndex = 17;
        this.toolTip1.SetToolTip(this.progressBar1, "Voltage in real units");
        // 
        // textBoxv
        // 
        this.textBoxv.Location = new System.Drawing.Point(226, 49);
        this.textBoxv.Name = "textBoxv";
        this.textBoxv.Size = new System.Drawing.Size(50, 20);
        this.textBoxv.TabIndex = 16;
        this.toolTip1.SetToolTip(this.textBoxv, "LEDs will start blinking very fast when the voltage drops below a threshold");
        this.textBoxv.TextChanged += new System.EventHandler(this.TextBoxvTextChanged);
        // 
        // labelv
        // 
        this.labelv.Location = new System.Drawing.Point(152, 52);
        this.labelv.Name = "labelv";
        this.labelv.Size = new System.Drawing.Size(50, 20);
        this.labelv.TabIndex = 15;
        this.labelv.Text = "---";
        // 
        // static44
        // 
        this.static44.Location = new System.Drawing.Point(17, 52);
        this.static44.Name = "static44";
        this.static44.Size = new System.Drawing.Size(115, 23);
        this.static44.TabIndex = 14;
        this.static44.Text = "Voltage warning [33]";
        // 
        // static41
        // 
        this.static41.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static41.Location = new System.Drawing.Point(17, 25);
        this.static41.Name = "static41";
        this.static41.Size = new System.Drawing.Size(115, 23);
        this.static41.TabIndex = 12;
        this.static41.Text = "Parameter [default]";
        // 
        // static43
        // 
        this.static43.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static43.Location = new System.Drawing.Point(226, 25);
        this.static43.Name = "static43";
        this.static43.Size = new System.Drawing.Size(82, 23);
        this.static43.TabIndex = 13;
        this.static43.Text = "New values";
        // 
        // static42
        // 
        this.static42.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.static42.Location = new System.Drawing.Point(138, 25);
        this.static42.Name = "static42";
        this.static42.Size = new System.Drawing.Size(82, 23);
        this.static42.TabIndex = 11;
        this.static42.Text = "Values in µC";
        // 
        // tabPage4
        // 
        this.tabPage4.Controls.Add(this.static99);
        this.tabPage4.Controls.Add(this.Battery);
        this.tabPage4.Controls.Add(this.groupBox12);
        this.tabPage4.Controls.Add(this.groupBox11);
        this.tabPage4.Controls.Add(this.groupBox10);
        this.tabPage4.Controls.Add(this.groupBox9);
        this.tabPage4.Location = new System.Drawing.Point(4, 22);
        this.tabPage4.Name = "tabPage4";
        this.tabPage4.Padding = new System.Windows.Forms.Padding(3);
        this.tabPage4.Size = new System.Drawing.Size(649, 366);
        this.tabPage4.TabIndex = 5;
        this.tabPage4.Text = "Realtime data";
        this.tabPage4.UseVisualStyleBackColor = true;
        // 
        // static99
        // 
        this.static99.Location = new System.Drawing.Point(56, 7);
        this.static99.Name = "static99";
        this.static99.Size = new System.Drawing.Size(468, 47);
        this.static99.TabIndex = 21;
        this.static99.Text = resources.GetString("static99.Text");
        // 
        // Battery
        // 
        this.Battery.Controls.Add(this.labelvolts);
        this.Battery.Controls.Add(this.pbVoltage);
        this.Battery.Location = new System.Drawing.Point(435, 216);
        this.Battery.Name = "Battery";
        this.Battery.Size = new System.Drawing.Size(134, 146);
        this.Battery.TabIndex = 20;
        this.Battery.TabStop = false;
        this.Battery.Text = "Battery voltage";
        // 
        // labelvolts
        // 
        this.labelvolts.Location = new System.Drawing.Point(6, 84);
        this.labelvolts.Name = "labelvolts";
        this.labelvolts.Size = new System.Drawing.Size(111, 23);
        this.labelvolts.TabIndex = 18;
        this.labelvolts.Text = "---";
        // 
        // pbVoltage
        // 
        this.pbVoltage.Location = new System.Drawing.Point(6, 58);
        this.pbVoltage.Maximum = 255;
        this.pbVoltage.Name = "pbVoltage";
        this.pbVoltage.Size = new System.Drawing.Size(122, 23);
        this.pbVoltage.Step = 1;
        this.pbVoltage.TabIndex = 16;
        // 
        // groupBox12
        // 
        this.groupBox12.Controls.Add(this.TrackBar7);
        this.groupBox12.Controls.Add(this.static58);
        this.groupBox12.Controls.Add(this.static55);
        this.groupBox12.Controls.Add(this.static61);
        this.groupBox12.Location = new System.Drawing.Point(435, 57);
        this.groupBox12.Name = "groupBox12";
        this.groupBox12.Size = new System.Drawing.Size(134, 153);
        this.groupBox12.TabIndex = 19;
        this.groupBox12.TabStop = false;
        this.groupBox12.Text = "Yaw";
        // 
        // TrackBar7
        // 
        this.TrackBar7.BackColor = System.Drawing.Color.Transparent;
        this.TrackBar7.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.TrackBar7.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.TrackBar7.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.TrackBar7.IndentHeight = 1;
        this.TrackBar7.IndentWidth = 1;
        this.TrackBar7.Location = new System.Drawing.Point(16, 66);
        this.TrackBar7.Maximum = 255;
        this.TrackBar7.Minimum = 0;
        this.TrackBar7.Name = "TrackBar7";
        this.TrackBar7.Orientation = System.Windows.Forms.Orientation.Horizontal;
        this.TrackBar7.Size = new System.Drawing.Size(100, 19);
        this.TrackBar7.TabIndex = 23;
        this.TrackBar7.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.TrackBar7.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.TrackBar7.TickFrequency = 128;
        this.TrackBar7.TickHeight = 1;
        this.TrackBar7.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.TrackBar7.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.TrackBar7.TrackerSize = new System.Drawing.Size(13, 13);
        this.TrackBar7.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.TrackBar7.TrackLineHeight = 9;
        this.TrackBar7.Value = 126;
        // 
        // static58
        // 
        this.static58.Location = new System.Drawing.Point(20, 38);
        this.static58.Name = "static58";
        this.static58.Size = new System.Drawing.Size(55, 15);
        this.static58.TabIndex = 11;
        this.static58.Text = "left";
        // 
        // static55
        // 
        this.static55.Location = new System.Drawing.Point(20, 92);
        this.static55.Name = "static55";
        this.static55.Size = new System.Drawing.Size(93, 20);
        this.static55.TabIndex = 8;
        this.static55.Text = "Gyro yaw velocity";
        // 
        // static61
        // 
        this.static61.Location = new System.Drawing.Point(85, 38);
        this.static61.Name = "static61";
        this.static61.Size = new System.Drawing.Size(36, 15);
        this.static61.TabIndex = 11;
        this.static61.Text = "right";
        // 
        // groupBox11
        // 
        this.groupBox11.Controls.Add(this.TrackBar12);
        this.groupBox11.Controls.Add(this.TrackBar11);
        this.groupBox11.Controls.Add(this.TrackBar10);
        this.groupBox11.Controls.Add(this.panel2);
        this.groupBox11.Controls.Add(this.panel4);
        this.groupBox11.Controls.Add(this.panelRollCenter);
        this.groupBox11.Controls.Add(this.label15);
        this.groupBox11.Controls.Add(this.panel1);
        this.groupBox11.Controls.Add(this.label10);
        this.groupBox11.Controls.Add(this.label9);
        this.groupBox11.Controls.Add(this.label8);
        this.groupBox11.Controls.Add(this.label6);
        this.groupBox11.Controls.Add(this.label7);
        this.groupBox11.Controls.Add(this.label5);
        this.groupBox11.Controls.Add(this.label4);
        this.groupBox11.Controls.Add(this.label3);
        this.groupBox11.Controls.Add(this.label2);
        this.groupBox11.Controls.Add(this.label1);
        this.groupBox11.Controls.Add(this.TrackBar9);
        this.groupBox11.Controls.Add(this.TrackBar8);
        this.groupBox11.Location = new System.Drawing.Point(56, 216);
        this.groupBox11.Name = "groupBox11";
        this.groupBox11.Size = new System.Drawing.Size(350, 146);
        this.groupBox11.TabIndex = 17;
        this.groupBox11.TabStop = false;
        this.groupBox11.Text = "Transmitter";
        // 
        // TrackBar12
        // 
        this.TrackBar12.BackColor = System.Drawing.Color.Transparent;
        this.TrackBar12.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.TrackBar12.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.TrackBar12.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.TrackBar12.IndentHeight = 1;
        this.TrackBar12.IndentWidth = 1;
        this.TrackBar12.Location = new System.Drawing.Point(285, 39);
        this.TrackBar12.Maximum = 255;
        this.TrackBar12.Minimum = 0;
        this.TrackBar12.Name = "TrackBar12";
        this.TrackBar12.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.TrackBar12.Size = new System.Drawing.Size(19, 80);
        this.TrackBar12.TabIndex = 28;
        this.TrackBar12.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.TrackBar12.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.TrackBar12.TickFrequency = 128;
        this.TrackBar12.TickHeight = 1;
        this.TrackBar12.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.TrackBar12.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.TrackBar12.TrackerSize = new System.Drawing.Size(13, 13);
        this.TrackBar12.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.TrackBar12.TrackLineHeight = 9;
        this.TrackBar12.Value = 0;
        // 
        // TrackBar11
        // 
        this.TrackBar11.BackColor = System.Drawing.Color.Transparent;
        this.TrackBar11.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.TrackBar11.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.TrackBar11.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.TrackBar11.IndentHeight = 1;
        this.TrackBar11.IndentWidth = 1;
        this.TrackBar11.Location = new System.Drawing.Point(96, 39);
        this.TrackBar11.Maximum = 255;
        this.TrackBar11.Minimum = 0;
        this.TrackBar11.Name = "TrackBar11";
        this.TrackBar11.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.TrackBar11.Size = new System.Drawing.Size(19, 80);
        this.TrackBar11.TabIndex = 27;
        this.TrackBar11.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.TrackBar11.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.TrackBar11.TickFrequency = 128;
        this.TrackBar11.TickHeight = 1;
        this.TrackBar11.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.TrackBar11.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.TrackBar11.TrackerSize = new System.Drawing.Size(13, 13);
        this.TrackBar11.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.TrackBar11.TrackLineHeight = 9;
        this.TrackBar11.Value = 126;
        // 
        // TrackBar10
        // 
        this.TrackBar10.BackColor = System.Drawing.Color.Transparent;
        this.TrackBar10.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.TrackBar10.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.TrackBar10.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.TrackBar10.IndentHeight = 1;
        this.TrackBar10.IndentWidth = 1;
        this.TrackBar10.Location = new System.Drawing.Point(22, 37);
        this.TrackBar10.Maximum = 255;
        this.TrackBar10.Minimum = 0;
        this.TrackBar10.Name = "TrackBar10";
        this.TrackBar10.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.TrackBar10.Size = new System.Drawing.Size(19, 80);
        this.TrackBar10.TabIndex = 26;
        this.TrackBar10.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.TrackBar10.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.TrackBar10.TickFrequency = 128;
        this.TrackBar10.TickHeight = 1;
        this.TrackBar10.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.TrackBar10.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.TrackBar10.TrackerSize = new System.Drawing.Size(13, 13);
        this.TrackBar10.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.TrackBar10.TrackLineHeight = 9;
        this.TrackBar10.Value = 0;
        // 
        // panel2
        // 
        this.panel2.BackColor = System.Drawing.Color.Green;
        this.panel2.Location = new System.Drawing.Point(43, 105);
        this.panel2.Name = "panel2";
        this.panel2.Size = new System.Drawing.Size(10, 10);
        this.panel2.TabIndex = 23;
        // 
        // panel4
        // 
        this.panel4.BackColor = System.Drawing.Color.Green;
        this.panel4.Location = new System.Drawing.Point(192, 92);
        this.panel4.Name = "panel4";
        this.panel4.Size = new System.Drawing.Size(10, 10);
        this.panel4.TabIndex = 23;
        // 
        // panelRollCenter
        // 
        this.panelRollCenter.BackColor = System.Drawing.Color.Green;
        this.panelRollCenter.Location = new System.Drawing.Point(192, 56);
        this.panelRollCenter.Name = "panelRollCenter";
        this.panelRollCenter.Size = new System.Drawing.Size(10, 10);
        this.panelRollCenter.TabIndex = 23;
        // 
        // label15
        // 
        this.label15.Location = new System.Drawing.Point(277, 21);
        this.label15.Name = "label15";
        this.label15.Size = new System.Drawing.Size(40, 18);
        this.label15.TabIndex = 24;
        this.label15.Text = "Hover";
        // 
        // panel1
        // 
        this.panel1.BackColor = System.Drawing.Color.Green;
        this.panel1.Location = new System.Drawing.Point(85, 74);
        this.panel1.Name = "panel1";
        this.panel1.Size = new System.Drawing.Size(10, 10);
        this.panel1.TabIndex = 22;
        // 
        // label10
        // 
        this.label10.Location = new System.Drawing.Point(274, 123);
        this.label10.Name = "label10";
        this.label10.Size = new System.Drawing.Size(50, 15);
        this.label10.TabIndex = 12;
        this.label10.Text = "Mot. off";
        // 
        // label9
        // 
        this.label9.Location = new System.Drawing.Point(306, 72);
        this.label9.Name = "label9";
        this.label9.Size = new System.Drawing.Size(32, 18);
        this.label9.TabIndex = 12;
        this.label9.Text = "-Acro";
        // 
        // label8
        // 
        this.label8.Location = new System.Drawing.Point(200, 123);
        this.label8.Name = "label8";
        this.label8.Size = new System.Drawing.Size(55, 15);
        this.label8.TabIndex = 11;
        this.label8.Text = "Yaw right";
        // 
        // label6
        // 
        this.label6.Location = new System.Drawing.Point(203, 21);
        this.label6.Name = "label6";
        this.label6.Size = new System.Drawing.Size(50, 15);
        this.label6.TabIndex = 11;
        this.label6.Text = "Roll right";
        // 
        // label7
        // 
        this.label7.Location = new System.Drawing.Point(144, 123);
        this.label7.Name = "label7";
        this.label7.Size = new System.Drawing.Size(50, 15);
        this.label7.TabIndex = 10;
        this.label7.Text = "Yaw left";
        // 
        // label5
        // 
        this.label5.Location = new System.Drawing.Point(144, 21);
        this.label5.Name = "label5";
        this.label5.Size = new System.Drawing.Size(50, 15);
        this.label5.TabIndex = 10;
        this.label5.Text = "Roll left";
        // 
        // label4
        // 
        this.label4.Location = new System.Drawing.Point(60, 123);
        this.label4.Name = "label4";
        this.label4.Size = new System.Drawing.Size(80, 15);
        this.label4.TabIndex = 9;
        this.label4.Text = "Nick backward";
        // 
        // label3
        // 
        this.label3.Location = new System.Drawing.Point(66, 21);
        this.label3.Name = "label3";
        this.label3.Size = new System.Drawing.Size(70, 15);
        this.label3.TabIndex = 9;
        this.label3.Text = "Nick forward";
        // 
        // label2
        // 
        this.label2.Location = new System.Drawing.Point(18, 123);
        this.label2.Name = "label2";
        this.label2.Size = new System.Drawing.Size(34, 15);
        this.label2.TabIndex = 8;
        this.label2.Text = "Idle";
        // 
        // label1
        // 
        this.label1.Location = new System.Drawing.Point(4, 21);
        this.label1.Name = "label1";
        this.label1.Size = new System.Drawing.Size(65, 15);
        this.label1.TabIndex = 7;
        this.label1.Text = "Full throttle";
        // 
        // TrackBar9
        // 
        this.TrackBar9.BackColor = System.Drawing.Color.Transparent;
        this.TrackBar9.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.TrackBar9.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.TrackBar9.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.TrackBar9.IndentHeight = 1;
        this.TrackBar9.IndentWidth = 1;
        this.TrackBar9.Location = new System.Drawing.Point(146, 101);
        this.TrackBar9.Maximum = 255;
        this.TrackBar9.Minimum = 0;
        this.TrackBar9.Name = "TrackBar9";
        this.TrackBar9.Orientation = System.Windows.Forms.Orientation.Horizontal;
        this.TrackBar9.Size = new System.Drawing.Size(100, 19);
        this.TrackBar9.TabIndex = 25;
        this.TrackBar9.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.TrackBar9.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.TrackBar9.TickFrequency = 128;
        this.TrackBar9.TickHeight = 1;
        this.TrackBar9.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.TrackBar9.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.TrackBar9.TrackerSize = new System.Drawing.Size(13, 13);
        this.TrackBar9.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.TrackBar9.TrackLineHeight = 9;
        this.TrackBar9.Value = 126;
        // 
        // TrackBar8
        // 
        this.TrackBar8.BackColor = System.Drawing.Color.Transparent;
        this.TrackBar8.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.TrackBar8.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.TrackBar8.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.TrackBar8.IndentHeight = 1;
        this.TrackBar8.IndentWidth = 1;
        this.TrackBar8.Location = new System.Drawing.Point(146, 38);
        this.TrackBar8.Maximum = 255;
        this.TrackBar8.Minimum = 0;
        this.TrackBar8.Name = "TrackBar8";
        this.TrackBar8.Orientation = System.Windows.Forms.Orientation.Horizontal;
        this.TrackBar8.Size = new System.Drawing.Size(100, 19);
        this.TrackBar8.TabIndex = 25;
        this.TrackBar8.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.TrackBar8.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.TrackBar8.TickFrequency = 128;
        this.TrackBar8.TickHeight = 1;
        this.TrackBar8.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.TrackBar8.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.TrackBar8.TrackerSize = new System.Drawing.Size(13, 13);
        this.TrackBar8.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.TrackBar8.TrackLineHeight = 9;
        this.TrackBar8.Value = 126;
        // 
        // groupBox10
        // 
        this.groupBox10.Controls.Add(this.TrackBar6);
        this.groupBox10.Controls.Add(this.TrackBar5);
        this.groupBox10.Controls.Add(this.TrackBar4);
        this.groupBox10.Controls.Add(this.static54);
        this.groupBox10.Controls.Add(this.static53);
        this.groupBox10.Controls.Add(this.staic59);
        this.groupBox10.Controls.Add(this.static60);
        this.groupBox10.Controls.Add(this.static52);
        this.groupBox10.Location = new System.Drawing.Point(271, 57);
        this.groupBox10.Name = "groupBox10";
        this.groupBox10.Size = new System.Drawing.Size(135, 153);
        this.groupBox10.TabIndex = 10;
        this.groupBox10.TabStop = false;
        this.groupBox10.Text = "Nick";
        // 
        // TrackBar6
        // 
        this.TrackBar6.BackColor = System.Drawing.Color.Transparent;
        this.TrackBar6.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.TrackBar6.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.TrackBar6.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.TrackBar6.IndentHeight = 1;
        this.TrackBar6.IndentWidth = 1;
        this.TrackBar6.Location = new System.Drawing.Point(54, 19);
        this.TrackBar6.Maximum = 255;
        this.TrackBar6.Minimum = 0;
        this.TrackBar6.Name = "TrackBar6";
        this.TrackBar6.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.TrackBar6.Size = new System.Drawing.Size(19, 100);
        this.TrackBar6.TabIndex = 26;
        this.TrackBar6.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.TrackBar6.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.TrackBar6.TickFrequency = 128;
        this.TrackBar6.TickHeight = 1;
        this.TrackBar6.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.TrackBar6.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.TrackBar6.TrackerSize = new System.Drawing.Size(13, 13);
        this.TrackBar6.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.TrackBar6.TrackLineHeight = 9;
        this.TrackBar6.Value = 126;
        // 
        // TrackBar5
        // 
        this.TrackBar5.BackColor = System.Drawing.Color.Transparent;
        this.TrackBar5.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.TrackBar5.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.TrackBar5.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.TrackBar5.IndentHeight = 1;
        this.TrackBar5.IndentWidth = 1;
        this.TrackBar5.Location = new System.Drawing.Point(30, 19);
        this.TrackBar5.Maximum = 255;
        this.TrackBar5.Minimum = 0;
        this.TrackBar5.Name = "TrackBar5";
        this.TrackBar5.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.TrackBar5.Size = new System.Drawing.Size(19, 100);
        this.TrackBar5.TabIndex = 25;
        this.TrackBar5.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.TrackBar5.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.TrackBar5.TickFrequency = 128;
        this.TrackBar5.TickHeight = 1;
        this.TrackBar5.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.TrackBar5.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.TrackBar5.TrackerSize = new System.Drawing.Size(13, 13);
        this.TrackBar5.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.TrackBar5.TrackLineHeight = 9;
        this.TrackBar5.Value = 126;
        // 
        // TrackBar4
        // 
        this.TrackBar4.BackColor = System.Drawing.Color.Transparent;
        this.TrackBar4.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.TrackBar4.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.TrackBar4.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.TrackBar4.IndentHeight = 1;
        this.TrackBar4.IndentWidth = 1;
        this.TrackBar4.Location = new System.Drawing.Point(6, 19);
        this.TrackBar4.Maximum = 255;
        this.TrackBar4.Minimum = 0;
        this.TrackBar4.Name = "TrackBar4";
        this.TrackBar4.Orientation = System.Windows.Forms.Orientation.Vertical;
        this.TrackBar4.Size = new System.Drawing.Size(19, 100);
        this.TrackBar4.TabIndex = 24;
        this.TrackBar4.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.TrackBar4.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.TrackBar4.TickFrequency = 128;
        this.TrackBar4.TickHeight = 1;
        this.TrackBar4.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.TrackBar4.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.TrackBar4.TrackerSize = new System.Drawing.Size(13, 13);
        this.TrackBar4.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.TrackBar4.TrackLineHeight = 9;
        this.TrackBar4.Value = 126;
        // 
        // static54
        // 
        this.static54.Location = new System.Drawing.Point(58, 123);
        this.static54.Name = "static54";
        this.static54.Size = new System.Drawing.Size(41, 20);
        this.static54.TabIndex = 8;
        this.static54.Text = "Acc(a)";
        // 
        // static53
        // 
        this.static53.Location = new System.Drawing.Point(29, 123);
        this.static53.Name = "static53";
        this.static53.Size = new System.Drawing.Size(36, 20);
        this.static53.TabIndex = 8;
        this.static53.Text = "Gy(a)";
        // 
        // staic59
        // 
        this.staic59.Location = new System.Drawing.Point(74, 99);
        this.staic59.Name = "staic59";
        this.staic59.Size = new System.Drawing.Size(55, 15);
        this.staic59.TabIndex = 11;
        this.staic59.Text = "backward";
        // 
        // static60
        // 
        this.static60.Location = new System.Drawing.Point(74, 23);
        this.static60.Name = "static60";
        this.static60.Size = new System.Drawing.Size(55, 15);
        this.static60.TabIndex = 11;
        this.static60.Text = "forward";
        // 
        // static52
        // 
        this.static52.Location = new System.Drawing.Point(0, 123);
        this.static52.Name = "static52";
        this.static52.Size = new System.Drawing.Size(34, 23);
        this.static52.TabIndex = 8;
        this.static52.Text = "Gy(v)";
        // 
        // groupBox9
        // 
        this.groupBox9.Controls.Add(this.TrackBar3);
        this.groupBox9.Controls.Add(this.TrackBar2);
        this.groupBox9.Controls.Add(this.static51);
        this.groupBox9.Controls.Add(this.TrackBar1);
        this.groupBox9.Controls.Add(this.static49);
        this.groupBox9.Controls.Add(this.static57);
        this.groupBox9.Controls.Add(this.static50);
        this.groupBox9.Controls.Add(this.static56);
        this.groupBox9.Location = new System.Drawing.Point(56, 57);
        this.groupBox9.Name = "groupBox9";
        this.groupBox9.Size = new System.Drawing.Size(187, 153);
        this.groupBox9.TabIndex = 9;
        this.groupBox9.TabStop = false;
        this.groupBox9.Text = "Roll";
        // 
        // TrackBar3
        // 
        this.TrackBar3.BackColor = System.Drawing.Color.Transparent;
        this.TrackBar3.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.TrackBar3.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.TrackBar3.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.TrackBar3.IndentHeight = 1;
        this.TrackBar3.IndentWidth = 1;
        this.TrackBar3.Location = new System.Drawing.Point(6, 93);
        this.TrackBar3.Maximum = 255;
        this.TrackBar3.Minimum = 0;
        this.TrackBar3.Name = "TrackBar3";
        this.TrackBar3.Orientation = System.Windows.Forms.Orientation.Horizontal;
        this.TrackBar3.Size = new System.Drawing.Size(100, 19);
        this.TrackBar3.TabIndex = 23;
        this.TrackBar3.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.TrackBar3.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.TrackBar3.TickFrequency = 128;
        this.TrackBar3.TickHeight = 1;
        this.TrackBar3.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.TrackBar3.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.TrackBar3.TrackerSize = new System.Drawing.Size(13, 13);
        this.TrackBar3.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.TrackBar3.TrackLineHeight = 9;
        this.TrackBar3.Value = 126;
        // 
        // TrackBar2
        // 
        this.TrackBar2.BackColor = System.Drawing.Color.Transparent;
        this.TrackBar2.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.TrackBar2.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.TrackBar2.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.TrackBar2.IndentHeight = 1;
        this.TrackBar2.IndentWidth = 1;
        this.TrackBar2.Location = new System.Drawing.Point(6, 66);
        this.TrackBar2.Maximum = 255;
        this.TrackBar2.Minimum = 0;
        this.TrackBar2.Name = "TrackBar2";
        this.TrackBar2.Orientation = System.Windows.Forms.Orientation.Horizontal;
        this.TrackBar2.Size = new System.Drawing.Size(100, 19);
        this.TrackBar2.TabIndex = 22;
        this.TrackBar2.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.TrackBar2.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.TrackBar2.TickFrequency = 128;
        this.TrackBar2.TickHeight = 1;
        this.TrackBar2.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.TrackBar2.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.TrackBar2.TrackerSize = new System.Drawing.Size(13, 13);
        this.TrackBar2.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.TrackBar2.TrackLineHeight = 9;
        this.TrackBar2.Value = 126;
        // 
        // static51
        // 
        this.static51.Location = new System.Drawing.Point(112, 96);
        this.static51.Name = "static51";
        this.static51.Size = new System.Drawing.Size(68, 20);
        this.static51.TabIndex = 8;
        this.static51.Text = "Acc angle";
        // 
        // TrackBar1
        // 
        this.TrackBar1.BackColor = System.Drawing.Color.Transparent;
        this.TrackBar1.BorderColor = System.Drawing.SystemColors.ActiveBorder;
        this.TrackBar1.Font = new System.Drawing.Font("Verdana", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.TrackBar1.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(123)))), ((int)(((byte)(125)))), ((int)(((byte)(123)))));
        this.TrackBar1.IndentHeight = 1;
        this.TrackBar1.IndentWidth = 1;
        this.TrackBar1.Location = new System.Drawing.Point(6, 39);
        this.TrackBar1.Maximum = 255;
        this.TrackBar1.Minimum = 0;
        this.TrackBar1.Name = "TrackBar1";
        this.TrackBar1.Orientation = System.Windows.Forms.Orientation.Horizontal;
        this.TrackBar1.Size = new System.Drawing.Size(100, 19);
        this.TrackBar1.TabIndex = 22;
        this.TrackBar1.TextTickStyle = System.Windows.Forms.TickStyle.None;
        this.TrackBar1.TickColor = System.Drawing.Color.FromArgb(((int)(((byte)(148)))), ((int)(((byte)(146)))), ((int)(((byte)(148)))));
        this.TrackBar1.TickFrequency = 128;
        this.TrackBar1.TickHeight = 1;
        this.TrackBar1.TickStyle = System.Windows.Forms.TickStyle.Both;
        this.TrackBar1.TrackerColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
        this.TrackBar1.TrackerSize = new System.Drawing.Size(13, 13);
        this.TrackBar1.TrackLineColor = System.Drawing.Color.FromArgb(((int)(((byte)(90)))), ((int)(((byte)(93)))), ((int)(((byte)(90)))));
        this.TrackBar1.TrackLineHeight = 9;
        this.TrackBar1.Value = 126;
        // 
        // static49
        // 
        this.static49.Location = new System.Drawing.Point(112, 42);
        this.static49.Name = "static49";
        this.static49.Size = new System.Drawing.Size(68, 20);
        this.static49.TabIndex = 8;
        this.static49.Text = "Gyro velocity";
        // 
        // static57
        // 
        this.static57.Location = new System.Drawing.Point(79, 21);
        this.static57.Name = "static57";
        this.static57.Size = new System.Drawing.Size(55, 15);
        this.static57.TabIndex = 11;
        this.static57.Text = "right";
        // 
        // static50
        // 
        this.static50.Location = new System.Drawing.Point(112, 69);
        this.static50.Name = "static50";
        this.static50.Size = new System.Drawing.Size(68, 20);
        this.static50.TabIndex = 8;
        this.static50.Text = "Gyro angle";
        // 
        // static56
        // 
        this.static56.Location = new System.Drawing.Point(6, 21);
        this.static56.Name = "static56";
        this.static56.Size = new System.Drawing.Size(55, 15);
        this.static56.TabIndex = 11;
        this.static56.Text = "left";
        // 
        // tabPage6
        // 
        this.tabPage6.Controls.Add(this.groupBox13);
        this.tabPage6.Controls.Add(this.avrdudeout);
        this.tabPage6.Location = new System.Drawing.Point(4, 22);
        this.tabPage6.Name = "tabPage6";
        this.tabPage6.Size = new System.Drawing.Size(649, 366);
        this.tabPage6.TabIndex = 6;
        this.tabPage6.Text = "Firmware update";
        this.tabPage6.UseVisualStyleBackColor = true;
        // 
        // groupBox13
        // 
        this.groupBox13.Controls.Add(this.label13);
        this.groupBox13.Controls.Add(this.flash);
        this.groupBox13.Controls.Add(this.progressBar4);
        this.groupBox13.Controls.Add(this.showavrout);
        this.groupBox13.Location = new System.Drawing.Point(12, 30);
        this.groupBox13.Name = "groupBox13";
        this.groupBox13.Size = new System.Drawing.Size(623, 107);
        this.groupBox13.TabIndex = 4;
        this.groupBox13.TabStop = false;
        this.groupBox13.Text = "Update firmware";
        // 
        // label13
        // 
        this.label13.Location = new System.Drawing.Point(491, 36);
        this.label13.Name = "label13";
        this.label13.Size = new System.Drawing.Size(100, 15);
        this.label13.TabIndex = 4;
        this.label13.Text = "---";
        this.label13.TextAlign = System.Drawing.ContentAlignment.BottomRight;
        // 
        // flash
        // 
        this.flash.Location = new System.Drawing.Point(26, 32);
        this.flash.Name = "flash";
        this.flash.Size = new System.Drawing.Size(137, 23);
        this.flash.TabIndex = 0;
        this.flash.Text = "Flash new firmware";
        this.flash.UseVisualStyleBackColor = true;
        this.flash.Click += new System.EventHandler(this.FlashClick);
        // 
        // progressBar4
        // 
        this.progressBar4.Location = new System.Drawing.Point(26, 61);
        this.progressBar4.Maximum = 5;
        this.progressBar4.Name = "progressBar4";
        this.progressBar4.Size = new System.Drawing.Size(565, 23);
        this.progressBar4.Step = 1;
        this.progressBar4.Style = System.Windows.Forms.ProgressBarStyle.Continuous;
        this.progressBar4.TabIndex = 2;
        // 
        // showavrout
        // 
        this.showavrout.Location = new System.Drawing.Point(169, 32);
        this.showavrout.Name = "showavrout";
        this.showavrout.Size = new System.Drawing.Size(104, 24);
        this.showavrout.TabIndex = 3;
        this.showavrout.Text = "Show output";
        this.showavrout.UseVisualStyleBackColor = true;
        this.showavrout.CheckedChanged += new System.EventHandler(this.ShowavroutCheckedChanged);
        // 
        // avrdudeout
        // 
        this.avrdudeout.Location = new System.Drawing.Point(12, 161);
        this.avrdudeout.Multiline = true;
        this.avrdudeout.Name = "avrdudeout";
        this.avrdudeout.ReadOnly = true;
        this.avrdudeout.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
        this.avrdudeout.Size = new System.Drawing.Size(623, 202);
        this.avrdudeout.TabIndex = 1;
        this.avrdudeout.Visible = false;
        // 
        // serialPort
        // 
        this.serialPort.BaudRate = 38400;
        this.serialPort.PortName = "COM5";
        this.serialPort.ReadTimeout = 1000;
        // 
        // timer1
        // 
        this.timer1.Interval = 50;
        this.timer1.Tick += new System.EventHandler(this.Timer1Tick);
        // 
        // pictureBox1
        // 
        this.pictureBox1.BackColor = System.Drawing.Color.Transparent;
        this.pictureBox1.ErrorImage = null;
        this.pictureBox1.InitialImage = null;
        this.pictureBox1.Location = new System.Drawing.Point(0, 0);
        this.pictureBox1.Name = "pictureBox1";
        this.pictureBox1.Size = new System.Drawing.Size(681, 50);
        this.pictureBox1.TabIndex = 6;
        this.pictureBox1.TabStop = false;
        // 
        // statusStrip1
        // 
        this.statusStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.toolStatusLabel});
        this.statusStrip1.Location = new System.Drawing.Point(0, 480);
        this.statusStrip1.Name = "statusStrip1";
        this.statusStrip1.Size = new System.Drawing.Size(681, 22);
        this.statusStrip1.SizingGrip = false;
        this.statusStrip1.TabIndex = 7;
        this.statusStrip1.Text = "statusStrip1";
        // 
        // toolStatusLabel
        // 
        this.toolStatusLabel.Name = "toolStatusLabel";
        this.toolStatusLabel.Size = new System.Drawing.Size(77, 17);
        this.toolStatusLabel.Text = "Not connected";
        // 
        // timersens
        // 
        this.timersens.Interval = 75;
        this.timersens.Tick += new System.EventHandler(this.TimersensTick);
        // 
        // openFileDialog1
        // 
        this.openFileDialog1.DefaultExt = "shr";
        this.openFileDialog1.Filter = "Shrediquette configurations|*.shr";
        this.openFileDialog1.Title = "Load settings from file and transfer to µC";
        // 
        // saveFileDialog1
        // 
        this.saveFileDialog1.DefaultExt = "shr";
        this.saveFileDialog1.Filter = "Shrediquette configurations|*.shr";
        this.saveFileDialog1.Title = "Save Shrediquette configuration file";
        // 
        // pingshrediquette
        // 
        this.pingshrediquette.Interval = 550;
        this.pingshrediquette.Tick += new System.EventHandler(this.PingshrediquetteTick);
        // 
        // label11
        // 
        this.label11.Font = new System.Drawing.Font("Courier New", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.label11.Location = new System.Drawing.Point(536, 53);
        this.label11.Name = "label11";
        this.label11.Size = new System.Drawing.Size(96, 26);
        this.label11.TabIndex = 8;
        this.label11.Text = "v 1.2";
        this.label11.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
        // 
        // startupdelay
        // 
        this.startupdelay.Interval = 500;
        this.startupdelay.Tick += new System.EventHandler(this.startupdelayTick);
        // 
        // linkLabel2
        // 
        this.linkLabel2.Location = new System.Drawing.Point(474, 454);
        this.linkLabel2.Name = "linkLabel2";
        this.linkLabel2.Size = new System.Drawing.Size(195, 25);
        this.linkLabel2.TabIndex = 9;
        this.linkLabel2.TabStop = true;
        this.linkLabel2.Text = "http://shrediquette.blogspot.com";
        this.linkLabel2.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
        this.linkLabel2.VisitedLinkColor = System.Drawing.Color.Navy;
        this.linkLabel2.LinkClicked += new System.Windows.Forms.LinkLabelLinkClickedEventHandler(this.LinkLabel2LinkClicked);
        // 
        // label17
        // 
        this.label17.Font = new System.Drawing.Font("Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
        this.label17.Location = new System.Drawing.Point(12, 456);
        this.label17.Name = "label17";
        this.label17.Size = new System.Drawing.Size(432, 23);
        this.label17.TabIndex = 10;
        this.label17.Text = "Any change has to be written to the tricopter manually (click \"Write all paramete" +
            "rs to µC\")";
        this.label17.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
        // 
        // lbch1
        // 
        this.lbch1.AutoSize = true;
        this.lbch1.Location = new System.Drawing.Point(10, 160);
        this.lbch1.Name = "lbch1";
        this.lbch1.Size = new System.Drawing.Size(25, 13);
        this.lbch1.TabIndex = 41;
        this.lbch1.Text = "000";
        this.lbch1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
        // 
        // lbch2
        // 
        this.lbch2.AutoSize = true;
        this.lbch2.Location = new System.Drawing.Point(35, 160);
        this.lbch2.Name = "lbch2";
        this.lbch2.Size = new System.Drawing.Size(25, 13);
        this.lbch2.TabIndex = 42;
        this.lbch2.Text = "000";
        this.lbch2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
        // 
        // lbch3
        // 
        this.lbch3.AutoSize = true;
        this.lbch3.Location = new System.Drawing.Point(60, 160);
        this.lbch3.Name = "lbch3";
        this.lbch3.Size = new System.Drawing.Size(25, 13);
        this.lbch3.TabIndex = 43;
        this.lbch3.Text = "000";
        this.lbch3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
        // 
        // lbch4
        // 
        this.lbch4.AutoSize = true;
        this.lbch4.Location = new System.Drawing.Point(85, 160);
        this.lbch4.Name = "lbch4";
        this.lbch4.Size = new System.Drawing.Size(25, 13);
        this.lbch4.TabIndex = 44;
        this.lbch4.Text = "000";
        this.lbch4.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
        // 
        // lbch5
        // 
        this.lbch5.AutoSize = true;
        this.lbch5.Location = new System.Drawing.Point(110, 160);
        this.lbch5.Name = "lbch5";
        this.lbch5.Size = new System.Drawing.Size(25, 13);
        this.lbch5.TabIndex = 45;
        this.lbch5.Text = "000";
        this.lbch5.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
        // 
        // lbch6
        // 
        this.lbch6.AutoSize = true;
        this.lbch6.Location = new System.Drawing.Point(135, 160);
        this.lbch6.Name = "lbch6";
        this.lbch6.Size = new System.Drawing.Size(25, 13);
        this.lbch6.TabIndex = 46;
        this.lbch6.Text = "000";
        this.lbch6.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
        // 
        // lbch7
        // 
        this.lbch7.AutoSize = true;
        this.lbch7.Location = new System.Drawing.Point(160, 160);
        this.lbch7.Name = "lbch7";
        this.lbch7.Size = new System.Drawing.Size(25, 13);
        this.lbch7.TabIndex = 47;
        this.lbch7.Text = "000";
        this.lbch7.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
        // 
        // lbch8
        // 
        this.lbch8.AutoSize = true;
        this.lbch8.Location = new System.Drawing.Point(185, 160);
        this.lbch8.Name = "lbch8";
        this.lbch8.Size = new System.Drawing.Size(25, 13);
        this.lbch8.TabIndex = 48;
        this.lbch8.Text = "000";
        this.lbch8.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
        // 
        // label42
        // 
        label42.Location = new System.Drawing.Point(10, 186);
        label42.Name = "label42";
        label42.Size = new System.Drawing.Size(200, 58);
        label42.TabIndex = 49;
        label42.Text = "Use trim function on your transmitter to make sure all channels that are centered" +
            " show as \"0\" for their value, as well as reverse any channels if needed.";
        // 
        // emergmask
        // 
        this.emergmask.Location = new System.Drawing.Point(226, 100);
        this.emergmask.Name = "emergmask";
        this.emergmask.Size = new System.Drawing.Size(175, 20);
        this.emergmask.TabIndex = 21;
        this.emergmask.Value = ((byte)(0));
        // 
        // led2mask
        // 
        this.led2mask.Location = new System.Drawing.Point(226, 74);
        this.led2mask.Name = "led2mask";
        this.led2mask.Size = new System.Drawing.Size(175, 20);
        this.led2mask.TabIndex = 1;
        this.led2mask.Value = ((byte)(0));
        // 
        // led1mask
        // 
        this.led1mask.Location = new System.Drawing.Point(226, 48);
        this.led1mask.Name = "led1mask";
        this.led1mask.Size = new System.Drawing.Size(175, 20);
        this.led1mask.TabIndex = 0;
        this.led1mask.Value = ((byte)(0));
        this.led1mask.OnChange += new TriGUI_v11.BitMask.Changed(this.OnLedMaskChange);
        // 
        // MainForm
        // 
        this.ClientSize = new System.Drawing.Size(681, 502);
        this.Controls.Add(this.label17);
        this.Controls.Add(this.linkLabel2);
        this.Controls.Add(this.label11);
        this.Controls.Add(this.statusStrip1);
        this.Controls.Add(this.tabControl);
        this.Controls.Add(this.pictureBox1);
        this.MaximizeBox = false;
        this.Name = "MainForm";
        this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
        this.Text = "TriGUI v1.2 by William Thielicke (w.th@gmx.de)";
        this.Shown += new System.EventHandler(this.FormShown);
        this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.schliessen);
        this.tabControl.ResumeLayout(false);
        this.tabPage1.ResumeLayout(false);
        this.RxD.ResumeLayout(false);
        this.RxD.PerformLayout();
        ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
        this.tabPage2.ResumeLayout(false);
        this.groupBox14.ResumeLayout(false);
        this.groupBox14.PerformLayout();
        this.groupBox2.ResumeLayout(false);
        this.groupBox2.PerformLayout();
        this.tabPage3.ResumeLayout(false);
        this.groupBox6.ResumeLayout(false);
        this.groupBox6.PerformLayout();
        this.groupBox5.ResumeLayout(false);
        this.groupBox5.PerformLayout();
        ((System.ComponentModel.ISupportInitialize)(this.trackBar15)).EndInit();
        this.groupBox4.ResumeLayout(false);
        this.groupBox3.ResumeLayout(false);
        this.groupBox3.PerformLayout();
        ((System.ComponentModel.ISupportInitialize)(this.trackBar14)).EndInit();
        this.groupBox1.ResumeLayout(false);
        this.groupBox1.PerformLayout();
        ((System.ComponentModel.ISupportInitialize)(this.trackBar13)).EndInit();
        this.tabPage5.ResumeLayout(false);
        this.groupBox15.ResumeLayout(false);
        this.groupBox15.PerformLayout();
        this.groupBox8.ResumeLayout(false);
        this.groupBox7.ResumeLayout(false);
        this.groupBox7.PerformLayout();
        this.tabPage4.ResumeLayout(false);
        this.Battery.ResumeLayout(false);
        this.groupBox12.ResumeLayout(false);
        this.groupBox11.ResumeLayout(false);
        this.groupBox10.ResumeLayout(false);
        this.groupBox9.ResumeLayout(false);
        this.tabPage6.ResumeLayout(false);
        this.tabPage6.PerformLayout();
        this.groupBox13.ResumeLayout(false);
        ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
        this.statusStrip1.ResumeLayout(false);
        this.statusStrip1.PerformLayout();
        this.ResumeLayout(false);
        this.PerformLayout();

    }

    public void LinkLabel1LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
    {
        Process.Start("http://www.villalachouette.de/william/krims/tricopter/TriGUI_tut.wmv");
    }

    public void LinkLabel2LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
    {
        Process.Start("http://shrediquette.blogspot.com");
    }

    public void PingshrediquetteTick(object sender, EventArgs e)
    {
        try {
            serialPort.DiscardOutBuffer();
        } catch (Exception) { }

        try {
            serialPort.DiscardInBuffer();
        } catch (Exception) { }

        getparams();

        if (searchlabel.Visible) {
            searchlabel.Visible = false;
            pingshrediquette.Interval = 900;
        } else {
            searchlabel.Visible = true;
            pingshrediquette.Interval = 350;
        }
    }

    public void ReadsettingsClick(object sender, EventArgs e)
    {
        this.progressBar2.Value = 0;
        this.labelok.Visible = false;
        if (this.serialPort.IsOpen) {
            this.openFileDialog1.InitialDirectory = Environment.CurrentDirectory;
            this.openFileDialog1.ShowDialog();
            string loadfrom = this.openFileDialog1.FileName;
            if ((loadfrom != "") && (Interaction.MsgBox("Transfer parameters from file '" + this.openFileDialog1.SafeFileName + "' to the \x00b5C?", MsgBoxStyle.OkCancel, "Sure?") == MsgBoxResult.Ok)) {
                string[] readtext = File.ReadAllLines(loadfrom);
                int VBt_i4L0 = readtext.Length;
                this.k = 1;
                while (this.k <= VBt_i4L0) {
                    this.outvar[this.k] = Conversions.ToByte(readtext[this.k - 1]);
                    this.k++;
                }
                if (readtext.Length < 0x1f) {
                    Interaction.MsgBox("You attempted to load an outdated settings file. The new firmware requires new settings.", MsgBoxStyle.OkOnly, null);
                } else {
                    this.serialPort.Write("ci!\r");
                    this.serialPort.Write(this.outvar, 1, 0x21);
                    this.timer1.Enabled = false;
                    this.k = 1;
                    do {
                        Thread.Sleep(100);
                        this.progressBar2.PerformStep();
                        this.k++;
                    }
                    while (this.k <= 0x21);
                    this.getparams();
                    this.timer1.Enabled = true;
                    this.progressBar2.Value = this.progressBar2.Maximum;
                }
            }
        } else {
            Interaction.MsgBox("You are not connected to Shrediquette DLX.", MsgBoxStyle.OkOnly, "Error");
            this.toolStatusLabel.Text = "Not connected";
        }
    }

    public void resetmCClick(object sender, EventArgs e)
    {
        this.progressBar2.Value = 0;
        this.labelok.Visible = false;
        try {
            this.pingshrediquette.Enabled = false;
            this.searchlabel.Visible = false;
            this.timer1.Enabled = false;
            this.connect.BackColor = Color.Transparent;
            this.serialPort.Close();
            this.textRxD.Clear();
            this.textRxD.Text = "Connection closed.";
            this.isconnected = false;
            this.btnRefresh.Enabled = true;
            this.comport.Enabled = true;
            this.connect.Enabled = true;
            this.resetmC.Enabled = false;
            this.readsettings.Enabled = false;
            this.writesettings.Enabled = false;
            this.Text = "TriGUI by William Thielicke (w.th@gmx.de)";
            this.tabControl.TabPages[1].Enabled = false;
            this.tabControl.TabPages[2].Enabled = false;
            this.tabControl.TabPages[3].Enabled = false;
            this.tabControl.TabPages[4].Enabled = false;
            this.writeall.Enabled = false;
            this.toolStatusLabel.Text = "Not connected";
        } catch (Exception exception1) {
            ProjectData.SetProjectError(exception1);
            ProjectData.ClearProjectError();
        }
    }

    public void schliessen(object sender, FormClosingEventArgs e)
    {
        try {
            this.timer1.Enabled = false;
            this.serialPort.Close();
            StreamWriter textfile2 = new StreamWriter(Environment.CurrentDirectory + @"\settings.gui");
            textfile2.WriteLine(RuntimeHelpers.GetObjectValue(this.comport.SelectedItem));
            textfile2.Close();
            this.pingshrediquette.Enabled = false;
            this.searchlabel.Visible = false;
        } catch (Exception exception1) {
            ProjectData.SetProjectError(exception1);
            ProjectData.ClearProjectError();
        }
    }

    public void ShowavroutCheckedChanged(object sender, EventArgs e)
    {
        if (this.showavrout.Checked) {
            this.avrdudeout.Visible = true;
        } else {
            this.avrdudeout.Visible = false;
        }
    }

    public void ShowdebugCheckedChanged(object sender, EventArgs e)
    {
        if (this.showdebug.Checked) {
            this.RxD.Visible = true;
        } else {
            this.RxD.Visible = false;
        }
    }

    public void startupdelayTick(object sender, EventArgs e)
    {
        this.comport.Text = "";
        Application.DoEvents();
        this.startupdelay.Enabled = false;
        this.tabControl.TabPages[1].Enabled = false;
        this.tabControl.TabPages[2].Enabled = false;
        this.tabControl.TabPages[3].Enabled = false;
        this.tabControl.TabPages[4].Enabled = false;
        this.comport.Items.Clear();
        string[] ports = SerialPort.GetPortNames();
        try {
            foreach (string port in ports) {
                try {
                    this.toolStatusLabel.Text = "Please wait, testing available COM ports (" + port.ToString() + ")...";
                    Application.DoEvents();
                    SerialPort VBt_refL0 = this.serialPort;
                    VBt_refL0.PortName = port;
                    VBt_refL0.ReadTimeout = int.Parse(Conversions.ToString(500));
                    VBt_refL0.Open();
                    VBt_refL0 = null;
                } catch (Exception exception1) {
                    ProjectData.SetProjectError(exception1);
                    Exception es = exception1;
                    ProjectData.ClearProjectError();
                } finally {
                    if (this.serialPort.IsOpen) {
                        this.comport.Items.Add(port);
                    }
                    this.serialPort.Close();
                }
            }
        } catch (Exception exception2) {
            ProjectData.SetProjectError(exception2);
            ProjectData.ClearProjectError();
        }
        try {
            string[] readtext = File.ReadAllLines(Environment.CurrentDirectory + @"\settings.gui");
            this.comport.SelectedItem = readtext[0];
            this.label13.Text = Conversions.ToString(Operators.ConcatenateObject("Using ", this.comport.SelectedItem));
        } catch (Exception exception3) {
            ProjectData.SetProjectError(exception3);
            ProjectData.ClearProjectError();
        }
        this.toolStatusLabel.Text = "Not connected";
    }

    public void TabControl1SelectedIndexChanged(object sender, EventArgs e)
    {
        if (tabControl.SelectedIndex == 4 || tabControl.SelectedIndex == 1) {
            if (serialPort.IsOpen) {
                try {
                    this.serialPort.DiscardOutBuffer();
                } catch (Exception) { }
                try {
                    serialPort.DiscardInBuffer();
                } catch (Exception) { }
                readsens = true;
                timersens.Enabled = true;
            }
        } else {
            try {
                serialPort.DiscardInBuffer();
            } catch (Exception) {
            }
            try {
                serialPort.DiscardOutBuffer();
            } catch (Exception) {
            }
            readsens = false;
            timersens.Enabled = false;
        }
    }

    public void TextBoxGTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxHTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxiTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxjTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxkTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxlTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxmTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxnTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxoTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxpTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxQTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxRTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxSTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxuTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxvTextChanged(object sender, EventArgs e)
    {
        int realvolts;
        checkinput(RuntimeHelpers.GetObjectValue(sender));

        try {
            realvolts = Convert.ToInt16(textBoxv.Text);
            voltagelevel.Text = "= " + String.Format("{0:0.0}", realvolts / 10.0f) + " Volts";
            progressBar1.Value = realvolts;
        } catch (Exception) { }
    }

    public void TextBoxwTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void TextBoxxTextChanged(object sender, EventArgs e)
    {
        this.checkinput(RuntimeHelpers.GetObjectValue(sender));
    }

    public void Timer1Tick(object sender, EventArgs e)
    {
        if (!serialPort.IsOpen) {
            timer1.Enabled = false;
        } else {
            if (readsens) {
                serialPort.Write("cs!\r");
            }

            if (serialPort.BytesToRead > 0) {
                try {
                    readvar = serialPort.ReadLine();
                } catch (Exception) {
                }

                try {
                    if (readvar.Contains("sp!")) {
                        pingshrediquette.Enabled = false;
                        timer1.Enabled = false;
                        Interaction.MsgBox("The firmware in the tricopter is outdated. Please download the new version! This GUI won't work otherwise.\n\rYou will not be able to use your old settings.\n\rSome things may have changed, including the direction of control inputs.\n\rYou have to check all settings again before mounting the propellers!\n\r\n\rThe first thing to do after you have flashed a new firmware, is to upload the standard parameters!", MsgBoxStyle.OkOnly, "Important!");
                    }

                    if (readvar.Contains("s#p!")) {
                        pingshrediquette.Enabled = false;
                        searchlabel.Visible = false;
                        readvar = "";
                        connect.BackColor = Color.LightGreen;
                        connect.Enabled = false;
                        Text = "TriGUI by William Thielicke (w.th@gmx.de) - CONNECTED";
                        textRxD.Clear();
                        resetmC.Enabled = true;
                        readsettings.Enabled = true;
                        writesettings.Enabled = true;
                        tabControl.TabPages[1].Enabled = true;
                        tabControl.TabPages[2].Enabled = true;
                        tabControl.TabPages[3].Enabled = true;
                        tabControl.TabPages[4].Enabled = true;
                        writeall.Enabled = true;
                        toolStatusLabel.Text = "Connected (" + serialPort.PortName + ", @" + serialPort.BaudRate + " baud) - Shrediquette found.";
                        isconnected = true;

                        if ((serialPort.BytesToRead > 0) && !readsens) {
                            bytesRead = this.serialPort.Read(variable, 0, 33);
                            bytecounter.Text = bytesRead.ToString() + "/33 bytes";
                            MainSettings.MotorsEnabled = variable[0] == 1 ? true : false;
                            rollgyrodir = this.variable[1];
                            nickgyrodir = this.variable[2];
                            yawgyrodir = this.variable[3];
                            xaccdir = this.variable[4];
                            yaccdir = this.variable[5];
                            acrop = this.variable[6];
                            acroi = this.variable[7];
                            hoverp = this.variable[8];
                            hoveri = this.variable[9];
                            hoverd = this.variable[10];
                            yawp = this.variable[11];
                            yawi = this.variable[12];
                            accinfluence = this.variable[13];
                            xaccscale = this.variable[14];
                            yaccscale = this.variable[15];
                            lfacro = this.variable[16];
                            lfhover = this.variable[17];
                            lfyaw = this.variable[18];
                            lfboost = this.variable[19];
                            minthrottle = this.variable[20];
                            voltage = this.variable[21];
                            xacc_offset = this.variable[22];
                            yacc_offset = this.variable[23];
                            throttlechannel = this.variable[24];
                            nickchannel = this.variable[25];
                            rollchannel = this.variable[26];
                            yawchannel = this.variable[27];
                            acrod = this.variable[28];
                            hoverdd = this.variable[29];
                            switchchannel = variable[30];

                            foreach (byte b in variable) {
                                textRxD.AppendText(b.ToString() + "\r\n");
                            }
                            try {
                                this.serialPort.DiscardInBuffer();
                            } catch (Exception exception2) {
                                ProjectData.SetProjectError(exception2);
                                ProjectData.ClearProjectError();
                            }
                        }
                        this.progressBar2.Value = this.progressBar2.Maximum;
                        this.labelok.Visible = true;
                        if ((((this.lfacro == 0) | (this.lfhover == 0)) | (this.xaccscale == 0)) | (this.yaccscale == 0)) {
                            Interaction.MsgBox("Welcome!\n\r\n\rThis seems to be the first time you connect to Shrediquette.\n\rYou should upload the default settings first (press 'Load file and write to \x00b5C' to load the file 'defaults.shr').\n\rDISMOUNT THE PROPELLERS!", MsgBoxStyle.OkOnly, "Notice");
                            Interaction.MsgBox("READ THIS!\n\r\n\rAfter uploading the default settings, it is extremely important that you check the channel assignments of you RX/TX. After you assigned all channels, write the parameters to Shrediquette and check again the settings of the channels under 'Realtime data'.\n\rIf your stick movements are reversed, enable servo-reverse in your transmitter. \n\rAgain...: DISMOUNT THE PROPELLERS!", MsgBoxStyle.OkOnly, "Important!");
                        }
                    }

                    if (readvar.Contains("r#s!")) {
                        // read in R/C channel data
                        if (serialPort.BytesToRead > 0) {
                            bytesRead = this.serialPort.Read(rcdata, 0, 24);
                            try {
                                this.serialPort.DiscardInBuffer();
                            } catch (Exception) { }
                            try {
                                this.serialPort.DiscardOutBuffer();
                            } catch (Exception) { }
                        }
                    }

                    if (readvar.Contains("ss!")) {
                        readvar = "";
                        if (serialPort.BytesToRead > 0) {
                            bytesRead = this.serialPort.Read(sensors, 1, 13);
                            bytesRead = serialPort.Read(rcdata, 0, 24);
                            try {
                                serialPort.DiscardInBuffer();
                            } catch (Exception) { }
                            try {
                                serialPort.DiscardOutBuffer();
                            } catch (Exception) { }
                        }
                    }
                    if (this.readvar.Contains("Error reading channel:")) {
                        try {
                            errorcounter++;
                            errorlabel.Text = errorcounter.ToString();
                        } catch (Exception) { }
                    }
                    textRxD.ScrollToCaret();
                    updateboxes();
                } catch (Exception exception6) {
                    ProjectData.SetProjectError(exception6);
                    this.textRxD.AppendText("Error in understanding tricopter...");
                    ProjectData.ClearProjectError();
                }
            }
        }
    }

    public void TimersensTick(object sender, EventArgs e)
    {
        try {
            this.TrackBar1.Value = 0xff - this.sensors[1];
            this.TrackBar2.Value = 0xff - this.sensors[2];
            this.TrackBar3.Value = 0xff - this.sensors[3];
            this.TrackBar4.Value = this.sensors[4];
            this.TrackBar5.Value = this.sensors[5];
            this.TrackBar6.Value = this.sensors[6];
            this.TrackBar7.Value = this.sensors[7];
            this.TrackBar10.Value = this.sensors[8];
            this.TrackBar11.Value = this.sensors[10];
            this.TrackBar8.Value = 0xff - this.sensors[9];
            this.TrackBar9.Value = 0xff - this.sensors[11];
            this.TrackBar12.Value = this.sensors[12];
            voltsfromcopter = sensors[13];
            pbVoltage.Value = voltsfromcopter;
            labelvolts.Text = String.Format("{0:0.0}", voltsfromcopter / 10.0f) + " volts";
            
            if ((this.sensors[10] >= 0x79) & (this.sensors[10] <= 0x80)) {
                this.panel1.Visible = true;
            } else {
                this.panel1.Visible = false;
            }
            if ((this.sensors[9] >= 0x79) & (this.sensors[9] <= 0x80)) {
                this.panelRollCenter.Visible = true;
            } else {
                this.panelRollCenter.Visible = false;
            }
            if ((this.sensors[11] >= 0x79) & (this.sensors[11] <= 0x80)) {
                this.panel4.Visible = true;
            } else {
                this.panel4.Visible = false;
            }
            if (this.sensors[8] <= 12) {
                this.panel2.Visible = true;
            } else {
                this.panel2.Visible = false;
            }

            // RC channels
            MACTrackBar[] list = { tbch1, tbch2, tbch3, tbch4, tbch5, tbch6, tbch7, tbch8 };
            Label[] llist = { lbch1, lbch2, lbch3, lbch4, lbch5, lbch6, lbch7, lbch8 };
            for (int i = 0; i < 8; i++) {
                list[i].Value = rcdata[i + 1];
                llist[i].Text = "" + (rcdata[i + 1] - 127);
            }

        } catch (Exception) {
        }
    }

    public void TrackBar13Scroll(object sender, EventArgs e)
    {
        int track = this.trackBar13.Value;
        int easyP = (int)Math.Round(Math.Round((double)(21.0 + (track * 23.4))));
        int easyI = (int)Math.Round(Math.Round((double)(21.0 + (track * 21.4))));
        int easyD = (int)Math.Round(Math.Round((double)(21.0 + (track * 21.4))));
        int easyD2 = (int)Math.Round(Math.Round((double)(track * 8.9)));
        this.textBoxi.Text = easyP.ToString();
        this.textBoxj.Text = easyI.ToString();
        this.textBoxk.Text = easyD.ToString();
        this.textBoxz.Text = easyD2.ToString();
    }

    public void TrackBar14Scroll(object sender, EventArgs e)
    {
        int track = this.trackBar14.Value;
        int easyP = (int)Math.Round(Math.Round((double)(10.0 + (track * 24.5))));
        int easyI = Convert.ToInt32(Math.Round(new decimal(5 + (track * 7))));
        int easyD = Convert.ToInt32(Math.Round(new decimal(track * 9)));
        this.textBoxG.Text = easyP.ToString();
        this.textBoxH.Text = easyI.ToString();
        this.textBoxy.Text = easyD.ToString();
    }

    public void TrackBar15Scroll(object sender, EventArgs e)
    {
        int track = this.trackBar15.Value;
        int easyP = Convert.ToInt32(decimal.Add(3M, Math.Round(new decimal(track * 0x11))));
        int easyI = Convert.ToInt32(Math.Round(new decimal(track * 11)));
        this.textBoxl.Text = easyP.ToString();
        this.textBoxm.Text = easyI.ToString();
    }

    public void updateboxes()
    {
        try {
            if (!this.readsens) {
                if (MainSettings.MotorsEnabled) {
                    this.deactivatemotors.Checked = false;
                } else {
                    this.deactivatemotors.Checked = true;
                }
                this.checkBoxB.Checked = this.rollgyrodir > 0;
                this.checkBoxB2.Checked = this.rollgyrodir > 0;
                this.checkBoxC.Checked = this.nickgyrodir > 0;
                this.checkBoxC2.Checked = this.nickgyrodir > 0;
                this.checkBoxD.Checked = this.yawgyrodir > 0;
                this.checkBoxD2.Checked = this.yawgyrodir > 0;
                this.checkBoxe.Checked = this.xaccdir > 0;
                this.checkBoxe2.Checked = this.xaccdir > 0;
                this.checkBoxf.Checked = this.yaccdir > 0;
                this.checkBoxf2.Checked = this.yaccdir > 0;
                this.labelG.Text = this.acrop.ToString();
                this.textBoxG.Text = this.acrop.ToString();
                this.labelH.Text = this.acroi.ToString();
                this.textBoxH.Text = this.acroi.ToString();
                this.labeli.Text = this.hoverp.ToString();
                this.textBoxi.Text = this.hoverp.ToString();
                this.labelj.Text = this.hoveri.ToString();
                this.textBoxj.Text = this.hoveri.ToString();
                this.labelk.Text = this.hoverd.ToString();
                this.textBoxk.Text = this.hoverd.ToString();
                this.labell.Text = this.yawp.ToString();
                this.textBoxl.Text = this.yawp.ToString();
                this.labelm.Text = this.yawi.ToString();
                this.textBoxm.Text = this.yawi.ToString();
                this.labeln.Text = this.accinfluence.ToString();
                this.textBoxn.Text = this.accinfluence.ToString();
                this.labelo.Text = this.xaccscale.ToString();
                this.textBoxo.Text = this.xaccscale.ToString();
                this.labelp.Text = this.yaccscale.ToString();
                this.textBoxp.Text = this.yaccscale.ToString();
                this.labelq.Text = this.lfacro.ToString();
                this.textBoxQ.Text = this.lfacro.ToString();
                this.labelr.Text = this.lfhover.ToString();
                this.textBoxR.Text = this.lfhover.ToString();
                this.labels.Text = this.lfyaw.ToString();
                this.textBoxS.Text = this.lfyaw.ToString();
                this.checkBoxT.Checked = this.lfboost > 0;
                this.checkBoxT2.Checked = this.lfboost > 0;
                this.labelu.Text = this.minthrottle.ToString();
                this.textBoxu.Text = this.minthrottle.ToString();
                this.labelv.Text = this.voltage.ToString();
                this.textBoxv.Text = this.voltage.ToString();
                this.labelw.Text = this.xacc_offset.ToString();
                this.textBoxw.Text = this.xacc_offset.ToString();
                this.labelx.Text = this.yacc_offset.ToString();
                this.textBoxx.Text = this.yacc_offset.ToString();
                this.labely.Text = this.acrod.ToString();
                this.textBoxy.Text = this.acrod.ToString();
                this.labelz.Text = this.hoverdd.ToString();
                this.textBoxz.Text = this.hoverdd.ToString();

                this.throttlebox.SelectedIndex = this.throttlechannel - 1;
                this.Nickbox.SelectedIndex = this.nickchannel - 1;
                this.rollbox.SelectedIndex = this.rollchannel - 1;
                this.yawbox.SelectedIndex = this.yawchannel - 1;
                this.switchbox.SelectedIndex = this.switchchannel - 1;
            }
        } catch (Exception exception1) {
            ProjectData.SetProjectError(exception1);
            ProjectData.ClearProjectError();
        }
    }

    public void WriteallClick(object sender, EventArgs e)
    {
        this.progressBar2.Value = 0;
        this.labelok.Visible = false;
        this.IsAbyte = 0;
        this.nrtextboxes = 0;
        this.checkbeforeupload();
        if (this.errorintx) {
            Interaction.MsgBox("RX/ TX channels are not clearly allocated! Aborting.", MsgBoxStyle.OkOnly, "Error");
        } else if (this.IsAbyte != this.nrtextboxes) {
            Interaction.MsgBox(((this.nrtextboxes - this.IsAbyte)).ToString() + " parameters don't seem to be bytes (range: 0 to 255). Please check this (textbox will be red if parameter is not a byte).", MsgBoxStyle.OkOnly, "Error");
        } else if (!this.serialPort.IsOpen) {
            Interaction.MsgBox("You are not connected to Shrediquette DLX.", MsgBoxStyle.OkOnly, "Error");
            this.toolStatusLabel.Text = "Not connected";
        } else if (Interaction.MsgBox("Transfer all parameters?", MsgBoxStyle.OkCancel, "Sure?") == MsgBoxResult.Ok) {
            this.serialPort.Write("ci!\r");
            if (this.deactivatemotors.Checked) {
                this.outvar[1] = 123;
            } else {
                this.outvar[1] = 1;
            }
            this.outvar[2] = (byte)Convert.ToInt32(this.checkBoxB2.Checked);
            this.outvar[3] = (byte)Convert.ToInt32(this.checkBoxC2.Checked);
            this.outvar[4] = (byte)Convert.ToInt32(this.checkBoxD2.Checked);
            this.outvar[5] = (byte)Convert.ToInt32(this.checkBoxe2.Checked);
            this.outvar[6] = (byte)Convert.ToInt32(this.checkBoxf2.Checked);
            this.outvar[7] = (byte)Convert.ToInt32(this.textBoxG.Text);
            this.outvar[8] = (byte)Convert.ToInt32(this.textBoxH.Text);
            this.outvar[9] = (byte)Convert.ToInt32(this.textBoxi.Text);
            this.outvar[10] = (byte)Convert.ToInt32(this.textBoxj.Text);
            this.outvar[11] = (byte)Convert.ToInt32(this.textBoxk.Text);
            this.outvar[12] = (byte)Convert.ToInt32(this.textBoxl.Text);
            this.outvar[13] = (byte)Convert.ToInt32(this.textBoxm.Text);
            this.outvar[14] = (byte)Convert.ToInt32(this.textBoxn.Text);
            this.outvar[15] = (byte)Convert.ToInt32(this.textBoxo.Text);
            this.outvar[0x10] = (byte)Convert.ToInt32(this.textBoxp.Text);
            this.outvar[0x11] = (byte)Convert.ToInt32(this.textBoxQ.Text);
            this.outvar[0x12] = (byte)Convert.ToInt32(this.textBoxR.Text);
            this.outvar[0x13] = (byte)Convert.ToInt32(this.textBoxS.Text);
            this.outvar[20] = (byte)Convert.ToInt32(this.checkBoxT2.Checked);
            this.outvar[0x15] = (byte)Convert.ToInt32(this.textBoxu.Text);
            this.outvar[0x16] = (byte)Convert.ToInt32(this.textBoxv.Text);
            this.outvar[0x17] = (byte)Convert.ToInt32(this.textBoxw.Text);
            this.outvar[0x18] = (byte)Convert.ToInt32(this.textBoxx.Text);
            this.outvar[0x19] = (byte)(this.throttlebox.SelectedIndex + 1);
            this.outvar[0x1a] = (byte)(this.Nickbox.SelectedIndex + 1);
            this.outvar[0x1b] = (byte)(this.rollbox.SelectedIndex + 1);
            this.outvar[0x1c] = (byte)(this.yawbox.SelectedIndex + 1);
            this.outvar[0x1d] = (byte)Convert.ToInt32(this.textBoxy.Text);
            this.outvar[30] = (byte)Convert.ToInt32(this.textBoxz.Text);
            this.outvar[31] = (byte)(this.switchbox.SelectedIndex + 1);
            this.outvar[0x20] = 0;
            this.outvar[0x21] = 0;
            this.serialPort.Write(this.outvar, 1, 0x21);
            this.timer1.Enabled = false;
            this.k = 1;
            do {
                Thread.Sleep(80);
                this.progressBar2.PerformStep();
                this.k++;
            }
            while (this.k <= 0x21);
            this.getparams();
            this.timer1.Enabled = true;
            this.progressBar2.Value = this.progressBar2.Maximum;
        }
    }

    public void writesettingsClick(object sender, EventArgs e)
    {
        try {
            if (this.deactivatemotors.Checked) {
                this.outvar[1] = 123;
            } else {
                this.outvar[1] = 1;
            }
            this.outvar[2] = (byte)Convert.ToInt32(this.checkBoxB2.Checked);
            this.outvar[3] = (byte)Convert.ToInt32(this.checkBoxC2.Checked);
            this.outvar[4] = (byte)Convert.ToInt32(this.checkBoxD2.Checked);
            this.outvar[5] = (byte)Convert.ToInt32(this.checkBoxe2.Checked);
            this.outvar[6] = (byte)Convert.ToInt32(this.checkBoxf2.Checked);
            this.outvar[7] = (byte)Convert.ToInt32(this.textBoxG.Text);
            this.outvar[8] = (byte)Convert.ToInt32(this.textBoxH.Text);
            this.outvar[9] = (byte)Convert.ToInt32(this.textBoxi.Text);
            this.outvar[10] = (byte)Convert.ToInt32(this.textBoxj.Text);
            this.outvar[11] = (byte)Convert.ToInt32(this.textBoxk.Text);
            this.outvar[12] = (byte)Convert.ToInt32(this.textBoxl.Text);
            this.outvar[13] = (byte)Convert.ToInt32(this.textBoxm.Text);
            this.outvar[14] = (byte)Convert.ToInt32(this.textBoxn.Text);
            this.outvar[15] = (byte)Convert.ToInt32(this.textBoxo.Text);
            this.outvar[0x10] = (byte)Convert.ToInt32(this.textBoxp.Text);
            this.outvar[0x11] = (byte)Convert.ToInt32(this.textBoxQ.Text);
            this.outvar[0x12] = (byte)Convert.ToInt32(this.textBoxR.Text);
            this.outvar[0x13] = (byte)Convert.ToInt32(this.textBoxS.Text);
            this.outvar[20] = (byte)Convert.ToInt32(this.checkBoxT2.Checked);
            this.outvar[0x15] = (byte)Convert.ToInt32(this.textBoxu.Text);
            this.outvar[0x16] = (byte)Convert.ToInt32(this.textBoxv.Text);
            this.outvar[0x17] = (byte)Convert.ToInt32(this.textBoxw.Text);
            this.outvar[0x18] = (byte)Convert.ToInt32(this.textBoxx.Text);
            this.outvar[0x19] = (byte)Convert.ToInt32((int)(this.throttlebox.SelectedIndex + 1));
            this.outvar[0x1a] = (byte)Convert.ToInt32((int)(this.Nickbox.SelectedIndex + 1));
            this.outvar[0x1b] = (byte)Convert.ToInt32((int)(this.rollbox.SelectedIndex + 1));
            this.outvar[0x1c] = (byte)Convert.ToInt32((int)(this.yawbox.SelectedIndex + 1));
            this.outvar[0x1d] = (byte)Convert.ToInt32(this.textBoxy.Text);
            this.outvar[30] = (byte)Convert.ToInt32(this.textBoxz.Text);
            this.outvar[31] = (byte)Convert.ToInt32((int)(this.switchbox.SelectedIndex + 1));
            this.outvar[0x20] = 0;
            this.outvar[0x21] = 0;
            try {
                this.saveFileDialog1.InitialDirectory = Environment.CurrentDirectory;
                this.saveFileDialog1.ShowDialog();
                string saveas = this.saveFileDialog1.FileName;
                if (saveas != "") {
                    StreamWriter textfile = new StreamWriter(saveas);
                    int VBt_i4L0 = this.outvar.Length - 1;
                    this.k = 1;
                    while (this.k <= VBt_i4L0) {
                        textfile.WriteLine(this.outvar[this.k].ToString());
                        this.k++;
                    }
                    textfile.Close();
                }
            } catch (Exception exception1) {
                ProjectData.SetProjectError(exception1);
                Interaction.MsgBox("Error. Most likely, you don't have the latest .NET framework installed.", MsgBoxStyle.OkOnly, null);
                ProjectData.ClearProjectError();
            }
        } catch (Exception exception2) {
            ProjectData.SetProjectError(exception2);
            Interaction.MsgBox("(Some) parameter fields are empty.", MsgBoxStyle.OkOnly, null);
            ProjectData.ClearProjectError();
        }
    }

    private void OnLedMaskChange()
    {
        Debug.WriteLine("Crap: " + led1mask.Value);
    }
}
