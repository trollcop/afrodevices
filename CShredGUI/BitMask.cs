using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace TriGUI_v11
{
    public partial class BitMask : UserControl
    {
        private byte BitValue = 0;
        private CheckBox[] boxes;

        public delegate void Changed();
        public event Changed OnChange;

        public BitMask()
        {
            InitializeComponent();
            boxes = new CheckBox[8];
            boxes[0] = bit8;
            boxes[1] = bit7;
            boxes[2] = bit6;
            boxes[3] = bit5;
            boxes[4] = bit4;
            boxes[5] = bit3;
            boxes[6] = bit2;
            boxes[7] = bit1;
        }

        private void RecalcBits(object sender, EventArgs e)
        {
            CheckBox box = sender as CheckBox;
            for (int i = 0; i < 8; i++) {
                if (box == boxes[i]) {
                    if (box.Checked)
                        BitValue |= (byte)(1 << i);
                    else
                        BitValue &= (byte)~(1 << i);
                }
            }
            if (OnChange != null)
                OnChange();
        }

        private void Recalc()
        {
            for (int i = 0; i < 8; i++) {
                boxes[i].Checked = (BitValue & (byte)(1 << i)) > 0;
            }
        }

        public byte Value
        {
            set
            {
                BitValue = value;
                Recalc();
            }
            get
            {
                return BitValue;
            }
        }

    }
}
