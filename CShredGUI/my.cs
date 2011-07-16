using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.VisualBasic.Devices;
using System.ComponentModel;
using System.Diagnostics;
using Microsoft.VisualBasic.ApplicationServices;
using Microsoft.VisualBasic;
using Microsoft.VisualBasic.CompilerServices;
using System.ComponentModel.Design;
using System.Windows.Forms;
using System.CodeDom.Compiler;
using System.Collections;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace TriGUI_v11.My
{
#if false
    [EditorBrowsable(EditorBrowsableState.Never), GeneratedCode("MyTemplate", "8.0.0.0")]
    internal class MyApplication : WindowsFormsApplicationBase
    {
        // Methods
        public MyApplication();
        [EditorBrowsable(EditorBrowsableState.Advanced), DebuggerHidden, STAThread]
        internal static void Main(string[] Args);
        protected override void OnCreateMainForm();
    }

    [GeneratedCode("MyTemplate", "8.0.0.0"), EditorBrowsable(EditorBrowsableState.Never)]
    internal class MyComputer : Computer
    {
        // Methods
        [EditorBrowsable(EditorBrowsableState.Never), DebuggerHidden]
        public MyComputer();
    }

    [GeneratedCode("MyTemplate", "8.0.0.0"), HideModuleName, StandardModule]
    internal sealed class MyProject
    {
        // Fields
        private static readonly ThreadSafeObjectProvider<MyApplication> m_AppObjectProvider;
        private static readonly ThreadSafeObjectProvider<MyComputer> m_ComputerObjectProvider;
        private static ThreadSafeObjectProvider<MyForms> m_MyFormsObjectProvider;
        private static readonly ThreadSafeObjectProvider<MyWebServices> m_MyWebServicesObjectProvider;
        private static readonly ThreadSafeObjectProvider<User> m_UserObjectProvider;

        // Methods
        [DebuggerNonUserCode]
        static MyProject();

        // Properties
        [HelpKeyword("My.Application")]
        internal static MyApplication Application { [DebuggerHidden] get; }
        [HelpKeyword("My.Computer")]
        internal static MyComputer Computer { [DebuggerHidden] get; }
        [HelpKeyword("My.Forms")]
        internal static MyForms Forms { [DebuggerHidden] get; }
        [HelpKeyword("My.User")]
        internal static User User { [DebuggerHidden] get; }
        [HelpKeyword("My.WebServices")]
        internal static MyWebServices WebServices { [DebuggerHidden] get; }

        // Nested Types
        [EditorBrowsable(EditorBrowsableState.Never), MyGroupCollection("System.Windows.Forms.Form", "Create__Instance__", "Dispose__Instance__", "My.MyProject.Forms")]
        internal sealed class MyForms
        {
            // Fields
            [ThreadStatic]
            private static Hashtable m_FormBeingCreated;
            public MainForm m_MainForm;
            public settingscompare m_settingscompare;

            // Methods
            [DebuggerHidden, EditorBrowsable(EditorBrowsableState.Never)]
            public MyForms();
            [DebuggerHidden]
            private static T Create__Instance__<T>(T Instance) where T : Form, new();
            [DebuggerHidden]
            private void Dispose__Instance__<T>(ref T instance) where T : Form;
            [EditorBrowsable(EditorBrowsableState.Never)]
            public override bool Equals(object o);
            [EditorBrowsable(EditorBrowsableState.Never)]
            public override int GetHashCode();
            [EditorBrowsable(EditorBrowsableState.Never)]
            internal Type GetType();
            [EditorBrowsable(EditorBrowsableState.Never)]
            public override string ToString();

            // Properties
            public MainForm MainForm { [DebuggerNonUserCode] get; [DebuggerNonUserCode] set; }
            public settingscompare settingscompare { [DebuggerNonUserCode] get; [DebuggerNonUserCode] set; }
        }

        [MyGroupCollection("System.Web.Services.Protocols.SoapHttpClientProtocol", "Create__Instance__", "Dispose__Instance__", ""), EditorBrowsable(EditorBrowsableState.Never)]
        internal sealed class MyWebServices
        {
            // Methods
            [EditorBrowsable(EditorBrowsableState.Never), DebuggerHidden]
            public MyWebServices();
            [DebuggerHidden]
            private static T Create__Instance__<T>(T instance) where T : new();
            [DebuggerHidden]
            private void Dispose__Instance__<T>(ref T instance);
            [DebuggerHidden, EditorBrowsable(EditorBrowsableState.Never)]
            public override bool Equals(object o);
            [EditorBrowsable(EditorBrowsableState.Never), DebuggerHidden]
            public override int GetHashCode();
            [DebuggerHidden, EditorBrowsable(EditorBrowsableState.Never)]
            internal Type GetType();
            [EditorBrowsable(EditorBrowsableState.Never), DebuggerHidden]
            public override string ToString();
        }

        [ComVisible(false), EditorBrowsable(EditorBrowsableState.Never)]
        internal sealed class ThreadSafeObjectProvider<T> where T : new()
        {
            // Fields
            [ThreadStatic, CompilerGenerated]
            private static T m_ThreadStaticValue;

            // Methods
            [DebuggerHidden, EditorBrowsable(EditorBrowsableState.Never)]
            public ThreadSafeObjectProvider();

            // Properties
            internal T GetInstance { [DebuggerHidden] get; }
        }
    }
#endif
}

