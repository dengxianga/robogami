using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using CppCsBridge;
using Timer = System.Windows.Forms.Timer;
using System.Diagnostics;

namespace FBE_CSharpUI
{
    public class TaskLogs
    {
        private Timer taskLongTimer;
        private int taskId;
        private string outfilename;
        private DateTime startTime;
        private TimeSpan duration;
        private System.IO.StreamWriter file;
        private List<string> lines;
        private Stopwatch stopWatch;
        private UIInstance uiInstance;
        public TaskLogs(int taskId, string outfilename, UIInstance uiInstance)
        {
            this.outfilename = outfilename;
            this.taskId = taskId;
            lines = new List<string>();
            this.uiInstance = uiInstance;
        }

        public void start()
        {
            startTime = DateTime.Now;
            duration = new TimeSpan(0, 0, 1);

            stopWatch = new Stopwatch();
            stopWatch.Start();


            // open file for writing 

            // if task one output the wobbliness
            if (taskId == 1)
            {
                taskLongTimer = new Timer();
                taskLongTimer.Tick += new EventHandler((object sender1, EventArgs e1) =>
                {
                    double top = uiInstance.getToppling();
                    double speed = uiInstance.getSpeed(); 
                    string topplling = " toppling = " + top + " speed = " + speed;
                    lines.Add(topplling);
                });

                taskLongTimer.Interval = 1000*2 ;
                taskLongTimer.Enabled = true;

            }



        }

        public void end()
        {

            stopWatch.Stop();
            TimeSpan ts = stopWatch.Elapsed;

            // Format and display the TimeSpan value.
            string elapsedTime = String.Format("{0:00}:{1:00}:{2:00}.{3:00}",
                ts.Hours, ts.Minutes, ts.Seconds,
                ts.Milliseconds / 10);
            string timetofinish = "RunTime " + elapsedTime;
            lines.Add(timetofinish);
            System.IO.File.WriteAllLines(outfilename, lines);
        }


    }

}
