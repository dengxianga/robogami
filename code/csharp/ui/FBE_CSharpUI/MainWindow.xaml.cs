using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing.Printing;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Interop;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Forms;
using System.Drawing;
using CppCsBridge;
using FabByExample.proto;
using HelixToolkit.Wpf;
using Xceed.Wpf.Toolkit;
using DragDropEffects = System.Windows.DragDropEffects;
using DragEventArgs = System.Windows.DragEventArgs;
using Drawing = CppCsBridge.Drawing;
using KeyEventArgs = System.Windows.Input.KeyEventArgs;
using Label = System.Windows.Controls.Label;
using MenuItem = System.Windows.Controls.MenuItem;
using OpenFileDialog = Microsoft.Win32.OpenFileDialog;
using Orientation = System.Windows.Controls.Orientation;
using Polygon = System.Windows.Shapes.Polygon;
using SaveFileDialog = Microsoft.Win32.SaveFileDialog;
using Timer = System.Windows.Forms.Timer;

/*
 * Some icons are from here
 * <div>Icons made by <a href="http://www.flaticon.com/authors/simpleicon" 
 * title="SimpleIcon">SimpleIcon</a>, 
 * <a href="http://www.flaticon.com/authors/budi-tanrim" 
 * title="Budi Tanrim">Budi Tanrim</a>, 
 * <a href="http://www.flaticon.com/authors/freepik" 
 * title="Freepik">Freepik</a> from <a href="http://www.flaticon.com" 
 * title="Flaticon">www.flaticon.com</a>
 * is licensed by <a href="http://creativecommons.org/licenses/by/3.0/" 
 * title="Creative Commons BY 3.0">CC BY 3.0</a></div>
 * 
 * <div>Icons made by <a href="http://www.flaticon.com/authors/freepik" title="Freepik">Freepik</a> from <a href="http://www.flaticon.com" 
 * title="Flaticon">www.flaticon.com</a>             is licensed by <a href="http://creativecommons.org/licenses/by/3.0/" 
 * title="Creative Commons BY 3.0">CC BY 3.0</a></div>
 * 
 * <div>Icons made by <a href="http://www.flaticon.com/authors/freepik" title="Freepik">Freepik</a>, 
 * <a href="http://www.flaticon.com/authors/daniel-bruce" 
 * title="Daniel Bruce">Daniel Bruce</a>, <a href="http://www.flaticon.com/authors/sarfraz-shoukat" 
 * title="Sarfraz Shoukat">Sarfraz Shoukat</a>, <a href="http://www.flaticon.com/authors/picol" 
 * title="Picol">Picol</a>, <a href="http://www.flaticon.com/authors/icomoon" title="Icomoon">Icomoon</a>, 
 * <a href="http://www.flaticon.com/authors/appzgear" title="Appzgear">Appzgear</a>, 
 * <a href="http://www.flaticon.com/authors/ocha" title="OCHA">OCHA</a>, 
 * <a href="http://www.flaticon.com/authors/simpleicon" title="SimpleIcon">SimpleIcon</a>, 
 * <a href="http://www.flaticon.com/authors/budi-tanrim" title="Budi Tanrim">Budi Tanrim</a> from <a href="http://www.flaticon.com" 
 * title="Flaticon">www.flaticon.com</a>             is licensed by <a href="http://creativecommons.org/licenses/by/3.0/" 
 * title="Creative Commons BY 3.0">CC BY 3.0</a></div>
*/


namespace FBE_CSharpUI {
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 


    public static class Prompt
    {
        public static int ShowDialog()
        {
            Form prompt = new Form();
            prompt.Width = 300;
            prompt.Height = 200;
            prompt.Text = "Animation Parameter";
            System.Windows.Forms.Label textLabel = new System.Windows.Forms.Label() { Left = 0, Top = 0, Text = "Number of Cycles:" };
            NumericUpDown inputBox = new NumericUpDown() { Left = 100, Top = 0, Width = 40 };
            inputBox.Value = 3;
            System.Windows.Forms.Label textLabel2 = new System.Windows.Forms.Label() { Left = 0, Top = 30, Text = "Step size:" };
            NumericUpDown inputBox2 = new NumericUpDown() { Left = 80, Top = 30, Width = 60 };
            inputBox2.DecimalPlaces = 3;
            inputBox2.Value = new decimal(new int[] {1, 0, 0, 65536});
            System.Windows.Forms.Button confirmation = new System.Windows.Forms.Button() { Text = "Ok", Left = 100, Width = 50, Top = 60 };
            confirmation.Click += (sender, e) => { prompt.Close(); };
            prompt.Controls.Add(confirmation);
            prompt.Controls.Add(textLabel);
            prompt.Controls.Add(inputBox);
            prompt.Controls.Add(textLabel2);
            prompt.Controls.Add(inputBox2);
            prompt.ShowDialog();
            return (int)inputBox.Value;
        }
    }



    public partial class MainWindow : Window {
        private const int GWL_STYLE = -16;
        private const int WS_SYSMENU = 0x80000;
        [DllImport("user32.dll", SetLastError = true)]
        private static extern int GetWindowLong(IntPtr hWnd, int nIndex);
        [DllImport("user32.dll")]
        private static extern int SetWindowLong(IntPtr hWnd, int nIndex, int dwNewLong);

        private const bool ENABLE_INITIAL_DRAGGING = true;

        private UIInstance uiInstance;
        private Sidebar sidebar;
        private const int thumbnailSize = 64;
        private List<ImageSource> savedViewThumbnails = new List<ImageSource>();
        private List<int> savedViewIds = new List<int>();
        private UI3D _ui3D;
        private UIStates _uiStates;
        private System.Windows.Point? lastPosition1;
        private System.Windows.Point? lastPositionSeq;
        private TemplateRef templateToAdd;
        private SaveFileDialog dialog;
        private Command command;
        private List<Mesh> ghostMesh;
        private List<Mesh> ghostMeshSave;
        private Drawing topViewDrawing;
         private BitmapImage topViewImage;
        private Vector drawing_translate;
        private Vector drawing_size; 
        private List<System.Windows.Point> jointLocations;
        private JointChoices jointChoices;
        List<int> currentSequence;
        TaskLogs taskLogs; 
        int draggedLabel;
        private double requiredSpeedTask;
        private bool doAnimation = false;
        private Timer animateTimer;

        private static string defaultPath = "..\\..\\data\\";
        //private static string defaultPath = "..\\Robogami\\data\\";

        public static readonly DependencyProperty IsShiftDownProperty = DependencyProperty.Register(
            "IsShiftDown", typeof (bool), typeof (MainWindow), new PropertyMetadata(default(bool)));

        public bool IsShiftDown {
            get { return (bool) GetValue(IsShiftDownProperty); }
            set { SetValue(IsShiftDownProperty, value); }
        }

        public static readonly DependencyProperty IsCtrlDownProperty = DependencyProperty.Register(
            "IsCtrlDown", typeof (bool), typeof (MainWindow), new PropertyMetadata(default(bool)));

        private static readonly RoutedUICommand[] _loadViewCommands = new[] {
            UICommands.LoadView1, UICommands.LoadView2,
            UICommands.LoadView3, UICommands.LoadView4
        };

        private Debugging Debugging;

        public bool IsCtrlDown {
            get { return (bool) GetValue(IsCtrlDownProperty); }
            set { SetValue(IsCtrlDownProperty, value); }
        }




        public BitmapImage ConvertDrawingToImage(Drawing drawing){

            //compute size of the drawing:
            double drawing_xmin, drawing_xmax, drawing_ymin, drawing_ymax;
            List<double> Xs = new List<double>();
            List<double> Ys = new List<double>();
            foreach (var face in drawing.faces)
            {
                foreach (var point in face.points)
                {
                    Xs.Add(point.X);
                    Ys.Add(point.Y);
                }
            }
            if (Xs.Count == 0 || Ys.Count == 0) return null;
            drawing_xmin = Xs.Min();
            drawing_xmax = Xs.Max();
            drawing_ymin = Ys.Min();
            drawing_ymax = Ys.Max();

            double width = drawing_xmax - drawing_xmin;
            double height = drawing_ymax - drawing_ymin;
            drawing_translate = new Vector(drawing_xmin, drawing_ymin);
            drawing_size = new Vector(width, height); 

            // create the canvas and draw the drawing
            Canvas auxCanvas = new Canvas();
            //auxCanvas.Background = new SolidColorBrush(Colors.Red);
            auxCanvas.Width = width;
            auxCanvas.Height = height;
            System.Windows.Size size = new System.Windows.Size(auxCanvas.Width, auxCanvas.Height);
            auxCanvas.Measure(size);
            auxCanvas.Arrange(new Rect(size));
            foreach (var face in drawing.faces)
            {
                Polygon polygon = new Polygon();
                foreach (var point in face.points)
                {
                    polygon.Points.Add(point - drawing_translate);
                }
                polygon.Fill = new SolidColorBrush(System.Windows.Media.Color.FromArgb(255, 160, 160, 160));
                auxCanvas.Children.Add(polygon);
            }

            //Console.Write(" drawing_translate.x = " + drawing_translate.X);
            //Console.Write(" drawing_translate.y = " + drawing_translate.Y);

            Transform transform = auxCanvas.LayoutTransform;
            // reset current transform (in case it is scaled or rotated)
            auxCanvas.LayoutTransform = null;

            // Get the size of canvas
            //System.Windows.Size size = new System.Windows.Size(auxCanvas.Width, auxCanvas.Height);
            // Measure and arrange the surface
            // VERY IMPORTANT
            auxCanvas.Measure(size);
            auxCanvas.Arrange(new Rect(size));

            // Create a render bitmap and push the surface to it
            RenderTargetBitmap renderBitmap =
            new RenderTargetBitmap(
                (int)size.Width,
                (int)size.Height,
                96d,
                96d,
                PixelFormats.Pbgra32);
            renderBitmap.Render(auxCanvas);

            // Create a file stream for saving image
            using (FileStream outStream = new FileStream(MainWindow.defaultPath + "bitmaoptest2.jpg", FileMode.Create))
            {
                // Use png encoder for our data
                PngBitmapEncoder encoder = new PngBitmapEncoder();
                // push the rendered bitmap to it
                encoder.Frames.Add(BitmapFrame.Create(renderBitmap));
                // save the data to the stream
                encoder.Save(outStream);
            }

            // Restore previously saved layout
            auxCanvas.LayoutTransform = transform;



            //create the bitmap image
            var bitmapImage = new BitmapImage();
            var bitmapEncoder = new PngBitmapEncoder();
            bitmapEncoder.Frames.Add(BitmapFrame.Create(renderBitmap));

            using (var stream = new MemoryStream())
            {
                bitmapEncoder.Save(stream);
                stream.Seek(0, SeekOrigin.Begin);

                bitmapImage.BeginInit();
                bitmapImage.CacheOption = BitmapCacheOption.OnLoad;
                bitmapImage.StreamSource = stream;
                bitmapImage.EndInit();
            }

            saveBitmapImageToFile(bitmapImage, MainWindow.defaultPath + "bitmaop.jpg");

                
            return bitmapImage;

        }

        private void saveBitmapImageToFile(BitmapImage image, String filename)
        {

            JpegBitmapEncoder encoder = new JpegBitmapEncoder();
            Guid photoID = System.Guid.NewGuid();

            encoder.Frames.Add(BitmapFrame.Create(image));

            using (var filestream = new FileStream(filename, FileMode.Create))
            encoder.Save(filestream);
        }


        public void updateMetrics()
        {


            //clear all and set the visibility for default
            MetricsGrid.Items.Clear();
            MetricsGrid1.Items.Clear();
            MetricsGrid1B.Items.Clear();
            MetricsGrid2.Items.Clear();
            MetricsGridSpeed.Items.Clear();
            MetricsGrid2.Visibility = System.Windows.Visibility.Collapsed;
            MetricsGrid1.Visibility = System.Windows.Visibility.Visible;
            MetricsGrid1B.Visibility = System.Windows.Visibility.Visible;
            MetricsGrid.Visibility = System.Windows.Visibility.Visible;
            MetricsGridSpeed.Visibility = System.Windows.Visibility.Collapsed;

            if (_uiStates.AvoidObstacle)
            {
                MetricsGridSpeed.Visibility = System.Windows.Visibility.Visible;
            }
          
            DrawingCanvas.Children.Clear();


            List<PointList2D> gaitPaths = new List<PointList2D>();
            List<double> rotationAngles = new List<double>();


            if(uiInstance.hasMainWorkingTemp()){

            if (_uiStates.ForwardTravel) // just update one gait and show one tab
            {
                MetricsGrid2.Visibility = System.Windows.Visibility.Visible;
                MetricsGrid1B.Visibility = System.Windows.Visibility.Visible;
                MetricsGrid1B.Visibility = System.Windows.Visibility.Collapsed;
                MetricsGrid.Visibility = System.Windows.Visibility.Collapsed;

                int i = 0;
                uiInstance.updateControllers(i);
                uiInstance.updateMetrics(_uiStates.NAnimRounds, _uiStates.DeltaAnim, false);
                string name = GaitList_ComboBox.Items[i].ToString();
                        

                bool achievedTaskSpeed = false;
                bool achievedTaskTopping = false;
                if (uiInstance.getSpeed() >= requiredSpeedTask)
                {
                    achievedTaskSpeed = true;
                }
                if (uiInstance.getToppling() <= 0.5)
                {
                    achievedTaskTopping = true;
                }

                MetricsGrid2.Items.Add(new { Param = name, error1 = achievedTaskSpeed, error2 = achievedTaskTopping });
                MetricsGrid1.Items.Add(new
                {
                    modelName = name,
                    speed = String.Format("{0:0.00}", uiInstance.getSpeed()),
                    wobbliness = String.Format("{0:0.00}", uiInstance.getToppling()),
                    ID = i

                });

                PointList2D gaitPath = uiInstance.getGaitPath();
                gaitPaths.Add(gaitPath);


            }
            else
            {
                MetricsGrid.Items.Add(new { FabCost = "Material (g)", Cost = String.Format("{0:0.00}", uiInstance.getFabCost()) });
                MetricsGrid.Items.Add(new { FabCost = "Electronics cost (US$)", Cost = String.Format("{0:0.00}", uiInstance.getElectronicsCost()) });
                MetricsGrid.Items.Add(new { FabCost = "Total Mass (g)", Cost = String.Format("{0:0.00}", uiInstance.getTotalMass()) });


                


                for (int i = -1; i < GaitList_ComboBox.Items.Count; i++)
                {

                    string name = "Sequence";
                    if (i >= 0)
                    {
                        uiInstance.updateControllers(i);
                        uiInstance.updateMetrics(_uiStates.NAnimRounds, _uiStates.DeltaAnim, false);
                        name = GaitList_ComboBox.Items[i].ToString();

                    }
                    else
                    {
                        uiInstance.updateMetrics(_uiStates.NAnimRounds, _uiStates.DeltaAnim, true);
                        double totaltime = uiInstance.getTotalSequenceTime();
                        //Console.WriteLine("total time" + totaltime);
                        MetricsGridSpeed.Items.Add(new { Name = "Total Sequence time: ", val = String.Format("{0:0.00}", totaltime) });

                    }

                    MetricsGrid1.Items.Add(new
                        {
                            modelName = name,
                            speed = String.Format("{0:0.00}", uiInstance.getSpeed()),
                            wobbliness = String.Format("{0:0.00}", uiInstance.getToppling()),
                            slip = String.Format("{0:0.00}", uiInstance.getSlip()),
                            ID = i

                        });

                    if (i >= 0)
                    {
                        MetricsGrid1B.Items.Add(new
                        {
                            modelName = name,
                            curvature = String.Format("{0:0.00}", uiInstance.getCurvature()),
                            variance = String.Format("{0:0.00}", uiInstance.getPerpendicularError()),
                            angle = String.Format("{0:0.00}", (180/3.14*uiInstance.getRotation())),
                            ID = i
                        });
                            rotationAngles.Add(uiInstance.getRotation()*180/3.14* _uiStates.NAnimRounds);

                    }

                    PointList2D gaitPath = uiInstance.getGaitPath();
                    gaitPaths.Add(gaitPath);
                }
            }


            if (_uiStates.DisableAllStabilityGuidance)
            {
                MetricsGrid1.Visibility = System.Windows.Visibility.Collapsed;
            }
            else
            {

                MetricsGrid1.Visibility = System.Windows.Visibility.Visible;
                DrawingPath.Draw(_uiStates, gaitPaths, DrawingCanvas, topViewImage, drawing_translate, drawing_size, topViewDrawing, _uiStates.AvoidObstacle,
                    rotationAngles,  uiInstance.getInterpSequence());
            }
            
            updateSelectedMetricCells();
        }
       }

 private void updateSelectedMetricCells()
    { 
        if (_uiStates.selecteMetric_Objective > 0)
        {
            if (_uiStates.selecteMetric_Objective < 4)
            {

                MetricsGrid1.CurrentCell = new DataGridCellInfo(MetricsGrid1.Items[_uiStates.selecteMetric_Gait], MetricsGrid1.Columns[_uiStates.selecteMetric_Objective]);
                MetricsGrid1.SelectedCells.Clear();
                MetricsGrid1.SelectedCells.Add(MetricsGrid1.CurrentCell);
                MetricsGrid1B.UnselectAllCells();

            }
            else
            {
                MetricsGrid1B.CurrentCell = new DataGridCellInfo(MetricsGrid1B.Items[_uiStates.selecteMetric_Gait], MetricsGrid1B.Columns[_uiStates.selecteMetric_Objective-3]);
                MetricsGrid1B.SelectedCells.Clear();
                MetricsGrid1B.SelectedCells.Add(MetricsGrid1.CurrentCell);
                MetricsGrid1.UnselectAllCells();
            }
        }
    }



 private void metrics1_SelectionChanged(object sender, EventArgs e)
{
    if (MetricsGrid1.CurrentItem != null)
    {
        MetricsGrid1B.UnselectAllCells();
        bool isUnselect = true;
        bool selectedColumn = false; 
        var currentRowIndex = MetricsGrid1.Items.IndexOf(MetricsGrid1.CurrentItem);
        if (currentRowIndex != _uiStates.selecteMetric_Gait)
        {
            isUnselect = false;
        }
        _uiStates.selecteMetric_Gait = currentRowIndex;

        if (MetricsGrid1.CurrentColumn.Header.Equals("Speed (mm/s)"))
        {
            //Console.WriteLine("selecteed the speed on column " + currentRowIndex);
            if (_uiStates.selecteMetric_Objective != 1)
            {
                isUnselect = false;
            }     
            _uiStates.selecteMetric_Objective = 1;
            selectedColumn = true;
        }
        if (MetricsGrid1.CurrentColumn.Header.Equals("Wobbliness (rad)"))
        {
            //Console.WriteLine("selecteed the speed on column " + currentRowIndex);
            if (_uiStates.selecteMetric_Objective != 2)
            {
                isUnselect = false;
            }
            selectedColumn = true;
            _uiStates.selecteMetric_Objective = 2;
        }

        if (MetricsGrid1.CurrentColumn.Header.Equals("Slip (mm)"))
        {
            //Console.WriteLine("selecteed the speed on column " + currentRowIndex);
            if (_uiStates.selecteMetric_Objective != 3)
            {
                isUnselect = false;
            }     
            _uiStates.selecteMetric_Objective = 3;
            selectedColumn = true;

        }

        if (!selectedColumn || isUnselect)
        {
            _uiStates.selecteMetric_Objective = -1;
            //MetricsGrid1.CurrentItem = null;
            MetricsGrid1.SelectedCells.Clear();
            //updateMetrics();
            
        }

        //MetricsGrid1B.UnselectAllCells();
       
    }        
}



 private void metrics2_SelectionChanged(object sender, EventArgs e)
 {
     bool isUnselect = true;
     bool selectedColumn = false ;
     if (MetricsGrid1B.CurrentItem != null)
     {
         var currentRowIndex = MetricsGrid1B.Items.IndexOf(MetricsGrid1B.CurrentItem);
         if (currentRowIndex != _uiStates.selecteMetric_Gait)
         {
             isUnselect = false;
         }
         _uiStates.selecteMetric_Gait = currentRowIndex;
         if (MetricsGrid1B.CurrentColumn.Header.Equals("Curvature ()"))
         {
             if (_uiStates.selecteMetric_Objective != 4)
             {
                 isUnselect = false;
             }            
             _uiStates.selecteMetric_Objective = 4;
             selectedColumn = true;

         }
         if (MetricsGrid1B.CurrentColumn.Header.Equals("Variance (mm)"))
         {
             if (_uiStates.selecteMetric_Objective != 5)
             {
                 isUnselect = false;
             }
             selectedColumn = true;
             _uiStates.selecteMetric_Objective = 5;
         }
         if (MetricsGrid1B.CurrentColumn.Header.Equals("Turn angle (rad)"))
         {
             if (_uiStates.selecteMetric_Objective != 6)
             {
                 isUnselect = false;
             }
             selectedColumn = true;
             _uiStates.selecteMetric_Objective = 6;
         }

         MetricsGrid1.UnselectAllCells();

         if (!selectedColumn || isUnselect)
         {
             _uiStates.selecteMetric_Objective = -1;
             //MetricsGrid1B.CurrentItem = null;
             MetricsGrid1B.UnselectAllCells();
             //updateMetrics();

         }
     }
 }


        public void initializeUIStates()
        {
            _uiStates = new UIStates();
            _uiStates.DeltaAnim = 0.01;
            _uiStates.NAnimRounds = 3;
            _uiStates.useSteadyState = false;
            _uiStates.PreventCollisions = true;
            _uiStates.TranslationCache = new TranslationCache(uiInstance);
            _uiStates.ForwardTravel = false;
            _uiStates.DesignCar = false;
            _uiStates.AlwaysSnapToGround = false;
            _uiStates.DisableAllStabilityGuidance = false;
            _uiStates.UserStudy_ID = 18; //define user ID
            _uiStates.saveFileName = "..\\..\\data\\userStudy\\userid_" +
    Convert.ToString(_uiStates.UserStudy_ID) + "\\task0.asciiproto";
            _uiStates.taskId = 0;
            _uiStates.taskLogFilename = "..\\..\\data\\userStudy\\userid_" +
            Convert.ToString(_uiStates.UserStudy_ID) + "\\task0_log.txt";
            requiredSpeedTask = 20000;
            _uiStates.motionSequenceLen = 5;
            _uiStates.SelectionChanged += _uiStates_SelectionChanged;
            _uiStates.DebugSelectionChanged += () => _ui3D.DrawLast();
            _uiStates.showGhost = false;
            _uiStates.niceColors = new List<System.Windows.Media.Brush>();
            _uiStates.niceColorsNoBrush = new List<System.Windows.Media.Color>();
            _uiStates.selecteMetric_Objective = -1;
            _uiStates.selecteMetric_Gait = -1;
        
            _uiStates.niceColors.Add(new SolidColorBrush(System.Windows.Media.Color.FromArgb(200, 91, 192, 235)));
            _uiStates.niceColors.Add(new SolidColorBrush(System.Windows.Media.Color.FromArgb(200, 250, 121, 33)));
            _uiStates.niceColors.Add(new SolidColorBrush(System.Windows.Media.Color.FromArgb(200, 60, 196, 69)));
            _uiStates.niceColors.Add(new SolidColorBrush(System.Windows.Media.Color.FromArgb(200, 205, 91, 234)));
            _uiStates.niceColors.Add(new SolidColorBrush(System.Windows.Media.Color.FromArgb(200, 253, 231, 76)));
            _uiStates.niceColors.Add(new SolidColorBrush(System.Windows.Media.Color.FromArgb(200, 229, 89, 52)));
            _uiStates.niceColors.Add(new SolidColorBrush(System.Windows.Media.Color.FromArgb(200, 98, 91, 234)));
            _uiStates.niceColors.Add(new SolidColorBrush(System.Windows.Media.Color.FromArgb(200, 155, 197, 61)));


            _uiStates.niceColorsNoBrush.Add((System.Windows.Media.Color.FromArgb(200, 91, 192, 235)));
            _uiStates.niceColorsNoBrush.Add((System.Windows.Media.Color.FromArgb(200, 250, 121, 33)));
            _uiStates.niceColorsNoBrush.Add((System.Windows.Media.Color.FromArgb(200, 60, 196, 69)));
            _uiStates.niceColorsNoBrush.Add((System.Windows.Media.Color.FromArgb(200, 205, 91, 234)));
            _uiStates.niceColorsNoBrush.Add((System.Windows.Media.Color.FromArgb(200, 253, 231, 76)));
            _uiStates.niceColorsNoBrush.Add((System.Windows.Media.Color.FromArgb(200, 229, 89, 52)));
            _uiStates.niceColorsNoBrush.Add((System.Windows.Media.Color.FromArgb(200, 98, 91, 234)));
            _uiStates.niceColorsNoBrush.Add((System.Windows.Media.Color.FromArgb(200, 155, 197, 61)));

            for (int i = 0; i < 10; i++)
            {
                _uiStates.niceColors.Add(new SolidColorBrush(System.Windows.Media.Colors.Black));

            }

        }


        public MainWindow()
        {

            OverlapPreventionHack.countOverlapnessDelegate = CountOverlapness;
            try
            {
                InitializeComponent();
            }
            catch (SystemException e)
            {
            }

            uiInstance = new UIInstance();

            this.FontSize = 14;


            string[] partTypes = { "bodies", "legs", "wheels", "peripherals"};//, "beams"};
            foreach (string partType in partTypes)
            {
                sidebar = new Sidebar(partType);
                sidebar.LoadMeshIcons(partType);
                switch (partType)
                {
                    case "bodies":
                        sidebar.PopulateContainer(SidebarPanelBodies);
                        break;
                    case "legs":
                        sidebar.PopulateContainer(SidebarPanelLegs);
                        break;
                    case "wheels":
                        sidebar.PopulateContainer(SidebarPanelWheels);
                        break;
                    //case "beams":
                    //    sidebar.PopulateContainer(SidebarPanelBeams);
                    //    break;
                    default:
                        sidebar.PopulateContainer(SidebarPanelPeripherals);
                        break;
                }
            }

            NewMeshView.MouseDown += DeselectBox_MouseDown;

            NewMeshView.AllowDrop = true;
            NewMeshView.DragEnter += cppGlSurface_DragEnter;
            NewMeshView.Drop += cppGlSurface_Drop;
            NewMeshView.DragOver += cppGlSurface_DragOver;

            SequenceCanvas.AllowDrop = true;
            SequenceCanvas.DragEnter += sequenceCanvas_DragEnter;
            SequenceCanvas.Drop += sequenceCanvas_Drop;
            SequenceCanvas.DragOver += sequenceCanvas_DragOver;


            NewMeshView.ModelUpDirection = new Vector3D(0, 1, 0);
            NewMeshView.Camera.UpDirection = new Vector3D(0, 1, 0); //y up
            //NewMeshView.Camera.Position = new Point3D(50, 450, 560);
            ((PerspectiveCamera)NewMeshView.Camera).FieldOfView = 75;

            initializeUIStates();
            Debugging = new Debugging(_uiStates);
            _ui3D = new UI3D();
            _ui3D.initialize(NewMeshView, OverlayView, Overlay2D, uiInstance, _uiStates, RefreshUIState, handleUpdatesFromShapeChange);
            NewMeshView.MouseUp += (sender, args) =>
            {
                RefreshViews();
            };
             ghostMesh = null;
            currentSequence = new List<int>();
            draggedLabel = -1;

            //JointInfoBox.ItemsSource = Enum.GetValues(typeof (ConnectionType)).Cast<ConnectionType>();




            dialog = new SaveFileDialog();
            dialog.Filter = "Proto file (.asciiproto)|*.asciiproto";

            command = new Command(uiInstance, _uiStates, this);

            //configurationChooser.Update += (i, theta) => {
            //    if (uiInstance.hasMainWorkignTemp()) {
            //       uiInstance.SetJointChoice(i, theta);
            //        RefreshViews();
            //        UpdateJointSelectionUI();
            //    }
            //};

            symmetryChooser.Update += (symm_ground, symm_legW, symm_legL, symm_spacing) =>
            {
                if (uiInstance.hasMainWorkingTemp())
                {
                    uiInstance.SetSymmetryChoice(symm_ground, symm_legW, symm_legL, symm_spacing);
                    RefreshViews();
                    UpdateSymmetrySelectionUI();
                }
            };

            UpdateSavedViewMenuItems();

            RefreshUIState();

            animateTimer = new Timer();
        }



        // Function as a part of the collision prevention hack: count # of pairwise overlaps.
        public static int CountOverlapness(Drawing drawing) {
            int result = 0;
            List<Polygon> polygons = new List<Polygon>();
            foreach (var face in drawing.faces) {
                Polygon polygon = new Polygon();
                foreach (var point in face.points) {
                    polygon.Points.Add(point);
                }
                polygons.Add(polygon);
            }
            for (int i = 0; i < polygons.Count; i++) {
                Polygon A = polygons[i];
                for (int j = 0; j < polygons.Count; j++) {
                    if (i != j) {
                        Polygon B = polygons[j];
                        if (CollisionDetector.PolygonCollision(A, B, new Vector(0, 0)).Intersect == true) {
                            result++;
                        }
                    }
                }
            }
            return result;
        }

        private void DeselectBox_MouseDown(object sender, MouseButtonEventArgs e) {
            if (e.OriginalSource is CameraController) {
                _ui3D.Deselect();
                RefreshUIState();
            }
        }



 

        private void cppGlSurface_DragEnter(object sender, DragEventArgs e) {
            if (e.Data.GetDataPresent("templateFilename") && !(true.Equals(e.Data.GetData("invalid")))) {
                // check if there were any unconnected templates before
                int Ntemp = uiInstance.numWorkingTemp();
                if (Ntemp > 1)
                {
                    uiInstance.HandleConnect();
                }
                doAnimation = false;

                // add the dragged template
                e.Data.SetData("invalid", true);
                e.Effects = DragDropEffects.Copy;
                //loading the template 
                MyData data = (MyData) e.Data.GetData("templateFilename");
                String templateFilename = data._fileName;
                int templateID = data._id;

                bool wasEmptyBefore = !uiInstance.hasMainWorkingTemp();
                // Console.Write("dropping template with " + templateFilename + " with ID " + templateID);
                templateToAdd = uiInstance.AddProtoTemplate(templateFilename, templateID, e.GetPosition(NewMeshView));
                //uiInstance.Rotate(templateToAdd, new Quaternion(new Vector3D(1, 0, 0), -90), templateToAdd.Center);
                //uiInstance.AddProtoTemplate(templateFilename, templateID, e.GetPosition(NewMeshView));
                //templateToAdd = uiInstance.getMainTemplate()->getTemp()
                Point3D center = templateToAdd.Center;
                
                //initiate the translation to translate this model to where the mouse is
                //Console.WriteLine(e.GetPosition(NewMeshView));
                Point3D? to = NewMeshView.Viewport.UnProject(e.GetPosition(NewMeshView));
                //Console.WriteLine(to);
                //Point3D? from = NewMeshView.Viewport.UnProject(new Point(NewMeshView.ActualWidth / 2, NewMeshView.ActualHeight / 2));
                Point3D? from = center;
                _uiStates.IsInitialDragging = true;
                handleUpdatesFromShapeChange(wasEmptyBefore);
                  

                if (to != null && from != null) {
                    if (ENABLE_INITIAL_DRAGGING)
                        _uiStates.TranslationCache.Translate(templateToAdd, to.Value - from.Value);
                    //uiState.SnappingState = null;
                    // // Console.WriteLine("translating model upon initial loading" + (to.Value - from.Value));
                }
                lastPosition1 = e.GetPosition(NewMeshView);
                //RefreshUIState();
            }
        }

        private void cppGlSurface_Drop(object sender, DragEventArgs e) {
            /*
            MyData data = (MyData)e.Data.GetData("templateFilename");
            String templateFilename = data._fileName;
            int templateID = data._id;
            // Console.Write("dropping template with " + templateFilename + " with ID " + templateID );
            uiInstance.AddProtoTemplate(templateFilename, templateID, e.GetPosition(NewMeshView));    
            
            RefreshUIState();
            */
            _uiStates.IsInitialDragging = false;
            _uiStates.TranslationCache.Commit();
            RefreshUIState();
        }

        private void cppGlSurface_DragOver(object sender, DragEventArgs e) {
            if (lastPosition1 != null) {
                System.Windows.Point curMouse = e.GetPosition(NewMeshView);
                Point3D? to = NewMeshView.Viewport.UnProject(curMouse);
                Point3D? from = NewMeshView.Viewport.UnProject(lastPosition1.Value);
                if (to != null && from != null) {
                    if (ENABLE_INITIAL_DRAGGING)
                        _uiStates.TranslationCache.Translate(templateToAdd, to.Value - from.Value);
                    //uiState.SnappingState = null;
                    //// // Console.WriteLine("translating model upon initial dragging over" + (to.Value - from.Value));                    
                }
                lastPosition1 = curMouse;
                //RefreshUIState();
            }
        }



        private void sequenceCanvas_DragEnter(object sender, DragEventArgs e)
        {
            if (e.Data.GetDataPresent("gaitOption") && !(true.Equals(e.Data.GetData("invalid"))))
            {
                e.Data.SetData("invalid", true);
                e.Effects = DragDropEffects.Copy;
                //loading the template 
                draggedLabel = (int)e.Data.GetData("gaitOption");
                //draggedLabel = 0; 
                Console.Out.WriteLine("dragging model :  " + (draggedLabel));                 
            }
        }

        private void sequenceCanvas_Drop(object sender, DragEventArgs e)
        {
            if (draggedLabel >= 0)
            {
      
                System.Windows.Point curMouse = e.GetPosition(SequenceCanvas);

                double width = (SequenceCanvas.ActualWidth);
                double boxWidth = Math.Min(100, (0.9 * width) / currentSequence.Count);
                double marginWidth = 0.5 * (width - currentSequence.Count * boxWidth);
                int sequencePos = (int)Math.Floor((curMouse.X - marginWidth) / boxWidth);
                currentSequence[sequencePos] = draggedLabel;
                GaitSequence.drawSequence((deleteId) =>
                {
                    currentSequence[deleteId] = -1;
                    updateSquenceGaitTab();
                    uiInstance.updateCurrentSequenceInWt(currentSequence);
                    updateMetrics();   
                }, _uiStates, currentSequence, SequenceCanvas);
            }
            draggedLabel = -1;
            uiInstance.updateCurrentSequenceInWt(currentSequence);
            updateMetrics(); 

        }



        private void sequenceCanvas_DragOver(object sender, DragEventArgs e)
        {
            if (draggedLabel >= 0)
            {
                System.Windows.Point curMouse = e.GetPosition(SequenceCanvas);

               double width = (SequenceCanvas.ActualWidth);
               double boxWidth = Math.Min(100, (0.9 * width) / currentSequence.Count);
               double marginWidth = 0.5 * (width - currentSequence.Count * boxWidth);
               int sequencePos = (int)Math.Floor((curMouse.X - marginWidth) / boxWidth);

               List<int> newSequence = new List<int>();
               for (int i = 0; i < currentSequence.Count; i++)
               {
                   newSequence.Add(currentSequence[i]);
               }
                newSequence[sequencePos] = draggedLabel;
                GaitSequence.drawSequence((deleteId) =>
                {
                    currentSequence[deleteId] = -1;
                    updateSquenceGaitTab();
                    uiInstance.updateCurrentSequenceInWt(currentSequence);
                    updateMetrics();   
                }, _uiStates, newSequence, SequenceCanvas);
                //Console.Out.WriteLine("sender.GetType().Name  " + (sender.GetType().Name));
                Console.Out.WriteLine("Postion  " + (curMouse.X) + ", " + curMouse.Y);
                Console.Out.WriteLine("sequencePos  " + sequencePos);                 


            }
        }


        private void RefreshUIState() {
            Stopwatch sw2 = new Stopwatch();
            sw2.Start();
            CommandManager.InvalidateRequerySuggested(); // force CanExecute re-calls
            sw2.Stop();

            Stopwatch sw3 = new Stopwatch();
            sw3.Start();
            RefreshViews();
            sw3.Stop();

            Stopwatch sw4 = new Stopwatch();
            sw4.Start();
            RefreshParameters();
            sw4.Stop();
            // Console.Write("invalidateRequerySuggested cost " + sw2.Elapsed + "\n");
            // Console.Write("refreshViews cost " + sw3.Elapsed + "\n");
            // Console.Write("refhreshParameters cost " + sw4.Elapsed + "\n");
        }

        private Dictionary<int, DoubleUpDown> paramUpDowns = new Dictionary<int, DoubleUpDown>();
        private bool suppressParamValueEvent = false;
        // Refresh list of parameters in parameters canvas
        private void RefreshParameters() {
            if (true) return; // disabling for now; causes slowness.
            stackPanelParameters.Children.Clear();
            paramUpDowns.Clear();
            int nParameters = uiInstance.getTemplate0Parameters().Count;
            for (int i = 0; i < nParameters; i++) {
                DoubleUpDown dud = new DoubleUpDown {Text = i.ToString()};
                int iCopy = i;
                dud.ValueChanged += (sender, args) => {
                    if (suppressParamValueEvent) {
                        return;
                    }
                    if (args.OldValue == null) {
                        return;
                    }
                    double deltaChange = (double) args.NewValue - (double) args.OldValue;

                    // // Console.WriteLine(iCopy.ToString() + " " + dud.Value.ToString());
                    uiInstance.updateTemp0ForIndividualParam(Convert.ToInt32(iCopy), deltaChange);
                    RefreshUIState();
                    UpdateParameters();
                };
                stackPanelParameters.Children.Add(dud);
                paramUpDowns[i] = dud;
            }
            UpdateParameters();
        }

        private void UpdateParameters() {
            List<double> qs = uiInstance.getTemplate0Parameters();
            for (int i = 0; i < qs.Count; i++) {
                var dud = paramUpDowns[i];

                // Suppress the value changed event so we don't trigger an infinite loop
                suppressParamValueEvent = true;
                dud.Value = qs[i];
                suppressParamValueEvent = false;
            }
        }


        // Refresh the canvas showing unfolded 2d faces and 3d meshes
        private void RefreshViews() {
            Stopwatch sw = new Stopwatch();
            Stopwatch sw1 = new Stopwatch();
            Stopwatch sw2 = new Stopwatch();
            Stopwatch sw3 = new Stopwatch();
            Stopwatch sw4 = new Stopwatch();


            sw.Start();
            Dictionary<TemplateRef, Drawing> drawings = uiInstance.getDrawings();
            sw1.Start();
            List<Mesh> meshes = uiInstance.getMeshes();
            sw1.Stop();
            // // Console.WriteLine("time to get meshes is " + sw1.Elapsed);
            sw2.Start();
            var selection = _uiStates.Selection;
            if (selection.Type != Selection.SelectionType.None) {
                var selectedTmpl = selection.SelectedTemplate;
                var selectedRootTmpl = selectedTmpl.Root;
                var drawingsToShow =
                    drawings.Where(kvp => kvp.Key.Root.Equals(selectedRootTmpl))
                        .ToDictionary(kvp => kvp.Key, kvp => kvp.Value);
                DrawingDraw.Draw(_uiStates, drawingsToShow, DrawingCanvas2, selectedTmpl, template =>
                {
                    _uiStates.Selection = Selection.Face(template);
                    RefreshUIState();
                });
            }

            sw2.Stop();
            // // Console.WriteLine("time for drawingDraw.Draw is " + sw2.Elapsed);
            sw3.Start();
            _ui3D.Draw(meshes, ghostMesh);
            sw3.Stop();
            // // Console.WriteLine("time for _ui3D.Draw is " + sw3.Elapsed);
            
            //this.DebugView.Items.Clear();
            Dictionary<string, DebuggableRef> debuggables = new Dictionary<string, DebuggableRef>();
            for (int i = 0; i < uiInstance.getTemplates().Count; i++) {
                var tmpl = uiInstance.getTemplates()[i];
                debuggables["Template " + i] = tmpl.Debuggable;
            }
            Debugging.render(debuggables, DebugView);
            sw.Stop();
            // // Console.WriteLine("time to refreshViews is " + sw.Elapsed);
        }

        private void CanAlwaysExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = true;
        }

        private void CanSnapExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = uiInstance != null && uiInstance.getTemplates().Count >= 2;
        }

        private void HandleNewCommand(object sender, ExecutedRoutedEventArgs e) {
            uiInstance.HandleClear(); //update meshviews
            DrawingCanvas.Children.Clear(); //clear 2d views
            DrawingCanvas2.Children.Clear(); //clear 2d views
            //InputMotionSequenceSize.Value = 0; 
            _uiStates.Selection = Selection.Empty;
            _uiStates.SnappingState = null;
            RefreshUIState();
            clearScene();
        }

        private void CanSaveExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = uiInstance.getTemplates().Count > 0;
        }

        private void HandleSaveCommand(object sender, ExecutedRoutedEventArgs e) {
            if (dialog.ShowDialog(this) == true) {
                uiInstance.Save(dialog.FileName);
            }
        }

        private void CanSelectParentExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = _uiStates != null && (_uiStates.Selection.Type == Selection.SelectionType.Template ||
                                                 _uiStates.Selection.Type == Selection.SelectionType.Face)
                                                 && !_uiStates.Selection.SelectedTemplate.Parent.IsNull;
        }

        private void HandleSelectParentCommand(object sender, ExecutedRoutedEventArgs e) {
            _uiStates.Selection = Selection.Template(_uiStates.Selection.SelectedTemplate.Parent);
            RefreshUIState();
        }

        private void CanSelectChildExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = false;
        }

        private void HandleSelectChildCommand(object sender, ExecutedRoutedEventArgs e) {
            // TODO: implement
            RefreshUIState();
        }

        private void CanDeleteSelectedExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = _uiStates != null && !_uiStates.Selection.SelectedTemplate.IsNull;
        }

        private void HandleDeleteSelectedCommand(object sender, ExecutedRoutedEventArgs e) {
            if (!_uiStates.Selection.SelectedTemplate.IsNull) {
                var tmpl = _uiStates.Selection.SelectedTemplate;
                if (tmpl.Root.Equals(tmpl)) {
                    uiInstance.removeTemplate(uiInstance.getTemplates().IndexOf(tmpl));
                }
                else {
                    tmpl.Remove();
                    uiInstance.handleDelete();
                }
                _uiStates.Selection = Selection.Empty;
                handleUpdatesFromShapeChange(true);
            }
        }

        private void CanSimulateExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = false;
        }

        private void HandleSimulateCommand(object sender, ExecutedRoutedEventArgs e) {
            // TODO: implement
            RefreshUIState();
        }


        private void CanAvoidObstacle(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = true;
        }

        private void HandleAvoidObstacle(object sender, ExecutedRoutedEventArgs e)
        {
            _uiStates.showGhost = false;
            RevertToOriginal.Visibility = System.Windows.Visibility.Collapsed;
            ShowGhost.Visibility = System.Windows.Visibility.Collapsed;
            ShowGhost.IsChecked = false;
            _uiStates.IsGuidingManipulation = false;
            GuideManipuations.IsChecked = false;

            _uiStates.saveFileName = "..\\..\\data\\userStudy\\userid_" +
    Convert.ToString(_uiStates.UserStudy_ID) + "\\task3.asciiproto";
            _uiStates.taskId = 3;
            _uiStates.taskLogFilename = "..\\..\\data\\userStudy\\userid_" +
            Convert.ToString(_uiStates.UserStudy_ID) + "\\task3_log.txt";

            uiInstance.HandleClear(); //update meshviews
            DrawingCanvas.Children.Clear(); //clear 2d views
            DrawingCanvas2.Children.Clear(); //clear 2d views
            _uiStates.Selection = Selection.Empty;
            _uiStates.SnappingState = null;

            // TODO: implement            
            ForwardModeB.IsChecked = false;
            ForwardModeA.IsChecked = false;
            DesignCar.IsChecked = false;
            AvoidObstable.IsChecked = true;

            _uiStates.DesignCar = false;
            _uiStates.AvoidObstacle = true;
            _uiStates.ForwardTravel = false;
            _uiStates.DisableAllStabilityGuidance = false;

            beamsTab.IsSelected = true;
            beamsTab.Visibility = System.Windows.Visibility.Collapsed;
            wheelsTab.Visibility = System.Windows.Visibility.Visible;
            legsTab.Visibility = System.Windows.Visibility.Visible;
            bodiesTab.Visibility = System.Windows.Visibility.Visible;
            peripheralsTab.Visibility = System.Windows.Visibility.Visible;
            openFileFromName("..\\..\\data\\proto2016\\examples\\user_study3.asciiproto");
            ghostMesh = null; 

            RefreshViews();
        }


        private void HandleDesignCar(object sender, ExecutedRoutedEventArgs e)
        {
            _uiStates.showGhost = false;
            RevertToOriginal.Visibility = System.Windows.Visibility.Collapsed;
            ShowGhost.Visibility = System.Windows.Visibility.Collapsed;
            ShowGhost.IsChecked = false;
            _uiStates.IsGuidingManipulation = false;
            GuideManipuations.IsChecked = false;

            _uiStates.saveFileName = "..\\..\\data\\userStudy\\userid_" +
    Convert.ToString(_uiStates.UserStudy_ID) + "\\task2.asciiproto";
            _uiStates.taskId = 2;
            _uiStates.taskLogFilename = "..\\..\\data\\userStudy\\userid_" +
            Convert.ToString(_uiStates.UserStudy_ID) + "\\task2_log.txt";
            uiInstance.HandleClear(); //update meshviews
            DrawingCanvas.Children.Clear(); //clear 2d views
            DrawingCanvas2.Children.Clear(); //clear 2d views
            _uiStates.Selection = Selection.Empty;
            _uiStates.SnappingState = null;

            // TODO: implement
            ForwardModeB.IsChecked = false;
            ForwardModeA.IsChecked = false;
            DesignCar.IsChecked = true;
            AvoidObstable.IsChecked = false;
            _uiStates.DesignCar = true;
            _uiStates.AvoidObstacle = false;
            _uiStates.ForwardTravel = false;
            _uiStates.DisableAllStabilityGuidance = false;

            bodiesTab.IsSelected = true;
            beamsTab.Visibility = System.Windows.Visibility.Collapsed;
            wheelsTab.Visibility = System.Windows.Visibility.Visible;
            legsTab.Visibility = System.Windows.Visibility.Collapsed;
            bodiesTab.Visibility = System.Windows.Visibility.Visible;
            peripheralsTab.Visibility = System.Windows.Visibility.Visible;
            ghostMesh = null; 

            RefreshViews();
        }

        private void HandleNoTask(object sender, ExecutedRoutedEventArgs e)
        {
            _uiStates.showGhost = false;
            RevertToOriginal.Visibility = System.Windows.Visibility.Collapsed;
            ShowGhost.Visibility = System.Windows.Visibility.Collapsed;
            ShowGhost.IsChecked = false;
            _uiStates.IsGuidingManipulation = false;
            GuideManipuations.IsChecked = false;

            _uiStates.showGhost = false;           
            _uiStates.saveFileName = "..\\..\\data\\userStudy\\test.asciiproto";
            uiInstance.HandleClear(); //update meshviews
            DrawingCanvas.Children.Clear(); //clear 2d views
            DrawingCanvas2.Children.Clear(); //clear 2d views
            _uiStates.Selection = Selection.Empty;
            _uiStates.SnappingState = null;
            // TODO: implement
            ForwardModeB.IsChecked = false;
            ForwardModeA.IsChecked = false;
            DesignCar.IsChecked = false;
            AvoidObstable.IsChecked = false;
            _uiStates.DesignCar = false;
            _uiStates.AvoidObstacle = false;
            _uiStates.ForwardTravel = false;
            _uiStates.DisableAllStabilityGuidance = false;

            bodiesTab.IsSelected = true;
            beamsTab.Visibility = System.Windows.Visibility.Collapsed;
            wheelsTab.Visibility = System.Windows.Visibility.Visible;
            legsTab.Visibility = System.Windows.Visibility.Visible;
            bodiesTab.Visibility = System.Windows.Visibility.Visible;
            peripheralsTab.Visibility = System.Windows.Visibility.Visible;

            RefreshViews();
            ghostMesh = null; 
        }

        private void openFileFromName(string filename)
        {
            double x = NewMeshView.ActualWidth / 2;
            double y = NewMeshView.ActualHeight / 2;
            System.Windows.Point point = new System.Windows.Point(x, y);
            bool wasEmptyBefore = !uiInstance.hasMainWorkingTemp();

            TemplateRef tmpl = uiInstance.AddProtoTemplate(filename, 0, point);
            //uiInstance.Rotate(tmpl, new Quaternion(new Vector3D(1, 0, 0), -90), tmpl.Center);
            uiInstance.Translate(tmpl, new Vector3D(0, 0, 0));
            handleUpdatesFromShapeChange(wasEmptyBefore);

        }


        private void HandleStartTimer(object sender, RoutedEventArgs e)
        {
            if (_uiStates != null)
            {
               taskLogs = new TaskLogs(_uiStates.taskId, _uiStates.taskLogFilename, uiInstance);
                taskLogs.start();
            }

        }

        private void HandleEndTimer(object sender, RoutedEventArgs e)
        {
            if (taskLogs != null)
            {
                taskLogs.end();
            }
            uiInstance.Save(_uiStates.saveFileName);
        }


        private void HandlForwardTravelA(object sender, ExecutedRoutedEventArgs e)
        {
            _uiStates.showGhost = false;
            RevertToOriginal.Visibility = System.Windows.Visibility.Visible; 
            ShowGhost.Visibility = System.Windows.Visibility.Visible;
            ShowGhost.IsChecked = false; 


            _uiStates.saveFileName = "..\\..\\data\\userStudy\\userid_" +
            Convert.ToString(_uiStates.UserStudy_ID) + "\\task1_modeA.asciiproto";
            _uiStates.taskId = 1;
            _uiStates.taskLogFilename = "..\\..\\data\\userStudy\\userid_" +
            Convert.ToString(_uiStates.UserStudy_ID) + "\\task1_modeA_log.txt";
            

            uiInstance.HandleClear(); //update meshviews
            DrawingCanvas.Children.Clear(); //clear 2d views
            DrawingCanvas2.Children.Clear(); //clear 2d views
            _uiStates.Selection = Selection.Empty;
            _uiStates.SnappingState = null;

            // TODO: implement
            ForwardModeB.IsChecked = false;
            ForwardModeA.IsChecked = true;
            DesignCar.IsChecked = false;
            AvoidObstable.IsChecked = false;
            _uiStates.DesignCar = false;
            _uiStates.AvoidObstacle = false;
            _uiStates.ForwardTravel = true;

            int id = _uiStates.UserStudy_ID;
            int showGuides = id % 2;
            int modelParam = ((id - showGuides) / 2) % 2;
            _uiStates.DisableAllStabilityGuidance = false;
            if (showGuides != 0)
            {
                _uiStates.DisableAllStabilityGuidance = true;
                _uiStates.IsGuidingManipulation = false;
                GuideManipuations.IsChecked = false;
            }
            else
            {
                
                _uiStates.IsGuidingManipulation = true;
                GuideManipuations.IsChecked = true;
            }
            if (modelParam == 0)
            {
                 requiredSpeedTask = 150;
                 openFileFromName("..\\..\\data\\proto2016\\examples\\user_study1_modelA_arrow.asciiproto");
            }
            else
            {
                requiredSpeedTask = 190;
                openFileFromName("..\\..\\data\\proto2016\\examples\\user_study1_modelB_arrow.asciiproto");
            }
            bodiesTab.IsSelected = true;
            beamsTab.Visibility = System.Windows.Visibility.Collapsed;
            wheelsTab.Visibility = System.Windows.Visibility.Visible;
            legsTab.Visibility = System.Windows.Visibility.Visible;
            bodiesTab.Visibility = System.Windows.Visibility.Visible;
            peripheralsTab.Visibility = System.Windows.Visibility.Visible;
            ghostMesh = null;
            ghostMeshSave = uiInstance.getMeshes();
            RefreshViews();
        }

        private void HandleDirectSave(object sender, ExecutedRoutedEventArgs e)
        {

            uiInstance.Save(_uiStates.saveFileName);

        }

        private void HandleForwardTravelB(object sender, ExecutedRoutedEventArgs e)
        {

            _uiStates.showGhost = false;
            RevertToOriginal.Visibility = System.Windows.Visibility.Visible;
            ShowGhost.Visibility = System.Windows.Visibility.Visible;
            ShowGhost.IsChecked = false; 

            
            _uiStates.saveFileName = "..\\..\\data\\userStudy\\userid_" +
                Convert.ToString(_uiStates.UserStudy_ID) + "\\task1_modeB.asciiproto";
            _uiStates.taskId = 1;
            _uiStates.taskLogFilename = "..\\..\\data\\userStudy\\userid_" +
            Convert.ToString(_uiStates.UserStudy_ID) + "\\task1_modeB_log.txt";

            uiInstance.HandleClear(); //update meshviews
            DrawingCanvas.Children.Clear(); //clear 2d views
            DrawingCanvas2.Children.Clear(); //clear 2d views
            _uiStates.Selection = Selection.Empty;
            _uiStates.SnappingState = null;

            // TODO: implement
            ForwardModeB.IsChecked = true;
            ForwardModeA.IsChecked = false;
            DesignCar.IsChecked = false;
            AvoidObstable.IsChecked = false;
            _uiStates.DesignCar = false;
            _uiStates.AvoidObstacle = false;
            _uiStates.ForwardTravel = true;
            int id = _uiStates.UserStudy_ID; 
            int showGuides = id % 2;
            int modelParam = ((id - showGuides) / 2) % 2;
            showGuides = 1 - showGuides;
            modelParam = 1 - modelParam;
            _uiStates.DisableAllStabilityGuidance = false;
            if (showGuides != 0)
            {
                _uiStates.DisableAllStabilityGuidance = true;
                _uiStates.IsGuidingManipulation = false;
                GuideManipuations.IsChecked = false;
            }
            else
            {
                _uiStates.IsGuidingManipulation = true;
                GuideManipuations.IsChecked = true;
            }
            if(modelParam == 0){
                requiredSpeedTask = 150;
                openFileFromName("..\\..\\data\\proto2016\\examples\\user_study1_modelA_arrow.asciiproto");
            }
            else
            {
                requiredSpeedTask = 190;
                openFileFromName("..\\..\\data\\proto2016\\examples\\user_study1_modelB_arrow.asciiproto");
            }
            bodiesTab.IsSelected = true;
            beamsTab.Visibility = System.Windows.Visibility.Collapsed;
            wheelsTab.Visibility = System.Windows.Visibility.Visible;
            legsTab.Visibility = System.Windows.Visibility.Visible;
            bodiesTab.Visibility = System.Windows.Visibility.Visible;
            peripheralsTab.Visibility = System.Windows.Visibility.Visible;
            ghostMesh = null;
            ghostMeshSave = uiInstance.getMeshes();
            RefreshViews();
        }

        private void HandleRevertToOriginal(object sender, ExecutedRoutedEventArgs e)
        {
            uiInstance.HandleClear(); //update meshviews
            DrawingCanvas.Children.Clear(); //clear 2d views
            DrawingCanvas2.Children.Clear(); //clear 2d views
            _uiStates.Selection = Selection.Empty;
            _uiStates.SnappingState = null;
            
            int id = _uiStates.UserStudy_ID;
            int showGuides = id % 2;
            int modelParam = ((id - showGuides) / 2) % 2;
            if (ForwardModeB.IsChecked)
            {
                showGuides = 1 - showGuides;
                modelParam = 1 - modelParam;
            }
            _uiStates.DisableAllStabilityGuidance = false;
            if (showGuides != 0)
            {
                _uiStates.DisableAllStabilityGuidance = true;
            }
            if (modelParam == 0)
            {
                openFileFromName("..\\..\\data\\proto2016\\examples\\user_study1_modelA.asciiproto");
            }
            else
            {
                openFileFromName("..\\..\\data\\proto2016\\examples\\user_study1_modelB.asciiproto");
            }
            RefreshViews();

        }

        private void ShowGhost_onClick(object sender, RoutedEventArgs e)
        {
            _uiStates.showGhost = !_uiStates.showGhost;
            ShowGhost.IsChecked = _uiStates.showGhost;
            if (_uiStates.showGhost)
            {
                ghostMesh = ghostMeshSave;
            }
            else
            {
                ghostMesh = null;
            }
            RefreshViews();
        }
        
        private void HandleSnapToGroundCommand(object sender, ExecutedRoutedEventArgs e)
        {
            _uiStates.AlwaysSnapToGround = !_uiStates.AlwaysSnapToGround;
            if (_uiStates.AlwaysSnapToGround)
            {
                uiInstance.HandleSnapToGround();
            }
            RefreshViews();
        }

        private void CanSnapToGroundExecute(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = uiInstance != null && uiInstance.getTemplates().Count >= 1;
        }

        private void CanAnimateExecute(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = true;
        }
        private void CanAnimateSlowExecute(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = true;
        }
        private void CanViewGaitExecute(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = true;
        }

        private void CanStablizeExecute(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = uiInstance != null && uiInstance.getTemplates().Count >= 1;
        }
        private void HandleStablizeCommand(object sender, ExecutedRoutedEventArgs e)
        {
            uiInstance.optimizeNone();
            RefreshViews();
        }


        private void CanGuideManipulationExecute(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = true;
        }
        private void HandleGuideManipulationCommand(object sender, ExecutedRoutedEventArgs e)
        {
            _uiStates.IsGuidingManipulation = !_uiStates.IsGuidingManipulation;
            GuideManipuations.IsChecked = _uiStates.IsGuidingManipulation;
        }

        private void CanDisableAllStabilityGuidance(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = true;
        }
        private void HandleDisableAllStabilityGuidance(object sender, ExecutedRoutedEventArgs e)
        {
            _uiStates.DisableAllStabilityGuidance = !_uiStates.DisableAllStabilityGuidance;
            if (_uiStates.DisableAllStabilityGuidance)
            {
                _uiStates.IsGuidingManipulation = false;
            }

        }

        void drawAnimation(List<Mesh> listOfGeo, List<bool> listOfStabilityCost, int slowdow)
        {
            if (doAnimation)
            {
                animateTimer.Stop();
                animateTimer.Enabled = false;
                doAnimation = false;
            }

            StopAnimate.IsEnabled = true;

            animateTimer = new Timer();
            DateTime startTime = DateTime.Now;
            TimeSpan duration = new TimeSpan(0, 0, 2);
            int counter = 0;
            doAnimation = true;
            animateTimer.Tick += new EventHandler((object sender1, EventArgs e1) =>
            {
                if (counter < listOfGeo.Count && doAnimation)
                {
                    List<Mesh> meshes = new List<Mesh>();
                    List<bool> colors = new List<bool>();
                    meshes.Add(listOfGeo[counter]);
                    colors.Add(listOfStabilityCost[counter]);
                    _ui3D.Draw(meshes, ghostMesh, colors, true);
                    //_ui3D.Draw(meshes);
                   // int round = (int)Math.Floor(_uiStates.DeltaAnim * counter);
                    
                   // if (round >= 0 && round < intervals.Count)
                    //{
                   //     int interval_length = intervals[round];
                   //     if (interval_length < 1)
                   //     {
                   //         interval_length = 1;
                   //     }
                   //     timer.Interval = interval_length;
                   // }
                    counter++;
                }
                else
                {
                    animateTimer.Stop();
                    doAnimation = false;
                    animateTimer.Enabled = false;
                    StopAnimate.IsEnabled = false;
                }
            });

            animateTimer.Interval = 12*slowdow;
            animateTimer.Enabled = true;


        }


        void handleAnimate(int gaitID, int slowMotio=1, bool changePose=true)
        {
            uiInstance.Save(_uiStates.saveFileName);

            if (doAnimation)
            {
                doAnimation = false;
                animateTimer.Stop();
                animateTimer.Enabled = false;
            }
            
            bool useSteadyState = _uiStates.useSteadyState;
            bool doSequence = false;
            if (gaitID >= 0)
            {
                uiInstance.updateControllers(gaitID);
                uiInstance.updateMetrics(_uiStates.NAnimRounds, _uiStates.DeltaAnim, false);
            }
            else
            {
                uiInstance.updateMetrics(_uiStates.NAnimRounds, _uiStates.DeltaAnim, true);
                doSequence = true;
                useSteadyState = false;
            }
            _ui3D.Deselect();
            List<Mesh> listOfGeo;
            if (changePose)
            {
                listOfGeo = uiInstance.getAllAnimationMeshes(useSteadyState, doSequence);
            }
            else
            {
                listOfGeo = uiInstance.getAllViewGaitMeshes();
            }
            //List<int> intervals = uiInstance.getListOfTimeIntervals(doSequence);
            List<bool> listOfStabilityCost = new List<bool>();
            for (int i = 0; i < listOfGeo.Count; i++)
            {
                listOfStabilityCost.Add(true);
            }
            if (_uiStates.AvoidObstacle)
            {
                listOfStabilityCost.Clear();
                listOfStabilityCost = uiInstance.getAllAnimationObstacleColision(doSequence);
            }
            //int NFramesPerRound = (int) Math.Round( 1.0/_uiStates.DeltaAnim);
            //for (int i = 0; i < _uiStates.NAnimRounds; i++)
            //{
           //     drawAnimation(listOfGeo.GetRange(i * NFramesPerRound, NFramesPerRound), listOfStabilityCost.GetRange(i * NFramesPerRound, NFramesPerRound), 10);
            //}

            //Console.WriteLine("-------------------------------------------------------------------- ");
            //Console.WriteLine("-------------------------------------------------------------------- ");
            //for (int i = 0; i < intervals.Count; i++)
           // {
           //     Console.WriteLine("Interval " + i + " = " + intervals[i]);
            //}

            drawAnimation(listOfGeo, listOfStabilityCost, slowMotio);
         
        }

        private void StopAnimate_clicked(object sender, RoutedEventArgs e)
        {
            doAnimation = false;
        }


        private void AnimateGait_clicked(object sender, RoutedEventArgs e)
        {
            handleAnimate(GaitList_ComboBox.SelectedIndex);
            
        }

        private void PreviewGait_clicked(object sender, RoutedEventArgs e)
        {
            handleAnimate(GaitList_ComboBox.SelectedIndex,1,false);

        }

        private void AnimateSequence_clicked(object sender, RoutedEventArgs e)
        {
            handleAnimate(-1);

        }

        private void HandleAnimateCommand(object sender, ExecutedRoutedEventArgs e) {
            handleAnimate(-1);
        }


        private void HandleViewAnimPosesCommand(object sender, ExecutedRoutedEventArgs e)
        {
            _ui3D.Deselect();


            List<Mesh> listOfGeo = uiInstance.getAllAnimationMeshes(_uiStates.useSteadyState, true);
            List<bool> obstacleAvoidance = uiInstance.getAllAnimationObstacleColision(true);
            List<Mesh> meshes = new List<Mesh>();
            List<bool> colors = new List<bool>();
            int step = 10;
            for (int i = 0; i < listOfGeo.Count; i = i + step)
            {
                if (_uiStates.AvoidObstacle)
                {
                    colors.Add(obstacleAvoidance[i]);
                }
                else
                {
                    colors.Add(true);
                }
                meshes.Add(listOfGeo[i]);
            }
            _ui3D.Draw(meshes, ghostMesh, colors, true);

        }

        private void HandleAnimateSlowCommand(object sender, ExecutedRoutedEventArgs e)
        {
            /*
            _ui3D.Deselect();           
            Timer timer = new Timer();
            DateTime startTime = DateTime.Now;
            TimeSpan duration = new TimeSpan(0, 0, 2);
            List<Mesh> listOfGeo = uiInstance.getAllAnimationMeshes(_uiStates.useSteadyState, true);
            List<bool> listOfStabilityCost = new List<bool>();
            for (int i = 0; i < listOfGeo.Count; i++)
            {
                listOfStabilityCost.Add(true);
            }
            if (_uiStates.AvoidObstacle)
            {
                listOfStabilityCost.Clear();
                listOfStabilityCost = uiInstance.getAllAnimationObstacleColision(true);
            }


            int counter = 0;
            timer.Tick += new EventHandler((object sender1, EventArgs e1) =>
            {
                if (counter < listOfGeo.Count)
                {
                    List<Mesh> meshes = new List<Mesh>();
                    List<bool> colors = new List<bool>();
                    meshes.Add(listOfGeo[counter]);
                    colors.Add(listOfStabilityCost[counter]);
                    _ui3D.Draw(meshes, ghostMesh, colors, true);
                    //_ui3D.Draw(meshes);
                    counter++;
                }
                else
                {
                    timer.Stop();
                }
            });

            timer.Interval = 100;
            timer.Enabled = true;
             * */


            handleAnimate(-1, 10); 
        }

        private void HandleViewGaitCommand(object sender, ExecutedRoutedEventArgs e)
        {
            Timer timer = new Timer();
            DateTime startTime = DateTime.Now;
            TimeSpan duration = new TimeSpan(0, 0, 2);
            List<Mesh> listOfGeo = uiInstance.getAllViewGaitMeshes();
            /*
            timer.Tick += new EventHandler((object sender1, EventArgs e1) =>
            {
                double i = (DateTime.Now - startTime).TotalMilliseconds/duration.TotalMilliseconds;
                if (i >= 1)
                {
                    i = 1;
                    List<Mesh> meshes = uiInstance.getAnimationMeshes(i);                    
                    _ui3D.Draw(meshes, NewMeshView);
                    timer.Stop();
                }
                else
                {
                    List<Mesh> meshes = uiInstance.getAnimationMeshes(i);
                    _ui3D.Draw(meshes, NewMeshView);
                }
            });*/

            int counter = 0;
            timer.Tick += new EventHandler((object sender1, EventArgs e1) =>
            {
                if (counter < listOfGeo.Count)
                {
                    List<Mesh> meshes = new List<Mesh>();
                    List<bool> colors = new List<bool>();
                    meshes.Add(listOfGeo[counter]);
                    colors.Add(true);
                    _ui3D.Draw(meshes, ghostMesh, colors, true);
                    //_ui3D.Draw(meshes);
                    counter++;
                }
                else
                {
                    timer.Stop();
                }
            });

            timer.Interval = 20;
            timer.Enabled = true;
        }

        private void CanDeleteTemplateExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = _uiStates != null && !_uiStates.Selection.SelectedTemplate.IsNull;
        }

        private void HandleDeleteTemplateCommand(object sender, ExecutedRoutedEventArgs e) {
            var rootTemplate = _uiStates.Selection.SelectedTemplate.Root;
            uiInstance.removeTemplate(uiInstance.getTemplates().IndexOf(rootTemplate));
            _uiStates.Selection = Selection.Empty;
            RefreshUIState();
        }

        private void CanAddConnectorExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = false;
        }

        private void HandleAddConnectorCommand(object sender, ExecutedRoutedEventArgs e) {
          /*  uiInstance.HandleConnect();
            _uiStates.SnappingState = null;


            if (_uiStates.AlwaysSnapToGround)
            {
                uiInstance.HandleSnapToGround();
            }
            uiInstance.updateControllers();
            updateMetrics();
            RefreshViews();
            */
        }

        private void HandleShowConnectorsCommand(object sender, ExecutedRoutedEventArgs e) {}

        private void HandleUsePerspectiveCameraCommand(object sender, ExecutedRoutedEventArgs e) {
            // TODO: implement
        }

        private void CanSaveGeometryExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = uiInstance != null && uiInstance.getTemplates().Count >= 1;
        }

        private void HandleSaveGeometryCommand(object sender, ExecutedRoutedEventArgs e) {

            uiInstance.SaveGeometry("sdflsjf");
            SaveFileDialog dialog = new SaveFileDialog();
            dialog.InitialDirectory = new FileInfo("..\\..\\data").FullName;
            dialog.FileName = "model.stl";
            dialog.Filter = "STL file (*.stl)|*.stl";

            if (dialog.ShowDialog() == true)
            {
                _uiStates.ExportMesh = true;
                _ui3D.Draw(uiInstance.getMeshes(), ghostMesh);

                var prefix = dialog.FileName.Substring(0, dialog.FileName.Length - ".stl".Length);
                int i = 1;
                //foreach (var pair in _uiStates.ExportedMesh)
               // {
                    var fileName = prefix + ".stl";
                    //uiInstance.exportSTL(pair.Value, fileName);
                    uiInstance.exportWorkingTempMesh(fileName);
                  //  i++;
                //}
            }
        }


        private void HandleEditAnimParams(object sender1, ExecutedRoutedEventArgs e1)
        {

            //int promptValue = Prompt.ShowDialog();

            Form prompt = new Form();
            prompt.Width = 300;
            prompt.Height = 200;
            prompt.Text = "Animation Parameter";
            
            System.Windows.Forms.Label textLabel = new System.Windows.Forms.Label() { Left = 0, Top = 0, Text = "Number of Cycles:" };
            NumericUpDown inputBox = new NumericUpDown() { Left = 100, Top = 0, Width = 40 };
            inputBox.Value = _uiStates.NAnimRounds;
            
            System.Windows.Forms.Label textLabel2 = new System.Windows.Forms.Label() { Left = 0, Top = 30, Text = "Step size:" };
            NumericUpDown inputBox2 = new NumericUpDown() { Left = 100, Top = 30, Width = 60 };
            inputBox2.DecimalPlaces = 3;
            inputBox2.Value = new decimal(_uiStates.DeltaAnim);

            System.Windows.Forms.CheckBox checkbox = new System.Windows.Forms.CheckBox() { Text = "Use steady state", Left = 0, Top = 60 };
            checkbox.Checked = _uiStates.useSteadyState;

            System.Windows.Forms.Button confirmation = new System.Windows.Forms.Button() { Text = "Ok", Left = 100, Width = 50, Top = 90 };
            confirmation.Click += (sender, e) => { prompt.Close(); };
            
            prompt.Controls.Add(confirmation);
            prompt.Controls.Add(textLabel);
            prompt.Controls.Add(inputBox);
            prompt.Controls.Add(textLabel2);
            prompt.Controls.Add(inputBox2);
            prompt.Controls.Add(checkbox); 
            prompt.ShowDialog();
           // return (int)inputBox.Value;

            _uiStates.DeltaAnim = (double)inputBox2.Value;
            _uiStates.NAnimRounds = (int)inputBox.Value;
            _uiStates.useSteadyState = checkbox.Checked;
            updateMetrics();

        }
        private void CanConnectExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = uiInstance != null && uiInstance.getTemplates().Count >= 2;
        }

        private void HandleConnectCommand(object sender, ExecutedRoutedEventArgs e) {
            //uiInstance.Save(_uiStates.saveFileName);

            uiInstance.HandleConnect();
            if (_uiStates.AlwaysSnapToGround)
            {
                uiInstance.HandleSnapToGround();
            }

            if (uiInstance.hasMainWorkingTemp())
            {
                _ui3D.Deselect();
                //UpdateSymmetrySelectionUI(selection.SelectedTemplate);
                handleUpdatesFromShapeChange(true);
            }
           
             _uiStates.SnappingState = null;
            //RefreshUIState();
             uiInstance.Save(_uiStates.saveFileName);
        }

        private void HandleSnapCommand(object sender, ExecutedRoutedEventArgs e) {
            if (_uiStates.SnappingState == null) {
                _uiStates.SnappingState = new SnappingState {
                    PatchPair = null,
                    Threshold = SnappingThreshold.InitialThreshold
                };
            }
            else {
                _uiStates.SnappingState.Threshold = _uiStates.SnappingState.Threshold.Decrease();
            }
//            _uiStates.SnappingState.PatchPair = uiInstance.HandleSnap(_uiStates.SnappingState.PatchPair, _uiStates.SnappingState.Threshold.MaxDistance);
            _uiStates.SnappingState.PatchPair = uiInstance.HandleSnap(null,
                _uiStates.SnappingState.Threshold.MaxDistance);

            RefreshUIState();
        }

        private void CanChangeMaterialExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = false;
        }

        private void HandleChangeMaterialCommand(object sender, ExecutedRoutedEventArgs e) {
            // TODO: implement or remove
        }

        private void HandleExitCommand(object sender, ExecutedRoutedEventArgs e) {
            Close();
        }

        private void MainWindow_OnKeyDown(object sender, KeyEventArgs e) {
            if (e.Key == Key.LeftShift) {
                IsShiftDown = true;
            }
            if (e.Key == Key.LeftCtrl) {
                IsCtrlDown = true;
            }
        }

        private void MainWindow_OnKeyUp(object sender, KeyEventArgs e) {
            if (e.Key == Key.LeftShift) {
                IsShiftDown = false;
            }
            if (e.Key == Key.LeftCtrl) {
                IsCtrlDown = false;
            }
        }

        private void MainWindow_OnActivated(object sender, EventArgs e) {
            IsShiftDown = Keyboard.IsKeyDown(Key.LeftShift);
            IsCtrlDown = Keyboard.IsKeyDown(Key.LeftCtrl);
        }

        private void MainWindow_OnDeactivated(object sender, EventArgs e) {
            IsShiftDown = false;
            IsCtrlDown = false;
        }

        private void CanSaveViewExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = true;
        }

        private void HandleSaveViewCommand(object sender, ExecutedRoutedEventArgs e) {
            //uiInstance.SaveView();
            RenderTargetBitmap rtb = new RenderTargetBitmap((int) NewMeshView.ActualWidth,
                (int) NewMeshView.ActualHeight, 96, 96, PixelFormats.Pbgra32);
            rtb.Render(NewMeshView);
            var rect = new Int32Rect(0, 0, rtb.PixelWidth, rtb.PixelHeight);
            if (rect.Width > rect.Height) {
                rect.X = (rect.Width - rect.Height)/2;
                rect.Width = rect.Height;
            }
            else {
                rect.Y = (rect.Height - rect.Width)/2;
                rect.Height = rect.Width;
            }
            var cropped = new CroppedBitmap(rtb, rect);
            var thumbnail = new TransformedBitmap(cropped, new ScaleTransform(
                (double) thumbnailSize/rect.Width, (double) thumbnailSize/rect.Height));
            savedViewThumbnails.Add(thumbnail);
            savedViewIds.Add(savedViewIds.Count == 0 ? 1 : savedViewIds.Max() + 1);
            UpdateSavedViewMenuItems();
        }

        private void UpdateSavedViewMenuItems() {
            var menu = SavedViewsBegin.Parent as MenuItem;
            var beginIndex = menu.Items.IndexOf(SavedViewsBegin);
            while (menu.Items[beginIndex + 1] != SavedViewsEnd) {
                menu.Items.RemoveAt(beginIndex + 1);
            }
            if (savedViewIds.Count == 0) {
                menu.Items.Insert(beginIndex + 1,
                    new MenuItem {
                        Header = "(No Saved Views)",
                        IsEnabled = false
                    });
            }
            else {
                int pos = beginIndex + 1;
                for (int i = 0; i < savedViewIds.Count; i++) {
                    var id = savedViewIds[i];
                    var image = savedViewThumbnails[i];
                    MenuItem loadItem = new MenuItem();
                    MenuItem replaceItem = new MenuItem();
                    MenuItem deleteItem = new MenuItem();

                    StackPanel loadItemPanel = new StackPanel {
                        Orientation = Orientation.Horizontal,
                        Margin = new Thickness(3),
                    };
                    loadItemPanel.Children.Add(new System.Windows.Controls.Image { Source = image, Height = thumbnailSize, Width = thumbnailSize });
                    loadItemPanel.Children.Add(new TextBlock(new Run("Load Saved View " + id)) {
                        VerticalAlignment = VerticalAlignment.Center,
                        Margin = new Thickness(6, 0, 0, 0)
                    });
                    loadItem.Header = loadItemPanel;

                    StackPanel replacePanel = new StackPanel {
                        Orientation = Orientation.Horizontal,
                        Margin = new Thickness(3),
                    };
                    replacePanel.Children.Add(new System.Windows.Controls.Image { Source = image, Height = thumbnailSize, Width = thumbnailSize });
                    replacePanel.Children.Add(new TextBlock(new Run("Save and Replace View " + id)) {
                        VerticalAlignment = VerticalAlignment.Center,
                        Margin = new Thickness(6, 0, 0, 0)
                    });
                    replaceItem.Header = replacePanel;

                    StackPanel deletePanel = new StackPanel {
                        Orientation = Orientation.Horizontal,
                        Margin = new Thickness(3),
                    };
                    deletePanel.Children.Add(new System.Windows.Controls.Image { Source = image, Height = thumbnailSize, Width = thumbnailSize });
                    deletePanel.Children.Add(new TextBlock(new Run("Delete Saved View " + id)) {
                        VerticalAlignment = VerticalAlignment.Center,
                        Margin = new Thickness(6, 0, 0, 0)
                    });
                    deleteItem.Header = deletePanel;

                    int iCopy = i;
                    if (i < _loadViewCommands.Length) {
                        loadItem.Command = _loadViewCommands[i];
                    }
                    else {
                        loadItem.Click += (sender, args) => LoadView(iCopy);
                    }
                    replaceItem.Click += (sender, args) => ReplaceView(iCopy);
                    deleteItem.Click += (sender, args) => DeleteView(iCopy);

                    DynamicMenu.SetIsNoModifierOnly(loadItem, true);
                    DynamicMenu.SetIsShiftOnly(replaceItem, true);
                    DynamicMenu.SetIsCtrlOnly(deleteItem, true);

                    menu.Items.Insert(pos, loadItem);
                    menu.Items.Insert(pos + 1, replaceItem);
                    menu.Items.Insert(pos + 2, deleteItem);
                    pos += 3;
                }
                menu.Items.Insert(pos, new MenuItem {
                    Header = "(Hold Ctrl / Shift to Modify)",
                    IsEnabled = false
                });
            }
        }

        private void DeleteView(int i) {
            //uiInstance.DeleteView(i);
            savedViewIds.RemoveAt(i);
            savedViewThumbnails.RemoveAt(i);
            UpdateSavedViewMenuItems();
            RefreshUIState();
        }

        private void ReplaceView(int i) {
            //uiInstance.SaveView(i);
            RenderTargetBitmap rtb = new RenderTargetBitmap((int) NewMeshView.ActualWidth,
                (int) NewMeshView.ActualHeight, 96, 96, PixelFormats.Pbgra32);
            rtb.Render(NewMeshView);
            var rect = new Int32Rect(0, 0, rtb.PixelWidth, rtb.PixelHeight);
            if (rect.Width > rect.Height) {
                rect.X = (rect.Width - rect.Height)/2;
                rect.Width = rect.Height;
            }
            else {
                rect.Y = (rect.Height - rect.Width)/2;
                rect.Height = rect.Width;
            }
            var cropped = new CroppedBitmap(rtb, rect);
            var thumbnail = new TransformedBitmap(cropped, new ScaleTransform(
                (double) thumbnailSize/rect.Width, (double) thumbnailSize/rect.Height));
            savedViewThumbnails[i] = thumbnail;
            UpdateSavedViewMenuItems();
            RefreshUIState();
        }

        private void LoadView(int idx) {
            // TODO: implement
            RefreshUIState();
        }

        private void CanLoadViewExecute(object sender, CanExecuteRoutedEventArgs e) {
            int idx = Array.IndexOf(_loadViewCommands, e.Command);
            e.CanExecute = idx < savedViewIds.Count;
        }

        private void HandleLoadViewCommand(object sender, ExecutedRoutedEventArgs e) {
            int idx = Array.IndexOf(_loadViewCommands, e.Command);
            if (idx < savedViewIds.Count)
                LoadView(idx);
        }

        private void CanDeleteAllSavedViewsExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = savedViewIds.Count > 0;
        }

        private void HandleDeleteAllSavedViewsCommand(object sender, ExecutedRoutedEventArgs e) {
            // TODO: implement
            UpdateSavedViewMenuItems();
            RefreshUIState();
        }

        private void File_Open_Click(object sender, RoutedEventArgs e) {
            // Configure save file dialog box
            OpenFileDialog dlg = new OpenFileDialog();
            //dlg.FileName = "Document"; // Default file name
            //dlg.DefaultExt = ".text"; // Default file extension
            //dlg.Filter = "Proto Templates (.proto)|*.proto"; // Filter files by extension 

            // Show save file dialog box
            Nullable<bool> result = dlg.ShowDialog();

            // Process save file dialog box results 
            if (result == true) {
                double x = NewMeshView.ActualWidth/2;
                double y = NewMeshView.ActualHeight/2;
                System.Windows.Point point = new System.Windows.Point(x, y);
                // Save document 
                string filename = dlg.FileName;

                TemplateRef tmpl = uiInstance.AddProtoTemplate(filename, 0, point);
                //uiInstance.Rotate(tmpl, new Quaternion(new Vector3D(1, 0, 0), -90), tmpl.Center);
                uiInstance.Translate(tmpl, new Vector3D(0, 0, 0));
                handleUpdatesFromShapeChange(true);
            }
        }

        private void Drawing_Type_Mesh_Click(object sender, RoutedEventArgs e) {
            // TODO: implement
        }

        private void Drawing_Type_Vertices_Click(object sender, RoutedEventArgs e) {
            // TODO: implement
        }

        private void Mesh_Type_Normal_Click(object sender, RoutedEventArgs e) {
            // TODO: implement
        }

        private void Mesh_Type_Point_Cloud_Click(object sender, RoutedEventArgs e) {
            // TODO: implement
        }

        private void Mesh_Type_Box_Click(object sender, RoutedEventArgs e) {
            // TODO: implement
        }

        private void Print_Generate_STL_Click(object sender, RoutedEventArgs e) {
            Window window = new Window();
            window.Width = 400;
            window.Height = 100;
            Label label = new Label{Padding = new Thickness(10), Content="Generating..."};
            window.Content = label;
            window.Title = "Generating Print...";
            Timer timer = new Timer();
            timer.Interval = 10;
            timer.Tick += (o, args) => {
                string progress = ProgressRetrieval.GetProgressString();
                label.Content = progress;
            };
            timer.Start();

            Thread thread = new Thread(() =>
            {
                uiInstance.generateFoldableSTL();
                Dispatcher.Invoke((Action)delegate
                {
                    timer.Stop();
                    window.Close();
                    RefreshSTLView();
                });
            });
            thread.Start();
            window.Loaded+= (o, args) => {
                var hwnd = new WindowInteropHelper(window).Handle;
                SetWindowLong(hwnd, GWL_STYLE, GetWindowLong(hwnd, GWL_STYLE) & ~WS_SYSMENU);
            };
            window.ShowDialog();
        }
        private void Optimize_Speed_Click(object sender, RoutedEventArgs e)
        {
            uiInstance.optimizeSpeed();
            RefreshViews();
        }
        private void Optimize_Geometry_Click(object sender, RoutedEventArgs e)
        {
            uiInstance.optimizeGeometry();
            RefreshViews();
        }
        private void Optimize_Cost_Click(object sender, RoutedEventArgs e)
        {
            uiInstance.optimizeCost();
            RefreshViews();
        }
        private void EvalOptimization_Click(object sender, RoutedEventArgs e)
        {
            uiInstance.evalStablization();
            RefreshViews();
        }
        private void Optimize_None_Click(object sender, RoutedEventArgs e)
        {
            uiInstance.optimizeNone();
            RefreshViews();
        }

        private void RefreshSTLView() {
            //default paths
            var hardStlPath = @"..\..\data\prints\result_hard.stl";
            var softStlPath = @"..\..\data\prints\result_soft.stl";
            ModelVisual3D visual = new ModelVisual3D();
            Model3DGroup model = new ModelImporter().Load(hardStlPath);
            visual.Content = model;
            foreach (var child in model.Children) {
                if (child is GeometryModel3D) {
                    var geometry = child as GeometryModel3D;
                    geometry.Material = new DiffuseMaterial(new SolidColorBrush(System.Windows.Media.Color.FromArgb(255, 255, 246, 231)));
                }
            }

            PrintView.Children.Clear();
            PrintView.Children.Add(new DefaultLights());
            PrintView.Children.Add(visual);
        }

        private void Print_Refresh_Click(object sender, RoutedEventArgs e) {
            RefreshSTLView();
        }

        private void PointerButton_OnClick(object sender, RoutedEventArgs e) {
            _uiStates.IsRotating = false;
            _uiStates.IsScaling = false;
            _uiStates.IsTranslating = false;
            _uiStates.IsMeasuring = false;
            _uiStates.IsTranslatingPart = false;
            _ui3D.Deselect();
            RefreshUIState();
        }

        private void RotationButton_OnClick(object sender, RoutedEventArgs e) {
            _uiStates.IsRotating = true;
            _uiStates.IsScaling = false;
            _uiStates.IsMeasuring = false;
            _uiStates.IsTranslating = false;
            _uiStates.IsTranslatingPart = false;
            _ui3D.Deselect();
            RefreshUIState();
        }

        private void ScaleButton_OnClick(object sender, RoutedEventArgs e) {
            _uiStates.IsScaling = true;
            _uiStates.IsRotating = false;
            _uiStates.IsMeasuring = false;
            _uiStates.IsTranslating = false;
            _uiStates.IsTranslatingPart = false;
            _ui3D.Deselect();
            RefreshUIState();
        }

        private void TranslatePartButton_OnClick(object sender, RoutedEventArgs e)
        {
            _uiStates.IsScaling = false;
            _uiStates.IsRotating = false;
            _uiStates.IsMeasuring = false;
            _uiStates.IsTranslating = false;
            _uiStates.IsTranslatingPart = true;
            _ui3D.Deselect();
            RefreshUIState();
        }

        private void MeasureButton_OnClick(object sender, RoutedEventArgs e)
        {
            _uiStates.IsScaling = false;
            _uiStates.IsRotating = false;
            _uiStates.IsMeasuring = true;
            _uiStates.IsTranslating = false;
            _uiStates.IsTranslatingPart = false;
            _ui3D.Deselect();
            RefreshUIState();
        }




    /*    private void JointInfoBox_OnSelectionChanged(object sender, SelectionChangedEventArgs e) {
            var selection = _uiStates.Selection;
            if (selection.Type == Selection.SelectionType.Patch) {
                var tmpl = selection.SelectedTemplate;
                var patch = selection.SelectedPatch;
                var connections = tmpl.Root.getAllConnectionsForPatch(patch);
                foreach (var connection in connections) {
                    var c = connection;
                    c.Type = (ConnectionType) JointInfoBox.SelectedValue;
                }
            }
        } */

        private void UpDownBase_OnValueChanged(object sender, RoutedPropertyChangedEventArgs<object> e) {
            var selection = _uiStates.Selection;
            if (selection.Type == Selection.SelectionType.Patch) {
                var tmpl = selection.SelectedTemplate;
                var patch = selection.SelectedPatch;
                var connections = tmpl.Root.getAllConnectionsForPatch(patch);
                foreach (var connection in connections) {
                    var c = connection;
                    c.Angle = (double) e.NewValue;
                }
            }
        }

        private void _uiStates_SelectionChanged() {
            bool isConnection = false;
            var selection = _uiStates.Selection;
            //this.ArticulationEditorControl.Visibility = this.SaveArticulationButton.Visibility = Visibility.Hidden;
            if (selection.Type == Selection.SelectionType.Patch) {
                var tmpl = selection.SelectedTemplate;
                var patch = selection.SelectedPatch;
                var connections = tmpl.Root.getAllConnectionsForPatch(patch);
               /* foreach (var connection in connections) {
                    isConnection = true;
                    JointInfoBox.SelectedIndex = (int) connection.Type;
                    JointInfoAngleBox.Value = connection.Angle;
                    
                    byte[] articulationSerialized = connection.GetArticulation();
                    this.ArticulationEditorControl.Visibility = articulationSerialized != null ? Visibility.Visible : Visibility.Collapsed;
                    this.SaveArticulationButton.Visibility = articulationSerialized != null ? Visibility.Visible : Visibility.Collapsed;
                    if (articulationSerialized != null) {
                        Articulation articulation =
                            ProtoBuf.Serializer.Deserialize<Articulation>(new MemoryStream(articulationSerialized));
                        ArticulationEditorControl.Edit(articulation);
                    }
                }*/
            }
            else if (selection.Type == Selection.SelectionType.Template) {
                var tmpl = selection.SelectedTemplate;
                var openscadDesign = tmpl.GetOpenscadDesign();
                if (openscadDesign != null) {
                    OpenscadDesign design =
                        ProtoBuf.Serializer.Deserialize<OpenscadDesign>(new MemoryStream(openscadDesign));
                    ScadEditor.Edit(design);
                }
            }

            //JointInfoBox.IsEnabled = isConnection;
            //JointInfoAngleBox.IsEnabled = isConnection;
        }


        private void UpdateSymmetrySelectionUI()
        {
           if (!uiInstance.hasMainWorkingTemp())
            {
                symmetryChooser.Visibility = Visibility.Collapsed;
            }
            else
            {
                symmetryChooser.Visibility = Visibility.Visible;
                var symmChoices = uiInstance.GetSymmetryChoices();
                if (symmChoices != null)
                {
                    symmetryChooser.Choose(symmChoices.symm_ground, symmChoices.symm_legW, symmChoices.symm_legL, symmChoices.symm_spacing);
                }
            }
        }


        private void updateGaitSuggestionsTab()
        {

            alternativeViews.Children.Clear();
            alternativeViews2.Children.Clear();

            if (jointChoices != null)
            {


                for (int i = 0; (i < jointChoices.Labelings.Count); i++)
                {
                    //if (i == currentlySelected) continue;
                    ConfigurationView view = new ConfigurationView();
                    view.Show(jointLocations, jointChoices.Labelings[i], topViewImage, drawing_translate, drawing_size);
                    int iCopy = i;



                    //view.MouseEnter += (sender, args) => view.Background = new SolidColorBrush(Colors.LightBlue);
                    //view.MouseLeave += (sender, args) => view.Background = new SolidColorBrush(Colors.White);
                    view.MouseUp += (sender, args) =>
                    {
                        uiInstance.updateJointGaitChoice(iCopy);
                        updateListOfSavedGait(-1);

                        EditGaitTab.IsEnabled = true;
                        SequenceTab.IsEnabled = true;

                        EditGaitTab.IsSelected = true;

                        updateEditGaitTab();
                        updateMetrics(); 

                        //                   SuggestionsTab.Visibility = System.Windows.Visibility.Visible;
                        //                  SuggestionsTab.IsSelected = false;

                        // create a new one and go to the edit button 
                        //UpdateJointSelectionUI();
                        //for (int j = 0; j < jointChoices.CurrentChoice.Count; j++)
                        //{
                        //  
                        //    jointChoices.CurrentChoice[j]= jointChoices.Labelings[iCopy].JointLabels[j];
                        //}
                        //Redraw(selectionCallbackManual, selectionCallbackChoice, jointChoices);
                    };
                    if ((i % 2) == 0)
                    {
                        alternativeViews.Children.Add(view);
                    }
                    else
                    {
                        alternativeViews2.Children.Add(view);
                    }



                }

            }
        }

private void updateSequenceCanvas(object sender, RoutedEventArgs e)
{
    updateSquenceGaitTab();   
}

private void updateSquenceGaitTab()
{

    List<List<int>> listofGaitLabels = new List<List<int>>();
    List<List<double>> listofGaitAngles = new List<List<double>>();
    for (int i = 0; i < GaitList_ComboBox.Items.Count; i++)
    {
        SingleGait gait = uiInstance.getSingleGait(i);
        List<int> labels = gait.JointLabels;
        List<double> angles = gait.JointAngles;
        listofGaitLabels.Add(labels);
        listofGaitAngles.Add(angles); 
    }
    if (GaitOptionsCanvas.IsLoaded)
    {
        if (jointLocations != null)
        {
            GaitSequence.drawGaitOptions(_uiStates, jointLocations, listofGaitLabels, listofGaitAngles, GaitOptionsCanvas, topViewImage, drawing_translate, drawing_size);
        }
    }
    // should be get the current sequence from the  uiIstance;
    if (SequenceCanvas.IsLoaded)
    {
        GaitSequence.drawSequence((deleteId) => { 
                currentSequence[deleteId] = -1;
                updateSquenceGaitTab();
                uiInstance.updateCurrentSequenceInWt(currentSequence);
                updateMetrics();   
                            }, _uiStates, currentSequence, SequenceCanvas);
    }
}

private void ChangeMotionSequenceSize_clicked(object sender, RoutedEventArgs e){
    if (uiInstance != null)
    {
        int val = (int)InputMotionSequenceSize.Value;
        _uiStates.motionSequenceLen = val;
        updateSquenceGaitTab();
        int addLen = val - currentSequence.Count;
        int remLen = -addLen;
        if(addLen >0){
            for (int i = 0; i < addLen; i++)
            {
                currentSequence.Add(-1);
            }
        }
        if (remLen > 0)
        {
            for (int i = 0; i < remLen; i++)
            {
                currentSequence.RemoveAt(currentSequence.Count - 1);
            }
        }
        updateSquenceGaitTab(); 
        uiInstance.updateCurrentSequenceInWt(currentSequence);
        updateMetrics(); 
    }
}

private void clearScene()
{
    handleUpdatesFromTopoChange();

    updateTheSelectedGaitTab();
    updateMetrics();
}



 private void updateEditGaitTab()
 {
     angleEditors.Children.Clear();
     DrawingGaitCanvas.Children.Clear();
     if(GaitList_ComboBox.SelectedIndex >=0){
     SingleGait gait = uiInstance.getSingleGait( GaitList_ComboBox.SelectedIndex);

     if (gait != null)
     {
         //DrawingGaitCanvas.Background = (_uiStates.niceColors[GaitList_ComboBox.SelectedIndex]);
         RenameGaitOptionBox.Text = gait.name;
         //InputCurvature.Value = gait.desiredAngle;


         //selectedView.Show(layout, labelings[currentlySelected]);
         DrawingGait.Draw((i, isup) =>
                        {
                            uiInstance.updateGaitSingleLable(gait.id, i, isup);
                            handleUpdatesFromGaitChange();
                            updateEditGaitTab();
                        }, gait.JointLabels, jointLocations, DrawingGaitCanvas, topViewImage, drawing_translate, drawing_size, topViewDrawing,
                        _uiStates.niceColorsNoBrush[GaitList_ComboBox.SelectedIndex]);



         if (gait.JointAngles.Count > 0)
         {

             for (int j = 0; j < gait.JointAngles.Count; j++)
             {
                 var jCopy = j;
                 // add angle feedback
                 AngleEditor angle = new AngleEditor(gait.JointAngles[jCopy], (theta) =>
                    {
                        Console.Write("Uptating theta for angle " + jCopy); 
                        uiInstance.updateGaitSingleAngle(gait.id, jCopy, theta);
                        updateMetrics();
                        //handleUpdatesFromGaitChange();
                        
                    });
                 angle.Show(jointLocations, j, gait.JointAngles[jCopy], topViewImage, drawing_translate, drawing_size, _uiStates.niceColorsNoBrush[GaitList_ComboBox.SelectedIndex]);
                 angleEditors.Children.Add(angle);
             }
         }
     }
     }
 }

 private void GaitList_ComboBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
 {
     e.Handled = true;
     int currentGaitEdit = GaitList_ComboBox.SelectedIndex;
     if (currentGaitEdit >= 0)
     {
         updateEditGaitTab();
     }
 }

 private void RenameEvent_clicked(object sender, RoutedEventArgs  e)
 {

     int gaitId = GaitList_ComboBox.SelectedIndex;
     String newName = RenameGaitOptionBox.Text;
     uiInstance.updateGaitSingleName(gaitId, newName);
     updateListOfSavedGait(GaitList_ComboBox.SelectedIndex);
     updateMetrics(); 
 }

/*
 private void ChangeDesiredDirectionEvent_clicked(object sender, RoutedEventArgs e)
 {

     int gaitId = GaitList_ComboBox.SelectedIndex;
     double newDirection = (double)InputCurvature.Value;
     if(uiInstance != null){
         uiInstance.updateGaitSingleDirection(gaitId, newDirection);
     }
 }
*/


 private void updateListOfSavedGait(int setId)
 {
      GaitList_ComboBox.Items.Clear();

    List<String> names = uiInstance.getAllGaitNames();
    foreach (String name in names)
    {
        GaitList_ComboBox.Items.Add(name);

    }

    GaitList_ComboBox.SelectedIndex = setId;

    if (setId < 0)
    {
           GaitList_ComboBox.SelectedIndex = names.Count - 1;
    }
}





 private void updateTheSelectedGaitTab_clicked (object sender, SelectionChangedEventArgs e)
 {
     e.Handled = true; 
     updateTheSelectedGaitTab(); 

 }

 private void updateTheSelectedGaitTab()
 {
     if (EditGaitTab.IsSelected)
     {
         updateEditGaitTab();
     }
     if (SuggestionsTab.IsSelected)
     {
         updateGaitSuggestionsTab();
     }
     if(SequenceTab.IsSelected){
         updateSquenceGaitTab();
     }

 }


private void clearCurrentSequence(){

    currentSequence.Clear();
    for (int i = 0; i < InputMotionSequenceSize.Value; i++)
    {
        currentSequence.Add(-1);
    }
    uiInstance.updateCurrentSequenceInWt(currentSequence);
    updateMetrics();   

}

private void handleUpdatesFromTopoChange()
{
    jointChoices = uiInstance.GetJointChoices();
    updateListOfSavedGait(0);

    SuggestionsTab.IsSelected = true;
    EditGaitTab.IsEnabled = false;
    SequenceTab.IsEnabled = false;

    currentSequence = uiInstance.getSequence();
    if (currentSequence != null)
    {
        InputMotionSequenceSize.Value = currentSequence.Count;
        if (currentSequence.Count == 0)
        {
            clearCurrentSequence();
        }
    }
    //updateTheSelectedGaitTab();
    //uiInstance.updateControllers(); // the get joint choices already does it
    UpdateSymmetrySelectionUI();
    _uiStates.selecteMetric_Gait = -1;
    _uiStates.selecteMetric_Objective = -1;

}

 private void handleUpdatesFromShapeChange(bool connectedNewPart)
 {
     //geometry updates
     topViewDrawing = uiInstance.getTopViewDrawing();
     topViewImage = ConvertDrawingToImage(topViewDrawing); 
     jointLocations = uiInstance.getJointLocations();
     RefreshUIState(); // includes RefreshViews();

     if (connectedNewPart){
         handleUpdatesFromTopoChange();
     }
     updateTheSelectedGaitTab();
     //uiInstance.updateControllers(GaitList_ComboBox.SelectedIndex);
     updateMetrics();
     //updateGaitSuggestionsTab();
 }

 private void handleUpdatesFromGaitChange()
 {
     // updateGaitSuggestionsTab();
     //updateEditGaitTab(uiInstance->getSingleGait(0)
     //uiInstance.updateControllers(GaitList_ComboBox.SelectedIndex);
     updateMetrics();
     //uiInstance.updateMetrics(_uiStates.NAnimRounds, _uiStates.DeltaAnim);

 }

        private void CanMultiSnapExecute(object sender, CanExecuteRoutedEventArgs e) {
            e.CanExecute = uiInstance != null && uiInstance.getTemplates().Count >= 2;
        }

        private void HandleMultiSnapCommand(object sender, ExecutedRoutedEventArgs e) {
            double maxDistance = 100;
            uiInstance.MultiSnap(maxDistance);
            RefreshUIState();
        }
        /*
        private void SaveArticulationTransformationsButtonClick(object sender, RoutedEventArgs e) {
            var selection = _uiStates.Selection;
            if (selection.Type == Selection.SelectionType.Patch)
            {
                var tmpl = selection.SelectedTemplate;
                var patch = selection.SelectedPatch;
                var connections = tmpl.Root.getAllConnectionsForPatch(patch);
                foreach (var connection in connections)
                {
                    MemoryStream memoryStream =new MemoryStream();
                    ProtoBuf.Serializer.Serialize(memoryStream, ArticulationEditorControl.Articulation);
                    connection.SetArticulation(memoryStream.GetBuffer(), uiInstance);

                    // this will call this optimization so we need to get it back and display it
                    byte[] articulationSerialized = connection.GetArticulation();
                    if (articulationSerialized != null)
                    {
                        Articulation articulation_update =
                            ProtoBuf.Serializer.Deserialize<Articulation>(new MemoryStream(articulationSerialized));
                        ArticulationEditorControl.Edit(articulation_update);
                    }



                }
            }
        }
        */
        private void SaveScadDesignButtonClick(object sender, RoutedEventArgs e) {
            var selection = _uiStates.Selection;
            if (selection.Type == Selection.SelectionType.Template) {
                var tmpl = selection.SelectedTemplate;
                MemoryStream memoryStream = new MemoryStream();
                ProtoBuf.Serializer.Serialize(memoryStream, ScadEditor.Design);
                tmpl.SetOpenscadDesign(memoryStream.GetBuffer());
                RefreshViews();
            }
        }

        private void HandlePreventFurtherOverlapCommand(object sender, ExecutedRoutedEventArgs e) {
            _uiStates.PreventCollisions = !_uiStates.PreventCollisions;
            if (_uiStates.PreventCollisions)
            {
                OverlapPreventionHack.countOverlapnessDelegate = CountOverlapness;
            }
            else
            {
                OverlapPreventionHack.countOverlapnessDelegate = drawing => 0;
            }
        }

        private void HandleShowOverlapsCommand(object sender, ExecutedRoutedEventArgs e)
        {
            _uiStates.ShowOverlaps = !_uiStates.ShowOverlaps;
            RefreshViews();
        }

        private void HandleForceReflectionCommand(object sender, ExecutedRoutedEventArgs e)
        {
            OverlapPreventionHack.SetAlwaysReflect(!OverlapPreventionHack.IsAlwaysReflect());
        }

        private void RotateY180DegClick(object sender, RoutedEventArgs e)
        {
            if (!_uiStates.Selection.SelectedTemplate.IsNull)
            {
                TemplateRef root = _uiStates.Selection.SelectedTemplate.Root;
                uiInstance.Rotate(root, new Quaternion(new Vector3D(0, 1, 0), 180), root.Center);
                RefreshUIState();
            }
        }

        private void CommandBox_OnKeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                command.Execute(CommandBox.Text);
                CommandBox.Text = "";
                RefreshUIState();
            }
        }

        private void SaveDrawingAsSVGClick(object sender, RoutedEventArgs e)
        {
            /*
            SaveFileDialog dialog = new SaveFileDialog();
            dialog.Filter = "SVG Files (*.svg)|*.svg";
            if (dialog.ShowDialog(this) == true)
            {
                SVGExport.ExportSVG(this.DrawingCanvas, dialog.FileName);
            }
             */
        }
    }
}