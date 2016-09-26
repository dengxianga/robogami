using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using CppCsBridge;
using Drawing = CppCsBridge.Drawing;

namespace FBE_CSharpUI
{
    /// <summary>
    /// Interaction logic for ConfigurationView.xaml
    /// </summary>
    public partial class AngleEditor : UserControl
    {

        private List<Point> jointLocations;
        private int jointID;
        private BitmapImage bitmapImage;
        private Vector drawing_translate;
        private Vector drawing_size;
        private double angle;
        private Color color; 
        public AngleEditor(double theta, Action<double> changeAngleCallBack)
        {
            InitializeComponent();
            SizeChanged += (sender, args) => Redraw();
            thetaSlider.Value = theta; 
            thetaSlider.ValueChanged += (sender, args) =>
            {
                changeAngleCallBack(thetaSlider.Value);
                angle = thetaSlider.Value;
                Redraw(); 
            };
           
        }



        public void Show(List<Point> jointLocations, int jointID, double angle,
            BitmapImage bitmapImage, Vector drawing_translate, Vector drawing_size, Color color)
        {
            this.jointLocations = jointLocations;
            this.jointID = jointID;
            this.bitmapImage = bitmapImage;
            this.drawing_size = drawing_size;
            this.drawing_translate = drawing_translate;
            this.angle = angle;
            this.color = color;
            Redraw();
        }



        private void Redraw()
        {

            if (mainCanvas.ActualWidth > 0)
            {
                if (jointLocations == null) return;

                mainCanvas.Children.Clear();
                Color darkWhite = Color.FromRgb(210, 210, 210); 

                //finding bounding box
                double xmin, xmax, ymin, ymax;
                List<double> Xs = new List<double>();
                List<double> Ys = new List<double>();
                foreach (var point in jointLocations)
                {
                    Xs.Add(point.X);
                    Ys.Add(point.Y);
                }
                Xs.Add(drawing_translate.X);
                Xs.Add(drawing_translate.X + drawing_size.X);
                Ys.Add(drawing_translate.Y);
                Ys.Add(drawing_translate.Y + drawing_size.Y);
                if (Xs.Count == 0 || Ys.Count == 0) return;
                xmin = Xs.Min();
                xmax = Xs.Max();
                ymin = Ys.Min();
                ymax = Ys.Max();

                double width = mainCanvas.ActualWidth - 4;
                double height = mainCanvas.ActualHeight - 4;
                double marginRatio = 0.1;
                double scale = Math.Min(width * (1 - marginRatio) / (xmax - xmin), height * (1 - marginRatio) / (ymax - ymin));
                Vector pCenter = 0.5 * new Vector(xmax + xmin, ymax + ymin);
                Vector center = 0.5 * new Vector(width + 4, height + 4);
                Vector translate = center - pCenter;
                Matrix matrix = Matrix.Identity;
                matrix.Translate(-pCenter.X, -pCenter.Y);
                matrix.Scale(scale, scale);
                matrix.Translate(pCenter.X, pCenter.Y);

                matrix.Translate(translate.X, translate.Y);

                //Polygon robotPolygon = new Polygon();
                //foreach (var point in layout.RobotShape)
                //{
                //    robotPolygon.Points.Add(point * matrix);
                // }
                //robotPolygon.Fill = Brushes.Azure;
                //robotPolygon.Stroke = Brushes.DarkSlateGray;
                //mainCanvas.Children.Add(robotPolygon);

                Image topview = GaitSequence.getImageFromBitMap(bitmapImage, color);
                topview.Width = bitmapImage.Width * scale;
                topview.Height = bitmapImage.Height * scale;
                Point cornerVertex = new Point(drawing_translate.X, drawing_translate.Y) * matrix;
                Canvas.SetLeft(topview, cornerVertex.X);
                Canvas.SetTop(topview, cornerVertex.Y);
                mainCanvas.Children.Add(topview);

                Color simpleBlack = Color.FromArgb(255, 100, 100, 100);
                int i = jointID;
                {
                    Point joint = jointLocations[i];

                    Ellipse ellipse = new Ellipse();
                    ellipse.Width = 20;
                    ellipse.Height = 20;
                    ellipse.Fill = new SolidColorBrush(darkWhite);
                    ellipse.Stroke = null;
                    ellipse.SetValue(Canvas.TopProperty, (joint * matrix).Y - 10);
                    ellipse.SetValue(Canvas.LeftProperty, (joint * matrix).X - 10);
                    mainCanvas.Children.Add(ellipse);

                    if (angle != 0)
                    {
                        GaitSequence.drawOrientationArrow(mainCanvas, (joint * matrix), (angle < 0));
                    }

                }
            }
        }
    }
}
