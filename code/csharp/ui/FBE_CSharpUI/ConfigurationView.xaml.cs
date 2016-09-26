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
    public partial class ConfigurationView : UserControl
    {
        public ConfigurationView()
        {
            InitializeComponent();
            SizeChanged += (sender, args) => Redraw();
        }

        private List<Point> jointLocations;
        private JointLabeling labeling;
        BitmapImage bitmapImage;
        Vector drawing_translate;
        Vector drawing_size;

        public void Show(List<Point> jointLocations, JointLabeling labeling, BitmapImage bitmapImage, Vector drawing_translate, Vector drawing_size)
        {
            this.jointLocations = jointLocations;
            this.labeling = labeling;
            this.bitmapImage = bitmapImage;
            this.drawing_size = drawing_size;
            this.drawing_translate = drawing_translate;

            Redraw( );
        }

        protected override Size MeasureOverride(Size constraint) {
            if (constraint.Height > constraint.Width*2/3) {
                return new Size(constraint.Width, constraint.Width * 2 / 3);
            }
            if (constraint.Width > constraint.Height* 3/2)
            {
                return new Size(constraint.Height * 3 / 2, constraint.Height);
            }
            return constraint;
        }

        private void Redraw()
        {

            if (mainCanvas.ActualWidth > 0)
            {
                if (jointLocations == null || labeling == null) return;

                mainCanvas.Children.Clear();

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


                //polygon.Fill = new SolidColorBrush(Color.FromArgb(255, 190, 190, 190));
                Image topview = new Image();
                topview.Source = bitmapImage;
                topview.Width = bitmapImage.Width * scale;
                topview.Height = bitmapImage.Height * scale;
                Point cornerVertex = new Point(drawing_translate.X, drawing_translate.Y) * matrix;
                Canvas.SetLeft(topview, cornerVertex.X);
                Canvas.SetTop(topview, cornerVertex.Y);
                mainCanvas.Children.Add(topview);
                Color darkWhite = Color.FromRgb(210, 210, 210); 

                Color simpleBlack = Color.FromArgb(255, 80, 80, 80);
                for (int i = 0; i < jointLocations.Count; i++)
                {
                    Point joint = jointLocations[i];
                    int label = labeling.JointLabels[i];
                    Ellipse ellipse = new Ellipse();
                    ellipse.Width = 20;
                    ellipse.Height = 20;
                    ellipse.Fill = new SolidColorBrush(darkWhite);
                    ellipse.Stroke = null;
                    ellipse.SetValue(Canvas.TopProperty, (joint * matrix).Y - 10);
                    ellipse.SetValue(Canvas.LeftProperty, (joint * matrix).X - 10);
                    mainCanvas.Children.Add(ellipse);


                    double angle = labeling.JointAngles[i];
                    if (angle != 0)
                    {
                        GaitSequence.drawOrientationArrow(mainCanvas, (joint * matrix), (angle < 0));
                    }
                        


                    Label text = new Label();
                    text.Content = label;
                    text.HorizontalContentAlignment = HorizontalAlignment.Center;
                    text.VerticalContentAlignment = VerticalAlignment.Center;
                    text.FontSize = 15;
                    text.Foreground = new SolidColorBrush(Colors.Black);
                    text.Width = 20;
                    text.Height = 20;
                    text.Padding = new Thickness(0);
                    text.SetValue(Canvas.TopProperty, (joint * matrix).Y - 10);
                    text.SetValue(Canvas.LeftProperty, (joint * matrix).X - 10);
                    if (label != 0)
                        mainCanvas.Children.Add(text);
                }
            }
        }
    }
}
