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
using Brushes = System.Windows.Media.Brushes;
using Color = System.Windows.Media.Color;
using Drawing = CppCsBridge.Drawing;

namespace FBE_CSharpUI
{
    class DrawingGait
    {
        static Brush polygonFill = new SolidColorBrush(Color.FromArgb(37, 255, 0, 0));
        static Brush selectedFill = new SolidColorBrush(Color.FromArgb(172, 60, 227, 53));
        static Brush hoverFill = new SolidColorBrush(Color.FromArgb(174, 223, 230, 56));


        public static void Draw(Action<int, bool> selectionCallback, List<int> labels,  List<Point> jointLocations, Canvas mainCanvas,
             BitmapImage bitmapImage, Vector drawing_translate, Vector drawing_size, Drawing drawing, Color color)
        {


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

            double width = Math.Max(mainCanvas.Width - 10, 0);
            double height = Math.Max(mainCanvas.Height - 10, 0);
            double marginRatio = 0.6;
            double scale = Math.Min(width * (1 - marginRatio) / (xmax - xmin), height * (1 - marginRatio) / (ymax - ymin));
            //Console.WriteLine(scale + " " + width + " " + height + " " + xmin + " " + xmax + " " + ymin + " " + ymax);
            Vector pCenter = 0.5 * new Vector(xmax + xmin, ymax + ymin);
            Vector center = 0.5 * new Vector(width + 10, height + 10);
            Vector translate = center - pCenter;
            Matrix matrix = Matrix.Identity;
            matrix.Translate(-pCenter.X, -pCenter.Y);
            matrix.Scale(scale, scale);
            matrix.Translate(pCenter.X, pCenter.Y);

            matrix.Translate(translate.X, translate.Y);

            //Polygon robotPolygon = new Polygon();
            //foreach (var point in jointChoices.Layout.RobotShape)
            //{
             //   robotPolygon.Points.Add(point * matrix);
            //}
            //robotPolygon.Fill = Brushes.Azure;
            //robotPolygon.Stroke = Brushes.DarkSlateGray;
            //mainCanvas.Children.Add(robotPolygon);



            // draw the intial shape
         //   foreach (var face in drawing.faces)
         //   {
         //       Polygon polygon = new Polygon();
         //       foreach (var point in face.points)
         //       {
         //           polygon.Points.Add(point * matrix);
          //      }
          //      polygon.Fill = new SolidColorBrush(Colors.Red);
          //      mainCanvas.Children.Add(polygon);
          //  }

           
            Color simpleBlack = Color.FromArgb(255, 0, 0, 0);
            Color darkWhite =   Color.FromRgb(210, 210, 210);

            Color hightlightColor = darkWhite;
            Image topview = GaitSequence.getImageFromBitMap(bitmapImage, color); 
            topview.Height = bitmapImage.Height * scale;
            topview.Width = bitmapImage.Width * scale;
            Point cornerVertex = new Point(drawing_translate.X, drawing_translate.Y) * matrix;
            Canvas.SetLeft(topview, cornerVertex.X);
            Canvas.SetTop(topview, cornerVertex.Y);
            mainCanvas.Children.Add(topview);

            for (int i = 0; i < jointLocations.Count; i++)
            {
                var iCopy = i;
                Point joint = jointLocations[i];
                int label = labels[i];
                Ellipse ellipse = new Ellipse();
                int rad = 12;
                ellipse.Width = 2*rad;
                ellipse.Height = 2*rad;
                ellipse.Fill = new SolidColorBrush(darkWhite);
                ellipse.Stroke = new SolidColorBrush(Colors.Black);
                ellipse.StrokeThickness = 2;
                ellipse.SetValue(Canvas.TopProperty, (joint * matrix).Y - rad);
                ellipse.SetValue(Canvas.LeftProperty, (joint * matrix).X - rad);
                mainCanvas.Children.Add(ellipse);

                if (label != 0)
                {
                    Polygon arrowDown = new Polygon();
                    arrowDown.Points.Add(new Point(-5 + (joint * matrix).X, 12 + (joint * matrix).Y));
                    arrowDown.Points.Add(new Point(5 + (joint * matrix).X, 12 + (joint * matrix).Y));
                    arrowDown.Points.Add(new Point(5 + (joint * matrix).X, 22 + (joint * matrix).Y));
                    arrowDown.Points.Add(new Point(10 + (joint * matrix).X, 22 + (joint * matrix).Y));
                    arrowDown.Points.Add(new Point(0 + (joint * matrix).X, 27 + (joint * matrix).Y));
                    arrowDown.Points.Add(new Point(-10 + (joint * matrix).X, 22 + (joint * matrix).Y));
                    arrowDown.Points.Add(new Point(-5 + (joint * matrix).X, 22 + (joint * matrix).Y));
                    arrowDown.Fill = new SolidColorBrush(simpleBlack);


                    arrowDown.MouseDown += (sender, args) =>
                    {
                        selectionCallback(iCopy, false);

                    };
                    arrowDown.MouseEnter += (sender, args) =>
                    {
                        arrowDown.Fill = new SolidColorBrush(hightlightColor);
                    };

                    arrowDown.MouseLeave += (sender, args) =>
                    {
                        arrowDown.Fill = new SolidColorBrush(simpleBlack);
                    };
                    mainCanvas.Children.Add(arrowDown);
                    Polygon arrowUP = new Polygon();
                    arrowUP.Points.Add(new Point(-5 + (joint * matrix).X, -12 + (joint * matrix).Y));
                    arrowUP.Points.Add(new Point(5 + (joint * matrix).X, -12 + (joint * matrix).Y));
                    arrowUP.Points.Add(new Point(5 + (joint * matrix).X, -22 + (joint * matrix).Y));
                    arrowUP.Points.Add(new Point(10 + (joint * matrix).X, -22 + (joint * matrix).Y));
                    arrowUP.Points.Add(new Point(0 + (joint * matrix).X, -27 + (joint * matrix).Y));
                    arrowUP.Points.Add(new Point(-10 + (joint * matrix).X, -22 + (joint * matrix).Y));
                    arrowUP.Points.Add(new Point(-5 + (joint * matrix).X, -22 + (joint * matrix).Y));
                    arrowUP.Fill = new SolidColorBrush(simpleBlack);
                    arrowUP.MouseDown += (sender, args) =>
                    {
                        selectionCallback(iCopy, true);

                    };

                    arrowUP.MouseEnter += (sender, args) =>
                    {
                        arrowUP.Fill = new SolidColorBrush(hightlightColor);
                    };

                    arrowUP.MouseLeave += (sender, args) =>
                    {
                        arrowUP.Fill = new SolidColorBrush(simpleBlack);
                    };
                    mainCanvas.Children.Add(arrowUP);





                        

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
                    mainCanvas.Children.Add(text);
                }
            }


        }
    }
}
