using System;
using System.Collections.Generic;
using System.Data.SqlTypes;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Media;
using System.Windows.Shapes;
using CppCsBridge;
using System.Windows.Media.Imaging;
using Brushes = System.Windows.Media.Brushes;
using Color = System.Windows.Media.Color;
using Drawing = CppCsBridge.Drawing;

namespace FBE_CSharpUI
{
    class DrawingPath
    {
        static Brush polygonFill = new SolidColorBrush(Color.FromArgb(37, 255, 0, 0));
        static Brush selectedFill = new SolidColorBrush(Color.FromArgb(172, 60, 227, 53));
        static Brush hoverFill = new SolidColorBrush(Color.FromArgb(174, 223, 230, 56));

        public static void Draw(UIStates uiStates,  List<PointList2D> paths,  Canvas canvas,
            BitmapImage bitmapImage, Vector drawing_translate, Vector drawing_size, Drawing drawing, bool isCakeTask, List<double> rotationAngles,
            List<int>gaitSequence
            )
        {



            canvas.Children.Clear();

            //List<Point> floorPoints = new List<Point>();
            //double x = 1000.0; // floor width / 2
            //double z = 1000.0; // floor length / 2
            //floorPoints.Add(new Point(-x, z));
            //floorPoints.Add(new Point(x, z));
            //floorPoints.Add(new Point(x, -z));
            //floorPoints.Add(new Point(-x, -z));


            //finding bounding box
            double xmin, xmax, ymin, ymax;
            List<double> Xs = new List<double>();
            List<double> Ys = new List<double>();
            foreach (var path in paths)
            {
                foreach (var point in path.points)
                {
                    Xs.Add(point.X);
                    Ys.Add(point.Y);
                }
            }
            Xs.Add(drawing_translate.X);
            Xs.Add(drawing_translate.X + drawing_size.X);
            Ys.Add(drawing_translate.Y);
            Ys.Add(drawing_translate.Y + drawing_size.Y);

            if (isCakeTask)
            {
                Xs.Add(-600);
                Ys.Add(-600);
                Ys.Add(+600);
            }


            //foreach (var point in floorPoints)
            //{
            //    Xs.Add(point.X);
            //    Ys.Add(point.Y);
            //}
            if (Xs.Count == 0 || Ys.Count == 0) return;
            xmin = Xs.Min();
            xmax = Xs.Max();
            ymin = Ys.Min();
            ymax = Ys.Max();

            double bsize = 100;

            double ixmin = Math.Floor(xmin / bsize) ;
            double ixmax = Math.Ceiling(xmax / bsize) ;
            double iymin = Math.Floor(ymin / bsize) ;
            double iymax = Math.Ceiling(ymax / bsize) ;

            xmin = ixmin * bsize;
            ymin = iymin * bsize;
            xmax = ixmax * bsize;
            ymax = iymax * bsize;



            double width = canvas.ActualWidth;
            double height = canvas.ActualHeight;
            double marginRatio = 0.1;
            double scale = Math.Min(width * (1 - marginRatio) / (xmax - xmin), height * (1 - marginRatio) / (ymax - ymin));
            Vector pCenter = 0.5 * new Vector(xmax + xmin, ymax + ymin);
            Vector center = 0.5 * new Vector(width, height);
            Vector translate = center - pCenter;
            Matrix matrix = Matrix.Identity;
            matrix.Translate(-pCenter.X, -pCenter.Y);
            matrix.Scale(scale, scale);
            matrix.Translate(pCenter.X, pCenter.Y);

            matrix.Translate(translate.X, translate.Y);
            //matrix.Scale(scale, scale);

           

            for (double i = ixmin; i < (ixmax); i = i + 1)
            {
                for (double j = iymin; j < iymax; j = j + 1)
                {
                    Polygon polygon = new Polygon();
                    polygon.Points.Add((new Point(i * bsize, j * bsize)) * matrix);
                    polygon.Points.Add((new Point((i + 1) * bsize, (j) * bsize)) * matrix);
                    polygon.Points.Add((new Point((i + 1) * bsize, (j + 1) * bsize)) * matrix);
                    polygon.Points.Add((new Point((i) * bsize, (j + 1) * bsize)) * matrix);
                    polygon.Stroke = Brushes.LightGray;
                    polygon.StrokeThickness = 1;
                    polygon.Fill = new SolidColorBrush(Color.FromArgb(255, 255, 255, 255));
                    canvas.Children.Add(polygon);
                }

            }




            // draw the intial shape
          //  foreach (var face in drawing.faces)
          //  {
          //      Polygon polygon = new Polygon();
          //      foreach (var point in face.points)
          //     {
          //         polygon.Points.Add(point * matrix);
          //      }
          //      polygon.Fill = new SolidColorBrush(Colors.Red);
           //     canvas.Children.Add(polygon);
           // }

            if (isCakeTask)
            {
                List<Point> positions = new List<Point>();
                positions.Add(new Point(-300 - 50, 0 - 50));
                positions.Add(new Point(-300 - 50, 400 - 50));
                positions.Add(new Point(-100 - 50, 200 - 50));
                positions.Add(new Point(-500 - 50, 200 - 50));
                positions.Add(new Point(-300 - 50, -400 - 50));
                positions.Add(new Point(-100 - 50, -200 - 50));
                positions.Add(new Point(-500 - 50, -200 - 50));

                
                for (int p = 0; p < positions.Count; p++)
                {
                    Rectangle rec = new Rectangle();
                    rec.Width = 100 * scale ;
                    rec.Height = 100 * scale;
                    rec.SetValue(Canvas.TopProperty, (positions[p] * matrix).Y);
                    rec.SetValue(Canvas.LeftProperty, (positions[p] * matrix).X);
                    rec.Fill = new SolidColorBrush(Colors.Black); 
                    canvas.Children.Add(rec);   
                }


                List<Point> boudsLocations = new List<Point>();
                List<Point> boudsSizes = new List<Point>();
                boudsLocations.Add(new Point(-100, 0)); boudsSizes.Add(new Point(4,  1200));
                boudsLocations.Add(new Point(-600, 400)); boudsSizes.Add(new Point(5,  400));
                boudsLocations.Add(new Point(-600, -400)); boudsSizes.Add(new Point(5,  400));
                boudsLocations.Add(new Point(-350, 600)); boudsSizes.Add(new Point(500,  5));
                boudsLocations.Add(new Point(-350, -600)); boudsSizes.Add(new Point(500,  5));
                for (int i = 0; i < boudsLocations.Count; i++)
                {

                    Point pos = new Point();
                    pos.X = boudsLocations[i].X - 0.5 * boudsSizes[i].X;
                    pos.Y = boudsLocations[i].Y - 0.5 * boudsSizes[i].Y;
                    Rectangle line = new Rectangle();
                    line.Width = boudsSizes[i].X * scale;
                    line.Height = boudsSizes[i].Y * scale;
                    line.SetValue(Canvas.TopProperty, (pos * matrix).Y);
                    line.SetValue(Canvas.LeftProperty, (pos * matrix).X);
                    if (i > 0)
                    {
                        line.Fill = new SolidColorBrush(Colors.Black);
                    }
                    else
                    {
                        line.Fill = new SolidColorBrush(Colors.DarkGray);
                    }
                    canvas.Children.Add(line);
                }
                Point cakepos = new Point(-600 -100, -100); 
                Ellipse cake = new Ellipse();
                cake.Width = 200 * scale;
                cake.Height = 200 * scale;
                cake.SetValue(Canvas.TopProperty, (cakepos * matrix).Y);
                cake.SetValue(Canvas.LeftProperty, (cakepos * matrix).X);
                cake.Fill = new SolidColorBrush(Colors.LightPink);
                canvas.Children.Add(cake); 

            }

            Image topview = new Image();
            topview.Source = bitmapImage;
            topview.Width = bitmapImage.Width * scale;
            topview.Height = bitmapImage.Height * scale;
            Point cornerVertex = new Point(drawing_translate.X, drawing_translate.Y) * matrix;
            Canvas.SetLeft(topview, cornerVertex.X);
            Canvas.SetTop(topview, cornerVertex.Y);
            //Console.Write(" top x = " + ((drawing_translate.X * scale) + translate.X));
            //Console.Write(" drawing_translate.X = " + drawing_translate.X);
            //Console.Write("  scale= " + scale);
            //Console.Write(" translate.X = " + translate.X);
            //Console.Write(" top y = " + ((drawing_translate.Y * scale) + translate.Y));
            //Console.Write(" drawing_translate.y = " + drawing_translate.Y);
            //Console.Write("  scale= " + scale);
            //Console.Write(" translate.y = " + translate.Y);
            canvas.Children.Add(topview);




            int count = -1;
            foreach (var path in paths)
            {
                List<Point> transformedPoints = new List<Point>();
                transformedPoints.Clear(); 
                foreach (var point in path.points)
                {
                    transformedPoints.Add(point * matrix);
                }
                if (transformedPoints.Count > 1)
                {
                    //Polygon topViewPoly = new Polygon();
                    //foreach (var point in topView.points)
                    //{
                    //   topViewPoly.Points.Add(point * matrix);
                    //}
                    //topViewPoly.Fill = new SolidColorBrush(Color.FromArgb(255, 100, 100, 100));
                    //canvas.Children.Add(topViewPoly);              
                    for (int i = 1; i < transformedPoints.Count; i++)
                    {
                        Line line = new Line();
                        line.X1 = transformedPoints[i - 1].X;
                        line.Y1 = transformedPoints[i - 1].Y;
                        line.X2 = transformedPoints[i].X;
                        line.Y2 = transformedPoints[i].Y;
                        if (count >= 0)
                        {
                            line.Stroke = uiStates.niceColors[count];
                        
                        }
                        else
                        {

                            /*
                            int refID = (int) Math.Floor(((double)(i-1) )/100.0 );
                            if (refID < 0) refID = 0;
                            int colorId = -1;
                            if (refID < gaitSequence.Count)
                            {
                                colorId = gaitSequence[refID];
                            } 
                            Console.WriteLine("trandformed oitn = " + transformedPoints.Count ); 
                            Console.WriteLine("i = " + i + " refId = " + refID + " gait = " + colorId); 
                            if (colorId >= 0 ){
                                line.Stroke = uiStates.niceColors[colorId];  //new SolidColorBrush(Colors.Black); 
                            }else{
                                line.Stroke = new SolidColorBrush(Colors.Gray); 

                            }
                             */
                            line.Stroke = new SolidColorBrush(Colors.Black); 
                              
                             
                        }
                        line.StrokeThickness = 5;
                        //if(count < 0){
                        canvas.Children.Add(line);
                        //}
                    }

                     if (count >= 0)
                    {
                        double rotAngle = rotationAngles[count];
                        Polygon Arrow = new Polygon();
                        double wa = 3.5 *2;
                        double wl = 2.5 * 2;
                        double hl = 14 * 2;
                        double ha = 7 * 2;

                        Matrix arrowMatrix = new Matrix();
                        arrowMatrix.Rotate(rotAngle + 90);
                        Point arrowCenter = transformedPoints[transformedPoints.Count - 1];
                        arrowMatrix.Translate(arrowCenter.X, arrowCenter.Y);                        
                        Arrow.Points.Add(new Point(-wl, 0) *arrowMatrix);
                        Arrow.Points.Add(new Point(-wl, hl)*arrowMatrix);
                        Arrow.Points.Add(new Point(- (wa + wl), hl)*arrowMatrix);
                        Arrow.Points.Add(new Point(0, hl + ha)*arrowMatrix);
                        Arrow.Points.Add(new Point(wa + wl, hl)*arrowMatrix);
                        Arrow.Points.Add(new Point(wl, hl)*arrowMatrix);
                        Arrow.Points.Add(new Point(wl, 0)*arrowMatrix);
                        Arrow.Fill = uiStates.niceColors[count];
                        canvas.Children.Add(Arrow);
                }

                }
                count++; 

            }
      }
    }
}
