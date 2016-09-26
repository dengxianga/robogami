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
    class GaitSequence
    {
        static Brush polygonFill = new SolidColorBrush(Color.FromArgb(37, 255, 0, 0));
        static Brush selectedFill = new SolidColorBrush(Color.FromArgb(172, 60, 227, 53));
        static Brush hoverFill = new SolidColorBrush(Color.FromArgb(174, 223, 230, 56));



        public static void drawTopView(BitmapImage bitmapImage, Vector drawing_translate, Vector drawing_size, Canvas mainCanvas)
        {


            mainCanvas.Children.Clear();

            //finding bounding box
            double xmin, xmax, ymin, ymax;
            List<double> Xs = new List<double>();
            List<double> Ys = new List<double>();
            Xs.Add(drawing_translate.X);
            Xs.Add(drawing_translate.X + drawing_size.X);
            Ys.Add(drawing_translate.Y);
            Ys.Add(drawing_translate.Y + drawing_size.Y);

            if (Xs.Count == 0 || Ys.Count == 0) return;
            xmin = Xs.Min();
            xmax = Xs.Max();
            ymin = Ys.Min();
            ymax = Ys.Max();

            double width = (mainCanvas.ActualWidth);
            double height = mainCanvas.ActualHeight;
            double marginRatio = 0.1;
            double scale = Math.Min(width * (1 - marginRatio) / (xmax - xmin), height * (1 - marginRatio) / (ymax - ymin));
            Vector pCenter = 0.5 * new Vector(xmax + xmin, ymax + ymin);
            Vector center = 0.5 * new Vector(width + 40, height + 40);
            Vector translate = center - pCenter;
            Matrix matrix = Matrix.Identity;
            matrix.Translate(-pCenter.X, -pCenter.Y);
            matrix.Scale(scale, scale);
            matrix.Translate(pCenter.X, pCenter.Y);

            matrix.Translate(translate.X, translate.Y);

            Rectangle rec = new Rectangle();
            rec.Width = 100;
            rec.Height = 100;
            double xMargin = 20;
            double yMargin = 20;
            rec.SetValue(Canvas.LeftProperty, xMargin);
            rec.SetValue(Canvas.TopProperty, yMargin);
            rec.Fill = new SolidColorBrush(Colors.Plum);
            mainCanvas.Children.Add(rec); 

            Color simpleBlack = Color.FromArgb(255, 80, 80, 80);



            int nStride = (bitmapImage.PixelWidth * bitmapImage.Format.BitsPerPixel + 7) / 8;
            byte[] pixelByteArray = new byte[bitmapImage.PixelHeight * nStride];
            bitmapImage.CopyPixels(pixelByteArray, nStride, 0);

            for (int i = 0; i < bitmapImage.PixelWidth*bitmapImage.PixelHeight; i++)
            {
                pixelByteArray[0 + i] = pixelByteArray[0 + i];
                pixelByteArray[1 + i] = 255;
            }

            WriteableBitmap colorBitmap = new WriteableBitmap(bitmapImage);
            colorBitmap.WritePixels(new Int32Rect(0, 0,
         bitmapImage.PixelWidth, bitmapImage.PixelHeight), pixelByteArray, nStride, 0); 
           
 

            Image topview = new Image();
            topview.Source = colorBitmap;
            topview.Width = bitmapImage.Width * scale;
            topview.Height = bitmapImage.Height * scale;
            Point cornerVertex = new Point(drawing_translate.X, drawing_translate.Y) * matrix;
            Canvas.SetLeft(topview, cornerVertex.X);
            Canvas.SetTop(topview, cornerVertex.Y);
            mainCanvas.Children.Add(topview);


        }




        public static void drawGaitOptions(UIStates uistates, List<Point> jointLocations, 
            List<List<int>> labelsList, List<List<double>> angleList,  Canvas mainCanvas, BitmapImage bitmapImage, Vector drawing_translate, Vector drawing_size)
        {


            if (mainCanvas.ActualWidth > 0)
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

                double width = (mainCanvas.ActualWidth) / labelsList.Count - 4;
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
                //foreach (var point in jointChoices.Layout.RobotShape)
                //{
                //   robotPolygon.Points.Add(point * matrix);
                //}
                //robotPolygon.Fill = Brushes.Azure;
                //robotPolygon.Stroke = Brushes.DarkSlateGray;
                //mainCanvas.Children.Add(robotPolygon);


                Color simpleBlack = Color.FromArgb(255, 100, 100, 100);
                Color hightlightColor = Colors.LightBlue;
                Color darkWhite =   Color.FromRgb(210, 210, 210); 
                for (int labelId = 0; labelId < labelsList.Count; labelId++)
                {
                    int labelCopy = labelId;
                    Rectangle rec = new Rectangle();
                    rec.Width = width;
                    rec.Height = height;
                    double xMargin =  (2 + labelId * (width + 4));
                    double yMargin = 2; 
                    rec.SetValue(Canvas.LeftProperty, xMargin);
                    rec.SetValue(Canvas.TopProperty, yMargin);
                    rec.Fill = uistates.niceColors[labelId];
                    rec.MouseDown += (sender, args) =>
                    {
                        DataObject data = new DataObject("gaitOption", labelCopy);
                        DragDrop.DoDragDrop(rec, data, DragDropEffects.All);
                    };

                    //mainCanvas.Children.Add(rec);




                    Image topview = getImageFromBitMap(bitmapImage, uistates.niceColorsNoBrush[labelId]);
                    topview.Width = bitmapImage.Width * scale;
                    topview.Height = bitmapImage.Height * scale;
                    Point cornerVertex = new Point(drawing_translate.X, drawing_translate.Y) * matrix;
                    Canvas.SetLeft(topview, cornerVertex.X);
                    Canvas.SetTop(topview, cornerVertex.Y);

                    mainCanvas.Children.Add(topview);


                    topview.MouseDown += (sender, args) =>
                    {
                        DataObject data = new DataObject("gaitOption", labelCopy);
                        DragDrop.DoDragDrop(rec, data, DragDropEffects.All);
                    };

                    for (int i = 0; i < jointLocations.Count; i++)
                    {
                        var iCopy = i;
                        Point joint = jointLocations[i];
                        int label = labelsList[labelId][i];
                        Ellipse ellipse = new Ellipse();
                        ellipse.Width = 20;
                        ellipse.Height = 20;
                        ellipse.Fill = new SolidColorBrush(darkWhite);
                        ellipse.Stroke = null;//  new RadialGradientBrush(Colors.Black, Colors.White);
                        ellipse.StrokeThickness = 5; 
                        ellipse.SetValue(Canvas.TopProperty, (joint * matrix).Y - 10);
                        ellipse.SetValue(Canvas.LeftProperty, (joint * matrix).X - 10);
                        mainCanvas.Children.Add(ellipse);


                                                    
     

                        double angle = angleList[labelId][i];
                        if(angle != 0){
                             drawOrientationArrow(mainCanvas, (joint * matrix), (angle < 0)); 
                        }
                        



                  

                        Label text = new Label();
                        text.Content = label;
                        text.HorizontalContentAlignment = HorizontalAlignment.Center;
                        text.VerticalContentAlignment = VerticalAlignment.Center;
                        text.FontSize = 15;
                        text.Foreground = new SolidColorBrush(Colors.Black);
                        //text.FontStyle = System.Drawing.FontStyle.Bold; 
                        text.Width = 20;
                        text.Height = 20;
                        text.Padding = new Thickness(0);
                        text.SetValue(Canvas.TopProperty, (joint * matrix).Y - 10);
                        text.SetValue(Canvas.LeftProperty, (joint * matrix).X - 10);
                        if (label != 0)
                             mainCanvas.Children.Add(text);
                    }
                    matrix.Translate((width + 4), 0);

                }

            }
        }


        public static Image getImageFromBitMap( BitmapImage bitmapImage, Color color){

            
                    int nStride = (bitmapImage.PixelWidth * bitmapImage.Format.BitsPerPixel + 7) / 8;
                    byte[] pixelByteArray = new byte[bitmapImage.PixelHeight * nStride];
                    bitmapImage.CopyPixels(pixelByteArray, nStride, 0);

                    for (int i = 0; i < bitmapImage.PixelWidth * bitmapImage.PixelHeight; i++)
                    {

                        if (pixelByteArray[0 + 4 * i] > 0)
                        {
                            pixelByteArray[0 + 4 * i] = color.B; // pixelByteArray[0 + 4 * i];                        
                            pixelByteArray[1 + 4 * i] = color.G; // uistates.niceColorsNoBrush[labelId + 1].R;
                            pixelByteArray[2 + 4 * i] = color.R; // uistates.niceColorsNoBrush[labelId + 1].G;

                            pixelByteArray[3 + 4 * i] = 200; //alpha 
                        }

                    }

                    WriteableBitmap colorBitmap = new WriteableBitmap(bitmapImage);
                    colorBitmap.WritePixels(new Int32Rect(0, 0,
                 bitmapImage.PixelWidth, bitmapImage.PixelHeight), pixelByteArray, nStride, 0); 

                    Image topview =   new Image();
                    topview.Source = colorBitmap;
                    

                return topview;
           
 
        }

        public static void drawOrientationArrow(Canvas mainCanvas, Point pos , bool isClockWise){
            Path arc_path = new Path();
            PathGeometry pathGeometry = new PathGeometry();
            PathFigure pathFigure = new PathFigure();
            ArcSegment arcSegment = new ArcSegment();
            arcSegment.IsLargeArc = true;
            //Set start of arc
            int rad = 11;
            if (!isClockWise)
            {
                pathFigure.StartPoint = new Point(pos.X - rad, pos.Y);
                //set end point of arc.
                arcSegment.Point = new Point(pos.X, pos.Y - rad);
                arcSegment.SweepDirection = SweepDirection.Counterclockwise;

            }
            else
            {
                pathFigure.StartPoint = new Point(pos.X - rad, pos.Y);
                //set end point of arc.
                arcSegment.Point = new Point(pos.X, pos.Y + rad);
                arcSegment.SweepDirection = SweepDirection.Clockwise;

            }
            arcSegment.Size = new Size(rad, rad);
            pathFigure.Segments.Add(arcSegment);
            pathGeometry.Figures.Add(pathFigure);
            arc_path.Data = pathGeometry;
            arc_path.Stroke = new SolidColorBrush(Colors.Black);
            arc_path.StrokeThickness = 3;
            Polygon triangle = new Polygon();
            if (!isClockWise)
            {
                triangle.Points.Add(new Point(pos.X - 8, pos.Y - rad));
                triangle.Points.Add(new Point(pos.X, pos.Y - rad + 4));
                triangle.Points.Add(new Point(pos.X, pos.Y - rad - 4));
            }
            else
            {
                triangle.Points.Add(new Point(pos.X - 8, pos.Y + rad));
                triangle.Points.Add(new Point(pos.X, pos.Y + rad + 4));
                triangle.Points.Add(new Point(pos.X, pos.Y + rad - 4));
            }

            triangle.Fill = new SolidColorBrush(Colors.Black);
            mainCanvas.Children.Add(arc_path);
            mainCanvas.Children.Add(triangle); 
    }

        public static void drawSequence(Action<int> deleteElemetFromSequence, UIStates uistates, List<int> sequence, Canvas mainCanvas)
        {

           
       

            mainCanvas.Children.Clear();

            double width = (mainCanvas.ActualWidth);
            double height = mainCanvas.ActualHeight ;
            if (width > 0)
            {
                double boxHeight = 0.4 * height;
                double boxWidth = Math.Min(100, (0.9 * width) / sequence.Count);
                double marginWidth = 0.5 * (width - sequence.Count * boxWidth) - 1;
                double marginHeight = 0.3 * height;

                for (int i = 0; i < sequence.Count; i++)
                {
                    int iCopy = i; 
                    Rectangle rec = new Rectangle();
                    rec.Width = boxWidth - 2;
                    rec.Height = boxHeight;
                    rec.SetValue(Canvas.TopProperty, marginHeight);
                    rec.SetValue(Canvas.LeftProperty, marginWidth + i * boxWidth);
                    int colorId = sequence[i];
                    if (colorId < 0)
                    {
                        rec.Fill = new SolidColorBrush(Color.FromArgb(255, 190, 190, 190));
                    }
                    else
                    {
                        if (colorId >= uistates.niceColors.Count)
                        {
                            rec.Fill = new SolidColorBrush(Colors.Black);
                        }
                        else
                        {
                            rec.Fill = uistates.niceColors[colorId];
                        }
                    }


                    

                    ContextMenu cm = new ContextMenu();
                    MenuItem del = new MenuItem();
                    del.Header = "delete";
                    cm.Items.Add(del);
                    rec.ContextMenu = cm;
                    del.Click += (sender, e) => {

                        deleteElemetFromSequence(iCopy);
                        
                    };
                    mainCanvas.Children.Add(rec);
                }

            }           

        }
    }
}