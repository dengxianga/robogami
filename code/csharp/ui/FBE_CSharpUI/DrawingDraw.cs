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
using Brushes = System.Windows.Media.Brushes;
using Color = System.Windows.Media.Color;
using Drawing = CppCsBridge.Drawing;

namespace FBE_CSharpUI
{
    class DrawingDraw
    {
        static Brush polygonFill = new SolidColorBrush(Color.FromArgb(37,255,0,0));
        static Brush selectedFill = new SolidColorBrush(Color.FromArgb(172,60,227,53));
        static Brush hoverFill = new SolidColorBrush(Color.FromArgb(174,223,230,56));

        public static void Draw(UIStates uiStates, Dictionary<TemplateRef, Drawing> drawings, Canvas canvas, TemplateRef selectedTemplate, Action<TemplateRef> selectionCallback) {
            canvas.Children.Clear();

            //finding bounding box
            double xmin, xmax, ymin, ymax;
            List<double> Xs = new List<double>();
            List<double> Ys = new List<double>();
            foreach (var drawing in drawings.Values) {
                foreach (var face in drawing.faces)
                {
                    foreach (var point in face.points)
                    {
                        Xs.Add(point.X);
                        Ys.Add(point.Y);
                    }
                }
            }
            if (Xs.Count == 0 || Ys.Count == 0) return;
            xmin = Xs.Min(); 
            xmax = Xs.Max(); 
            ymin = Ys.Min(); 
            ymax = Ys.Max();

            double width = canvas.ActualWidth;
            double height = canvas.ActualHeight;
            double marginRatio = 0.1;
            double scale = Math.Min(width * (1 - marginRatio) / (xmax - xmin), height * (1 - marginRatio) / (ymax - ymin));
            Vector pCenter = 0.5 * new Vector(xmax + xmin, ymax + ymin);
            Vector center = 0.5*new Vector(width, height);
            Vector translate = center - pCenter;
            Matrix matrix = Matrix.Identity;
            matrix.Translate(-pCenter.X, -pCenter.Y);
            matrix.Scale(scale,scale);
            matrix.Translate(pCenter.X,pCenter.Y);

            matrix.Translate(translate.X,translate.Y);
            //matrix.Scale(scale, scale);

            // adding polygons
            Dictionary<TemplateRef, Polygon> polygons = new Dictionary<TemplateRef, Polygon>();

            int seletedIndex=0;
            foreach (var kvp in drawings) {
                var template = kvp.Key;
                var drawing = kvp.Value;
                foreach (var face in drawing.faces)
                {
                    if (template.Equals(selectedTemplate))
                    {
                        seletedIndex = polygons.Count;
                    }
                    Polygon polygon = new Polygon();
                    foreach (var point in face.points)
                    {
                        polygon.Points.Add(point * matrix);
                    }
                    polygons[template] = polygon;
                    canvas.Children.Add(polygon);
                }
            }

            Vector vec = new Vector(0,0);
            foreach (var kvp in polygons) {
                TemplateRef template = kvp.Key;
                Polygon A = kvp.Value;

                bool intersect = false;
                foreach (var B in polygons.Values) {
                    if (A != B)
                    {
                        if (uiStates.ShowOverlaps && CollisionDetector.PolygonCollision(A, B, vec).Intersect == true)
                        {
                            intersect = true;
                        }
                    }
                }
                if (selectedTemplate.IsAncestorOf(template))
                {
                    A.Fill = selectedFill;
                    A.Stroke = Brushes.Black;
                }
                else if(intersect == true)
                {
                    A.Stroke = Brushes.Red;
                    A.StrokeThickness = 2;
                    A.Fill = polygonFill;
                }
                else
                {
                    A.Stroke = Brushes.Black;                   
                    A.Fill = Brushes.Linen;
                   
                }

                //adding mouse click event handler
                A.MouseDown += (sender, args) =>
                {
                    selectionCallback(template);
                };

                Brush old = null;
                A.MouseEnter += (sender, args) =>
                {                                
                    old = A.Fill;
                    A.Fill = hoverFill;

                };
                A.MouseLeave += (sender, args) =>
                {
                    A.Fill = old;
                };
            }
        }
    }
}
