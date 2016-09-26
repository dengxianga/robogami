using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using System.Windows.Controls;

namespace FBE_CSharpUI
{
    class TranslatePartGizmo
    {
        private TranslatePartVisual3D x, y;
        private TranslationVisual3D x1, y1, x2, y2;
        private ContainerUIElement3D containerX, containerY, containerX1, containerY1, containerX2, containerY2;
        private static readonly Vector3D[] AXES = { new Vector3D(1, 0, 0), new Vector3D(0, 1, 0), new Vector3D(0, 0, 1) };
        private int whichAxis = 0;
        private Point lastPosition;
        private Viewport3D viewport;
        private Vector3D xAxis = AXES[0];
        private Vector3D yAxis = AXES[1];
        private Point3D center;
        private Point screenCenter;


        public event Action<ScaleTransform> Scaled;

        public TranslatePartGizmo(Viewport3D viewport)
        {
            this.viewport = viewport;
            x = new TranslatePartVisual3D();
            y = new TranslatePartVisual3D();
            x1 = new TranslationVisual3D(new Point3D(0, 0, 0), new Vector3D(1, 0, 0));
            y1 = new TranslationVisual3D(new Point3D(0, 0, 0), new Vector3D(1, 0, 0));
            x2 = new TranslationVisual3D(new Point3D(0, 0, 0), new Vector3D(-1, 0, 0));
            y2 = new TranslationVisual3D(new Point3D(0, 0, 0), new Vector3D(-1, 0, 0));

            containerX = new ContainerUIElement3D();
            containerX.Children.Add(x);
            containerY = new ContainerUIElement3D();
            containerY.Children.Add(y);

            containerX1 = new ContainerUIElement3D();
            containerY1 = new ContainerUIElement3D();
            containerX1.Children.Add(x1);
            containerY1.Children.Add(y1);

            containerX2 = new ContainerUIElement3D();
            containerY2 = new ContainerUIElement3D();
            containerX2.Children.Add(x2);
            containerY2.Children.Add(y2);




            x.Material = new DiffuseMaterial(Brushes.LightCoral.MakeTransparent(0.6));
            y.Material = new DiffuseMaterial(Brushes.LightSkyBlue.MakeTransparent(0.6));
            x1.Material = new DiffuseMaterial(Brushes.Red);
            y1.Material = new DiffuseMaterial(Brushes.Blue);
            x2.Material = new DiffuseMaterial(Brushes.Red);
            y2.Material = new DiffuseMaterial(Brushes.Blue);

            var scales = new[] { x, y };
            var containers = new[] { containerX, containerY };
            for (int i = 0; i < 2; i++)
            {
                int iCopy = i;
                var scale = scales[i];
                var container = containers[i];
                var curMat = scale.Material;
                container.MouseEnter += (sender, args) =>
                {
                    scale.Material = new DiffuseMaterial(Brushes.Yellow);
                };
                container.MouseLeave += (sender, args) =>
                {
                    scale.Material = curMat;
                };
                container.MouseDown += (sender, args) =>
                {
                    Mouse.Capture(container);
                    whichAxis = iCopy + 1;
                    lastPosition = args.GetPosition(container);
                };
                container.MouseMove += (sender, args) =>
                {
                    if (whichAxis != 0)
                    {
                        Point curPosition = args.GetPosition(container);
                        //  double dy = Math.Pow(2,(curPosition.Y - lastPosition.Y)/200);
                        //  double dx = Math.Pow(2, (curPosition.X - lastPosition.X)/200);

                        Point screenCenter = viewport.Point3DtoPoint2D(center);
                        Viewport3DHelper.Point3DtoPoint2D(viewport, center);
                        Vector sX = viewport.Point3DtoPoint2D(center + xAxis) - screenCenter;
                        Vector sY = viewport.Point3DtoPoint2D(center + yAxis) - screenCenter;

                        ScaleTransform scaling = null;
                        if (whichAxis == 1)
                        {
                            //scale along x
                            Vector offset = curPosition - lastPosition;
                            double proj = sX * offset / sX.Length;
                            scaling = new ScaleTransform(0.4*proj, 0);

                        }
                        else if (whichAxis == 2)
                        {
                            //scale along y
                            Vector offset = curPosition - lastPosition;
                            double proj = sY * offset / sY.Length;
                            scaling = new ScaleTransform(0, 0.4 * proj);
                        }


                        if (Scaled != null)
                        {
                            Scaled(scaling);
                        }

                        lastPosition = curPosition;
                    }
                };
                container.MouseUp += (sender, args) =>
                {
                    Mouse.Capture(null);
                    whichAxis = 0;
                };

            }
        }

        public void Update(Point3D center, Vector3D xAxis, Vector3D yAxis, double xStabilityDir, double yStabilityDir)
        {
            this.center = center;
            this.xAxis = xAxis;
            this.yAxis = yAxis;
            // Console.WriteLine("xAxis = " + xAxis);
            // Console.WriteLine("yAxis = " + yAxis);
            Matrix3D mat = new Matrix3D();
            Vector3D zAxis = Vector3D.CrossProduct(xAxis, yAxis);
            mat.M11 = xAxis.X;
            mat.M21 = yAxis.X;
            mat.M31 = zAxis.X;
            mat.M12 = xAxis.Y;
            mat.M22 = yAxis.Y;
            mat.M32 = zAxis.Y;
            mat.M13 = xAxis.Z;
            mat.M23 = yAxis.Z;
            mat.M33 = zAxis.Z;
            mat.OffsetX = center.X;
            mat.OffsetY = center.Y;
            mat.OffsetZ = center.Z;

            Matrix3D yRotate = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), 90)).Value;

            double yScaleVal = 3* Math.Max(Math.Min(Math.Abs(yStabilityDir) * 10, 1.0), 0.3);
            double xScaleVal = 3 * Math.Max(Math.Min(Math.Abs(xStabilityDir) * 10, 1.0), 0.3);

            //Matrix3D yTransMatrix3D = Matrix3D.Multiply(yRotate, mat);
            //Matrix3D yTransMatrix3D1 = yTransMatrix3D;
            Matrix3D yScale = new ScaleTransform3D(new Vector3D(yScaleVal, yScaleVal, yScaleVal)).Value;
            Matrix3D xScale = new ScaleTransform3D(new Vector3D(xScaleVal, xScaleVal, xScaleVal)).Value;

            Matrix3D xScale_center = Matrix3D.Multiply(Matrix3D.Multiply((new TranslateTransform3D(-30, 0, -3)).Value, xScale), (new TranslateTransform3D(30, 0, 3)).Value);
            Matrix3D yScale_center = Matrix3D.Multiply(Matrix3D.Multiply((new TranslateTransform3D(-30, 0, -3)).Value, yScale), (new TranslateTransform3D(30, 0, 3)).Value);
            Matrix3D yTransMatrix3DArrow = Matrix3D.Multiply(yScale_center, Matrix3D.Multiply(yRotate, mat));
            Matrix3D xTransMatrix3DArrow = Matrix3D.Multiply(xScale_center, mat);



            Matrix3D yTransMatrix3D = Matrix3D.Multiply(yRotate, mat);
            Matrix3D xTransMatrix3D = mat;


            containerY.Transform = new MatrixTransform3D(yTransMatrix3D);
            containerX.Transform = new MatrixTransform3D(xTransMatrix3D);
            containerY1.Transform = new MatrixTransform3D(yTransMatrix3DArrow);
            containerX1.Transform = new MatrixTransform3D(xTransMatrix3DArrow);
            containerY2.Transform = new MatrixTransform3D(yTransMatrix3DArrow);
            containerX2.Transform = new MatrixTransform3D(xTransMatrix3DArrow);


            const double epsilon = 0;
            if (Math.Abs(xStabilityDir) <= epsilon)
            {
                setArrowVisible(containerX1, false);
                setArrowVisible(containerX2, false);
            }
            else if (xStabilityDir > epsilon)
            {
                //turn off positive pointing arrow
                // turn on negative pointing arrow
                setArrowVisible(containerX1, false);
                setArrowVisible(containerX2, true);
            }
            else
            {
                setArrowVisible(containerX1, true);
                setArrowVisible(containerX2, false);
            }

            // For y direction gizmo
            if (Math.Abs(yStabilityDir) <= epsilon)
            {
                setArrowVisible(containerY1, false);
                setArrowVisible(containerY2, false);
            }
            else if (yStabilityDir > epsilon)
            {
                //turn off positive pointing arrow
                // turn on negative pointing arrow
                setArrowVisible(containerY1, false);
                setArrowVisible(containerY2, true);
            }
            else
            {
                setArrowVisible(containerY1, true);
                setArrowVisible(containerY2, false);
            }

        }

        public void SetVisible(bool visible)
        {
            containerX.Visibility = visible ? Visibility.Visible : Visibility.Collapsed;
            containerY.Visibility = visible ? Visibility.Visible : Visibility.Collapsed;

        }

        public void setArrowVisible(UIElement3D arrowContainer, bool visible)
        {
            if (visible)
            {
                arrowContainer.Visibility = Visibility.Visible;
            }
            else
            {
                arrowContainer.Visibility = Visibility.Collapsed;
            }

        }

        public void setAllArrowsVisible(bool visible)
        {
            if (visible)
            {
                containerX1.Visibility = Visibility.Visible;
                containerY1.Visibility = Visibility.Visible;
                containerX2.Visibility = Visibility.Visible;
                containerY2.Visibility = Visibility.Visible;
            }
            else
            {
                containerX1.Visibility = Visibility.Collapsed;
                containerY1.Visibility = Visibility.Collapsed;
                containerX2.Visibility = Visibility.Collapsed;
                containerY2.Visibility = Visibility.Collapsed;
            }

        }

        public void AddToViewport(Viewport3D viewport)
        {
            viewport.Children.Add(containerX);
            viewport.Children.Add(containerY);
            viewport.Children.Add(containerX1);
            viewport.Children.Add(containerY1);
            viewport.Children.Add(containerX2);
            viewport.Children.Add(containerY2);
        }
    }
}
