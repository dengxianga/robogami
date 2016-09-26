using System;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

namespace FBE_CSharpUI
{
    class TorusVisual3D : ParametricSurface3D
    {
        private double _innerRadius;
        private double _outerRadius;

        public TorusVisual3D(double innerRadius, double outerRadius)
        {
            _innerRadius = innerRadius;
            _outerRadius = outerRadius;
            this.MeshSizeU = 50;
            this.MeshSizeV = 20;
        }

        public void SetRadii(double innerRadius, double outerRadius)
        {
            _innerRadius = innerRadius;
            _outerRadius = outerRadius;
            UpdateModel();
        }


        protected override Point3D Evaluate(double u, double v, out System.Windows.Point textureCoord)
        {
            textureCoord = new System.Windows.Point(0, 0);
            return Torus(u * Math.PI * 2, v * Math.PI * 2);
        }
        private Point3D Torus(double u, double v)
        {
            double R = (_innerRadius + _outerRadius)/2;
            double r = (_outerRadius - _innerRadius) / 2;
            double x = (R + r * Math.Cos(v)) * Math.Cos(u);
            double z = (R + r * Math.Cos(v)) * Math.Sin(u);
            double y = r * Math.Sin(v);
            return new Point3D(x, y, z);
        }
    }
}