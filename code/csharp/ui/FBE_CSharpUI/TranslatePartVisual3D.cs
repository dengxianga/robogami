using System;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

namespace FBE_CSharpUI
{
    class TranslatePartVisual3D : ModelVisual3D
    {
        private double _barLength;
        private double _barRadius;
        private double _sphereRadius;
        
        private SphereVisual3D ball;
        private ModelVisual3D bar;
        private Geometry3D barGeometry;
        private Material _material;

        public Material Material
        {
            get { return _material; }
            set { _material = value;
                UpdateModel();
            }
        }
        public TranslatePartVisual3D()
        {
            SetParameters(30, 2, 3);
        }

        public void SetParameters(double barLength, double barRadius, double sphereRadius)
        {
            _barLength = barLength;
            _barRadius = barRadius;
            _sphereRadius = sphereRadius;
            UpdateModel();
        }
        
        private void UpdateModel()
        {
            Children.Clear();

            ball = new SphereVisual3D() {Material=Material, Center=new Point3D(_barLength, 0, 0), Radius = _sphereRadius};

            MeshBuilder mb = new MeshBuilder();
            mb.AddCylinder(new Point3D(0, 0, 0), new Point3D(_barLength, 0, 0), _barRadius, 36);
            mb.AddCone(new Point3D(_barLength, 0, 0), new Point3D(_barRadius * 4 + _barLength, 0, 0), 2 * _barRadius, true, 10);
            mb.AddSphere(new Point3D(0, 0, 0), _barRadius / 2);
            barGeometry = mb.ToMesh(true);
            bar = new ModelVisual3D() {Content = new GeometryModel3D(barGeometry, Material)};

            //Children.Add(ball);
            Children.Add(bar);
            //TranslationVisual3D arrow = new TranslationVisual3D();
            
        }
    }
}