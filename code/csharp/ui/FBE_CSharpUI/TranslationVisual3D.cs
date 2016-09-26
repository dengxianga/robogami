using System;
using System.Windows;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Xceed.Wpf.Toolkit.Core.Converters;

namespace FBE_CSharpUI
{
    class TranslationVisual3D:ModelVisual3D
    {
        private double _barLength;
        private double _barRadius;
        private double _coneRadius;
        private double _coneLength;
        private Vector3D _dir;
        private Point3D _center;

        private ModelVisual3D cone;
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
        public TranslationVisual3D(Point3D center, Vector3D dir)
        {
            SetParameters(5, 1, 3, 1.5, center, dir);
        }
        public TranslationVisual3D(double barLength, double barRadius, double coneLength, double coneRadius, Point3D center, Vector3D dir)
        {
            _center = center;
            _dir = dir;
            SetParameters(barLength, barRadius, coneLength, coneRadius, center, dir);

        }

        public void SetParameters(double barLength, double barRadius, double coneLength, double coneRadius, Point3D center, Vector3D dir)
        {
            _barLength = barLength;
            _barRadius = barRadius;
            _coneRadius = coneRadius;
            _coneLength = coneLength;
            _center = center;
            _dir = dir;
            UpdateModel();
        }

        private void UpdateModel()
        {
            Children.Clear();

            MeshBuilder mb = new MeshBuilder();
            //mb.AddCylinder(new Point3D(0, 0, 0), new Point3D(_barLength, 0, 0), _barRadius, 36);           
            //mb.AddSphere(new Point3D(0, 0, 0), _barRadius / 2);
            //mb.AddCone(new Point3D(_barLength,0,0), new Point3D(_barLength+_coneLength,0,0), _coneRadius,true,10);
            mb.AddCylinder(_center, _center + _barLength*_dir, _barRadius, 36);
            mb.AddSphere(new Point3D(0, 0, 0), _barRadius / 2);
            mb.AddCone(_center + _barLength * _dir, _center + (_barLength+_coneLength)*_dir, _coneRadius, true, 10);
            barGeometry = mb.ToMesh(true);
            bar = new ModelVisual3D() {Content = new GeometryModel3D(barGeometry, Material)};            
            Children.Add(bar);  
            this.Transform = new TranslateTransform3D(30,0,3);
        }
    }
}