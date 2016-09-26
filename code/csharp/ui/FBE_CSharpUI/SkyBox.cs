using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Policy;
using System.Text;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;

namespace FBE_CSharpUI
{
    public class SkyBox : ModelVisual3D
    {
        #region Fields

        private ScaleTransform3D scale;
        private static string defaultPath = "..\\..\\code\\csharp\\ui\\FBE_CSharpUI\\Resources\\";
        //private static string defaultPath = "..\\Robogami\\Resources\\";

        #endregion

        #region Dependency Properties

        public double Size
        {
            get { return (double)GetValue(SizeProperty); }
            set { SetValue(SizeProperty, value); }
        }

        public static readonly DependencyProperty SizeProperty =
        DependencyProperty.Register("Size", typeof(double),
        typeof(SkyBox),
        new FrameworkPropertyMetadata(OnSizeChanged)
        );

        private static void OnSizeChanged(DependencyObject sender,
        DependencyPropertyChangedEventArgs e)
        {
            SkyBox sb = sender as SkyBox;

            double size = sb.Size;

            sb.scale.ScaleX = size;
            sb.scale.ScaleY = size;
            sb.scale.ScaleZ = size;
        }

        #endregion

        #region Constructor

        public SkyBox()
        {
            
            Model3DGroup sides = new Model3DGroup();

            Point3D[] p = new Point3D[] {
                new Point3D(-1, 1, -1),
                new Point3D(-1, -1, -1),
                new Point3D(1, -1, -1),
                new Point3D(1, 1, -1),
                new Point3D(1, 1, 1),
                new Point3D(1, -1, 1),
                new Point3D(-1, -1, 1),
                new Point3D(-1, 1, 1)
};

            Int32Collection triangleIndices = new Int32Collection(
            new int[] { 0, 1, 2, 2, 3, 0 });

            PointCollection textCoords = new PointCollection(
            new Point[] {
                new Point(0,0),
                new Point(0,1),
                new Point(1,1),
                new Point(1,0)
});

            MeshGeometry3D quad = new MeshGeometry3D();
            quad.Positions.Add(p[0]);
            quad.Positions.Add(p[1]);
            quad.Positions.Add(p[2]);
            quad.Positions.Add(p[3]);
            quad.TriangleIndices = triangleIndices;
            quad.TextureCoordinates = textCoords;
           // sides.Children.Add(new GeometryModel3D(quad, GetSideMaterial("north")));  
            sides.Children.Add(new GeometryModel3D(quad, GetSideMaterial("bgroundSimple")));

            quad = new MeshGeometry3D();
            quad.Positions.Add(p[4]);
            quad.Positions.Add(p[5]);
            quad.Positions.Add(p[6]);
            quad.Positions.Add(p[7]);
            quad.TriangleIndices = triangleIndices;
            quad.TextureCoordinates = textCoords;
            //sides.Children.Add(new GeometryModel3D(quad, GetSideMaterial("south")));
            sides.Children.Add(new GeometryModel3D(quad, GetSideMaterial("bgroundSimple")));

            quad = new MeshGeometry3D();
            quad.Positions.Add(p[1]);
            quad.Positions.Add(p[6]);
            quad.Positions.Add(p[5]);
            quad.Positions.Add(p[2]);
            quad.TriangleIndices = triangleIndices;
            quad.TextureCoordinates = textCoords;
            //sides.Children.Add(new GeometryModel3D(quad, GetSideMaterial("down")));
            sides.Children.Add(new GeometryModel3D(quad, GetSideMaterial("bgroundSimple")));

            quad = new MeshGeometry3D();
            quad.Positions.Add(p[7]);
            quad.Positions.Add(p[6]);
            quad.Positions.Add(p[1]);
            quad.Positions.Add(p[0]);
            quad.TriangleIndices = triangleIndices;
            quad.TextureCoordinates = textCoords;
            //sides.Children.Add(new GeometryModel3D(quad, GetSideMaterial("west")));
            sides.Children.Add(new GeometryModel3D(quad, GetSideMaterial("bgroundSimple")));

            quad = new MeshGeometry3D();
            quad.Positions.Add(p[3]);
            quad.Positions.Add(p[2]);
            quad.Positions.Add(p[5]);
            quad.Positions.Add(p[4]);
            quad.TriangleIndices = triangleIndices;
            quad.TextureCoordinates = textCoords;
            //sides.Children.Add(new GeometryModel3D(quad, GetSideMaterial("east")));
            sides.Children.Add(new GeometryModel3D(quad, GetSideMaterial("bgroundSimple"))); //s2

            quad = new MeshGeometry3D();
            quad.Positions.Add(p[7]);
            quad.Positions.Add(p[0]);
            quad.Positions.Add(p[3]);
            quad.Positions.Add(p[4]);
            quad.TriangleIndices = triangleIndices;
            quad.TextureCoordinates = textCoords;
           // sides.Children.Add(new GeometryModel3D(quad, GetSideMaterial("up")));
            sides.Children.Add(new GeometryModel3D(quad, GetSideMaterial("bgroundSimple")));


            this.scale = new ScaleTransform3D(1, 1, 1);
            this.Transform = this.scale;
            this.Content = sides;
        }

        private Material GetSideMaterial(string sideFilename)
        {
            /*ImageBrush ib = new ImageBrush(
                new BitmapImage(new Uri("Resources\\" + sideFilename + ".jpg", UriKind.Relative)
            ));*/
            ImageBrush ib = new ImageBrush(
                new BitmapImage(
                    new Uri(SkyBox.defaultPath + sideFilename + ".jpg", UriKind.Relative
                    )
                )
            );

            ib.ViewportUnits = BrushMappingMode.Absolute;
            ib.TileMode = TileMode.None;
            MaterialGroup group = new MaterialGroup();
            group.Children.Add(new DiffuseMaterial(Brushes.Black));
            group.Children.Add(new EmissiveMaterial(ib));
            return group;
        }

        #endregion
    }
}
