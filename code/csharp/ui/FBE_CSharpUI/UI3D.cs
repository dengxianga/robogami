using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using HelixToolkit.Wpf;
using CppCsBridge;
using Brushes = System.Windows.Media.Brushes;
using Color = System.Windows.Media.Color;
using Point = System.Windows.Point;
using System.Diagnostics;

namespace FBE_CSharpUI
{
    public class UI3D
    {
        //image source note - 
        // background image - with trees
        // http://www.deluxevectors.com/vector/nature/vector-background-illustration-1.html  
        //


        #region OriginalMaterial and OriginalBackMaterial properties
        // These are generated using the attachedProperty macro.

        public static string defaultPath = "..\\..\\code\\csharp\\ui\\FBE_CSharpUI\\Resources\\";
        //public static string defaultPath = "..\\Robogami\\Resources\\";

        public static readonly DependencyProperty OriginalMaterialProperty = DependencyProperty.RegisterAttached(
            "OriginalMaterial", typeof (Material), typeof (UI3D), new PropertyMetadata(default(Material)));

        public static void SetOriginalMaterial(DependencyObject element, Material value) {
            element.SetValue(OriginalMaterialProperty, value);
        }

        public static Material GetOriginalMaterial(DependencyObject element) {
            return (Material) element.GetValue(OriginalMaterialProperty);
        }

        public static readonly DependencyProperty OriginalBackMaterialProperty = DependencyProperty.RegisterAttached(
            "OriginalBackMaterial", typeof (Material), typeof (UI3D), new PropertyMetadata(default(Material)));

        public static void SetOriginalBackMaterial(DependencyObject element, Material value) {
            element.SetValue(OriginalBackMaterialProperty, value);
        }

        public static Material GetOriginalBackMaterial(DependencyObject element) {
            return (Material) element.GetValue(OriginalBackMaterialProperty);
        }
        #endregion


        private static Material DefaultMaterial = new DiffuseMaterial(Brushes.LightGray);
        //static Brush ghostFill = new SolidColorBrush(Color.FromArgb(100, 223, 230, 56));
        static Brush ghostFill = new SolidColorBrush(Color.FromArgb(100, 130, 224, 255));
        
        private static Material GhostMaterial = new DiffuseMaterial(ghostFill);
        private static Material NormalMaterial;
        private static  Material SelectedMaterial = new DiffuseMaterial(Brushes.Gold);
        private static  Material SelectedBackMaterial = new DiffuseMaterial(Brushes.Gold);
        private static  Material HighlightMaterial = new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(255, 0, 178, 237)));
        private static  Material DebugMaterial = new DiffuseMaterial(Brushes.YellowGreen);
        private static Material InitialDraggingMaterial = new DiffuseMaterial(Brushes.PaleGoldenrod);
        private static  Material BackMaterial = new DiffuseMaterial(Brushes.LightGray);
        private static  Material FloorMaterial = new DiffuseMaterial(Brushes.LightGray);
        private static  Material NonStableMaterial = new DiffuseMaterial(Brushes.LightCoral);
        private static  Material NonStableBackMaterial = new DiffuseMaterial(Brushes.LightCoral);
        private static  Material StableMaterial = new DiffuseMaterial(Brushes.LightGreen);
        private static  Material StableBackMaterial = new DiffuseMaterial(Brushes.LightGreen);

        private static readonly Material UnselectedMaterial = new DiffuseMaterial(Brushes.LightGray);
            //new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(150, 165, 212, 212)));

        private RotationGizmo rotationGizmo;
        private ScaleGizmo scaleGizmo;
        private TextBlock dimensions;
        //private ScaleGizmo translatePartGizmo;
        private TranslatePartGizmo translatePartGizmo;
        private UIInstance uiInstance;
        private UIStates uiState;
        private Action updateCallback;
        private Action<bool> geoChangeCallBack;
        private Point? lastPosition;

        private HelixViewport3D viewport;
        private Viewport3D overlay;
        private Canvas overlay2D;
        private int drawnfoodItems;
        private Dictionary<TemplateRef, ModelUIElement3D> cachedUIElements =
            new Dictionary<TemplateRef, ModelUIElement3D>();

        private static readonly SolidColorBrush HightlightAddition = new SolidColorBrush(Color.FromRgb(75, 50, 0));

        public void initialize(HelixViewport3D viewport, Viewport3D overlay, Canvas overlay2D, UIInstance uiInstance, UIStates uiState, Action updateCallback, Action<bool> geoChangeCallBack)
        {
            drawnfoodItems = 0; 
            //picking a set of materials with graphical texture
            //HighlightMaterial = GetModelMaterial("gold4", 100);
            //DefaultMaterial = GetModelMaterial("plastic4",100); //plastic4
            InitialDraggingMaterial = DefaultMaterial;
            SelectedMaterial = GetModelMaterial("gold3",100);           
            //BackMaterial = GetModelMaterial("plastic6", 100);
            BackMaterial = DefaultMaterial;
            SelectedBackMaterial = SelectedMaterial;
            //StableMaterial = GetModelMaterial("greenPlastic", 100);
            StableBackMaterial = StableMaterial;
            //NonStableMaterial = GetModelMaterial("redPlastic", 100);
            NonStableBackMaterial = NonStableMaterial;
            ModelVisual3D floor = GetFloor("tile3", 0.05);  //fabric5 is good, 6 is better, 7 has patterns, 8 show tiles , 9 has patterns, old using tile2

            

            this.uiInstance = uiInstance;
            this.updateCallback = updateCallback;
            this.uiState = uiState;
            this.uiState.Selection = Selection.Empty;
            this.viewport = viewport;
            this.overlay = overlay;
            this.overlay2D = overlay2D;

            //scale gizmo hook handlers
            scaleGizmo = new ScaleGizmo(overlay);
            scaleGizmo.SetVisible(false);
            scaleGizmo.setAllArrowsVisible(false);
            scaleGizmo.AddToViewport(overlay);
            scaleGizmo.Update(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 1, 0), 0, 0);

            dimensions = new TextBlock();
            dimensions.Text = "0, 0";
            dimensions.TextAlignment = TextAlignment.Left;
            dimensions.FontSize = 12;
            Canvas.SetTop(dimensions, 2);
            Canvas.SetLeft(dimensions, 12);
            overlay2D.Children.Add(dimensions);
            dimensions.Visibility = System.Windows.Visibility.Hidden;

            //translatePart gizmo hook handlers
            translatePartGizmo = new TranslatePartGizmo(overlay);
            //translatePartGizmo = new ScaleGizmo(overlay);
            translatePartGizmo.SetVisible(false);
            translatePartGizmo.setAllArrowsVisible(false);
            translatePartGizmo.AddToViewport(overlay);
            translatePartGizmo.Update(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 1, 0), 0, 0);


            
            /*
            const double maxScalingPerStep = 2;

            // Scale the given amount in the given axis, but bound the scaling to the maximum scaling per step.
            // Return the amount of scaling left to do.
            Func<int, double, double> scaleWithBound = (axis, desiredAmount) => {
                Stopwatch sw1 = new Stopwatch();
                sw1.Start();                                                
                double scaleAmount = desiredAmount;
                if (desiredAmount == 1)
                {
                    return 1;
                }
                if (desiredAmount > maxScalingPerStep) {
                    scaleAmount = maxScalingPerStep;
                }
                else if (desiredAmount < 1/maxScalingPerStep) {
                    scaleAmount = 1/maxScalingPerStep;
                }

                if (uiState.SnappingState != null && uiState.SnappingState.PatchPair != null)
                {
                    Stopwatch sw = new Stopwatch();
                    sw.Start();
                    uiInstance.Scale(uiState.Selection.SelectedTemplate, axis, scaleAmount, uiState.SnappingState.PatchPair, uiState.PreventCollisions);
                    sw.Stop();
                    // Console.WriteLine("time taken for uiInstance.Scale is " + sw.Elapsed);
                }
                else
                {
                    Stopwatch sw = new Stopwatch();
                    sw.Start();
                    uiInstance.Scale(uiState.Selection.SelectedTemplate, axis, scaleAmount, null, uiState.PreventCollisions);
                    sw.Stop();
                    // Console.WriteLine("time taken for uiInstance.Scale is " + sw.Elapsed);
                }
                updateCallback();
                sw1.Stop();
                // Console.WriteLine("time for scalWithBound() is " + sw1.Elapsed);
                return desiredAmount/scaleAmount;
            };
            Func<int, double, double> scaleWithoutBound = (axis, desiredAmount) =>
            {
                Stopwatch sw1 = new Stopwatch();
                sw1.Start();
                double scaleAmount = desiredAmount;
                if (desiredAmount == 1)
                {
                    return 1;
                }
                if (uiState.SnappingState != null && uiState.SnappingState.PatchPair != null)
                {
                    Stopwatch sw = new Stopwatch();
                    sw.Start();
                    uiInstance.Scale(uiState.Selection.SelectedTemplate, axis, scaleAmount, uiState.SnappingState.PatchPair, uiState.PreventCollisions);
                    sw.Stop();
                    // Console.WriteLine("time taken for uiInstance.Scale is " + sw.Elapsed);
                }
                else
                {
                    Stopwatch sw = new Stopwatch();
                    sw.Start();
                    uiInstance.Scale(uiState.Selection.SelectedTemplate, axis, scaleAmount, null, uiState.PreventCollisions);
                    sw.Stop();
                    // Console.WriteLine("time taken for uiInstance.Scale is " + sw.Elapsed);
                }
                updateCallback();
                sw1.Stop();
                // Console.WriteLine("time for scalWithBound() is " + sw1.Elapsed);
                return desiredAmount / scaleAmount;
            };



            Func<int, double, double> TranslatePartWithBound = (axis, desiredAmount) =>
            {
                Stopwatch sw1 = new Stopwatch();
                sw1.Start();
                double scaleAmount = desiredAmount;
                if (desiredAmount == 0)
                {
                    return 0;
                }
                if (desiredAmount > maxScalingPerStep)
                {
                    scaleAmount = maxScalingPerStep;
                }
                else if (desiredAmount < - maxScalingPerStep)
                {
                    scaleAmount = -maxScalingPerStep;
                }

                Stopwatch sw = new Stopwatch();
                sw.Start();
                uiInstance.translatePart(uiState.Selection.SelectedTemplate, axis, scaleAmount);
                sw.Stop();
                // Console.WriteLine("time taken for uiInstance.Scale is " + sw.Elapsed);
                updateCallback();
                sw1.Stop();
                // Console.WriteLine("time for scalWithBound() is " + sw1.Elapsed);
                return desiredAmount -scaleAmount;
            };

            var remainingScale = new Point(1, 1);
            DispatcherTimer timerScale = new DispatcherTimer();
            timerScale.Tick += (sender, args) =>
            {
                Stopwatch sw = new Stopwatch();
                remainingScale.X = scaleWithBound(0, remainingScale.X);
                remainingScale.Y = scaleWithBound(1, remainingScale.Y);
                if (Math.Abs(remainingScale.X -1) < 1e-5 && Math.Abs(remainingScale.Y -1) < 1e-5) {
                    uiState.DisableExpensiveOperations = false;    
                    sw.Start();
                    // Console.WriteLine("time taken for each scaling with in a timer click is " + sw.Elapsed);
                    timerScale.Stop();
                    updateCallback();
                }
            };
            timerScale.Interval = new TimeSpan(0, 0, 0, 0, 1);

            DispatcherTimer timerTranlate = new DispatcherTimer();
            timerTranlate.Tick += (sender, args) =>
            {
                Stopwatch sw = new Stopwatch();
                remainingScale.X = TranslatePartWithBound(0, remainingScale.X);
                remainingScale.Y = TranslatePartWithBound(1, remainingScale.Y);
                if (Math.Abs(remainingScale.X) < 1e-5 && Math.Abs(remainingScale.Y) < 1e-5)
                {
                    uiState.DisableExpensiveOperations = false;
                    sw.Start();
                    // Console.WriteLine("time taken for each scaling with in a timer click is " + sw.Elapsed);
                    timerTranlate.Stop();
                    updateCallback();
                }
            };
            timerTranlate.Interval = new TimeSpan(0, 0, 0, 0, 1);
*/
            scaleGizmo.Scaled += (s) => {
                Stopwatch sw = new Stopwatch();
                sw.Start();
                if (uiState.SnappingState != null && uiState.SnappingState.PatchPair != null)
                {
                   uiInstance.Scale(uiState.Selection.SelectedTemplate, 0, s.ScaleX, uiState.SnappingState.PatchPair, uiState.PreventCollisions);
                   uiInstance.Scale(uiState.Selection.SelectedTemplate, 1, s.ScaleY, uiState.SnappingState.PatchPair, uiState.PreventCollisions);
                }
                else
                {
                    uiInstance.Scale(uiState.Selection.SelectedTemplate, 0, s.ScaleX, null, uiState.PreventCollisions);
                    uiInstance.Scale(uiState.Selection.SelectedTemplate, 1, s.ScaleY, null, uiState.PreventCollisions);               
                }
                geoChangeCallBack(false);
                SubElementFace? selectedFace = uiState.Selection.SelectedTemplate.getSubElementFace(0);
                dimensions.Text = string.Format("width: {0,7:N3}, height: {1,7:N3}", selectedFace.Value.width, selectedFace.Value.height);
                //timerScale.Start();
                sw.Stop();

                // Console.WriteLine("time for scaleGizmo.Scaled " + sw.Elapsed);
            };

            translatePartGizmo.Scaled += (s) =>
            {
                Stopwatch sw = new Stopwatch();
                sw.Start();
                if (uiState.SnappingState != null && uiState.SnappingState.PatchPair != null)
                {
                    uiInstance.translatePart(uiState.Selection.SelectedTemplate, 0, s.ScaleX);
                    uiInstance.translatePart(uiState.Selection.SelectedTemplate, 1, s.ScaleY);
                }
                else
                {
                    uiInstance.translatePart(uiState.Selection.SelectedTemplate, 0, s.ScaleX);
                    uiInstance.translatePart(uiState.Selection.SelectedTemplate, 1, s.ScaleY);
                }
                geoChangeCallBack(false); 
                //timerTranlate.Start();
                sw.Stop();

                // Console.WriteLine("time for scaleGizmo.Scaled " + sw.Elapsed);
            };

            //rotation gizmo hook handlers
            rotationGizmo = new RotationGizmo();
            rotationGizmo.SetVisible(false);
            rotationGizmo.AddToViewport(overlay);
            rotationGizmo.Update(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 1, 0),
                new Vector3D(0, 0, 1),
                viewport);
            rotationGizmo.Rotated += (r) =>
            {
                var rotation = r as AxisAngleRotation3D;
                if (uiState.SnappingState == null || uiState.SnappingState.PatchPair == null)
                {
                    uiInstance.Rotate(uiState.Selection.SelectedTemplate, new Quaternion(rotation.Axis, rotation.Angle),
                        uiState.Selection.SelectedTemplate.Center);
                }
                else
                {
                    var patchPair = uiState.SnappingState.PatchPair;
                    Point3D p1 = patchPair.MainTemplatePatch.Vertex1;
                    Point3D p2 = patchPair.MainTemplatePatch.Vertex2;
                    Point3D midP = ((p1.ToVector3D() + p2.ToVector3D()) / 2).ToPoint3D();
                    uiInstance.Rotate(uiState.Selection.SelectedTemplate, new Quaternion(rotation.Axis, rotation.Angle),
                        midP);
                }
                geoChangeCallBack(false);
                //uiInstance.updateMetrics(this.uiState.NAnimRounds, this.uiState.DeltaAnim);
            };

            //viewport.Children.Add(new GridLinesVisual3D {Normal = new Vector3D(0, 1, 0)});
            //make a plane at y=0 and add to viewport
            viewport.Children.Add(floor);
            SkyBox sb = new SkyBox();
            RoutedEventHandler cameraChangedHandler = (sender, args) => {
                var center = viewport.Camera.Position;
                Transform3DGroup gr = new Transform3DGroup();
                gr.Children.Add(new ScaleTransform3D(10000, 10000, 10000));
                gr.Children.Add(new TranslateTransform3D(center.ToVector3D()));
                sb.Transform = gr;
            };
            cameraChangedHandler(null, null);
            viewport.CameraChanged += cameraChangedHandler;
            viewport.Children.Add(sb);
            //viewport.Children.Add(new DefaultLights());
            viewport.Children.Add(new AwesomeLights(viewport));
            overlay.Children.Add(new DefaultLights());
        }

        private List<Vector3D> FindTwoPerpNorms(Vector3D v)
        {
            double threshold = 0.7;
            List<Vector3D> res = new List<Vector3D>();
            Vector3D x = new Vector3D(1, 0, 0);
            Vector3D y = new Vector3D(0, 1, 0);
            Vector3D n1, n2;

            if (Math.Abs(Vector3D.DotProduct(v, x)) < threshold)
            {
                res.Add(n1 = Vector3D.CrossProduct(x, v));
                res.Add(Vector3D.CrossProduct(v, n1));
                
            }
            else
            {
                res.Add(n2 = Vector3D.CrossProduct(y, v));
                res.Add(Vector3D.CrossProduct(v,n2));
                
            }
            return res;
        }

        // Draw using the last drawn meshes. Use only if you know what you're doing (which you probably don't).
        public void DrawLast() {
            Draw(uiState.LastDrawnMeshes, null);
        }

        public void Draw(List<Mesh> meshes, List<Mesh> ghostMeshes, List<bool> colors = null, bool isAnimating = false )
        {
            uiState.LastDrawnMeshes = meshes;
            uiState.TranslationCache.ClearElements();
            List<TemplateRef> closesetTemp = uiInstance.getClosesetElements();
            List<List<Patch3>> closesetPactches = uiInstance.getClosesetPatch();            
            bool isConnecting = false;
            if (closesetTemp.Count > 0)
            {
                isConnecting = true;
            }
            foreach (var mesh in meshes) {
                var tmplRoot = mesh.tmpl.Root;
                if (!uiState.TranslationCache.LowestZ.ContainsKey(tmplRoot)) {
                    uiState.TranslationCache.LowestZ[tmplRoot] = tmplRoot.LowestZ;
                }
            }
            if (uiState.ExportMesh) {
                uiState.ExportedMesh = new Dictionary<TemplateRef, Model3DGroup>();
            }
            HashSet<TemplateRef> drawnArticulations = new HashSet<TemplateRef>();
            if (colors != null && meshes.Count != colors.Count)
            {
                throw new Exception("not same size!!!!");
            }
            int lightsIndex = 0;
            for (int i = 0; i < viewport.Children.Count; i++)
            {
                if (viewport.Children[i] is LightSetup)
                {
                    lightsIndex = i;
                    break;
                }
            }


            if (uiState.AvoidObstacle &&  (drawnfoodItems ==0) )
            {
                
                // bounds
                List<Point3D> boudsLocations = new List<Point3D>();
                List<Point3D> boudsSizes = new List<Point3D>();
                boudsLocations.Add(new Point3D(-100,  2,   0)); boudsSizes.Add(new Point3D(4, 4, 1200));
                boudsLocations.Add(new Point3D(-600, 50, 400)); boudsSizes.Add(new Point3D(5, 100, 400));
                boudsLocations.Add(new Point3D(-600, 50,-400)); boudsSizes.Add(new Point3D(5, 100, 400));
                boudsLocations.Add(new Point3D(-350, 50, 600)); boudsSizes.Add(new Point3D(500, 100, 5));
                boudsLocations.Add(new Point3D(-350, 50,-600)); boudsSizes.Add(new Point3D(500, 100, 5));



                for(int i = 0; i< boudsLocations.Count; i++){
                    MeshBuilder mb = new MeshBuilder();
                    Point3D p = new Point3D(0, 0, 0);
                    mb.AddBox(boudsLocations[i], boudsSizes[i].X, boudsSizes[i].Y, boudsSizes[i].Z);

                    // We'll skip adding spheres, it's a bit slow.
                    //mb.AddSphere(points[0], 0.25);
                    //mb.AddSphere(points[1], 0.25);
                    var obstacleGeo = mb.ToMesh(true);
                    byte alpha = 50;
                    if (i > 0)
                    {
                        alpha = 200;
                    }
                    var obstacleModel = new GeometryModel3D(obstacleGeo, new MaterialGroup()
                    {
                        Children = new MaterialCollection() { new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(200, alpha, alpha, alpha))) }
                    });

                    ModelVisual3D visual = new ModelVisual3D() { Content = obstacleModel };
                    viewport.Children.Add(visual);
                    drawnfoodItems++;
                }

                List<String> filenames = new List<String>();
                List<Point3D> positions = new List<Point3D>(); 
                filenames.Add("..//..//data//foods//lemon_rot.stl");
                
                filenames.Add("..//..//data//foods//cucumber_rot.stl");
                filenames.Add("..//..//data//foods//corn_rot.stl");
                filenames.Add("..//..//data//foods//garlic_rot.stl");
                filenames.Add("..//..//data//foods//brocolli_rot.stl");
                filenames.Add("..//..//data//foods//carrot_rot.stl");
                filenames.Add("..//..//data//foods//tomato_rot.stl");
                filenames.Add("..//..//data//foods//cake_rot.stl");

                positions.Add(new Point3D(-300, 0, 0 ));
                positions.Add(new Point3D(-300, 0, 400 ));
                positions.Add(new Point3D(-100, 0, 200 ));
                positions.Add(new Point3D(-500, 0, 200 ));
                positions.Add(new Point3D(-300, 0, -400 ));
                positions.Add(new Point3D(-100, 0, -200));
                positions.Add(new Point3D(-500, 0, -200));
                positions.Add(new Point3D(-600, -200, 00 ));

                List<Brush> VegetableColors = new List<Brush>();

                VegetableColors.Add(new SolidColorBrush(Color.FromRgb(247,232,10)));
                VegetableColors.Add(new SolidColorBrush(Colors.DarkGreen));
                VegetableColors.Add(new SolidColorBrush(Colors.Gold));
                VegetableColors.Add(new SolidColorBrush(Colors.AntiqueWhite));
                VegetableColors.Add(new SolidColorBrush(Colors.Green));
                VegetableColors.Add(new SolidColorBrush(Colors.Orange));
                VegetableColors.Add(new SolidColorBrush(Color.FromRgb(247, 49, 10)));
                VegetableColors.Add(new SolidColorBrush(Colors.Pink));  



                for( int i = 0; i < filenames.Count; i++){
                    var r = new HelixToolkit.Wpf.StLReader();
                    Model3DGroup  model = r.Read(filenames[i]);

                    Transform3DGroup modelTransform = uniformPositionAndScale(model, (i < 7));
                    modelTransform.Children.Add(new TranslateTransform3D(positions[i].X, 50, positions[i].Z));
                    model.Transform = modelTransform;
                    MeshGeometry3D vegeGeo = (MeshGeometry3D)((GeometryModel3D)model.Children[0]).Geometry;
                    
                    var vegeModel = new GeometryModel3D(vegeGeo, new MaterialGroup()
                    {
                        Children = new MaterialCollection() { new DiffuseMaterial(VegetableColors[i]) }
                    });
                    vegeModel.Transform = modelTransform; 
                    viewport.Children.Add(new ModelVisual3D() { Content = vegeModel });
                    drawnfoodItems++;

                    // add the cube
                    if (i < 7)
                    {
                        MeshBuilder mb_bounds = new MeshBuilder();
                        mb_bounds.AddBox(new Point3D(positions[i].X, 50, positions[i].Z), 100, 100, 100);
                        var boundseGeo = mb_bounds.ToMesh(true);
                        var boundsModel = new GeometryModel3D(boundseGeo, new MaterialGroup()
                        {
                            Children = new MaterialCollection() { new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(100, 230, 230, 230))) }
                        });

                        ModelVisual3D visual3 = new ModelVisual3D() { Content = boundsModel };

                        viewport.Children.Add(visual3);
                        drawnfoodItems++;
                    }

                }

            }


            int keepFoodItems = 0;
            if (uiState.AvoidObstacle)
            {
                keepFoodItems = drawnfoodItems;
            }
            else
            {
                drawnfoodItems = 0;
            }
            while (viewport.Children.Count > lightsIndex + 1 + keepFoodItems)
            {
                viewport.Children.RemoveAt(lightsIndex + 1 + keepFoodItems);
            }

            lightsIndex = 0;
            for (int i = 0; i < overlay.Children.Count; i++)
            {
                if (overlay.Children[i] is DefaultLights)
                {
                    lightsIndex = i;
                    break;
                }
            }

            while (overlay.Children.Count > lightsIndex + 1)
            {
                overlay.Children.RemoveAt(lightsIndex + 1);
            }

            meshes =
                meshes.OrderBy(
                    mesh =>
                        uiState.Selection.SelectedTemplate.IsNull
                            ? 0
                            : uiState.Selection.SelectedTemplate.IsAncestorOf(mesh.tmpl) ? 0 : 1).ToList();

            this.rotationGizmo.SetVisible(false);
            this.scaleGizmo.SetVisible(false);
            this.scaleGizmo.setAllArrowsVisible(false);
            this.dimensions.Visibility = System.Windows.Visibility.Hidden;
            this.translatePartGizmo.SetVisible(false);
            this.translatePartGizmo.setAllArrowsVisible(false);

            if (!uiState.Selection.SelectedTemplate.IsNull && uiState.IsRotating)
            {
                if (uiState.SnappingState != null && uiState.SnappingState.PatchPair != null)
                {
                    //draw the one-torus gizmo
                    this.rotationGizmo.SetxAxisVisible();
                    Point3D p1 = uiState.SnappingState.PatchPair.MainTemplatePatch.Vertex1;
                    Point3D p2 = uiState.SnappingState.PatchPair.MainTemplatePatch.Vertex2;
                    Point3D midP = ((p1.ToVector3D() + p2.ToVector3D()) / 2).ToPoint3D();
                    Vector3D edgeDir = p2 - p1;
                    edgeDir.Normalize();
                    Vector3D y, z;
                    y = FindTwoPerpNorms(edgeDir)[0];
                    z = FindTwoPerpNorms(edgeDir)[1];
                    this.rotationGizmo.Update(midP, edgeDir, y, z, viewport);

                    //highlight the edge on the mainTemplate
                    Patch3 e = new Patch3(uiState.SnappingState.PatchPair.MainTemplatePatch);
                    List<Point3D> points = new List<Point3D>();
                    foreach (var v in e.vertices)
                    {
                        points.Add(v);
                    }
                    Geometry3D edgeGeometry;
                    MeshBuilder mb = new MeshBuilder();
                    mb.AddCylinder(points[0], points[1], 2, 36);
                    mb.AddSphere(points[0], 1);
                    mb.AddSphere(points[1], 1);
                    var edgeGeo = mb.ToMesh(true);
                    var edgeModel = new GeometryModel3D(edgeGeo, new DiffuseMaterial(Brushes.GreenYellow));

                    ModelUIElement3D edgeEle = new ModelUIElement3D();
                    edgeEle.Model = edgeModel;
                    viewport.Children.Add(edgeEle);

                }
                else
                {
                    this.rotationGizmo.SetVisible(true);
                    this.rotationGizmo.Update(uiState.Selection.SelectedTemplate.Center, new Vector3D(1, 0, 0),
                        new Vector3D(0, 1, 0),
                        new Vector3D(0, 0, 1), viewport);
                }

            }
            else if (!uiState.Selection.SelectedTemplate.IsNull && uiState.IsTranslatingPart)
            {
                SubElementFace? selectedFace = uiState.Selection.SelectedTemplate.getSubElementFace(0);
                if (selectedFace != null)
                {
                    this.translatePartGizmo.SetVisible(true);
                    this.translatePartGizmo.setAllArrowsVisible(true);
                    double xStabilityDir = 0;
                    double yStabilityDir = 0;
                    if (uiState.IsGuidingManipulation)
                    {
                        xStabilityDir = uiInstance.GetStabilityDir(uiState.Selection.SelectedTemplate, uiState.Selection.SelectedTemplate.Id, 2,
                            uiState.selecteMetric_Gait, uiState.selecteMetric_Objective);
                        yStabilityDir = uiInstance.GetStabilityDir(uiState.Selection.SelectedTemplate, uiState.Selection.SelectedTemplate.Id, 3,
                            uiState.selecteMetric_Gait, uiState.selecteMetric_Objective);
                    }
                    //Console.WriteLine(String.Format("{0:0.00}",  xStabilityDir));
                    //Console.WriteLine(String.Format("{0:0.00}",  yStabilityDir));
                    
                   // double xStabilityDir = 0;
                    //double yStabilityDir = 0;
                    this.translatePartGizmo.Update(selectedFace.Value.center,
                        Vector3D.CrossProduct(selectedFace.Value.up, selectedFace.Value.normal), selectedFace.Value.up, xStabilityDir, yStabilityDir);
                    //uiInstance.updateMetrics();
                }
            }
            else if (!uiState.Selection.SelectedTemplate.IsNull && uiState.IsScaling)
            {
                SubElementFace? selectedFace = uiState.Selection.SelectedTemplate.getSubElementFace(0);
                if (selectedFace != null)
                {
                    this.scaleGizmo.SetVisible(true);
                    this.scaleGizmo.setAllArrowsVisible(true);
                    this.dimensions.Visibility = System.Windows.Visibility.Visible;
                    double xStabilityDir = 0;
                    double yStabilityDir = 0;
                    
                    if (uiState.IsGuidingManipulation)
                    {
                        xStabilityDir = uiInstance.GetStabilityDir(uiState.Selection.SelectedTemplate, uiState.Selection.SelectedTemplate.Id, 0,
                            uiState.selecteMetric_Gait, uiState.selecteMetric_Objective);
                        yStabilityDir = uiInstance.GetStabilityDir(uiState.Selection.SelectedTemplate, uiState.Selection.SelectedTemplate.Id, 1,
                            uiState.selecteMetric_Gait, uiState.selecteMetric_Objective);
                        //Console.WriteLine("---------------------------- x dir = " + xStabilityDir);
                        //Console.WriteLine("---------------------------- y dir = " + yStabilityDir);
                    }
                    //Console.WriteLine(String.Format("{0:0.00}", xStabilityDir));
                    //Console.WriteLine(String.Format("{0:0.00}", yStabilityDir));

                    //double xStabilityDir = -0.5;
                     //double yStabilityDir = -1;
                    this.scaleGizmo.Update(selectedFace.Value.center,
                        Vector3D.CrossProduct(selectedFace.Value.up, selectedFace.Value.normal), selectedFace.Value.up, xStabilityDir, yStabilityDir);
                    dimensions.Text = string.Format("width: {0,7:N3}, height: {1,7:N3}", selectedFace.Value.width, selectedFace.Value.height);
                  //  parent.updateMetrics();
                }
            }
            else if (!uiState.Selection.SelectedTemplate.IsNull && uiState.IsMeasuring)
            {
                if (uiState.Selection.Type == Selection.SelectionType.Patch)
                {
                    PatchRef p = uiState.Selection.SelectedPatch;
                    double length = p.length;
                    this.dimensions.Visibility = System.Windows.Visibility.Visible;
                    dimensions.Text = string.Format("length: {0:N3}", length);
                }
                else
                {
                    SubElementFace? selectedFace = uiState.Selection.SelectedTemplate.getSubElementFace(0);
                    if (selectedFace != null)
                    {
                        this.dimensions.Visibility = System.Windows.Visibility.Visible;
                        dimensions.Text = string.Format("width: {0,7:N3}, height: {1,7:N3}", selectedFace.Value.width, selectedFace.Value.height);
                    }
                }
            }
            Func<Point3D, TemplateRef, Point> mapTextureCoord = (p, tmpl) =>
            {
                var axisX = new Vector3D(1, 0, -1);
                var axisY = new Vector3D(1, -2, 1);
                axisX.Normalize();
                axisY.Normalize();      
                var x = Vector3D.DotProduct(p - tmpl.Root.Center, axisX);
                var y = Vector3D.DotProduct(p - tmpl.Root.Center, axisY);
                return new Point(x, y); 
                
            };


            if (ghostMeshes != null)
            {
                foreach (Mesh ghostMesh in ghostMeshes)
                {
                    var geo = new MeshGeometry3D();
                    var faceIndices = new List<int>();
                    foreach (Face3 f in ghostMesh.faces)
                    {
                        faceIndices.AddRange(f.points);
                    }
                    foreach (var p in ghostMesh.points)
                    {
                        geo.Positions.Add(p);
                        geo.TextureCoordinates.Add(mapTextureCoord(p, ghostMesh.tmpl));
                    }
                    foreach (var n in ghostMesh.normals)
                    {
                        geo.Normals.Add(n);
                    }
                    foreach (var faceIndex in faceIndices)
                    {
                        geo.TriangleIndices.Add(faceIndex);
                    }

                    var material = GhostMaterial;


                    var model = new GeometryModel3D(geo, material);

                    model.BackMaterial = GhostMaterial;
                    model.Material = GhostMaterial;
                    SetOriginalMaterial(model, model.Material);
                    SetOriginalBackMaterial(model, model.BackMaterial);
                    ModelVisual3D visual = new ModelVisual3D { Content = model };
                    viewport.Children.Add(visual);
                }
            }
            var counter = 0;
            foreach (Mesh m in meshes)
            {
                var mCopy = m;
                var tmpl = m.tmpl;
                {
                    var geo = new MeshGeometry3D();
                    var faceIndices = new List<int>();
                    foreach (Face3 f in m.faces)
                    {
                        faceIndices.AddRange(f.points);
                    }
                    foreach (var p in m.points)
                    {
                        geo.Positions.Add(p);
                        geo.TextureCoordinates.Add(mapTextureCoord(p, tmpl));
                    }
                    foreach (var n in m.normals)
                    {
                        geo.Normals.Add(n);
                    }
                    foreach (var faceIndex in faceIndices)
                    {
                        geo.TriangleIndices.Add(faceIndex);
                    }

                    if (colors != null)
                    {
                        if (!colors[counter])
                        {
                            NormalMaterial = NonStableMaterial;
                            BackMaterial = NonStableBackMaterial;
                        }
                        else
                        {
                            NormalMaterial = DefaultMaterial;
                            BackMaterial = DefaultMaterial;
                        }
                    }
                    else
                    {
                        NormalMaterial = DefaultMaterial;
                        BackMaterial = DefaultMaterial;
                    }

                    var material = NormalMaterial;
                    var backMaterial = BackMaterial;
                    if (uiState.IsInitialDragging) {
                        material = backMaterial = InitialDraggingMaterial;
                    }
                    else
                    {
                        if (uiState.IsRotating || uiState.IsTranslating || uiState.IsScaling || uiState.IsMeasuring)
                        {

                            if (uiState.Selection.Type == Selection.SelectionType.Template ||
                                uiState.Selection.Type == Selection.SelectionType.Face)
                            {
                                if (uiState.Selection.SelectedTemplate.IsAncestorOf(tmpl))
                                {
                                    material = new MaterialGroup() { Children = new MaterialCollection() { material, new EmissiveMaterial(HightlightAddition) } };
                                    backMaterial = new MaterialGroup() { Children = new MaterialCollection() { backMaterial, new EmissiveMaterial(HightlightAddition) } };
                                }
                                else
                                {
                                    material = UnselectedMaterial;
                                    backMaterial = UnselectedMaterial;
                                }
                            }
                        }
                    }

                    foreach (TemplateRef c in closesetTemp)
                    {
                        if (c.Equals(m.tmpl))
                        {

                            material = GhostMaterial;
                            backMaterial = GhostMaterial;

                        }
                    }
                    for (int i = 0; i < closesetPactches.Count; i++)
                    {
                        {
                            TemplateRef thiselement = closesetTemp[i];
                            foreach (Patch3 e in closesetPactches[i])
                            {
                                List<Point3D> points = new List<Point3D>();
                                foreach (var v in e.vertices)
                                {
                                    points.Add(v);
                                }
                                Geometry3D edgeGeometry;
                                MeshBuilder mb = new MeshBuilder();
                                mb.AddCylinder(points[0], points[1], 3, 6);

                                var edgeGeo = mb.ToMesh(true);
                                var edgeModel = new GeometryModel3D(edgeGeo, HighlightMaterial);

                                //ModelUIElement3D edgeEle = new ModelUIElement3D();
                                //edgeEle.Model = edgeModel;
                                //edgeModel.Material = HighlightMaterial;
                                //SetOriginalMaterial(edgeModel, edgeModel.Material);
                                //var model = new GeometryModel3D(geo, material);                    
                                //model.BackMaterial = backMaterial;
                                //SetOriginalMaterial(model, model.Material);



                                ModelVisual3D visualPatch = new ModelVisual3D { Content = edgeModel };
                                //ModelUIElement3D visual = new ModelUIElement3D { Model = model };
                                viewport.Children.Add(visualPatch);
                                //// Console.WriteLine("AAAAA: " + tmpl.Root.Debuggable.getDebugInfo().shortDescription);
                                uiState.TranslationCache.AddElement(thiselement.Root, visualPatch);

                            }


                        }
                    }
                    var model = new GeometryModel3D(geo, material);
                    
                    model.BackMaterial = backMaterial;

                    SetOriginalMaterial(model, model.Material);
                    SetOriginalBackMaterial(model, model.BackMaterial);



                    if (uiState.ExportMesh)
                    {
                        var currentTmpl = tmpl;
                        while (!currentTmpl.IsNull && !currentTmpl.IsIndependent)
                        {
                            currentTmpl = currentTmpl.Parent;
                        }
                        if (!currentTmpl.IsNull)
                        {
                            if (!uiState.ExportedMesh.ContainsKey(currentTmpl))
                            {
                                uiState.ExportedMesh[currentTmpl] = new Model3DGroup();
                            }
                            uiState.ExportedMesh[currentTmpl].Children.Add(model);
                        }
                    }

                    if (uiState.IsInitialDragging || isAnimating)
                    {
                        // if initial dragging, don't create a UIElement3D to block mouse events
                        ModelVisual3D visual = new ModelVisual3D { Content = model };
                        //ModelUIElement3D visual = new ModelUIElement3D { Model = model };
                        viewport.Children.Add(visual);
                        //// Console.WriteLine("AAAAA: " + tmpl.Root.Debuggable.getDebugInfo().shortDescription);
                        uiState.TranslationCache.AddElement(tmpl.Root, visual);
                    }
                    else {
                        ModelUIElement3D ele;

                        if (cachedUIElements.ContainsKey(tmpl)) {
                            ele = cachedUIElements[tmpl];
                            ele.Model = model;
                            ele.Transform = Transform3D.Identity;
                            uiState.TranslationCache.AddElement(tmpl.Root, ele);
                        }
                        else {
       

                            
                            ele = new ModelUIElement3D {Model = model};
                            cachedUIElements[tmpl] = ele;
                            uiState.TranslationCache.AddElement(tmpl.Root, ele);



                            ele.MouseEnter += (sender, args) => {
                                if (uiState.IsScaling || uiState.IsTranslatingPart || uiState.IsRotating || uiState.IsMeasuring)
                                {
                                    if (!isConnecting)
                                    {
                                        ((GeometryModel3D)ele.Model).Material = new MaterialGroup { Children = new MaterialCollection { GetOriginalMaterial(ele.Model), new EmissiveMaterial(HightlightAddition) } }; ;
                                        ((GeometryModel3D)ele.Model).BackMaterial = new MaterialGroup { Children = new MaterialCollection { GetOriginalBackMaterial(ele.Model), new EmissiveMaterial(HightlightAddition) } }; ;
                                    }
                                }
                            };
                            ele.MouseLeave += (sender, args) => {
                                    if (!isConnecting)
                                    {
                                        ((GeometryModel3D)ele.Model).Material = GetOriginalMaterial(ele.Model);
                                        ((GeometryModel3D)ele.Model).BackMaterial = GetOriginalBackMaterial(ele.Model);
                                    }
                                
                            };

                            ele.MouseDown += (sender, args) => {

                                if (args.ChangedButton != MouseButton.Left) return;
                                if (uiState.IsRotating) {
                                    uiState.Selection = Selection.Template(tmpl.Root);
                                }
                                else if (uiState.IsScaling || uiState.IsTranslatingPart) {
                                    uiState.Selection = Selection.Face(tmpl);
                                }
                                else {
                                    uiState.Selection = Selection.Template(tmpl);
                                }

                                updateCallback();
                                lastPosition = args.GetPosition(viewport);
                                Mouse.Capture(ele);
                            };
                            ele.MouseMove += (sender, args) => {
                                if (lastPosition != null) {
                                    Point3D? to = viewport.Viewport.UnProject(args.GetPosition(viewport));
                                    Point3D? from = viewport.Viewport.UnProject(lastPosition.Value);
                                    if (to != null && from != null) {
                                        //uiInstance.Translate(tmpl.Root, to.Value - from.Value);
                                        uiState.TranslationCache.Translate(tmpl.Root, to.Value - from.Value);
                                        uiState.SnappingState = null;
                                        //// Console.WriteLine("translating");
                                        //updateCallback();
                                    }
                                    lastPosition = args.GetPosition(viewport);
                                }
                            };
                            ele.MouseUp += (sender, args) => {
                                lastPosition = null;
                                Mouse.Capture(null);
                                uiState.TranslationCache.Commit();
                                updateCallback();
                            };



                        }

                        if (false) // I don't understand this part of the code
                        {
                            if (uiState.DebugSelection.Type == Selection.SelectionType.Template &&
                                uiState.DebugSelection.SelectedTemplate.IsAncestorOf(tmpl))
                            {
                                #region Upon hovering in debugging view, highlight item and save material

                                Debugging.SetDebuggingTag(model, new[] { model.Material, model.BackMaterial });
                                model.Material = model.BackMaterial = DebugMaterial;

                                #endregion

                                viewport.Children.Remove(ele);
                                if (!overlay.Children.Contains(ele))
                                {
                                    overlay.Children.Add(ele);
                                }
                            }
                            else
                            {
                                #region Restore material before debugging window mouse hover

                                if (Debugging.GetDebuggingTag(model) != null)
                                {
                                    Material[] savedMaterials = (Material[])Debugging.GetDebuggingTag(model);
                                    model.Material = savedMaterials[0];
                                    model.BackMaterial = savedMaterials[1];
                                }

                                #endregion

                                overlay.Children.Remove(ele);
                                viewport.Children.Add(ele);
                            }
                        }
                        else
                        {
                            overlay.Children.Remove(ele);
                            viewport.Children.Add(ele);
                        }
                    }
                }
                
                // for each patch, draw it
                //foreach (Patch3 e in mCopy.patches.Where(p => tmpl.getAllConnectionsForPatch(p.patchRef).Count > 0))
                if (!uiState.IsInitialDragging) {
                    foreach (Patch3 e in mCopy.patches) {
                        List<Point3D> points = new List<Point3D>();
                        foreach (var v in e.vertices) {
                            points.Add(v);
                        }
                        Geometry3D edgeGeometry;
                        MeshBuilder mb = new MeshBuilder();
                        mb.AddCylinder(points[0], points[1], 2, 6);
                        // We'll skip adding spheres, it's a bit slow.
                        //mb.AddSphere(points[0], 1);
                        //mb.AddSphere(points[1], 1);
                        var edgeGeo = mb.ToMesh(true);
                        var edgeModel = new GeometryModel3D(edgeGeo, new EmissiveMaterial(Brushes.Transparent));

                        ModelUIElement3D edgeEle = new ModelUIElement3D();
                        edgeEle.Model = edgeModel;

                        if (uiState.Selection.SelectedPatch.Equals(e.patchRef)) {
                            edgeModel.Material = HighlightMaterial;
                        }
                        SetOriginalMaterial(edgeModel, edgeModel.Material);
                        
                        if (tmpl.Root.getAllConnectionsForPatch(e.patchRef).Count > 0) {
                            edgeEle.MouseEnter += (sender, args) => {
                                if (uiState.IsMeasuring)
                                {
                                    edgeModel.Material = HighlightMaterial;
                                }
                            };
                            edgeEle.MouseLeave += (sender, args) => {
                                edgeModel.Material = GetOriginalMaterial(edgeModel);
                            };
                            edgeEle.MouseDown += (sender, args) => {
                                if (args.ChangedButton != MouseButton.Left) return;
                                PatchRef pr = e.patchRef;
                                pr.length = (points[0] - points[1]).Length;
                                uiState.Selection = Selection.Patch(tmpl, pr);
                                //List<ConnectionRef> connectionRefs = tmpl.Root.getAllConnectionsForPatch(e.patchRef);
                                //ConnectionRef firstConnection = connectionRefs[0];
                                //uiState.Selection = Selection.Edge(tmpl, e.edgeid);

                                //Console.WriteLine(points[0].X + ", " + points[0].Y + ", " + points[0].Z);
                                //Console.WriteLine(points[1].X + ", " + points[1].Y + ", " + points[1].Z);
                                //Console.WriteLine((points[0]-points[1]).Length); 
                                //Console.WriteLine(uiState.Selection.SelectedPatch.length); 
                                updateCallback();
                            };

                            edgeEle.MouseUp += (sender, args) => {
                                updateCallback();
                            };
                        }
                        if (uiState.DebugSelection.Type == Selection.SelectionType.Patch &&
                            uiState.DebugSelection.SelectedPatch.Equals(e.patchRef)) {
                            edgeModel.Material = edgeModel.BackMaterial = DebugMaterial;
                            overlay.Children.Add(edgeEle);
                        }
                        else if (tmpl.Root.getAllConnectionsForPatch(e.patchRef).Count > 0) {
                            viewport.Children.Add(edgeEle);
                        }
                    }

                    if (!drawnArticulations.Contains(tmpl.Root)) {
                        drawnArticulations.Add(tmpl.Root);
                        foreach (var pair in tmpl.Root.getArticulationAxes()) {
                            var center = pair.Item1;
                            var axis = pair.Item2;
                            //Console.Write("finding articulations!!!------------------------------------------------------");
                            //Console.Write("axis: is ( " + axis.ToString());
                            //Console.Write("center: is ( " + center.ToString());
                            
                            //turn off the axis drawing for now
                            if (false) {
                                var a = center - 5*axis;
                                var b = center + 5*axis;

                                MeshBuilder mb = new MeshBuilder();
                                mb.AddCylinder(a, b, 2, 6);
                                // We'll skip adding spheres, it's a bit slow.
                                //mb.AddSphere(a, 1);
                                //mb.AddSphere(b, 1);
                                var axisGeo = mb.ToMesh(true);
                                var axisGeoModel = new GeometryModel3D(axisGeo, new DiffuseMaterial(Brushes.Violet));
                                viewport.Children.Add(new ModelVisual3D() {Content = axisGeoModel});
                            }
                        }
                    }
                }
                //draw the edges
                if (!isAnimating)
                {                                  
                    foreach (Patch3 e in mCopy.patches)
                    {
                        List<Point3D> points = new List<Point3D>();
                        foreach (var v in e.vertices)
                        {
                            points.Add(v);
                        }
                        Geometry3D edgeGeometry;
                        MeshBuilder mb = new MeshBuilder();
                        mb.AddCylinder(points[0], points[1], 0.5, 6);
                        // We'll skip adding spheres, it's a bit slow.
                        //mb.AddSphere(points[0], 0.25);
                        //mb.AddSphere(points[1], 0.25);
                        var edgeGeo = mb.ToMesh(true);
                        var edgeModel = new GeometryModel3D(edgeGeo, new MaterialGroup()
                        {
                            Children = new MaterialCollection() { new DiffuseMaterial(Brushes.Black), new EmissiveMaterial(Brushes.Gray) }
                        });

                        ModelVisual3D visual = new ModelVisual3D() { Content = edgeModel };
                        viewport.Children.Add(visual);
                        uiState.TranslationCache.AddElement(tmpl.Root, visual);
                    }
                }
               

                counter++;
            }



            

            uiState.ExportMesh = false;
        }




        public Transform3DGroup uniformPositionAndScale(Model3DGroup model, bool isVege)
        {

            Transform3DGroup modelTransform = new Transform3DGroup();
            Rect3D  bounds = model.Bounds;
            Vector3D center = 0.5 * (new Vector3D(bounds.SizeX, bounds.SizeY, bounds.SizeZ))
                    + (new Vector3D(bounds.X, bounds.Y, bounds.Z));
            modelTransform.Children.Add ( new TranslateTransform3D(-center.X, -center.Y, -center.Z));
            double maxSize = Math.Max((Math.Max(bounds.SizeX, bounds.SizeY)), bounds.SizeZ);
            
            double scaleAmount = 100 / maxSize;
            if (!isVege)
                scaleAmount = 2 * scaleAmount;
             modelTransform.Children.Add ( new ScaleTransform3D(scaleAmount, scaleAmount, scaleAmount));

                return modelTransform; 
        }

        public void Deselect()
        {
            uiState.Selection = Selection.Empty;
        }

        //get a material with the texture of the fileName
        private Material GetModelMaterial(string fileName, double scale, bool emissive=false)
        {
            ImageBrush ib = new ImageBrush(
                new BitmapImage(
                    new Uri(UI3D.defaultPath + fileName + ".jpg", UriKind.Relative
                    )
                )
            );

            ib.ViewportUnits = BrushMappingMode.Absolute;
            ib.TileMode = TileMode.Tile;
            ib.Transform = new ScaleTransform(scale, scale);
            MaterialGroup group = new MaterialGroup();
            if (emissive) {
                group.Children.Add(new DiffuseMaterial(Brushes.Black));
                group.Children.Add(new EmissiveMaterial(ib));
            }
            else {
                group.Children.Add(new DiffuseMaterial(ib));
            }
            return group;
        }

        //get a plane with y=0
        private ModelVisual3D GetFloor(string fileName, double scale)
        {
            double x = 1000.0; // floor width / 2
            double z = 1000.0; // floor length / 2
            double floorDepth = -0.1; // give the floor some depth so it doesn't collide with other faces on the floor

            Point3DCollection points = new Point3DCollection(20);
            Point3D point;
            //top of the floor
            point = new Point3D(-x, 0, z);// Floor Index - 0
            points.Add(point);
            point = new Point3D(x, 0, z);// Floor Index - 1
            points.Add(point);
            point = new Point3D(x, 0, -z);// Floor Index - 2
            points.Add(point);
            point = new Point3D(-x, 0, -z);// Floor Index - 3
            points.Add(point);

            //indices
            int[] indices = new int[] { 0, 1, 2, 0, 2, 3/*, 4, 5, 7, 5, 6, 7, 8, 9, 11, 9, 10, 11, 12, 13, 15, 13, 14, 15, 16, 17, 19, 17, 18, 19 */};           
            Point[] tx = new Point[] {new Point(0, 0), new Point(0, 1), new Point(1, 1), new Point(1, 0)};
            var geo = new MeshGeometry3D();
            //var floor = new CubeVisual3D();
            //floor.Transform = new ScaleTransform3D(1000, 20, 1000);

            geo.Positions = points;
            geo.TriangleIndices = new Int32Collection(indices);
            geo.TextureCoordinates = new PointCollection(tx);
            GeometryModel3D gm =  new GeometryModel3D(geo,FloorMaterial);
            var floor = new ModelVisual3D(){Content = gm};
            floor.Transform = new TranslateTransform3D(0, floorDepth, 0);        
            gm.Material = GetModelMaterial(fileName, scale, true);

            /*
            MaterialGroup group = new MaterialGroup();
            group.Children.Add(new DiffuseMaterial(Brushes.Black));
            var imageBrush = new ImageBrush(new BitmapImage(new Uri("..\\..\\code\\csharp\\ui\\FBE_CSharpUI\\Resources\\floor1.jpg", UriKind.Relative)));
            imageBrush.TileMode = TileMode.Tile;
            //imageBrush.ViewboxUnits = BrushMappingMode.Absolute;
            imageBrush.ViewportUnits = BrushMappingMode.Absolute;
            //imageBrush.Stretch = Stretch.None;
            imageBrush.Transform = new ScaleTransform(0.05, 0.05);
            group.Children.Add(new EmissiveMaterial(imageBrush));
            gm.Material = group;
             */

            return floor;
        }

    }
    
}