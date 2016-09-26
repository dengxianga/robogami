using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Color = System.Windows.Media.Color;

namespace FBE_CSharpUI
{
    public class AwesomeLights : LightSetup
    {
        private event Action<HelixViewport3D> GotViewport;

        public AwesomeLights(HelixViewport3D viewport)
        {
            GotViewport(viewport);
        }

        /// <summary>
        /// Adds the lights to the element.
        /// 
        /// </summary>
        /// <param name="lightGroup">The light group.
        ///             </param>
        protected override void AddLights(Model3DGroup lightGroup)
        {
            // These are the lights from DefaultLights, let's tune them down a bit
            lightGroup.Children.Add(new DirectionalLight(Color.FromRgb(20, 20, 20), new Vector3D(-1.0, -1.0, -1.0)));
            lightGroup.Children.Add(new DirectionalLight(Color.FromRgb(20, 20, 20), new Vector3D(1.0, -1.0, -0.1)));
            lightGroup.Children.Add(new DirectionalLight(Color.FromRgb(20, 20, 20), new Vector3D(0.1, 1.0, -1.0)));
            lightGroup.Children.Add(new DirectionalLight(Color.FromRgb(20, 20, 20), new Vector3D(0.1, 0.1, 1.0)));
            lightGroup.Children.Add(new AmbientLight(Color.FromRgb(50, 50, 50)));
            //ShowLights = true;
            // Then add a few point lights

            var cameraLight = new PointLight(Color.FromRgb(255, 255, 255), new Point3D());
            cameraLight.LinearAttenuation = 0.01;
            //BindingOperations.SetBinding(cameraLight, PointLight.PositionProperty, new Binding("Position") {Source=camera});
            GotViewport += viewport =>
            {
                cameraLight.Position = viewport.Camera.Position;
                viewport.CameraChanged += (sender, args) =>
                {
                    cameraLight.Position = viewport.Camera.Position;
                };
            };
            lightGroup.Children.Add(cameraLight);

            //three point lights
            
            var scale = 20;
            Func<Point3D, Point3D> scaler = p => (p.ToVector3D()*scale).ToPoint3D();
            lightGroup.Children.Add(new PointLight(Colors.White, scaler(new Point3D(-3.54, 6.62, 4.98))) { ConstantAttenuation = 1.5 / 0.86});
            lightGroup.Children.Add(new PointLight(Colors.White, scaler(new Point3D(2.89, 5.60, 6.71))) { ConstantAttenuation = 1.5 / 0.84 });
            lightGroup.Children.Add(new PointLight(Colors.White, scaler(new Point3D(3.10, 8.80, -4.47))) { ConstantAttenuation = 1.5 / 0.580 });
            lightGroup.Children.Add(new PointLight(Colors.White, scaler(new Point3D(-6.02, 8.07, -2.16))) { ConstantAttenuation = 1.5 / 0.32 });
            
            //ShowLights = true;
        }
    }
}
