using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Input;

namespace FBE_CSharpUI
{
    public static class UICommands {
        private static readonly Type ownerType = typeof (Window);
        public static readonly RoutedUICommand SelectParent = new RoutedUICommand("Select Parent", "SelectParent", ownerType);
        public static readonly RoutedUICommand SelectChild = new RoutedUICommand("Select Child", "SelectChild", ownerType);
        public static readonly RoutedUICommand SaveGeometry = new RoutedUICommand("Save Geometry", "SaveGeometry", ownerType);
        public static readonly RoutedUICommand Reload = new RoutedUICommand("Reload", "Reload", ownerType);
        public static readonly RoutedUICommand Exit = new RoutedUICommand("Exit", "Exit", ownerType);
        public static readonly RoutedUICommand DeleteSelected = new RoutedUICommand("Delete Selected", "DeleteSelected", ownerType);
        public static readonly RoutedUICommand DeleteTemplate = new RoutedUICommand("Delete Template Being Added", "DeleteTemplate", ownerType);
        public static readonly RoutedUICommand Snap = new RoutedUICommand("Snap", "Snap", ownerType);
        public static readonly RoutedUICommand MultiSnap = new RoutedUICommand("Multi Snap", "MultiSnap", ownerType);
        public static readonly RoutedUICommand Connect = new RoutedUICommand("Connect", "Connect", ownerType);
        public static readonly RoutedUICommand ChangeMaterial = new RoutedUICommand("Change Material", "ChangeMaterial", ownerType);
        public static readonly RoutedUICommand UsePerspectiveCamera = new RoutedUICommand("UsePerspectiveCamera", "Use Perspective Camera", ownerType);
        public static readonly RoutedUICommand UpdateConnectors = new RoutedUICommand("Update Connectors", "UpdateConnectors", ownerType);
        public static readonly RoutedUICommand HighlightConnectors = new RoutedUICommand("Highlight Connectors", "HighlightConnectors", ownerType);
        public static readonly RoutedUICommand Simulate = new RoutedUICommand("Simulate", "Simulate", ownerType);
        public static readonly RoutedUICommand ViewGait = new RoutedUICommand("ViewGait", "ViewGait", ownerType);
        public static readonly RoutedUICommand Animate = new RoutedUICommand("Animate", "Animate", ownerType);
        public static readonly RoutedUICommand AnimateSlow = new RoutedUICommand("AnimateSlow", "AnimateSlow", ownerType);
        public static readonly RoutedUICommand GuideManipulation = new RoutedUICommand("GuideManipulation", "GuideManipulation", ownerType);
        public static readonly RoutedUICommand DisableGuidance = new RoutedUICommand("DisableGuidance", "DisableGuidance", ownerType);
        public static readonly RoutedUICommand Stablize = new RoutedUICommand("Stablize", "Stablize", ownerType);
        public static readonly RoutedUICommand SaveView = new RoutedUICommand("Save View", "SaveView", ownerType);
        public static readonly RoutedUICommand DeleteAllSavedViews = new RoutedUICommand("Delete All Saved Views", "DeleteAllSavedViews", ownerType);
        public static readonly RoutedUICommand PreventFurtherOverlap = new RoutedUICommand("Prevent Further Overlaps", "PreventFurtherOverlap", ownerType);
        public static readonly RoutedUICommand ShowOverlaps = new RoutedUICommand("Show Overlaps in Drawing", "ShowOverlaps", ownerType);
        public static readonly RoutedUICommand SnapToGround = new RoutedUICommand("Snap to Ground", "SnapToGround", ownerType);
        public static readonly RoutedUICommand ForceReflection = new RoutedUICommand("Enable Reflection", "EnableReflection", ownerType);
        public static readonly RoutedUICommand AvoidObstacle = new RoutedUICommand("Avoid Obstacle", "Avoid Obstacle", ownerType);
        public static readonly RoutedUICommand ViewAnimPoses = new RoutedUICommand("View Anim Poses", "ViewAnimPoses", ownerType);
        public static readonly RoutedUICommand ChangeAnimParams = new RoutedUICommand("ChangeAnimParams", "ChangeAnimParams", ownerType);
        public static readonly RoutedUICommand ForwardTravelA = new RoutedUICommand("ForwardTravelA", "ForwardTravelA", ownerType);
        public static readonly RoutedUICommand ForwardTravelB = new RoutedUICommand("ForwardTravelB", "ForwardTravelB", ownerType);
        public static readonly RoutedUICommand DesignCar = new RoutedUICommand("DesignCar", "DesignCar", ownerType);
        public static readonly RoutedUICommand NoTask = new RoutedUICommand("NoTask", "NoTask", ownerType);
        public static readonly RoutedUICommand DirectSave = new RoutedUICommand("DirectSave", "DirectSave", ownerType);
        public static readonly RoutedUICommand RevertToOriginal = new RoutedUICommand("RevertToOriginal", "RevertToOriginal", ownerType);


        public static readonly RoutedUICommand LoadView1 = new RoutedUICommand("Load View 1", "LoadView1", ownerType);
        public static readonly RoutedUICommand LoadView2 = new RoutedUICommand("Load View 2", "LoadView2", ownerType);
        public static readonly RoutedUICommand LoadView3 = new RoutedUICommand("Load View 3", "LoadView3", ownerType);
        public static readonly RoutedUICommand LoadView4 = new RoutedUICommand("Load View 4", "LoadView4", ownerType);

        
        static UICommands() {
            SelectParent.InputGestures.Add(new KeyGesture(Key.Up, ModifierKeys.Control));
            SelectChild.InputGestures.Add(new KeyGesture(Key.Down, ModifierKeys.Control));
            Exit.InputGestures.Add(new KeyGesture(Key.F4, ModifierKeys.Alt));
            DeleteSelected.InputGestures.Add(new KeyGesture(Key.Delete));
            DeleteTemplate.InputGestures.Add(new KeyGesture(Key.Delete, ModifierKeys.Control));
            Snap.InputGestures.Add(new KeyGesture(Key.Space));
            MultiSnap.InputGestures.Add(new KeyGesture(Key.Space, ModifierKeys.Control));
            ChangeMaterial.InputGestures.Add(new KeyGesture(Key.M, ModifierKeys.Control));
            UsePerspectiveCamera.InputGestures.Add(new KeyGesture(Key.P, ModifierKeys.Control));
            SaveView.InputGestures.Add(new KeyGesture(Key.V, ModifierKeys.Control));
            //Stablize.InputGestures.Add(new KeyGesture(Key.S));

            LoadView1.InputGestures.Add(new KeyGesture(Key.F1));
            LoadView2.InputGestures.Add(new KeyGesture(Key.F2));
            LoadView3.InputGestures.Add(new KeyGesture(Key.F3));
            LoadView4.InputGestures.Add(new KeyGesture(Key.F4));

            Connect.InputGestures.Add(new KeyGesture(Key.A, ModifierKeys.Control));
            
        }
    }
}
