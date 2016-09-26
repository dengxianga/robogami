using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;

namespace FBE_CSharpUI
{
    class DynamicMenu
    {
        public static readonly DependencyProperty IsShiftOnlyProperty = DependencyProperty.RegisterAttached(
            "IsShiftOnly", typeof (bool), typeof (DynamicMenu), new PropertyMetadata(default(bool)));

        public static void SetIsShiftOnly(DependencyObject element, bool value) {
            element.SetValue(IsShiftOnlyProperty, value);
        }

        public static bool GetIsShiftOnly(DependencyObject element) {
            return (bool) element.GetValue(IsShiftOnlyProperty);
        }

        public static readonly DependencyProperty IsCtrlOnlyProperty = DependencyProperty.RegisterAttached(
            "IsCtrlOnly", typeof (bool), typeof (DynamicMenu), new PropertyMetadata(default(bool)));

        public static void SetIsCtrlOnly(DependencyObject element, bool value) {
            element.SetValue(IsCtrlOnlyProperty, value);
        }

        public static bool GetIsCtrlOnly(DependencyObject element) {
            return (bool) element.GetValue(IsCtrlOnlyProperty);
        }

        public static readonly DependencyProperty IsNoModifierOnlyProperty = DependencyProperty.RegisterAttached(
            "IsNoModifierOnly", typeof (bool), typeof (DynamicMenu), new PropertyMetadata(default(bool)));

        public static void SetIsNoModifierOnly(DependencyObject element, bool value) {
            element.SetValue(IsNoModifierOnlyProperty, value);
        }

        public static bool GetIsNoModifierOnly(DependencyObject element) {
            return (bool) element.GetValue(IsNoModifierOnlyProperty);
        }
    }
}
