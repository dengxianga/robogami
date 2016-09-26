using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media;

namespace FBE_CSharpUI
{
    static class Util
    {
        public static SolidColorBrush MakeTransparent(this SolidColorBrush brush, double opacity)
        {
            return new SolidColorBrush { Color = brush.Color.MakeTransparent(opacity) };
        }

        public static Color MakeTransparent(this Color color, double opacity)
        {
            return Color.FromArgb((byte)(color.A * opacity), color.R, color.G, color.B);
        }
    }
}
