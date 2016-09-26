using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using CppCsBridge;

namespace FBE_CSharpUI
{
    /// <summary>
    /// Interaction logic for ConfigurationChooser.xaml
    /// </summary>
    public partial class SymmetryChooser : UserControl
    {
        public SymmetryChooser()
        {
            InitializeComponent();
            updateButton.Click += (sender, args) => {
                Action<bool, bool, bool, bool> temp = Update;
                if (temp != null)
                {
                     temp(symmGroundCheck.IsChecked.Value,
                         symmLegWCheck.IsChecked.Value, 
                         symmLegLCheck.IsChecked.Value,
                         symmSpacingCheck.IsChecked.Value);
                }
            };
        }

          public void Choose(bool symm_gound, bool symm_legW, bool symm_legL, bool symm_spacing) {
              symmGroundCheck.IsChecked = symm_gound;
              symmLegWCheck.IsChecked = symm_legW;
              symmLegLCheck.IsChecked = symm_legL;
              symmSpacingCheck.IsChecked = symm_spacing;
        }

        public event Action<bool, bool, bool, bool> Update;
    }

}
