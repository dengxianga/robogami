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
using FabByExample.proto.symbolic;
using Control = FabByExample.proto.symbolic.Control;
using SymbolicTransform=FabByExample.proto.symbolic.Transform;

namespace FBE_CSharpUI
{
    /// <summary>
    /// Interaction logic for TransformationEditor.xaml
    /// </summary>
    public partial class TransformationEditor : UserControl
    {
        public TransformationEditor()
        {
            InitializeComponent();
        }

        public SymbolicTransform EditingTransform { get; private set; }
        public event Action Deleted;

        public void Edit(SymbolicTransform xform) {
            this.EditingTransform = xform;
            if (xform.control.inputs.piecewiselinear == null) {
                xform.control = new Control();
                xform.control.inputs = new ControlMappingFunction();
                xform.control.inputs.piecewiselinear = new PiecewiseLinearFunction1D();
            }
            RefreshUI();
        }

        private void RefreshUI() {
           // if (EditingTransform.type == SymbolicTransform.TransformType.PRISMATIC_TRANSFORM)
           // {
           //    TypeText.Content = "Translation Transform";
           //}
           // else {
           //     TypeText.Content = "Rotation Transform";
           // }

            var piecewiseLinear = EditingTransform.control.inputs.piecewiselinear;

            ValuesList.Children.Clear();
            foreach (var pair in piecewiseLinear.timesandvalues) {
                Grid row = new Grid();
                row.ColumnDefinitions.Add(
                    new ColumnDefinition { Width = new GridLength(2, GridUnitType.Star) });
                row.ColumnDefinitions.Add(
                    new ColumnDefinition { Width = new GridLength(2, GridUnitType.Star) });
                row.ColumnDefinitions.Add(
                    new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });

                TextBox timeBox = new TextBox();
                timeBox.Margin = new Thickness(3);
                timeBox.Text = pair.timeStamp.ToString();
                Grid.SetColumn(timeBox, 0);
                TextBox valueBox = new TextBox();
                valueBox.Margin = new Thickness(3);
                valueBox.Text = pair.value.ToString();
                Grid.SetColumn(valueBox, 1);
                //Button deleteButton = new Button();
               // deleteButton.Content = "Delete";
               // deleteButton.Margin = new Thickness(3);
               // Grid.SetColumn(deleteButton, 2);
                row.Children.Add(timeBox);
                row.Children.Add(valueBox);
                //row.Children.Add(deleteButton);

                var pair1 = pair;
                timeBox.LostFocus += (sender, args) => {
                    double parsed;
                    if (double.TryParse(timeBox.Text, out parsed) && Math.Abs(parsed - pair1.timeStamp) > 1e-5) {
                        pair1.timeStamp = parsed;
                        RefreshUI();
                    }
                };

                valueBox.LostFocus += (sender, args) => {
                    double parsed;
                    if (double.TryParse(valueBox.Text, out parsed) && Math.Abs(parsed - pair1.value) > 1e-5) {
                        pair1.value = parsed;
                        RefreshUI();
                    }
                };

                //deleteButton.Click += (sender, args) => {
                //    piecewiseLinear.timesandvalues.Remove(pair1);
                //    RefreshUI();
                //};
                ValuesList.Children.Add(row);
            }
        }

        private void AddButtonClick(object sender, RoutedEventArgs e)
        {
            EditingTransform.control.inputs.piecewiselinear.timesandvalues.Add(new TimeAndValuePair() { timeStamp = 1, value = 1 });
            RefreshUI();
        }

        private void DeleteButtonClick(object sender, RoutedEventArgs e) {
            Action temp = Deleted;
            if (temp != null) {
                temp();
            }
        }
    }
}
