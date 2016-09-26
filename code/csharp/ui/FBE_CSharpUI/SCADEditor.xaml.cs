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
using FabByExample.proto;

namespace FBE_CSharpUI
{
    /// <summary>
    /// Interaction logic for SCADEditor.xaml
    /// </summary>
    public partial class SCADEditor : UserControl
    {
        public SCADEditor()
        {
            InitializeComponent();
        }

        public OpenscadDesign Design { get; private set; }

        private TextBox makeEditor(object initValue, Action<string, TextBox> callback) {
            TextBox textBox = new TextBox();
            textBox.Width = 70;
            textBox.Text = initValue.ToString();
            textBox.LostFocus += (sender, args) => {
                callback(textBox.Text, textBox);
            };
            textBox.KeyDown += (sender, args) =>
            {
                callback(textBox.Text, textBox);
            };
            textBox.TextChanged += (sender, args) =>
            {
                callback(textBox.Text, textBox);
            };
            return textBox;
        }

        public void Edit(OpenscadDesign design) {
            Design = design;
            ParametersPanel.Children.Clear();
            CodeEditor.Text = design.code;
            foreach (var param in design.parameter)
            {
                var param1 = param;
                StackPanel panel = new StackPanel {
                    Orientation = Orientation.Horizontal,
                    HorizontalAlignment = HorizontalAlignment.Center
                };
                panel.Children.Add(makeEditor(param1.name, (s, _) => param1.name = s));
                panel.Children.Add(new Label() {Content = " = "});
                panel.Children.Add(makeEditor(param1.initial_value, (s, tb) => {
                    double value;
                    if (double.TryParse(s, out value)) {
                        param1.initial_value = value;
                        tb.Background = null;
                    }
                    else {
                        tb.Background = Brushes.LightPink;
                    }
                }));
                var deleteButton = new Button {Content="Delete", Margin = new Thickness(5, 0, 0, 0)};
                deleteButton.Click += (sender, args) => {
                    design.parameter.Remove(param1);
                    Edit(design);
                };
                panel.Children.Add(deleteButton);
                panel.Margin = new Thickness(5);
                ParametersPanel.Children.Add(panel);
            }
            var addButton = new Button() {Content = "Add"};
            addButton.Click+= (sender, args) => {
                design.parameter.Add(new OpenscadParameter{name="p", initial_value = 1});
                Edit(design);
            };
            ParametersPanel.Children.Add(addButton);
        }

        private void CodeEditor_TextChanged(object sender, EventArgs e) {
            if (Design != null) {
                Design.code = CodeEditor.Text;
            }
        }
    }
}
