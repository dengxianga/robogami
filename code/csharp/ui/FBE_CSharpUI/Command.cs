using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Windows;
using System.Windows.Media.Media3D;
using CppCsBridge;
using FabByExample.proto.symbolic;

namespace FBE_CSharpUI
{
    class Command
    {
        public UIInstance UiInstance { get; set; }
        public UIStates UiStates { get; set; }
        public MainWindow MainWindow { get; set; }

        public Command(UIInstance uiInstance, UIStates uiStates, MainWindow mainWindow)
        {
            UiInstance = uiInstance;
            UiStates = uiStates;
            MainWindow = mainWindow;
        }

        public void Execute(string command)
        {
                if (command.StartsWith("rotate "))
                {
                    string[] split = command.Split(' ');
                    Vector3D axis = new Vector3D(double.Parse(split[1]), double.Parse(split[2]), double.Parse(split[3]));
                    Quaternion q = new Quaternion(axis, double.Parse(split[4]));
                    var tmpl = UiStates.Selection.SelectedTemplate;
                    if (!tmpl.IsNull)
                    {
                        UiInstance.Rotate(tmpl, q, tmpl.Center);
                    }
                    else
                    {
                        Console.WriteLine("No template selected.");
                    }
                }
                else if (command.StartsWith("translate "))
                {
                    string[] split = command.Split(' ');
                    Vector3D trans = double.Parse(split[4])*new Vector3D(double.Parse(split[1]), double.Parse(split[2]), double.Parse(split[3]));
                    
                    var tmpl = UiStates.Selection.SelectedTemplate;
                    if (!tmpl.IsNull)
                    {
                        UiInstance.Translate(tmpl, trans);
                    }
                    else
                    {
                        Console.WriteLine("No template selected.");
                    }
                }
                else if (command.StartsWith("scad example")) {
                    UiInstance.makeExampleScadTemplate();
                }
                else if (command == "reflect on")
                {
                    OverlapPreventionHack.SetAlwaysReflect(true);
                }
                else if (command == "reflect off")
                {
                    OverlapPreventionHack.SetAlwaysReflect(false);
                }
                else if (command == "test label selection") {
                    JointLayout layout = new JointLayout();
                    layout.RobotShape = new List<Point> {new Point(0, 10), new Point(10, 10), new Point(10, 0), new Point(0, 0)};
                    layout.Joints = new List<Point> {new Point(0, 10), new Point(10, 10), new Point(10, 0), new Point(0, 0)};
                    List<JointLabeling> labelings = new List<JointLabeling> {
                        new JointLabeling {JointLabels = new List<int> {1, 3, 2, 4}},
                        new JointLabeling {JointLabels = new List<int> {1, 2, 2, 4}},
                        new JointLabeling {JointLabels = new List<int> {1, 1, 2, 4}},
                        new JointLabeling {JointLabels = new List<int> {1, 1, 3, 4}}
                    };
                    //MainWindow.configurationChooser.Choose(UiInstance, layout, labelings, 1, 40);
                }
                else if (command.StartsWith("add constraint "))
                {
                    // Example:
                    // add constraint 2(#0) + -3(#1) + (#3) = 0
                    // Means that
                    // Suppose the selected template has parameters defined as:
                    //   param #0 = width, param #1 = height, param #2 = depth, param #3 = radius
                    // Then this means 2 * width - 3 * height + radius = 0

                    Regex regex = new Regex("^(.*)([=<>])(.+)$");
                    string exprWithoutSpaces = command.Substring("add constraint ".Length).Replace(" ", "");
                    Match match = regex.Match(exprWithoutSpaces);
                    if (match.Success)
                    {
                        string expr = match.Groups[1].Value;
                        string oper = match.Groups[2].Value;
                        string constant = match.Groups[3].Value;
                        string[] terms = expr.Split('+');
                        Regex termRegex = new Regex(@"^(.*)[(]#(\d+)[)]$");
                        try
                        {
                            Tuple<double, int>[] parsedTerms = terms.Select(term =>
                            {
                                Match termMatch = termRegex.Match(term);
                                if (termMatch.Success)
                                {
                                    string coeffStr = termMatch.Groups[1].Value;
                                    string indexStr = termMatch.Groups[2].Value;
                                    double coeff = coeffStr == "" ? 1 : coeffStr == "-" ? -1 : double.Parse(coeffStr);
                                    int index = int.Parse(indexStr);
                                    return Tuple.Create(coeff, index);
                                }
                                else
                                {
                                    throw new ArgumentException("Invalid term: " + term);
                                }
                            }).ToArray();

                            bool isEquality = oper == "=";
                            double constantTerm = double.Parse(constant);
                            if (oper == ">")
                            {
                                parsedTerms = parsedTerms.Select(t => Tuple.Create(-t.Item1, t.Item2)).ToArray();
                                constantTerm = -constantTerm;
                            }
                            TemplateRef selectedTemplate = UiStates.Selection.SelectedTemplate;
                            if (selectedTemplate.IsNull)
                            {
                                MessageBox.Show("Must select a template first.");
                                return;
                            }
                            selectedTemplate.AddConstraint(parsedTerms, constantTerm, isEquality);
                        }
                        catch (ArgumentException ex)
                        {
                            MessageBox.Show(ex.Message);
                            return;
                        }
                        catch (FormatException ex)
                        {
                            MessageBox.Show(ex.Message);
                            return;
                        }
                    }
                    else
                    {
                        MessageBox.Show("Invalid command");
                    }
                }
        }
    }
}
