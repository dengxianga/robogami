using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using MessageBox = System.Windows.MessageBox;
using Panel = System.Windows.Controls.Panel;

namespace FBE_CSharpUI
{

    public struct MyData
    {
        public string _fileName;
        public int _id;

        public MyData(string fileName, int id)
        {
            _fileName = fileName;
            _id = id;
        }
    }

    internal class Sidebar {

        private List<ImageSource> meshIcons = new List<ImageSource>();
        private List<int> ids = new List<int>();
        private List<string> fileNames = new List<string>();
        private List<string> iconNames = new List<string>();

        private static string defaultPath = "..\\..\\data\\";
        //private static string defaultPath = "..\\Robogami\\data\\";

        private string fileToLoad = "loadFinal.txt";
        private string protoPathDir = Sidebar.defaultPath + "proto2016\\"; //previously in protofinal, not proto2016
        private string iconPathDir = Sidebar.defaultPath + "ui_icons\\";
        private string iconPathDir1 = "\\" + Sidebar.defaultPath + "ui_icons\\";

        public Sidebar()
        {
            Setup();
        }

        public Sidebar(string partType)
        {
            Setup(partType);
        }

        public void Setup()
        {
            string line;          
            int counter = 0;
            // Read the file and parse all the information 
            System.IO.StreamReader file =
               new System.IO.StreamReader(iconPathDir + fileToLoad);
            while ((line = file.ReadLine()) != null)
            {
                string[] words = line.Split(' '); // sample words would be [p0, filename,1];
                string iconName = words[0];
                string fileName = words[1];
                int id = Convert.ToInt32(words[2]);
                iconNames.Add(iconName);
                fileNames.Add(fileName);
                ids.Add(id);
                counter++;
            }
            file.Close();
        }

        public void Setup(string partType)
        {
            string line;
            int counter = 0;
            // Read the file and parse all the information 
            System.IO.StreamReader file =
               new System.IO.StreamReader(iconPathDir + "load" + partType + ".txt");
            while ((line = file.ReadLine()) != null)
            {
                string[] words = line.Split(' '); // sample words would be [p0, filename,1];
                string iconName = words[0];
                string fileName = words[1];
                int id = Convert.ToInt32(words[2]);
                iconNames.Add(iconName);
                fileNames.Add(fileName);
                ids.Add(id);
                counter++;
            }
            file.Close();
        }

        public void LoadMeshIcons(string partType)
        {
            foreach (string iconName in iconNames)
            {
                //string imagePath = String.Format("\\..\\..\\data\\UI_protoPics\\{0}.png", iconName); //either way works
                string imagePath = iconPathDir1 + partType + "\\" + iconName + ".png";
                var source = new BitmapImage(new Uri(Environment.CurrentDirectory + imagePath));
                meshIcons.Add(source);
            }
        }

        public void PopulateContainer(Panel panel) {
            panel.Children.Clear();
            for (int i = 0; i < meshIcons.Count; i++) {
                var icon = meshIcons[i];
                Image image = new Image();
                image.Margin = new Thickness(4);
                image.Source = icon;
                RenderOptions.SetBitmapScalingMode(image, BitmapScalingMode.HighQuality);
                panel.Children.Add(image);
                image.Width = 96;
                image.Height = 96;
                int i1 = i;

                String fileName1 = protoPathDir + fileNames[i1] + "\\template.asciiproto";
                MyData objectData = new MyData(fileName1,ids[i]);
                
                image.MouseDown += (sender, args) => {
                    DataObject data = new DataObject("templateFilename", objectData);             
                    DragDrop.DoDragDrop(image, data, DragDropEffects.All);
                };
            }
            panel.SizeChanged -= panel_SizeChanged;
            panel.SizeChanged += panel_SizeChanged;
        }

        private void panel_SizeChanged(object sender, SizeChangedEventArgs e) {
//            foreach (var child in ((Panel) sender).Children) {
//                ((Image) child).Width = e.NewSize.Width/3;
//            }
        }
    }
}
