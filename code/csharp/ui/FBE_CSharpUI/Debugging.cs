using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using CppCsBridge;
using External;
using System.Windows.Controls;

namespace FBE_CSharpUI
{
    
    class Debugging {
        private UIStates uiStates;
        private bool RENDER_DEBUGGING = false;
        public Debugging(UIStates uiStates) {
            this.uiStates = uiStates;
        }

        public void render(Dictionary<String, DebuggableRef> debuggables, TreeListView view) {
            if (RENDER_DEBUGGING)
            {
                foreach (var pair in debuggables)
                {
                    var name = pair.Key;
                    var root = pair.Value;
                    TreeViewItem node = null;
                    bool exists = false;
                    for (int i = 0; i < view.Items.Count; i++)
                    {
                        TreeViewItem t = (TreeViewItem) view.Items[i];
                        if (((DebugItem) t.Header).name == name)
                        {
                            node = t;
                            exists = true;
                        }
                    }
                    node = dfs(name, root, node);
                    if (!exists)
                    {
                        view.Items.Add(node);
                    }
                }
                List<TreeViewItem> toRemove = new List<TreeViewItem>();
                for (int i = 0; i < view.Items.Count; i++)
                {
                    TreeViewItem t = (TreeViewItem)view.Items[i];
                    if (!debuggables.ContainsKey(((DebugItem)t.Header).name))
                    {
                        toRemove.Add(t);
                    }
                }
                foreach (var item in toRemove)
                {
                    view.Items.Remove(item);
                }
            }
            
        }

        public class DebugItem {
            private readonly string _name;
            private readonly string _value;

            public string name {
                get { return _name; }
            }

            public string value {
                get { return _value; }
            }

            public DebugItem(string name, string value) {
                _name = name;
                _value = value;
            }
        }

        private Dictionary<DebuggableRef, TreeViewItem> itemMap = new Dictionary<DebuggableRef, TreeViewItem>(); 

        private TreeViewItem dfs(String rootName, DebuggableRef root, TreeViewItem existing)
        {
            var info = root.getDebugInfo();
            TreeViewItem item = existing ?? new TreeViewItem();
            item.Header = new DebugItem(rootName, info.shortDescription);
            item.MouseEnter += (sender, args) => {
                var over = Mouse.DirectlyOver;
                if (over is FrameworkElement && ((FrameworkElement)over).TemplatedParent is ContentPresenter && ((ContentPresenter)((FrameworkElement)over).TemplatedParent).Content == item.Header) {
                    item.Background = Brushes.Gold;
                    MaybeSelect(root);
                }
            };
            item.MouseLeave += (sender, args) =>
            {
                item.Background = null;
                Deselect();
            };

            item.PreviewMouseMove += (sender, args) =>
            {
                var over = Mouse.DirectlyOver;
                if (over is FrameworkElement && ((FrameworkElement)over).TemplatedParent is ContentPresenter && ((ContentPresenter)((FrameworkElement)over).TemplatedParent).Content == item.Header)
                {
                    item.Background = Brushes.Gold;
                    MaybeSelect(root);
                }
                else {
                    item.Background = null;
                    Deselect(root);
                }
            };
            itemMap[root] = item;
            foreach (var propName in info.propNames) {
                TreeViewItem child = null;
                bool exists = false;
                for (int i = 0; i < item.Items.Count; i++) {
                    TreeViewItem t = (TreeViewItem) item.Items[i];
                    if (((DebugItem) t.Header).name == propName) {
                        child = t;
                        exists = true;
                    }
                }
                if (info.props[propName] is DebuggableRef && info.aggregationPropNames.Contains(propName)) {
                    child = dfs(propName, (DebuggableRef) info.props[propName], child);
                } else if (info.props[propName] is DebuggableRef && !info.aggregationPropNames.Contains(propName)) {
                    // this is a reference link
                    child = child ?? new TreeViewItem();
                    var referenced = ((DebuggableRef)info.props[propName]);
                    if (referenced.IsNull) {
                        child.Header = new DebugItem(propName, "(null)");
                    }
                    else {
                        child.Header = new DebugItem(propName, "Reference to " + referenced.getDebugInfo().typeName);
                        child.MouseEnter += (sender, args) =>
                        {
                            child.Background = Brushes.Gold;
                            MaybeSelectReference(referenced);
                        };
                        child.MouseLeave += (sender, args) =>
                        {
                            child.Background = null;
                            Deselect();
                        };
                    }
                }
                else {
                    child = child ?? new TreeViewItem();
                    child.Header = new DebugItem(propName, info.props[propName].ToString());
                }
                if (!exists) {
                    item.Items.Add(child);
                }
            }
            List<TreeViewItem> copy = item.Items.Cast<TreeViewItem>().ToList();
            item.Items.Clear();
            foreach (var name in info.propNames)
            {
                item.Items.Add((from it in copy where (it.Header as DebugItem).name == name select it).First());
            }
            return item;
        }

        private void Deselect(DebuggableRef dr) {
            if (uiStates.DebugSelection.Type == Selection.SelectionType.Template &&
                uiStates.DebugSelection.SelectedTemplate.Debuggable.Equals(dr) ||
                uiStates.DebugSelection.Type == Selection.SelectionType.Patch &&
                !dr.AsPatch.IsNull && dr.AsPatch.Equals(uiStates.DebugSelection.SelectedPatch)) {
                    uiStates.DebugSelection = Selection.Empty;
                    foreach (var item in itemMap.Values)
                    {
                        item.Background = null;
                    }
            }
        }

        private void Deselect()
        {
            uiStates.DebugSelection = Selection.Empty;
            foreach (var item in itemMap.Values)
            {
                item.Background = null;
            }
        }

        private void MaybeSelect(DebuggableRef root) {
            if (!root.AsTemplate.IsNull) {
                uiStates.DebugSelection = Selection.Template(root.AsTemplate);
            }
            else if (!root.AsPatch.IsNull) {
                uiStates.DebugSelection = Selection.Patch(root.AsPatch.OwningTemplate, root.AsPatch);
            }
        }

        private void MaybeSelectReference(DebuggableRef referenced) {
            MaybeSelect(referenced);
            if (itemMap.ContainsKey(referenced)) {
                itemMap[referenced].Background = Brushes.YellowGreen;
                TreeViewItem item = itemMap[referenced].Parent as TreeViewItem;
                while(item != null) {
                    item.Background = Brushes.Aquamarine;
                    item = item.Parent as TreeViewItem;
                }
            }
        }

        public static readonly DependencyProperty DebuggingTagProperty = DependencyProperty.RegisterAttached(
            "DebuggingTag", typeof (object), typeof (Debugging), new PropertyMetadata(default(object)));

        public static void SetDebuggingTag(DependencyObject element, object value) {
            element.SetValue(DebuggingTagProperty, value);
        }

        public static object GetDebuggingTag(DependencyObject element) {
            return (object) element.GetValue(DebuggingTagProperty);
        }
    }
}
