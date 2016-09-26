using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;
using CppCsBridge;

namespace FBE_CSharpUI
{
    public class TranslationCache {
        private readonly UIInstance _uiInstance;

        public TranslationCache(UIInstance uiInstance) {
            _uiInstance = uiInstance;
        }

        private Dictionary<TemplateRef, List<Visual3D>> elements = new Dictionary<TemplateRef, List<Visual3D>>();
        private Dictionary<TemplateRef, Vector3D> translations = new Dictionary<TemplateRef, Vector3D>();
        private Dictionary<TemplateRef, double> lowestZ = new Dictionary<TemplateRef, double>();

        public Dictionary<TemplateRef, double> LowestZ {
            get { return lowestZ; }
        }

        public void ClearElements() {
            elements.Clear();
            lowestZ.Clear();
        }

        public void AddElement(TemplateRef tmpl, Visual3D element) {
            if (!elements.ContainsKey(tmpl)) {
                elements[tmpl] = new List<Visual3D>();
            }
            elements[tmpl].Add(element);
        }

        public void Translate(TemplateRef tmpl, Vector3D by) {
            if (!translations.ContainsKey(tmpl)) {
                translations[tmpl] = new Vector3D(0, 0, 0);
            }
            translations[tmpl] += by;
            foreach (var visual in elements[tmpl]) {
                //Console.WriteLine("AAAAA: " + tmpl.Debuggable.getDebugInfo().shortDescription);
                Vector3D offset = this[tmpl];
                if (offset.Y + lowestZ[tmpl] < 0) {
                    offset.Y = -lowestZ[tmpl];
                }
                visual.Transform = new TranslateTransform3D(offset);
            }
        }

        public Vector3D this[TemplateRef tmpl] {
            get {
                if (translations.ContainsKey(tmpl)) {
                    return translations[tmpl];
                }
                return new Vector3D(0, 0, 0);
            }
        }

        public void Commit() {
            foreach (var pair in translations) {
                _uiInstance.Translate(pair.Key, pair.Value);
            }
            translations.Clear();
        }
    }
}
