using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using CppCsBridge;

namespace FBE_CSharpUI
{
    public struct Selection
    {
        public enum SelectionType
        {
            Face,
            Edge,
            Template,
            Patch,
            None
        };

        private readonly SelectionType _type;
        private readonly TemplateRef _selectedTemplate;
        private readonly int? _selectedEdgeId;
        private readonly PatchRef _selectedPatch;

        public TemplateRef SelectedTemplate
        {
            get { return _selectedTemplate; }
        }

        public PatchRef SelectedPatch
        {
            get { return _selectedPatch; }
        }
        
        public int? SelectedEdgeId
        {
            get { return _selectedEdgeId; }
        }

        public SelectionType Type
        {
            get { return _type; }
        }

        public bool IsEmpty
        {
            get { return _type == SelectionType.None; }
        }

        private Selection(SelectionType type, TemplateRef selectedTemplate, int? selectedEdgeId, PatchRef selectedPatch)
        {
            _selectedTemplate = selectedTemplate;
            _selectedPatch = selectedPatch;
            _selectedEdgeId = selectedEdgeId;
            _type = type;
        }

        public static Selection Face(TemplateRef template)
        {
            return new Selection(SelectionType.Face, template, null, PatchRef.Null);
        }

        public static Selection Template(TemplateRef template)
        {
            return new Selection(SelectionType.Template, template, null, PatchRef.Null);
        }

        public static Selection Edge(TemplateRef template, int edgeId)
        {
            return new Selection(SelectionType.Edge, template, edgeId, PatchRef.Null);
        }

        public static Selection Patch(TemplateRef template, PatchRef patchRef)
        {
            return new Selection(SelectionType.Patch, template, null, patchRef);
        }

        public static readonly Selection Empty = new Selection(SelectionType.None, TemplateRef.Null, null, PatchRef.Null);
    }
}
