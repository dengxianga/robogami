using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using CppCsBridge;

namespace FBE_CSharpUI
{
    public class UIStates
    {
        private Selection _selection;
        private Selection _debugSelection;
        public bool IsRotating { get; set; }
        public bool IsScaling { get; set; }
        public bool IsTranslatingPart { get; set; }
        public bool IsTranslating { get; set; }
        public bool IsMeasuring { get; set; }
        public bool IsGuidingManipulation { get; set; }
        public bool DisableExpensiveOperations { get; set; }
        public bool DisableAllStabilityGuidance { get; set; }
        public bool AlwaysSnapToGround { get; set; }
        public bool AvoidObstacle { get; set; }
        public bool DesignCar { get; set; }
        public bool ForwardTravel { get; set; }
        public int UserStudy_ID {get; set;} 
        public int NAnimRounds { get; set; }
        public double DeltaAnim { get; set; }
        public string saveFileName { get; set; }
        public bool showGhost { get; set; }
        public bool useSteadyState { get; set; }
        public int motionSequenceLen { get; set; } 
        public SnappingState SnappingState { get; set; }
        public List<Brush> niceColors { get; set; }
        public List<Color> niceColorsNoBrush { get; set; }
        public int selecteMetric_Gait { get; set; }
        public int selecteMetric_Objective { get; set; }
        public int taskId { get; set; }
        public string taskLogFilename { get; set; }

        public bool IsInitialDragging { get; set; }
        public TranslationCache TranslationCache { get; set; }

        // Indicates that the next Draw() call should compute the exported mesh.
        public bool ExportMesh { get; set; }
        // Map from "independent" templates to its mesh.
        public Dictionary<TemplateRef, Model3DGroup> ExportedMesh { get; set; }

        // Last drawn meshes, used for optimization to avoid unnecessary geometry recomputation.
        public List<Mesh> LastDrawnMeshes { get; set; }


        public Selection Selection
        {
            get { return _selection; }
            set
            {
                _selection = value;
                Action temp = SelectionChanged;
                if (temp != null)
                {
                    temp();
                }
            }
        }

        public event Action SelectionChanged;


        public Selection DebugSelection
        {
            get { return _debugSelection; }
            set
            {
                _debugSelection = value;
                Action temp = DebugSelectionChanged;
                if (temp != null)
                {
                    temp();
                }
            }
        }

        public bool PreventCollisions { get; set; }
        public bool ShowOverlaps { get; set; }

        public event Action DebugSelectionChanged;
    }

    public class SnappingState
    {
        public PatchPair PatchPair { get; set; }
        public SnappingThreshold Threshold { get; set; }
    }

    public class SnappingThreshold
    {
        const double INITIAL_THRESHOLD_DISTANCE = 100;
        public double MaxDistance { get; set; }
        public SnappingThreshold Decrease()
        {
            return new SnappingThreshold { MaxDistance = MaxDistance / 1.5 };
        }
        public static SnappingThreshold InitialThreshold
        {
            get
            {
                return new SnappingThreshold { MaxDistance = INITIAL_THRESHOLD_DISTANCE };
            }
        }
    }

}
