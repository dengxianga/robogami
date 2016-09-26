#ifndef _FOLDABLE_GRAPH_H
#define _FOLDABLE_GRAPH_H

#include <stdio.h>
#include <list>
#include <math.h>

#include "symbolic.h"
#include "Printable.h"
#include "PrintingParameters.h"

namespace FabByExample
{

	class Template; 
	class NewPatch; 
	class Template; 
	class NewPatchLine2D3D;

	struct MotorMountLocations{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Eigen::Vector2d point1;
		Eigen::Vector2d dir;
		int orientation;
		std::vector<double> spacings;
	};

	class FoldableNode{
	protected:
		Template* refTemp;
		int nodeId;
	public:
		FoldableNode(Template* _refTemp, int id){
			refTemp = _refTemp;
			nodeId = id;
		}
		Template* getRefTemp(){return refTemp;}
		virtual bool checkHasLinePatch(NewPatch * patch, int * newEdgeID) = 0;
		int getID(){return nodeId;}
	};

	class FoldableNode_Face : public FoldableNode{
	public:
		Face2S* face; 
		FoldableNode_Face(Template* _refTemp, int id, Face2S* _face): FoldableNode(_refTemp, id){
			face = _face;
		}
		bool checkHasLinePatch(NewPatch * patch, int * newEdgeID);

	};

	class FoldableNode_Mesh : public FoldableNode{
	public:
 		Mesh3S* mesh;
 		NewPatchLine2D3D* myPatch;
 		NewPatchLine2D3D* connectedPatch;
 
		FoldableNode_Mesh(Template* _refTemp, int id, Mesh3S* _mesh, NewPatchLine2D3D* _myPatch, NewPatchLine2D3D* _connectedPatch): FoldableNode(_refTemp, id){
			myPatch = _myPatch;
			connectedPatch = _connectedPatch;
			mesh = _mesh;
 		}
 		bool checkHasLinePatch(NewPatch * patch, int * newEdgeID);
 
	};

	class FoldableEdge{
	public:
		enum FoldableEdgeType{FOLD, TEETH,TOUCH,HINGE,BALLJOINT,PRISMATIC,NONPRINTED, FILL};
		virtual FoldableEdgeType getType() = 0;
		double getAngle(){return angle;}
		FoldableEdge(double _angle){angle = _angle;}
		std::vector<FoldableNode*> connectedNodes;
		static double getTotalShrinkAmount(FoldableEdgeType type, double ang);
	protected:
		double angle;
	};

	class FoldableEdge_LineConn : public FoldableEdge{
	public:
		int edgeID1;
		int edgeID2;
		FoldableEdge_LineConn(double _angle, FoldableNode* _node1, FoldableNode* _node2,
							int _edgeID1, int _edgeID2, FoldableEdge::FoldableEdgeType _type);

		FoldableEdgeType getType(){return type;}
	private: 
		FoldableEdgeType type;
	};


	class FoldableGraph {
	private:
		std::vector<FoldableNode*> nodes;
		std::vector<FoldableEdge*> edges;

	public:

		FoldableGraph(Template* temp);
		~FoldableGraph();

		void display(); 
		bool getPatchLineCorrespondance(NewPatch * patch, FoldableNode** node1, int* edgeID1);
		//void generateSVG(SymbolicAssignment sa, int width, int height, std::string path);
		PrintableDesign generatePrintableDesign();
	};

}
#endif