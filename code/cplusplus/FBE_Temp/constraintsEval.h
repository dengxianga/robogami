#pragma once
#include "NewPatch.h"
#include "../FBE_Manipulations/ConnectingInfo.h"

namespace FabByExample{
class Template;
class TemplateElement;
class Constraint;
class NewPatchLine2D3D;
class FlatLinearSystem;
class SymbolicController;
class PWLinearController;
class KinChain;

class ConstraintsEval{
public:
	static void updateTempByChangingOneParam(Template* tmpl, int parameter, double displacement);
	static void updateTemplateBySubElementFaceScaling(TemplateElement* tempElement, int subElementId, int axis, double scale, PatchPair const& snappingConstraint);
	static void updateTemplateBySubElementFaceScalingWithFixedAmount(TemplateElement* tempElement, int subElementId, int axis, double scale, PatchPair const& snappingConstraint);
	static double ConstraintsEval::updateTemplateForSnapping(Template* mainTemplate, Template* addTemplate, 
	Template* result, std::vector<Constraint*> snapconstraints);
	
	static void updateTemplateBySubElementFaceTranslate(KinChain* kinChain, TemplateElement* tempElement, int subElementId, int axis, double scale);
	
	static double updateTemplateForSnappingWithEdges(Template* mainTemplate, Template* addTemplate, NewPatchLine2D3D* mainEdge, NewPatchLine2D3D* addEdge, 
		bool isOpposite, bool isOpposite2D, double maxDistance, ConnectingInfo & connInfo);
	static double updateTemplateForMultiSnappingWithEdges(Template* mainTemplate, Template* addTemplate, const vector<PatchPair>& patchPairs, bool actuallyUpdate=true);

	static void enforceConstraints(Template* tmpl);
	static void enforceConstraintsSymmetry(Template* tmpl, bool isSpacing);
	static void enforceConstraintsConnect(Template* tmpl);

	static void extractFeasibilityConstraints(Template * tmpl, FlatLinearSystem* flat); 

	static void putContactPointsOnTheGround(Template* tmpl); 
	static void optimizeController(SymbolicController * symbController, PWLinearController* linearController);
	static void extractFeasibilityConstraintsFixingSomePameters(Template* tmpl, std::vector<Symbol> & fixSymbols, FlatLinearSystem* flat);


};
}
