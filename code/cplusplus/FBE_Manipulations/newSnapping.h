#ifndef _NEW_SNAPPING_
#define _NEW_SNAPPING_

#include <Eigen/Dense>
#include <list>
#include <vector>
#include "NewPatch.h"

namespace FabByExample{
class ConstraintTransfer;
class ValidFits;
class ConstraintData;
class NewPatch;
class Constraint;
class Template;
class ConnectingInfo;
class NewPatchLine2D3D;


class NewSnapping{
public:
	NewSnapping(Template* mainTemplate, Template* addTemplate, bool usingContext = false);
	~NewSnapping();
	void snap(PatchPair const& current, double maxDistance, ConnectingInfo & connectingInfo);
	void multiSnap(double maxDistance, ConnectingInfo* connectingInfo, int numClosestPatches=1);
	
private:
	Template* mainTemplate;
	Template* addTemplate;
	std::vector<NewPatchLine2D3D*> mainPatchEdges;
	std::vector<NewPatchLine2D3D*> additionalPatchEdges;

};


}
#endif