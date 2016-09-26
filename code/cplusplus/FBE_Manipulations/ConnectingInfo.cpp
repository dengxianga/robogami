#include "ConnectingInfo.h"
#include "NewPatch.h"



using namespace FabByExample;


ConnectingInfo::ConnectingInfo(){

}

ConnectingInfo::~ConnectingInfo(){
	matchingNewPatches.clear();
	connectingPoints.clear();
}

void ConnectingInfo::clear(){
	matchingNewPatches.clear();
	connectingPoints.clear();
}


void ConnectingInfo::addPatchPair(NewPatch* wt_patch, NewPatch * add_patch, bool isOpposite, bool isOpposite2D){
	PatchPair newPatchPair(wt_patch, add_patch, isOpposite, isOpposite2D);
	matchingNewPatches.push_back(newPatchPair);

}
