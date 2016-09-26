#include "newSnapping.h"
#include "template.h"
#include "templateElement.h"
#include "constraints.h"
#include "myOptimizer.h"
#include "element.h"
#include "ConstraintsEval.h"
#include "ConnectingInfo.h"
#include <unordered_set>
#include "NewConnection.h"
#include "element_symbolic.h"

using namespace FabByExample;

std::vector<NewPatch*> getPatchesForTemplate(Template* temp) {
	std::vector<TemplateElement*> elements;
	std::vector<NewPatch*> patches;
	temp->getAllElementTempList(elements);
	for (int i = 0; i < elements.size(); i++) {
		auto element = elements[i];
		auto element_patches = element->getPatches();
		auto size = element_patches.size();
		for (int j = 0; j < size; j++){
			patches.push_back(element_patches[j]);
		}
	}
	if (patches.size() == 0){
		throw 0;
	}
	return patches;
}

std::unordered_set<NewPatch*> getPatchSetForTemplate(Template* temp) {
	std::vector<TemplateElement*> elements;
	std::unordered_set<NewPatch*> patches;
	temp->getAllElementTempList(elements);
	for (int i = 0; i < elements.size(); i++) {
		auto element = elements[i];
		auto element_patches = element->getPatches();
		auto size = element_patches.size();
		for (int j = 0; j < size; j++){
			patches.insert(element_patches[j]);
		}
	}
	if (patches.size() == 0){
		throw 0;
	}
	return patches;
}

std::vector<NewPatch*>  getConnectedPatchesForTemplate(Template* tmpl){

	std::vector<NewConnection*> connections;
	std::unordered_set<NewPatch*> uniquePatches;
	std::unordered_set<NewPatch*> patchesInCurrentTmpl = getPatchSetForTemplate(tmpl);

	if (tmpl->getID()!= 0){
		tmpl = tmpl->getRoot();
	}
	tmpl->getConnections(connections, TreeScope::DESCENDANTS);
	
	for (int i = 0; i < connections.size(); i++) {
		NewConnection* c = connections[i];
		std::vector<NewPatch*> ps = c->getPatches();
		for each (NewPatch* p in ps){
			// only insert if the patch has connection and is in this template
			if (uniquePatches.find(p) == uniquePatches.end() && patchesInCurrentTmpl.find(p) != patchesInCurrentTmpl.end()){
				uniquePatches.insert(p);
			}
		}
		
	}
	
	
	std::vector<NewPatch*> patches;
	for (auto itr = uniquePatches.begin(); itr != uniquePatches.end(); ++itr) {
		patches.push_back(*itr);
		NewPatchLine2D3D* edge = dynamic_cast<NewPatchLine2D3D*>((*itr));		
	}

	if (patches.size() == 0){
		throw 0;
	}
	return patches;
}

NewSnapping::NewSnapping(Template* mainTemplate, Template* addTemplate, bool usingContext){
	this->mainTemplate = mainTemplate;
	this->addTemplate = addTemplate;
	std::vector<NewPatch*> mainPatches;
	std::vector<NewPatch*> additionalPatches;

	//initialize the data

	// we will only get the list of patches that have some connections in the example
	if (usingContext){
		mainPatches = getConnectedPatchesForTemplate(mainTemplate);
		additionalPatches = getConnectedPatchesForTemplate(addTemplate);

		for (int i = 0; i < mainPatches.size(); i++) {
			NewPatchLine2D3D* edge = dynamic_cast<NewPatchLine2D3D*>(mainPatches[i]);
			if (edge != nullptr) {
				mainPatchEdges.push_back(edge);
			}
		}

		for (int i = 0; i < additionalPatches.size(); i++) {
			NewPatchLine2D3D* edge = dynamic_cast<NewPatchLine2D3D*>(additionalPatches[i]);
			if (edge != nullptr) {
				additionalPatchEdges.push_back(edge);
			}
		}

	}
	
	//we will simply get all the patches from both templates
	
	else{
	
		mainPatches = getPatchesForTemplate(mainTemplate);
		additionalPatches = getPatchesForTemplate(addTemplate);

		for (int i = 0; i < mainPatches.size(); i++) {
			NewPatchLine2D3D* edge = dynamic_cast<NewPatchLine2D3D*>(mainPatches[i]);
			if (edge != nullptr) {
				mainPatchEdges.push_back(edge);
			}
		}

		for (int i = 0; i < additionalPatches.size(); i++) {
			NewPatchLine2D3D* edge = dynamic_cast<NewPatchLine2D3D*>(additionalPatches[i]);
			if (edge != nullptr) {
				additionalPatchEdges.push_back(edge);
			}
		}
	}

}




NewSnapping::~NewSnapping(){
		mainPatchEdges.clear();
		additionalPatchEdges.clear();	
}





void NewSnapping::snap(PatchPair const& current, double maxDistance, ConnectingInfo & connectingInfo ) {

	connectingInfo.clear();

	// Edges:
	double minEdgeDist = 1e300;
	NewPatchLine2D3D *minMainEdge = nullptr, *minAdditionalEdge = nullptr;
	NewPatchLine2D3D::DirectionResult minDir;
	if (current.wt_patch != nullptr) {
		minMainEdge = dynamic_cast<NewPatchLine2D3D*>(current.wt_patch);
		minAdditionalEdge = dynamic_cast<NewPatchLine2D3D*>(current.add_patch);
		minDir = current.isOpposite ? NewPatchLine2D3D::DirectionResult::OPPOSITE : NewPatchLine2D3D::DirectionResult::SAME;
	}
	else {
		for (int i = 0; i < mainPatchEdges.size(); i++) {
			for (int j = 0; j < additionalPatchEdges.size(); j++) {
				NewPatchLine2D3D* mainEdge = mainPatchEdges[i];
				NewPatchLine2D3D* additionalEdge = additionalPatchEdges[j];
				auto dir = mainEdge->matchDirection(additionalEdge);
				if (dir == NewPatchLine2D3D::DirectionResult::DIFFERENT) {
					continue;
				}
				if (mainEdge->parentConnection != nullptr && !mainEdge->parentConnection->isIncomplete()
					|| additionalEdge->parentConnection != nullptr && !additionalEdge->parentConnection->isIncomplete()) {
					continue;
				}
				double dist = abs(mainEdge->getDistanceToPatch(additionalEdge));
				if (dist < minEdgeDist) {
					minEdgeDist = dist;
					minMainEdge = mainEdge;
					minAdditionalEdge = additionalEdge;
					minDir = dir;
				}
			}
		}
	}
	if (minMainEdge != nullptr && minAdditionalEdge != nullptr){
		ConstraintsEval::updateTemplateForSnappingWithEdges(mainTemplate,
			addTemplate, minMainEdge, minAdditionalEdge, minDir == NewPatchLine2D3D::DirectionResult::OPPOSITE, minMainEdge->isDrawingEdgeCounterClockwise() != minAdditionalEdge->isDrawingEdgeCounterClockwise(), maxDistance, connectingInfo);
		//connectingInfo.addPatchPair(minMainEdge, minAdditionalEdge, minDir == NewPatchLine2D3D::DirectionResult::OPPOSITE);
	}
}



/*
void NewSnapping::snap(ConnectingInfo & connectingInfo ) {

	connectingInfo.clear();

	auto currentQ = addTemplate->getFullQ();
	std::vector<Mesh3S> meshesBackup;
	std::vector<Drawing2S> drawingsBackup;
	std::vector<Element*> elements;
	addTemplate->getAllElementList(elements);
	for each (auto ele in elements) {
		meshesBackup.push_back(*(dynamic_cast<Element_Symbolic*>(ele)->getMesh3S()));
		drawingsBackup.push_back(*(dynamic_cast<Element_Symbolic*>(ele)->getDrawing2S()));
	}

	auto restore = [&]() {
		addTemplate->updateFullQ(currentQ);
		for (int i = 0; i < elements.size(); i++) {
			*dynamic_cast<Element_Symbolic*>(elements[i])->getMesh3S() = meshesBackup[i];
			*dynamic_cast<Element_Symbolic*>(elements[i])->getDrawing2SNonConst() = drawingsBackup[i];
		}
	};

	// Edges:
	double minCost = 1e300;
	NewPatchLine2D3D *minMainEdge = nullptr, *minAdditionalEdge = nullptr;
	addTemplate->pauseRecalculation = true;
	for (int i = 0; i < mainPatchEdges.size(); i++) {
		for (int j = 0; j < additionalPatchEdges.size(); j++) {
			for (int k = 0; k < 2; k++) {
				NewPatchLine2D3D* mainEdge = mainPatchEdges[i];
				NewPatchLine2D3D* additionalEdge = additionalPatchEdges[j];
				if (abs(mainEdge->getDirection().dot(additionalEdge->getDirection())) < 0.9) continue;
				if (k == 1) additionalEdge = additionalEdge->flip();
				double cost = ConstraintsEval::updateTemplateForSnappingWithEdges(mainTemplate, addTemplate, mainEdge, additionalEdge);
				restore();
				if (cost < minCost) {
					minCost = cost;
					minMainEdge = mainEdge;
					minAdditionalEdge = additionalEdge;
				}
			}
		}
	}
	addTemplate->pauseRecalculation = false;
	
	printf("MIN COST IS %lf", minCost);
	if (minMainEdge != nullptr && minAdditionalEdge != nullptr) {	
			ConstraintsEval::updateTemplateForSnappingWithEdges(mainTemplate,
				addTemplate, minMainEdge, minAdditionalEdge);
			connectingInfo.addPatchPair(minMainEdge, minAdditionalEdge);
	}
}

*/

class RecursionHelper {
private:
	typedef NewPatchLine2D3D Patch;
	vector<PatchPair> pairs;
	unordered_set<Patch*> chosen;
	int N;
public:
	const vector<Patch*>* addEdges;
	const vector<vector<Patch*>>* mainPatches;
	int numClosestPatches;
	std::function<void(const vector<PatchPair>&)> callback;

private:
	void recurse(int i) {
		const auto& mainPatches = *this->mainPatches;
		const auto& addEdges = *this->addEdges;
		if (i == N) {
			callback(pairs);
		}
		else {
			int tried = 0;
			int index = 0;
			while (tried < numClosestPatches && index < mainPatches[i].size()) {
				if (chosen.find(mainPatches[i][index]) != chosen.end()) {
					index++;
					continue;
				}
				Patch* mainEdge = mainPatches[i][index];
				Patch* addEdge = addEdges[i];
				auto dir = mainEdge->matchDirection(addEdge);
				pairs.push_back(PatchPair(mainEdge, addEdge,
					dir == NewPatchLine2D3D::DirectionResult::OPPOSITE,
					dynamic_cast<NewPatchLine2D3D*>(mainEdge)->isDrawingEdgeCounterClockwise()
					!= dynamic_cast<NewPatchLine2D3D*>(addEdge)->isDrawingEdgeCounterClockwise()));
				tried++;
				index++;
				chosen.insert(mainEdge);

				recurse(i + 1);

				pairs.pop_back();
				chosen.erase(chosen.find(mainEdge));
			}
			pairs.push_back(PatchPair(nullptr, nullptr, false, false));
			recurse(i + 1);
			pairs.pop_back();
		}
	}

public:
	void go() {
		N = addEdges->size();
		pairs.clear();
		chosen.clear();
		recurse(0);
	}

};


void NewSnapping::multiSnap(double maxDistance, ConnectingInfo* connectingInfo, int numClosestPatches) {
	typedef NewPatchLine2D3D Patch;
	connectingInfo->clear();

	// Edges:
	std::vector<NewConnection*> connections;
	addTemplate->getConnections(connections, TreeScope::DESCENDANTS);
	std::vector<NewConnection*> halfOpenConns;
	for each (auto conn in connections) {
		for each (auto patch in conn->getPatches()) {
			if (dynamic_cast<PlaceHolderPatch*>(patch) != nullptr) {
				halfOpenConns.push_back(conn);				
				break;
			}			
		}
		
	}

	std::vector<Patch*> halfOpenPatches;
	for each (auto conn in halfOpenConns) {
		for each (auto patch in conn->getPatches()) {
			if (dynamic_cast<Patch*>(patch) != nullptr) {
				halfOpenPatches.push_back(dynamic_cast<Patch*>(patch));
				break;
			}
		}
	}
	
	LOG(INFO) << "Found " << halfOpenPatches.size() << " half-open patches in the additional template";

	// Contain the list of eligible patches for every half open patch
	std::vector<std::vector<Patch*> > mainPatches;

	for (int i = 0; i < halfOpenPatches.size(); i++) {
		std::vector<Patch*> patches;
		auto addEdge = halfOpenPatches[i];
		for (int j = 0; j < mainPatchEdges.size(); j++) {
			auto mainEdge = mainPatchEdges[j];
			auto dir = mainEdge->matchDirection(addEdge);
			if (dir == Patch::DirectionResult::DIFFERENT) {
				continue;
			}
			if (mainEdge->parentConnection != nullptr && !mainEdge->parentConnection->isIncomplete()) {
				continue;
			}
			patches.push_back(mainEdge);
		}

		sort(patches.begin(), patches.end(), [=](Patch* a, Patch* b) -> bool {
			double aDist = abs(a->getDistanceToPatch(addEdge));
			double bDist = abs(b->getDistanceToPatch(addEdge));
			return aDist < bDist;
		});

		LOG(INFO) << "Additional Patch #" << i << " has " << patches.size() << " eligible patches in main";
		while (patches.size() > numClosestPatches) {
			patches.pop_back();
		}
		
		mainPatches.push_back(patches);
	}

	for (int i = halfOpenPatches.size() - 1; i >= 0; i--) {
		if (mainPatches[i].empty()) {
			halfOpenPatches.erase(halfOpenPatches.begin() + i);
			mainPatches.erase(mainPatches.begin() + i);
		}
	}

	LOG(INFO) << halfOpenPatches.size() << " half-open patches have eligible patches in main";

	// Try to snap the given pairs.
	// If success, return the cost
	// If fail, return -1
	auto trySnap = [=](std::vector<PatchPair> const& pairsToSnap) -> double {
		return ConstraintsEval::updateTemplateForMultiSnappingWithEdges(mainTemplate, addTemplate, pairsToSnap, false);
	};

	int total = 1;
	int N = halfOpenPatches.size();
	for (int i = 0; i < N; i++) {
		total *= mainPatches[i].size();
	}

	double minCost = numeric_limits<double>::max();
	std::vector<PatchPair> bestPairs;

	RecursionHelper helper;
	helper.addEdges = &halfOpenPatches;
	helper.mainPatches = &mainPatches;
	helper.callback = [&](const vector<PatchPair>& pairs) -> void {

		double badnessForNotSnapping = 10000;

		double notSnappingBadness = 0;
		std::vector<PatchPair> snappingPairs;
		for each (auto const& pp in pairs) {
			if (pp.wt_patch == nullptr) {
				notSnappingBadness += badnessForNotSnapping;
			}
			else {
				snappingPairs.push_back(pp);
			}
		}
		double cost = trySnap(snappingPairs) + notSnappingBadness;
		if (cost != -1 && cost < minCost) {
			minCost = cost;
			bestPairs = snappingPairs;
		}
	};
	helper.numClosestPatches = numClosestPatches;
	helper.go();

	for each (auto patchPair in bestPairs) {
		connectingInfo->addPatchPair(patchPair.wt_patch, patchPair.add_patch, patchPair.isOpposite, patchPair.isOpposite2D);
	}

	ConstraintsEval::updateTemplateForMultiSnappingWithEdges(mainTemplate, addTemplate, bestPairs);
}