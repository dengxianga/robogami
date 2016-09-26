#define testingFunc true
#include "abstractUI.h"
#include "TemplateProtoConverter.h"
#include "element_symbolic.h"
#include "../FBE_Printable/FoldableGraph.h"
#include "../FBE_Temp/articulation.h"
#include "../FBE_Kinematics/KinChain.h"
#include <TemplateManipulations.h>
#include <templateElement.h>
#include "../FBE_Temp/geometry.h"

#define DEBUG false

using namespace FabByExample;
using namespace std;

static void initializeLogging() {
	google::InitGoogleLogging("FabByExample");

	// This sets the minimum level of logging to output. The severity level goes as:
	//  google::INFO < google::WARNING < google::ERROR.
	// So for example if you set this to google::WARNING, then LOG(INFO) messages will
	// not show up. VLOG(x) counts as INFO.
	google::SetStderrLogging(google::INFO);

	// Set the verbose logging level. Only VLOG(x) where x <= this value will show up.
	FLAGS_v = 4;
}

abstractUI::abstractUI() {
	initializeLogging();
}

//WEI TO DO: ask why do evalMesh
unordered_map<Template*, ::TriMesh*> abstractUI::getMeshes(){
	PROFILE_THIS(__FUNCTION__);

	unordered_map<Template*, ::TriMesh*> meshes;
	for each (auto workingTemplate in workingTemplates) {
		vector<Element*> elements;
		workingTemplate->getTemplate()->getAllElementList(elements);
		for each(Element* e in elements){
			Geometry* g = e->getGeo();
			meshes[e->getRefTemplateElement()] = g->getMesh();
		}
	}
	return meshes;
}

/*
unordered_map<Template*, ::TriMesh*> abstractUI::getKinMeshes(){
	PROFILE_THIS(__FUNCTION__);

	unordered_map<Template*, ::TriMesh*> meshes;
	if(workingTemplates.size() >0){
		auto workingTemplate = workingTemplates[0];
		vector<Element*> elements;
		workingTemplate->getTemplate()->getAllElementList(elements);
		for each(Element* e in elements){
			Geometry* g = e->getGeo();
			meshes[e->getRefTemplateElement()] = g->getMesh();
		}
	}
	return meshes;
}
*/


drawing::Drawing abstractUI::getTopViewDrawing(){
	PROFILE_THIS(__FUNCTION__);

	
	if(workingTemplates.size() >0){
		auto workingTemplate = workingTemplates[0];
		return workingTemplate->getTopViewDrawing();
	}
	drawing::Drawing d;
	return d;
}

/*
Return a map from each element template to its drawing. Includes both the main template
and the additional template (whatever exists).
*/
unordered_map<Template*, const drawing::Drawing*> abstractUI::getDrawings() {
	PROFILE_THIS(__FUNCTION__);
	unordered_map<Template*, const drawing::Drawing*> result;
	vector<Template*> roots;
	for each (auto workingTemplate in workingTemplates) {
		roots.push_back(workingTemplate->getTemplate());
	}

	for each (Template* root in roots) {
		vector<Element*> elements;
		root->getAllElementList(elements);
		for (int i = 0; i < elements.size(); i++) {
			auto ele = elements[i];
			auto symb = dynamic_cast<Element_Symbolic*>(ele);
			if (symb != nullptr) {
				result[ele->getRefTemplateElement()] = symb->evalDrawing();
			}
		}
	}
	return result;
}

Template* loadTemplateFromProto(int id) {
	TemplateProtoConverter converter;
	ostringstream filename;
	filename << "..\\..\\data\\UI_protoPics\\temp_" << id << ".asciiproto";
	auto ts = converter.loadFromFile(filename.str().c_str(), true);
	//auto ts = converter.loadFromFile()
	return converter.ConvertToTemplate(*ts);
}

Template* loadTemplateFromProtoByFilename(string filename) {
	TemplateProtoConverter converter;
	auto ts = converter.loadFromFile(filename.c_str());
	if(ts == nullptr)
		return nullptr;
	else{
		return converter.ConvertToTemplate(*ts);
	}
}


WorkingTemplate* abstractUI::loadAdditionalProtoTemplateByFilename(string filename, int templateID) {

	if (DEBUG) {
		LOG(INFO) << "Loading template with filename " << filename << " with id " << templateID;

		LOG(INFO) << "Loading standard mesh...";
	}
	auto tmpl = loadTemplateFromProtoByFilename(filename);
	
	tmpl->updateFullQ(tmpl->getFullQ());
	Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
	Eigen::Vector3d z = Eigen::Vector3d::Zero(3);
	z[2] = 1;
	Eigen::Vector3d y = Eigen::Vector3d::Zero(3);
	y[1] = 1;
	Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(z, y);
	//tmpl->rotate(center, rotation); 
	tmpl->recomputeCenter();
	tmpl->translate(-tmpl->getCenter());
	tmpl->updateFullQ(tmpl->getFullQ());
	
	bool loadGaits = false;
	if(workingTemplates.size() == 0){
		loadGaits = true;
	}
	//translate template up to the ground
	tmpl->moveToGround();
	auto childTmpl = tmpl->findDescendantByID(templateID);
	childTmpl->segregate();
	auto workingTemplate = new WorkingTemplate(childTmpl);


	if(loadGaits){
		TemplateProtoConverter converter;
		auto ts = converter.loadFromFile(filename.c_str());
		converter.AddGaitInfoToKinChain(*ts, workingTemplate->kinchain);
	}



	workingTemplates.push_back(workingTemplate);
	if (DEBUG) {
		LOG(INFO) << "Done loading standard mesh...";
	}
	return workingTemplate;
}

WorkingTemplate* abstractUI::addTemplateByFilename(string filename, int templateID, int x, int y) {
	WorkingTemplate* wt = loadAdditionalProtoTemplateByFilename(filename, templateID);
	return wt;
}

void abstractUI::saveProto(std::string filename){

	TemplateProtoConverter converter;
	proto::TemplateSet* proto = converter.ConvertToProto(workingTemplates[0]->tmpl);
	converter.AddGaitInfoToProto( workingTemplates[0]->kinchain, proto); 
	converter.saveToFile(*proto, filename.c_str(), true);
	delete proto;

}


void abstractUI::handleClear() {
	workingTemplates.clear();
}

PatchPair abstractUI::handleSnap(PatchPair const& current, double maxDistance) {
	if (workingTemplates.size() < 2) {
		if (DEBUG) {
			LOG(WARNING) << "Not snapping; 2 templates required.";
		}
		return PatchPair(nullptr, nullptr, false, false);
	}
	if (workingTemplates.size() > 2) {
		if (DEBUG) {
			LOG(WARNING) << "More than 2 templates are present. Snapping using the oldest two";
		}
	}
	return TemplateManipulations::Snap(
		workingTemplates[0]->getTemplate(),
		workingTemplates[1]->getTemplate(),
		current,
		maxDistance,
		connectingInfo);
}

void abstractUI::handleMultiSnap(double maxDistance) {
	if (workingTemplates.size() < 2) {
		if (DEBUG) {
			LOG(WARNING) << "Not snapping; 2 templates required.";
		}
		return;
	}
	if (workingTemplates.size() > 2) {
		if (DEBUG) {
			LOG(WARNING) << "More than 2 templates are present. Snapping using the oldest two";
		}
	}
	return TemplateManipulations::MultiSnap(
		workingTemplates[0]->getTemplate(),
		workingTemplates[1]->getTemplate(),
		maxDistance,
		&connectingInfo);
}

void abstractUI::handleConnect() {
	if (workingTemplates.size() < 2) {
		if (DEBUG) {
			LOG(WARNING) << "Not snapping; 2 templates required.";
		}
		return;
	}
	if (workingTemplates.size() > 2) {
		if (DEBUG) {
			LOG(WARNING) << "More than 2 templates are present. Connecting using the oldest two";
		}
	}
	Template* composed = TemplateManipulations::ConnectWithGrammar(
		workingTemplates[0]->getTemplate(),
		workingTemplates[1]->getTemplate());
		//connectingInfo);
	auto workingTemplate = new WorkingTemplate(composed);
	workingTemplate->replaceShoulderJoints();
	workingTemplates.erase(workingTemplates.begin());
	workingTemplates.erase(workingTemplates.begin());
	workingTemplates.push_back(workingTemplate);
}

std::vector<NewPatch*> abstractUI::getClossestPatch() {
	std::vector <NewPatch*> result;
	if (workingTemplates.size() < 2) {
		return result;
	}
	if (workingTemplates.size() > 2) {
		if (DEBUG) {
			LOG(WARNING) << "More than 2 templates are present. Connecting using the oldest two";
		}
	}
	return TemplateManipulations::findClossestElement(
		workingTemplates[0]->getTemplate(),
		workingTemplates[1]->getTemplate());
	NewPatch* p;
}

void abstractUI::snapToGround() {
	if (workingTemplates.size() < 1) {
		if (DEBUG) {
			LOG(WARNING) << "Not snapping; 2 templates required.";
		}
		return;
	}
	workingTemplates[0]->contrainAllContactPoints();
	ConstraintsEval::enforceConstraints(workingTemplates[0]->getTemplate());	
	//ConstraintsEval::putContactPointsOnTheGround(workingTemplates[0]->getTemplate());
}



void abstractUI::generateFoldableSTL() {
	if (workingTemplates.size() < 1) {
		if (DEBUG) {
			LOG(WARNING) << "A template is required to generate STL.";
		}
		return;
	}
	if (workingTemplates.size() > 1) {
		if (DEBUG) {
			LOG(WARNING) << "More than 1 template is present. Generating STL using the oldest.";
		}
	}
	auto rootTmpl = workingTemplates[0]->getTemplate();
	auto foldable = new FoldableGraph(rootTmpl);
	PrintableDesign pf = foldable->generatePrintableDesign();
	pf.generatePrint();
}

void abstractUI::ensureAboveGround() {
	PROFILE_THIS(__FUNCTION__);
	
	for each (auto workingTemplate in workingTemplates) {
		double lowestZ = 0;
		lowestZ = min(lowestZ, workingTemplate->getTemplate()->getLowestZ());
		if (lowestZ < 0){
			workingTemplate->getTemplate()->translate(Eigen::Vector3d(0, -lowestZ, 0));
		}

	}
}

void abstractUI::rotate(Template* tmpl, double x, double y, double z, double w, double centerX, double centerY, double centerZ)
{
	Eigen::Vector3d center;
	center << centerX, centerY, centerZ;
	tmpl->rotate(center, Eigen::Quaterniond(w, x, y, z));
	ensureAboveGround();
}

void abstractUI::translate(Template* tmpl, double x, double y, double z)
	{
	tmpl->translate(Eigen::Vector3d(x, y, z));
	ensureAboveGround();
}


void abstractUI::scale(TemplateElement* tmpl, int subElementID, int axis, double amount, PatchPair const& snappingConstraint, bool preventCollisions){
	using namespace std;
	PROFILE_THIS(__FUNCTION__);

	preventCollisions = false;
	if (preventCollisions) {
		PROFILE_THIS("overlap prevention in abstractUI::scale");
		Template* root = tmpl->getRoot();
		Eigen::VectorXd origQ = root->getFullQ();
		int overlapness = OverlapPreventionHack::countOverlapness(root->getAllDrawing());
		
		ConstraintsEval::updateTemplateBySubElementFaceScaling(tmpl, 0, axis, amount, snappingConstraint);

		int newOverlapness = OverlapPreventionHack::countOverlapness(root->getAllDrawing());
		if (newOverlapness > overlapness) {
			if (DEBUG) {
				LOG(INFO) << "Overlap detected; reverting to old params";
			}
			root->updateFullQ(origQ);
		}
	}
	else {
		if (DEBUG) {
			VLOG(3) << "scaling template by amout " << amount << std::endl;
		}
		ConstraintsEval::updateTemplateBySubElementFaceScaling(tmpl, 0, axis, amount, snappingConstraint);
	}

	//update all articulations
	std::vector<NewConnection*> connections;
	tmpl->getRoot()->getConnections(connections, TreeScope::DESCENDANTS);
	//printf("We have %d connections\n", connections.size());
	PROFILED {
		PROFILE_THIS("update articulations in abstractUI::scale");
		for each (auto nc in connections) {
			if (nc->getArticulation() != nullptr) {
				//printf("\nUpdating articulation\n", connections.size());
				auto art = nc->getArticulation();
				art->updateParameters(SymbolicAssignment::USE_CURRENT_VALUES);
				art->updateTransform();
			}
		}
	};
	ensureAboveGround();
}



void abstractUI::translatePart(TemplateElement* tmpl, int subElementID, int axis, double amount){
	using namespace std;
	PROFILE_THIS(__FUNCTION__);

	WorkingTemplate * workingTemplate;
	for each (auto w in workingTemplates) {
		if (w->getTemplate() == tmpl->getRoot()) {
			if (w != workingTemplates[0]) {
				LOG(WARNING) << "Analyzing stability of not the oldest template";
			}
			workingTemplate = w;
		}
	}

	if (DEBUG) {
		VLOG(3) << "part translation of template by amout " << amount << std::endl;
	}
	ConstraintsEval::updateTemplateBySubElementFaceTranslate(workingTemplate->kinchain, tmpl, 0, axis, amount);

	//update all articulations
	std::vector<NewConnection*> connections;
	tmpl->getRoot()->getConnections(connections, TreeScope::DESCENDANTS);
	//printf("We have %d connections\n", connections.size());
	PROFILED {
		PROFILE_THIS("update articulations in abstractUI::scale");
		for each (auto nc in connections) {
			if (nc->getArticulation() != nullptr) {
				//printf("\nUpdating articulation\n", connections.size());
				auto art = nc->getArticulation();
				art->updateParameters(SymbolicAssignment::USE_CURRENT_VALUES);
				art->updateTransform();
			}
		}
	};
	ensureAboveGround();
}


double abstractUI::getStabilityDir(TemplateElement* tmpl, int subElementID, int axis, int gaitId, int objective){
	for each (auto workingTemplate in workingTemplates) {
		if (workingTemplate->getTemplate() == tmpl->getRoot()) {
			if (workingTemplate != workingTemplates[0]) {
				if (DEBUG) {
					LOG(WARNING) << "Analyzing stability of not the oldest template";
				}
			}
			return workingTemplate->getDirStability(tmpl, 0, axis, gaitId, objective);
		}
	}
	if (DEBUG) {
		LOG(ERROR) << "Stability cannot be analyzed for a template not in a working template!";
	}
	return 0;
}


void abstractUI::generateStationary3DStl(){


	std::string destfilename = "UIPrint.stl" ;

	WorkingTemplate * wTemp =workingTemplates[0];

	wTemp->kinchain->updateControllers(0);
	wTemp->kinchain->setRounds(0);
	wTemp->kinchain->updateGait(0.0);

	FoldableGraph * foldGraph = new FoldableGraph(wTemp->getTemplate()); 
	PrintableDesign pf = foldGraph->generatePrintableDesign();
	
	std::vector<double> times;
	times.push_back(0.0);


	for (int itime = 0; itime < times.size(); ++itime) {
		wTemp->kinchain->updateTime(times[itime]);
		Geometry * geo = wTemp->kinchain->getGeometry();

		std::stringstream dest;
		dest << "..\\..\\data\\prints\\" << itime << destfilename << "_anim.stl";

		geo->write(dest.str());
	}

	pf.generatePrint("result", true, wTemp->kinchain, times);

}