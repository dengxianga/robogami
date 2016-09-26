#include "NewConnection.h"
#include "NewPatch.h"
#include "templateElement.h"
#include "articulation.h"
#include "controller.h"
#include "constraintsEval.h"
using namespace FabByExample;
using namespace std;


NewConnection::NewConnection(vector<NewPatch *> patches, double angle, ConnectionType connectionType)
{
	this->patches = patches;
	this->angle = angle;
	this->connectionType = connectionType;
	this->is2dSeparate = false;
}

vector<TemplateElement *> NewConnection::getTemplates(){
	vector<TemplateElement *> templates;
	for (int i = 0; i < patches.size(); i++){
		patches[i]->getElement()->getAllElementTempList(templates);
	}

	//remove any non-unique elements:
	std::unique (templates.begin(), templates.end(), [](TemplateElement * x, TemplateElement * y) -> bool { return x == y; } );

	return templates;
}


NewConnection::~NewConnection(void)
{
}

bool NewConnection::isIncomplete() {
	for each (auto patch in patches) {
		if (patch->getType() == NewPatch::PatchType::PatchPlaceHolder) {
			return true;
		}
	}
	return false;
}

bool NewConnection::isRelated(Template * temp)
{
	bool rel = false;
	for each (auto p in patches){
		if (p->isRelated(temp)){
			rel = true;
		}
	}
	return rel;
}


bool NewConnection::hasPatch(NewPatch * patch)
{
	bool rel = false;
	for each (auto p in patches){
		if (p == patch){
			rel = true;
		}
	}
	return rel;
}
FabDebugging::DebugInfo* NewConnection::getDebugInfo() {
	auto info = new FabDebugging::DebugInfo();
	info->setTypeName("NewConnection");
	info->setShortDescription(concat() << "Connection" << (isIncomplete() ? " (Half)" : ""));
	const char* enumNames[] = { "FOLD", "TEETH", "TOUCH", "HINGE", "BALLJOINT", "PRISMATIC", "NONE" };
	info->putStringProperty("type", enumNames[connectionType]);
	info->putDoubleProperty("angle", angle);
	for (int i = 0; i < patches.size(); i++) {
		if (dynamic_cast<PlaceHolderPatch*>(patches[i]) == nullptr) {
			info->putReferenceProperty(concat() << "patches[" << i << "]", patches[i]);
		}
		else {
			info->putStringProperty(concat() << "patches[" << i << "]", concat() << "placeholder" << (dynamic_cast<PlaceHolderPatch*>(patches[i])->isOpposite ? " (opposite)" : ""));
		}
	}
	if (getArticulation() != nullptr) {
		info->putAggregationProperty(concat() << "articulation", getArticulation());
	}
	return info;
}

//----------------------------------------------------------------------------------

JointConnection::JointConnection(vector<NewPatch *> patches, double angle, Articulation * _articulation, NewConnection::ConnectionType connType=NewConnection::ConnectionType::HINGE):
NewConnection( patches, angle, connType)
{
	articulation = _articulation;
}

void JointConnection::setArticulation(Articulation* _articulation) {
	if(this->articulation == nullptr){
		this->articulation = _articulation;
	}else{
		SymbolicController* symbController = dynamic_cast<SymbolicController*>(this->articulation->transformations[0]->controller);
		if(symbController == nullptr){
			this->articulation = _articulation;
		}else{
		PWLinearController* linearController = dynamic_cast<PWLinearController*>(_articulation->transformations[0]->controller);
			if(linearController != nullptr){
				ConstraintsEval::optimizeController(symbController, linearController);
				symbController->updateSymbols();
			}
		}
	}
}

void JointConnection::updateParameters(SymbolicAssignment const& env) {
	PROFILE_THIS(__FUNCTION__);
	if (articulation != nullptr) {
		articulation->updateParameters(env);
	}
}
