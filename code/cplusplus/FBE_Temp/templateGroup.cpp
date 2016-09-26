#include "templateGroup.h"
#include "templateElement.h"
#include "element.h"
#include "constraints.h"
#include "NewPatch.h"

#include <sstream>
using namespace FabByExample;
using namespace std;


void Template::displayTree(int identN){
	for (int i = 0; i< identN; i++){
		cout << " - " ;
	}
	cout<< name << endl;
	for each (auto temp in children){
		temp->displayTree(identN +1);
	}

}

void TemplateGroup::getGeometryList(vector<Geometry*>   & geometries){
	for each (auto child in children) {
		child->getGeometryList(geometries);
	}
}

void TemplateGroup::getAllElementList(vector<Element*>& elements) {
	for each (auto child in children) {
		child->getAllElementList(elements);
	}
}




void TemplateGroup::getAllElementTempList(vector<TemplateElement*>& elements) {
	for each (auto child in children) {
		child->getAllElementTempList(elements);
	}
}

int TemplateGroup::getFullQLen() {
	PROFILE_THIS(__FUNCTION__);
	int qLen = numSymbols();
	for each (auto child in children) {
		int child_len = child->getFullQLen();
		qLen = qLen + child_len;
	}

	return qLen;
}


Eigen::VectorXd TemplateGroup::getFullQ()
{
	Eigen::VectorXd joinedQ(getFullQLen());
	joinedQ.head(numSymbols()) = getLocalQ();
	int init_pos = numSymbols();
	for each (auto child in children) {
		int len = child->getFullQLen();
		joinedQ.segment(init_pos, len) = child->getFullQ();
		init_pos = init_pos + len;
	}

	return joinedQ;
}

Eigen::VectorXd TemplateGroup::getOrigFullQ()
{
	Eigen::VectorXd joinedQ(getFullQLen());
	joinedQ.head(numSymbols()) = getOrigLocalQ();
	int init_pos = numSymbols();
	for each (auto child in children) {
		int len = child->getFullQLen();
		joinedQ.segment(init_pos, len) = child->getOrigFullQ();
		init_pos = init_pos + len;
	}

	return joinedQ;
}

void TemplateGroup::updateFullQ(const Eigen::VectorXd& q)
{
	PROFILE_THIS(__FUNCTION__);

	updateLocalQ(q.head(numSymbols()));
	int init_pos = numSymbols();
	for each (auto child in children) {
		int len = child->getFullQLen();
		Eigen::VectorXd childQ = q.segment(init_pos, len);
		child->updateFullQ(childQ);
		init_pos = init_pos + len;
	}

	if (getParent() == nullptr) {
		vector<NewConnection *> conns = getConnections(TreeScope::DESCENDANTS);
		for each (auto conn in conns) {
			conn->updateParameters(SymbolicAssignment::USE_CURRENT_VALUES);
		}
	}
}

void TemplateGroup::addCurrentEnvTo(ConcreteSymbolicAssignment& env)
{
	PROFILE_THIS(__FUNCTION__);
	for each (auto child in children) {
		child->addCurrentEnvTo(env);
	}
	for (int i = 0; i < numSymbols(); i++)
	{
		env.map[getSymbol(i)] = q[i];
	}
}

double TemplateGroup::getArea() {
	vector<TemplateElement*> tempelements;
	getAllElementTempList(tempelements);

	double area = 0;
	for (int i = 0; i < tempelements.size(); i++){
		area = area+ tempelements[i]->getArea();
	}
	return area;
}


TemplateGroupBuilder::TemplateGroupBuilder(int id) {
	tmpl = new TemplateGroup(id);
}

TemplateGroupBuilder::~TemplateGroupBuilder() {
	if (tmpl != nullptr) {
		delete tmpl;
	}
}

void TemplateGroupBuilder::setName(string const& name) {
	tmpl->setName(name);
}

void TemplateGroupBuilder::addChild(Template* child) {
	tmpl->addChild(child);
}

bool TemplateGroupBuilder::isLeaf() {
	return false;
}

TemplateElementBuilder* TemplateGroupBuilder::asElement() {
	return nullptr;
}

TemplateGroupBuilder* TemplateGroupBuilder::asGroup() {
	return this;
}

void TemplateGroupBuilder::addConstraint(Constraint* constraint) {
	tmpl->addConstraint(constraint);
}

NewPatch * TemplateGroupBuilder::createAndAddPeripheralPatch(NewPatchLine2D3D *  edge_id1, NewPatchLine2D3D *  edge_id2, Eigen::Vector3d normal){
	
	PeripheralPatch* p = new PeripheralPatch(tmpl, edge_id1, edge_id2, normal);
	tmpl->addPatch(p);
	return p;
}

void TemplateGroupBuilder::addConnection(NewConnection * conn) {
	this->tmpl->addConnection(conn);
}


FabDebugging::DebugInfo* TemplateGroup::getDebugInfo() {
	auto info = new FabDebugging::DebugInfo();
	info->setTypeName("TemplateGroup");
	info->setShortDescription(concat() << "TemplateGroup (" << tempID << ")");
	info->putIntProperty("# Params", numSymbols());
	for (int i = 0; i < numSymbols(); i++) {
		info->putDoubleProperty(concat() << "q[" << describeSymbol(i) << "]", q[i]);
	}
	int j = 0;
	for (auto it = children.begin(); it != children.end(); it++) {
		info->putAggregationProperty(concat() << "children[" << j << "]", *it);
		j++;
	}
	j = 0;
	for (auto it = connections.begin(); it != connections.end(); it++) {
		info->putAggregationProperty(concat() << "connections[" << j << "]", *it);
		j++;
	}
	j = 0;
	for each (auto constraint in constraints) {
		info->putAggregationProperty(concat() << "constraints[" << j << "]", constraint);
		j++;
	}
	return info;
}