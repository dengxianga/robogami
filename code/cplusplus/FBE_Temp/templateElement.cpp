#include "templateGroup.h"
#include "templateElement.h"
#include "element_symbolic.h"
#include <limits.h>
#include "NewPatch.h"
#include "constraints.h"


using namespace FabByExample;
using namespace std;

void TemplateElement::getGeometryList(vector<Geometry*>  & geometries){
	geometries.push_back(element->getGeo());
}

void TemplateElement::getAllElementList(vector<Element*>& elements) {
	elements.push_back(element);
}


void TemplateElement::getAllElementTempList(vector<TemplateElement*>& elements) {
	elements.push_back(this);
}


Eigen::VectorXd TemplateElement::getFullQ()
{
	return getLocalQ();
}

Eigen::VectorXd TemplateElement::getOrigFullQ()
{
	return getOrigLocalQ();
}

int TemplateElement::getFullQLen()
{
	return numSymbols();
}

void TemplateElement::updateFullQ(const Eigen::VectorXd& q)
{ 
	updateLocalQ(q);
}

double TemplateElement::getArea(){
	double area = 0;
	TriMesh::BBox bbox = element->evalBoundingBox();
	for (int ax1 = 0; ax1 < 3; ax1++){
		for (int ax2 = 0; ax2 < 3; ax2++){
			if(ax1 != ax2){
				area += (bbox.max[ax1]-bbox.min[ax1])*(bbox.max[ax2]-bbox.min[ax2]);
			}
		}
	}
	return area;
}

NewPatchLine2D3D * TemplateElement::createAndAddPatch(int edge_id){
	Element_Symbolic * elem = static_cast<Element_Symbolic*>(this->getElement());
	for (int i = 0; i < elem->getDrawing2S()->edges.size(); i++){
		Edge2S edge = elem->getDrawing2S()->edges[i];

		if (edge.id == edge_id){
			//first, get the indices of the edge
			int vertex1 = edge.vertice1;
			int vertex2 = edge.vertice2;
			
			NewPatchLine2D3D * newPatch =  new NewPatchLine2D3D(this, edge_id, vertex1, vertex2);
			
			this->patches.push_back( newPatch );
			return newPatch;

		}
	}
	return nullptr; 
}



void TemplateElement::addCurrentEnvTo(ConcreteSymbolicAssignment& env)
{
	for (int i = 0; i < numSymbols(); i++)
	{
		env.map[getSymbol(i)] = q[i];
	}
}

void TemplateElement::updateLocalQ(const Eigen::VectorXd& newQ)
{
	PROFILE_THIS(__FUNCTION__);
	q = newQ;
	if (!getRoot()->pauseRecalculation) {
		element->computeNewGeo();
	}
}

TemplateElementBuilder::TemplateElementBuilder(int id) {
	tmpl = new TemplateElement(id);
}

TemplateElementBuilder::~TemplateElementBuilder() {
	if (tmpl != nullptr) {
		delete tmpl;
	}
}

void TemplateElementBuilder::setName(string const & name) {
	tmpl->setName(name);
}

void TemplateElementBuilder::setElement(Element* element) {
	tmpl->element = element;
	element->setRefTempElement(tmpl);
}

bool TemplateElementBuilder::isLeaf() {
	return true;
}

TemplateElementBuilder* TemplateElementBuilder::asElement() {
	return this;
}

TemplateGroupBuilder* TemplateElementBuilder::asGroup() {
	return nullptr;
}

void TemplateElementBuilder::addConstraint(Constraint* constraint) {
	tmpl->addConstraint(constraint);
}

NewPatch * TemplateElementBuilder::createAndAddPatch(int edge_id) {
	return tmpl->createAndAddPatch(edge_id);
}

NewPatch *TemplateElementBuilder::createAndAddServoLinePatch(Point3S & v1, Point3S & v2, Eigen::Vector3d normal ) {
	ServoLinePatch* p = new ServoLinePatch(tmpl, v1, v2, normal);
	tmpl->addPatch(p);
	return p;
}

NewPatch * TemplateElementBuilder::createAndAddPeripheralPatch(NewPatchLine2D3D *  edge_id1, NewPatchLine2D3D *  edge_id2, Eigen::Vector3d normal){
	
	PeripheralPatch* p = new PeripheralPatch(tmpl, edge_id1, edge_id2, normal);
	tmpl->addPatch(p);
	return p;
}

NewPatch * TemplateElementBuilder::createAndAddServoPointPatch(Point3S & center, LinearExpr & separation, Eigen::Vector3d normal) {
	ServoPointPatch* p = new ServoPointPatch(tmpl, center, separation, normal);
	tmpl->addPatch(p);
	return p;
}

void TemplateElementBuilder::addConnection(NewConnection * conn) {
	this->tmpl->addConnection(conn);
}

DebugInfo* TemplateElement::getDebugInfo() {
	auto info = new DebugInfo();
	info->setTypeName("TemplateElement");
	info->setShortDescription(concat() << "TemplateElement (" << tempID << ")");
	info->putAggregationProperty("mesh", dynamic_cast<Element_Symbolic*>(this->element)->getMesh3S());
	info->putIntProperty("# Params", numSymbols());
	for (int i = 0; i < numSymbols(); i++) {
		info->putDoubleProperty(concat() << "q[" << describeSymbol(i) << "]", q[i]);
	}
	int j = 0;
	for (auto it = connections.begin(); it != connections.end(); it++) {
		info->putAggregationProperty(concat() << "connections[" << j << "]", *it);
		j++;
	}
	j = 0;
	for (auto it = patches.begin(); it != patches.end(); it++) {
		info->putAggregationProperty(concat() << "patches[" << j << "]", *it);
		j++;
	}
	j = 0;
	for each (auto constraint in constraints) {
		info->putAggregationProperty(concat() << "constraints[" << j << "]", constraint);
		j++;
	}
	return info;
}