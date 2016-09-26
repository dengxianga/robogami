#ifndef _TEMPLATE_GROUP_
#define _TEMPLATE_GROUP_


#include "template.h"

namespace FabByExample{
class DataTraits;

class TemplateGroup : public Template {
public:
	void getGeometryList(vector<Geometry*>  & geometries);
	void getAllElementList(vector<Element*>& elements);
	void getAllElementTempList(vector<TemplateElement*>& elements);
	bool isLeaf() {return false;}

	virtual void addCurrentEnvTo(ConcreteSymbolicAssignment& env);
	Eigen::VectorXd getFullQ();
	int getFullQLen();
	Eigen::VectorXd getOrigFullQ();
	void updateFullQ(const Eigen::VectorXd & q);

	double getArea (); 

	DebugInfo* getDebugInfo();

private:
	TemplateGroup(int id) : Template(id) {}
	friend class TemplateGroupBuilder;
	
	DISALLOW_COPY_AND_ASSIGN(TemplateGroup);
};

class TemplateGroupBuilder : public TemplateBuilder {
public:
	TemplateGroupBuilder(int id);
	~TemplateGroupBuilder();
	void setName(string const& name);
	void addChild(Template* child);
	void addConstraint(Constraint* constraint);
	NewPatch * createAndAddPeripheralPatch(NewPatchLine2D3D *  edge_id1, NewPatchLine2D3D *  edge_id2, Eigen::Vector3d normal);
	void addConnection(NewConnection * conn);
	TemplateGroup* build() {
		auto res = tmpl;
		tmpl = nullptr;
		return res;
	}
	bool isLeaf();
	TemplateElementBuilder* asElement();
	TemplateGroupBuilder* asGroup();
	Template* getResultingPointer() { return tmpl; }
		void addContactPoint(ContactPoint* p){
		tmpl->addContactPoint(p);
	}
private:
	TemplateGroup* tmpl;

	DISALLOW_COPY_AND_ASSIGN(TemplateGroupBuilder);
};

}


#endif