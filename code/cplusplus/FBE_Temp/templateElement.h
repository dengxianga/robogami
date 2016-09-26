#ifndef _TEMPLATE_ELEMENT_
#define _TEMPLATE_ELEMENT_

#include "template.h"
#include <vector>
#include "NewConnection.h"


namespace FabByExample {

class NewPatchLine2D3D;

class TemplateElement : public Template {
public:

#pragma region Overriding memebers, see Template for documentation.
	void getAllElementList(vector<Element*>& elements);
	void getAllElementTempList(vector<TemplateElement*>& elements);

	bool isLeaf() {
		return true;
	}

	void getGeometryList(vector<Geometry*>& geometries);

	virtual void updateLocalQ(const Eigen::VectorXd& newQ);
	Eigen::VectorXd getFullQ();
	Eigen::VectorXd getOrigFullQ();
	int getFullQLen();
	virtual void updateFullQ(const Eigen::VectorXd& q);
	virtual void addCurrentEnvTo(ConcreteSymbolicAssignment& env);

	double getArea();

	DebugInfo* getDebugInfo();
#pragma endregion

	// Get the underlying element encapsulated by this TemplateElement.
	Element* getElement() {
		return element;
	}

	// Create a patch for the given edge, and also add the patch to the element.
	NewPatchLine2D3D* createAndAddPatch(int edgeId);
	

	// Get a reference to the array of patches this element holds.


private:
	Element* element; // The underlying element.
	
	friend class TemplateElementBuilder;

	// Construct a TemplateElement. Reserved for use by TemplateElementBuilder.
	TemplateElement(int id) : Template(id) {}

	DISALLOW_COPY_AND_ASSIGN(TemplateElement);
};

class TemplateElementBuilder : public TemplateBuilder {
public:
	TemplateElementBuilder(int id);
	~TemplateElementBuilder();
	void setName(string const& name);
	void setElement(Element* element);
	void addConstraint(Constraint* constraint);

	TemplateElement* build() {
		auto res = tmpl;
		tmpl = nullptr;
		return res;
	}

	bool isLeaf();
	TemplateElementBuilder* asElement();
	TemplateGroupBuilder* asGroup();

	Template* getResultingPointer() {
		return tmpl;
	}
	void addContactPoint(ContactPoint* p){
		tmpl->addContactPoint(p);
	}

	NewPatch* createAndAddPatch(int edge_id);
	NewPatch* createAndAddServoLinePatch(Point3S & v1, Point3S & v2, Eigen::Vector3d normal  ) ;
	NewPatch* createAndAddServoPointPatch(Point3S & center, LinearExpr &separtion, Eigen::Vector3d normal );
	NewPatch* createAndAddPeripheralPatch(NewPatchLine2D3D *  edge_id1, NewPatchLine2D3D *  edge_id2, Eigen::Vector3d normal);

	void addConnection(NewConnection* conn);

private:
	vector<pair<int, int>> patch_edgeid_id_pairs;

	TemplateElement* tmpl;

	DISALLOW_COPY_AND_ASSIGN(TemplateElementBuilder);
};

}


#endif
