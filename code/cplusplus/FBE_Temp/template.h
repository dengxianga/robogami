#pragma once
#include <vector>
#include "TriMesh.h"
#include "NewConnection.h"
#include <symbolic.h>
#include "debugging.h"
#include "element.h"
#include "contactPoint.h"
#include "constraints.h"
#include "semantics.h"

namespace FabByExample{

class Constraint;
class Geometry;
class TemplateElement;
class CenterOfMass;
class NewPatchLine2D3D;
// A specifier of which part of the tree is requested, when getting "all" something,
// such as getting all constraints, all symbols, all connections in a particular part
// of the tree.
// 
// Note that scopes such as ENTIRE_TREE is not included because it can be simply done
// by getting the root and then using DESCENDANTS.

struct SymmetryChoices{
	bool symm_ground;
	bool symm_legW;
	bool symm_legL;
	bool symm_spacing;
	SymmetryChoices(){
		symm_ground = 0;
		symm_legW = 0;
		symm_legL = 0;
		symm_spacing = 0;
	}
};


enum TreeScope {
	// Get only the requested properties of this template itself.
	SELF,

	// Get self, and all ancestors, in the order of
	// self, parent, parent's parent, ..., root.
	ANCESTORS,

	// Same as ANCESTORS but excluding SELF.
	STRICT_ANCESTORS,

	// Get self, and all descendants, in the Depth First Search order. For example, if
	// the template tree is
	// T1 (T2 (T3, T4), T5)
	// The iteration order is:
	// T1, T2, T3, T4, T5.
	DESCENDANTS,

	// Same as DESCENDANTS but excluding SELF.
	STRICT_DESCENDANTS,
};

class Template : public IDefinesSymbols, public Debuggable {
public:

	// Base constructor that initializes the most basics of a template and sets its ID.
	Template(int id);
	void addParameter(std::string newName, double q0);
	bool isIndependent;
	virtual ~Template();
	void clear(); 

#pragma region Most basic properties of the template itself
	// Directly changes the ID of this template, with no consideration whatsoever. Do not use unless you
	// know what this implies.
	void setId(int id) {
		tempID = id;
	}

	// Get the ID of this template.
	int getID() {
		return tempID;
	}

	// Get a friendly name of this template.
	string getName() {
		return name;
	}

	// Overwrite the friendly name of this template.
	void setName(string name) {
		this->name = name;
	}
#pragma endregion

#pragma region Structures of the template tree
	// Returns true iff this template is a leaf.
	virtual bool isLeaf() = 0;

	// Returns the parent of this template, or nullptr if this is the root.
	Template* getParent() {
		return parent;
	}

	// Returns the array of direct children of this parent.
	vector<Template *> const& getChildren() {
		return children;
	}

	// Returns the root of this template tree.
	Template* getRoot();

	// Returns true iff this template is equal or a descendent of the other template.
	bool isDescendentOf(Template* other);

	// Returns true iff this template is equal or an ancestor of the other template.
	bool isAncestorOf(Template* other) {
		return other->isDescendentOf(this);
	}

	// Returns the closest ancestor that is also an ancestor of the other template.
	Template* getLowestCommonAncestor(Template* other);

	// Get a list of all the leaf elements into the given elements array.
	virtual void getAllElementList(vector<Element*>& elements) = 0;
	vector<Element*> getAllElementList() {
		vector<Element*> ret;
		getAllElementList(ret);
		return ret;
	}

	// Get a list of all the leaf template elements into the given elements array.
	virtual void getAllElementTempList(vector<TemplateElement*>& elements) = 0;
	vector<TemplateElement*> getAllElementTempList() {
		vector<TemplateElement*> ret;
		getAllElementTempList(ret);
		return ret;
	}

	// Get all the templates in the given scope relative to this template.
	void getTemplatesByScope(vector<Template*>& elements, TreeScope scope);
	vector<Template*> getTemplatesByScope(TreeScope scope) {
		vector<Template*> ret;
		getTemplatesByScope(ret, scope);
		return ret;
	}

	// Find a descendant template with the given ID, or nullptr if there isn't one.
	Template* findDescendantByID(int id);

	// Prints some debugging information about this tree.
	void displayTree(int identN);

#pragma endregion

#pragma region Structural modifications

	// Add the given template as a direct child of this template, and make this template its parent.
	void addChild(Template* child);

	// Segregate this template from its parent, making it an isolated template subtree
	// where this template is the root.
	//
	// WARNING: After this call, the original template tree is no longer valid, and only
	// the subtree rooted at this template is valid.
	void segregate();

	// Remove this template from the parent template. The difference between this and
	// segregate() is that remove() will make this template invalid instead, and
	// preserves the validity of the parent template and the tree it's contained in.
	//
	void remove();

#pragma endregion

#pragma region Constraints

	// Get all the constraints in the given scope relative to this template.
	void getConstraints(vector<Constraint*>& constraints, TreeScope scope);

	vector<Constraint*> getConstraints(TreeScope scope) {
		vector<Constraint*> ret;
		getConstraints(ret, scope);
		return ret;
	}

	// Get a reference to the constraints array held by this template.
	// Semantically equivalent to getConstraints(SELF), but faster.
	const vector<Constraint*>& getConstraints() {
		return constraints;
	}

	// Get all the constraints in the given scope relative to this template that relates (refers) to this
	// template.
	void getAssociatedConstraints(vector<Constraint*>& constraints, TreeScope scope);
	vector<Constraint*> getAssociatedConstraints(TreeScope scope = TreeScope::SELF) {
		vector<Constraint*> ret;
		getAssociatedConstraints(ret, scope);
		return ret;
	}


	// Add the given constraint to this template as a new constraint.
	void addConstraint(Constraint* constraint) {
		constraints.push_back(constraint);
	}

	void addContactPoint(ContactPoint* & p){
		contactPoints.push_back(p);
	}

	// Remove a constraint from the constraints held directly by this template.
	void removeConstraint(Constraint* c);

	void removeAllConstraintsOfType(Constraint::ConstraintType type);
#pragma endregion

#pragma region Geometry
	// Compose the geometries under this template subtree to form one single geometry.
	Geometry* getGeometry(bool withnormals = true);

	// Get the list of geometries under this template subtree.
	virtual void getGeometryList(vector<Geometry*>& geometries) = 0;

	// Evaluate the current symbolic bounding box, where the comparisons of linear expressions are
	// based on the current parameter values.
	virtual BBox3S evalLocalBoundingBox();

	// Get the current bounding box numerically.
	TriMesh::BBox evalBoundingBox();

	// Get a composed drawing containing the drawings of all the elements under this template subtree.
	drawing::Drawing* getAllDrawing();

#pragma endregion

#pragma region Symbolics
	// Get all symbols in the given scope relative to this template.
	void getSymbols(vector<Symbol>& symbols, TreeScope scope);
	vector<Symbol> getSymbols(TreeScope scope = TreeScope::SELF) {
		vector<Symbol> ret;
		getSymbols(ret, scope);
		return ret;
	}

	// Implementations of IDefinesSymbols. See IDefinesSymbols in symbolic.h for documentation.
	virtual int numSymbols() const;
	virtual string describeSymbol(int id) const;
	virtual double getCurrentValue(int id) const;
	virtual void setCurrentValue(int id, double _val);
	virtual std::string getSymbolName(int id) const;

	// Adds the current symbolic assignments of this template and all its descendants to the
	// given ConcreteSymbolicAssignment.
	virtual void addCurrentEnvTo(ConcreteSymbolicAssignment& env) = 0;

	// Reset the original parameter values (symbolic assignments) that this template tree was
	// initialized with. It will reset the param values for all descendant templates as well.
	// Note that this should be called with root.
	virtual void resetInitialParamValues();

	// Get a vector of the parameter values for this template only.
	Eigen::VectorXd getLocalQ() const {
		return q;
	}

	// Get a vector of the original parameter values for this template only.
	Eigen::VectorXd getOrigLocalQ() const {
		return origQ;
	}

	// Set the parameter values for this template only, and update the geometry.
	virtual void updateLocalQ(const Eigen::VectorXd& newQ) {
		q = newQ;
	}

	// Get a vector of the parameter values for this template and also its descendants, in DFS order.
	virtual Eigen::VectorXd getFullQ() = 0;
	// Get the length of the vector that would've been returned by getFullQ(), also equal to the length
	// of the vector that would've been returned by getSymbols(DESCENDANTS).
	virtual int getFullQLen() = 0;
	// Get a vector of the original parameter values for this template and also its descendants.
	virtual Eigen::VectorXd getOrigFullQ() = 0;
	// Set the parameter values for this template and its descendants, and update the geometries.
	// The given vector must also be in DFS order.
	virtual void updateFullQ(const Eigen::VectorXd& q) = 0;

#pragma endregion

#pragma region Transformations

	// Get the value of the lowest Z coordinate of the entire mesh formed by all the template elements
	// under this template subtree.
	double getLowestZ();
	// Get the point that gives the lowest Z coordinate in the current symbolic environment.
	Point3S getLowestZPoint();
	// Move the template so that its lowest Z coordinate is 0.
	void moveToGround();

	// Get the current cached center of the template. May not be accurate. To ensure accuracy, call
	// recomputeCenter().
	Eigen::Vector3d getCenter();
	// Recompute the currently believed center of the template. The center is simply the center of
	// the bounding box.
	void recomputeCenter();
	// Compute the center of mass.
	CenterOfMass computeCenterOfMass();

	// Permanently rotate the whole mesh under this template subtree around the given center by the
	// given quarternion.
	void rotate(Eigen::Vector3d const& center, Eigen::Quaterniond const& rotation);

	// Permanently translate the whole mesh under this template subtree by the given translation
	// vector.
	void translate(Eigen::Vector3d const& translation);

	// Permanently translate the whole 2D drawing under this template subtree by the given translation
	// vector.
	void translate2d(Eigen::Vector2d const& translation);


#pragma endregion

#pragma region Patches & Connections
	// Get all patches under this template subtree.
	void getAllPatches(vector<NewPatch*>& patches);
	void getAllUnusedPatches(vector<NewPatch*>& patches);

	vector<NewPatch*> getAllPatches() {
		vector<NewPatch*> ret;
		getAllPatches(ret);
		return ret;
	}
	vector<NewPatch*> getAllUnusedPatches() {
		vector<NewPatch*> ret;
		getAllUnusedPatches(ret);
		return ret;
	}
	const vector<NewPatch*>& getPatches() {
		return patches;
	}

	// Get all connections in the given scope relative to this template.
	void getConnections(vector<NewConnection *>& conn, TreeScope scope);

	vector<NewConnection*> getConnections(TreeScope scope = TreeScope::SELF) {
		vector<NewConnection*> ret;
		getConnections(ret, scope);
		return ret;
	}

	// Add a new connection to the template.
	void addConnection(NewConnection* conn);
	// Remove a connection from the template.
	void removeConnection(NewConnection* conn);
#pragma endregion

#pragma region Miscellaneous
	// TODO(adriana): Document this or delete please
	virtual double getArea() =0;

	// TODO(adriana): Document this please
	void getContactPoints(vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& contpoints);
	// TODO(robin): Remove.
	bool pauseRecalculation;

	// Get the debugging information. See debugging.h
	DebugInfo* getDebugInfo() = 0;

	// Forcefully redefine the parameters. Not recommended.
	void forceRedefineParams(const vector<string>& paramNames, const Eigen::VectorXd& q);

	void addPatch(NewPatch* p);
#pragma endregion



#pragma region Semantics
	// TODO(adriana): Document this or delete please
	Semantics::PartType getPartType(){return semantics.partType;}
	Semantics::PrintMethod getPrintMethod(){return semantics.printMethod;}
	void setPrintMethod(Semantics::PrintMethod _printMethod){semantics.printMethod = _printMethod;}
	void setPartType(Semantics::PartType _partType){semantics.partType = _partType;}


#pragma endregion

	vector<ContactPoint*> contactPoints;
	vector<ContactPoint*> getAllContactPoints();
	vector<NewConnection*> getAllConnections();
	vector<ContactPoint*> getOneContactPointsForEachPart();
	Template* parent; // parent pointer

	SymmetryChoices symmetryChoices;
protected:

	int tempID; // template ID
	string name; // template name
	vector<Template*> children; // array of children
	vector<NewPatch *> patches; // the patches of this template element.

	Geometry* tempGeo; // cached computation of the current geometry.

	int numParams; // number of parameters defined by this template
	vector<string> parameterNames; // names of parameters defined by this template
	Eigen::VectorXd origQ; // the vector of the original parameter values
	Eigen::VectorXd q; // the vector of the current parameter values

	Eigen::Vector3d center; // cached computation of the current center.

	vector<Constraint *> constraints; // constraints directly held by this template
	vector<NewConnection *> connections; // connections directly held by this template
	friend class TemplateBuilder;
	DISALLOW_COPY_AND_ASSIGN(Template);
	Semantics semantics;

};

class TemplateElementBuilder;
class TemplateGroupBuilder;

class TemplateBuilder {
public:
	virtual Template* getResultingPointer() = 0;
	virtual Template* build() = 0;
	virtual void setName(string const& name) = 0;
	virtual bool isLeaf() = 0;
	virtual void addConstraint(Constraint* constraint) = 0;
	virtual NewPatch * createAndAddPeripheralPatch(NewPatchLine2D3D *  edge_id1, NewPatchLine2D3D *  edge_id2, Eigen::Vector3d normal) = 0;
	virtual TemplateElementBuilder* asElement() = 0;
	virtual TemplateGroupBuilder* asGroup() = 0;
	virtual ~TemplateBuilder() {}
	virtual void addConnection(NewConnection * conn) = 0;
	virtual void defineParameters(const vector<string>& parameterNames, const Eigen::VectorXd& initialValues);
	virtual void addContactPoint(ContactPoint* p) =0; 
protected:
	TemplateBuilder() {}
	DISALLOW_COPY_AND_ASSIGN(TemplateBuilder);
};
}