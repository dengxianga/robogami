#ifndef __COMPONENT__
#define __COMPONENT__

#include "TriMesh.h"
#include "mathVec.h"
#include <Eigen/Dense>
#include "XForm.h"
#include <list>

namespace FabByExample{

class Edge;
class VectorXd;
class MatrixXd;
class PrimitiveSet;
class Design;
class Tree;
class Geometry;
class Functionality;


class Component{
public:
	enum ComponentSet {ALL, PARTS, CONNS};
	
	int auxflag;
	bool isSubstructure;
		
	~Component();

	void deleteGeos();

	//basics
	std::list<Edge*> edges;
	std::string getGenName();
	Design* getDesign() {return parentDesign;}
	Component(Design* _parentDesign, int _id, std::string);
	int getId();
	void printComponent();
	void simplePrintComponent(int n);
	std::string getFullName();
	Geometry* getGeometry();
	Geometry* getGeometryInGlobalCoord();
	virtual bool getIsLeaf() = 0;
	void makeSubstructure(){isSubstructure = true;}

	//tree related functions
	void setParrent(Component* _parent){parent = _parent;}
	matrix9f getGlobalRot(){ return globalRot;}
	vector3f getGlobalTrans () {return globalTrans;}
	void addEdge(Component* _child, std::string relativeMatrixValue);
	void addEdge(Component* _child, const std::vector<double>& relativeMatrix);
	std::string getName();
	Component* getParent() {return parent;}
	std::list<Edge*> getEdges(){ return edges;}
	Tree* getTree();
	int removeChild(Edge* edgeChild);
	Edge* getEdgeTo(Component* comp);
	int removeChildFromComp(Component* comp);
	void addChild(Edge* edgeChild);
	//functionality related funcitions - in making tree
	int isFunc();
	void addFunctionality(Functionality*  func);
	bool hasSons();
	bool isFatherOfComponent(Component* comp);
	bool isRoot();
	std::list<Component*>  Component::getLeafs();


	// range and type related functions : this should only be for the nodes?
	void setVariableAxis(int ax){isVariableAxis[ax] = true;}
	bool getVariableAxis(int ax){ return isVariableAxis[ax];}
	int isPart();
	int isConnector();

	
	// have to check this yet
	Geometry* getGeometryMinFunc(double alpha, Functionality*  func);
	Geometry* getGeometryMinFuncs(std::vector<double> alphas, std::vector<Functionality*> funcs);
	vector3f getCenterOfAxisInGlobalCoord();
	void setGlobalTrans(matrix9f rot, vector3f trans);

	
	// functionality
	std::vector<Functionality*> getFunctionalitiesThatAffectSubassemply();
	xform getTranfToCenterFromFucntionality(Functionality*  func);	
	xform getFucntionalityTransformation(double alpha, Functionality*  func);		
	xform getTranfFromCenterFromFucntionality(Functionality*  func);
	matrix9f getRotFromFucntionality(double alpha, Functionality*  func);	
	vector3f getTransFromFucntionality(double alpha, Functionality*  func);
	vector3f getPointClossestToFuncCenter(Functionality*  func);	
	vector3f getAxisFromFucntionality(Functionality*  func);
	vector3f getTransOfMinsFromFucntionality(double alpha, Functionality*  func);
	std::string getFuncInfo(Functionality*  func, double alpha);
	bool gethasFunctionality(Functionality* func);
	Functionality* getFunc(){return hasFunctionality;}
	bool getaboveFunctionality(Functionality* func);
	bool checkIfFunctionalityIsChild(Functionality* func);


	//more on getting geometry 
	point sizeOfOrgTemplate();
	point getGlobalMin();
	TriMesh::BBox getBBoxGlobalCoords();
	std::vector<TriMesh*> getOriginalMeshes(ComponentSet cs);
	

	

	//Materials and functionality vectors
	virtual void addMaterialToVector(Eigen::VectorXd & materialVector) = 0;
	void getMaterialVector(Eigen::VectorXd & materialVector);
	void getFunctionalityVector(Eigen::VectorXd & funcVec);
	virtual double getVolume() = 0;
	std::string getGlobalTranf();


	//added later
	void removeConnectorGroup();
	void removeMiddleNode();

protected:
	int thisId;
	std::string name;
	Geometry* geometry;
	Geometry * orig_geometry; 
	Geometry* geometryInGlobalCoords; 
	matrix9f globalRot;
	vector3f globalTrans;
	Functionality* hasFunctionality;
	Component* parent;
	Design* parentDesign;

	bool hasRange;
	bool isVariableAxis[3];


	
	
};

}


#endif