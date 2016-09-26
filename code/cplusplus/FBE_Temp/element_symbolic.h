#ifndef _ELEMENT_SYMBOLIC_
#define _ELEMENT_SYMBOLIC_
#include "CenterOfMass.h"
#include <symbolic.h>
#include <drawing.h>
#include "element.h"
namespace FabByExample{
class Geometry;
class TemplateElement;

class Element_Symbolic : public Element {
public:
	// Specifies a face that may be individually manipulated
	struct SubElementFace {
		string name;  // name of the face (from the Mesh3S)
		vector<int> trigs;  // Index of the triangles in the mesh that belong to this face

		// bounding box of this face.
		Eigen::VectorXd min[2];  // min x, min y: coefficients in front of the parameters
		Eigen::VectorXd max[2];  // max x, max y
		double minConst[2];  // min x, min y: constant term of the linear expressions
		double maxConst[2];  // max x, max y, constant term
		// LinearExpr version of the bounding box.
		BBox2S bboxS;

		// normal of the face
		Eigen::Vector3d normal;
		// up direction of the bounding box (normalized, perpendicular to normal)
		Eigen::Vector3d up;
		// Center position of the face
		Eigen::Vector3d center;
		//
		double width;
		double height;
	};

	// Struct that holds the specification for what logical sub-elements (faces / edges)
	// exist in this element that can be individually manipulated.
	struct SubElementsSpec {
		vector<SubElementFace> faces;
	};

	// Construct an Element_Symbolic using the given symbolic mesh and optionally
	// a symbolic 2D drawing.
	Element_Symbolic(Mesh3S* mesh3S, Drawing2S* drawing2S = nullptr);

	virtual ~Element_Symbolic();

	// Latently set the referring template element.
	virtual void setRefTempElement(TemplateElement* refTempElement);

	// Recompute the geometry, as well as the sub element specifications.
	void computeNewGeo();
	// Compute the bounding box of this element.
	::TriMesh::BBox evalBoundingBox();

	// Get the subelements computed by this element. The returned pointer should be
	// interpreted as an array, where the size of the array is getNumSubElements().
	SubElementsSpec const* getSubElements();
	// Get the number of subelements.
	int getNumSubElements();

	// Evaluate the symbolic drawing with the current environment, and return the
	// evaluated numeric drawing.
    drawing::Drawing const* evalDrawing();

	// Get a pointer to the symbolic drawing.
	Drawing2S* getDrawing2S();
	// Get a pointer to the symbolic mesh.
	Mesh3S* getMesh3S();

	// transformation from 2d to 3d
	void get2D3Dpoints(Eigen::MatrixXd& points2D, Eigen::MatrixXd& points3D);
	Eigen::MatrixXd get2Dto3Dtransform();
	Eigen::MatrixXd get3Dto2Dtransform();

	// TODO(adriana): Document these 5 functions please.
	void addCenterOfMass(CenterOfMass &centerOfMass);
	void Element_Symbolic::computeSymbCenterOfMass();

	void computeTriangularization();

	void rotateCenterOfMass(Eigen::Vector3d const& center, Eigen::Quaterniond const& rotation);
	void translateCenterOfMass(Eigen::Vector3d const& trans);

	// Return a linear combination of the vertices of the mesh. This is a special kind of LinearExpr, where
	// each Symbol has nullptr as owner and K as id, where K is the index of the vertex. The constant term
	// of the LinearExpr is ignored.
	LinearExpr getSymbVertexExpr(Eigen::Vector3d v);

	// Evaluate a symbolic vertex expression (same definition as return value of getSymbVertexExpr) to the
	// symbolic point.
	Point3S evalSymbVertexExpr(LinearExpr const& le);

private:
	// These contain special linearexprs, in which each Symbol has nullptr as owner and K as id, where K is the index of the vertex.
	// The constant term of the LinearExpr is ignored. This representation allows us to keep triangle indices while be able to adapt
	// to changes to Mesh3S without retriangularizing.
	std::vector<std::vector<LinearExpr>> triIndexes;
	std::vector<int> faceIndices;
	Mesh3S* mesh3S;
    Drawing2S* drawing2S;
	SubElementsSpec* subElements;
	CenterOfMassSymb symbCenterOfMass;

	DISALLOW_COPY_AND_ASSIGN(Element_Symbolic);
};

}
#endif