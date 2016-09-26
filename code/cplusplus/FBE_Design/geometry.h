#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__
#include <vector>
#include "TriMesh.h"
#include <Eigen/Dense>

//#include "btBulletDynamicsCommon.h"
//#include "BulletCollision/CollisionShapes/btBoxShape.h"

class btBvhTriangleMeshShape;
class btTriangleIndexVertexArray ;
class btCollisionShape; 

namespace FabByExample{



class Geometry{
public:

	TriMesh *mesh;
	Geometry(std::string filename);
	Geometry(TriMesh * _mesh);
	Geometry();
	Geometry(const std::vector<TriMesh*> & meshvec);
	~Geometry() ;

	TriMesh* getMesh();
	std::string getSourceFileName();
	void addMesh(const TriMesh* m, bool calcnormals = true);
	void write(std::string file);
	bool checkIntersection(Geometry* geo2, double err =0);
	static bool checkBBoxIntersection(TriMesh::BBox box1, TriMesh::BBox box2, double err);
	static point getBBoxIntersection(TriMesh::BBox box1, TriMesh::BBox box2, double err);
	bool checkIntersectionWithResolution(Geometry* geo2);
	bool checkIfIsContainedIn(Geometry* geo2);
	double getBBoxVolume();
	void getUniformSamples(std::vector<point>  & points, std::vector<point>  & normals, double sampPar);
	point getCenter();
	point getMin();
	point getSize();
	TriMesh* getTransformedMesh(point & transVec, point & scaleVec);
	void rotY();
	void applyScale(point & scaleVec);
	void applyTrans(point & scaleVec);
	void applyRot(std::vector<double> & rotVec);
	void add(Geometry* geo, bool calcnormals = true);
	Geometry* compress(); 
	void addAllPoints(std::vector<point> & points);
	void addAllPoints(std::vector<Eigen::Vector3d> & points);

	void buildBtCollision();
	void destroyBtCollision();

	btCollisionShape* m_btTriangleMesh;
	btTriangleIndexVertexArray * m_btTriangleIndexVertexArray;

private:
	std::string sourceFileName;
};

}

#endif