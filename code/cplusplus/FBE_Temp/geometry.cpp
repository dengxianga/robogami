#include "geometry.h"
#include "params.h"
#include "TriMesh.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include "XForm.h"
#include "debugging.h"
#include "TriMesh_algo.h"

#define MARGIN   1

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

using namespace std;
using namespace FabByExample;


class BulletWorldContext {
public:
	BulletWorldContext() {
		collisionConfiguration = new btDefaultCollisionConfiguration();
		dispatcher = new btCollisionDispatcher(collisionConfiguration);
		btVector3 worldMin(-1000,-1000,-1000);
		btVector3 worldMax(1000,1000,1000);
		broadphase = new btAxisSweep3(worldMin,worldMax);
		collisionWorld = new btCollisionWorld(dispatcher,broadphase,collisionConfiguration);
		btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
	};
	~BulletWorldContext() {
		delete collisionConfiguration;
		delete dispatcher;
		delete broadphase;
		delete collisionWorld;
	}
	
	struct btRt : public btCollisionWorld::ContactResultCallback
	{
		virtual	btScalar	addSingleResult(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
		{
			collide = true;
			return 0;
		}

		virtual bool needsCollision(btBroadphaseProxy* proxy0) const
		{
			bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
			collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
			return collides;
		}

		bool collide;
	};
	bool Collide( btCollisionShape * g0,
		          btCollisionShape * g1, float margin = 0.0) {
		if (g0 != NULL && g1 != NULL) {
			btCollisionObject  c0;
			btCollisionObject  c1;

			btMatrix3x3 basisA;
			basisA.setIdentity();

			btMatrix3x3 basisB;
			basisB.setIdentity();

			c0.getWorldTransform().setBasis(basisA);
			c1.getWorldTransform().setBasis(basisB);

			if (g0 != NULL && g1 != NULL) {
				g0->setMargin(margin);
				g1->setMargin(margin);
				c0.setCollisionShape(g0);
				c1.setCollisionShape(g1);
				colRet.collide = false;
				collisionWorld->contactPairTest(&c0, &c1, colRet);
				return colRet.collide;
			}
		}
		return false;
	}


private:
	btRt colRet;
	btDefaultCollisionConfiguration* collisionConfiguration ;
	btCollisionDispatcher* dispatcher ;
	btBroadphaseInterface*	broadphase ;
	btCollisionWorld* collisionWorld;
};


// Create a single instance of the bullet world for collision detection
static BulletWorldContext  worldContext;


Geometry::Geometry(std::string filename) 
: m_btTriangleIndexVertexArray(NULL), m_btTriangleMesh(NULL), sourceFileName(filename)
{
	//if(MODEL_TYPE == 0){
		stringstream fileName1, fileName2;
		fileName1 << SHARED_DATA_DIR << "ParametricDesigns\\STL\\" << filename;
		fileName2 << SHARED_DATA_DIR << "STL\\" << filename;

		std::ifstream myfile (fileName1.str());
		if (myfile.is_open())
		{
			myfile.close();
			mesh = TriMesh::read(fileName1.str().c_str());
		}
		else{
			mesh = TriMesh::read(fileName2.str().c_str());
		}
		if(mesh == NULL){
			std::string fileName2_str = fileName2.str();
			fileName2_str.replace(7, 1, "C");
			mesh = TriMesh::read(fileName2_str.c_str());
			if(mesh == NULL){
				LOG(FATAL) << "error opening file:" << fileName2_str;
			}
		}
	//}
	//if(MODEL_TYPE == 1){
	//	std::string fileName2 = filename;
	//	fileName2.insert(0, "..\\SW2010\\Test Chairs\\STL\\" ); 
	//	mesh = TriMesh::read(fileName2.c_str());
	//	if(mesh == NULL){
	//		std::cout << "error opening file:" << fileName2.c_str() << std::endl;
	//		system("pause");
	//	}

	//}
}


Geometry::Geometry(TriMesh * _mesh) 
	 :  m_btTriangleIndexVertexArray(NULL), m_btTriangleMesh(NULL) {
	mesh = _mesh;
}


Geometry::Geometry()
	 :  m_btTriangleIndexVertexArray(NULL), m_btTriangleMesh(NULL) {
	mesh = new TriMesh;

}

Geometry::Geometry(const std::vector<TriMesh*> & meshvec) 
	 :  m_btTriangleIndexVertexArray(NULL), m_btTriangleMesh(NULL) {
	mesh = new TriMesh;
	for (unsigned int i =0; i< meshvec.size(); i++){
		addMesh(meshvec[i]);
	}

	m_btTriangleMesh = NULL;
	m_btTriangleIndexVertexArray = NULL;
}

Geometry::~Geometry() {
	if (mesh != NULL)
		delete mesh;
	destroyBtCollision();
}
TriMesh* Geometry::getMesh(){
	//std::cout << "getmesh" << std::endl; system("PAUSE"); 
	return mesh;
  }
std::string Geometry::getSourceFileName() {
	return sourceFileName;
}
void Geometry::addMesh(const TriMesh* m, bool calcnormals){

	int onv = mesh->vertices.size();
	if (calcnormals) {
		mesh->need_normals();
	}
	mesh->vertices.insert(mesh->vertices.end(), m->vertices.begin(), m->vertices.end());

	for(unsigned int i=0; i< m->faces.size();i++) {
		mesh->faces.push_back( TriMesh::Face( m->faces[i].v[0] + onv, 
											m->faces[i].v[1] + onv, 
											m->faces[i].v[2] + onv) );
	}



	if (!m->normals.empty()){
		mesh->normals.insert(mesh->normals.end(), m->normals.begin(),m->normals.end());
	}
	else if (calcnormals) {
		mesh->normals.clear();
		mesh->need_normals();
	}

}

void Geometry::write(std::string file){
	mesh->need_normals();
	mesh->need_tstrips();
	mesh->need_bsphere();
	mesh->write(file.c_str());
}

bool Geometry::checkIntersection(Geometry* geo2, double err){
	TriMesh *m = getMesh();
	m->need_bbox();
	TriMesh *m2 = geo2->getMesh();
	m2->need_bbox();

	//this->buildBtCollision();
	//geo2->buildBtCollision();
	//return worldContext.Collide(this->m_btTriangleMesh, geo2->m_btTriangleMesh, MARGIN);

	//std::cout << " m1 =  (" <<  m->bbox.min[0] <<  ","  << m->bbox.max[0]  << ") ("  << m->bbox.min[1] <<  "," <<  m->bbox.max[1] <<  ") ("  <<  m->bbox.min[2] << "," << m->bbox.max[2] << ")" << std::endl;
	//std::cout << " m2 =  (" <<  m2->bbox.min[0] <<  ","  << m2->bbox.max[0]  << ") ("  << m2->bbox.min[1] <<  "," <<  m2->bbox.max[1] <<  ") ("  <<  m2->bbox.min[2] << "," << m2->bbox.max[2] << ")" << std::endl;
		
	bool intersection = false;
	if (    ( (m->bbox.min[0] - m2->bbox.max[0]) < err) && ( (m2->bbox.min[0] - m->bbox.max[0]) < err )&& 
   			( (m->bbox.min[1] - m2->bbox.max[1]) < err) && ( (m2->bbox.min[1] - m->bbox.max[1]) < err )&& 
			( (m->bbox.min[2] - m2->bbox.max[2]) < err) && ( (m2->bbox.min[2] - m->bbox.max[2]) < err )){
		intersection = true;	
	}
	return intersection;
		
}

bool Geometry::checkBBoxIntersection(TriMesh::BBox bbox1, TriMesh::BBox bbox2, double err){
		
	bool intersection = false;
	if (    ( (bbox1.min[0] - bbox2.max[0]) < err) && ( (bbox2.min[0] - bbox1.max[0]) < err )&& 
   			( (bbox1.min[1] - bbox2.max[1]) < err) && ( (bbox2.min[1] - bbox1.max[1]) < err )&& 
			( (bbox1.min[2] - bbox2.max[2]) < err) && ( (bbox2.min[2] - bbox1.max[2]) < err )){
		intersection = true;	
	}
	return intersection;


}

point Geometry::getBBoxIntersection(TriMesh::BBox bbox1, TriMesh::BBox bbox2, double err){
	point vec;

	VLOG(3) << err;
	for(int ax = 0; ax <3; ax++){
		if ( ((bbox1.min[ax] - bbox2.max[ax]) < err) && ((bbox2.min[ax] - bbox1.max[ax]) < err )){
			vec[ax] = 0;
		}
		else{
			vec[ax] = 1;
		}
	}
	return vec;

}


double computeDist(point face1[], point face2[]){

	double dist = 99999999;

	for(int ax1 = 0; ax1 < 3; ax1++){
		for(int ax2 = 0; ax2 < 3; ax2++){
			double newdist = len(face1[ax1] - face2[ax2]);
			if(newdist < dist)
				dist = newdist;
		}
	}

	return dist;
}

// This function is obsoleted, the checkIntersection should be faster
bool Geometry::checkIntersectionWithResolution(Geometry* geo2){
/*	if(!checkIntersection(geo2))
		return false;
//	return worldContext.Collide(this, geo2);
	double dist  = 999999999;

	for(size_t i =0; i< mesh->faces.size(); i++){
		for(size_t j =0; j< geo2->mesh->faces.size(); j++){
			point face1[3];
			point face2[3];
			for (int ax = 0; ax< 3; ax++){
				face1[ax] = mesh->vertices[mesh->faces[i].v[0]];
				face1[ax] = mesh->vertices[mesh->faces[i].v[0]];
				face1[ax] = mesh->vertices[mesh->faces[i].v[0]];

				face2[ax] = geo2->mesh->vertices[geo2->mesh->faces[i].v[0]];
				face2[ax] = geo2->mesh->vertices[geo2->mesh->faces[i].v[0]];
				face2[ax] = geo2->mesh->vertices[geo2->mesh->faces[i].v[0]];
			}

			double newDist = computeDist(face1, face2);
			if(newDist < dist)
				dist = newDist;

		}
	}

	//std::cout <<  "calculated dist = " << dist << std::endl;
	//system("pause"); 
	if(dist < 3)
		return true;
		
	return false;


	*/
	this->buildBtCollision();
	geo2->buildBtCollision();
	return worldContext.Collide(this->m_btTriangleMesh, geo2->m_btTriangleMesh, MARGIN);


}


bool Geometry::checkIfIsContainedIn(Geometry* geo2){

	TriMesh *m = getMesh();
	m->need_bbox();
	TriMesh *m2 = geo2->getMesh();
	m2->need_bbox();
	double dx = m->bbox.max[0] - m->bbox.min[0];
	double dy = m->bbox.max[1] - m->bbox.min[1];
	double dz = m->bbox.max[2] - m->bbox.min[2];

	double dx2 = m2->bbox.max[0] - m2->bbox.min[0];
	double dy2 = m2->bbox.max[1] - m2->bbox.min[1];
	double dz2 = m2->bbox.max[2] - m2->bbox.min[2];


	bool isContained = false;
	if ( ((dx2-dx)> -0.01) && ((dy2-dy)> -0.01) && ((dz2-dz)> -0.01)){
		isContained = true;	
	}

	//std::cout << "------------------------" << std::endl;
	//std::cout << " big box =  (" <<  dx2 <<  ","  <<  dy2 <<  "," <<  dz2 <<  ")" << std::endl; 
	//std::cout << " sml box =  (" <<  dx <<  ","  <<  dy <<  "," <<  dz <<  ")" << std::endl; 

	return isContained;
}



double Geometry::getBBoxVolume(){

	mesh->need_bbox();
	double vol = 1;
	for(int i = 0; i< 3; i++){
		vol*= (mesh->bbox.max[i] - mesh->bbox.min[i]);
	}

	return vol;
}

void Geometry::getUniformSamples(std::vector<point>  & points, std::vector<point>  & normals, double sampPar){

	mesh->calculateFaceAreas() ;
	mesh->need_normals();
	mesh->need_tstrips();
	mesh->need_bsphere();
	double min_area = std::numeric_limits< double> ::infinity();
	double max_area = 0;
	for(int i = 0; i < (int)mesh->faces.size(); ++i)
    {
		double area = mesh->faces[i].area;
		if(area< min_area){
			min_area = area;
		}
		if(area > max_area){
			max_area = area;
		}
	}

	//point minPoint(std::numeric_limits< float> ::infinity(), std::numeric_limits< float> ::infinity(), std::numeric_limits< float> ::infinity());
	for(int i = 0; i < (int)mesh->faces.size(); ++i)
    {
		double area = mesh->faces[i].area;	
		//double samplesPar = area/max_area*sampPar; 
		double samplesPar = area/sampPar; 
		int N = round(sqrt(samplesPar));
		if(N> 0){
			//std::cout <<"triagle area = " << area << std::endl;
			const point & v0 = mesh->vertices[mesh->faces[i].v[0]];    
			const point & v1 = mesh->vertices[mesh->faces[i].v[1]];    
			const point & v2 = mesh->vertices[mesh->faces[i].v[2]];   
			vec a = v0-v1, b = v1-v2, c = v2-v0;
			double l2a = len2(a), l2b = len2(b), l2c = len2(c);
			vec facenormal = a CROSS b;
			float delta = 1.0f/(N - (1.0f/10.0f));
			for(float l0 = delta/20; l0 < 1; l0 += delta){
				for(float l1 = delta/20; l1 < 1; l1 += delta){
					if ((l0 + l1) <= 1){
						point thispoint = l0*v0 + l1*v1 + (1- l0 - l1)*v2;
						points.push_back(thispoint);
						normals.push_back(normalize(facenormal));
						//if(thispoint[0]< minPoint[0]){ minPoint[0] = thispoint[0];}
						//if(thispoint[1]< minPoint[1]){ minPoint[1] = thispoint[1];}
						//if(thispoint[2]< minPoint[2]){ minPoint[2] = thispoint[2];}

					}
				}
			}
		}
	}

	//for(int i= 0; i< points.size(); i++){
	//	points[i] = points[i] - minPoint;
	//}

	//std::cout << "minPoint = " << minPoint << std::endl;
	//return minPoint; 
}

point Geometry::getCenter(){

	mesh->need_bbox();
	point center = mesh->bbox.max+ mesh->bbox.min;
	for(int i = 0; i<3; i++){
		center[i] = center[i]/2;
	}
	return center;
}

point Geometry::getMin(){
	mesh->need_bbox();
	return  mesh->bbox.min;
}

point Geometry::getSize(){
	mesh->need_bbox();
	return  mesh->bbox.size();
}


TriMesh* Geometry::getTransformedMesh(point & transVec, point & scaleVec){
	Geometry *m = new Geometry;
	m->addMesh(mesh);
	m->mesh->need_bbox();
	double prevX = m->mesh->bbox.min[0];
	double prevY = m->mesh->bbox.min[1];
	double prevZ = m->mesh->bbox.min[2];
	xform trans_pre(1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			- prevX,  - prevY, -prevZ, 1);
	xform trans_pos(1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			prevX,  prevY, prevZ, 1);
	xform scale(scaleVec[0], 0, 0, 0,
				0,scaleVec[1], 0, 0,
				0, 0, scaleVec[2], 0,
				0, 0, 0, 1);
	xform trans(1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			transVec[0], transVec[1], transVec[2], 1);

	
	apply_xform(m->mesh, trans_pre);			
	apply_xform(m->mesh, scale);
	apply_xform(m->mesh, trans_pos);
	apply_xform(m->mesh, trans);


	m->getMesh()->need_normals();
	m->getMesh()->need_tstrips();
	m->getMesh()->need_bsphere();
	
	return m->mesh;
}

void Geometry::applyTrans(point & transVec){
	
//	xform trans(1, 0, 0, 0,
//			0, 1, 0, 0,
//			0, 0, 1, 0,
//			transVec[0], transVec[1], transVec[2], 1);

//	apply_xform(mesh, trans);
	trans(mesh, transVec);
//	mesh->need_normals();
//	mesh->need_tstrips();
//	mesh->need_bsphere();

	destroyBtCollision();
}

void Geometry::applyRot(std::vector<double> & rotVec){
	
	xform trans(rotVec[0], rotVec[1], rotVec[2], 0,
				rotVec[3], rotVec[4], rotVec[5], 0,
				rotVec[6], rotVec[7], rotVec[8], 0,
				0, 0, 0, 1);

	apply_xform(mesh, trans);

	mesh->need_normals();
	mesh->need_tstrips();
	mesh->need_bsphere();

}

void Geometry::applyScale(point & scaleVec)
{

	mesh->need_bbox();
	TriMesh::BBox prevBox = mesh->bbox;
	

#pragma omp parallel for
	for(unsigned int vidx =0; vidx<mesh->vertices.size(); vidx++) {
		mesh->vertices[vidx] = 
			(mesh->vertices[vidx] - prevBox.min) * scaleVec + prevBox.min;
	}
	mesh->bbox.min = (mesh->bbox.min - prevBox.min) * scaleVec + prevBox.min;
	mesh->bbox.max= (mesh->bbox.max - prevBox.min) * scaleVec + prevBox.min;
	if (mesh->bsphere.valid) {
		mesh->bsphere.valid = false;
	}
	/*
	apply_xform(mesh, trans_pre);			
	apply_xform(mesh, scale);
	apply_xform(mesh, trans_pos);

	apply_xform(mesh, trans_pos*scale*trans_pre);			
	*/
//	mesh->need_normals();
//	mesh->need_tstrips();
//	mesh->need_bsphere();
	
}

void Geometry::rotY(){

	mesh->need_normals();
	mesh->need_tstrips();
	mesh->need_bsphere();
	
	xform rotate(0, 0, 1, 0,
				0, 1, 0, 0,
				-1, 0, 0, 0,
				0, 0, 0, 1);

	apply_xform(mesh, rotate);	


	mesh->need_normals();
	mesh->need_tstrips();
	mesh->need_bsphere();
	mesh->need_bbox();
}

void Geometry::add(Geometry* geo, bool calcnormals){
	destroyBtCollision();
	addMesh(geo->getMesh(), calcnormals);
}

Geometry * Geometry::compress(){
	
	size_t MAX_VERT = 2000;

	if(mesh->vertices.size() < MAX_VERT)
		return this;

	TriMesh * compMesh = mesh->crunch(); 
	int n_iter = 0;
	while((compMesh->vertices.size() > MAX_VERT) && (n_iter < 10)){
		TriMesh *compMesh2 = mesh->crunch(); 
		delete compMesh;
		compMesh = compMesh2;
		n_iter++; 
	}

	Geometry* compGeo = new Geometry(compMesh);

	return compGeo; 
}
void Geometry::addAllPoints(std::vector<point> & points){
	for (int i = 0; i < mesh->vertices.size(); i++){
		bool shoudAdd = true;
		for(int j = 0; j <points.size(); j++){
			//if(len2(mesh->vertices[i] - points[j]) < 0.1){
			//	shoudAdd = false;
			//}
		}
		if(shoudAdd)
			points.push_back(mesh->vertices[i]);
	}
	//std::cout << "npoints = " << points.size() << std::endl;
}
void Geometry::addAllPoints(std::vector<Eigen::Vector3d> & points) {
	for (int i = 0; i < mesh->vertices.size(); i++){
		Eigen::Vector3d temppoint(mesh->vertices[i][0],mesh->vertices[i][1],mesh->vertices[i][2]);
		points.push_back(temppoint);
	}
}


void Geometry::destroyBtCollision() {
	if (m_btTriangleIndexVertexArray != NULL)
		delete m_btTriangleIndexVertexArray;
	if (m_btTriangleMesh != NULL)
		delete m_btTriangleMesh ;
}


void Geometry::buildBtCollision() {
	if (mesh->faces.size() == 0)
		return;
	
	

	if (m_btTriangleIndexVertexArray == NULL) {
		m_btTriangleIndexVertexArray = new btTriangleIndexVertexArray(mesh->faces.size(),
			&mesh->faces[0].v[0],
			sizeof(mesh->faces[0]), 
			mesh->vertices.size(),
			(btScalar*) &mesh->vertices[0][0],
			sizeof(mesh->vertices[0])
			);
	}
	
	if (m_btTriangleIndexVertexArray != NULL && m_btTriangleMesh == NULL)  {
		btGImpactMeshShape  *gImageShape = new btGImpactMeshShape(m_btTriangleIndexVertexArray);
		gImageShape->setLocalScaling(btVector3(1,1,1));
		gImageShape->setMargin(0.0);
		gImageShape->updateBound();
		m_btTriangleMesh  =  gImageShape ;
	}
}