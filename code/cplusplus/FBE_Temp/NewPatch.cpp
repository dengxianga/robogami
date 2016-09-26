#include "NewPatch.h"
#include "templateElement.h" 
#include "element_symbolic.h"
#include "../FBE_Design/geometry.h"
using namespace FabByExample;

const int DIST_EPSILON = 10;

void ServoSpacing::display(){
	VLOG(3) << "Servo spacing with position" << alpha.eval(SymbolicAssignment::USE_CURRENT_VALUES) << std::endl;
	VLOG(3) << "Associated Templates : ";
	for each (auto t in associatedTemplates){
		VLOG(3) << t->getName() << ", " ;
	}
	VLOG(3) << std::endl;
			
	VLOG(3) << "  height = " << separation_h.eval(SymbolicAssignment::USE_CURRENT_VALUES) << std::endl;
	VLOG(3) << "  weights = " ;
	for each (auto w in separation_w){
		VLOG(3) << w.eval(SymbolicAssignment::USE_CURRENT_VALUES) << " ";
	}
	VLOG(3) << std::endl;

}


NewPatch::~NewPatch(){
}


bool NewPatch::isRelated(Template * temp){
	auto tempList = temp->getTemplatesByScope(TreeScope::DESCENDANTS);
	for each (auto t in tempList){
		if( t == element){
			return true;
		}
	}
	return false;
}

double NewPatchLine2D3D::getDistanceToPoint(const Eigen::Vector3d & refPoint){
	//Eigen::Vector3d current_pos = getReferencePoint();
	//Eigen::Vector3d line_point_to_point = current_pos - refPoint;

	//Eigen::Vector3d direction = this->getDirection();
	//Eigen::Vector3d proj = line_point_to_point.dot(direction) * direction;

	//Eigen::Vector3d diff = line_point_to_point - proj;

	//return diff.norm();

	Eigen::Vector3d v1 = this->getVertex1();
	Eigen::Vector3d v2 = this->getVertex2();


	Eigen::Vector3d direction = this->getDirection();

	if ((refPoint - v1).dot(direction) < 0 ){
		return (refPoint - v1).norm();
	}
	else if ((refPoint - v2).dot(direction)>0){
		return (refPoint - v2).norm();
	}
	else{
		//in the middle of the line segment
		Eigen::Vector3d line_point_to_point = v1 - refPoint;
		Eigen::Vector3d proj = line_point_to_point.dot(direction) * direction;
		Eigen::Vector3d diff = line_point_to_point - proj;
		return diff.norm();
	}

	




}

Eigen::Vector3d NewPatchLine2D3D::getDirection() {
	Point3S vertex2 = this->getVertexS2();
	Point3S vertex1 = this->getVertexS1();
	// NOTE: Only works for symbolic elements
	return (vertex2.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES) - vertex1.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES)).normalized();
}

Eigen::Vector3d NewPatchLine2D3D::getReferencePoint() {
	Point3S vertex2 = this->getVertexS2();
	Point3S vertex1 = this->getVertexS1();
	// NOTE: Only works for symbolic elements
	return vertex1.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
}

Eigen::Vector3d NewPatchLine2D3D::getVertex1() {
	Point3S vertex2 = this->getVertexS2();
	Point3S vertex1 = this->getVertexS1();
	// NOTE: Only works for symbolic elements
	return vertex1.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
}

Eigen::Vector3d NewPatchLine2D3D::getVertex2() {
	Point3S vertex2 = this->getVertexS2();
	Point3S vertex1 = this->getVertexS1();
	// NOTE: Only works for symbolic elements
	return vertex2.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
}

Eigen::Vector2d NewPatchLine2D3D::getReferencePoint2D() {
	Point2S drawingVertex2 = this->getDrawingVertexS2();
	Point2S drawingVertex1 = this->getDrawingVertexS1();
	// NOTE: Only works for symbolic elements
	return drawingVertex1.evalVector2d(SymbolicAssignment::USE_CURRENT_VALUES);
}
Eigen::Vector2d NewPatchLine2D3D::getDrawingVertex1() {
	Point2S drawingVertex2 = this->getDrawingVertexS2();
	Point2S drawingVertex1 = this->getDrawingVertexS1();
	// NOTE: Only works for symbolic elements
	return drawingVertex1.evalVector2d(SymbolicAssignment::USE_CURRENT_VALUES);
}
Eigen::Vector2d NewPatchLine2D3D::getDrawingVertex2() {
	Point2S drawingVertex2 = this->getDrawingVertexS2();
	Point2S drawingVertex1 = this->getDrawingVertexS1();
	// NOTE: Only works for symbolic elements
	return drawingVertex2.evalVector2d(SymbolicAssignment::USE_CURRENT_VALUES);
}


Eigen::Vector2d NewPatchLine2D3D::getDirection2D() {
	Point2S drawingVertex2 = this->getDrawingVertexS2();
	Point2S drawingVertex1 = this->getDrawingVertexS1();
	return (drawingVertex2.evalVector2d(SymbolicAssignment::USE_CURRENT_VALUES) - drawingVertex1.evalVector2d(SymbolicAssignment::USE_CURRENT_VALUES)).normalized();
}


NewPatchLine2D3D::NewPatchLine2D3D(Template* _element, int edgeId, int vertexId1, int vertexId2) : NewPatch(_element), edgeId(edgeId) {
	this->vertexId1 = vertexId1;
	this->vertexId2 = vertexId2;
}

bool NewPatchLine2D3D::isTheSame(NewPatch * patch2){
	// TODO: is this necessary at all?
	if(patch2->getType() == NewPatch::PatchType::LINE2D3D){
		NewPatchLine2D3D * patchLine2 = (NewPatchLine2D3D *)patch2;
		if(patchLine2->getElement() != this->getElement())
			return false;

		if((patchLine2->getDirection() - this->getDirection()).norm() > 0.1)
			return false;

		return true;
	}
	return false;
}

double NewPatchLine2D3D::compare(NewPatch* _reference){

	if(_reference->getType() == NewPatch::PatchType::LINE2D3D){

		NewPatchLine2D3D * refLine = (NewPatchLine2D3D *)_reference;
		Eigen::Vector3d n2 = refLine->getDirection();
		Eigen::Vector3d n1 = this->getDirection();
		double proj = n1.dot(n2);
		double normalError = (n1- proj*n2).norm();
		if(normalError >0.1){
			return std::numeric_limits<double>::infinity();
		}

		double pointError = (this->getDirection().cross(getReferencePoint() - refLine->getReferencePoint())).norm();
		return pointError;

	}
	return std::numeric_limits<double>::infinity();
}

// Get the distance between two patches, 
// which are defined to be the mean distance between the 
// three pairs of points (v1, midPoint, v2)

double NewPatchLine2D3D::getDistanceToPatch( NewPatchLine2D3D*  refPatch){
#if 0
	const Eigen::Vector3d vr1 = refPatch->getVertex1();
	const Eigen::Vector3d vr2 = refPatch->getVertex2();
	const Eigen::Vector3d vr3 = (vr1 + vr2) / 2;
	const Eigen::Vector3d v1 = this->getVertex1();
	const Eigen::Vector3d v2 = this->getVertex2();
	const Eigen::Vector3d v3 = (v1 + v2) / 2; //midPoints

	double distances[6];
	distances[0] = refPatch->getDistanceToPoint(v1);
	distances[1] = refPatch->getDistanceToPoint(v2);
	distances[2] = refPatch->getDistanceToPoint(v3);
	distances[3] = this->getDistanceToPoint(vr1); 
	distances[4] = this->getDistanceToPoint(vr2);
	distances[5] = this->getDistanceToPoint(vr3);

	//find the mean of all distances
	double sum = 0.0;
	for (int i = 1; i < 6; i++){
		sum += distances[i];

	}
	return sum / 6;
#else
	return ((refPatch->getVertex1() + refPatch->getVertex2()) / 2 - (this->getVertex1() + this->getVertex2()) / 2).norm();
#endif

}

const int sameDirectionThreshold = 20;
NewPatchLine2D3D::DirectionResult NewPatchLine2D3D::matchDirection(NewPatchLine2D3D* otherPatch) {
	auto dir = getDirection();
	auto otherDir = otherPatch->getDirection();
	auto dot = dir.dot(otherDir);
	if (dot > cos(sameDirectionThreshold * 3.1415 / 180)) {
		return DirectionResult::SAME;
	}
	else if (dot < -cos(sameDirectionThreshold * 3.1415 / 180)) {
		return DirectionResult::OPPOSITE;
	}
	else {
		return DirectionResult::DIFFERENT;
	}
}

Element_Symbolic* NewPatchLine2D3D::getElement(){
	return dynamic_cast<Element_Symbolic*>((dynamic_cast<TemplateElement*>(element))->getElement());
}
Point3S NewPatchLine2D3D::getVertexS1(){
	return this->getElement()->getMesh3S()->vertices[vertexId1];
}
Point3S NewPatchLine2D3D::getVertexS2(){
	return this->getElement()->getMesh3S()->vertices[vertexId2];
}
Point2S NewPatchLine2D3D::getDrawingVertexS1(){
	return this->getElement()->getDrawing2S()->vertices[vertexId1];
}
Point2S NewPatchLine2D3D::getDrawingVertexS2(){
	return this->getElement()->getDrawing2S()->vertices[vertexId2];

}

FabDebugging::DebugInfo* NewPatchLine2D3D::getDebugInfo() {
	auto info = new FabDebugging::DebugInfo();
	info->setTypeName("NewPatch");
	info->setShortDescription(concat() << "Edge Patch [" << vertexId1 << " -> " << vertexId2 << "] @ " << (long long)(this));
	info->putReferenceProperty("element", element);
	info->putReferenceProperty("connection", parentConnection);
	auto dir = getDirection();
	info->putStringProperty("direction", concat() << "(" << dir[0] << ", " << dir[1] << ", " << dir[2] << ")");
	return info;
}

namespace {
	double areaOfTriangle(Eigen::Vector2d const& a, Eigen::Vector2d const& b, Eigen::Vector2d const& c){
		return 0.5 * (a[0] * b[1] + b[0] * c[1] + c[0] * a[1] - a[1] * b[0] - b[1] * c[0] - c[1] * a[0]);
	}
}


// Returns whether the drawing edge goes counterclockwise around the face.
// Currently assumes shape is convex.
bool NewPatchLine2D3D::isDrawingEdgeCounterClockwise() {
	Eigen::Vector2d sum = Eigen::Vector2d::Zero();
	int numVertices = 0;
	for each (auto const& p2s in this->getElement()->getDrawing2S()->vertices) {
		sum += p2s.evalVector2d(SymbolicAssignment::USE_CURRENT_VALUES);
		numVertices++;
	}
	sum /= numVertices;
	auto v1 = getDrawingVertex1();
	auto v2 = getDrawingVertex2();
	auto result = areaOfTriangle(sum, v1, v2) > 0;
	return result;
}



//------------------------------------------------------------------------------

ServoLinePatch::ServoLinePatch(Template * _element, Point3S _vertex1, Point3S _vertex2, Eigen::Vector3d _normal ) : NewPatch(_element){

	vertex1 = _vertex1;
	vertex2 =_vertex2;
	normal = _normal;
}


double ServoLinePatch::getDistanceToPoint(const Eigen::Vector3d & refPoint){

	Eigen::Vector3d v1 = vertex1.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
	Eigen::Vector3d v2 = vertex2.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);


	Eigen::Vector3d direction = (v2 - v1).normalized();

	if ((refPoint - v1).dot(direction) < 0 ){
		return (refPoint - v1).norm();
	}
	else if ((refPoint - v2).dot(direction)>0){
		return (refPoint - v2).norm();
	}
	else{
		//in the middle of the line segment
		Eigen::Vector3d line_point_to_point = v1 - refPoint;
		Eigen::Vector3d proj = line_point_to_point.dot(direction) * direction;
		Eigen::Vector3d diff = line_point_to_point - proj;
		return diff.norm();
	}

	
}



void ServoLinePatch::rotate(Eigen::Vector3d const& center_rot, Eigen::Quaterniond const& rotation){

	vertex1 = vertex1.translate(-center_rot);
	vertex1 = vertex1.times(rotation.toRotationMatrix());
	vertex1 = vertex1.translate(center_rot);


	vertex2 = vertex2.translate(-center_rot);
	vertex2 = vertex2.times(rotation.toRotationMatrix());
	vertex2 = vertex2.translate(center_rot);
	
	normal = rotation.matrix()*normal;

}

void ServoLinePatch::translate(Eigen::Vector3d const& trans){

	vertex1 = vertex1.translate(trans);
	vertex2 = vertex2.translate(trans);
}



Geometry* ServoLinePatch::getDebugGeo(){

	Eigen::Quaterniond rot_q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),normal);
	auto rot_mat = rot_q.toRotationMatrix();
	std::vector<double> rot(9);
	for (int i = 0; i< 9; i++){
	rot[i] = rot_mat(i);
	}
	Geometry * geo = new Geometry();
	std::vector<Eigen::Vector3d> points;
	points.push_back(this->vertex1.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES));
	points.push_back(this->vertex2.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES));
	for each ( auto p in points){
		point p_point (p.x(), p.y(), p.z());
		Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\arrow.obj"));
		sphere->applyRot(rot);
		sphere->applyTrans(p_point);		
		geo->add(sphere);
	}

	return geo;
}


//------------------------------------------------------------------------------

ServoPointPatch::ServoPointPatch(Template * _element, Point3S _center, LinearExpr _separation, Eigen::Vector3d _normal ) : NewPatch(_element){

	center = _center;	
	separation = _separation;
	normal = _normal;
}


double ServoPointPatch::getDistanceToPoint(const Eigen::Vector3d & refPoint){

	Eigen::Vector3d v1 = center.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
	
	return (v1 - refPoint).norm();
	
	
}



void ServoPointPatch::rotate(Eigen::Vector3d const& center_rot, Eigen::Quaterniond const& rotation){

	center = center.translate(-center_rot);
	center = center.times(rotation.toRotationMatrix());
	center = center.translate(center_rot);
	normal = rotation.matrix()*normal;
	
}

void ServoPointPatch::translate(Eigen::Vector3d const& trans){

	center = center.translate(trans);
}


Geometry* ServoPointPatch::getDebugGeo(){

	Eigen::Quaterniond rot_q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),normal);
	auto rot_mat = rot_q.toRotationMatrix();
	std::vector<double> rot(9);
	for (int i = 0; i< 9; i++){
	rot[i] = rot_mat(i);
	}
	Geometry * geo = new Geometry();
	std::vector<Eigen::Vector3d> points;
	points.push_back(center.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES));
	for each ( auto p in points){
		point p_point (p.x(), p.y(), p.z());
		Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\arrow.obj"));
		sphere->applyRot(rot);
		sphere->applyTrans(p_point);		
		geo->add(sphere);
	}

	return geo;
}



Eigen::Vector3d ServoPointPatch::getCenter() {
	return center.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
}


//------------------------------------------------------------------------------

PeripheralPatch::PeripheralPatch(Template * _element, NewPatchLine2D3D *_p1, NewPatchLine2D3D *_p2, Eigen::Vector3d _normal ): NewPatch(_element){

	normal = _normal;
	p1 = _p1;
	p2 = _p2;
}


double PeripheralPatch::getDistanceToPoint(const Eigen::Vector3d & refPoint){

// NEEDS IMPLEEMNTATION
	return 0; 
	
}

void PeripheralPatch::rotate(Eigen::Vector3d const& center_rot, Eigen::Quaterniond const& rotation){

	normal = rotation.matrix()*normal;
	
}


double PeripheralPatch::getDistanceToPatch( PeripheralPatch*  refPatch){


	Eigen::Vector3d  center1 = 0.25*(this->getLinePatch1()->getVertex1() + 
						this->getLinePatch1()->getVertex2() +
						this->getLinePatch2()->getVertex1() +
						this->getLinePatch2()->getVertex2());

	Eigen::Vector3d center2 = 0.25*(refPatch->getLinePatch1()->getVertex1() + 
					refPatch->getLinePatch1()->getVertex2() +
					refPatch->getLinePatch2()->getVertex1() +
					refPatch->getLinePatch2()->getVertex2());

	return (center1-center2).norm();

//	double d1 = p1->getDistanceToPatch(refPatch->getLinePatch1())
//				+ p2->getDistanceToPatch(refPatch->getLinePatch2());
//
//	double d2 = p1->getDistanceToPatch(refPatch->getLinePatch2())
//				+ p2->getDistanceToPatch(refPatch->getLinePatch1());
//
//	double dist = (d1<d2)? d1:d2;
// 	return dist;

}

double PeripheralPatch::getSideOrientationDistanceToPatch( PeripheralPatch*  refPatch){


	Eigen::Vector3d dMain = p1->getDirection();
	Eigen::Vector3d dAdd = refPatch->getLinePatch1()->getDirection();
	Eigen::Vector3d dAddOpp = (-1.0)*dAdd;
	Eigen::Vector3d dAddFinal = ((dMain - dAdd).norm() < (dMain + dAdd).norm()) ? dAdd : dAddOpp;


	double dist = (dMain - dAddFinal).norm();
	return dist;

}


void PeripheralPatch::writeToFile(std::string filename){

	Geometry * geo = element->getGeometry();
	std::vector<Eigen::Vector3d> points;
	points.push_back(p1->getVertex1());
	points.push_back(p1->getVertex2());
	points.push_back(p2->getVertex1());
	points.push_back(p2->getVertex2());
	Eigen::Quaterniond rot_q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),normal);
	auto rot_mat = rot_q.toRotationMatrix();
	std::vector<double> rot(9);
	for (int i = 0; i< 9; i++){
		rot[i] = rot_mat(i);
	}

	for each ( auto p in points){
		point p_point (p.x(), p.y(), p.z());
		Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\arrow.obj"));
		sphere->applyRot(rot);		
		sphere->applyTrans(p_point);
		geo->add(sphere);
	}
	geo->write(filename); 

}



Geometry* PeripheralPatch::getDebugGeo(){

	Eigen::Quaterniond rot_q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),normal);
	auto rot_mat = rot_q.toRotationMatrix();
	std::vector<double> rot(9);
	for (int i = 0; i< 9; i++){
	rot[i] = rot_mat(i);
	}
	Geometry * geo = new Geometry();
	std::vector<Eigen::Vector3d> points;
	points.push_back(p1->getVertex1());
	points.push_back(p1->getVertex2());
	points.push_back(p2->getVertex1());
	points.push_back(p2->getVertex2());
	for each ( auto p in points){
		point p_point (p.x(), p.y(), p.z());
		Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\arrow.obj"));
		sphere->applyRot(rot);
		sphere->applyTrans(p_point);		
		geo->add(sphere);
	}

	return geo;
}
