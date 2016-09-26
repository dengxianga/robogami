#include "articulation.h"
#include "XForm.h"
#include "TriMesh_algo.h"
#include "TriMesh.h"
#include "geometry.h"
#include "controller.h"
using namespace FabByExample;




SymbolicTransformation::SymbolicTransformation(Controller * _controller, Point3S & _symbAxis, bool isTrans){

	//TODO: initailize controler

	if(_controller == nullptr){
		LOG(ERROR) << "the controller is null";
	}else{
		controller = _controller;
	}

	controller->display();

	//TODO: initailize symbolic axis
	symbAxis = _symbAxis; 

	//TODO: initailize axis
	//axis << 0.0, 0.0, -1.0;

	//Initialize val
	val = 0;
	//updateTime(0);
	this->isTrans = isTrans;

}
SymbolicTransformation::SymbolicTransformation(){
	
	//TODO: initailize controler
	controller = new LinearController(3.14/2, 0);


	//TODO: initailize symbolic axis

	//TODO: initailize axis
	//axis << -1.0, 0.0, 0.0;
	axis << 0, 1, 0;

	//Initialize val
	val = 0;
	//updateTime(0);
		
}

void SymbolicTransformation::updateParameters(SymbolicAssignment const& env){

	axis = symbAxis.evalVector3d(env); 
	//axis << -1.0, 0.0, 0.0;
	//std::cout << "updated axis" << axis.transpose() <<  std::endl;
	//system("pause"); 

	axis.normalize();
	updateTransform();
}

void SymbolicTransformation::updateTime(double t){

	val = controller->getVal(t);
	//std::cout << "the new val is " << val <<std::endl;
	updateTransform();
}




void SymbolicTransformation::updateTransform(){
	//TODO: switch case type: rotation translation

	if(isTrans){
		transformation = Eigen::Translation<double,3>(val*axis);
	}
	else{
		transformation =  Eigen::AngleAxisd(val,axis);
	}
	//cout << "Updated transform: " << endl;
	//cout << transformation.matrix() << endl;
}

FabDebugging::DebugInfo* SymbolicTransformation::getDebugInfo() {
	auto info = new FabDebugging::DebugInfo();
	info->setTypeName("SymbolicTransformation");
	info->setShortDescription("SymbolicTransformation");
	info->putStringProperty("symbAxis.x", symbAxis.x.toString());
	info->putStringProperty("symbAxis.y", symbAxis.y.toString());
	info->putStringProperty("symbAxis.z", symbAxis.z.toString());
	return info;
}


Articulation::Articulation(){

	//TODO: initialize center symbolic
	//symbCenter = _symbCenter; 

	//TODO: initialize center
	center.x() = -5.0;
	center.y() = -50.0;
	center.z() = -5.0;


	//TODO: create tranformations
	SymbolicTransformation* trans = new SymbolicTransformation();
	transformations.push_back(trans);
	

}

void Articulation::addTranformation(SymbolicTransformation* trans ){
	transformations.push_back(trans);
}

Articulation::Articulation(Point3S & _symbCenter){

	//TODO: initialize center symbolic
	symbCenter = _symbCenter; 

	//TODO: initialize center
//	center.x() = -5.0;
//	center.y() = -50.0;
//	center.z() = -5.0;


	//TODO: create tranformations
//	SymbolicTransformation* trans = new SymbolicTransformation();
//	transformations.push_back(trans);

}



void Articulation::updateParameters(SymbolicAssignment const& env){

	//update center
	//TODO: replace with symbolic
	center = symbCenter.evalVector3d(env); 
//	std::cout << "new center = " << center.transpose() << std::endl;
//	system("pause"); 
//	center.x() = -5.0;
//	center.y() = -50.0;
//	center.z() = -5.0;


	//update all transforms
	for(int i = 0; i < transformations.size(); i ++){
		transformations[i]->updateParameters(env);
	}

	updateTransform();
}


void Articulation::updateTime(double t){

	for(int i = 0; i < transformations.size(); i ++){
		transformations[i]->updateTime(t);
	}
	updateTransform();
}

void Articulation::transformMesh(TriMesh* mesh, bool hasnormals) {
	Eigen::MatrixXd mat = transformation.matrix();

	
	xform trans(mat(0,0), mat(1,0), mat(2,0), mat(3,0),
				mat(0,1), mat(1,1), mat(2,1), mat(3,1),
				mat(0,2), mat(1,2), mat(2,2), mat(3,2),
				mat(0,3), mat(1,3), mat(2,3), mat(3,3));

	bool debugTrans = true;
	if(debugTrans){
		point px = point(1.0,0,0);
		point py = point(0,1.0,0);
		point pz = point(0,0,1.0);
		point npx = trans*px;
		point npy = trans*py;
		point npz = trans*pz;

		//std::cout << "tranformed point x "  << npx[0] << ", " << npx[1] << ", " << npx[2] << std::endl;
		//std::cout << "tranformed point y "  << npy[0] << ", " << npy[1] << ", " << npy[2] << std::endl;
		//std::cout << "tranformed point z "  << npz[0] << ", " << npz[1] << ", " << npz[2] << std::endl;
	}
	//system("pause"); 
	apply_xform(mesh, trans, hasnormals);

}

void Articulation::transformGeo(Geometry* geo, bool hasnormals){
	
	//std::cout << transformation.matrix() << std::endl;
	this->transformMesh(geo->mesh, hasnormals);

}

void Articulation::transformVec(Eigen::Vector3d & vec){
	vec = transformation*vec;
}

void Articulation::transformPoints(vector<point> & pts){
	Eigen::MatrixXd mat = transformation.matrix();

	
	xform trans(mat(0,0), mat(1,0), mat(2,0), mat(3,0),
				mat(0,1), mat(1,1), mat(2,1), mat(3,1),
				mat(0,2), mat(1,2), mat(2,2), mat(3,2),
				mat(0,3), mat(1,3), mat(2,3), mat(3,3));

	
	for (auto pt_it = pts.begin(); pt_it != pts.end(); ++pt_it) {
		*pt_it = trans * (*pt_it);
	}

}

void Articulation::updateTransform(){

	//std::cout << "updating transform" << std::endl;
	//system("pause"); 
	Eigen::Affine3d uncentering, recentering;
	uncentering= Eigen::Translation<double,3>((-1)*center);
	recentering= Eigen::Translation<double,3>(center);
	transformation = Eigen::Affine3d::Identity();
	
	transformation = uncentering*transformation;

	if(transformations.size() > 1){
		LOG(ERROR) << "not just one transformation";
	}

	for(int i = 0; i < transformations.size(); i ++){
		transformation = transformations[i]->transformation *transformation;
	}
	transformation = recentering*transformation;
	

	//std::cout << "after recentering" << std::endl;
	//std::cout << transformation.matrix() << std::endl;
	//system("pause");

}

void Articulation::transformSelf(const Eigen::Matrix4d& transform) {
	center = (transform * Eigen::Vector4d(center[0], center[1], center[2], 1)).head(3);
	for (int i = 0; i < transformations.size(); i++) {
		Eigen::Vector3d axis = transformations[i]->axis;
		axis = (transform * Eigen::Vector4d(axis[0], axis[1], axis[2], 0)).head(3);
		transformations[i]->axis = axis;
	}
}

void Articulation::rotate(Eigen::Vector3d const& center, Eigen::Quaterniond const& rotation){


	//do the center
	
	// std::cout << "is going to rotate the articulation" << std::endl; 


	symbCenter = symbCenter.translate(-center);
	symbCenter = symbCenter.times(rotation.toRotationMatrix());
	symbCenter = symbCenter.translate(center);
	
	for (int i = 0; i < transformations.size(); i++) {
		Point3S & axis = transformations[i]->symbAxis;
		axis = axis.times(rotation.toRotationMatrix());
	}

	updateTransform();
}


void Articulation::translate(Eigen::Vector3d const& trans){
	//do the center	
	// std::cout << "is going to rotate the articulation" << std::endl; 
	symbCenter = symbCenter.translate(trans);	
	updateTransform();
}

FabDebugging::DebugInfo* Articulation::getDebugInfo() {
	auto info = new FabDebugging::DebugInfo();
	info->setTypeName("Articulation");
	info->setShortDescription("Articulation");
	const char* enumNames[] = { "FOLD", "TEETH", "TOUCH", "HINGE", "BALLJOINT", "PRISMATIC", "NONE" };
	info->putStringProperty("symbCenter.x", symbCenter.x.toString());
	info->putStringProperty("symbCenter.y", symbCenter.y.toString());
	info->putStringProperty("symbCenter.z", symbCenter.z.toString());
	for (int i = 0; i < transformations.size(); i++) {
		info->putAggregationProperty(concat() << "transformation[" << i << "]", transformations[i]);
	}
	return info;
}