#include "design.h" 
#include "component.h"
#include "connectorsGroup.h"
#include "featureGen.h"
#include <time.h>
#include "TriMesh_algo.h"
#include "params.h"
#include <Eigen/Dense>
#include <limits>
#include "mathVec.h"
#include "myoptimizer.h"
#include "geometry.h"
#include "dataTraits.h"
#include "functionality.h"
#include "tree.h"

using namespace std;
using namespace FabByExample;


Component::Component(Design* _parentDesign, int _id, std::string _name){
	parentDesign = _parentDesign;
	thisId = _id;
	name = _name;
	geometryInGlobalCoords = NULL;
	hasFunctionality = NULL;
	globalRot.LoadIdentity();
	globalTrans.LoadZero();
	//tempGeoGC = NULL;
	//tempScale = vector3f(1, 1, 1);
	//tempTrans.LoadZero();
	for(int ax = 0; ax< 3; ax ++)
		isVariableAxis[ax] = false;
	isSubstructure = false;

}

Component::~Component(){
	//std::cout << "deleting Component" << name << "... " ;
	if (geometry != NULL)
		delete geometry;
	if (geometryInGlobalCoords != NULL)
		delete geometryInGlobalCoords;
	//std::cout << " done." << std::endl; 
}

void Component::deleteGeos(){
	if (geometry != NULL){
		delete geometry;
		geometry = NULL;
	}
	if (geometryInGlobalCoords != NULL){
		delete geometryInGlobalCoords;
		geometryInGlobalCoords = NULL;
	}
}
void Component::addEdge(Component* _child, std::string relativeMatrixValue){
	Edge* newedge;
	newedge = new Edge(_child, relativeMatrixValue);
	edges.push_back(newedge);
}
void Component::addEdge(Component* child, const vector<double>& relativeMatrix){
	Edge* newEdge;
	newEdge = new Edge(child, relativeMatrix);
	edges.push_back(newEdge);
}
std::string Component::getName(){
	return name;
}
int Component::getId(){
	return thisId;
}




void Component::printComponent(){
	if(getIsLeaf()){
		std::cout << "LeafComponent" << std::endl;
		std::cout << "name = " << name << std::endl;
		std::cout << "id = " << thisId << std::endl;
	}
	else{
		std::cout << "BranchComponent" << std::endl;
		std::cout << "name = " << name << std::endl;
		std::cout << "id = " << thisId << std::endl;
		std::list<Edge*>::iterator it;
		std::cout << "Edges: " <<  std::endl;
		for ( it=edges.begin() ; it != edges.end(); ++it ){
				//std::cout << "= Edge = " << std::endl;
				(*it)->printEdge();
		}
	}
}


void Component::simplePrintComponent(int n){
	for(int i = 0; i< n; i++){
		std::cout << "    ";
	}
	if(n ==0){
			std::cout << "-> " << thisId << " - " << name << std::endl ;
	}
	else{
		std::cout << "-> " << thisId << " - " << name << " -  Parent: " << parent->getId() << std::endl ;
	}
	std::list<Edge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
		(*it)->printEdge();		
		//std::cout << "= Edge = " << std::endl;
		(*it)->getChild()->simplePrintComponent(n+1);
	}
}
//void Component::makeLeaf(std::string _geometryFile){
//	geometryFile = _geometryFile;
//	if(geometry)
//		delete geometry;
//	geometry = new Geometry(geometryFile);
//	isLeaf = true;
//}


Geometry* Component::getGeometry(){

	if(getIsLeaf()){
		//std::cout << "In Leaf Component get Geometry." << name << std::endl;  
		return geometry;
	}
	//std::cout << "In Branch Component get Geometry : "<< name << std::endl; //system("PAUSE"); 
	if (geometry == NULL){
		geometry = new Geometry;
		std::list<Edge*>::iterator it;
		for ( it=edges.begin() ; it != edges.end(); ++it ){
			Geometry *m = new Geometry;
			
			TriMesh *mp  = (*it)->getChild()->getGeometry()->getMesh();
			m->addMesh(mp);
			vector<double> rm =  (*it)->getRelativeMatrix();
			xform xf(1, 0, 0, 0,
					 0, 1, 0, 0,
					 0, 0, 1, 0,
					 0, 0, 0, 1);
			xform trans(1, 0, 0, 0,
					 0, 1, 0, 0,
					 0, 0, 1, 0,
					 rm[9], rm[10], rm[11] , 1);
			xform rot(rm[0], rm[1], rm[2], 0,
					 rm[3], rm[4], rm[5], 0,
					 rm[6], rm[7], rm[8], 0,
					 0, 0, 0, 1);


			
			apply_xform(m->mesh, rot);			
			apply_xform(m->mesh, trans);
		    geometry->addMesh(m->mesh);
			delete  m;
			//geometry->write("data//bbb.off"); system("PAUSE"); 
		}
	}
	return geometry;
}

Geometry* Component::getGeometryInGlobalCoord(){
	//std::cout << "here C 1" <<std::endl;


	if(geometryInGlobalCoords == NULL){

		//std::cout << "computing geometry for " << name << "..."  << std::endl;
		//system("pause");
		geometryInGlobalCoords = new Geometry;
		//std::cout << "here C 2" <<std::endl;	
		Geometry *mp  = getGeometry();
		//std::cout << "here C 3" <<std::endl;
		geometryInGlobalCoords->addMesh(mp->getMesh());


		xform trans(1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					globalTrans.vertex[0], globalTrans.vertex[1], globalTrans.vertex[2] , 1);
		xform rot(globalRot.matrix[0], globalRot.matrix[1], globalRot.matrix[2], 0,
					globalRot.matrix[3], globalRot.matrix[4], globalRot.matrix[5], 0,
					globalRot.matrix[6], globalRot.matrix[7], globalRot.matrix[8], 0,
					0, 0, 0, 1);
			
		apply_xform(geometryInGlobalCoords->mesh, rot);			
		apply_xform(geometryInGlobalCoords->mesh, trans);

		//geometryInGlobalCoords->mesh->need_normals();
		geometryInGlobalCoords->mesh->need_tstrips();
		geometryInGlobalCoords->mesh->need_bsphere();
		geometryInGlobalCoords->mesh->need_bbox();
		//std::cout << "here C 4" <<std::endl;
	}
	//std::cout << "here C 5" <<std::endl;
	return geometryInGlobalCoords;

}

std::string Component::getGlobalTranf(){
	std::stringstream tranfString;
	tranfString << globalRot.matrix[0] << " " << globalRot.matrix[1] << " " << globalRot.matrix[2] << " " ;
	tranfString <<	globalRot.matrix[3]<< " " <<globalRot.matrix[4] << " " <<globalRot.matrix[5] << " " ;
	tranfString << globalRot.matrix[6]<< " " <<globalRot.matrix[7]<< " " <<globalRot.matrix[8] << " " ;
	tranfString << globalTrans.vertex[0]<< " " <<globalTrans.vertex[1]<< " " <<globalTrans.vertex[2];
	return tranfString.str();
}



Geometry* Component::getGeometryMinFunc(double alpha, Functionality*  func){

	//std::cout << "getting geometry = " << name << std::endl;
	Geometry *mp  = getGeometryInGlobalCoord();
	if (gethasFunctionality(func) == true){
		Geometry* geometryMinRot = new Geometry;
		geometryMinRot->addMesh(mp->getMesh());
		xform transToCenter = getTranfToCenterFromFucntionality(func);
		apply_xform(geometryMinRot->mesh, transToCenter);
		xform transFunc = getFucntionalityTransformation(alpha, func);		
		apply_xform(geometryMinRot->mesh, transFunc);
		xform transFromCenter = getTranfFromCenterFromFucntionality(func);
		apply_xform(geometryMinRot->mesh, transFromCenter);
		//geometryInGlobalCoords->mesh->need_normals();
		geometryMinRot->mesh->need_tstrips();
		geometryMinRot->mesh->need_bsphere();
		return geometryMinRot;
	}
	return mp;

}


Geometry* Component::getGeometryMinFuncs(std::vector<double> alphas, std::vector<Functionality*> funcs){


	//std::cout << "getting geometry = " << name << std::endl;
	Geometry *mp  = getGeometryInGlobalCoord();
	for (int id = 0; id< alphas.size(); id++)
		if (gethasFunctionality(funcs[id]) == true){
			return(getGeometryMinFunc(alphas[id], funcs[id]));
	}
	return mp;

}








vector3f Component::getCenterOfAxisInGlobalCoord(){

	//TriMesh *m = getGeometryInGlobalCoord()->getMesh();
	//m->need_bbox();
	//vector3f center(m->bbox.min[0], m->bbox.min[1], m->bbox.min[2]);
	
//	center = globalRot*center;			
//	center = center + globalTrans;

	
	vector3f center(0,0,0);
	center = globalRot*center;			
	center = center + globalTrans;

	//std::cout << "center 1 = [" << center.vertex[0]  << ", " << center.vertex[1] << ", " << center.vertex[2] << std::endl;

	//	std::cout << "center 2 = [" << center.vertex[0]  << ", " << center.vertex[1] << ", " << center.vertex[2] << std::endl;

	//	system("pause");

	return center;

}

void Component::setGlobalTrans(matrix9f rot, vector3f trans){
	globalRot = rot;
	globalTrans = trans;

}

int Component::isPart(){
	if (name[0] == 'P'){
		//std::cout << "-----" << name << " --- is part" << std::endl;
		return 1;
	}
	//std::cout << "-----" << name << " --- is connectoe" << std::endl;
	return 0;

}

int Component::isConnector(){
	if (name[0] == 'C'){
		//std::cout << "-----" << name << " --- is connector" << std::endl;
		return 1;
	}
	//std::cout << "-----" << name << " --- is part" << std::endl;
	return 0;

}


int Component::isFunc(){
	if (name.compare( 0, 5,"FUNC_") == 0){
		//std::cout << "-----" << name << " --- is func" << std::endl;
		return 1;
	}
	//std::cout << "-----" << name << " --- is not func" << std::endl;
	return 0;

}


void Component::addFunctionality(Functionality*  func){
	hasFunctionality = func;
}

Tree* Component::getTree(){return parentDesign->getTree();}


int Component::removeChild(Edge* edgeChild){
	std::list<Edge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
		if((*it)->getChild()->getId() == edgeChild->getChild()->getId()){
			edges.erase(it);
			//system("pause");
			return 0;
		}
	}
	return 1;
}

Edge* Component::getEdgeTo(Component* comp){
	std::list<Edge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
		if((*it)->getChild()->getId() == comp->getId()){
			return (*it);
		}
	}
	return NULL;
}



int Component::removeChildFromComp(Component* comp){
	std::list<Edge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
		if((*it)->getChild()->getId() == comp->getId()){
			edges.erase(it);
			//system("pause");
			return 0;
		}
	}
	return 1;
}

void Component::addChild(Edge* edgeChild){
	edges.push_back(edgeChild);
	edgeChild->getChild()->setParrent(this);
}

std::vector<Functionality*> Component::getFunctionalitiesThatAffectSubassemply(){
	return getTree()->getFunctionalitiesForComp(this);
}
	
xform Component::getTranfToCenterFromFucntionality(Functionality*  func){
	if (gethasFunctionality(func)){
		//Functionality * func = tree->getFunctionalities()[id];
		xform trans(1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					-func->getCenter().vertex[0], -func->getCenter().vertex[1], -func->getCenter().vertex[2] , 1);
		return trans;
	}
		xform notrans(1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1);
		return notrans;

}



xform Component::getFucntionalityTransformation(double alpha, Functionality*  func){
	if (gethasFunctionality(func)){
			//Functionality * func = tree->getFunctionalities()[id];
			return func->getMinXfrom(alpha);
		}
		xform notrans(1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1);
		return notrans;
}		
xform Component::getTranfFromCenterFromFucntionality(Functionality*  func){
	if (gethasFunctionality(func)){
		//Functionality * func = tree->getFunctionalities()[id];
		xform trans(1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					func->getCenter().vertex[0], func->getCenter().vertex[1], func->getCenter().vertex[2] , 1);
		return trans;
	}
		xform notrans(1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1);
		return notrans;

}


matrix9f Component::getRotFromFucntionality(double alpha, Functionality*  func){
	if (gethasFunctionality(func)){
		//Functionality * func = tree->getFunctionalities()[id];
		return func->getRot(alpha);
	}

	matrix9f rot;
	rot.LoadIdentity();
	return rot;

}


vector3f Component::getTransFromFucntionality(double alpha, Functionality*  func){
	if (gethasFunctionality(func)){
		//Functionality * func = tree->getFunctionalities()[id];
		return func->getTrans(alpha);
	}

	vector3f trans;
	trans.LoadZero();
	return trans;

}

vector3f Component::getAxisFromFucntionality(Functionality*  func){

	vector3f vecIndex;
	vecIndex.LoadZero();

	if (gethasFunctionality(func) == false){
		return vecIndex;
	} 


	if (gethasFunctionality(func)){
		//Functionality * func = tree->getFunctionalities()[id];
		if (func->getAxis() == Functionality::FuncAxis::X) {vecIndex.vertex[0]=1;}
		if (func->getAxis() == Functionality::FuncAxis::Y) {vecIndex.vertex[1]=1;}
		if (func->getAxis() == Functionality::FuncAxis::Z) {vecIndex.vertex[2]=1;}
	}
	return vecIndex;
	

}



vector3f Component::getPointClossestToFuncCenter(Functionality*  func){

	vector3f transOrg, vec, vecIndex;
	vec.LoadZero();
	vecIndex.Set(1,1,1);

	if (gethasFunctionality(func) == false){
		return vec;
	} 


	if (gethasFunctionality(func)){
		//Functionality * func = tree->getFunctionalities()[id];
		TriMesh *m = getGeometryInGlobalCoord()->getMesh();
		double dist, newdist;
		if (func->getAxis() == Functionality::FuncAxis::X) {vecIndex.vertex[0]=0;}
		if (func->getAxis() == Functionality::FuncAxis::Y) {vecIndex.vertex[1]=0;}
		if (func->getAxis() == Functionality::FuncAxis::Z) {vecIndex.vertex[2]=0;}
		dist = 100000000;
		for (int bx= 0; bx <2; bx++){
			for (int by= 0; by <2; by++){
				for (int bz= 0; bz <2; bz++){
					transOrg.Set(m->bbox.min[0] + bx*(m->bbox.max[0] - m->bbox.min[0]), m->bbox.min[1] + by*(m->bbox.max[1] - m->bbox.min[1]), m->bbox.min[2] + bz*(m->bbox.max[2] - m->bbox.min[2]));
					newdist = func->getCenter().dist(transOrg, vecIndex);
					//std::cout << "func->getCenter() = [" << func->getCenter().vertex[0] << ", " << func->getCenter().vertex[1] << ", " << func->getCenter().vertex[2] << std::endl;
					//std::cout << "transOrg = [" << transOrg.vertex[0] << ", " << transOrg.vertex[1] << ", " << transOrg.vertex[2] << std::endl;
					
					//std::cout << "dist = " << newdist << std::endl;
					if (newdist < dist){
						vec.Set(bx,by,bz);
						dist = newdist;
						//std::cout << ".................................got smallest - vec = [" << vec.vertex[0] << ", " << vec.vertex[1] << ", " << vec.vertex[2] <<"] dist = " << dist << std::endl;
					}
				}
			}
		}

	}
	return vec;
	

}

vector3f Component::getTransOfMinsFromFucntionality(double alpha, Functionality*  func){

	vector3f trans,  transOrgF, vec;
	trans.LoadZero();

	if (gethasFunctionality(func) == false){
		return trans;
	} 


	if (gethasFunctionality(func)){
		//Functionality * func = tree->getFunctionalities()[id];
		trans.Set(1,1,1);
		trans = func->getRot(alpha)*trans;
		//std::cout << "alpha = " << alpha << "id = " << id << std::endl;
		//std::cout << "TRANS = " << trans.vertex[0]<< " " <<  trans.vertex[1] << " " <<  trans.vertex[2] << std::endl;
		//system("pause");

		TriMesh *m = getGeometryInGlobalCoord()->getMesh();
		vector3f vec = getPointClossestToFuncCenter( func);
		transOrgF.Set(m->bbox.min[0] + vec.vertex[0]*(m->bbox.max[0] - m->bbox.min[0]), m->bbox.min[1] + vec.vertex[1]*(m->bbox.max[1] - m->bbox.min[1]), m->bbox.min[2] + vec.vertex[2]*(m->bbox.max[2] - m->bbox.min[2]));
	
		trans = transOrgF - func->getCenter();
		trans = func->getRot(alpha)*trans;
		trans = trans + func->getTrans(alpha);
		trans = trans + func->getCenter(); 
		trans = trans - transOrgF;
	}

	//if (gethasFunctionality(id)){
	//	Functionality * func = tree->getFunctionalities()[id];
	//	trans = func->getCenter() - func->getRot(alpha)*func->getCenter();
	//	trans = trans + func->getTrans(alpha);
	//}


	//Geometry* g2 = new Geometry;
	//g2->addMesh(m);
	//xform transToCenter = getTranfToCenterFromFucntionality(id);
	//apply_xform(g2->mesh, transToCenter);
	//xform transFunc = getFucntionalityTransformation(alpha, id);		
	//apply_xform(g2->mesh, transFunc);
	//xform transFromCenter = getTranfFromCenterFromFucntionality(id);
	//apply_xform(g2->mesh, transFromCenter);
	//g2->mesh->need_bbox();
	//trans.Set(g2->mesh->bbox.min[0], g2->mesh->bbox.min[1], g2->mesh->bbox.min[2]);


	//std::cout << "TRANS = " << trans.vertex[0]<< " " <<  trans.vertex[1] << " " <<  trans.vertex[2] << std::endl;
	//std::cout << "ORGTR = " << transOrg.vertex[0]<< " " <<  transOrg.vertex[1] << " " <<  transOrg.vertex[2] << std::endl;
	//std::cout << "DIFFS = " << trans.vertex[0]<< " " <<  trans.vertex[1] << " " <<  trans.vertex[2] << std::endl;
	

	return trans;

}


bool Component::hasSons(){
	if(edges.empty() == true){
		return false;
	}
	return true;
}

string Component::getFullName(){
	string fullName = getTree()->getName();
	fullName.append("_");
	std::stringstream ss; ss << thisId;
	fullName.append(ss.str());

	return(fullName);

}


//string Component::getFuncRange(std::string funcfileName){
//
//	string funcRange;
//	if(!hasFunctionality){
//		return "0 0";
//	}
//
//	int nfunc = 0;
//	string minrange("0");
//	string maxrange("0");
//	if(func->getmin() !=  0){
//		minrange = "1";
//		addFuncInfo(funcfileName, 0);
//	}
//	if(func->getmax() !=  0){
//		maxrange = "1";
//		addFuncInfo(funcfileName, 1);
//	}
//
//	//stringstream st;
// //   st << nfunc;
// //   string strNfunc = st.str();
//
//	funcRange = minrange;
//	funcRange.append(" ");
//	funcRange.append(maxrange);
//
//	return(funcRange);
//
//	return "";
//}

std::string Component::getFuncInfo(Functionality*  func, double alpha){
	
	matrix9f mrot = getRotFromFucntionality(alpha, func);
	vector3f mtrans = getTransFromFucntionality(alpha, func);
	vector3f cpoint = getPointClossestToFuncCenter(func);
	//vector3f mtrans = getTransOfMinsFromFucntionality(alpha, id);
	vector3f faxis = getAxisFromFucntionality(func);


	for(int i= 0;  i<9; i++){
		//mrot.matrix[i] = abs(mrot.matrix[i]);
		if (abs(mrot.matrix[i])  < 0.001)
			 mrot.matrix[i] = 0;
	}

	stringstream st;

	st << mtrans.vertex[0]<< " " <<  mtrans.vertex[1] << " " <<  mtrans.vertex[2] << " ";
	st << mrot.matrix[0]  << " " << mrot.matrix[1] << " " << mrot.matrix[2] << " " << mrot.matrix[3] << " " << mrot.matrix[4] << " " << mrot.matrix[5] << " " << mrot.matrix[6] << " " << mrot.matrix[7] << " " << mrot.matrix[8];
	st << " " << cpoint.vertex[0]<< " " <<  cpoint.vertex[1] << " " <<  cpoint.vertex[2];
	st << " " << faxis.vertex[0]<< " " <<  faxis.vertex[1] << " " <<  faxis.vertex[2];



	string funcTrans = st.str();

	return funcTrans;

	
}

//bool Component::checkCoplanarity(Component* comp2, int i){
//	bool iscop =  false;
//	TriMesh *m1  = getGeometryInGlobalCoord()->getMesh();
//	//m1->
//	TriMesh *m2  = comp2->getGeometryInGlobalCoord()->getMesh();
//	double m1min = m1->bbox.min[i];
//	double m1max = m1->bbox.max[i];
//	double m2min = m2->bbox.min[i];
//	double m2max = m2->bbox.max[i];
//
//	if (abs(m1min - m2min) < 10){
//		iscop = true;
//	}
//	if (abs(m1min - m2max) < 10){
//		iscop = true;
//	}
//	if (abs(m1max - m2min) < 10){
//		iscop = true;
//	}
//	if (abs(m1max - m2max) < 10){
//		iscop = true;
//	}
//	if (iscop){
//		//std::cout <<  "Found coplanarity: " << getId() << " - " << comp2->getId() << std::endl;
//	}
//
//	return iscop;
//
//}

bool Component::gethasFunctionality(Functionality* func){
	if (func == NULL)
		return false;	
	if (isRoot())
		return false;	
	if (hasFunctionality != NULL){
		if (hasFunctionality->getId() == func->getId())
			return true;
	}
	return (getParent()->gethasFunctionality(func));
}


bool Component::checkIfFunctionalityIsChild(Functionality* func){
	if (func == NULL)
		return false;	
	if (hasFunctionality != NULL){
		if (hasFunctionality->getId() == func->getId())
			return false;	
	}
	return getaboveFunctionality( func);
}


bool Component::isFatherOfComponent(Component* comp){
	if(isRoot()){
		return true;
	}

	bool isFather = false;
	Component* parent = comp;

	while (!parent->isRoot() ){
		if ((parent->getId() == thisId)){
			isFather = true;
		}	
		parent = parent->getParent();
	}

	return isFather;


}


bool Component::getaboveFunctionality(Functionality* func){
	if (func == NULL)
		return false;	
	if (hasFunctionality != NULL){
		if (hasFunctionality->getId() == func->getId())
			return true;	
	}	
	std::list<Edge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
		if((*it)->getChild()->getaboveFunctionality(func))
			return true;
	}
	return false;
}




bool Component::isRoot(){ 
	if (getTree()->getRoot()->getId() == thisId)
		return true;
	return false;
}









point Component::getGlobalMin(){

	std::list<Component*> nodes = getLeafs();
	std::list<Component*>::iterator it, it2;
	point minPoint(99999, 99999, 99999);

	for ( it=nodes.begin() ; it != nodes.end(); it++ ){
		Component * comp = (*it);
		if(!comp->isConnector()){
			comp->getGeometryInGlobalCoord()->getMesh()->need_bbox();
			point nminPoint = comp->getGeometryInGlobalCoord()->getMesh()->bbox.min;
	
			for(int ax = 0; ax < 3; ax++){
				if(nminPoint[ax] < minPoint[ax])
					minPoint[ax] = nminPoint[ax];
			}		
		}
	}
	return minPoint;
}






std::vector<TriMesh*> Component::getOriginalMeshes(Component::ComponentSet cs){
	
	std::vector<TriMesh*>meshes;

	std::list<Component*> nodes = getLeafs();
	std::list<Component*>::iterator it;



	for ( it=nodes.begin() ; it != nodes.end(); it++ ){
		Component * comp = (*it);
		if((comp->isConnector() && (cs == ComponentSet::PARTS)) ||
			(!comp->isConnector() && (cs == ComponentSet::CONNS))){
		}else{
		//std::cout << "comp:: = " << comp->getId() << " - " << comp->getName() << std::endl;

			//system("pause");
			Geometry *m = comp->getGeometryInGlobalCoord();
			//m->getMesh()->need_tstrips();
	
			meshes.push_back(m->getMesh());
		}

	}

	return meshes;
}








std::list<Component*>  Component::getLeafs(){

	std::list<Component*> leafComponents;

	if(getIsLeaf() == true){
		leafComponents.push_back(this);
		return leafComponents;
	}

	list<Edge*> edges = getEdges();
	list<Edge*>::iterator it =  edges.begin();
	for ( it=edges.begin() ; it != edges.end(); ++it ){ 
		std::list<Component*> edgeleafs = (*it)->getChild()->getLeafs();
		list<Component*>::iterator it2 =  edgeleafs.begin();
		for ( it2 =edgeleafs.begin() ; it2 != edgeleafs.end(); ++it2 ){ 
				leafComponents.push_back(*it2);
		}
	}
	return leafComponents;
}



point Component::sizeOfOrgTemplate(){

	
	TriMesh::BBox box = getBBoxGlobalCoords();
	
	return box.size();

	
}




void Component::getMaterialVector(Eigen::VectorXd & materialVector){

	materialVector = Eigen::VectorXd::Zero(N_MATERIALS);
	addMaterialToVector(materialVector);
	double totalVol = materialVector.sum();
	materialVector = materialVector/totalVol;

}

void Component::getFunctionalityVector(Eigen::VectorXd & funcVec){

	funcVec = Eigen::VectorXd::Zero(N_FUNC_TYPES);
	std::vector<Functionality*> funcsforComp = getTree()->getFunctionalitiesForComp(this);
	//std::cout <<  "the size of the func is = " << funcsforComp.size() << std::endl;
	for (unsigned int i = 0; i < funcsforComp.size(); i++ ){
		if(funcsforComp[i]->getType() == Functionality::FuncType::ROT){
			funcVec[0]+= 1;
		}else if(funcsforComp[i]->getType() == Functionality::FuncType::TRANS){
			funcVec[1]+= 1;
		}
	
	}

}


TriMesh::BBox Component::getBBoxGlobalCoords(){
	TriMesh::BBox box;
	//std::cout << "here D11 " <<std::endl;
	std::list<Component*> nodes =  getLeafs();
	std::list<Component*>::iterator it;
	for ( it=nodes.begin() ; it != nodes.end(); it++ ){
		//std::cout << "leaf node = " << (*it)->getName() <<std::endl;
		//if((*it)->isPart()){
			//std::cout << "leaf node = " << (*it)->getName() <<std::endl;
			TriMesh* auxmesh = (*it)->getGeometryInGlobalCoord()->getMesh();
			auxmesh->need_bbox();
			box = box + auxmesh->bbox;
		//}
	}

	return box;


}


	std::string Component::getGenName(){
		std::string orgGeofileName(name);
		//cout << "filename = " << fileName << endl;
		size_t found= orgGeofileName.find(".", 2);
		if (found > 100)
			found = orgGeofileName.find("-", 1);
		orgGeofileName.resize(found);
		orgGeofileName.append(".STL"); 
		return orgGeofileName;
	}






void Component::removeConnectorGroup(){

	std::list<Edge*> copyEdges = this->getEdges();
	std::list<Edge*>::iterator it;
	for ( it=copyEdges.begin() ; it != copyEdges.end(); ++it ){
		(*it)->getChild()->removeConnectorGroup();
	}
	copyEdges.clear();
	if(!this->getIsLeaf()){

		bool allEdgesAreCoon = true;
		for ( it=edges.begin() ; it != edges.end(); ++it ){
			Component * comp = (*it)->getChild();
			if(!comp->getIsLeaf()){
				allEdgesAreCoon = false;
			}else{
				if(!comp->isConnector()){
					allEdgesAreCoon = false;
				}
			}
		}

		if(allEdgesAreCoon){
			std::cout <<" will remove = " << this->getName() << std::endl;
			this->removeMiddleNode();
		}
	}
}

void Component::removeMiddleNode(){

	Component * thisParent = this->getParent();

	std::list<Edge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
		Component * comp = (*it)->getChild();
		thisParent->addEdge(comp, (*it)->getRelativeMatrixValue());
		comp->setParrent(thisParent);
	}


	Edge* edge = thisParent->getEdgeTo(this);
	thisParent->removeChild(edge);

}

