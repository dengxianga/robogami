#include "KinChain.h"
//#include "geometry.h"
#include "../FBE_Temp/geometry.h"
#include "articulation.h"
#include "template.h"
#include "templateElement.h"
#include "NewPatch.h"
#include "CenterOfMass.h"
#include "XForm.h"
#include "TriMesh_algo.h"
#include "GeometricOperations.h"
#include "controller.h"
#include "element_motion.h"
#include <ctime>

#define INVERTTREE     false
#define DEBUG_OPTION   false
// for updateGait function
#define debugStability false
#define debugSlip      false
#define debugAnim      false
#define debugAnim2     false
#define debugAnimInterp false
#define debugSteadyState false
#define DEBUG_PATH	   false
#define USE_ONLY_CONTACT_POINTS false
using namespace FabByExample;



KinRigidTransform::KinRigidTransform(){
	rot = Eigen::Matrix3d ::Identity();
	trans = Eigen::Vector3d::Zero();
	clear();
}

void KinRigidTransform::apply( Geometry * geo, bool updatenormals){
	this->apply(geo->mesh, updatenormals);
}

void KinRigidTransform::apply(TriMesh * mesh, bool updatenormals){
	xform trans(rot(0,0), rot(1,0), rot(2,0), 0,
			rot(0,1), rot(1,1), rot(2,1), 0,
			rot(0,2), rot(1,2), rot(2,2), 0,
			this->trans(0), this->trans(1), this->trans(2), 1);
		
	apply_xform(mesh, trans, updatenormals);		
}

void KinRigidTransform::transform(std::vector<Eigen::Vector3d> & vec){

	//std::cout << rot << std::endl<< std::endl<<trans<< std::endl<< std::endl;

	for (auto it = vec.begin(); it != vec.end(); ++it) {
		//std::cout << rot*(*it)<< std::endl<< std::endl;
		//std::cout << rot*(*it)+trans<< std::endl<< std::endl;

		(*it) = rot*(*it)+ trans;
	}
}

void KinRigidTransform::transform(Eigen::Vector3d & vec){

	vec = rot*vec + trans;
}

void KinRigidTransform::transformPoint(point & p){
	Eigen::Vector3d vec(p[0], p[1], p[2]);
	vec = rot*vec + trans;
	p[0] = vec.x();
	p[1] = vec.y();
	p[2] = vec.z();
}


void KinRigidTransform::clear(){
	normal = Eigen::Vector3d::Zero();
	normal[1] = 1;

	trans =  Eigen::Vector3d::Zero();

	rot = Eigen::Matrix3d::Identity();

}

void KinRigidTransform::combine(KinRigidTransform & newTransf){

//	std::cout << "before rot = \n" << rot << std::endl;
//	std::cout << "before trand " << trans.transpose() << std::endl;
	rot = newTransf.rot*rot;
	trans = newTransf.rot*trans + newTransf.trans;
	
//	std::cout << "after rot = \n" << rot << std::endl;
//	std::cout << "after trans " << trans.transpose() << std::endl;
//	system("pause");
}


void KinRigidTransform::display(){
	std::cout << "rot = \n" << rot<< std::endl;
	std::cout << "trans " << trans.transpose() << std::endl;


}


//--------------------------------------------------------------------------------


void KinNode::addChild(KinNode* child){
	children.push_back(child);
	child->parent = this;
}

void KinNode::replaceChild(KinNode* child_old, KinNode* child_new){

	for(int i = 0; i < children.size(); i++){
		if(children[i] == child_old){
			children[i] = child_new;
		}
	}
}


void KinNode::removeChild(KinNode * child){
	for(int i = 0; i < children.size(); i++){
		if(children[i] == child){
			children.erase(children.begin() + i);
		}
	}
}

std::vector<KinNode*> KinNode::getNeighbors() {
	std::vector<KinNode*> neighbors = children;
	neighbors.push_back(parent);
	return neighbors;
}


//--------------------------------------------------------------------------------
KinNode_Part::KinNode_Part(): KinNode(){
}

KinNode_Part::KinNode_Part(TemplateElement* el): KinNode(){
	elements.push_back(el);
}

bool KinNode_Part::checkContainsElement(int elID){
	for(int i = 0; i < elements.size() ; i++){
		if(elements[i]->getID() == elID){
			return true;
		}
	}
	return false;
}

void KinNode_Part::add(KinNode_Part* part){
	// add the elemensts
	for(int i = 0; i < part->elements.size(); i++){
		elements.push_back(part->elements[i]);
	}

	// add the children
	for(int i = 0; i < part->children.size(); i++){
		children.push_back(part->children[i]);
		part->children[i]->parent = this;
	}
	
	//make the parent of node 2 be the parent of node1
	if(part->parent != nullptr){
		if(parent != nullptr){
			LOG(ERROR) << "multiple parents on combined nodes";
		}else{
			part->parent->replaceChild(this, part);
		}
	}




}

void KinNode_Part::display(int N, bool doTree){
	std::stringstream dispInit;
	for(int i = 0; i<= N; i++){
		dispInit << "--"; 
	}
	VLOG(3) << dispInit.str() << "> Part: ";
	for(int i = 0; i < elements.size(); i++){
		VLOG(3) << dispInit.str() << ">    Element: " << elements[i]->getName();	
	}
	VLOG(3) << dispInit.str() << ">  Number of contact points  : " <<contactpoints.size(); 	

	if(doTree){
		for(int i = 0; i < children.size(); i++){
			children[i]->display(N+1, true);
		}
	}
}

void  KinNode_Part::addGeometry(Geometry* geo, bool withnormals){

	for(int i = 0; i < children.size(); i++){
	    children[i]->addGeometry(geo, withnormals);
	}

	for(int i = 0; i < elements.size(); i++){
		Geometry * partgeo = elements[i]->getGeometry(withnormals);
		geo->addMesh(partgeo->mesh, withnormals);
	}

}

void KinNode_Part::transformToRootFrame(vector<point>& pts) {
	if (!hasNoParent()) {
		parent->transformToRootFrame(pts);
	}
}

void KinNode_Part::transformToRootFrame(Geometry* geo) {
	if (!hasNoParent()) {
		parent->transformToRootFrame(geo);
	}
}

void KinNode_Part::transformToRootFrame(TriMesh* mesh) {
	if (!hasNoParent()) {
		parent->transformToRootFrame(mesh);
	}
}

Geometry* KinNode_Part::getCurrentGeoRelativeToRoot() {
	Geometry* geo = new Geometry();

	for(int i = 0; i < elements.size(); i++){
		Geometry * partgeo = elements[i]->getGeometry();
		geo->addMesh(partgeo->mesh);
	}

	this->transformToRootFrame(geo);

	/*
	KinNode* currentNode = this;
	while (!currentNode->hasNoParent()) {
		currentNode = currentNode->parent;

		if (currentNode->getType() == KinNode::JOINT) {
			((KinNode_Joint*)currentNode)->getArticulation()->transformGeo(geo);
		}
	}
	*/

	return geo;
}


void  KinNode_Part::addCenterOfMass(CenterOfMass & centerOfMass){

	for(int i = 0; i < children.size(); i++){
	    children[i]->addCenterOfMass(centerOfMass);
	}

	centerOfMass.add(getCenter());

}

CenterOfMass KinNode_Part::getCenter() {

	CenterOfMass centerOfMass;

	for(int i = 0; i < elements.size(); i++){
		CenterOfMass newCenter = elements[i]->computeCenterOfMass();
		//Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\sphere2.stl"));
		//std::stringstream filename;
		//filename << "..\\..\\data\\test\\eleCenter_" << elements[i]->getName() << ".stl";
		//sphere->applyTrans(point(newCenter.center.x(), newCenter.center.y(), newCenter.center.z()));
		//sphere->write(filename.str());


		centerOfMass.add(newCenter);
	}

	return centerOfMass;
}

std::vector<Eigen::Vector3d> KinNode_Part::getAllAttachmentPoints() {
	std::vector<Eigen::Vector3d> pointsToReturn;

	for (auto jointit = this->children.begin(); jointit != this->children.end(); ++jointit) {
		pointsToReturn.push_back(((KinNode_Joint*)(*jointit))->getCenter().center);
	}

	return pointsToReturn;
}

void KinNode_Part::collectControlThetas(std::vector<double>& allthetas, std::vector<bool>& iscontact, std::vector<bool>& ismoving, double t) {
	for (auto jointit = this->children.begin(); jointit != this->children.end(); ++jointit) {
		(*jointit)->collectControlThetas(allthetas, iscontact, ismoving, t);
	}
}

void KinNode_Part::broadcastControlThetas(std::list<double>& allthetas, double t) {
	for (auto jointit = this->children.begin(); jointit != this->children.end(); ++jointit) {
		(*jointit)->broadcastControlThetas(allthetas, t);
	}
}


void KinNode_Part::getAllTemplatesFromUnrestrictedNodes(Template* restrictedTemp, std::vector<Template*> templates){
	bool isRestrict = false;

	for each (auto p in this->elements){
		if (p->isDescendentOf(restrictedTemp)){
			isRestrict = true;
		}
	}

	if(! isRestrict){
		for each (auto p in this->elements){
			templates.push_back(p);
		}
	}


}


//----------------------------------------------------------------------------------

KinNode_Joint::KinNode_Joint(): KinNode(){
	isDriving = false;
}

KinNode_Joint::KinNode_Joint(Articulation* _articulation): KinNode(){
	articulation = _articulation;
	isDriving = false;
}

void KinNode_Joint::display(int N, bool doTree){
	std::stringstream dispInit;
	for(int i = 0; i<= N; i++){
		dispInit << "--"; 
	}
	VLOG(3) << dispInit.str() << "> " <<  "Joint  ";

	if(doTree){
		for(int i = 0; i < children.size(); i++){
			children[i]->display(N+1, true);
		}
	}else{
		VLOG(3) << dispInit.str() << "> " <<  "This joint connects  " ;
		this->parent->display(2, false);
		for(int i = 0; i < children.size(); i++){
			children[i]->display(2, false);
		}
	}


}

void KinNode_Joint::reset() {
	isDriving = false;
	resetControllers();
}

void KinNode_Joint::resetControllers() {
	std::vector<SymbolicTransformation*> atrans = getArticulation()->transformations;
	for (std::vector<SymbolicTransformation*>::iterator transit = atrans.begin(); transit != atrans.end(); ++transit) {
		if ((*transit)->controller->isSymbolic() ) {
			((SymbolicController*)((*transit)->controller))->clearLinearController();
		}
	}
}


void  KinNode_Joint::addGeometry(Geometry * geo, bool withnormals){


	Geometry * thisGeo = new Geometry();
	for(int i = 0; i < children.size(); i++){
	    children[i]->addGeometry(thisGeo, withnormals);
	}

	articulation->transformGeo(thisGeo, withnormals);

	geo->add(thisGeo, withnormals);
	delete  thisGeo;
	//geo->write("..\\..\\data\\test\\tranformgeo.stl");
	//system("pause");

}

void KinNode_Joint::transformToRootFrame(vector<point>& pts) {
	getArticulation()->transformPoints(pts);

	if (!hasNoParent()) {
		parent->transformToRootFrame(pts);
	}
}

void KinNode_Joint::transformToRootFrame(Geometry* geo) {
	getArticulation()->transformGeo(geo);

	if (!hasNoParent()) {
		parent->transformToRootFrame(geo);
	}
}

void KinNode_Joint::transformToRootFrame(TriMesh* mesh) {
	getArticulation()->transformMesh(mesh);

	if (!hasNoParent()) {
		parent->transformToRootFrame(mesh);
	}
}

void  KinNode_Joint::addCenterOfMass(CenterOfMass & centerOfMass){

	for(int i = 0; i < children.size(); i++){
	    children[i]->addCenterOfMass(centerOfMass);
	}

	//articulation->transformVec(centerOfMass.center);
}

CenterOfMass KinNode_Joint::getCenter() {
	CenterOfMass centerOfMass;

	centerOfMass.center = this->getArticulation()->getCenter();

	return centerOfMass;
}

std::vector<Eigen::Vector3d> KinNode_Joint::getAllAttachmentPoints() {
	std::vector<Eigen::Vector3d> pointToReturn;

	pointToReturn.push_back(getCenter().center);

	return pointToReturn;
}

void KinNode_Joint::collectControlThetas(std::vector<double>& allthetas, std::vector<bool>& iscontact, std::vector<bool>& ismoving, double t) {
	std::vector<SymbolicTransformation*> atrans = this->getArticulation()->transformations;
	for (std::vector<SymbolicTransformation*>::iterator transit = atrans.begin(); transit != atrans.end(); ++transit) {
		if ((*transit)->controller->isSymbolic() ) {
			SymbolicController* sc = (SymbolicController*)((*transit)->controller);
			allthetas.push_back(sc->getLinearController()->getVal(t));
			iscontact.push_back(sc->getLinearController()->inContact(t));
			ismoving.push_back(sc->getLinearController()->isMoving(t));
		}
	}

	for (auto jointit = this->children.begin(); jointit != this->children.end(); ++jointit) {
		(*jointit)->collectControlThetas(allthetas, iscontact, ismoving, t);
	}
}

void KinNode_Joint::broadcastControlThetas(std::list<double>& allthetas, double t) {
	std::vector<SymbolicTransformation*> atrans = this->getArticulation()->transformations;
	for (std::vector<SymbolicTransformation*>::iterator transit = atrans.begin(); transit != atrans.end(); ++transit) {
		if ((*transit)->controller->isSymbolic() ) {
			SymbolicController* sc = (SymbolicController*)((*transit)->controller);
			sc->getLinearController()->addPair(t, allthetas.front());
			allthetas.pop_front();
		}
	}

	for (auto jointit = this->children.begin(); jointit != this->children.end(); ++jointit) {
		(*jointit)->broadcastControlThetas(allthetas, t);
	}
}

void KinNode_Joint::updateTime(double t){
	articulation->updateTime(t);
}

bool KinNode_Joint::isSame(KinNode * node){

	if(node->getType() != KinNode::KinNodeType::JOINT)
		return false;

	KinNode_Joint* joint = dynamic_cast<KinNode_Joint*>(node);

	//some hack for now!
	if (joint->children.size() != 1)
		return false;
	if (children.size() != 1)
		return false;
	

	if(parent != joint->parent)
		return false;

	if(children[0] != joint->children[0])
		return false;

	LOG(WARNING) << "found the same joint and will remorve!";

	return true;

}

//----------------------------------------------------------------------------------


void KinChain::removeReduntantJoints(){

	std::list<KinNode*>::iterator it, it2, itnext2;
	for (it = nodes.begin(); it!=nodes.end(); ++it){
		if((*it)->getType() == KinNode::KinNodeType::JOINT){
			it2 = it;
			it2++;

			for (; it2!=nodes.end();){
				itnext2 = it2; itnext2++;
				if((*it2)->isSame(*it)){
					(*it2)->parent->removeChild(*it2);
					nodes.erase(it2);
				}
				it2 = itnext2;
			}
		}


	}

}

void KinChain::fixLockedJoints() {
	// search for loops of size 3 and fix the angles of the controllers
	for (std::list<KinNode*>::iterator nodeit = nodes.begin(); nodeit != nodes.end(); ++nodeit) {
		if ((*nodeit)->getType() != KinNode::JOINT) {
			continue;
		}

		// it's a joint - go down 2 levels
		KinNode* child = (*nodeit)->children[0]; // there is one child because it's a joint
		KinNode* parent = (*nodeit)->parent;     

		std::vector<KinNode*> childjoints = child->getNeighbors();
		std::vector<KinNode*> parentjoints = parent->getNeighbors();

		for (std::vector<KinNode*>::iterator childit = childjoints.begin(); childit != childjoints.end(); ++childit) {
			if ((*childit) == (*nodeit)) {
				VLOG(3) << "skipping original node in child list" << std::endl;
				continue;
			}

			KinNode* childcheck = (*childit)->parent;
			if (childcheck == child) {
				childcheck = (*childit)->children[0];
			}

			for (std::vector<KinNode*>::iterator parentit = parentjoints.begin(); parentit != parentjoints.end(); ++parentit) {
				if ((*parentit) == (*nodeit)) {
					VLOG(3) << "skipping original node in parent list" << std::endl;
					continue;
				}

				KinNode* parentcheck = (*parentit)->parent;
				if (parentcheck == parent) {
					parentcheck = (*parentit)->children[0];
				}

				if (parentcheck == childcheck) {
					//((KinNode_Joint*)(*nodeit))->isFixed = true;
					//((KinNode_Joint*)(*childit))->isFixed = true;
					//((KinNode_Joint*)(*parentit))->isFixed = true;

					// TODO: calculate fixed angle
					VLOG(3) << "WARNING: not implemented: calculate angle of joint" << std::endl;
				}
			}
		}
	}
}

KinChain::KinChain(Template* temp){
	//parse temp and create the chain 
	//step 1: retrieve all the faces
	
	std::vector<TemplateElement*> tempElements;
	temp->getAllElementTempList(tempElements);
	nodes.clear();
	for(int i = 0; i < tempElements.size(); i ++){
		if(dynamic_cast<Element_Motion*> (tempElements[i]->getElement()) ==  nullptr){
			KinNode_Part* newNode = new KinNode_Part(tempElements[i]);
			//newNode->display(0, false);
			nodes.push_back((KinNode*) newNode);		
		}
	}
	//std::cout << "initially had this many nodes " << nodes.size() <<std::endl;

	//step 2: retrieve all the edges amd group nodes
	std::vector<NewConnection *> connections;  
	temp->getConnections(connections, TreeScope::DESCENDANTS);
	for(int i = 0; i < connections.size(); i++){
		NewConnection * conn = connections[i];
		if (conn->isIncomplete()) {
			continue;
		}
		KinNode_Part* node1; KinNode_Part* node2;
		int edgeID1, edgeID2;
		if(INVERTTREE){
			getNodeFromPatch(conn->patches[1], &node1);
			getNodeFromPatch(conn->patches[0], &node2);
		}else{
			getNodeFromPatch(conn->patches[0], &node1);
			getNodeFromPatch(conn->patches[1], &node2);
		}
		NewConnection::ConnectionType connType = conn->getConnectionType();

		// combine nodes that are not connected by a joint
		if(node1 != node2){
			if(!NewConnection::isJoint(connType)){
				//combine 2 nodes
				//std::cout << "will combine nodes " << std::endl; 
				node1->add(node2);
				// now has to remove the node from the graoh 
				std::list<KinNode*>::iterator it;
				for (it = nodes.begin(); it!=nodes.end();){
					std::list<KinNode*>::iterator it2 = it;
					++it2;
					KinNode* erNode = (KinNode*) node2;
					if(erNode == (*it)){
						nodes.erase(it);
						//std::cout << "erased node" << std::endl;
					}
					it = it2; 
				}
			}
		}
	
	}

	//step 3: retrieve all the edges and add joints
	for(int i = 0; i < connections.size(); i++){
		NewConnection * conn = connections[i];
		JointConnection * jointConn = dynamic_cast<JointConnection*>(conn);
		if(jointConn != nullptr){
			KinNode_Part* node1; KinNode_Part* node2;
			int edgeID1, edgeID2;
			if(INVERTTREE){
				getNodeFromPatch(conn->patches[1], &node1);
				getNodeFromPatch(conn->patches[0], &node2);
			}else{
				getNodeFromPatch(conn->patches[0], &node1);
				getNodeFromPatch(conn->patches[1], &node2);
			}
			if(node1 != node2){
				//Todo: create the articulation
				//std::cout << "will add an articulation " << std::endl;
				
				KinNode_Joint* newJoint = new KinNode_Joint(jointConn->getArticulation());
				nodes.push_back((KinNode*) newJoint); 
				node1->addChild((KinNode*)newJoint);
				newJoint->addChild((KinNode*)node2);
			}			
		}
	
	}


	//AddTheContacPoints
	for each (auto tmpl in temp->getTemplatesByScope(TreeScope::DESCENDANTS)) {
		if(tmpl->contactPoints.size() >0){
			KinNode_Part* node = getNodeFromTemplate(tmpl);  
			for each ( auto c in tmpl->contactPoints){
				node->contactpoints.push_back(c);
			}
		}
	}



	removeReduntantJoints();
	//fixLockedJoints(); // theoretically not needed if KinChain is actually a tree

	//displayNodes();
	//system("pause"); 

	//check the tree
	int N_NodesWithNoParent = 0;
	std::list<KinNode*>::iterator it;
	for (it = nodes.begin(); it!=nodes.end(); ++it){
		if((*it)->hasNoParent()){
			root = (*it); 
			N_NodesWithNoParent++;
		}
	}

	//this->displayTree(); 
	//displayNodes();

	if(N_NodesWithNoParent != 1){
		LOG(ERROR) << "this is not a tree!";
		displayNodes();
		//system("pause"); 
	}

	// find driving thetas
	//labelDrivingThetas();

	// initialize rootpose to zero
	KinRigidTransform zeroPose;
	rootPose.push_back(zeroPose);


	computeGaitOptions();



}

KinNode* KinChain::find(TemplateElement* tempEl) {
	for each (KinNode* n in this->nodes) {
		if (n->getType() == KinNode::KinNodeType::PART) {
			KinNode_Part* np = dynamic_cast<KinNode_Part*>(n);
			for each (TemplateElement* te in np->elements) {
				if (te == tempEl) {
					return np;
				}
			}
		}
	}

	return NULL;
}
	

std::list<KinNode*> KinChain::getLeaves() {
	std::list<KinNode*> leaves;

	// do BFS
	std::list<KinNode*> tosearch;
	tosearch.push_back(root);
	while (!tosearch.empty()) {
		KinNode* nodeToCheck = tosearch.front();

		if (nodeToCheck->children.size() == 0) { // is a leaf
			leaves.push_back(nodeToCheck);
		} else {
			for (auto it = nodeToCheck->children.begin(); it != nodeToCheck->children.end(); ++it) {
				if ((*it)->parent != nodeToCheck) {
					// do not add the node to the list - it will be searched in another branch
					LOG(ERROR) << "this is not a tree: contains cycles";
					displayNodes();
				} else {
					tosearch.push_back(*it);
				}
			}
		}

		tosearch.pop_front();
	}

	return leaves;
}

std::vector<Eigen::Vector3d> KinChain::getAllContactPoints(double t, WhichContacts incontact) {

	if (t != NULL) {
		updateTime(t);
	}

	std::vector<Eigen::Vector3d> contacts;

	// find body plane
	//Eigen::Vector3d bodypoint;
	//Eigen::Vector3d bodynorm;
	//std::vector<Eigen::Vector3d> bodyattach = this->getRoot()->getAllAttachmentPoints();
	//GeometricOperations::calcPlane(bodyattach, bodypoint, bodynorm);

	// find appendage meshes
	std::list<KinNode*> appendages = getLeaves();
	
	for (auto appit = appendages.begin(); appit!= appendages.end(); ++appit) {
		vector<SymbolicTransformation*> trans = ((KinNode_Joint*)(*appit)->parent)->getArticulation()->transformations;

		// find all contact points
		vector<ContactPoint*> cp = ((KinNode_Part*)(*appit))->contactpoints;
		//std::cout << cp.size() << std::endl;
		std::vector<point> points;
				
		for (int i = 0; i<cp.size(); ++i) {
			points.push_back(cp[i]->cPoint.evalpoint(SymbolicAssignment::USE_CURRENT_VALUES));
		}
		(*appit)->transformToRootFrame(points);

		PWLinearController * pwlinearController;
		WhichContacts type = WhichContacts::ALL;
		if (trans[0]->controller->isSymbolic()) {
			pwlinearController = ((SymbolicController*)(trans[0]->controller))->getLinearController();
		}else{
			GrammarController* grammarController = dynamic_cast<GrammarController*>(trans[0]->controller);
			if(grammarController != nullptr){
				pwlinearController = grammarController->getLinearController(); 
				switch (grammarController->getType()) {
				case (GrammarController::GrammarcontrollerType::LEG) :
				case (GrammarController::GrammarcontrollerType::DOUBLE_ELBOW) :
					type = WhichContacts::LEGS;
					break;
				case (GrammarController::GrammarcontrollerType::WHEEL) :
					type = WhichContacts::WHEELS;
					break;
				default:
					type = WhichContacts::NONCONTACT;
				}
			}
		}			

		// assign type based on # contacts
		if (type == WhichContacts::ALL) {
			if (cp.size() == 1) {
				type = WhichContacts::LEGS;
			} else {
				type = WhichContacts::WHEELS;
			}
		}
		if(pwlinearController->inContact(t)) {
				// is in contact

			if (incontact != WhichContacts::NONCONTACT && incontact != WhichContacts::LEGS && type == WhichContacts::WHEELS) {
					// hack for wheels
					//Eigen::Vector3d pcenter = (*appit)->getCenter().center;
					//Eigen::Vector3d p1(points[0][0], points[0][1], points[0][2]);
					//double r = (p1-pcenter).norm();
					//pcenter(1) -= r;

					// find lowest contact
					int lowidx = 0; double lowy = points[0][1];
					for (int checkidx = 1; checkidx < points.size(); ++checkidx) {
						if (points[checkidx][1] < lowy) {
							lowidx = checkidx;
							lowy = points[checkidx][1];
						}
					}
					Eigen::Vector3d pcenter(points[lowidx][0], points[lowidx][1], points[lowidx][2]);

					contacts.push_back(pcenter);
				} else if (incontact != WhichContacts::NONCONTACT && incontact != WhichContacts::WHEELS && type == WhichContacts::LEGS) {
					for (int addidx = 0; addidx < points.size(); ++addidx) {
						Eigen::Vector3d p1(points[addidx][0], points[addidx][1], points[addidx][2]);
						contacts.push_back(p1);
					}
				}

				/*
				int minid = -1;
				double min = std::numeric_limits<double>::max();
				for(int i = 0; i < points.size(); i++){
					Eigen::Vector3d pi(points[i][0], points[i][1], points[i][2]);
					double dist = bodynorm.dot(pi-bodypoint);

					if (dist < min){
						if (incontact != WhichContacts::CONTACT && minid >= 0) {
							Eigen::Vector3d p1(points[minid][0], points[minid][1], points[minid][2]);
							contacts.push_back(p1);
						}
						min = dist;
						minid = i;
					}
				}

				if (incontact != WhichContacts::NONCONTACT && minid >= 0) {
					Eigen::Vector3d p1(points[minid][0], points[minid][1], points[minid][2]);
					contacts.push_back(p1);
				}
				*/
			} else if (incontact != WhichContacts::CONTACT) {
				if (incontact != WhichContacts::WHEELS && type == WhichContacts::LEGS ) {
					for (int addidx = 0; addidx < points.size(); ++addidx) {
						Eigen::Vector3d p1(points[addidx][0], points[addidx][1], points[addidx][2]);
						contacts.push_back(p1);
					}
				}
				else if (incontact == WhichContacts::WHEELS && type == WhichContacts::WHEELS) {
					// hack for wheels
					//Eigen::Vector3d pcenter = (*appit)->getCenter().center;
					//Eigen::Vector3d p1(points[0][0], points[0][1], points[0][2]);
					//double r = (p1-pcenter).norm();
					//pcenter(1) -= r;

					// find lowest contact
					int lowidx = 0; double lowy = points[0][1];
					for (int checkidx = 1; checkidx < points.size(); ++checkidx) {
						if (points[checkidx][1] < lowy) {
							lowidx = checkidx;
							lowy = points[checkidx][1];
						}
					}
					Eigen::Vector3d pcenter(points[lowidx][0], points[lowidx][1], points[lowidx][2]);

					contacts.push_back(pcenter);
				} else if (incontact != WhichContacts::LEGS && incontact != WhichContacts::WHEELS) {
					std::cout <<"adding all contact points" << std::endl;
					for (int i = 0; i < points.size(); ++i) {
						Eigen::Vector3d p1(points[i][0], points[i][1], points[i][2]);
						contacts.push_back(p1);
					}
				}
			}
		
	}

	return contacts;

}


void KinChain::getNodeFromPatch(NewPatch * patch, KinNode_Part** node){
	bool found = false;
	std::list<KinNode*>::iterator it;
	for (it = nodes.begin(); it!=nodes.end(); ++it){
		if((*it)->getType() == KinNode::KinNodeType::PART){
			KinNode_Part * nodePart = dynamic_cast<KinNode_Part*>(*it);
			for each (auto el in patch->getElement()->getAllElementTempList()){
				if(nodePart->checkContainsElement(el->getID())){
					(*node) = nodePart;
					found = true;
				}
			}
		}
	}
	if(!found){
		LOG(FATAL) << "could not find patch correspondance!";
	}
}

KinNode_Part* KinChain::getNodeFromTemplate(Template* temp){
	KinNode_Part* node;
	bool found = false;
	std::list<KinNode*>::iterator it;
	for (it = nodes.begin(); it!=nodes.end(); ++it){
		if((*it)->getType() == KinNode::KinNodeType::PART){
			KinNode_Part * nodePart = dynamic_cast<KinNode_Part*>(*it);
			for each (auto el in temp->getAllElementTempList()){
				if(nodePart->checkContainsElement(el->getID())){
					node = nodePart;
					found = true;
				}
			}
		}
	}
	if(!found){
		LOG(FATAL) << "could not find template correspondance!";
		system("pause");
	}
	return node;
}


void KinChain::displayNodes(){
	VLOG(3) << "These are the nodes";
	std::list<KinNode*>::iterator it;
	for (it = nodes.begin(); it!=nodes.end(); ++it){
		(*it)->display(0, false);
	}
	VLOG(3) << "--------------------------------------------";

}

void KinChain::displayTree(){
	VLOG(3) << "This is the tree";
	root->display(0, true);
	VLOG(3) << "--------------------------------------------";

}

KinRigidTransform KinChain::getPoseAtT(double t) {

	KinRigidTransform toreturn;

	std::pair<double, int> roundinfo = this->adjustTimeToSS(t);
	KinRigidTransform ssT;
	if (roundinfo.second > 0) {
		ssT = this->getSSTransform();
	}
	t = roundinfo.first;

	// linear interpolation (assume constant deltaT)
	int startidx = (int)(t/dt);
	if (startidx >= rootPose.size()) {startidx = rootPose.size() - 1;}

	int endidx = startidx + 1;
	if (endidx >= rootPose.size()) {endidx = startidx;}
	double prop = (double)t/(double)(dt) - startidx;

	toreturn.trans = (1-prop)*rootPose[startidx].trans + prop*rootPose[endidx].trans;

	// interpolate rotation matrix using slerp on quaternions
	Eigen::Quaterniond startQuat(rootPose[startidx].rot);
	Eigen::Quaterniond endQuat(rootPose[endidx].rot);
	Eigen::Quaterniond interpQuat = startQuat.slerp(prop, endQuat);
	toreturn.rot = interpQuat.toRotationMatrix();

	for (int extrarounds = 0; extrarounds < roundinfo.second; ++extrarounds) {
		toreturn.combine(ssT);
	}

	return toreturn;
}

void KinChain::updateCurrentPose(double t) {
	this->currentRootPose = getPoseAtT(t);
}

void KinChain::updateTime( double t){

	int current_round = std::floor(t - dt/2);
	if(current_round <0){
		current_round = 0;
	}
	double tmod = (t) - current_round;
	//std::cout << tmod <<std::endl;
	
	std::list<KinNode*>::iterator it;
	for (it = nodes.begin(); it!=nodes.end(); ++it){
		(*it)->updateTime(tmod);
	}

	updateCurrentPose(t);
}

Geometry* KinChain::getGeometry(bool withnormals){

	Geometry* geo = new Geometry();
	root->addGeometry(geo, withnormals);
	//geo->write("..\\..\\data\\test\\orig_pose_beforeapp.stl");
	currentRootPose.apply(geo);
	//std::cout << "----------------------current root pose" << std::endl;
	//currentRootPose.display();
	return geo ;

}

CenterOfMass  KinChain::getCenterOfMass(){
	CenterOfMass centerofmass;
	root->addCenterOfMass(centerofmass);
	currentRootPose.transform(centerofmass.center);
	return centerofmass;
}

Eigen::MatrixXd multiplyMatrix(Eigen::MatrixXd V, Eigen::MatrixXd Ut) {
	Eigen::MatrixXd VUt(V.rows(), Ut.cols()); 

	for (int Vi = 0; Vi < V.rows(); ++Vi) {
		for (int Uj = 0; Uj < Ut.cols(); ++ Uj) {
			VUt(Vi, Uj) = 0;
			for (int kmult = 0; kmult < V.cols(); ++kmult) {
				VUt(Vi, Uj) += V(Vi, kmult) * Ut(kmult, Uj);
			}
		}
	}

	return VUt;
}

double getFixedPointTranformation(Eigen::MatrixXd & mat1 ,Eigen::MatrixXd & mat2, KinRigidTransform & transf){

	//std::cout << "mat1 = \n " << mat1 << std::endl;
	//std::cout << "mat2 = \n " << mat2 << std::endl;

	transf.clear();
	double error = 0;

	if ((mat1-mat2).norm() < .01 || mat1.rows() == 1) {
		//std::cout << "translation only\n" << std::endl;
		transf.rot = Eigen::Matrix3d::Identity();
		transf.trans = (mat1.block(0,0,1,3) - mat2.block(0,0,1,3)).transpose();
	} else if (false) { 
		// FIXME: assume no rotation, just find difference between point averages

		//std::cout << "\nMAT1:\n" << mat1 << std::endl;
		//std::cout << "\nMAT2:\n" <<  mat2 << std::endl;
		
		Eigen::VectorXd matsum = (mat1.colwise().sum() - mat2.colwise().sum());
		//std::cout << "\nMAT1 sum:\n" << sum << std::endl;

		transf.trans = matsum/mat1.rows();

		//std::cout << "trans:\n" <<  transf.trans << std::endl;

		Eigen::MatrixXd temp = (mat1.rowwise()+transf.trans.transpose());
		error = (temp-mat2).norm();
	} else {
		// QUESTION: Does this work for <3 pts?
		// use only (x,y coordinates since it stays in the plane)
		Eigen::MatrixXd mat1red(mat1.rows(), 2);
		mat1red.block(0,0,mat1.rows(), 1) << mat1.block(0,0,mat1.rows(),1);
		mat1red.block(0,1,mat1.rows(), 1) << mat1.block(0,2,mat1.rows(),1);

		Eigen::MatrixXd mat2red(mat1.rows(), 2);
		mat2red.block(0,0,mat2.rows(), 1) << mat2.block(0,0,mat2.rows(),1);
		mat2red.block(0,1,mat2.rows(), 1) << mat2.block(0,2,mat2.rows(),1);

		if (debugSlip) {
			std::cout << "\nMAT1:\n" << mat1 << std::endl;
			std::cout << "\nMAT2:\n" << mat2 << std::endl;
			std::cout << "\nMAT1RED:\n" << mat1red << std::endl;
			std::cout << "\nMAT2RED:\n" << mat2red << std::endl;
		}

		// find the rotation and translation that minimizes least squares error;

		// centroids of points:
		Eigen::VectorXd centroid1 = mat1red.colwise().sum()/mat1red.rows();
		Eigen::VectorXd centroid2 = mat2red.colwise().sum()/mat2red.rows();

		//std::cout << "\ncentroid1:\n" << centroid1 << std::endl;
		//std::cout << "\ncentroid2:\n" <<  centroid2 << std::endl;

		Eigen::MatrixXd recenter1 = mat1red.rowwise()-centroid1.transpose();
		Eigen::MatrixXd recenter2 = mat2red.rowwise()-centroid2.transpose();
		
		//std::cout << "\nrecenter1:\n" << recenter1 << std::endl;
		//std::cout << "\nrecenter2:\n" <<  recenter2 << std::endl;

		Eigen::MatrixXd S = recenter1.transpose()*recenter2;

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::MatrixXd V = svd.matrixV();
		Eigen::MatrixXd U = svd.matrixU();
		Eigen::MatrixXd Ut = U.transpose();
		Eigen::MatrixXd VUt = multiplyMatrix(V,Ut); 
		
		if (debugSlip) {
			std::cout << "U\n" << U << "\nV\n" << V << "\nU'\n" << Ut << std::endl;
		}

		double D = VUt.determinant();
		Eigen::MatrixXd W = Eigen::MatrixXd::Identity(V.cols(), U.rows());
		W(V.cols()-1, U.rows()-1) = D;

		if (debugSlip) {
			std::cout << "V*U'\n" << VUt << "\ndet\n" << D << std::endl;
		}

		Eigen::MatrixXd rotmat = multiplyMatrix(multiplyMatrix(V, W), Ut); //V*W*U.transpose();
		transf.rot << rotmat(0,0), 0, rotmat(0,1), 0, 1, 0, rotmat(1,0), 0, rotmat(1,1);
		
		if (debugSlip) {
			std::cout << "W\n" << W << "\nrotmat\n" << rotmat << std::endl;

			std::cout << "\nANS:\n" <<  transf.rot << std::endl;
		}

		Eigen::Vector2d transmat = centroid2-rotmat*centroid1;
		transf.trans << transmat(0), 0, transmat(1);

		error = ((transf.rot*mat1.transpose()).colwise()+transf.trans-mat2.transpose()).norm();
		if (debugSlip) {
			std::cout<<"error: " << error << std::endl;
		}
	}
	return error;
	
}

void getSimpleTranformation(Geometry * geo, KinRigidTransform & rtrans, std::vector<Eigen::Vector3d> & points){

	/*
	std::vector<point> points;
	geo->addAllPoints(points);
	*/
	int min = -1;
	double minPoint =std::numeric_limits<double>::max();
	for(int i = 0; i < points.size(); i++){		
		if (points[i][1] < minPoint){
			min = i;
			minPoint = points[i][1] ;
		}
	}


	Eigen::Vector3d p1;
	if (min == -1) {
		p1 = Eigen::Vector3d(0, 0, 0);
	}
	else {
		p1 = Eigen::Vector3d(points[min][0], points[min][1], points[min][2]);
	}

	Eigen::Vector3d z = Eigen::Vector3d::Zero(3);
	z[1] = 1;

	rtrans.normal = z;
	rtrans.normal.normalize();

	Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors( rtrans.normal, z);
	rtrans.rot = q.toRotationMatrix();
	rtrans.trans = -1.0*rtrans.rot*p1;
	rtrans.trans(0) = 0;
	rtrans.trans(2) = 0;
}

void getSimpleTranformation(Geometry * geo, KinRigidTransform & rtrans){
	std::vector<Eigen::Vector3d> points;
	geo->addAllPoints(points);
	getSimpleTranformation(geo, rtrans, points);
}

bool isTriangle (Eigen::Vector3d a1, Eigen::Vector3d a2, Eigen::Vector3d b){
	
	
	if((a1 -a2).norm() < .1 ){
		return false; // this should never happen
	}

	if((a1 -b).norm() < .1 ){
		return false; // this should never happen
	}

	if((a2 -b).norm() < .1 ){
		return false; // this should never happen
	}
	

	Eigen::Vector3d dist1 = a1 - a2; dist1.normalize();
	Eigen::Vector3d dist2 = a1 - b;  dist2.normalize();

	if (DEBUG_OPTION) {
		std::cout << a1 << std::endl << a2 << std::endl << b<< std::endl<< std::endl;
		std::cout << dist1 << std::endl << dist2 << std::endl<< std::endl;
	}

	Eigen::Vector3d triArea =  dist1.cross(dist2);

	if (DEBUG_OPTION) {
		std::cout<<"triarea: " << triArea<< std::endl;
	}

	if((triArea).norm() < 0.1){ // 5 deg.
		return false;
	}

	return true;

}

bool isTriangle (point a1, point a2, point b){
	if(len(a1 -a2) < .1 ){
		return false; // this should never happen
	}

	if(len(a1 -b) < .1 ){
		return false; // this should never happen
	}

	if(len(a2 -b) < .1 ){
		return false; // this should never happen
	}

	point triArea =  ((a1 - a2) CROSS (a1 - b));

	if(len(triArea) < 0.1){
		return false;
	}

	return true;

}


static bool pointYCompoarator(point &A, point &B){
	return A[1] < B[1];
}


void writeLowPoints(Geometry * geo, std::vector<point> &baseTriangle, std::string name){
		
		Geometry * sphere1 = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
		sphere1->applyTrans(baseTriangle[0]);
		//sphere1->write("..\\..\\data\\test\\cpoint1.stl");
		Geometry * sphere2 = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
		sphere2->applyTrans(baseTriangle[1]);
		//sphere2->write("..\\..\\data\\test\\cpoint2.stl");
		Geometry * sphere3 = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
		sphere3->applyTrans(baseTriangle[2]);
		//sphere3->write("..\\..\\data\\test\\cpoint3.stl");
		Geometry * lowPoints = new Geometry();
		lowPoints->add(sphere1);
		lowPoints->add(sphere2);
		lowPoints->add(sphere3);
		lowPoints->add(geo);
		std::stringstream filename;
		filename << "..\\..\\data\\test\\lowPoints_" << name << ".stl";
		lowPoints->write(filename.str());
}

int getContactPoints(Geometry* geo, std::vector<Eigen::Vector3d> & points, std::vector<int> & contactPoints) {
	int count = 0;

	contactPoints.clear();
	for(int i = 0; i < points.size(); i++){
		if(points[i][1] < 0.1){
			bool doadd = true;
			
			for (int j = 0; j < contactPoints.size(); ++j) {
				if ((points[contactPoints[j]] - points[i]).norm() < 0.01) {
					doadd = false;
					break;
				}
			}

			if (doadd) {
				contactPoints.push_back(i);
				count++; 
			}
		}
		//if (points[i][1] < .1) {
		//	std::cout << points[i].transpose() << std::endl;
		//}
		if (points[i][1] < -0.2) {
			std::cout << "point went below ground: " << std::endl;
			std::cout << points[i].transpose();
			std::cout << std::endl;
			//system("pause");
		}
	}
	return count;
}

void KinChain::clearGaitInfo() {
	stabilityCost.clear();
	slipCost.clear();
	rootPose.clear();
	resetControllers();
	nrounds = 0;
	steadyStateRounds.first = 0;
	steadyStateRounds.second = 0;

	currentRootPose.clear();
	rootPose.push_back(currentRootPose);
}

void KinChain::resetControllers() {
	// traverse tree using DFS and clear control poses
	std::list<KinNode*> tosearch;
	tosearch.push_back(root);
	// BFS
	while (!tosearch.empty()) {
		KinNode* nodeToCheck = tosearch.front();

		// if is a joint with symbolic controller, clear the evaluated motion
		if (nodeToCheck->getType() == KinNode::JOINT) {
			((KinNode_Joint*)nodeToCheck)->resetControllers();
		}

		if (nodeToCheck->children.size() > 0) { // has children
			for (auto it = nodeToCheck->children.begin(); it != nodeToCheck->children.end(); ++it) {
				if ((*it)->parent != nodeToCheck) {
					// do not add the node to the list - it will be searched in another branch
					LOG(ERROR) << "this is not a tree: contains cycles";
					displayNodes();
				} else {
					tosearch.push_back(*it);
				}
			}
		}

		tosearch.pop_front();
	}

}

void KinChain::resetDrivers() {
	// traverse tree using BFS and clear control poses
	std::list<KinNode*> tosearch;
	tosearch.push_back(root);

	// BFS
	while (!tosearch.empty()) {
		KinNode* nodeToCheck = tosearch.front();

		// if is a joint with symbolic controller, clear the evaluated motion
		if (nodeToCheck->getType() == KinNode::JOINT) {
			((KinNode_Joint*)nodeToCheck)->isDriving = false;
		}

		if (nodeToCheck->children.size() > 0) { // has children
			for (auto it = nodeToCheck->children.begin(); it != nodeToCheck->children.end(); ++it) {
				if ((*it)->parent != nodeToCheck) {
					// do not add the node to the list - it will be searched in another branch
					LOG(ERROR) << "this is not a tree: contains cycles";
					displayNodes();
				} else {
					tosearch.push_back(*it);
				}
			}
		}

		tosearch.pop_front();
	}

}

std::pair<int, int> KinChain::getSteadyStateIdx() {
	std::pair<int,int> ssr = this->getSteadyStateRounds();
	int firstIdx = ((double)ssr.first)/this->dt;
	int lastIdx = ((double)ssr.second)/(this->dt);
	return std::pair<int,int>(firstIdx, lastIdx);
}

double KinChain::getSSRotation() {
	std::pair<int,int> ssr = this->getSteadyStateRounds();
	if (ssr.first == ssr.second) {return 0;}

	KinRigidTransform startT = this->getPoseAtT(ssr.first);
	KinRigidTransform endT = this->getPoseAtT(ssr.second);

	Eigen::AngleAxisd theta(endT.rot * startT.rot.transpose());

	Eigen::Vector3d y(0,1,0);
	double t3 = theta.angle();
	if (theta.axis().dot(y) < 0) { t3 = -t3; }

	return t3;
}

double KinChain::getMeanCurvature() {
	Eigen::Vector3d vel = this->getSSTranslation();

	double theta = this->getSSRotation();
	double h = vel.norm();
	//std::cout << "h = " << h << std::endl;
	double rho = 1000* 2. * sin(abs(theta) / 2.) / h ; // curvature in m^(-1)

	return rho;
}

Eigen::Vector2d KinChain::getCenterOfRotation() {
	std::pair<int,int> ssr = this->getSteadyStateRounds();
	if (ssr.first == ssr.second) { return Eigen::Vector2d::Zero(); }

	KinRigidTransform ssT = this->getSSTransform();
	
	this->updateTime(ssr.first);
	Eigen::Vector3d p0 = this->getCenterOfMass().center;

	Eigen::Vector3d pf(p0);
	ssT.transform(pf);
	
	//VLOG(3) << "startpose = " << p0.transpose() << std::endl;
	//VLOG(3) << "endpose = " << pf.transpose() << std::endl;

	double x1 = p0[0], y1 = p0[2];
	double x2 = pf[0], y2 = pf[2];
	double t1 = atan2(y2-y1, x2-x1);
	double h = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
	
	double t3 = this->getSSRotation();

	double t2 = (M_PI - abs(t3)) / 2.;
	if (t3 < 0) { t2 = -t2; }
	//VLOG(3) << "t1 = " << t1 << "; t2 = " << t2 << "; t3 = " << t3 << std::endl;
	double R = h / 2. / sin(abs(t3) / 2.) ; // radius in mm
	//VLOG(3) << "h = " << h << ";R = " << R << std::endl;

	double x3 = R*cos(t1-t2) + x1;
	double y3 = R*sin(t1-t2) + y1;

	//VLOG(3) << "x3 = " << x3  << "; y3 = " << y3 << std::endl;

	return Eigen::Vector2d(x3, y3);
}

double KinChain::getMeanAngle() {
	std::pair<int,int> ssr = this->getSteadyStateRounds();
	if (ssr.first == ssr.second) {return 0;}

	Eigen::Vector2d C = this->getCenterOfRotation();

	this->updateTime(ssr.first);
	Eigen::Vector3d startPose = this->getCenterOfMass().center;

	//std::cout << "C = " << C.transpose() << std::endl;
	//std::cout << "startPose = " << startPose.transpose() << std::endl;

	return atan2(startPose[0]-C[0], C[1]-startPose[2]);
}

double KinChain::getStability() {
    double worstCaseStability = -std::numeric_limits<double>::max();
	std::pair<int, int> ssidx = getSteadyStateIdx();
	int firstIdx = ssidx.first; int lastIdx = ssidx.second;
	if (firstIdx == lastIdx) { return 0; }

	if (firstIdx >= stabilityCost.size() || lastIdx >= stabilityCost.size()) {
		std::cout << "PROBLEM: stabilityCost vector not long enough. Need indices ";
		std::cout << firstIdx << " through " << lastIdx << " but size is " << stabilityCost.size() << std::endl;
		std::cout << "Calculating over entire stability vector.\n";
		if (firstIdx >= stabilityCost.size()) {
			firstIdx = 0;
			lastIdx = stabilityCost.size()-1;
		} else {
			lastIdx = stabilityCost.size()-1;
		}
	}

	for (int idx = firstIdx; idx <= lastIdx; ++idx) {
		//std::cout << stabilityCost[idx] << " " << worstCaseStability << std::endl;
		if(stabilityCost[idx] > worstCaseStability){
			worstCaseStability = stabilityCost[idx];
		}
	}

	return worstCaseStability;
}

double KinChain::getSlip(bool doSequence) {
	double totalSlip = 0;

	std::pair<int, int> ssr = getSteadyStateRounds();
	if (ssr.first == ssr.second) {return 0;}

	double totalTime = 0;
	if(doSequence){
		std::vector<double> framesPerCycle = getTimesPerCycle(doSequence);
		for(int i = ssr.first; i < ssr.second; i++){
			totalTime += (double)(framesPerCycle[i]);
		}
	}else{
		totalTime = this->getTimeOfOneCycle()*(ssr.second - ssr.first);
	}

	std::pair<int, int> ssidx = getSteadyStateIdx();
	int firstIdx = ssidx.first; int lastIdx = ssidx.second;
	if (firstIdx == lastIdx) {return 0;}

	if (firstIdx >= slipCost.size() || lastIdx >= slipCost.size()) {
		std::cout << "PROBLEM: slipCost vector not long enough. Need indices ";
		std::cout << firstIdx << " through " << lastIdx << " but size is " << slipCost.size() << std::endl;
		std::cout << "Calculating over entire slip vector.\n";
		if (firstIdx >= slipCost.size()) {
			firstIdx = 0;
			lastIdx = slipCost.size()-1;
		} else {
			lastIdx = slipCost.size()-1;
		}
	}

	double slipsum = 0;

	for (int idx = firstIdx; idx < lastIdx; ++idx) {
		slipsum += slipCost[idx];
	}
	slipsum /= (totalTime);

    return slipsum;
}

KinRigidTransform KinChain::getSSTransform() {
	std::pair<int,int> ssr = this->getSteadyStateRounds();

	KinRigidTransform startT = this->getPoseAtT(ssr.first);
	
	KinRigidTransform endT = this->getPoseAtT(ssr.second);

	KinRigidTransform toreturn;
	toreturn.rot = endT.rot * startT.rot.transpose();
	toreturn.trans = endT.trans - toreturn.rot*startT.trans;
	
	return toreturn;
}

Eigen::Vector3d KinChain::getSSTranslation() {
	std::pair<int,int> ssr = this->getSteadyStateRounds();

	if (ssr.first == ssr.second) { return Eigen::Vector3d(0,0,0); }

	KinRigidTransform ssT = this->getSSTransform();
	
	this->updateTime(ssr.first);
	Eigen::Vector3d c0 = this->getCenterOfMass().center;

	Eigen::Vector3d cnew(c0);
	ssT.transform(cnew);

	return cnew-c0;
}


double KinChain::getSpeed(bool doSequence) {

	double totalDistance = 0 ;

	std::pair<int,int> ssr = this->getSteadyStateRounds();
	if (ssr.first == ssr.second) { return 0; }

	double totalTime = 0;
	if(doSequence){
		std::vector<double> framesPerCycle = getTimesPerCycle(doSequence);
		for(int i = ssr.first; i < ssr.second; i++){
			totalTime += (double)(framesPerCycle[i]);
		}
	}else{
		totalTime = this->getTimeOfOneCycle()*(ssr.second - ssr.first);
	}
	
	double t = ssr.first;
	updateTime(t);
	Eigen::Vector3d combefore = this->getCenterOfMass().center;
	
	for (int r = ssr.first; r < ssr.second; ++r) { 
		updateTime(r+1);
		Eigen::Vector3d comafter = this->getCenterOfMass().center;
			
		totalDistance += (comafter-combefore).norm();
		combefore = comafter;
	}
	//std::cout << totalDistance << "  " << totalTime << std::endl;
	return totalDistance/totalTime;

	//double R = 100./this->getMeanCurvature();
	//double theta = this->getSSRotation();

	//std::cout << "R = " << R << "; theta = " << theta << std::endl;
	
	//Eigen::Vector3d velocity = this->getSSTranslation();
	//std::cout << "R*theta = " << R*abs(theta) << "; |vel| = " << velocity.norm() << std::endl;
	//std::cout << "linear vel" << (velocity).norm()/((ssr.second-ssr.first)*this->getTimeOfOneCycle()) << std::endl;


	//return R*abs(theta) / ((ssr.second-ssr.first)*this->getTimeOfOneCycle());
	//return R*abs(theta) / totalTime;
	
}


double KinChain::getTimeOfOneCycle(){

	//compute max cycle speed 
	double maxCycleSpeed = 0; 
	int count = 0; 
	for each( auto n in nodes){
		if( n->getType() == KinNode::KinNodeType::JOINT){
			KinNode_Joint * joint = dynamic_cast<KinNode_Joint*>(n);
			GrammarController* controller = dynamic_cast<GrammarController*>(joint->getArticulation()->transformations[0]->controller);
			double currentCycleSpeed = controller->getLinearController()->getMaxCycleSpeed();
			 if( maxCycleSpeed < currentCycleSpeed){
				 maxCycleSpeed = currentCycleSpeed;
			 }
		}
	}

	double motorSpeed = 0.9*(M_PI/3.)/0.1;// 54 degrees per 0.1 sec 
	return maxCycleSpeed/motorSpeed; 

}



double KinChain::getForwardTravel(double angle) {
	std::pair<int, int> ssr = getSteadyStateRounds();

	if (ssr.first == ssr.second) {return 0;}

	Eigen::Vector3d vel = this->getSSTranslation();

	Eigen::Vector3d unitcompare(cos(angle), 0, sin(angle));

	return (vel.dot(unitcompare))/((ssr.second-ssr.first)*this->getTimeOfOneCycle());

}

double KinChain::getToppling(bool doSequence) {

	double totalError = 0;

	std::pair<int, int> ssr = getSteadyStateRounds();
	if (ssr.first == ssr.second) {return 0;}


	double totalTime = 0;
	if(doSequence){
		std::vector<double> framesPerCycle = getTimesPerCycle(doSequence);
		for(int i = ssr.first; i < ssr.second; i++){
			totalTime += (double)(framesPerCycle[i]);
		}
	}else{
		totalTime = this->getTimeOfOneCycle()*(ssr.second - ssr.first);
	}

	double t = ssr.first;
	KinRigidTransform transf = this->getPoseAtT(t);
	for (double t = ssr.first; t < ssr.second; t = t + dt) { 
		Eigen::Quaterniond q(transf.rot);
		KinRigidTransform transf2 = this->getPoseAtT(t+dt);
		Eigen::Quaterniond q2(transf2.rot);

		double angle = q.angularDistance(q2);
		totalError += abs(angle);
		transf = transf2;
	}
	return totalError/totalTime;
}

double KinChain::getTotalTimeSequence(){

	double totalTime = 0;
	if(gaitInterpolationSequence.size() >0){
		std::vector<double> framesPerCycle = getTimesPerCycle(true);
		
		std::cout << "got times per cycle" << std::endl;
		for(int i =0; i < framesPerCycle.size(); i++){
			totalTime += (double)(framesPerCycle[i]);
		}
	}
	return totalTime;
}

double KinChain::getObjectiveParam(int id, bool doSequence, double desiredCurvature){
	
	switch(id){
	case 1: 
		return getSpeed(doSequence);
		break;
	case 2:
		return (-1)*this->getToppling(doSequence);
		break;
	case 3:
		return (-1)*this->getSlip(doSequence);
		break;
	case 4:
		return (-1)*abs(this->getMeanCurvature() - desiredCurvature);
		break;
	case 5:
		return (-1)*this->getPerpendicularError();
		break;
	case 6:
		return (-1)*this->getSSRotation();
		break;

	default: 
		return 2;
		break;
			
	}

	return 2; 
	

}


void KinChain::getTranformation(Geometry * geo, KinRigidTransform & rtrans, std::vector<Eigen::Vector3d> & points){
	rtrans.clear();
	//find the lowest three points

	// find body plane
	//Eigen::Vector3d bodypoint, bodynorm;
	//std::vector<Eigen::Vector3d> bodyattach = getRoot()->getAllAttachmentPoints();
	//GeometricOperations::calcPlane(bodyattach, bodypoint, bodynorm);

	std::vector<Eigen::Vector3d> baseTriangle(points);
	// sort contact points according to z value, lowest points at the beginning
	std::sort(baseTriangle.begin(), baseTriangle.end(),
		[] (const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
			return lhs[1] < rhs[1];
		});

	//std::cout << "orig" << points<< std::endl << std::endl << std::endl;
	if (DEBUG_OPTION) {
		VLOG(3) << "sorted" << baseTriangle << std::endl;
	}
	
	/*
	int lowestPoints[3];
	double minPoints[3];
	std::vector<Eigen::Vector3d> baseTriangle(0);
	bool inserted = false;
	for(int i = 0; i < 3; i++) {
		inserted = false;
		double dist = points[i][1];
		for (int j = 0; j < i; ++j) {
			if (dist < minPoints[j]) {
				baseTriangle.insert(baseTriangle.begin()+j, points[i]);
				for (int ishift = i; ishift > j; --ishift) {
					minPoints[ishift] = minPoints[ishift-1];
					lowestPoints[ishift] = lowestPoints[ishift-1];
				}
				minPoints[j] = dist;
				lowestPoints[j] = i;
				inserted = true;
				break;
			}
		}
		if (!inserted) {
			baseTriangle.push_back(points[i]);
			minPoints[i] = dist;
			lowestPoints[i] = i;
		}
	}
	*/
	//std::cout<<baseTriangle.at(0) << std::endl;
	//std::cout<<baseTriangle.at(1) << std::endl;
	//std::cout<<baseTriangle.at(2) << std::endl;
	//std::cout << minPoints[0] << " "  << minPoints[1] << " "  << minPoints[2] << std::endl;

	//set random 3 points that are the lowest
	/*
	baseTriangle.push_back(points[0]);
	baseTriangle.push_back(points[1]);
	baseTriangle.push_back(points[2]);
	double foundSecond = false;
	int i = 1;
	while(!foundSecond){	
		if(len(points[i] - baseTriangle[0]) > 30){
			baseTriangle.push_back(points[i]);
			foundSecond = true;
		}
		i++;
	}
	double foundThird = false;
	i = 0;
	while(!foundThird){	
		if(isTriangle(baseTriangle[0], baseTriangle[1], points[i])){
			baseTriangle.push_back(points[i]);
			foundThird = true;
		}
		i++;
	}
	writeLowPoints(geo, baseTriangle, "initial");
	

	// for each point
	int rep_count = 0;
	for(int i = 3; i < points.size(); i++){		
		//std::cout << "Checking point: " << points[i] << std::endl;

		double dist = points[i][1];

		if(isTriangle(baseTriangle[0], baseTriangle[1], points[i])){
			if (dist < minPoints[0]) {
				// shift everything over
				baseTriangle.insert(baseTriangle.begin(), points[i]);
				baseTriangle.pop_back();
				minPoints[2] = minPoints[1]; minPoints[1] = minPoints[0]; minPoints[0] = dist;
				lowestPoints[2] = lowestPoints[1]; lowestPoints[1] = lowestPoints[0]; lowestPoints[0] = i;
				//std::cout << "Inserting at 0" << std::endl;
			} else if (dist < minPoints[1]) {
				// shift everything over
				baseTriangle.insert(baseTriangle.begin()+1, points[i]);
				baseTriangle.pop_back();
				minPoints[2] = minPoints[1]; minPoints[1] = dist;
				lowestPoints[2] = lowestPoints[1]; lowestPoints[1] = i;
				//std::cout << "Inserting at 1" << std::endl;
			} else if (dist < minPoints[2]) {
				// shift everything over
				baseTriangle.pop_back();
				baseTriangle.push_back(points[i]);
				minPoints[2] = dist;
				lowestPoints[2] = i;
				//std::cout << "Inserting at 2" << std::endl;
			}
		
		}
	} 
	*/
		/*
		if (points[i][1] < baseTriangle[2][1]){
			if(isTriangle(baseTriangle[0], baseTriangle[1], points[i])){
				baseTriangle[2] = points[i];
				std::sort(baseTriangle.begin(), baseTriangle.end(), pointYCompoarator);
				//std::stringstream name ; name <<"rep_ " << rep_count;
				//writeLowPoints(geo, baseTriangle, name.str());
				//rep_count++;
			}else{
				if (points[i][1] < baseTriangle[1][1]){
					if(isTriangle(baseTriangle[0], baseTriangle[2], points[i])){
						baseTriangle[1] = points[i];
						std::sort(baseTriangle.begin(), baseTriangle.end(), pointYCompoarator);
						//std::stringstream name ; name <<"rep_ " << rep_count;
						//writeLowPoints(geo, baseTriangle, name.str());
						//rep_count++;
					}else{
						if (points[i][1] < baseTriangle[0][1]){
							if(isTriangle(baseTriangle[1], baseTriangle[2], points[i])){
								baseTriangle[0] = points[i];
								std::sort(baseTriangle.begin(), baseTriangle.end(), pointYCompoarator);
								//std::stringstream name ; name <<"rep_ " << rep_count;
								//writeLowPoints(geo, baseTriangle, name.str());
								//rep_count++;
							}
						}
					}
				}
			}
		}
	}
	

	for(int i = 0; i < points.size(); i++){		
		if (points[i][1] < baseTriangle[2][1]){
			if(isTriangle(baseTriangle[0], baseTriangle[1], points[i])){
				baseTriangle[2] = points[i];
				std::sort(baseTriangle.begin(), baseTriangle.end(), pointYCompoarator);
				//std::stringstream name ; name <<"rep_ " << rep_count;
				//writeLowPoints(geo, baseTriangle, name.str());
				//rep_count++;
			}else{
				if (points[i][1] < baseTriangle[1][1]){
					if(isTriangle(baseTriangle[0], baseTriangle[2], points[i])){
						baseTriangle[1] = points[i];
						std::sort(baseTriangle.begin(), baseTriangle.end(), pointYCompoarator);
						//std::stringstream name ; name <<"rep_ " << rep_count;
						//writeLowPoints(geo, baseTriangle, name.str());
						//rep_count++;
					}else{
						if (points[i][1] < baseTriangle[0][1]){
							if(isTriangle(baseTriangle[1], baseTriangle[2], points[i])){
								baseTriangle[0] = points[i];
								std::sort(baseTriangle.begin(), baseTriangle.end(), pointYCompoarator);
								//std::stringstream name ; name <<"rep_ " << rep_count;
								//writeLowPoints(geo, baseTriangle, name.str());
								//rep_count++;
							}
						}
					}
				}
			}
		}
	}

	*/
	
	Eigen::Vector3d p1 = baseTriangle[0];

	int i = 1;
	while (i < baseTriangle.size() && (baseTriangle[i]-p1).norm() < .1) {++i;}
	Eigen::Vector3d p2 = baseTriangle[i];

	// find next point that is sufficiently far away
	++i;
	while (i < baseTriangle.size() && !isTriangle(p1, p2, baseTriangle[i])) { ++i; }
	if (i == baseTriangle.size()) {
		i = 2;
		if (DEBUG_OPTION) {
			std::cout << "setting i to 2\n\n";
			std::cout << baseTriangle << std::endl;
		}
	}

	Eigen::Vector3d p3 = baseTriangle[i];
	
	if (DEBUG_OPTION) {
		std::cout << "p1 = " << p1.transpose() << std::endl;
		std::cout << "p2 = " << p2.transpose() << std::endl;
		std::cout << "p3 = " << p3.transpose() << std::endl;
	}
	//system("pause"); 

	//	point ppoint = points[lowestPoints[0]];
	//	std::cout << "pppoiint = " << points[lowestPoints[0]][0] <<", " << points[lowestPoints[0]][1] <<", " << points[lowestPoints[0]][2] << std::endl;

	
			
	if(0){
		 std::cout << "p1 = " << p1.transpose() << std::endl; 
		std::cout << "p2 = " << p2.transpose() << std::endl; 
		std::cout << "p3 = " << p3.transpose() << std::endl;

		Geometry * sphere1 = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
		//sphere1->applyTrans(baseTriangle[0]);
		//sphere1->write("..\\..\\data\\test\\cpoint1.stl");
		Geometry * sphere2 = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
		//sphere2->applyTrans(baseTriangle[1]);
		//sphere2->write("..\\..\\data\\test\\cpoint2.stl");
		Geometry * sphere3 = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
		//sphere3->applyTrans(baseTriangle[2]);
		//sphere3->write("..\\..\\data\\test\\cpoint3.stl");
		Geometry * lowPoints = new Geometry();
		lowPoints->add(sphere1);
		lowPoints->add(sphere2);
		lowPoints->add(sphere3);
		lowPoints->add(geo);
		lowPoints->write("..\\..\\data\\test\\lowPoints.stl");
		//system("pause");
	}

	/*
	//std::cout << "size of cross is " << (p2-p1).cross(p3 - p1).norm() << std::endl;
	if((p2-p1).cross(p3 - p1).norm() < 0.1){
		//std::cout << "is too small" << std::endl;
		int new_i = 0;
		double minVal = std::numeric_limits<double>::max();
		//system("pause");
		for(int i = 0; i < points.size(); i++){		
			Eigen::Vector3d pnew(points[i][0], points[i][1], points[i][2]);
			double err = (p2-p1).cross(pnew - p1).norm();
			if((err >0.1) && (points[i][1] < minVal)){
				//std::cout << "pnew = " << pnew.transpose() << std::endl;
				minVal = points[i][1];
				new_i = i;
				//std::cout << "new_i = " << new_i << std::endl;
				//system("pause");
			}
		}
		p3 = Eigen::Vector3d(points[new_i][0], points[new_i][1], points[new_i][2]);
	}*/

	rtrans.normal = (p2-p1).cross(p3 - p1);
	//std::cout<<rtrans.normal<<std::endl<<std::endl;

	rtrans.normal.normalize();
	Eigen::Vector3d z = Eigen::Vector3d::Zero(3);
	z[1] = 1;
	rtrans.normal *= (rtrans.normal.dot(z) > 0)? 1.0:-1.0; 	
	Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z, rtrans.normal);
	Eigen::AngleAxisd aa = Eigen::AngleAxisd(q);
	//std::cout << "scaling by " << abs(aa.angle()) << std::endl;

	// check angular distance
	if (abs(aa.angle()) > M_PI/10.0 && abs(aa.angle()-2*M_PI) > M_PI/10.0) {
		//std::cout << " is too much, lowering to pi/10 \n";
		aa = Eigen::AngleAxisd(M_PI/10.0, aa.axis());
	}


	rtrans.rot = aa.toRotationMatrix().transpose();
	//std::cout<<rtrans.rot << std::endl<<std::endl;
	//std::cout<<q.toRotationMatrix().transpose()<< std::endl<<std::endl;

	//std::cout<<p1.transpose()*rtrans.rot.transpose()<< std::endl;
	//std::cout<<p2.transpose()*rtrans.rot.transpose() << std::endl;
	//std::cout<<p3.transpose()*rtrans.rot.transpose() << std::endl;

	rtrans.trans = -1.0*rtrans.rot*p1;
	rtrans.trans(0) = 0;
	rtrans.trans(2) = 0;


	//return rtrans;

}

static bool compareMinPoints( std::pair<double,Eigen::Vector3d> &A, std::pair<double,Eigen::Vector3d> &B){
	return A.first > B.first;
}

bool KinChain::rotateToGround(Geometry * geo, CenterOfMass & com, KinRigidTransform & rtrans, std::vector<Eigen::Vector3d> & points){

	rtrans.clear();
	
	// count contact points
	std::vector<int> contactPoints;
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> cpLoc;
	int count = getContactPoints(geo, points, contactPoints);
	if (DEBUG_OPTION) {
		std::cout << count << " contact points\n";
	}
	for (int i = 0; i < count; ++i) {
		Eigen::Vector2d proj(points[contactPoints[i]].x(), points[contactPoints[i]].z());
		cpLoc.push_back(proj);
		if (DEBUG_OPTION) {
			std::cout << proj.transpose() << std::endl;
		}
	}

	// STEP 1: find axis of rotation
	if (count == 0) {
		// no contact points: call simpleTranformation
		getSimpleTranformation(geo, rtrans, points);
		return false;
	}

	Eigen::Vector3d rotationCenter;
	Eigen::Vector3d rotationAxis;
	Eigen::Vector3d perpDir;
	Eigen::Vector3d center = com.center;
	int rotCenterID = -1, rotAxisID = -1;

	// get stability error
	double error = GeometricOperations::getStabilityCost(com,cpLoc,rotCenterID, rotAxisID);

	if (DEBUG_OPTION) {
		std::cout << "center of mass = " << com.center << std::endl;
		std::cout << "error = " << error << std::endl;
		if (rotCenterID >= 0) {
			std::cout << "rotcenter ( " << rotCenterID << ") = " << points[contactPoints[rotCenterID]].transpose() << std::endl;
		}
		if (rotAxisID >= 0) {
			std::cout << "rotaxis ( " << rotAxisID << ") = " << points[contactPoints[rotAxisID]].transpose() << std::endl;
		}
	}

	// if it is stable, return identity
	if (error < 0) {
		Eigen::Vector3d z = Eigen::Vector3d::Zero(3);
		z[1] = 1;

		rtrans.normal = z;
		rtrans.normal.normalize();

		Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors( rtrans.normal, z);
		rtrans.rot = q.toRotationMatrix();

		rtrans.trans = Eigen::Vector3d::Zero(3);
		return true;
	}

	// otherwise, find the axis of rotation using nearest edge info from getStabilityCost
	rotationCenter = points[contactPoints[rotCenterID]];
	if (rotAxisID >= 0) {
		// if the point z-coordinate is above 0.1 (i.e., the robot is tilted), rotate so the contact points are on the ground
		if (abs(points[contactPoints[rotAxisID]].y()-rotationCenter.y()) > .1) {

			if (count > 2) {
				// find the plane of best fit of all the contact points
				Eigen::MatrixXd X(count, 3);
				for (int i = 0; i < count; ++i) {
					X.row(i) << points[contactPoints[i]].x(), points[contactPoints[i]].y(), points[contactPoints[i]].z();
				}
			
				if (DEBUG_OPTION) {
					std::cout << "\nX:\n" << X << std::endl;
				}
			
				// centroid of points:
				Eigen::VectorXd centroid = X.colwise().sum()/count;

				Eigen::MatrixXd recenterX = X.rowwise()-centroid.transpose();

				if (DEBUG_OPTION) {
					std::cout << "recenterX\n" << recenterX << std::endl;
				}

				Eigen::JacobiSVD<Eigen::MatrixXd> svd(recenterX, Eigen::ComputeThinU | Eigen::ComputeThinV);
				Eigen::MatrixXd V = svd.matrixV();
		
				if (DEBUG_OPTION) {
					std::cout << "V\n" << V << std::endl;
				}
		
				Eigen::Vector3d z = Eigen::Vector3d::Zero(3);
				z[1] = 1;
				Eigen::Vector3d toaxis = V.col(2); toaxis.normalize();

				if (toaxis.dot(z) < 0) { // robot should be in approximately right orientation
					toaxis = -toaxis;
				}

				if (DEBUG_OPTION) {
					std::cout << "TOAXIS: " << toaxis.transpose() << std::endl;
				}

				Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(toaxis, z);
				rtrans.rot = q.toRotationMatrix();

				// back-calculate normal vector
				rtrans.normal = rtrans.rot.transpose() * z;
				rtrans.normal.normalize();

				centroid.y() = 0;
				rtrans.trans = centroid - rtrans.rot*centroid;

				if (DEBUG_OPTION) {
					rtrans.display();

					std::cout << "TRANSFORMED POINTS: \n";
					for (int i = 0; i < count; ++i) {
						std::cout << (rtrans.rot * points[contactPoints[i]] + rtrans.trans).transpose() << std::endl;
					}
				}
			} else {
				// rotate to rotaxisid is also on the ground
				Eigen::Vector3d fromaxis = points[contactPoints[rotAxisID]] - rotationCenter;
				Eigen::Vector3d toaxis(fromaxis); toaxis.y() = 0;
				fromaxis.normalize(); toaxis.normalize();

				Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(fromaxis, toaxis);
				rtrans.rot = q.toRotationMatrix();

				// back-calculate normal vector
				Eigen::Vector3d z = Eigen::Vector3d::Zero(3);
				z[1] = 1;
				rtrans.normal = rtrans.rot.transpose() * z;
				rtrans.normal.normalize();

				rtrans.trans = rotationCenter - rtrans.rot*rotationCenter;
			}
			return false;
		}


		// axis of rotation is line between two contacts
		rotationAxis = (points[contactPoints[rotAxisID]]-rotationCenter);
		perpDir.x() = -rotationAxis.z();
		perpDir.y() = 0;
		perpDir.z() = rotationAxis.x();
	} else {
		// axis of rotation is perpendicular to line between contact and center of mass
		perpDir = center-rotationCenter;
		perpDir.y() = 0;
		rotationAxis.x() = perpDir.z();
		rotationAxis.y() = 0;
		rotationAxis.z() = -perpDir.x();
	}
	perpDir.normalize();
	rotationAxis.normalize();

	if (DEBUG_OPTION) {
		std::cout << "rotationCenter" << rotationCenter << std::endl;
		std::cout << "rotationAxis" << rotationAxis << std::endl;
		std::cout << "perpDir" << perpDir << std::endl;
	}

	// STEP 2: perform rotation
	// angle of center of mass
	double dirRot = (center-rotationCenter).dot(perpDir);
	if (dirRot < 0) {
		perpDir = -perpDir;
	}
	int pointid = -1;
	double rotamount = -2;
	Eigen::Vector3d fromaxis;
	
	// find least amount of rotation to set a contact on the ground
	for (int i = 0; i < points.size(); ++i) {
		if (i == contactPoints[rotCenterID] || (points[i]-points[contactPoints[rotCenterID]]).norm()<0.1) {
			// skip the contact points about which rotation is happening
			continue;
		}

		if (rotAxisID >= 0 && (i == contactPoints[rotAxisID] || (points[i]-points[contactPoints[rotAxisID]]).norm()<0.1)) {
			// skip the contact points about which rotation is happening
			continue;
		}

		Eigen::Vector3d temp_fromaxis = points[i]-rotationCenter;
		Eigen::Vector3d temp_proj = (temp_fromaxis.dot(rotationAxis)) * rotationAxis;
		Eigen::Vector3d temp_perpaxis = temp_fromaxis - temp_proj;
		temp_perpaxis.normalize();
		double temp_rotamount = temp_perpaxis.dot(perpDir);

		if (temp_rotamount > rotamount) {
			pointid = i;
			rotamount = temp_rotamount;
			fromaxis = temp_perpaxis;
		}
	}

	if (pointid >= 0) {
		Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(fromaxis, perpDir);
		rtrans.rot = q.toRotationMatrix();

		if (DEBUG_OPTION) {
			std::cout << "rotation amount: " << rotamount << "\n fromaxis: " << fromaxis << std::endl;
			std::cout << "point id = " << pointid << "\n point = " << points[pointid].transpose() << std::endl;
		}

		// back-calculate normal vector
		Eigen::Vector3d z = Eigen::Vector3d::Zero(3);
		z[1] = 1;
		rtrans.normal = rtrans.rot.transpose() * z;
		rtrans.normal.normalize();

		rtrans.trans = rotationCenter - 1.0*rtrans.rot*rotationCenter;
		rtrans.trans[1] = rtrans.trans[1] - rotationCenter[1];
	} else {
		// couldn't find another contact point, return identity transform
		Eigen::Vector3d z = Eigen::Vector3d::Zero(3);
		z[1] = 1;

		rtrans.normal = z;
		rtrans.normal.normalize();

		Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors( rtrans.normal, z);
		rtrans.rot = q.toRotationMatrix();

		rtrans.trans = Eigen::Vector3d::Zero(3);
	}

	return false;

}

void KinChain::setRounds(int _rounds) {
	rounds = _rounds;
}


void KinChain::updateControllersWithInterpolation(GaitInterpInfo gaitInfo){
	if( gaitInfo.isInterp){
		updateControllersForInterpolationCase(gaitInfo.gait1, gaitInfo.gait2);

	}else{
		updateControllers(gaitInfo.gait);
	}
} 

void KinChain::updateGait(double deltaT, bool doSequence ) {
	clearGaitInfo();
	
	if(rounds >0){
	// UPDATING to calculate until steady state!!!

	// round to the nearest whole time step deltaT
	dt = 1./((int)(1./deltaT));

if(root->children.size() >0){

	clock_t begin_time_full = clock();

	std::vector<int> contactPoints;
	std::vector<int> prevContactPoints;
	std::vector<Eigen::Vector3d> prevpoints;
	std::vector<Eigen::Vector3d> points;
	KinRigidTransform fixedPointTranfn;
	KinRigidTransform rtrans;	
	//KinRigidTransform globalTransf;	
	
	Eigen::Vector3d yaxis(0,1,0);

	int animID = -1; 
	double t = 0.0;
	double steadystate = false;

	this->nrounds = 0;
	while ( ((!doSequence) &&(!steadystate && nrounds < max(3*rounds, 10))) ||
		(doSequence && (nrounds <gaitInterpolationSequence.size()))){

		if(doSequence){
			rounds = gaitInterpolationSequence.size(); 
			updateControllersWithInterpolation(gaitInterpolationSequence[nrounds]);
		}

		
		nrounds = nrounds + 1;
		//std::cout << "changing from t = " << t << " to " << nrounds-1 << std::endl;
		//std::cout << "size of anim vector is " << rootPose.size() << std::endl;
		//t = 1.0*nrounds-1;
		if (debugSteadyState) {
			std::cout << "nrounds = " << nrounds << std::endl;
		}

		for(; t<= 1.0*nrounds+deltaT/2;  t=t+deltaT){

			animID++;

			if (animID > 0) {
				rootPose.push_back(rootPose[animID-1]);
			}
		
			double sliperror = 0;

			//Step1: initialize the geometry
			this->updateTime(t);
		
			if (debugAnim) {
				//this->currentRootPose = rootPose[animID];
				std::cout << "current root pose at beginning" << std::endl;
				this->currentRootPose.display();
				//std::cout << "rootPose[animID-1] at beginning is " << std::endl;
				//rootPose[animID-1].display(); 
				std::cout << "rootPose[animID] at beginning is " << std::endl;
				rootPose[animID].display(); 
			}
			//if (t > 2.08) {
			//	std::cout << "pausing for time, t = " << t << std::endl;
			//}

			Geometry * geo = this->getGeometry(false);
		
			if(debugStability || debugAnim){
				geo->write("..\\..\\data\\test\\orig_pose.stl");
			}

			if(debugAnim){
				//geo = this->getGeometry();
				//geo->write("..\\..\\data\\test\\orig_pose2.stl");
				//rtrans.apply(geo);
				//fixedPointTranfn.apply(geo);
				//geo->write("..\\..\\data\\test\\before_pose2.stl");

				//std::cout << "global transf " << std::endl;
				//globalTransf.display();
			}
			//system("pause"); 


			//Step 2: get potential contact points
			std::vector<Eigen::Vector3d> points; points.clear();
			std::vector<bool> iswheel; iswheel.clear();
			if (USE_ONLY_CONTACT_POINTS) {
				points = getAllContactPoints(NULL, WhichContacts::LEGS);
				iswheel.insert(iswheel.end(),points.size(), false);
				std::vector<Eigen::Vector3d> pointsw = getAllContactPoints(NULL, WhichContacts::WHEELS);

				//std::cout<<points.size() << " " << pointsw.size() << std::endl;

				points.insert(points.end(), pointsw.begin(), pointsw.end());
				if (debugSlip) {
					std::cout<<points<<std::endl;
				}
				iswheel.insert(iswheel.end(), pointsw.size(), true);
				rootPose[animID].transform(points);
			} else {
				geo->addAllPoints(points);
				iswheel.insert(iswheel.end(),points.size(), false);
			}

			if (debugStability) {
				//Geometry * pointsGeo = new Geometry();
				//for (auto it = points.begin(); it != points.end(); ++it) {
				//	Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
				//	sphere->applyTrans(point((*it).x(), (*it).y(), (*it).z()));
				//	pointsGeo->add(sphere);				
				//}	
				//pointsGeo->write("..\\..\\data\\test\\points_orig.stl");
			}

			if (debugAnim) {
				std::cout<<rootPose[animID].rot << std::endl << rootPose[animID].trans << std::endl <<std::endl;
			}
		

			if (debugStability) {
				for (auto it = points.begin(); it != points.end(); ++it) {
					//std::cout<<(*it)<<std::endl<<std::endl;
				}
			}

			//Step 3: put the lowest point on the ground
			getSimpleTranformation(geo, rtrans, points);
			rtrans.apply(geo, false);	
			rtrans.transform(points);
			rootPose[animID].combine(rtrans);
			this->currentRootPose.combine(rtrans);
			if (debugSlip) {
				rootPose[animID].display();
			}

			//Step 4: rotate about center of mass until 3 points on the ground
			//Step 4a: find the the consistent contact points
			int count = getContactPoints(geo, points, contactPoints);
			if (debugStability) {
				std::cout << "# contact points = " << count << std::endl;
			}
		
			bool isstable = false;
		
			int N_iter = 0;


			if(debugStability || debugAnim){
				geo->write("..\\..\\data\\test\\intermed_poseA.stl");
				for(int i = 0; i < points.size(); i++){
					//std::cout << points[i]<< std::endl;
				}
			}
		
			CenterOfMass com = getCenterOfMass();

			// stabilize every time step
			while ((!isstable && points.size() > 2) && (N_iter <20)) { //(count < 3 || (t<deltaT && !isstable && points.size()>2) ) {
				N_iter++;
				if (debugAnim) {
					std::cout << "NITER = " << N_iter << std::endl;
				}

			//while (count < 3 ) {
				// Step 4b: rotate about center of mass
				// assumes lowest point is on the ground
				isstable = rotateToGround(geo, com, rtrans, points);
				if (debugStability) {
					std::cout << "RTRANS \n";
					rtrans.display();
				}
				rtrans.apply(geo, false);
				rtrans.transform(points);
				rootPose[animID].combine(rtrans);
				this->currentRootPose.combine(rtrans);
				if (debugSlip) {
					rootPose[animID].display();
				}
				rtrans.transform(com.center);

				if(debugStability || debugAnim){
					geo->write("..\\..\\data\\test\\intermed_poseB.stl");
					for(int i = 0; i < points.size(); i++){
						//std::cout << points[i]<< std::endl;
					}
				}

				if (debugStability) {
					if(!isstable && (N_iter >10)){
						std::cout << "not stable at t = " << t << "n_iter = " << N_iter <<  std::endl;
						std::cout << "# contact points = " << count << std::endl;
						for (int i = 0; i < count; ++i) {
							std::cout << points[contactPoints[i]].transpose() << std::endl;
						}
						std::cout << "\nCOM: " << com.center.transpose();
						std::cout << "\nRTRANS:";
						rtrans.display();
						std::cout << std::endl;
						std::cout << "\ncurrent root pose:";
						currentRootPose.display();
						std::cout << std::endl;
					}
				}

				count = getContactPoints(geo, points, contactPoints);
			}

	//		if(!isstable){
	//			system("pause"); 
	//		}

			if (debugSlip) {
				std::cout << "# contact points = " << count << std::endl;
			}

			// Step 5: compute new position for no slip
			// Step 5a: compare contact points
			std::vector<int> notMovingPointsPrev;
			std::vector<int> notMovingPointsNew;
			for(int j = 0; j < contactPoints.size(); j++){
				// TODO: COMPARE CONTACT POINT IDS!! - right now uses arbitrary threshold
				for(int i = 0; i < prevContactPoints.size() ; i++){
					if(prevContactPoints[i] == contactPoints[j]) {
						if (!iswheel[contactPoints[j]] || (prevpoints[prevContactPoints[i]]-points[contactPoints[j]]).norm() < 5) {
							notMovingPointsPrev.push_back(prevContactPoints[i]);
							notMovingPointsNew.push_back(contactPoints[j]);
						}
					}
				}
			}


			//Step 5b: compute slip error
			fixedPointTranfn.clear();
			//VLOG(3) << "not moving points = " << notMovingPointsNew.size(); 
			if(notMovingPointsPrev.size() > 0){
				Eigen::MatrixXd prevFixPointsMat(notMovingPointsPrev.size(), 3);
				Eigen::MatrixXd fixPointsMat(notMovingPointsPrev.size(), 3);
				for(int i = 0; i< notMovingPointsPrev.size(); i++){
					prevFixPointsMat(i, 0) = prevpoints[notMovingPointsPrev[i]][0]; 
					prevFixPointsMat(i, 1) = prevpoints[notMovingPointsPrev[i]][1]; 
					prevFixPointsMat(i, 2) = prevpoints[notMovingPointsPrev[i]][2]; 
					//prevFixPointsMat(i, 3) = 1;
					fixPointsMat(i, 0) = points[notMovingPointsNew[i]][0]; 
					fixPointsMat(i, 1) = points[notMovingPointsNew[i]][1]; 
					fixPointsMat(i, 2) = points[notMovingPointsNew[i]][2]; 
					//fixPointsMat(i, 3) = 1;

				}

				sliperror = getFixedPointTranformation(fixPointsMat, prevFixPointsMat, fixedPointTranfn);
				sliperror /= sqrt(((double)notMovingPointsPrev.size()));

			}else{
				//VLOG(3) << "there are no fixed points";
			}

			if (debugSlip) {
				geo->write("..\\..\\data\\test\\beforeSlip.stl");

				if (t>0.45) {
					std::cout << "t = " << t << std::endl;
				}
				std::cout << "slip = " << sliperror << std::endl;
				std::cout << "fixed point transform normal = " << fixedPointTranfn.normal.transpose() << std::endl;
				std::cout << "rot = " << fixedPointTranfn.rot << std::endl;
				std::cout << "rtrans = " << fixedPointTranfn.trans << std::endl;
				rootPose[animID].display();
			}
		
			
			fixedPointTranfn.apply(geo, false);
			fixedPointTranfn.transform(points);
		
			rootPose[animID].combine(fixedPointTranfn);
			this->currentRootPose.combine(fixedPointTranfn);

			if (debugSlip) {
				geo->write("..\\..\\data\\test\\afterSlip.stl");
				rootPose[animID].display();
			}
			fixedPointTranfn.transform(com.center);
			//Step 6: stability analysis
		
			//compute the center of mass
			//CenterOfMass com = getCenterOfMass();

			if(debugStability){
				Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
				sphere->applyTrans(point(com.center.x(), com.center.y(), com.center.z()));
				sphere->write("..\\..\\data\\test\\center_orig.stl");
			}				

			//rootPose[animID+1].transform(com.center);

			std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>  contactPoints2D;
		
			//Geometry * saveGeo = new Geometry();
			for(int i = 0; i < contactPoints.size(); i++){
				Eigen::Vector2d point2d (points[contactPoints[i]][0],points[contactPoints[i]][2]);
				//std::cout << "point2d =  [ "<< point2d.transpose() << std::endl;
				contactPoints2D.push_back(point2d);
				if(debugStability){
					//Geometry * sphere1 = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
					//sphere1->applyTrans(point(point2d(0), 0, point2d(1)));
					//std::stringstream s_filename;
					//s_filename << "..\\..\\data\\test\\cpoint" << i << ".stl";
					//sphere1->write(s_filename.str());
				}
				if(debugAnim2){
					//Geometry * contP = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
					//contP->applyTrans(points[contactPoetints[i]]);
					//saveGeo->add(contP);
				}
			}
			if(debugAnim2){
				//saveGeo->add(geo);
				//std::stringstream save_filename;
				//save_filename << "..\\..\\data\\test\\cpoint" << animID << ".stl";
				//saveGeo->write(save_filename.str());
			}

			double stability = GeometricOperations::getStabilityCost( com, contactPoints2D);

			stabilityCost.push_back(stability);
			if (debugStability) {
				std::cout << "stability = " << stability << std::endl;
			}
		
			//Step 6: update data
			//globalTransf.clear();
			prevpoints.clear();
			//geo->addAllPoints(prevpoints);
			prevContactPoints.clear();
			for(int i = 0; i < points.size(); i++){
				prevpoints.push_back(points[i]);
			}
			for(int i = 0; i < contactPoints.size(); i++){
				prevContactPoints.push_back(contactPoints[i]);
			}
		
			//anim.push_back(geo);
			slipCost.push_back(sliperror);
		
			if(debugAnim){
				std::cout << "rtrans " << std::endl;
				rootPose[animID].display();

				//VLOG(3) << "fixedPointTranfn transf  " << fixedPointTranfn.trans.transpose();
			}
			if(debugStability || debugAnim){
				geo->write("..\\..\\data\\test\\after_pose.stl");
				//system("pause"); 
			}

			if (debugAnim) {
				VLOG(3) << "done t = " << t;
				std::cout << "current root pose at end" << std::endl;
				this->currentRootPose.display();
				std::cout << "rootPose[animID]  at end" << std::endl;
				rootPose[animID].display();
			}

			delete geo;

		}


		KinRigidTransform rootposetocheck = this->currentRootPose;
		if (debugSteadyState) {
			std::cout << "CURRENT ROOT POSE\n";
			std::cout << rootposetocheck.rot << std::endl;
		}

		if(doSequence){
				steadystate = false;
				this->steadyStateRounds.first = 0;
				this->steadyStateRounds.second = gaitInterpolationSequence.size();
		}else{
			// check steady state condition
			for (int iround = 0; iround < nrounds; iround++) {
				KinRigidTransform poseAtT = this->getPoseAtT(1.0*iround);
				Eigen::Matrix3d relativeRot = rootposetocheck.rot * poseAtT.rot.transpose();
				Eigen::AngleAxisd relativeAxis(relativeRot);

				if (debugSteadyState) {
					std::cout << "ROOT POSE AT TIME " << iround << std::endl;
					std::cout << poseAtT.rot << std::endl;
					std::cout << "relative angle, axis" << std::endl;
					std::cout << relativeAxis.axis()<< std::endl;
				}

				// check rotation only about y axis
				if (abs(relativeAxis.axis().dot(yaxis)) > 0.99) {
					// is in a loop!
					this->steadyStateRounds.first = iround;
					this->steadyStateRounds.second = nrounds;
					steadystate = true;
					if (debugSteadyState) {
						std::cout << "It's a match!\n";
					}
					break;
				}
			}
		}
	}

	if (this->nrounds == 10 && this->steadyStateRounds.second == 0) {
		this->steadyStateRounds.second = 10;
	}

	//std::cout << "t = " << t << "; size of rootpose = " << rootPose.size() << std::endl;

	//outfile << "finished  in " << double( clock() - begin_time_full ) / (double)CLOCKS_PER_SEC << " seconds!" << std::endl; 

	//rootPose.erase(rootPose.begin());
	
} else{
	steadyStateRounds.first = 0;
	steadyStateRounds.second = 0;
	int animID = -1; 
	for(double t = 0; t<= 1.0;  t=t+deltaT){
		animID++;

		rootPose.push_back(rootPose[animID]);
	}
}
}
}

std::vector<double> KinChain::getTimeStamps() {
	std::vector<double> alltimes;

	set<double> temptimes;

	for (auto it = nodes.begin(); it != nodes.end(); ++it) {
		if ((*it)->getType() != KinNode::JOINT) {continue;}

		std::vector<SymbolicTransformation*> atrans = ((KinNode_Joint*)(*it))->getArticulation()->transformations;
		for (std::vector<SymbolicTransformation*>::iterator transit = atrans.begin(); transit != atrans.end(); ++transit) {
			if ((*transit)->controller->isSymbolic() ) {
				SymbolicController* sc = (SymbolicController*)((*transit)->controller);
				std::list<double> sctimes = sc->getElementVals()->getTimeStamps();
				temptimes.insert(sctimes.begin(), sctimes.end());
			}
		}
	}
	
	auto firstit = temptimes.begin();
	auto lastit = temptimes.end(); --lastit;

	// remove negative numbers
	while ((*firstit) < 0) {++firstit;}
	while ((*lastit) > 1) {--lastit;}
	++lastit;
	
	alltimes.assign( firstit, lastit );

	return alltimes;
}


std::vector<int> KinChain::getInterpGaitSequenceForDrawingPath(){
	std::vector<int> result;
	for (int i = 0; i < gaitInterpolationSequence.size(); i++){
		if(gaitInterpolationSequence[i].isInterp){
			result.push_back(gaitInterpolationSequence[i].gait1);
		}else{
			result.push_back(gaitInterpolationSequence[i].gait); 
		}
	}
return result; 


}
std::vector<std::pair<double,double>> KinChain::getSSPath(bool doSequence) {
	std::vector<std::pair<double,double>>  path;
	std::pair<int, int> ssr = this->getSteadyStateRounds();


	int animatedRounds = this->rounds;
	if(doSequence){
		animatedRounds = gaitInterpolationSequence.size();
	}

	KinRigidTransform t0 = this->getPoseAtT(0);
	KinRigidTransform tfirst = this->getPoseAtT(ssr.first);

	for(double t = 1.0*ssr.first; t<= 1.0*ssr.first+(animatedRounds);  t = t+dt){
		this->updateTime(t);
		Eigen::Vector3d center = this->getCenterOfMass().center;

		center = tfirst.rot.transpose()*(center-tfirst.trans);
		t0.transform(center);

		path.push_back(std::pair<double,double>(center.x(), center.z()));
	}
	
	return path;
}
std::pair<double, int> KinChain::adjustTimeToSS(double t) {
	std::pair<int, int> ssr = this->getSteadyStateRounds();
	int mod = 0;
	if (ssr.second > 0 && t > ssr.second) {
		t -= ssr.second; ++mod;
		int diff = ssr.second - ssr.first;
		while (t > diff) {
			t -= diff;
			++mod;
		}
		t += ssr.first;
	}

	return std::pair<double, int>(t, mod);
}

std::vector<std::pair<double,double>> KinChain::getPath() {
	std::vector<std::pair<double,double>>  path;

	for(double t = 0; t<= 1.0*rounds;  t = t+dt){
		this->updateTime(t);
		//Eigen::Vector3d center = this->getCenterOfMass().center;
		//Geometry * geo = this->getGeometry();
		Eigen::VectorXd center = this->getCenterOfMass().center;
		path.push_back(std::pair<double,double>(center.x(), center.z()));
		//if(DEBUG_PATH){
			//std::stringstream filename_geo;
			//std::stringstream filename_center;
			//std::stringstream filename_floor;
			//filename_geo << "..\\..\\data\\test\\geo_anim_" << t*1000 << ".stl";
			//filename_center << "..\\..\\data\\test\\center_anim_" << t*1000 << ".stl";
			//filename_floor << "..\\..\\data\\test\\floor_anim_" << t*1000 << ".stl";
			//Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
			//Geometry * sphere2 = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
			//sphere->applyTrans(center);
			//sphere2->applyTrans(point(center[0], 0, center[2]));
			//sphere->write(filename_center.str());
			//sphere2->write(filename_floor.str());
			//geo->write(filename_geo.str());
		//}
		//delete geo;
	}
	
	return path;
}

double KinChain::getPerpendicularError() {
	if(root->children.size() <= 0){ return 0; }

	double worstCaseStability = -std::numeric_limits<double>::max();
	std::pair<int, int> ssr = getSteadyStateRounds();

	if (ssr.first == ssr.second) { return 0; }

	Eigen::Vector2d CoR = this->getCenterOfRotation();
	double R = 100./this->getMeanCurvature();
	double perperror = 0;

	for(double t = 1.0*ssr.first; t<= 1.0*ssr.second;  t = t+dt){
		this->updateTime(t);
		Eigen::Vector3d center = this->getCenterOfMass().center;
		Eigen::Vector2d projcenter(center.x(),center.z());

		double error = (projcenter-CoR).norm() - R;

		perperror += error*error; // add perpendicular distance
	}
	
	return sqrt(perperror/(ssr.second-ssr.first));
}

/*
std::vector<Geometry*> KinChain::getAnimation( bool useSState, int ratio, bool doSequence) {
	std::vector<Geometry*> anim;
	double t;
	double ttoadd = (useSState) ? this->getSteadyStateRounds().first : 0;
	for(t = 0.0; t<= 1.0*this->rounds +dt*((double)ratio)/2.;  t = t+dt*(double)(ratio)){
		if(doSequence){
			//updateControllers(gaitSequence[(int)floor(t)]); 
			int cycle = (int)floor(t - dt/2);
			if(cycle < 0){
				cycle = 0;
			}
			updateControllersWithInterpolation(gaitInterpolationSequence[cycle]);
		}
		this->updateTime(t + ttoadd);
		Geometry * geo = this->getGeometry();
		anim.push_back(geo);
	}
	//std::cout << "time t = " << t << "; size of anim = " << anim.size();
	
	return anim;
}
*/

std::vector<Geometry*> KinChain::getAnimation( bool useSState, int ratio, bool doSequence) {
	std::vector<Geometry*> anim;
	double t;
	double ttoadd = (useSState) ? this->getSteadyStateRounds().first : 0;
	std::vector<int> framesPerCycle = this->getFramesPerCycle(doSequence);
	if (debugAnimInterp) {
		for each ( auto s in framesPerCycle){
			std::cout << "----------------- frames per cycle = " << s << std::endl;
		}
	}


	double dt_cycle = 1.0;
	if(framesPerCycle[0] >0 ) dt_cycle =  1.0/framesPerCycle[0];
	for(t = 0.0; t<= 1.0*this->rounds +dt_cycle*((double)ratio)/2.;  t = t+dt_cycle*(double)(ratio)){
		int cycle = (int)floor(t - dt_cycle/2);
		if(cycle < 0){
				cycle = 0;
		}
		if(doSequence){
			//updateControllers(gaitSequence[(int)floor(t)]); 
			updateControllersWithInterpolation(gaitInterpolationSequence[cycle]);
		}
		this->updateTime(t + ttoadd);
		Geometry * geo = this->getGeometry();
		anim.push_back(geo);
		

		int cycle_forFrame = (int)floor(t +0.0000001);
		if(cycle_forFrame > (framesPerCycle.size() -1)){
			cycle_forFrame = framesPerCycle.size() -1;
		}
		dt_cycle = 1.0;
		if(framesPerCycle[cycle_forFrame] >0 ) dt_cycle =  1.0/framesPerCycle[cycle_forFrame];
		if (debugAnimInterp) {
			std::cout << "t = " << t << "cycle = " << cycle  << "cycle_forFrame " << cycle_forFrame << std::endl;
		}
	}
	if (debugAnimInterp) {
		std::cout << " total anim Size = " << anim.size() << std::endl;
	}
	
	return anim;
}


std::vector<double> KinChain::getTimesPerCycle(bool doSequence){
	std::vector<double> timesPerCycle;
	bool hackToMakeZeroTimeDoNothing = false;
	for(int t= 0; t<this->rounds; t++){
		if(doSequence){
			updateControllersWithInterpolation(gaitInterpolationSequence[t]);
			hackToMakeZeroTimeDoNothing = false;
			if ((gaitInterpolationSequence[t].isInterp == false) && (gaitInterpolationSequence[t].gait == -1)){
				hackToMakeZeroTimeDoNothing = true;
			}
		}
		double time = getTimeOfOneCycle();
		if(hackToMakeZeroTimeDoNothing){
			time = 0.5;
		}
		timesPerCycle.push_back((time)); 
	}
	return timesPerCycle;
}

std::vector<int> KinChain::getFramesPerCycle(bool doSequence){
	std::vector<double> timesPerCycle = getTimesPerCycle(doSequence);
	std::vector<int> framesPerCycle;
	for(int t= 0; t< timesPerCycle.size(); t++){
		int frames = (int) round(timesPerCycle[t]*100); // assuming 60 frames per second
		framesPerCycle.push_back((frames)); 
	}
	return framesPerCycle;
}

int KinChain::totalDegreesOfFreedom() {
	int ndof = 0;

	for (auto nodeit = nodes.begin(); nodeit!=nodes.end(); ++nodeit) {
		if ((*nodeit)->getType() == KinNode::JOINT) {
			ndof += ((KinNode_Joint*)(*nodeit))->getArticulation()->transformations.size();
		}
	}

	return ndof;
}

void KinChain::collectControlThetas(std::vector<double>& allthetas, std::vector<bool>& iscontact, std::vector<bool>& ismoving, double t)
{
	// do a DFS
	root->collectControlThetas(allthetas, iscontact, ismoving, t);

	return;
}

void KinChain::broadcastControlThetas(std::list<double>& allthetas, double t)
{
	// do a DFS
	root->broadcastControlThetas(allthetas, t);

	return;
}


//------------------------------------------------------------------------------------------------


struct GaitConfig{
	Eigen::VectorXd assignment;
	double speed;
	double stability;
	std::vector<double> configIdPerRun;
};

bool gaitSpeedStabilityCompare (GaitConfig i,GaitConfig j) { 
	if(i.stability == j.stability){
		return(i.speed < j.speed);
	}
	return (i.stability<j.stability); 
}

struct gait_compare : public std::unary_function<GaitConfig, bool>
{
  explicit gait_compare(const GaitConfig &baseline) : baseline(baseline) {}
  bool operator() (const GaitConfig &arg)
  { 
	bool isEqual = true;
	for(int i = 0; i < arg.configIdPerRun.size(); i++){
		if( arg.configIdPerRun[i] != baseline.configIdPerRun[i]){
			isEqual = false;
		}
	}
	return isEqual;
  
  }
  GaitConfig baseline;
};





Eigen::VectorXd convertFromBinary(int i, int N_size){
	Eigen::VectorXd conn = Eigen::VectorXd::Zero(N_size);
	int num = i;
	for(unsigned j= 0; j < N_size; j++){
		 double r = num%2;
		 num /= 2;
		 conn(j) = r;
	}

	return conn;
}


std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> KinChain::getJointPositiont(){


	std::vector<KinNode*> primaryLegJoints = root->children;
	Eigen::VectorXd wheels = Eigen::VectorXd::Zero(primaryLegJoints.size()); // is 1 if this limb is a wheel
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>  limbPositions(primaryLegJoints.size());
	for (unsigned i = 0; i < primaryLegJoints.size(); i++){
		KinNode_Joint* joint = dynamic_cast<KinNode_Joint*>(primaryLegJoints[i]);
		if(joint == nullptr){
			LOG(ERROR) << "the root points to a part" << std::endl;
		}
		limbPositions[i] = Eigen::Vector2d(joint->getArticulation()->getCenter().x(),joint->getArticulation()->getCenter().z());
	
	}

	return limbPositions;

}


void KinChain::computeGaitOptions(){
	gaitOptions.gaitOptions.clear();
	gaitOptions.legPositions.clear();
	gaitOptions.legPositions.clear();
	std::vector<KinNode*> primaryLegJoints = root->children;
	Eigen::VectorXd wheels = Eigen::VectorXd::Zero(primaryLegJoints.size()); // is 1 if this limb is a wheel
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>  limbPositions(primaryLegJoints.size());
	std::vector<int> legOrientations;
	for (unsigned i = 0; i < primaryLegJoints.size(); i++){
		KinNode_Joint* joint = dynamic_cast<KinNode_Joint*>(primaryLegJoints[i]);
		if(joint == nullptr){
			LOG(ERROR) << "the root points to a part" << std::endl;
		}
		auto part = dynamic_cast<KinNode_Part*>(joint->children[0]);
		GrammarController * gController = dynamic_cast<GrammarController*>(joint->getArticulation()->transformations[0]->controller);
		if(gController->getType() == GrammarController::GrammarcontrollerType::WHEEL){
			wheels(i) = 1;
		}
		joint->getArticulation()->updateParameters(SymbolicAssignment::USE_CURRENT_VALUES);
		limbPositions[i] = Eigen::Vector2d(joint->getArticulation()->getCenter().x(),joint->getArticulation()->getCenter().z());
	

		legOrientations.push_back(gController->getMulti());
				

	}

	gaitOptions.legPositions = limbPositions;
	// compute the bounding box for draweing
	auto rootPart = (dynamic_cast<KinNode_Part*>(root));
	TriMesh::BBox bbox;
	for each ( auto e in rootPart->elements){
		bbox = bbox + e->evalBoundingBox();
	}
	gaitOptions.polygon.push_back(Eigen::Vector2d(bbox.min[0], bbox.min[2]));
	gaitOptions.polygon.push_back(Eigen::Vector2d(bbox.min[0], bbox.max[2]));
	gaitOptions.polygon.push_back(Eigen::Vector2d(bbox.max[0], bbox.max[2]));
	gaitOptions.polygon.push_back(Eigen::Vector2d(bbox.max[0], bbox.min[2]));


	CenterOfMass center  = getCenterOfMass();

	int Nlimbs = limbPositions.size();


	//Step1: set which configurations are valid and what is the stability error for each of them
	//std::vector<std::pair<Eigen::VectorXd, double>> allowedMovingConfigs;
	Eigen::VectorXd onesVec = Eigen::VectorXd::Ones(Nlimbs);
	std::vector<std::pair<bool, double>> configurationValidation(1<<(Nlimbs));
	for(unsigned i = 0; i < (1<<(Nlimbs)) ; i++){
		configurationValidation[i] = std::pair<bool, double>(0,0);

		bool isValid = true; 
		Eigen::VectorXd configuration = Eigen::VectorXd::Zero(Nlimbs);
		for(unsigned j= 0; j < (Nlimbs); j++){
			if((i& (1<<j)) != 0){
				configuration(j) = 1;  // 1 if the leg is moving 0 if it is on the ground
			}
		}
		//std::cout << "i = " << i << "configuration " << configuration.transpose() << std::endl;
		//check if one of the wheels are off - this is invalid
		if(((1.0)*wheels).dot(configuration) >0){
			isValid = false;
		}
		//check if there are less than 3 points on the ground - this is invalid 
		if((onesVec - configuration).sum() <3){
			isValid = false;
		}
		if((configuration).sum() <1){
			isValid = false;
		}
		if(isValid){
			std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>  auxContactPoints;
			for(unsigned c = 0; c < limbPositions.size(); c++){
				if(!configuration(c)){
					auxContactPoints.push_back(limbPositions[c]);
				}
			}
			//for each ( auto c in auxContactPoints){
			//	std::cout << "auxContactPoints = " << c.transpose() << std::endl;
			//}
			//std::cout << "center = " << center.center.transpose() << std::endl;
			double error = GeometricOperations::getStabilityCost(center, auxContactPoints);
			//std::cout << "found stability error = " << error << std::endl;
			if(error < 20){
				if (error < 0){
					error = 0;
				}
				configurationValidation[i].first = 1;
				configurationValidation[i].second = error;
				//allowedMovingConfigs.push_back(std::pair<Eigen::VectorXd, double>(configuration,error));
			}
			auxContactPoints.clear();
		}
	}

	if(DEBUG_OPTION){
		std::cout << "Wheels:" << wheels.transpose() << std::endl;
		std::cout << "Allowed Configurations:" << std::endl;
		for(int i = 0; i < configurationValidation.size(); i++){
			if(configurationValidation[i].first){
				std::cout << "cofiguration " << i << " - stab = " << configurationValidation[i].second << std::endl;
			}
		}
	}

	//int N_configurations = allowedMovingConfigs.size();
	//Eigen::MatrixXd configMat(N_configurations, Nlimbs);
	//for(int i = 0; i < allowedMovingConfigs.size(); i++){
	//	configMat.row(i) = allowedMovingConfigs[i].first.transpose();
	//}

	//std::cout << "configMat = " << configMat << std::endl;
	int nRuns = 5;
	//Step2: loop over all possible assingments and check if they are ok 

	std::vector<GaitConfig> possibleGaits;
	std::vector<double> zeroVecSTD = std::vector<double>(nRuns-1);
	for (int i = 0; i< zeroVecSTD.size(); i++){zeroVecSTD[i] =0;}
	for(unsigned i = 0; i <pow((double) nRuns,(double)(Nlimbs)) ; i++){
		GaitConfig gait;
		bool isAllowed = true;
		Eigen::VectorXd conn = Eigen::VectorXd::Zero(Nlimbs);
		int num = i;
		gait.configIdPerRun = zeroVecSTD;
		for(unsigned j= 0; j < Nlimbs; j++){
			 int r = num%nRuns;
			 num /= nRuns;
			 conn(j) = r;
			 // leg j in on on time = r
			 if(r >0){
				 gait.configIdPerRun[r-1] = gait.configIdPerRun[r-1] + pow(2.0, (double)j);
			 }
			 if((r == 0) && !wheels(j)){
				 isAllowed = false; 
			 }
		}

		//std::cout <<"gait.configIdPerRun = ";
		//for each (auto a in gait.configIdPerRun)
		//	std::cout << a << ", ";
		//std::cout << std::endl;


		if(isAllowed){
			// check if each run is allowed. 			
			//check if it does not already enough
			//if(true){
			//make sure that all wheels move
			//double error = (conn.dot(notWheels)
 			bool hasReachedTheLast = false;
			gait.stability = 0;
			gait.speed = nRuns-1;
			gait.assignment = conn; 
			for( unsigned r = 0; r < nRuns-1; r++){
				gait.stability = gait.stability + configurationValidation[(int)gait.configIdPerRun[r]].second;
				if(hasReachedTheLast && (gait.configIdPerRun[r] !=0)){
					isAllowed = false;
				}
				if((gait.configIdPerRun[r] ==0)){
					if((!hasReachedTheLast)){
						gait.speed = r;
					}
					hasReachedTheLast = true;
				}else{
					if(!configurationValidation[(int)gait.configIdPerRun[r]].first){
						isAllowed = false;
					}
				}
			}

		//	std::cout << "is allowed" << std::endl;
		//	std::cout << "for i = " << i << "gait = " << conn.transpose() << "speed = " << gait.speed << "configIdPerRun = " ; // << std::endl;
		//	for(int b = 0; b< gait.configIdPerRun.size(); b++){ std::cout << gait.configIdPerRun[b] <<  " ";}

		}	
		
			
		if(isAllowed){


			std::sort(gait.configIdPerRun.begin(), gait.configIdPerRun.end());
			//std::cout << "sorted  = " ; // << std::endl;
			//for(int b = 0; b< gait.configIdPerRun.size(); b++){ std::cout << gait.configIdPerRun[b] <<  " ";}
			//std::cout << std::endl;
			
			if(possibleGaits.size() ==0){
				//std::cout << "------------is empty so is adding " << std::endl;
				possibleGaits.push_back(gait);
			}else{ 
				if(std::find_if(possibleGaits.begin(), possibleGaits.end(), gait_compare(gait)) == possibleGaits.end()){
					//std::cout << "------------did not find is adding " << std::endl;
					possibleGaits.push_back(gait);			
				}
			}
		}


			/*if ((conn.transpose()*configMat).sum() == Nlimbs){
			Eigen::VectorXd sumtoOne = wheels;
			GaitConfig gait;
			gait.assignment = Eigen::VectorXd::Zero(sumtoOne.size());
			gait.speed = 0;
			gait.stability = 0;
			for(unsigned j= 0; j < (N_configurations); j++){
				if(conn(j)!=0){
					sumtoOne = sumtoOne + allowedMovingConfigs[j].first;
					gait.speed = gait.speed+1;
					gait.stability = gait.stability + allowedMovingConfigs[j].second;
					gait.assignment = gait.assignment + gait.speed*allowedMovingConfigs[j].first;
				}
			}
			//std::cout << "sumtoOne " << sumtoOne.transpose() << std::endl;
			//std::cout << "(onesVec -sumtoOne).norm() " << (onesVec -sumtoOne).norm() << std::endl;
			if((onesVec -sumtoOne).norm() <=0.001){
				possibleGaits.push_back(gait);
				std::cout << "found a gait " << gait.assignment.transpose() << " - speed = " << gait.speed << " - stab = " << gait.stability << std::endl;

			}
		}
		*/
	}





	//Step 3: sort for stability and speed
	std::sort(possibleGaits.begin(), possibleGaits.end(), gaitSpeedStabilityCompare);

	if(DEBUG_OPTION){
		std::cout << "Found Gaits: " << possibleGaits.size() <<  std::endl;
		for(int i = 0; i < possibleGaits.size(); i++){
			std::cout << "cofiguration " << possibleGaits[i].assignment.transpose()  << " - speed = " << possibleGaits[i].speed << " - stab = " << possibleGaits[i].stability << std::endl;
		}
	}



	// create the 3 default gait options:
	std::vector<std::pair<int, double> >simpleGait_straight;
	for(int i = 0; i< Nlimbs; i++){
		if(wheels(i)){
			simpleGait_straight.push_back(std::pair<int, double>(0, 60.));
		}else{
			simpleGait_straight.push_back(std::pair<int, double>(1, 60.));
		}
	}
	gaitOptions.gaitOptions.push_back(simpleGait_straight);
	std::vector<int> sides;
	bool hasLeft = false;
	bool hasRight = false;
	for(int i = 0; i< Nlimbs; i++){
		if(legOrientations[i] < 0){
			hasLeft = true;
		}
		if(legOrientations[i] > 0){
			hasRight = true;
		}
	}
	if(hasLeft && hasRight){
		std::vector<std::pair<int, double> >simpleGait_right;
		std::vector<std::pair<int, double> >simpleGait_left;
		for(int i = 0; i< Nlimbs; i++){
			if(wheels(i)){
				simpleGait_right.push_back(std::pair<int, double>(0, legOrientations[i]*30.));
				simpleGait_left.push_back(std::pair<int, double>(0, -1.*legOrientations[i]*30.));
			}else{
				simpleGait_right.push_back(std::pair<int, double>(1, legOrientations[i]*30.));
				simpleGait_left.push_back(std::pair<int, double>(1, -1.*legOrientations[i]*30.));
			}
		}

		gaitOptions.gaitOptions.push_back(simpleGait_right);
		gaitOptions.gaitOptions.push_back(simpleGait_left);

	}
		
	
	


	// add the stable gait options;
	if(possibleGaits.size() == 0){
		std::vector<std::pair<int, double> >defautl_gait;
		int count = 1;
		for(int i = 0; i< Nlimbs; i++){
			if(wheels(i)){
				defautl_gait.push_back(std::pair<int, double>(0, 30.));
			}else{
				defautl_gait.push_back(std::pair<int, double>(count, 30.));
				count++;
			}
		}
		gaitOptions.gaitOptions.push_back(defautl_gait);
	}
	for(int i = 0; i < possibleGaits.size(); i++){
		std::vector<std::pair<int, double>  > gaitOption(possibleGaits[i].assignment.size());
		for(int j = 0; j <gaitOption.size(); j++){
			//if(wheels.sum() ==0){
			//	gaitOption[j] = possibleGaits[i].assignment(j) - 1;
			//}else{
				gaitOption[j].first = possibleGaits[i].assignment(j);
				gaitOption[j].second =  30; 
			//}
		}
		gaitOptions.gaitOptions.push_back(gaitOption);
	}

	//createNewGaitOptionFromSuggestion(0);
	//selectedGaitOption = defautl_gait; 
	

}



void KinChain::updateSequence(std::vector<int> _sequence){ 
		gaitSequence.clear(); 
		gaitInterpolationSequence.clear();
		for (int i = 0 ; i < _sequence.size(); i++) {
			gaitSequence.push_back(_sequence[i]); 
			
			if(i >0){
				if(_sequence[i]!= _sequence[i-1]){
					GaitInterpInfo gaitInterp(_sequence[i-1], _sequence[i]);
					gaitInterpolationSequence.push_back(gaitInterp);
				}
			}
			GaitInterpInfo gaitInterp(_sequence[i]);
			//std::cout << "squence i [" << i  <<"] = "<< _sequence[i] << std::endl;
			gaitInterpolationSequence.push_back(gaitInterp);

		}
}

void KinChain::createNewGaitOptionFromSuggestion(int id){ // theta is given in degrees


	std::stringstream name;
	name << "gait " << savedGaits.size();
	GaitInfo* gaitInfo = new GaitInfo();
	gaitInfo->name = name.str();
	gaitInfo->desiredDirection = 0;

	std::vector<std::pair<int, double>> newGaitOption(gaitOptions.gaitOptions[id].size());
	for( int i = 0; i < gaitOptions.gaitOptions[id].size(); i++){
		std::pair<int, double> jointInfo;
		jointInfo.first = gaitOptions.gaitOptions[id][i].first;
		jointInfo.second = gaitOptions.gaitOptions[id][i].second;
		gaitInfo->jointInfo.push_back(jointInfo);

	}


	savedGaits.push_back(gaitInfo); 
}



void getAllControlls(KinNode* node, std::vector<Controller*>& controllers){

	auto jointNode = dynamic_cast<KinNode_Joint*>(node);
	if(dynamic_cast<KinNode_Joint*>(node) != nullptr){
		Controller* controller = (dynamic_cast<KinNode_Joint*>(node))->getArticulation()->transformations[0]->controller;
		controllers.push_back(controller);	
	}
	for each (auto n in node->children){
		getAllControlls(n, controllers);
	}


}

static bool jointInfoCompare(std::pair<int, double> a, std::pair<int, double> b)
{
	return (a.first< b.first);
}


void KinChain::updateControllers(int gaitID){ // theta is given in degrees

	currentGaitID = gaitID;

	if(savedGaits.size() >0){ // there are saved gaits
		if(savedGaits[0]->jointInfo.size() > 0){ // there are multiple joits.
	
			std::vector<KinNode*> primaryLegJoints = root->children;

			if((savedGaits.size() >gaitID) && (gaitID >= 0 )){
						int Nlabels = (std::max_element(savedGaits[gaitID]->jointInfo.begin(), savedGaits[gaitID]->jointInfo.end(), jointInfoCompare))->first;
					for (unsigned i = 0; i < primaryLegJoints.size(); i++){
						double multi = dynamic_cast<GrammarController*>((dynamic_cast<KinNode_Joint*>(primaryLegJoints[i]))->getArticulation()->transformations[0]->controller)->getMulti();
						std::vector<Controller*> controllers;
						getAllControlls(primaryLegJoints[i], controllers); 
						for each( auto c in controllers){
							(dynamic_cast<GrammarController*>(c))->updateParameters(savedGaits[gaitID]->jointInfo[i].second, Nlabels, savedGaits[gaitID]->jointInfo[i].first -1, multi);
						}
					}
				}
			else{

					for (unsigned i = 0; i < primaryLegJoints.size(); i++){
						double multi = dynamic_cast<GrammarController*>((dynamic_cast<KinNode_Joint*>(primaryLegJoints[i]))->getArticulation()->transformations[0]->controller)->getMulti();
						std::vector<Controller*> controllers;
						getAllControlls(primaryLegJoints[i], controllers); 
						for each( auto c in controllers){
							(dynamic_cast<GrammarController*>(c))->updateParameters(0, 1, 0, multi);
						}

					}

			}
		}
	}
}


void KinChain::updateControllersForInterpolationCase(int gaitID1, int gaitID2){ // theta is given in degrees
	currentGaitID = -100;

	if(savedGaits.size() >0){ // there are saved gaits
		if(savedGaits[0]->jointInfo.size() > 0){ // there are multiple joits.
	
			std::vector<KinNode*> primaryLegJoints = root->children;
			for (unsigned i = 0; i < primaryLegJoints.size(); i++){
				double multi = dynamic_cast<GrammarController*>((dynamic_cast<KinNode_Joint*>(primaryLegJoints[i]))->getArticulation()->transformations[0]->controller)->getMulti();
				std::vector<Controller*> controllers;
				getAllControlls(primaryLegJoints[i], controllers); 
				double theta1 = 0;
				double theta2 = 0;
				if(gaitID1 >=0)
					theta1 = savedGaits[gaitID1]->jointInfo[i].second;
				if(gaitID2 >= 0)
					theta2 = savedGaits[gaitID2]->jointInfo[i].second;
				for each( auto c in controllers){
					(dynamic_cast<GrammarController*>(c))->updateParametersForInterpolation(theta1, theta2);
				}

			}

		}
	}
}

void KinChain::outputMotionToFile(std::string filepath){

	//"..\\..\\data\\test\\stabilityUI.txt"
	ofstream myfile;
	std::string motionFilename = filepath+"motion.txt";

	myfile.open(motionFilename);

	int count = 0; 
	for each( auto n in nodes){
		if( n->getType() == KinNode::KinNodeType::JOINT){
			KinNode_Joint * joint = dynamic_cast<KinNode_Joint*>(n);
			GrammarController* controller = dynamic_cast<GrammarController*>(joint->getArticulation()->transformations[0]->controller);
			std::string motionInfo = controller->getLinearController()->getMotionInfo();
			myfile << "Motion information for joint " << count << std::endl;  
			myfile << motionInfo << "\n";
			auto center = joint->getArticulation()->getCenter();
			Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
			std::stringstream posFileName;
			posFileName << filepath << "joint" << count << ".stl";
			sphere->applyTrans(point(center.x(), center.y(), center.z()));
			sphere->write(posFileName.str());
			count++; 
		}
	}
	myfile.close();
	std::string geoFilename= filepath+"fullgeo.stl";
		
	this->getGeometry()->write(geoFilename);

}

void KinChain::updateGaitSingleAngle(int gaitId, int i, double theta){
	savedGaits[gaitId]->jointInfo[i].second = theta;
}
std::vector<std::string> KinChain::getListofSavedGaits(){ 
	
	std::vector<std::string> names; 
	for each ( auto g in savedGaits){
		names.push_back(g->name);
	}

	return names;
}

void KinChain::updateGaitSingleJoint(int gaitID, int ind , bool isup){
	
	if (savedGaits[gaitID]->jointInfo[ind].first !=0){
		int add = 1;
		if(!isup)
			add =-1;
		savedGaits[gaitID]->jointInfo[ind].first = savedGaits[gaitID]->jointInfo[ind].first+add;
		if(savedGaits[gaitID]->jointInfo[ind].first <1){
			savedGaits[gaitID]->jointInfo[ind].first =1;
		}

		int Nlabels = (std::max_element(savedGaits[gaitID]->jointInfo.begin(), savedGaits[gaitID]->jointInfo.end(), jointInfoCompare))->first;
		for (int labelid = 1; labelid < Nlabels; labelid++){
			bool isUsed = false;
			for each ( auto labelInfo in savedGaits[gaitID]->jointInfo){
				if(labelInfo.first == labelid){
					isUsed = true;
				}
			}
			if(!isUsed){
				for (int i = 0; i < savedGaits[gaitID]->jointInfo.size(); i++){
					if(savedGaits[gaitID]->jointInfo[i].first> labelid){
						savedGaits[gaitID]->jointInfo[i].first = savedGaits[gaitID]->jointInfo[i].first-1;
					}
				}
			}

		}

	}
}
