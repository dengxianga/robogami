#include "KinChain.h"
#include "DrivingJointSelection.h"
#define PRINT_DEBUG false

using namespace FabByExample;


//------------------------- KinDriverAbstraction---------------------------------------------------------

std::vector<KinDriverAbstraction*> KinDriverAbstraction::getFirstNeighbours(){
	return connectedNodes;
}
std::vector<KinDriverAbstraction*> KinDriverAbstraction::getSecondNeighbours(){
	std::vector<KinDriverAbstraction*> secondNeighbours;
	for each ( auto neighbour in connectedNodes){
		for each (auto secondNeighbour in neighbour->getFirstNeighbours()){
			if (secondNeighbour != this){
				secondNeighbours.push_back(secondNeighbour);
			}
		}
	}

	return secondNeighbours;
}

//------------------------- PART---------------------------------------------------------



KinDriverAbstraction_Part::KinDriverAbstraction_Part(KinNode_Part * _node, int _id): KinDriverAbstraction(_id){

	node = _node;
	isDetermined_position = false;

}

bool KinDriverAbstraction_Part::isFullyDetermined(){
	return isDetermined_position;

}

bool KinDriverAbstraction_Part::discoverInformation(){
	
	if (isDetermined_position)
		return false;

	// case1: 2 adjacentet joints have position
	int N_AdjacentDeteminedJoints = 0;
	for each(auto neighbour in this->getFirstNeighbours() ){
		if(neighbour->iAmAJointAndHavePosition()){
			N_AdjacentDeteminedJoints++;
		}
	}
	if(N_AdjacentDeteminedJoints >=2){
		isDetermined_position = true;
		return true;
	}

	// case1: adjacentet joint has angle and it's neighbour has position
	int N_AdjacentDeteminedJointsWithPart = 0;
	for each(auto jointNeighbour in this->getFirstNeighbours() ){
		if(jointNeighbour->iAmAJointAndHaveAngle()){
			bool hasAPosition = false;
			for each (auto partNeighbour in jointNeighbour->getFirstNeighbours()){
				if(partNeighbour->iAmAPartAndHavePosition() &&(partNeighbour != this)){
					hasAPosition = true;
				}
			}
			if(hasAPosition){
				N_AdjacentDeteminedJointsWithPart++;
			}
		}
	}
	if(N_AdjacentDeteminedJointsWithPart >=1){
		isDetermined_position = true;
		return true;
	}

	return false;

}

void KinDriverAbstraction_Part::printConnections(){
	std::cout << "PART  ["<< id  <<  "] -> ";
	for each ( auto c in connectedNodes){
		std::cout << c->getId() << "  ";
	}
	std::cout << std::endl;
}
void KinDriverAbstraction_Part:: printInfo(){
	std::cout << "PART  ["<< id  <<  "] -> ";
	if(isDetermined_position){
		std::cout << "__POS__  ";
	}else{
		std::cout << "_______  ";
	}


	std::cout << std::endl;
		

}



//------------------------- Joint---------------------------------------------------------



KinDriverAbstraction_Joint::KinDriverAbstraction_Joint(KinNode_Joint * _node, bool _canBeSetToDriving, int _id): KinDriverAbstraction(_id){

	node = _node;
	canBeSetToDriving = _canBeSetToDriving;
	isDetermined_position = false;
	isDetermined_angle = false;

}

bool KinDriverAbstraction_Joint::isFullyDetermined(){
	return (isDetermined_position && isDetermined_angle);

}

bool KinDriverAbstraction_Joint::discoverInformation(){
	
	bool hasDiscoveredSomething = false;

	if (isDetermined_position && isDetermined_angle)
		return false;

	
	//Step1: look at neighbours
	int N_AdjacentDeteminedPositions = 0;
	for each(auto neighbour in this->getFirstNeighbours() ){
		if(neighbour->iAmAPartAndHavePosition()){
			N_AdjacentDeteminedPositions++;
		}
	}
	// case1: Position from one adjacent Part
	if(!isDetermined_position &&(N_AdjacentDeteminedPositions >=1)){
		hasDiscoveredSomething = true;
		isDetermined_position = true;
	}
	// case2: Position from 2 adjacent Part
	if(!isDetermined_angle &&(N_AdjacentDeteminedPositions >=2)){
		hasDiscoveredSomething = true;
		isDetermined_angle = true;
	}

	//Step2: look at neighbours
	int N_SecondAdjacentDeteminedPositions = 0;
	for each(auto partNeighbour in this->getFirstNeighbours()){
		bool neighbourPartHasAPositionedAngle = false;
		for each ( auto joinNeighbour in partNeighbour->getFirstNeighbours()){
			if((joinNeighbour != this) && (joinNeighbour->iAmAJointAndHavePosition())){
				neighbourPartHasAPositionedAngle = true;
			}
		}
		if(neighbourPartHasAPositionedAngle){
			N_SecondAdjacentDeteminedPositions++;
		}
	}
	if(!this->isFullyDetermined()  && (N_SecondAdjacentDeteminedPositions >=2)){
		hasDiscoveredSomething = true;
		isDetermined_position = true;
		isDetermined_angle = true;
	}


	return hasDiscoveredSomething;

}

bool KinDriverAbstraction_Joint::makeMeADriver(){

	if(!isDetermined_angle || (canBeSetToDriving)){
		isDriving = true;
		isDetermined_angle = true;
		return true;
	}

	return false;
}

void KinDriverAbstraction_Joint::printConnections(){
	std::cout << "JOINT ["<< id  <<  "] -> ";
	for each ( auto c in connectedNodes){
		std::cout << c->getId() << "  ";
	}
	std::cout << std::endl;
}
void KinDriverAbstraction_Joint:: printInfo(){
	std::cout << "JOINT ["<< id  <<  "] -> ";
	if(isDetermined_position){
		std::cout << "__POS__  ";
	}else{
		std::cout << "_______  ";
	}
	if(isDetermined_angle){
		std::cout << "_ANGLE_  ";
	}else{
		std::cout << "_______  ";
	}

	std::cout << std::endl;
		

}

// -------------------------- DrivingJoinSelection ---------------------------------------------------------------

KinDriverAbstraction* addToList(KinNode* node, std::vector<KinDriverAbstraction*> &list, KinDriverAbstraction* ground){

	KinNode_Part * node_part = dynamic_cast<KinNode_Part*>(node);
	KinNode_Joint * node_joint = dynamic_cast<KinNode_Joint*>(node);
	KinDriverAbstraction* newAbst;
	if(node_part != nullptr){
		newAbst = new KinDriverAbstraction_Part(node_part, list.size());
	}
	if(node_joint != nullptr){
		newAbst = new KinDriverAbstraction_Joint(node_joint, true, list.size());
	}
	list.push_back(newAbst);
	for each (auto child in node->children){
		KinDriverAbstraction* neighbour = addToList(child, list, ground);
		newAbst->addConnection(neighbour);
		neighbour->addConnection(newAbst);
	}
	if(node->children.size() == 0){
		auto auxAbst = new KinDriverAbstraction_Joint(nullptr, false, list.size());
		list.push_back(auxAbst);
		newAbst->addConnection(auxAbst);
		auxAbst->addConnection(newAbst);
		ground->addConnection(auxAbst);
		auxAbst->addConnection(ground);
	}

	return newAbst;
}


int DrivingJointSelection::computeDrivingJoints(KinChain* initialChain){


	std::vector<KinDriverAbstraction*> chainAbstraction;


	// step1: create a linkage based on the kin chain
	KinDriverAbstraction_Part* ground = new KinDriverAbstraction_Part(nullptr, -1);
	addToList(initialChain->getRoot(), chainAbstraction, ground);
	chainAbstraction.push_back(ground);
	if(PRINT_DEBUG){
		for each( auto c in chainAbstraction)
			c->printConnections();
	}
	chainAbstraction[0]->fixInitialPosition();

	//step2: loop adding new informtation
	int N_addedDrivers = 0; 
	bool isComplete = false;
	while(!isComplete){

		// loop and try to add struff
		bool addedMoreInfo = false;
		for each( auto n in chainAbstraction){
			bool newInfo = n->discoverInformation();
			if (newInfo == true){
				addedMoreInfo = true;
			}
		}
		//loop and check if it is complete
		isComplete = true;
		for each( auto n in chainAbstraction){
			if(!n->isFullyDetermined()){
				isComplete = false;
			}
		}
		if(PRINT_DEBUG){
			for each( auto c in chainAbstraction)
				c->printInfo();
		}

		// add aditional driver if necessary
		if(!addedMoreInfo && !isComplete){
			N_addedDrivers++;
			bool addedDriver = false;
			for each( auto n in chainAbstraction){
				if(!addedDriver){
					if (n->makeMeADriver()){
						addedDriver = true;
					}
				}
			}
			if (! addedDriver){
				std::cout << "error: could not add a new driver" << std::endl;
				system("pause"); 
			}
			if(PRINT_DEBUG){
				std::cout << "added a new driving joint " << std::endl;
				for each( auto c in chainAbstraction)
					c->printInfo();
			}
		}

	}


	//step3: return resutls;
	std::cout <<" number of added drivers " << N_addedDrivers<< std::endl;

	return N_addedDrivers;

}