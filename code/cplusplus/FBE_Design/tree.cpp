#include <iostream>
#include <fstream>
#include "tree.h"
#include "component.h"
#include "geometry.h"
#include "functionality.h"

using namespace FabByExample;
using namespace std;

Edge::Edge(Component* _child, std::string _relativeMatrixValue){
	child = _child;
	relativeMatrixValue = _relativeMatrixValue;
	// convert matrix string into vector of doubles
	std::string relativeVal;
	std::stringstream ss(relativeMatrixValue);
	while (ss.good()) {
		ss >> relativeVal;
		//cout << "relativeVal = [" << relativeVal << "]" << endl; 
		std::string::iterator i = relativeVal.begin();
		while (isspace(*i)) relativeVal.erase(i);
			std::string::reverse_iterator j = relativeVal.rbegin();
		while (isspace(*j)) {
			relativeVal.resize(relativeVal.length()-1);
			j = relativeVal.rbegin();
		}
		relativeMatrix.push_back(atof(relativeVal.c_str()));
	}




}

Edge::Edge(Component* child, const vector<double>& relativeMatrix){
	this->child = child;
	this->relativeMatrix = relativeMatrix;
}

void Edge::printEdge(){
	//std::cout << "Edge to = " << child->getId() << std::endl;
//	for (int i = 0 ; i < relativeMatrix.size(); i++) {
//		std::cout << relativeMatrix[i] << " ";
//	}
//	std::cout << std::endl;

}

//-----------------------------------------------------------------------------------------------------




Tree::Tree(Design* _parentDesign, std::string _objectName){
	objectName = _objectName;
	parentDesign = _parentDesign;
}

Tree::~Tree(){
	for (std::list<Component*>::iterator it = components.begin(); it != components.end(); ++it){
		delete (*it);
	}
	components.clear();
	for (std::vector<Functionality*>::iterator it = functionalities.begin(); it != functionalities.end(); ++it){
		delete (*it);
	}
	functionalities.clear();
}


void Tree::addSubstructure(Component * comp){
	std::string name = comp->getName();
	if ( (name.compare(0,3, "NA.") == 0) || (name.compare(0,3, "SA.") == 0)){
		// do not add		
	}else{
		substructures.push_back(comp);
		comp->makeSubstructure(); 
	} 
}

Component* Tree::addComponent(int _id, std::string _name, bool isLeaf){
	if (isLeaf){
		std::string fileName(_name);
		size_t found=fileName.find("-");
		if(found < 200)
			fileName.resize(found);
		fileName.append(".STL"); 
		ComponentLeaf* newComponet = new ComponentLeaf(parentDesign, _id, _name, fileName);
		components.push_back(newComponet);
		return newComponet;
	}else{
		ComponentNode* newComponet = new ComponentNode(parentDesign, _id, _name);
		components.push_back(newComponet);
		if (_id == 10000){
			root = newComponet;
		}
		return newComponet;
	}
}

void Tree::addRepetedSubassembly(Component* parentComp, Component* copyComp, int val){

	std::list<Edge*>::iterator it;
	list<Edge*> edg = copyComp->getEdges();
	//std::cout << "edg = " << edg.size() << std::endl; system("pause"); 	
	for ( it= edg.begin() ; it != edg.end(); ++it ){
		Component* compChild = (*it)->getChild();
		Component* newCompChild = addComponent(compChild->getId() + val, compChild->getName(), compChild->getIsLeaf());
		addEdge(parentComp->getId(), newCompChild->getId(), (*it)->getRelativeMatrixValue());
		addRepetedSubassembly(newCompChild, compChild, val);
	}

}

//system("pause");



void Tree::read(){
		std::list<Component*>::iterator it;
		cout << "READ" << endl; 
		for ( it=components.begin() ; it != components.end(); ++it ){
			std::cout << "=== New Component in tree === " << std::endl;
			(*it)->printComponent();
		}

}


void Tree::simpleRead(){
		std::list<Component*>::iterator it;
		cout << "================ TREE ================" << endl; 
		root->simplePrintComponent(0);
		cout << "================ END TREE ================" << endl; 

}

void Tree::readComponents(){
		std::list<Component*>::iterator it;
		cout << "READ COMPONENTS" << endl; 
		for ( it=components.begin() ; it != components.end(); ++it ){
			std::cout << (*it)->getId() << " --- " << (*it)->getName() << std::endl;
		}

}

void Tree::write(std::string filename){
		std::list<Component*>::iterator it;
		for ( it=components.begin() ; it != components.end(); ++it ){
			if (!(*it)->getName().compare(objectName)){
				//cout << "Writing" << endl; 
				(*it)->getGeometry()->write(filename);
			}
		}

}


Component* Tree::getComponentFromID (int id){
	Component* component;
	std::list<Component*>::iterator it;
	for ( it=components.begin() ; it != components.end(); ++it ){
		if((*it)->getId() == id)
			component = *it;
	}
	return component;
}



int Tree::getIdFromNames (std::string pname, std::string sname){
	Component* component;
	std::list<Component*>::iterator it;
	for ( it=components.begin() ; it != components.end(); ++it ){
		if((*it)->getName().compare(sname) == 0){
			if((*it)->getParent()->getName().compare(pname) == 0){
				component = *it;
				return component->getId();
			}
		}
	}
	return -1;
}



void Tree::addEdge(int agg_id, int ist_id, std::string relativeMatrixValue){
		
	Component* aggComponent = this->getComponentFromID(agg_id);
	Component* istComponent = this->getComponentFromID(ist_id);
	aggComponent->addEdge(istComponent, relativeMatrixValue);
	istComponent->setParrent(aggComponent);

}



void Tree::updateGlobalTrans(Component* comp, matrix9f rot2, vector3f trans2){
	//std::cout << "at updateglobaltrans = " << comp->getName() << std::endl; 
	comp->setGlobalTrans(rot2, trans2);
	std::list<Edge*>::iterator it;
	list<Edge*> edg = comp->getEdges();
	//std::cout << "edg = " << edg.size() << std::endl; system("pause"); 	
	for ( it= edg.begin() ; it != edg.end(); ++it ){
			matrix9f rot1((*it)->getRelativeMatrix()[0], (*it)->getRelativeMatrix()[1], (*it)->getRelativeMatrix()[2], (*it)->getRelativeMatrix()[3], (*it)->getRelativeMatrix()[4], (*it)->getRelativeMatrix()[5], (*it)->getRelativeMatrix()[6], (*it)->getRelativeMatrix()[7], (*it)->getRelativeMatrix()[8]);
			vector3f trans1((*it)->getRelativeMatrix()[9], (*it)->getRelativeMatrix()[10], (*it)->getRelativeMatrix()[11]);
			matrix9f newrot = rot1*rot2;
			vector3f newtrans = trans2 + rot2.Transpose()*trans1;
			updateGlobalTrans((*it)->getChild(), newrot, newtrans);
	}

}

 //system("pause"); 




void Tree::saveAllLeafComponentsInGlobalCoords(std::string filelname){

	Geometry* geo = new Geometry;
	std::list<Component*>::iterator it;
	for ( it=components.begin() ; it != components.end(); ++it ){ if((*it)->getIsLeaf() == true){
		std::cout << "here" << std::endl;
		geo->addMesh((*it)->getGeometryInGlobalCoord()->getMesh());
	}}
	geo->write(filelname);
}


//void Tree::saveAllLeafComponentsInGlobalCoordsMinFunc(std::string filelname, double alpha, int id){
//
//	Geometry* geo = new Geometry;
//	std::list<Component*>::iterator it;
//	for ( it=components.begin() ; it != components.end(); ++it ){ if((*it)->getIsLeaf() == true){
//		std::cout << "here" << std::endl;
//		Geometry* auxgeo = (*it)->getGeometryMinFunc(alpha, id);
//		geo->addMesh(auxgeo->getMesh());
//	}}
//	geo->write(filelname);
//}


std::list<Component*>  Tree::getLeafComponents(){

	std::list<Component*> leafComponents;
	std::list<Component*>::iterator it;
	for ( it=components.begin() ; it != components.end(); ++it ){ if((*it)->getIsLeaf() == true){
		leafComponents.push_back(*it);
	}}
	return leafComponents;
}


std::list<Component*>  Tree::getLeafsOfComp(Component* comp){

	std::list<Component*> leafComponents;
	std::list<Component*>::iterator it;
	for ( it=components.begin() ; it != components.end(); ++it ){ if((*it)->getIsLeaf() == true){
		leafComponents.push_back(*it);
	}}
	return leafComponents;
}


void Tree::checkAllIntersections(string filename){
	ofstream myfile (filename);
	if (!myfile.is_open())
	{
		cout << "Unable to open file" << endl;
	}
  
	std::list<Component*>::iterator it, it2;
	for ( it=components.begin() ; it != components.end(); ++it ){ if((*it)->getIsLeaf() == true){
		it2 = it; it2++;
		for ( 0; it2 != components.end(); ++it2 ){ if((*it2)->getIsLeaf() == true){
			//cout << "Comparing = " << (*it)->getName() << " and " << (*it2)->getName() << endl;
			bool intersection = (*it)->getGeometryInGlobalCoord()->checkIntersection((*it2)->getGeometryInGlobalCoord());
			if ( intersection == true){	
				myfile  << "New Intersection" << endl;
				myfile <<  (*it)->getId() << endl;
				myfile <<  (*it2)->getId() << endl;

			}
		}}
	}}
    myfile.close();
}


void Tree::setFunctionality(){

	//std::cout<<"setting funcitonality " << std::endl;
	//read();
	//std::cout<<"setting funcitonality ---" << std::endl;
	//system("pause");
	std::list<Edge*>::iterator it;
	list<Edge*> edg = root->getEdges();
	for ( it= edg.begin() ; it != edg.end(); ++it ){
			recursion(root,(*it));
	}
	//read();
	//system("pause");

}



void Tree::recursion(Component* parent, Edge* compEdg){
	// fix the possible translation that is part of the functionality component assembly
	if (compEdg->getChild()->isFunc()){
		//std::cout<<"found fucntionality" << std::endl;
		std::list<Edge*>::iterator it;
		list<Edge*> edg = compEdg->getChild()->getEdges();
		Edge*  nextAssembly;
		Edge* coordinate;
		for ( it= edg.begin() ; it != edg.end(); ++it ){
				if((*it)->getChild()->getName().compare( 0, 11, "coordinates") == 0 ){
					coordinate = (*it);
				}
				else{
					nextAssembly = (*it);
				}
				(*it)->addtransformation(compEdg->getRelativeMatrix()); 
		}

		//cout << "adding functionality to " << nextAssembly->getChild()->getName() << " ->" <<  compEdg->getChild()->getName() << endl;
		//std::cout << "global rotation = [" << compEdg->getChild()->getGlobalRot().matrix[0] << ", " << compEdg->getChild()->getGlobalRot().matrix[1] << ", " << compEdg->getChild()->getGlobalRot().matrix[2] << ", " << compEdg->getChild()->getGlobalRot().matrix[3] << ", " << compEdg->getChild()->getGlobalRot().matrix[4] << ", " << compEdg->getChild()->getGlobalRot().matrix[5] << "]" << std::endl;
		//std::cout << "global translation = [" << compEdg->getChild()->getGlobalTrans().vertex[0] << ", " << compEdg->getChild()->getGlobalTrans().vertex[1] << ", " << compEdg->getChild()->getGlobalTrans().vertex[2] << "]" << std::endl;
		//system("pause");
		Functionality*	func = new Functionality( coordinate->getChild()->getCenterOfAxisInGlobalCoord(), compEdg->getChild()->getName(), functionalities.size()); // compEdg->getChild()->getGlobalRot(), compEdg->getChild()->getGlobalTrans());
		functionalities.push_back(func);
		nextAssembly->getChild()->addFunctionality(func);		


		parent->removeChild(compEdg);
		parent->addChild(nextAssembly);
		//readComponents();
		//system("pause");
		removeComponent(compEdg->getChild());
		removeComponent(coordinate->getChild());
		//readComponents();
		//system("pause");
		recursion(parent, nextAssembly); 

	}
	else{
		std::list<Edge*>::iterator it;
		list<Edge*> edg = compEdg->getChild()->getEdges();
		for ( it= edg.begin() ; it != edg.end(); ++it ){
				recursion(compEdg->getChild(), (*it));
		}
	}


}

int Tree::removeComponent(Component* comp){
	std::list<Component*>::iterator it;
	for ( it=components.begin() ; it != components.end(); ++it ){
		if((*it)->getId() == comp->getId()){
			components.erase(it);
			//system("pause");
			return 0;
		}
	}
	return 1;
}



//how to combine translations and rotations
int Tree::removeComponentfromTree(Component* comp){
		Component* parent = comp->getParent();
		std::list<Edge*>::iterator it;
		list<Edge*> edg = comp->getEdges();
		for ( it= edg.begin() ; it != edg.end(); ++it ){
			parent->addChild((*it));
		}
		parent->removeChildFromComp(comp);
		return(	removeComponent(comp));

}


void Tree::removeAllConnectors(){
	std::list<Component*>::iterator it, it2;
	for ( it=components.begin() ; it != components.end();  ){
		it2 = it;
		it2++;
		std::cout << "at component = " << (*it)->getName() << std::endl;
		if((*it)->getIsLeaf()){
			if((*it)->isConnector()){
				removeComponentfromTree(*it);
			}		
		}
		it = it2;
	}
}

std::vector<Functionality*> Tree::getFunctionalitiesForComp(Component * maincomp){

	std::vector<Functionality*> funcsForComp;

	for ( unsigned int itt= 0; itt < functionalities.size(); itt++){
		if(maincomp->checkIfFunctionalityIsChild(functionalities[itt])){
			funcsForComp.push_back(functionalities[itt]);
		}
	}
	
	return funcsForComp;

}