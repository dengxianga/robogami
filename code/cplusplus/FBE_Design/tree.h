#ifndef __TREE__
#define __TREE__

#include<string>
#include<list>
#include<vector>
#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <iterator>
#include <stdio.h>
#include <sstream>


#include "component.h"
#include "componentTypes.h"

namespace FabByExample{

class Edge{
private:
	Component* child;
	std::vector<double> relativeMatrix;
	std::string relativeMatrixValue;
public:
	Edge(Component* _child, std::string relativeMatrixValue);
	Edge(Component* _child, const std::vector<double>& relativeMatrix);
	void printEdge();
	Component* getChild(){
		return child;
	}
	std::string getRelativeMatrixValue() {
		return relativeMatrixValue;
	}
	std::vector<double> getRelativeMatrix(){
		return relativeMatrix;
	}
	void addtransformation(std::vector<double> relMat) {} // fill in woth proper thing}
	~Edge(){
		std::cout<< "in delete edge" << std::endl;
	}
};



class Tree{
private:
	std::list<Component*> components;
	std::list<Component*> substructures;
	std::vector<Functionality*> functionalities;	
	std::string objectName;
	Component* root;
	Design* parentDesign;

public:
	Tree(Design* parentDesign, std::string _objectName);
	~Tree();
	void pushBackComp(Component* comp){components.push_back(comp);}
	void addSubstructure(Component* comp);
	std::string getName(){return objectName;}
	Component* getRoot(){ return root;}
	std::list<Component*> getSubstructures() {return substructures;}
	std::list<Component*> getComponents() {return components;}
	std::vector<Functionality*>getFunctionalities() {return functionalities;}	
	int getNumFuncts(){return functionalities.size();}
	Component* addComponent(int _id, std::string _name, bool isLeaf);
	void addRepetedSubassembly(Component* parentComp, Component* copyComp, int val);
	void read();
	void simpleRead();
	void readComponents();
	Component* getComponentFromID (int id);
	int getIdFromNames (std::string pname, std::string sname);
	void addEdge(int agg_id, int ist_id, std::string relativeMatrixValue);
	//void Tree::addReference (int id);
	void write(std::string filename);
	void updateGlobalTrans(Component* comp, matrix9f rot, vector3f trans);
	void saveAllLeafComponentsInGlobalCoords(std::string filelname);
	//void saveAllLeafComponentsInGlobalCoordsMinFunc(std::string filelname, double  alpha, int id);
	std::list<Component*>  getLeafComponents();
	std::list<Component*>  getLeafsOfComp(Component* comp);
	void checkAllIntersections(std::string filelname);
	void setFunctionality();
	void recursion(Component* parent, Edge* compEdg);
	int removeComponent(Component* comp);
	int removeComponentfromTree(Component* comp);
	void removeAllConnectors();
	std::vector<Functionality*>getFunctionalitiesForComp(Component * maincomp);

};

}
#endif