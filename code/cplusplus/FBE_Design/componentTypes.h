#ifndef __COMPONENT_TYPES__
#define __COMPONENT_TYPES__

#include "component.h"


namespace FabByExample{

class GNode;
class PartInfo;

class ComponentLeaf: public Component{

public:

	ComponentLeaf(Design* _tree, int _id, std::string _name, std::string _geometryFile);

	bool getIsLeaf() {return true;}

	void setMaterial(int _material){
		material = _material;
	}
	void addMaterialToVector(Eigen::VectorXd & materialVector);
	double getVolume();
	unsigned int getMaterial(){return material;}

	//range information
	std::string getRange(PartInfo* partInfo);
	std::string getRangeBasedOnOrgGeo(PartInfo* partInfo, Geometry* orgGeo, matrix9f myrot);
	std::string getRangeBasedOnBestRotForGeo(PartInfo* partInfo, Geometry* orgGeo);



private:
	unsigned int material;


};

class ComponentNode: public Component{

public:

	ComponentNode(Design* _tree, int _id, std::string _name) :Component( _tree,  _id, _name){
		geometry = NULL;
	}
	ComponentNode::ComponentNode(GNode* gnode, int _id, std::string _name);

	bool getIsLeaf() {return false;}

	void addMaterialToVector(Eigen::VectorXd  & materialVector){
		std::list<Component*> leafs = getLeafs();
		std::list<Component*>::iterator it;
		for ( it=leafs.begin() ; it != leafs.end(); it++ ){
			(*it)->addMaterialToVector(materialVector);
		} 
	}

	double getVolume(){
		double volume = 0;
		std::list<Component*> leafs = getLeafs();
		std::list<Component*>::iterator it;
		for ( it=leafs.begin() ; it != leafs.end(); it++ ){
			volume += (*it)->getVolume();
		} 	
		return volume;
	}

};

}
#endif