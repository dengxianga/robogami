#include "component.h"
#include "componentTypes.h"
#include "TriMesh_algo.h"
#include "tree.h"
#include "graph.h"
#include "geometry.h"
#include "dataTraits.h"
using namespace std;
using namespace FabByExample;

ComponentLeaf::ComponentLeaf(Design* _tree, int _id, std::string _name, std::string _geometryFile) :Component(_tree, _id, _name){
	//std::cout << "create new geomery " << std::endl;
	orig_geometry = new Geometry(_geometryFile);
	geometry = orig_geometry;
	//if(this->isConnector()){
	//	geometry = orig_geometry->compress(); 
	//}
	material = -1;
}

void ComponentLeaf::addMaterialToVector(Eigen::VectorXd & materialVector){
	if (isPart()){
		if (material < 0){
			std::cout << "Error: material not initialized" << std::endl; system("pause");
		}
		else{
			materialVector(material) += getVolume();
		}
	}
}

double ComponentLeaf::getVolume(){
	return geometry->getBBoxVolume(); // which geometry actually? 
}

string ComponentLeaf::getRange(PartInfo* partInfo){

	//std::cout << "Name: " << this->getName()<< std::endl;
	matrix9f myrot;
	myrot.LoadIdentity();

	// get original geometry in global coordinates
	Geometry *orgGeo = new Geometry(partInfo->name);
	xform trans(1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				globalTrans.vertex[0], globalTrans.vertex[1], globalTrans.vertex[2] , 1);
	xform rot(globalRot.matrix[0], globalRot.matrix[1], globalRot.matrix[2], 0,
				globalRot.matrix[3], globalRot.matrix[4], globalRot.matrix[5], 0,
				globalRot.matrix[6], globalRot.matrix[7], globalRot.matrix[8], 0,
				0, 0, 0, 1);			
	apply_xform(orgGeo->mesh, rot);			
	apply_xform(orgGeo->mesh, trans);




	string range;
	if (this->getGeometryInGlobalCoord()->checkIfIsContainedIn(orgGeo) == true)
		range = getRangeBasedOnOrgGeo(partInfo, orgGeo, globalRot);
	else
		range = getRangeBasedOnBestRotForGeo(partInfo, orgGeo);
	return(range);

	delete orgGeo;

}


string ComponentLeaf::getRangeBasedOnOrgGeo(PartInfo* partInfo, Geometry* orgGeo, matrix9f myrot){
	stringstream range;


	vector3f ex(1, 0, 0);
	vector3f ey(0, 1, 0);
	vector3f ez(0, 0, 1);

	std::vector<vector3f> rotVecs(3);

	rotVecs[0] = myrot*ex;
	rotVecs[1] = myrot*ey;
	rotVecs[2] = myrot*ez;

	std::vector<int> errors(3);


	for (int ax = 0; ax< 3; ax++){
		errors[ax] = 2;
		string xstring;
		if (abs(rotVecs[ax].vertex[0]) < 0.0001)
			errors[ax]--;
		else
			xstring =partInfo->range[0].getString();
		if (abs(rotVecs[ax].vertex[1]) < 0.0001)
			errors[ax]--;
		else
			xstring = partInfo->range[1].getString();
		if (abs(rotVecs[ax].vertex[2]) < 0.0001)
			errors[ax]--;
		else
			xstring = partInfo->range[2].getString();
		if(ax < 2)
			range << xstring << " ";
		else
			range << xstring;
	}


	if((errors[0] + errors[1] + errors[2]) > 0){
		std::cout << "NO ROTATION FOUND" << std::endl;
		std::cout << "Strange rotation errors in comp = " << this->getName() << std::endl;
		for (int ax = 0; ax< 3; ax++){
			std::cout << "e["<< ax<<"] = ( " << rotVecs[ax].vertex[0] << ", " << rotVecs[ax].vertex[1] << ", " << rotVecs[ax].vertex[2]<< ")" << std::endl;
		}
		stringstream range2;
		for (int ax = 0; ax< 3; ax++){
			range2 << "f "  << getGeometryInGlobalCoord()->getSize()[ax];
			if(ax < 2)
				range << " ";
		
		}
		return(range2.str());

	}
		

	return(range.str());
}

string ComponentLeaf::getRangeBasedOnBestRotForGeo(PartInfo* partInfo, Geometry* orgGeo){
	string range("needs rot!");
	//std::cout << "Componene " << this->name << " needs rot " << std::endl;

	matrix9f myrot, rotx, roty, rotz;
	//myrot.LoadIdentity();
	myrot = globalRot;
	rotx.LoadIdentity(); rotx.RotateX(90);
	roty.LoadIdentity(); roty.RotateY(90);
	rotz.LoadIdentity(); rotz.RotateZ(90);
	xform xformx(rotx.matrix[0], rotx.matrix[1], rotx.matrix[2], 0, rotx.matrix[3], rotx.matrix[4], rotx.matrix[5], 0, rotx.matrix[6], rotx.matrix[7], rotx.matrix[8], 0, 0, 0, 0, 1);
	xform xformy(roty.matrix[0], roty.matrix[1], roty.matrix[2], 0, roty.matrix[3], roty.matrix[4], roty.matrix[5], 0, roty.matrix[6], roty.matrix[7], roty.matrix[8], 0, 0, 0, 0, 1);
	xform xformz(rotz.matrix[0], rotz.matrix[1], rotz.matrix[2], 0, rotz.matrix[3], rotz.matrix[4], rotz.matrix[5], 0, rotz.matrix[6], rotz.matrix[7], rotz.matrix[8], 0, 0, 0, 0, 1);

	myrot.RotateX(90);
	apply_xform(orgGeo->mesh, xformx);	
	if (this->getGeometryInGlobalCoord()->checkIfIsContainedIn(orgGeo) == true){
		range = getRangeBasedOnOrgGeo(partInfo, orgGeo, myrot);
		//range.append(" ------- trans x");
		return(range);
	}
	myrot.RotateY(90);
	apply_xform(orgGeo->mesh, xformy);	
	if (this->getGeometryInGlobalCoord()->checkIfIsContainedIn(orgGeo) == true){
		range = getRangeBasedOnOrgGeo(partInfo, orgGeo, myrot);
		//range.append(" ------- trans xy");
		return(range);
	}
	myrot.RotateZ(90);
	apply_xform(orgGeo->mesh, xformz);	
	if (this->getGeometryInGlobalCoord()->checkIfIsContainedIn(orgGeo) == true){
		range = getRangeBasedOnOrgGeo(partInfo, orgGeo, myrot);
		//range.append(" ------- trans xyz");
		return(range);
	}
	myrot.RotateX(90);
	apply_xform(orgGeo->mesh, xformx);	
	if (this->getGeometryInGlobalCoord()->checkIfIsContainedIn(orgGeo) == true){
		range = getRangeBasedOnOrgGeo(partInfo, orgGeo, myrot);
		//range.append(" ------- trans xyzx");
		return(range);
	}
	myrot.RotateY(90);
	apply_xform(orgGeo->mesh, xformy);	
	if (this->getGeometryInGlobalCoord()->checkIfIsContainedIn(orgGeo) == true){
		range = getRangeBasedOnOrgGeo(partInfo, orgGeo, myrot);
		//range.append(" ------- trans xyzxy");
		return(range);
	}

	
	std::cout << "There is an error in this piece - the dimentions don't match with original" << std::endl;

	system("pause");
	return range;
}


ComponentNode::ComponentNode(GNode* gnode, int _id, std::string _name) :
			Component( gnode->getComponent()->getDesign(),  _id, _name){

	geometry = NULL;
	Component * thisParent = gnode->getComponent()->getParent();
	this->setParrent(thisParent);
	thisParent->addEdge(this, "1 0 0 0 1 0 0 0 1 0 0 0");
	
	std::list<Component*> comps = gnode->getComponents();
	std::list<Component*>::iterator it;
	for(it = comps.begin(); it!=comps.end(); it++){
		(*it)->setParrent(this);
		Edge* edge = thisParent->getEdgeTo(*it);
		if(edge == NULL){
			std::cout << "error finding edge" << std::endl; system("pause"); 
		}
		thisParent->removeChild(edge);
		addEdge(*it, edge->getRelativeMatrixValue());
	}
	comps.clear();
}