#include "tree.h" 
#include "functionality.h"
#include "connectorsGroup.h"
#include "TriMesh.h"
#include "featureGen.h"
#include <time.h>
#include "TriMesh_algo.h"
#include "params.h"
#include <limits>
#include "mathVec.h"
#include "myoptimizer.h"

using namespace std;
using namespace FabByExample;

Functionality::Functionality(vector3f _center, std::string _functionalityName, int _ID){
	Tempcenter = _center;
	center = _center;
	thisId = _ID;
	//std::cout << "center 2= [" << center.vertex[0]  << ", " << center.vertex[1] << ", " << center.vertex[2] << std::endl;

	functionalityName = _functionalityName;
	//globalRot = _globalRot;
	//globalTrans = _globalTrans;
	std::vector<std::string> parts;
	size_t found;
    found=functionalityName.find("_");
	//std::cout << "func name = " << functionalityName << std::endl;
	while(found!=std::string::npos){
		found=functionalityName.find("_");
		parts.push_back(functionalityName.substr(0,found));
		functionalityName = functionalityName.substr(found+1);
		found=functionalityName.find("_");
	}  
	parts.push_back(functionalityName);		
	//std::vector<std::string>::iterator it;
	//for ( it=parts.begin() ; it < parts.end(); it++ )
	//	std::cout << *it << std::endl;
	if (parts[1].compare(0,3, "ROT")== 0){
		//std::cout << "is rot " << std::endl;
		type = FuncType::ROT;
	}
	else{
		if (parts[1].compare(0,5, "TRANS")== 0){
			//std::cout << "is tarns " << std::endl;
			type = FuncType::TRANS;
		}
			else{
				if (parts[1].compare(0,8, "POINTROT")== 0){
					//std::cout << "is POINTROT "<< std::endl;
					type = FuncType::PROT;
				}
			}
	}
	axis = FuncAxis::NONE;
	if(parts.size() > 2){
		if (parts[2].compare(0,1, "X") == 0){
			//std::cout << "is X " << std::endl;
			axis = FuncAxis::X;
		}
		else{
			if (parts[2].compare(0,1, "Y") == 0){
				//std::cout << "is Y " << std::endl;
				axis = FuncAxis::Y;
			}
				else{
					if (parts[2].compare(0,1, "Z") == 0){
						//std::cout << "is Z "<< std::endl;
						axis = FuncAxis::Z;
					}
				}
		}
	}
	min = 0; 
	if(parts.size() > 3){
		min = atoi(parts[3].c_str());
		//std::cout << "min is  "<<  min << std::endl;
	}	
	max =0;
	if(parts.size() > 4){
		max = atoi(parts[4].c_str());
		//std::cout << "max is  "<<  max << std::endl;
	}


	//printFunctionality();
	//system("pause");

}

void Functionality::printFunctionality(){

	std::cout << "Fucntionality: " ;
	switch ( type ) {
	case FuncType::ROT:
		std::cout << " ROT" ;
	  break;
	case FuncType::TRANS:
		std::cout << " TRANS" ;
	  break;
	case FuncType::PROT:
		std::cout << " PROT" ;
	  break;
	default:
	  break;
	} 

	switch ( axis ) {
	case FuncAxis::X:
		std::cout << " - X - " << min << " - " << max ;
	  break;
	case FuncAxis::Y:
		std::cout << " - Y - " << min << " - " << max;
	  break;
	case FuncAxis::Z:
		std::cout << " - Z - " << min << " - " << max ;
	  break;
	default:
	  break;
	} 

	std::cout << std::endl;

}


xform Functionality::getMinXfrom(double alpha){

	double med = -(1-alpha)*min + alpha*max;

	//std::cout << "Fucntionality: " ;
	if(type == FuncType::ROT){
		//std::cout << " ROT" ;
		matrix9f rot;
		rot.LoadIdentity();
		switch ( axis ) {
			case FuncAxis::X:
				rot.RotateX(med);
				//std::cout << " - X - " << med << " - " << max ;
			  break;
			case FuncAxis::Y:
				rot.RotateY(med);
				//std::cout << " - Y - " << med << " - " << max;
			  break;
			case FuncAxis::Z:
				rot.RotateZ(med);
				//std::cout << " - Z - " << med << " - " << max ;
			  break;
			default:
			  break;
			} 
		xform rotXform(rot.matrix[0], rot.matrix[1], rot.matrix[2], 0,
				rot.matrix[3], rot.matrix[4], rot.matrix[5], 0,
				rot.matrix[6], rot.matrix[7], rot.matrix[8], 0,
				0, 0, 0, 1);
		return rotXform;
	}

	if(type == FuncType::TRANS){
		//std::cout << " ROT" ;
		vector3f trans;
		trans.LoadZero();
		switch ( axis ) {
			case FuncAxis::X:
				trans.vertex[0] = med*25.4;
			  break;
			case FuncAxis::Y:
				trans.vertex[1] = med*25.4;
			  break;
			case FuncAxis::Z:
				trans.vertex[2] = med*25.4;
			  break;
			default:
			  break;
		} 
		xform transXform(1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
				    trans.vertex[0], trans.vertex[1], trans.vertex[2], 1);
		return transXform;
	}

		xform noXform(1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
				    0, 0, 0, 1);
		return noXform;

}


matrix9f Functionality::getRot(double med){

	//double med = -(1-alpha)*min + alpha*max;


	//std::cout << "rotating = " <<  med << std::endl;
	matrix9f rot;
	rot.LoadIdentity();

	if(type == FuncType::ROT){
		switch ( axis ) {
			case FuncAxis::X:
				rot.RotateX(med);
				//std::cout << " - X - " << med << " - " << max << std::endl;
			  break;
			case FuncAxis::Y:
				rot.RotateY(-med);
				//std::cout << " - Y - " << med << " - " << max << std::endl;
			  break;
			case FuncAxis::Z:
				rot.RotateZ(med);
				//std::cout << " - Z - " << med << " - " << max  << std::endl;
			  break;
			default:
			  break;
			} 
	}
	
	rot = rot.Transpose();


	//std::cout << rot.matrix[0]  << " " << rot.matrix[1] << " " << rot.matrix[2] << " " << rot.matrix[3] << " " << rot.matrix[4] << " " << rot.matrix[5] << " " << rot.matrix[6] << " " << rot.matrix[7] << " " << rot.matrix[8] << std::endl;

	return rot;


}


vector3f Functionality::getTrans(double med){

	//double med = -(1-alpha)*min + alpha*max;


	//std::cout << "med = " <<  med << std::endl;
	vector3f trans;
	trans.LoadZero();
	
	if(type == FuncType::TRANS){
		//std::cout << " ROT" ;
		switch ( axis ) {
			case FuncAxis::X:
				trans.vertex[0] = med*25.4;
			  break;
			case FuncAxis::Y:
				trans.vertex[1] = med*25.4;
			  break;
			case FuncAxis::Z:
				trans.vertex[2] = med*25.4;
			  break;
			default:
			  break;
		} 


	}
	
	return trans;


}


bool Functionality::checkMatch(Functionality* otherFunc){

	if(type != otherFunc->getType()) {
		return false;
	}
	if(min != otherFunc->getmin()) {
		return false;
	}
	if(max != otherFunc->getmax()) {
		return false;
	}
	return true;

}

