#ifndef __FUNCTIONALITY__
#define __FUNCTIONALITY__

#include "mathVec.h"
#include "XForm.h"
#include <vector>


namespace FabByExample{

struct FuncState{
	std::string name;
	std::vector<double> alphas;
};

class Functionality{
		
public:

	enum FuncType { ROT, TRANS, PROT };
	enum FuncAxis { NONE, X, Y, Z };

	int getId(){return thisId;}
	int getmin(){return min;}
	int getmax(){return max;}
	FuncType getType() {return type;}
	FuncAxis getAxis(){return axis;}
	Functionality(vector3f _center, std::string _functionalityName, int _ID); // matrix9f _globalRot, vector3f _globalTrans);
	void printFunctionality();    
	vector3f getCenter(){return center;}
	//vector3f getCenterInGlobalCoords(){return(globalRot*center + globalTrans);}
	xform getMinXfrom(double alpha);
	matrix9f getRot(double alpha);
	vector3f getTrans(double alpha);
	bool checkMatch(Functionality* otherFunc);
	std::string getName(){return functionalityName;}

private:
	int thisId;
	vector3f center;
	vector3f Tempcenter;
	std::string functionalityName;
	FuncType type;
	FuncAxis axis;
	int min;
	int max;
	//matrix9f globalRot;
	//vector3f globalTrans;
	


};


}

#endif