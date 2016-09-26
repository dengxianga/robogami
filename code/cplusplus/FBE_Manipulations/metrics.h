#ifndef _METRICS_
#define _METRICS_

#include <vector>
namespace FabByExample{

class WorkingTemplate;

class MetricsInfo{
public:
	MetricsInfo(){
		fabCost = 0;
		isStable = false;
		speed = 0; 
		error = 0;
		slip = 0;
		perperror = 0;
		curvature = 0;
		meanangle = 0;
		rotation = 0;
		travel = 0;
		electronicsCost = 0;
		toppling = 0;
		totalMass = 0;
		path.push_back(std::pair<double,double>(0,0));
		path.push_back(std::pair<double,double>(0,1));
		path.push_back(std::pair<double,double>(1,0));
		path.push_back(std::pair<double,double>(1,1));
	}
	double fabCost;
	bool isStable;
	double speed; 
	double error;
	double slip;
	double perperror;
	double curvature;
	double meanangle;
	double rotation;
	double travel;
	double electronicsCost;
	double toppling;
	double totalMass;
	std::vector<std::pair<double,double>>  path;
};

class Metrics{

public:
	Metrics(WorkingTemplate* _wt);
	double getFabricationCost(); 
	double getElecCost(); 
	bool checkStability();
	double Metrics::getSpeed();
	double getError();
	double getTotalMass();

private:
	WorkingTemplate* wt;
};


}



#endif

