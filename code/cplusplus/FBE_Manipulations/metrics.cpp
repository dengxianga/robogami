#include "metrics.h"
#include "WorkingTemplate.h"
#include "KinChain.h"
#include "ElectronicsGraph.h"
#include "template.h"
#include "CenterOfMass.h"
#include "GeometricOperations.h"
#include "geometry.h"
using namespace FabByExample;

Metrics::Metrics(WorkingTemplate* _wt){
	wt = _wt;
	

}

double Metrics::getFabricationCost(){

	CenterOfMass centerofMass = wt->getTemplate()->computeCenterOfMass();
	double fabCost = centerofMass.mass * 0.391/1000; // density from measurements

	int nservos = wt->elecgraph->getNumServos();

	double elecMass = 0;
	if (nservos > 0) {
		elecMass = 4.26 * nservos+ // servos
			3.894 + 7.376 + // arduino + breakout board
			4.145; // battery
	}

	//std::cout << "Fabricaiton cost is: " << fabCost << std::endl;
	return fabCost;

}

double Metrics::getTotalMass(){

	CenterOfMass centerofMass = wt->getTemplate()->computeCenterOfMass();
	double fabCost = centerofMass.mass * 0.391/1000; // density from measurements

	int nservos = wt->elecgraph->getNumServos();

	double elecMass = 0;
	if (nservos > 0) {
		elecMass = 4.26 * nservos+ // servos
			3.894 + 7.376 + // arduino + breakout board
			4.145; // battery
	}

	//std::cout << "Fabricaiton cost is: " << fabCost << std::endl;
	return fabCost + elecMass;

}

double Metrics::getElecCost() {
	int nservos = wt->elecgraph->getNumServos();

	if (nservos == 0) { return 0; }

	return 3.12 * nservos + // servos
		9.95 + // arduino
		1.33; // battery
}


double Metrics::getSpeed(){


	//double delta = 0.1;
	//std::vector<double> stabilityCost;
	//wt->kinchain->updateGait(stabilityCost, 0.1, false, 1);
	//std::vector<Geometry*> anim = wt->kinchain->getAnimation();
	return wt->kinchain->getSpeed();

}





double Metrics::getError()
{
	//double delta = 0.1;
	//std::vector<double> stabilityCost;
	//wt->kinchain->updateGait(stabilityCost, 0.1, false, 1);
	//std::vector<Geometry*> anim = wt->kinchain->getAnimation();
	return wt->kinchain->getStability();
}


bool Metrics::checkStability(){
	//double delta = 0.1;
	//std::vector<double> stabilityCost;
	//wt->kinchain->updateGait(stabilityCost, 0.1, false, 1);
	//std::vector<Geometry*> anim = wt->kinchain->getAnimation();
	double initialStability = wt->kinchain->getStability();
	
	double epsilon = 0.5;
	if (initialStability > epsilon)
		return false;

	return true;

}

/*
bool Metrics::checkStability(){
	PROFILE_THIS(__FUNCTION__);

	CenterOfMass centerofMass = temp->computeCenterOfMass();
	//std::cout << "computed center of mass in " << double( clock() - begin_time_com ) / (double)CLOCKS_PER_SEC << " seconds!" << std::endl; 

	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>  contactPoints;
	temp->getContactPoints(contactPoints); 
	//std::cout << "computed contactPoints in " << double( clock() - begin_time_cpoints ) / (double)CLOCKS_PER_SEC << " seconds!" << std::endl; 

	// copute convex hull of center points
	std::vector<int> hull;
	GeometricOperations::computeConvexHull(contactPoints, hull);


	// check if center is in convex hull

	//std::cout << "convex hull is " << std::endl;
	//for(int i = 0; i < hull.size(); i++){
	//	std::cout << "hull [" << i << "] = " << hull[i] << std::endl;
	//}

	bool isStable = true;
	double stabDist = 0;
	Eigen::Vector2d center;
	center.x() = centerofMass.center.x();
	center.y() = centerofMass.center.z();
	for(int i = 0; i < hull.size(); i++){
		int next_i = i+1;
		if(next_i == hull.size()){
			next_i = 0;
		}
		double newDist = GeometricOperations::mycross(contactPoints[hull[i]], center, contactPoints[hull[next_i]]);
		if(newDist > 0){
			//std::cout << "is out " << std::endl;
			isStable = false;
			if(stabDist < newDist){
				stabDist = newDist;
			}
		}else{
			//std::cout << "is in!" << std::endl;
		}
	}

	//std::cout << "finished everything in " << double( clock() - begin_time_full ) / (double)CLOCKS_PER_SEC << " seconds!" << std::endl; 


	return isStable;
}

*/

