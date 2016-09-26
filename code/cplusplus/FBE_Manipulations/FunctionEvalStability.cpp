#include "FunctionEvalStability.h"
#include "template.h"
#include "workingTemplate.h"
#include "KinChain.h"
#include "ReducedEval.h"
#include "Geometry.h"
#include "CenterOfMass.h"
using namespace FabByExample;

FunctionEvalStability::FunctionEvalStability(WorkingTemplate* wt, ReducedEval * _redEval,  StabilizationObjective _objective,
	Eigen::MatrixXd & _A_geo, Eigen::VectorXd & _B_geo){
	this->wt = wt;
	//origQ = wt->getTemplate()->getFullQ();
	objective = _objective;
	redEval = _redEval;
	A_geo = _A_geo;
	B_geo = _B_geo;
}

double FunctionEvalStability::computeStabilityObjective(){

	double delta = 0.1;
	//std::vector<double> stabilityCost;
	//kinchain = new KinChain(templateElement->getRoot());
	wt->kinchain->updateGait(0.05);
	std::vector<Geometry*> anim = wt->kinchain->getAnimation();
	double currentStability = wt->kinchain->getStability();

	double otherObjective = 0;

	switch(objective){
	case StabilizationObjective::GEOMETRY:{
			Eigen::VectorXd currentQ = wt->getTemplate()->getFullQ();
			otherObjective = 0.5*(A_geo*currentQ - B_geo).norm();
		}	break;
	case StabilizationObjective::SPEED:{
		point p0 = anim[0]->getCenter();
		point pf = anim[anim.size() -1]->getCenter();
		//anim[0]->write("..\\..\\data\\test\\anim_o.stl");
		//anim[anim.size() -1]->write("..\\..\\data\\test\\anim_f.stl");
		Eigen::Vector2d v0(p0[0], p0[2]);
		Eigen::Vector2d vf(pf[0], pf[2]);
		//VLOG(3)  << v0.transpose() << std::endl;
		//VLOG(3)  << vf.transpose() << std::endl;
		double speed = (v0 - vf).norm();
		//VLOG(3)  << speed << std::endl;
		otherObjective = 500.0;
		if(speed >1){
			otherObjective = 500.0/speed;
			//system("pause");
		}
		//VLOG(3)  << "otherObjective = " << otherObjective << std::endl;
		otherObjective = otherObjective*10;
		} break;								
	case StabilizationObjective::FABCOST:{
		CenterOfMass centerofMass = wt->getTemplate()->computeCenterOfMass();
		double fabCost = centerofMass.mass ;
		otherObjective = fabCost/1000;
		}break;
	}

	//VLOG(3)  << "stability = " << currentStability << std::endl;
	//VLOG(3)  << "other = " << otherObjective << std::endl;
	double totalEval = 100*currentStability + otherObjective;
	//VLOG(3)  << "totalEval = " << totalEval << std::endl;


	//wt->getTemplate()->updateFullQ(origQ);

	return totalEval;

}


double FunctionEvalStability::eval(double x){


	Eigen::VectorXd currentQ;
	redEval->getQ(origT + dir*x, currentQ);
	wt->getTemplate()->updateFullQ(currentQ);

	return FunctionEvalStability::computeStabilityObjective() ;



}