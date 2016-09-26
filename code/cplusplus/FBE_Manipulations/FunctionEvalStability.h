#pragma once
#include <Eigen/Dense>
#include "FunctionEval.h"

namespace FabByExample {

class WorkingTemplate;
class ReducedEval;

class FunctionEvalStability: public FunctionEval{
public:
	enum StabilizationObjective{NONE, GEOMETRY, FABCOST, SPEED, TRAVEL, PERPENDICULAR_ERROR, CURVATURE, WOBBLINESS};

	FunctionEvalStability(WorkingTemplate* _wt, ReducedEval * _redEval, 
		StabilizationObjective _objective, Eigen::MatrixXd & _A_geo, Eigen::VectorXd & _B_geo);
	void setDir(Eigen::VectorXd _dir){dir = _dir;}
	void setStartingPoint(Eigen::VectorXd _origT){origT = _origT;}
	double eval(double x);
	
	double computeStabilityObjective();

private:
	StabilizationObjective objective;
	WorkingTemplate* wt; 
	Eigen::VectorXd dir;
	Eigen::VectorXd origT; 
	ReducedEval * redEval;

	Eigen::MatrixXd A_geo;
	Eigen::VectorXd B_geo;

};
}