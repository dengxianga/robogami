#pragma once

#include <Eigen/Dense>
#include <list>
#include <vector>
#include "symbolic.h" 
#include "KinChain.h"

namespace FabByExample{
class WorkingTemplate;
class ReducedEval;

struct InternalMotionOptimizerResult{
	double stabilityCost;
	double slipCost;
	double speedCost; 
	std::vector<std::pair<Symbol, double>> motionSpecification; 
};


struct OptimizationWeights{
	double stabilityWeight;
	double slipWeight;
	double speedWeight;
	double thetaWeight;
	OptimizationWeights(double _stabilityWeight, double _slipWeight, double _speedWeight, double _thetaWeight){
		stabilityWeight = _stabilityWeight;
		slipWeight = _slipWeight;
		speedWeight = _speedWeight;
		thetaWeight = _thetaWeight;
	}
};


class InternalMotionOptimizer{
public: 

	InternalMotionOptimizer(WorkingTemplate* _wTemplate, OptimizationWeights _weights);
	InternalMotionOptimizerResult optimize();
	void optimizeThetas(double &result);
	void optimizeGait(double &result);
	
	Eigen::Vector3d bodypoint;
	Eigen::Vector3d bodynorm;

	std::vector<double> alltimes;
	std::vector<double> origtheta;
	std::list<KinNode*> appendages;
	std::vector<bool> iscontact;
	std::vector<bool> ismoving;
	std::vector<int>  ncontact;

	int nDOF;
	int nparam;

	WorkingTemplate* wTemplate;
	OptimizationWeights weights;
private:
	std::vector<Symbol> parameters;
	std::vector<double> ub;
	std::vector<double> lb;

};



}