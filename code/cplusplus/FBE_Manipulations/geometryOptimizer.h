#pragma once

#include <Eigen/Dense>
#include <list>
#include <vector>
#include "symbolic.h" 
#include "FunctionEvalStability.h"

namespace FabByExample{
class WorkingTemplate;
class ReducedEval;


struct GeometryOptimizationWeights{
	double stabilityWeight;
	double slipWeight;
	double speedWeight;
	double geometryWeight;
	double straightnessWeight;
	double travelWeight;
	double curvatureWeight;
	double wobblyWeight;
	GeometryOptimizationWeights(double _stabilityWeight, double _slipWeight, double _speedWeight, double _geometryWeight, double _straightnessWeight, double _travelWeight, double _curvatureWeight, double _wobblyWeight){
		stabilityWeight = _stabilityWeight;
		slipWeight = _slipWeight;
		speedWeight = _speedWeight;
		geometryWeight = _geometryWeight;
		straightnessWeight = _straightnessWeight;
		travelWeight = _travelWeight;
		curvatureWeight = _curvatureWeight;
		wobblyWeight = _wobblyWeight;
	}
};

struct GeometryResults {
	Eigen::VectorXd params;
	double slip;
	double stability;
	double geometryCost;
	double speed;
	double perperror;
	double travel;
	double curvature;
	double wobbliness;

	double getMetric(GeometryOptimizationWeights weights) {
		return (slip*weights.slipWeight) +
		-(speed*weights.speedWeight) +
		(stability*weights.stabilityWeight)+
		(geometryCost*weights.geometryWeight)+
		(perperror*weights.straightnessWeight)+
		(travel*weights.travelWeight)+
		(curvature*weights.curvatureWeight)+
		(wobbliness*weights.wobblyWeight);
	}

	double getMetric(FunctionEvalStability::StabilizationObjective stabObjective) {
		switch(stabObjective) {
			case FunctionEvalStability::StabilizationObjective::GEOMETRY : {
				return geometryCost;
				} break;
			case FunctionEvalStability::StabilizationObjective::SPEED : {
				return speed;
				} break;
			case FunctionEvalStability::StabilizationObjective::NONE : {
				return stability;
				} break;
			case FunctionEvalStability::StabilizationObjective::PERPENDICULAR_ERROR : {
				return perperror;
				} break;
			case FunctionEvalStability::StabilizationObjective::TRAVEL : {
				return travel;
				} break;
			case FunctionEvalStability::StabilizationObjective::CURVATURE : {
				return curvature;
				} break;
			case FunctionEvalStability::StabilizationObjective::WOBBLINESS : {
				return wobbliness;
				} break;
		}
	}
};



class GeometryOptimizer{
public:
	GeometryOptimizer();
	GeometryOptimizer(WorkingTemplate* _wTemplate, GeometryOptimizationWeights _weights);
	bool optimize( );

	double computeGeometryCost(const Eigen::VectorXd  & x);

	std::pair<Eigen::MatrixXd, Eigen::VectorXd> ineqConstraints;
	std::pair<Eigen::MatrixXd, Eigen::VectorXd> geometryCost;
	std::vector<double> ub;
	std::vector<double> lb;
	WorkingTemplate* wTemplate;
	ReducedEval* redEval;
	Eigen::VectorXd origConfiguration;
	Eigen::VectorXd origRedConfiguration;
	GeometryOptimizationWeights weights;
};


}
