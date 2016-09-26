#include "geometryOptimizer.h"
#include "workingTemplate.h"
#include "FriendlyLinearSystem.h"
#include "constraintsEval.h"
#include "ReducedEval.h"
#include "KinChain.h"
#include "articulation.h"
#include "controller.h"
#include "geometry.h"
#include "element_motion.h"
#include <nlopt.hpp>
#define DEBUG_GEOCOST false
#define DEBUG_OPTION 1
// DEBUG_OPTIONs:
// 0 : simple things, mid and big things
// 1 : mid things and big things
// 2 : just the big things
// 3 : nothing

using namespace FabByExample;






GeometryOptimizer::GeometryOptimizer(WorkingTemplate* _wTemplate, GeometryOptimizationWeights _weights):weights(_weights){
	
	wTemplate = _wTemplate;
	vector<Symbol> symbols = wTemplate->getTemplate()->getSymbols(TreeScope::DESCENDANTS);
	std::vector<Symbol> fixedMotionParameters;
	//find the symbols that will be constrained 
	for each ( auto n in wTemplate->kinchain->getNodes()){
		KinNode_Joint* joint = dynamic_cast<KinNode_Joint*>(n);
		if(joint != nullptr){
			if(!joint->isDriving){
				Controller* controller = joint->getArticulation()->transformations[0]->controller;
				if(dynamic_cast<SymbolicController*>(controller) != nullptr){
					Template * motionElement = (Template*)(dynamic_cast<SymbolicController*>(controller))->getElementMotion()->getRefTemplateElement();
					//ConcreteSymbolicAssignment env;
					//motionElement->addCurrentEnvTo(env);
					std::pair<Symbol, Symbol> legLiftParameter;
					std::pair<Symbol, Symbol> legMovingParameter;
//					for each (auto entry in env.map) {
					for each (auto s in symbols){
						if(((Template*)s.owner) == motionElement){
							fixedMotionParameters.push_back(s);
						}
					}
				}
			}
		}
	}

	if(DEBUG_OPTION <= 0){
		std::cout << "there were " << fixedMotionParameters.size() << " motion parameters found." << std::endl;
		for each ( auto p in fixedMotionParameters){
			std::cout << "motion paramter is owned by = " << ((Template*)p.owner)->getName() << " and has name: " << p.owner->getSymbolName(p.id) << std::endl;
		}
	}


	// Reduce the DOF
	FlatLinearSystem flat;
	origConfiguration = _wTemplate->getTemplate()->getFullQ(); 
	ConstraintsEval::extractFeasibilityConstraintsFixingSomePameters(wTemplate->getTemplate(), fixedMotionParameters, &flat); 
	redEval = new ReducedEval(flat.Aeq_red, flat.Beq_red, flat.Aineq, flat.Bineq, origConfiguration); 
	redEval->getT(origConfiguration, origRedConfiguration);

	if(DEBUG_OPTION <= 1){
		std::cout << "number of initial parameters = " << origConfiguration.size() << std::endl;
		std::cout << "number of final parameters = " << origRedConfiguration.size() << std::endl;
	}

	bool  feasibility = redEval->evalFeasibility(origRedConfiguration);
	if(!feasibility){
		std::cout << "the initial point is not feasible" << std::endl; 
		system("pause");
	}

	//set constraints
	ineqConstraints.first =  redEval->C;
	ineqConstraints.second =  redEval->D;

	//set bounds
	for(int i = 0; i < origRedConfiguration.size(); i ++){
		double minEps, maxEps, otherRange;
		Eigen::VectorXd dir = Eigen::VectorXd::Zero(origRedConfiguration.size());
		dir(i) = 1; 
		redEval->getFeasibleRange(origRedConfiguration, dir, &otherRange, &maxEps);
		dir(i) = -1; 
		redEval->getFeasibleRange(origRedConfiguration, dir, &otherRange, &minEps);
		ub.push_back(maxEps);
		lb.push_back(-minEps);
	}

	

	//computeLegsOnFloorError(origRedConfiguration);

	//reduce geometry cost 
	redEval->getMatEquationFromQ2T(flat.Aopt, 	(-1)*flat.Bopt, geometryCost.first, geometryCost.second); 
	if(DEBUG_GEOCOST){
		std::cout <<  "the number of constraints on geometry are " << flat.Bopt.size() << std::endl;
		std::cout << "the cost of th opt is  with minus" << (flat.Aopt* origConfiguration - flat.Bopt).norm() << std::endl;
		std::cout << "the cost of th opt is  with plus" << (flat.Aopt* origConfiguration + flat.Bopt).norm() << std::endl;
		std::cout << "the cost of th opt after tarans  with minus" << (geometryCost.first*origRedConfiguration - geometryCost.second).norm() << std::endl;
		std::cout << "the cost of th opt after tarans  with plus" << (geometryCost.first*origRedConfiguration + geometryCost.second).norm() << std::endl;
		std::cout << "the initial geometry cost is " << computeGeometryCost(origRedConfiguration) << std::endl;
		system("pause");
	}
}


struct OptimizerConstraintData{
	int id;
	GeometryOptimizer* optimizer;
	OptimizerConstraintData(GeometryOptimizer * _optimizer, int _id){
		optimizer = _optimizer;
		id = _id;
	}
};

double GeometryOptimizer::computeGeometryCost(const Eigen::VectorXd  & x){
	return (geometryCost.first*x + geometryCost.second).norm();
}


double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *data){
	
	
	if(DEBUG_OPTION <= 0){
		std::cout << "the geo cost is ";
	}

	GeometryOptimizer* optimizer = reinterpret_cast<GeometryOptimizer*>(data);
	Eigen::VectorXd newQ;

	Eigen::VectorXd  xtemp = Eigen::VectorXd::Zero(x.size());
	for (int i =0; i <  x.size(); i++){
		xtemp(i) = x[i];
	}

	optimizer->redEval->getQ(xtemp, newQ);

	optimizer->wTemplate->getTemplate()->updateFullQ(newQ);
	//std::cout << "Going to compute gait ";
	clock_t  t_begin = clock();
	optimizer->wTemplate->kinchain->updateGait(0.05);
	//std::cout << "finished computing gate in  " <<  double( clock() - t_begin ) / (double)CLOCKS_PER_SEC << " seconds " << std::endl;
	
	if(DEBUG_OPTION <= 0){
		std::cout <<  optimizer->computeGeometryCost(xtemp) << std::endl;		
	}

	if(DEBUG_GEOCOST){
		optimizer->wTemplate->getTemplate()->getGeometry()->write("..\\..\\data\\testGeoCost.stl");
		system("pause");
	}
	
	return (optimizer->wTemplate->kinchain->getSlip()*optimizer->weights.slipWeight) +
		-(optimizer->wTemplate->kinchain->getSpeed()*optimizer->weights.speedWeight) +
		(optimizer->wTemplate->kinchain->getStability()*optimizer->weights.stabilityWeight)+
		(optimizer->computeGeometryCost(xtemp)*optimizer->weights.geometryWeight)+
		(optimizer->wTemplate->kinchain->getPerpendicularError()*optimizer->weights.straightnessWeight)+
		(optimizer->wTemplate->kinchain->getForwardTravel()*optimizer->weights.travelWeight)+
		(optimizer->wTemplate->kinchain->getMeanCurvature()*optimizer->weights.curvatureWeight)+
		(optimizer->wTemplate->kinchain->getToppling()*optimizer->weights.wobblyWeight);


}

double stabilityConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
	GeometryOptimizer *opt = reinterpret_cast<GeometryOptimizer*>(data);

	if(DEBUG_OPTION <= 0){
		std::cout << "the stability cost is " << opt->wTemplate->kinchain->getStability() << std::endl;		
	}

	return opt->wTemplate->kinchain->getStability();
}

double linearConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
   if(DEBUG_OPTION <= 0){
		std::cout << "the linear cost is ";
	}
   
    OptimizerConstraintData *d = reinterpret_cast<OptimizerConstraintData*>(data);
	auto opt = 	(d->optimizer); 
	Eigen::VectorXd lin = 	opt->ineqConstraints.first.row(d->id);
	//std::cout << "lin is " << lin.transpose() << std::endl;
	double r = 0;
	for(int i = 0; i< lin.size(); i++){
		r += lin(i)*x[i];
	}

	r -= opt->ineqConstraints.second(d->id);
	//std::cout << "r = " << r << std::endl;

	if(DEBUG_OPTION <= 0){
		std::cout << r << std::endl;
	}

    return r;
}






bool GeometryOptimizer::optimize(){                                                                      

	//setup
	std::cout << origRedConfiguration.size() << std::endl;
	//system("pause");
	
	nlopt::opt opt(nlopt::LN_COBYLA, origRedConfiguration.size());

	//std::cout << lb.size()  << " " << ub.size() << std::endl;
	//std::cout << lb << std::endl << std::endl << ub << std::endl;

	//system("pause");
	//opt.set_lower_bounds(lb);
	//opt.set_upper_bounds(ub);
	opt.set_min_objective(costFunction, (void*)this);

	for (int i = 0; i < ineqConstraints.first.rows(); i++){
		OptimizerConstraintData * data = new OptimizerConstraintData(this, 0);
		//std::cout << "adding ineqConstraint" << i << ineqConstraints.second(i) << std::endl;
		opt.add_inequality_constraint(linearConstraint, (void*)data, 0);
	}
	//opt.add_inequality_constraint(stabilityConstraint, (void*)this, 0);
	opt.set_xtol_rel(1e-4);

	std::vector<double> tol(2);
	tol[0] = 0; tol[1] =0;

	//optimization
	std::vector<double> x = EigenHelper::eigen2std(origRedConfiguration);
	double result;

	clock_t init, final;
	init=clock();
	opt.optimize(x, result);
	final=clock()-init;

	if (DEBUG_OPTION <= 1){
	std::cout << "Solution found in " << (double)final / ((double)CLOCKS_PER_SEC) << "seconds.\n";
	}

	//set the template to current values and compute the times
	Eigen::VectorXd finalQ;
	bool isFeasible = redEval->getQ(EigenHelper::std2eigen(x), finalQ);

	
	// reset the template to the original values and output the result configuration
	//x_vec= wTemplate->getTemplate()->getFullQ();
	//wTemplate->getTemplate()->updateFullQ(origConfiguration);

	if (DEBUG_OPTION <= 1){
	std::cout << "Feasibility of solution: " << isFeasible << std::endl;
	}
	Eigen::VectorXd feasibleX;
	redEval->getClosestFeasiblePoint(EigenHelper::std2eigen(x), feasibleX);
	Eigen::VectorXd feasibleFinalQ;
	isFeasible = redEval->getQ(feasibleX, feasibleFinalQ);

	if (DEBUG_OPTION <= 1){
	std::cout << "distance to feasible solution" << (feasibleFinalQ - finalQ).norm() << std::endl;
	}

	if(DEBUG_OPTION <= 0){
		std::cout << "the bounds are (size " << ub.size() << ", " << finalQ.size() << ")" << std::endl;
		for(int i = 0; i < ub.size(); i++){
			std::cout << lb[i] << ", " << ub[i] << ", val = " << finalQ[i] << ", feasible val = " << feasibleFinalQ[i] << std::endl;
		}
		std::cout << std::endl;

	}

	if (!isFeasible) {std::cout<<"Still not feasible :( \n" << std::endl;}

	wTemplate->getTemplate()->updateFullQ(feasibleFinalQ);

	return isFeasible;
}