#include "internalMotionOptimizer.h"
#include "workingTemplate.h"
#include "FriendlyLinearSystem.h"
#include "constraintsEval.h"
#include "ReducedEval.h"
#include "Geometry.h"
#include "GeometricOperations.h"
#include "CenterOfMass.h"

#include <nlopt.hpp>

#define PI 3.14159265358979323846

using namespace FabByExample;

bool showCostVals = true;
bool showConstraintVals = false;
bool drawGeo = false;

struct iOptimizerConstraintData{
	int tid;
	int DOFid;
	InternalMotionOptimizer* optimizer;
	
	iOptimizerConstraintData(InternalMotionOptimizer * _optimizer, int _tid, int _DOFid){
		optimizer = _optimizer;
		tid = _tid;
		DOFid = _DOFid;
	}
};

InternalMotionOptimizer::InternalMotionOptimizer(WorkingTemplate* _wTemplate, OptimizationWeights _weights): weights(_weights){
	wTemplate = _wTemplate;

	ConcreteSymbolicAssignment env;
	_wTemplate->getTemplate()->addCurrentEnvTo(env);
	for each (auto entry in env.map) {
		std::string SymbolName = entry.first.owner->getSymbolName(entry.first.id);
		if (SymbolName.find("motion_val" ) != std::string::npos) {
			parameters.push_back(entry.first);
		}
	}
	
	KinChain* kinchain = wTemplate->kinchain;

	// compile the time intervals list and get parameter info
	nDOF = kinchain->totalDegreesOfFreedom();

	alltimes = kinchain->getTimeStamps();
	origtheta.clear(); iscontact.clear(); ismoving.clear(); ncontact.clear();

	for (auto tid = alltimes.begin(); tid != alltimes.end(); ++tid) {
		kinchain->updateTime(*tid);
		kinchain->collectControlThetas(origtheta, iscontact, ismoving, *tid);

		// count number of contacts at each time step
		int this_ncontact = 0;
		for (int i = iscontact.size()-nDOF; i<iscontact.size(); ++i) {
			if (iscontact[i]) {this_ncontact++;}
		}
		ncontact.push_back(this_ncontact);
	}
	nparam = origtheta.size();

	// find body plane
	std::vector<Eigen::Vector3d> bodyattach = kinchain->getRoot()->getAllAttachmentPoints();
	GeometricOperations::calcPlane(bodyattach, bodypoint, bodynorm);

}


InternalMotionOptimizerResult InternalMotionOptimizer::optimize(){
	InternalMotionOptimizerResult result;
	result.slipCost =0;
	result.speedCost = 0;
	result.stabilityCost = 0;

	for each (auto s in parameters){
		result.motionSpecification.push_back(std::pair<Symbol, double>(s,0));
	}

	return result;
}

// costs
double thetaCostFunction(const std::vector<double> &x, std::vector<double> &grad, void *data){
	// sum of squared difference from input parameters

	InternalMotionOptimizer* toOptimize = reinterpret_cast<InternalMotionOptimizer*>(data);

	double difference = 0;

	for (int paramid = 0; paramid < x.size(); ++paramid) {
		//std::cout << x.at(paramid) << " " << toOptimize->origtheta[paramid] << std::endl;
		double diff = x.at(paramid) - toOptimize->origtheta[paramid];
		while (diff > PI) {diff -= 2*PI;}
		while (diff < -PI) {diff += 2*PI;}
		difference += diff*diff;
	}

	if (showCostVals) {
		std::cout<< "theta cost: " << difference << "   ";
	}
	
	return difference;

}

double slipCostFunction(const std::vector<double> &x, std::vector<double> &grad, void *data){
	if (showCostVals) {
		std::cout<<"slip cost: NaN   " << std::endl;
	}

	return 0;
}

double motionCostFunction(const std::vector<double> &x, std::vector<double> &grad, void *data){
	InternalMotionOptimizer* toOptimize = reinterpret_cast<InternalMotionOptimizer*>(data);

	return thetaCostFunction(x, grad, data)*toOptimize->weights.thetaWeight + 
		slipCostFunction(x, grad, data)*toOptimize->weights.slipWeight;
}

// constraints
double contactPlaneConstraints(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
	double SOSerror = 0;
	
	iOptimizerConstraintData *ci = reinterpret_cast<iOptimizerConstraintData*>(data);

	KinChain* kinchain = ci->optimizer->wTemplate->kinchain;
	//double currtime = ci->optimizer->alltimes[ci->tid];
	int tid = ci->tid;

	//for (int tid = 0; tid < ci->optimizer->alltimes.size(); ++tid) {
		double currtime = ci->optimizer->alltimes[tid];

		std::list<double> thetasAtT(x.begin()+tid*ci->optimizer->nDOF, x.begin()+(tid+1)*ci->optimizer->nDOF);
		kinchain->broadcastControlThetas(thetasAtT, currtime);
		//kinchain->updateTime(currtime);

		Eigen::Vector3d groundpoint, groundnorm;
		std::vector<Eigen::Vector3d> groundattach = kinchain->getAllContactPoints(currtime, KinChain::CONTACT);

		if (groundattach.size() > 3) {
			SOSerror += GeometricOperations::calcPlane(groundattach, groundpoint, groundnorm);
	
			/*
			// check all in a plane
			for (auto gp_it = groundattach.begin(); gp_it != groundattach.end(); ++gp_it) {
				double distFromPlane = groundnorm.dot((*gp_it)-groundpoint);

				if (showConstraintVals) {
					std::cout << "PTS: "<< (*gp_it).x() << " " << (*gp_it).y() << " " << (*gp_it).z()  << " --- " << distFromPlane << std::endl;
				}

				SOSerror += distFromPlane*distFromPlane;
			}
			*/
		
		}
	//}

	if (showConstraintVals) {
		std::cout << "SOS plane error = " << SOSerror << std::endl;
	}
	if (drawGeo) {
		Geometry * geo = kinchain->getGeometry();
		std::stringstream geoFileName;
		geoFileName << "..\\..\\data\\test\\transformedGeo.stl";
		geo->write(geoFileName.str());
	}
	

	return SOSerror;
}

double groundVBodyPlaneConstraints(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
	// returns max distance (positive = above plane) of contactpoint from body plane
	double error = -std::numeric_limits<double>::max();

	iOptimizerConstraintData *ci = reinterpret_cast<iOptimizerConstraintData*>(data);

	KinChain* kinchain = ci->optimizer->wTemplate->kinchain;
	double currtime = ci->optimizer->alltimes[ci->tid];
	
	std::list<double> thetasAtT(x.begin()+ci->tid*ci->optimizer->nDOF, x.begin()+(ci->tid+1)*ci->optimizer->nDOF);
	kinchain->broadcastControlThetas(thetasAtT, currtime);
	//kinchain->updateTime(currtime);
	
	// find contact points
	std::vector<Eigen::Vector3d> groundattach = kinchain->getAllContactPoints(currtime, KinChain::CONTACT);

	//std::cout << "BODYPOINT: "<< ci->optimizer->bodypoint.x() << " " << ci->optimizer->bodypoint.y() << " " << ci->optimizer->bodypoint.z()  << std::endl;
	//std::cout << "BODYNORM: "<< ci->optimizer->bodypoint.x() << " " << ci->optimizer->bodypoint.y() << " " << ci->optimizer->bodypoint.z()  << std::endl;

	if (drawGeo) {
		Geometry * geo = kinchain->getGeometry();
		std::stringstream geoFileName;
		geoFileName << "..\\..\\data\\test\\transformedGeo.stl";
		geo->write(geoFileName.str());
	}
	
	// check all in a plane
	for (auto gp_it = groundattach.begin(); gp_it != groundattach.end(); ++gp_it) {
		double distFromPlane = ci->optimizer->bodynorm.dot((*gp_it)-ci->optimizer->bodypoint);
		//std::cout << "PTS: "<< (*gp_it).x() << " " << (*gp_it).y() << " " << (*gp_it).z()  << " --- " << distFromPlane << std::endl;

		if (distFromPlane > error) {
			error = distFromPlane;
		}
	}

	if (showConstraintVals) {
		std::cout << "dist ground from body = " << error << std::endl;
	}
	
	return error;
}

double aboveGroundConstraints(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
	// returns max distance (positive = below plane) of joint from ground plane
	double error = -std::numeric_limits<double>::max();

	iOptimizerConstraintData *ci = reinterpret_cast<iOptimizerConstraintData*>(data);

	KinChain* kinchain = ci->optimizer->wTemplate->kinchain;
	double currtime = ci->optimizer->alltimes[ci->tid];
	
	std::list<double> thetasAtT(x.begin()+ci->tid*ci->optimizer->nDOF, x.begin()+(ci->tid+1)*ci->optimizer->nDOF);
	kinchain->broadcastControlThetas(thetasAtT, currtime);
	//kinchain->updateTime(currtime);
	
	// find ground plane
	Eigen::Vector3d groundpoint, groundnorm;
	std::vector<Eigen::Vector3d> groundattach = kinchain->getAllContactPoints(currtime, KinChain::CONTACT);
	GeometricOperations::calcPlane(groundattach, groundpoint, groundnorm);
	
	// every joint above ground
	for (auto nodeit = kinchain->getNodes().begin(); nodeit != kinchain->getNodes().end(); ++nodeit) {
		if ((*nodeit)->getType() == KinNode::JOINT) {
			double distFromPlane = -groundnorm.dot(((*nodeit)->getCenter().center)-groundpoint);

			if (distFromPlane > error) { error = distFromPlane; }
		}
	}
	
	std::vector<Eigen::Vector3d> otherpoints = kinchain->getAllContactPoints(currtime, KinChain::NONCONTACT);
	for (auto nodeit = otherpoints.begin(); nodeit != otherpoints.end(); ++nodeit) {
		double distFromPlane = -groundnorm.dot((*nodeit)-groundpoint);

		if (distFromPlane > error) { error = distFromPlane; }
	}

	if (showConstraintVals) {
		std::cout << "dist from ground = " << error << std::endl;
	}

	return error;
}


void outOfPlaneConstraints(unsigned m, double *result, unsigned n, const double* x, double *gradient, void*data) {
	int residx = 0;

	// returns max distance (positive = above plane) of contactpoint from body plane
	iOptimizerConstraintData *ci = reinterpret_cast<iOptimizerConstraintData*>(data);

	KinChain* kinchain = ci->optimizer->wTemplate->kinchain;
	double currtime = ci->optimizer->alltimes[ci->tid];
	
	std::list<double> thetasAtT(x+ci->tid*ci->optimizer->nDOF, x+(ci->tid+1)*ci->optimizer->nDOF);
	kinchain->broadcastControlThetas(thetasAtT, currtime);
	//kinchain->updateTime(currtime);
	
	// find ground plane
	Eigen::Vector3d groundpoint, groundnorm;
	std::vector<Eigen::Vector3d> groundattach = kinchain->getAllContactPoints(currtime, KinChain::CONTACT);
	GeometricOperations::calcPlane(groundattach, groundpoint, groundnorm);
	

	//std::cout << "BODYPOINT: "<< ci->optimizer->bodypoint.x() << " " << ci->optimizer->bodypoint.y() << " " << ci->optimizer->bodypoint.z()  << std::endl;
	//std::cout << "BODYNORM: "<< ci->optimizer->bodypoint.x() << " " << ci->optimizer->bodypoint.y() << " " << ci->optimizer->bodypoint.z()  << std::endl;

	if (drawGeo) {
		Geometry * geo = kinchain->getGeometry();
		std::stringstream geoFileName;
		geoFileName << "..\\..\\data\\test\\transformedGeo.stl";
		geo->write(geoFileName.str());
	}
	
	// every joint above ground
	for (auto nodeit = kinchain->getNodes().begin(); nodeit != kinchain->getNodes().end(); ++nodeit) {
		if ((*nodeit)->getType() == KinNode::JOINT) {
			double distFromPlane = -groundnorm.dot(((*nodeit)->getCenter().center)-groundpoint);

			result[residx] = distFromPlane;
			++residx;
		}
	}
	
	// all contacting contant points in a plane
	for (auto gp_it = groundattach.begin(); gp_it != groundattach.end(); ++gp_it) {
		double distFromPlane = ci->optimizer->bodynorm.dot((*gp_it)-ci->optimizer->bodypoint);
		//std::cout << "PTS: "<< (*gp_it).x() << " " << (*gp_it).y() << " " << (*gp_it).z()  << " --- " << distFromPlane << std::endl;

		result[residx] = distFromPlane;
		++residx;
	}
	
	// other noncontacting labeled contact points above ground
	std::vector<Eigen::Vector3d> otherpoints = kinchain->getAllContactPoints(currtime, KinChain::NONCONTACT);
	for (auto nodeit = otherpoints.begin(); nodeit != otherpoints.end(); ++nodeit) {
		double distFromPlane = -groundnorm.dot((*nodeit)-groundpoint);

		result[residx] = distFromPlane;
		++residx;
	}


	if (showConstraintVals) {
		std::cout << "dist ground from body = " ;
		for (int i = 0; i<m; ++i)
		{
			std::cout <<result[i] << "  ";
		}
		std::cout<<std::endl;
	}
	
	return;

}


double noMoveConstraints(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
	iOptimizerConstraintData *ci = reinterpret_cast<iOptimizerConstraintData*>(data);

	return x[ci->tid*ci->optimizer->nDOF+ci->DOFid]-x[(ci->tid+1)*ci->optimizer->nDOF+ci->DOFid];
}

double firstLastConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
	double SOSerror = 0;

	InternalMotionOptimizer *imo = reinterpret_cast<InternalMotionOptimizer*>(data);
	KinChain* kinchain = imo->wTemplate->kinchain;
	
	std::list<double> thetasAt0(x.begin(), x.begin()+imo->nDOF);
	kinchain->broadcastControlThetas(thetasAt0, imo->alltimes.front());
	
	std::list<double> thetasAtT(x.end()-imo->nDOF, x.end());
	kinchain->broadcastControlThetas(thetasAtT, imo->alltimes.back());

	// simplistic version: contact points are the same
	std::vector<Eigen::Vector3d> contact1 = kinchain->getAllContactPoints(0, KinChain::CONTACT);
	std::vector<Eigen::Vector3d> contact2 = kinchain->getAllContactPoints(1, KinChain::CONTACT);

	// assume they have the same number of points
	for (int cidx = 0; cidx < contact1.size(); ++cidx) {
		double dx = contact1[cidx].x() - contact2[cidx].x();
		double dy = contact1[cidx].y() - contact2[cidx].y();
		double dz = contact1[cidx].z() - contact2[cidx].z();

		SOSerror += dx*dx + dy*dy + dz*dz;
	}

	if (showConstraintVals) {
		std::cout << "first last contact error: " << SOSerror << std::endl;
	}

	return SOSerror;
}

void InternalMotionOptimizer::optimizeThetas(double &result) {
	KinChain* kinchain = wTemplate->kinchain;

	kinchain->clearGaitInfo();

	//Geometry * geo = kinchain->getGeometry();
	//CenterOfMass com = getCenterOfMass();

	int nappendages = this->appendages.size();
	int ncontactpts = kinchain->getAllContactPoints(0,KinChain::ALL).size();

	// SETUP OPTIMIZATION:
	nlopt::opt opt(nlopt::LN_COBYLA, nparam);
	//TODO: get actual min and max values
	lb.insert(lb.begin(), nparam, -2*PI);
	ub.insert(ub.begin(), nparam, 2*PI);
	opt.set_min_objective(motionCostFunction, (void*)(this));
	//iOptimizerConstraintData * datac = new iOptimizerConstraintData(this, 0, 0);
	//opt.set_min_objective(contactPlaneConstraints, (void*)(datac));

	for (int i = 0; i < alltimes.size()-1; i++){

		for (int j = 0; j<nDOF; ++j) {
			iOptimizerConstraintData * data = new iOptimizerConstraintData(this, i, j);
			
			// constraint information
			if (!ismoving[i*nDOF+j]) {
				// no movement between timesteps
				opt.add_equality_constraint(noMoveConstraints, (void*)(data), 1e-6);
			}
		}
		iOptimizerConstraintData * data = new iOptimizerConstraintData(this, i, 0);
		
		// contacts are in a plane
		opt.add_equality_constraint(contactPlaneConstraints, (void*)(data), 1e-6);
		
		// noncontact points are out of plane
		//opt.add_inequality_constraint(aboveGroundConstraints, (void*)(data)); // <= 0
		const vector<double> tol(nDOF+ncontactpts, 0);
		opt.add_inequality_mconstraint(outOfPlaneConstraints, (void*)(data), tol); // <= 0
		
		// ground plane is below body plane
		//opt.add_inequality_constraint(groundVBodyPlaneConstraints, (void*)(data)); // <= 0
		
	}

	// first and last contact points are the same
	opt.add_equality_constraint(firstLastConstraint, (void*)(this), 1e-6);
	opt.set_ftol_abs(0.01);
	opt.set_xtol_rel(0.01);
	//opt.set_maxtime(60);

	//optimization
	std::vector<double> x(origtheta);
	nlopt::result stopreason = opt.optimize(x, result);

	// PRINT RESULT
	std::cout<< "OUTPUT "<< std::endl;
	std::cout<<"stopreason: " << stopreason << std::endl;
	std::cout<<"cost: " << result << std::endl;
	for (int tid = 0; tid< alltimes.size(); ++tid) {
		for (int did = 0; did <nDOF; ++did) {
			std::cout<< x[tid*nDOF+did] <<"  ";
		}
		std::cout << std::endl;
	}

	// save to file for testing
	int i = 0;
	for (auto t_it = alltimes.begin(); t_it != alltimes.end(); ++t_it) {
		std::list<double> thetasAtT(x.begin()+i*nDOF, x.begin()+(i+1)*nDOF);
		kinchain->broadcastControlThetas(thetasAtT, *t_it);

		kinchain->updateTime(*t_it);
		Geometry * geo = kinchain->getGeometry();
		std::stringstream geoFileName;
		geoFileName << "..\\..\\data\\test\\animate" << i << ".stl";
		geo->write(geoFileName.str());
		++i;
	}

	// check constraints
	iOptimizerConstraintData * data = new iOptimizerConstraintData(this, 0, 0);
	showConstraintVals = true;
	vector<double> temp(1,1);
	contactPlaneConstraints(x, temp, data);
	std::vector<double> result(nDOF+ncontactpts);
	outOfPlaneConstraints((unsigned)nDOF+ncontactpts, (double*)(&(result[0])), nDOF*alltimes.size(), (double*)(&(x[0])), NULL, data);
	firstLastConstraint(x, temp, (void*)this);
	
	return;
}

void InternalMotionOptimizer::optimizeGait(double & result) {
}