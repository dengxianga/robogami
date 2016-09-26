#include "workingTemplate.h"
#include "constraintsEval.h"
#include "metrics.h"
#include "KinChain.h"
#include "ElectronicsGraph.h"
#include "util.h"
#include "FunctionEval.h"
#include "FunctionEvalStability.h"
#include "geometryOptimizer.h"
#include "MyOptimizer.h"
#include "template.h"
#include "FriendlyLinearSystem.h"
#include "ReducedEval.h"
#include <geometry.h>
#include "articulation.h"
#include "controller.h"
#include "GeometricOperations.h"

#define DEBUG_GERDIR false

#define DEBUG false
#define DEBUG_OPTIMIZE true

using namespace FabByExample;

WorkingTemplate::WorkingTemplate(Template* tmpl) {
	metrics = new Metrics(this);
	kinchain = new KinChain(tmpl);
	elecgraph = new ElectronicsGraph(kinchain);
	metricsInfo = new MetricsInfo();
	this->tmpl = tmpl;

	tmpl->recomputeCenter();
	tmpl->updateFullQ(tmpl->getFullQ());
}

WorkingTemplate::~WorkingTemplate(){
	delete this->metrics;
}

void WorkingTemplate::writeMeshToFile(std::string filename){
	tmpl->getGeometry()->write(filename);

}

Geometry* WorkingTemplate::getAnimation(double t){
	kinchain->updateTime(t);
	return kinchain->getGeometry();		
}


bool checkLineIntersection(const Eigen::Vector2d & a1, const Eigen::Vector2d& b1, const Eigen::Vector2d &a2, const Eigen::Vector2d &b2){
	Eigen::Matrix2d mat;
	mat.col(0) = (b1-a1);
	mat.col(1) = (a2-b2);
	Eigen::Vector2d vec = a2-a1;


	//std::cout << "a1 = " << a1 << std::endl;
	//std::cout << "a2 = " << a2 << std::endl;
	//std::cout << "b1 = " << b1 << std::endl;
	//std::cout << "b1 = " << b2 << std::endl;
	//std::cout << "mat = " << mat << std::endl;
	//std::cout << "vec = " << vec << std::endl;
	if(abs(mat.determinant()) < 0.00001){
		//std::cout << "zero det " << std::endl;
		return false;
	}
	Eigen::Vector2d t =  mat.inverse()*vec;
	//std::cout <<" t = " <<  t << std::endl;

	if (t.x() <0 || t.x() > 1){
		return false;
	}

	if (t.y() <0 || t.y() > 1){
		return false;
	}

	return true; 


}

bool checkVegeColision(point p1, point p2){
	
	Eigen::Vector2d sA(p1[0], p1[2]);
	Eigen::Vector2d sB(p2[0], p2[2]);



	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> cubecenters;
	
	
	cubecenters.push_back(Eigen::Vector2d(-300 , 0 ));
	cubecenters.push_back(Eigen::Vector2d(-300 , 400 ));
	cubecenters.push_back(Eigen::Vector2d(-100 , 200 ));
	cubecenters.push_back(Eigen::Vector2d(-500 , 200 ));
	cubecenters.push_back(Eigen::Vector2d(-300 , -400 ));
	cubecenters.push_back(Eigen::Vector2d(-100 , -200 ));
	cubecenters.push_back(Eigen::Vector2d(-500 , -200 ));

	Eigen::Vector2d side1(-50, -50);
	Eigen::Vector2d side2(-50, 50);
	Eigen::Vector2d side3(50, 50);
	Eigen::Vector2d side4(50, -50);

	for( int i = 0 ; i < cubecenters.size(); i++){
		if(checkLineIntersection(sA, sB, ( cubecenters[i] + side1), (cubecenters[i] + side2)))
			return true;
		if(checkLineIntersection(sA, sB, ( cubecenters[i] + side2), (cubecenters[i] + side3)))
			return true;
		if(checkLineIntersection(sA, sB, ( cubecenters[i] + side3), (cubecenters[i] + side4)))
					return true;

		if(checkLineIntersection(sA, sB, ( cubecenters[i] + side4), (cubecenters[i] + side1)))
					return true;

	}

	if(checkLineIntersection(sA, sB, Eigen::Vector2d(-1000, -1000), Eigen::Vector2d(-1000, -200)))
					return true;
	if(checkLineIntersection(sA, sB, Eigen::Vector2d(-1000, 1000), Eigen::Vector2d(-1000, 200)))
					return true;
	if(checkLineIntersection(sA, sB, Eigen::Vector2d(-1000, -1000), Eigen::Vector2d(0, -1000)))
						return true;
	if(checkLineIntersection(sA, sB, Eigen::Vector2d(-1000, 1000), Eigen::Vector2d(0, 1000)))
						return true;

	return false; 
}

bool checkLineColision( point p1, point p2){
	
	double maxHeight = 20;
	Eigen::Vector3d v1(p1[0],p1[1],p1[2]);
	Eigen::Vector3d v2(p2[0],p2[1],p2[2]);

	// get the part of the vector that is below the maxHeight
	if(v1.y() > v2.y()){
		v2 = v1;
		v1 = Eigen::Vector3d(p2[0], p2[1], p2[2]);
	}

	if (v1.y() > maxHeight){
		return false;
	}
	if( v2.y() > maxHeight){
		Eigen::Vector3d dir = v2 -v1;
		dir.normalize();
		// cutoff: (v1 + c*dir).y() = maxHeight
		if(dir.y() > 0){
			double c = (maxHeight - v1.y())/dir.y(); 
			v2 = v1 + c*dir;
		}
	}
	// check if the shorter vertor (v1,v2) crosses the 
	double minx = v2.x() < v1.x()? v2.x(): v1.x();
	double maxx = v2.x() > v1.x()? v2.x(): v1.x();

	if(minx >0){
		return false;
	}
	if(maxx < 0){
		return false;
	}
	return true;
}


bool checkCollision(Geometry* geo){

	TriMesh* mesh = geo->getMesh();
	bool collision = false;
	for (int i = 0; i < mesh->faces.size(); i++){		
		collision = collision || checkVegeColision(mesh->vertices[mesh->faces[i].v[0]], mesh->vertices[mesh->faces[i].v[1]]);
		collision = collision || checkVegeColision(mesh->vertices[mesh->faces[i].v[1]], mesh->vertices[mesh->faces[i].v[2]]);
		collision = collision || checkVegeColision(mesh->vertices[mesh->faces[i].v[2]], mesh->vertices[mesh->faces[i].v[0]]);
	}
	return collision;

}



double WorkingTemplate::getNewAnimation(bool useSteadyState, vector<Geometry*> & anim, bool dosequence, vector<double> &stabilityCost, bool computeColision, vector<bool> &colisionInfo){
	tmpl->updateFullQ(tmpl->getFullQ());
	//double delta = 0.01;
	//kinchain->updateGait(0.01, rounds);
	anim = kinchain->getAnimation(useSteadyState,1 , dosequence);	

	/*
	for (int i = 0; i < anim.size() ; ++i) {
		std::stringstream geoFileName;
		geoFileName << "..\\..\\data\\test\\animate" << i << ".stl";
		anim[i]->write(geoFileName.str());
	}
	*/

	if(computeColision){
		for each (auto geo in anim){
			colisionInfo.push_back(checkCollision(geo));
		}
	}
	stabilityCost = kinchain->getStabilityVector();

	/*
	//vector<Geometry*> animcopy(anim);
	vector<double> stabilitycopy(stabilityCost);
	double speed = kinchain->getSpeed();
	double nframes = anim.size();
	point transorig = anim[nframes-1]->getCenter()-anim[0]->getCenter();
	
	for (int iround = 1; iround < rounds; ++iround) {
		//anim.insert(anim.end(), animcopy.begin(), animcopy.end());
		point trans(transorig);
		trans[0]*=iround; trans[1]*=iround; trans[2]*=iround;
		
		for (int j = 0; j< nframes; ++j) {
			Geometry* newframe = new Geometry();
			newframe->add(anim[j]);
			newframe->applyTrans(trans);
			anim.push_back(newframe);
			//anim[iround*nframes+j]->applyTrans(trans);
		}
		
		stabilityCost.insert(stabilityCost.begin(), stabilitycopy.begin(), stabilitycopy.end());
	}*/

	return kinchain->getStability();
}


double WorkingTemplate::getViewGait(vector<Geometry*> & anim){
	tmpl->updateFullQ(tmpl->getFullQ());
	//double delta = 0.01;
	kinchain->clearGaitInfo();
	double deltaT = 0.01;
	for(double t = 0; t<= 1.0;  t=t+deltaT){
		kinchain->updateTime(t);
		Geometry * geo = kinchain->getGeometry();
		anim.push_back(geo);
	}

	vector<Geometry*> animcopy(anim);
	int rounds =2 ;
	for (int iround = 1; iround < rounds; ++iround) {
		anim.insert(anim.end(), animcopy.begin(), animcopy.end());
	}
	return 0;
}

//if negative, then 
double WorkingTemplate::getDirStability(TemplateElement* templateElement, int subElementId, 
	int axis, int gaitId, int objectiveId){
	
		double deltaTrans = 10; 
		double deltaScaleP = 10; 
		double deltaScaleM = -10; 

		if(objectiveId >0){

		double desiredCurvature = 0;
if(objectiveId < 4){
	gaitId --; 
}
  bool doSequence = false;
  if(gaitId == -1){
	  doSequence = true;
  }
  else{
	  kinchain->updateControllers(gaitId);
	  desiredCurvature = kinchain->getDesiredDirection(gaitId);
  }
	double delta = 0.01;
	int nrounds = 2;
	kinchain->setRounds(nrounds);
	Eigen::VectorXd currentQ =  tmpl->getFullQ();
	tmpl->updateFullQ(currentQ);

	if(DEBUG_GERDIR){
		VLOG(3) << "Computing direction for axis =" << axis << std::endl;

		tmpl->getGeometry()->write("..//..//data//meshDirDebug_before.stl");
	}


	kinchain->updateGait(delta, doSequence);
	double currentStability = kinchain->getObjectiveParam(objectiveId, doSequence, desiredCurvature);
	
	PatchPair p(nullptr, nullptr,false, false);
	if(axis>1){
		ConstraintsEval::updateTemplateBySubElementFaceTranslate(this->kinchain, templateElement, subElementId, axis-2, deltaTrans);
	}else{
		ConstraintsEval::updateTemplateBySubElementFaceScalingWithFixedAmount(templateElement, subElementId, axis, deltaScaleP, p);
	}

	kinchain->updateGait(delta, doSequence);
	double updateStability_front = kinchain->getObjectiveParam(objectiveId, doSequence, desiredCurvature);

	if(DEBUG_GERDIR){
		tmpl->getGeometry()->write("..//..//data//meshDirDebug_after1.stl");
	}

	tmpl->updateFullQ(currentQ);

	if(axis>1){
		ConstraintsEval::updateTemplateBySubElementFaceTranslate(this->kinchain, templateElement, subElementId, axis-2, -1.0*deltaTrans);
	}else{
		ConstraintsEval::updateTemplateBySubElementFaceScalingWithFixedAmount(templateElement, subElementId, axis, deltaScaleM, p);
	}

	kinchain->updateGait(delta, doSequence);
	double updateStability_back = kinchain->getObjectiveParam(objectiveId, doSequence, desiredCurvature);


	if(DEBUG_GERDIR){
		VLOG(3) << "the currentStability " << currentStability << std::endl;
		VLOG(3) << "the updateStability " << updateStability_front << std::endl;
		VLOG(3) << "the updateStability " << updateStability_back << std::endl;
		VLOG(3)<< "the direction values " <<updateStability_front - updateStability_back<< std::endl;
		tmpl->getGeometry()->write("..//..//data//meshDirDebug_after2.stl");
		system("pause"); 
	}


			//VLOG(3) << "the currentStability " << currentStability << std::endl;
		//VLOG(3) << "the updateStability " << updateStability_front << std::endl;
		//VLOG(3) << "the updateStability " << updateStability_back << std::endl;
		//VLOG(3)<< "the direction values " <<updateStability_front - updateStability_back<< std::endl;
	tmpl->updateFullQ(currentQ);

	double movingBack = updateStability_back - currentStability;
	double movingFront = updateStability_front - currentStability;

	if( (movingBack < 0.01) && (movingFront <0.01)){
		//std::cout << "both directions are bad!! " << std::endl;
		return 0; // neither direction helps
	}
	if( movingBack > movingFront)
		return movingBack/(abs(currentStability)); 
	if( movingFront > movingBack)
		return (-1.0)*movingFront/(abs(currentStability)); 

	return   (updateStability_back - updateStability_front)/(abs(currentStability)); 
}
		return 0; 
}

void WorkingTemplate::computeMetrics(int Nrounds, double delta, bool doSequence =false){

	metricsInfo->fabCost = metrics->getFabricationCost();
	metricsInfo->electronicsCost = metrics->getElecCost();

	//double delta = 0.1;
	//std::vector<double> stabilityCost;
	//tmpl->updateFullQ(tmpl->getFullQ());
	kinchain->setRounds(Nrounds);
	kinchain->updateGait(delta, doSequence);
	//std::vector<Geometry*> anim = kinchain->getAnimation();
	metricsInfo->speed = kinchain->getSpeed(doSequence);
	
	double initialStability = kinchain->getStability();
	metricsInfo->isStable = true;
	double epsilon = 0.02;
	if (initialStability > epsilon)
		metricsInfo->isStable = false;

	metricsInfo->error = initialStability;
	
	metricsInfo->slip = kinchain->getSlip(doSequence);

	metricsInfo->perperror = kinchain->getPerpendicularError();

	metricsInfo->travel = kinchain->getForwardTravel();
	
	metricsInfo->path = kinchain->getSSPath(doSequence);

	metricsInfo->curvature = kinchain->getMeanCurvature();

	metricsInfo->meanangle = kinchain->getMeanAngle();

	metricsInfo->rotation = (-1)*kinchain->getSSRotation();
	
	metricsInfo->toppling = kinchain->getToppling(doSequence); 

	metricsInfo->totalMass = metrics->getTotalMass();

}


std::vector<std::pair<double,double>> WorkingTemplate::getTopView(){
		std::vector<std::pair<double,double>> topView;
		Geometry* geo = this->tmpl->getGeometry();
		std::vector<Eigen::Vector3d> points; 
		geo->addAllPoints(points);
		std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>  contactPoints;
		for each (auto p in points){
			contactPoints.push_back(Eigen::Vector2d(p.x(), p.z()));
		}

		std::vector<int> hull;
		GeometricOperations::computeConvexHull(contactPoints, hull);
		for each (int i in hull){
			topView.push_back(std::pair<double,double>(contactPoints[i].x(), contactPoints[i].y()));
		}
		//delete geo;
		return topView;

}

void WorkingTemplate::getDiniteDiffStability(double eps, Eigen::VectorXd & grad){

	Eigen::VectorXd currentQ = tmpl->getFullQ();
	tmpl->updateFullQ(currentQ);
	double delta = 0.1;
	//std::vector<double> stabilityCost;
	//kinchain = new KinChain(templateElement->getRoot());
	kinchain->updateGait(0.1);
	//std::vector<Geometry*> anim = kinchain->getAnimation();
	double currentStability = kinchain->getStability();

	grad = Eigen::VectorXd::Zero(currentQ.size());

	for (int i = 0; i < currentQ.size(); i++){
		//LOG(INFO) << "i = " << i << std::endl;
		tmpl->updateFullQ(currentQ);
		ConstraintsEval::updateTempByChangingOneParam(tmpl, i, -eps);
		kinchain->updateGait(0.1);
		//tmpl->getGeometry()->write("..\\..\\data\\test\\geo_plus.stl");
		double minStability = kinchain->getStability();
		tmpl->updateFullQ(currentQ);
		ConstraintsEval::updateTempByChangingOneParam(tmpl, i, eps);
		kinchain->updateGait(0.1);
		//tmpl->getGeometry()->write("..\\..\\data\\test\\geo_minus.stl");
		double maxStability = kinchain->getStability();


		//system("pause");
		grad(i) = maxStability - minStability;
		VLOG(3) << "grad(" << i << ")  max = " << maxStability << std::endl;
		VLOG(3) << "grad(" << i << ") min = " << minStability<< std::endl;
		
	}

}


void WorkingTemplate::evalStabilization(){
	
	tmpl->getGeometry()->write("..\\..\\data\\test\\geo_current.stl");
	Eigen::VectorXd oldQ = tmpl->getFullQ();
	double delta = 0.1;
	//std::vector<Geometry*> anim;
	//std::vector<double> stabilityCost;

	clock_t begin_time_full = clock();

	kinchain->updateGait(0.1);
	double initialStability = kinchain->getStability();
	VLOG(3)  << "initial  stability =  " << initialStability << std::endl;
	
	//point p0 = anim[0]->getCenter();
	//point pf = anim[anim.size() -1]->getCenter();
	//Eigen::Vector2d v0(p0[0], p0[2]);
	//Eigen::Vector2d vf(pf[0], pf[2]);
	//VLOG(3)  << v0.transpose() << std::endl;
	//VLOG(3)  << vf.transpose() << std::endl;
	double speed = kinchain->getSpeed(); //(v0 - vf).norm();
	VLOG(3)  << "initial spped = " << speed << std::endl;
	std::cout << "computed intial stuff in " << double( clock() - begin_time_full ) / (double)CLOCKS_PER_SEC << " seconds!" << std::endl; 

	//system("pause");


	clock_t begin_time_eval = clock();

	double currentStability; 

//	stabilize(FunctionEvalStability::StabilizationObjective::NONE);
//	tmpl->getGeometry()->write("..\\..\\data\\test\\optimized_none.stl");
//	
//	VLOG(3) <<"done optimization NONE" << std::endl;
//	currentStability = kinchain->getAnimation(anim, stabilityCost, 0.1, false, 1);
//	p0 = anim[0]->getCenter();
//	pf = anim[anim.size() -1]->getCenter();
//	v0 = Eigen::Vector2d(p0[0], p0[2]);
//	vf = Eigen::Vector2d(pf[0], pf[2]);
//	speed = (v0 - vf).norm();
//	VLOG(3)  << "final spped = " << speed << std::endl;
//	VLOG(3) << "final stability =  " << currentStability << std::endl;
//
	tmpl->updateFullQ(oldQ);
	stabilize(FunctionEvalStability::StabilizationObjective::GEOMETRY);
	tmpl->getGeometry()->write("..\\..\\data\\test\\optimized_geo.stl");
	
	VLOG(3)<<"done optimization GEOMETRY" << std::endl;
	kinchain->updateGait(0.1);
	currentStability = kinchain->getStability();
	//p0 = anim[0]->getCenter();
	//pf = anim[anim.size() -1]->getCenter();
	//v0 = Eigen::Vector2d(p0[0], p0[2]);
	//vf = Eigen::Vector2d(pf[0], pf[2]);
	speed = kinchain->getSpeed(); //(v0 - vf).norm();
	VLOG(3)  << "final spped = " << speed << std::endl;
	VLOG(3) << "final stability =  " << currentStability << std::endl;

//	tmpl->updateFullQ(oldQ);
//	stabilize(FunctionEvalStability::StabilizationObjective::FABCOST);
//	tmpl->getGeometry()->write("..\\..\\data\\test\\optimized_fabcost.stl");
//	
//	VLOG(3) <<"done optimization FABCOST" << std::endl;
//	currentStability = kinchain->getAnimation(anim, stabilityCost, 0.1, false, 1);
//	p0 = anim[0]->getCenter();
//	pf = anim[anim.size() -1]->getCenter();
//	v0 = Eigen::Vector2d(p0[0], p0[2]);
//	vf = Eigen::Vector2d(pf[0], pf[2]);
//	speed = (v0 - vf).norm();
//	VLOG(3)  << "final spped = " << speed << std::endl;
//	VLOG(3) << "final stability =  " << currentStability << std::endl;

	tmpl->updateFullQ(oldQ);
	stabilize(FunctionEvalStability::StabilizationObjective::SPEED);
	tmpl->getGeometry()->write("..\\..\\data\\test\\optimized_speed.stl");

	VLOG(3) <<"done optimization SPEED" << std::endl;
	kinchain->updateGait(0.1);
	currentStability = kinchain->getStability();
	//p0 = anim[0]->getCenter();
	//pf = anim[anim.size() -1]->getCenter();
	//v0 = Eigen::Vector2d(p0[0], p0[2]);
	//vf = Eigen::Vector2d(pf[0], pf[2]);
	speed = kinchain->getSpeed(); //(v0 - vf).norm();
	VLOG(3)  << "final spped = " << speed << std::endl;
	VLOG(3) << "final stability =  " << currentStability << std::endl;
	tmpl->updateFullQ(oldQ);
}

GeometryResults WorkingTemplate::callOptimization(GeometryOptimizationWeights weights, bool& success) {
	if (DEBUG) {
		kinchain->updateGait(0.01);
		std::cout << "the stability is " << kinchain->getStability() << std::endl;
		std::cout << "the slip is " << kinchain->getSlip() << std::endl;
		std::cout << "the speed is " << kinchain->getSpeed() << std::endl;
		std::cout << "the perperror is " << kinchain->getPerpendicularError() << std::endl;
		std::cout << "the travel is " << kinchain->getForwardTravel() << std::endl;
	}
	GeometryResults results;

	try {
		GeometryOptimizer opt(this, weights);
		success = opt.optimize();

		kinchain->updateGait(0.01);
	
		if (kinchain->getStability() > 0 && success) {// do it again (NOTE: usually converges in 2 tries)
			if (DEBUG_OPTIMIZE) {
				std::cout << "Unstable (" << kinchain->getStability() << "), trying again...." << std::endl;
			}
			GeometryOptimizer opt2(this, weights);
			success = opt2.optimize();
		}

		kinchain->updateGait(0.01);
		results.params = getTemplate()->getFullQ();
		results.stability = kinchain->getStability();
		results.slip = kinchain->getSlip();
		results.speed = kinchain->getSpeed();
		results.perperror = kinchain->getPerpendicularError();
		results.travel = kinchain->getForwardTravel();
		results.curvature = kinchain->getMeanCurvature();
		results.wobbliness = kinchain->getToppling();
		Eigen::VectorXd redparams;
		opt.redEval->getT(results.params,redparams);
		results.geometryCost = opt.computeGeometryCost(redparams);
		if (DEBUG) {
			std::cout << "the stability is " << kinchain->getStability() << std::endl;
			std::cout << "the slip is " << kinchain->getSlip() << std::endl;
			std::cout << "the speed is " << kinchain->getSpeed() << std::endl;
			std::cout << "the geometry cost is " << results.geometryCost << std::endl;
			std::cout << "the perperror is " << kinchain->getPerpendicularError() << std::endl;
			std::cout << "the travel is " << kinchain->getForwardTravel() << std::endl;
		}	

		// revert and return results
		getTemplate()->updateFullQ(opt.origConfiguration);

	} catch (int e) {
		success = false;
	}

	return results;
}

bool WorkingTemplate::stabilize(FunctionEvalStability::StabilizationObjective stabObjective ){

	GeometryOptimizationWeights weights(10, 0, 0, 0, 0, 0, 0, 0);

	int minOrMax = 1; // 1 for min, -1 for max
	switch(stabObjective) {
		case FunctionEvalStability::StabilizationObjective::GEOMETRY : {
			weights.stabilityWeight = 0;
			weights.geometryWeight = 10;
			} break;
		case FunctionEvalStability::StabilizationObjective::SPEED : {
			weights.stabilityWeight = 0;
			weights.speedWeight = 10;
			weights.geometryWeight = 0.001; 
			minOrMax = -1;
			} break;
		case FunctionEvalStability::StabilizationObjective::PERPENDICULAR_ERROR : {
			weights.stabilityWeight = 0;
			weights.straightnessWeight = 10;
			} break;
		case FunctionEvalStability::StabilizationObjective::TRAVEL : {
			weights.stabilityWeight = 0;
			weights.travelWeight = 10;
			minOrMax = 1;
			} break;
		case FunctionEvalStability::StabilizationObjective::CURVATURE : {
			weights.stabilityWeight = 0;
			weights.curvatureWeight = 10;
			weights.speedWeight = .1;
			minOrMax = 1;
			} break;
		case FunctionEvalStability::StabilizationObjective::WOBBLINESS : {
			weights.stabilityWeight = 0;
			weights.wobblyWeight = 10;
			weights.curvatureWeight = 0.1;
			weights.speedWeight = 0.1;
			weights.geometryWeight = 0.01; 
			} break;
		case FunctionEvalStability::StabilizationObjective::NONE : {
			weights.stabilityWeight = 50;
			} break;
	}

	return this->stabilize(weights);
}

bool WorkingTemplate::stabilize(GeometryOptimizationWeights weights){

	kinchain->updateGait(0.01);
	double oldStability = kinchain->getStability() ;
	double oldSlip = kinchain->getSlip();
	double oldSpeed = kinchain->getSpeed();
	double oldStraightness = kinchain->getPerpendicularError();
	double oldTravel = kinchain->getForwardTravel();
	double oldCurvature = kinchain->getMeanCurvature();
	double oldWobbliness = kinchain->getToppling();

	std::cout << "stability weight: " << weights.stabilityWeight << std::endl;
	std::cout << "slip weight: " << weights.slipWeight << std::endl;
	std::cout << "speed weight: " << weights.speedWeight << std::endl;
	std::cout << "geometry weight: " << weights.geometryWeight << std::endl;
	std::cout << "straightness weight: " << weights.straightnessWeight << std::endl;
	std::cout << "travel weight: " << weights.travelWeight << std::endl;
	std::cout << "curvature weight: " << weights.curvatureWeight << std::endl;
	std::cout << "wobbly weight: " << weights.wobblyWeight << std::endl;

	
	
	// THREE TRIALS
	GeometryResults minresults;
	minresults.params = getTemplate()->getFullQ();
	minresults.geometryCost = 0;
	minresults.slip= oldSlip;
	minresults.stability = oldStability;
	minresults.speed = oldSpeed;
	minresults.perperror = oldStraightness;
	minresults.travel = oldTravel;
	minresults.curvature = oldCurvature;
	minresults.wobbliness = oldWobbliness;

	int numTrials = 1;

	bool success = false;

	for (int i = 0; i < numTrials; ++i) {

		if (DEBUG_OPTIMIZE){
			std::cout <<"TRIAL " << i << std::endl;
		}

		bool isuccess = false;

		GeometryResults iresults = callOptimization(weights, isuccess);

		// compare results
		if (isuccess && iresults.getMetric(FunctionEvalStability::StabilizationObjective::NONE) < 0) {
			if ((iresults.getMetric(weights) < minresults.getMetric(weights)) ||
				(minresults.getMetric(FunctionEvalStability::StabilizationObjective::NONE) > 0)){
				if (DEBUG_OPTIMIZE) {
					std::cout << "BETTER SOLUTION: metric is " << iresults.getMetric(weights) << " compared to " << minresults.getMetric(weights) << std::endl;
				}
				minresults = iresults;
				success = true;
			} else {
				if (DEBUG_OPTIMIZE) {
					std::cout << "WORSE SOLUTION: metric is " << iresults.getMetric(weights) << " compared to " << minresults.getMetric(weights) << std::endl;
				}

			}
		} else {
			if (DEBUG_OPTIMIZE) {
				std::cout << "UNSTABLE SOLUTION: metric is " << iresults.getMetric(weights) << " compared to " << minresults.getMetric(weights) << std::endl;
			}
		}
		minresults = iresults;
		success = true;

		kinchain->updateGait(0.01);
	}

	getTemplate()->updateFullQ(minresults.params);

	return success;
}

bool WorkingTemplate::stabilize_old(FunctionEvalStability::StabilizationObjective stabObjective ){

	clock_t begin_time_full = clock();

	FlatLinearSystem flat;
	Eigen::VectorXd oldQ = tmpl->getFullQ(); 
	ConstraintsEval::extractFeasibilityConstraints(this->tmpl, &flat); 
	ReducedEval* redEval = new ReducedEval(flat.Aeq_red, flat.Beq_red, flat.Aineq, flat.Bineq, oldQ); 
	Eigen::VectorXd oldT;
	redEval->getT(oldQ, oldT);

	bool  feasibility = redEval->evalFeasibility(oldT);
	//std::cout << "feasibility = " << feasibility << std::endl;
	if(!feasibility){
		std::cout << "the initial point is not feasible" << std::endl; 
		Eigen::VectorXd newT;
		redEval->getClosestFeasiblePoint(oldT, newT);
		oldT = newT;
		Eigen::VectorXd fixQ;
		redEval->getQ(oldT, fixQ);
		getTemplate()->updateFullQ(fixQ);
		VLOG(3) << "is going to save the file" << std::endl;
		getTemplate()->getGeometry()->write("..\\..\\data\\test\\geo_current_fixed.stl");
		VLOG(3) << "saved!" << std::endl;
	}


	//step1: compute the gradient
	Eigen::VectorXd grad = Eigen::VectorXd::Zero(oldT.size());

	double delta = 0.1;
	//std::vector<Geometry*> anim;
	//std::vector<double> stabilityCost;
	//kinchain = new KinChain(templateElement->getRoot());
	double currentStability = kinchain->getStability();

	FunctionEvalStability *f = new FunctionEvalStability(this, redEval, stabObjective,flat.Aopt, flat.Bopt);

	int niter = 0;
	while ( (currentStability >0) && (niter <7)){


		bool  feasibility = redEval->evalFeasibility(oldT);
		//std::cout << "feasibility = " << feasibility << std::endl;
		if(!feasibility){
			std::cout << "the initial point is not feasible" << std::endl; 
			Eigen::VectorXd newT;
			redEval->getClosestFeasiblePoint(oldT, newT);
			oldT = newT;
		}

		//compute gradient
		double epsilon = 1;
		for (int i = 0; i < grad.size(); i++){
			//LOG(INFO) << "i = " << i << std::endl;
			double minEps, maxEps, otherRange;
			Eigen::VectorXd dir = Eigen::VectorXd::Zero(oldT.size());
			dir(i) = 1; 
			redEval->getFeasibleRange(oldT, dir, &otherRange, &maxEps);
			dir(i) = -1; 
			redEval->getFeasibleRange(oldT, dir, &otherRange, &minEps);
			//VLOG(3) << "minEps ("<< i << ") = "<< minEps << std::endl;
			//VLOG(3) << "maxEps ("<< i << ") = "<< maxEps << std::endl;
			
				Eigen::VectorXd t_left_opt = oldT; t_left_opt(i) += 0.1;// maxEps/3;
				Eigen::VectorXd t_right_opt = oldT; t_right_opt(i) -= 0.1;//minEps/3;
			if(1){
				t_left_opt = oldT; t_left_opt(i) +=  maxEps/3;
				t_right_opt = oldT; t_right_opt(i) -= minEps/3;
			}
			Eigen::VectorXd t_left = t_left_opt;
			if(!redEval->evalFeasibility(t_left)){
				 redEval->getClosestFeasiblePoint(t_left_opt, t_left);
				 std::cout << "is infeasible left! " << std::endl;
	 			 system( "pause"); 
			}
			Eigen::VectorXd t_right = t_right_opt;
			if(!redEval->evalFeasibility(t_right)){
				redEval->getClosestFeasiblePoint(t_right_opt, t_right);
				 std::cout << "is infeasible right ! " << std::endl;
				 system( "pause"); 
			}
			//VLOG(3) << "oldT = "<< oldT.transpose() << std::endl;
			//VLOG(3) << "t_left = "<< t_left.transpose() << std::endl;
			//VLOG(3) << "t_right = "<< t_right.transpose() << std::endl;

			Eigen::VectorXd minQ, maxQ;
			redEval->getQ(t_right, maxQ);
			redEval->getQ(t_left, minQ);

			tmpl->updateFullQ(minQ);
			double minStability = f->computeStabilityObjective(); // kinchain->getAnimation(anim, stabilityCost, 0.1, false, 1);
			tmpl->updateFullQ(maxQ);
			double maxStability = f->computeStabilityObjective(); // kinchain->getAnimation(anim, stabilityCost, 0.1, false, 1);

			grad(i) = maxStability - minStability;
			//VLOG(3) << "grad(" << i << ")  max = " << maxStability << std::endl;
			//VLOG(3) << "grad(" << i << ") min = " << minStability<< std::endl;
			//VLOG(3) << "grad(" << i << ") = " << grad(i) << std::endl;
			//system("pause");
		}

		if( grad.norm() > 0.01){

			grad = grad/grad.norm();
			//VLOG(7) << "grad = " << grad.transpose() << std::endl;		
			double minRange, maxRange;
			redEval->getFeasibleRange(oldT, grad, &minRange, &maxRange);
		

			// line search
			f->setStartingPoint(oldT);
			f->setDir(grad);
			double resultVal;
			double sol;
			minRange = 0;
			maxRange = maxRange/2;
			//VLOG(7) << "max range " << maxRange << std::endl;
			//VLOG(7) << "min range " << minRange << std::endl;
			MyOptimizer::lineSearch(f, minRange, maxRange, (maxRange - minRange)/10000, &resultVal, &sol);

			//update
			 //std::cout << "the solution id " << sol << std::endl;
			 Eigen::VectorXd optT = oldT + sol*grad; 
			 Eigen::VectorXd optQ;
			 redEval->getQ(optT, optQ);
			 getTemplate()->updateFullQ(optQ);

			 kinchain->updateGait(0.1);
			 currentStability = kinchain->getStability();

			 oldT = optT;

			 //VLOG(3) << "Iter = " << niter << " -> stability = " << currentStability << std::endl;
		}else{
			VLOG(7) << "gradient is zero" << std::endl;
		}
	
		 niter++;
	}

	if((currentStability > 0.2) && (stabObjective != FunctionEvalStability::StabilizationObjective::NONE)){
		VLOG(3) << "COULD NOT FIND FEASIBLE POINT SO RUNING WITH NO OTHER OPT " << std::endl;
		stabilize(FunctionEvalStability::StabilizationObjective::NONE);
		//getTemplate()->updateFullQ(oldQ);
		return false;
	}

	std::cout << "computed stabilization in " << double( clock() - begin_time_full ) / (double)CLOCKS_PER_SEC << " seconds!" << std::endl; 

	return true;
}

static bool alphaValueComparitor(LinearExpr &A, LinearExpr &B){
	return A.eval(SymbolicAssignment::USE_CURRENT_VALUES) < B.eval(SymbolicAssignment::USE_CURRENT_VALUES);
}


bool evalConnectedLegs(Template* t1, Template *t2){

	std::vector<NewPatch*> servoPatches1;
	for each( auto p in t1->getAllPatches()){
		if(dynamic_cast<ServoPointPatch*>(p) != nullptr){
			servoPatches1.push_back(p);
		}
	}
	std::vector<NewPatch*> servoPatches2;
	for each( auto p in t2->getAllPatches()){
		if(dynamic_cast<ServoPointPatch*>(p) != nullptr){
			servoPatches2.push_back(p);
		}
	}

	
	Template* root = t1->getRoot();
	std::vector<NewConnection*> connections;
	for each (auto t in root->getTemplatesByScope(TreeScope::DESCENDANTS)){
		auto newConn = root->getConnections();
		connections.insert(connections.end(), newConn.begin(), newConn.end());
	}
	bool isConnected = false;
	for each (auto s1 in servoPatches1){
		for each (auto s2 in servoPatches2){
			for each (auto c in connections){
				auto patches = c->getPatches();
				if(std::find(patches.begin(), patches.end(), s1) != patches.end()){
					if(std::find(patches.begin(), patches.end(), s2) != patches.end()){
						isConnected = true;
					}
				}
			}
		}
	}
	
	return isConnected; 

}

void WorkingTemplate::equalizeLinkLength(){

	std::vector<Template*> legParts;
	for each (auto t in tmpl->getTemplatesByScope(TreeScope::DESCENDANTS)){
		if (t->getPartType() == Semantics::LEG){
			legParts.push_back(t);
		}
	}

	//find the linkages
	std::vector<std::vector<Template*>> linkLegs;
	for each (auto t in legParts){
		bool foundConnectedLinkage = false;
		for (int l = 0; l < linkLegs.size(); l++){
			bool isConnected = false;
			for each (auto link in linkLegs[l]){
				if(evalConnectedLegs(t, link)){
					isConnected = true;
				}
			}
			if(isConnected){
				if(foundConnectedLinkage){
					VLOG(3) << "found that is is connected to multiple linkages ";
					system("pause");
				}
				foundConnectedLinkage = true;
				linkLegs[l].push_back(t);
			}
		}
		if(!foundConnectedLinkage){
			std::vector<Template*> newLinkage;
			newLinkage.push_back(t);
			linkLegs.push_back(newLinkage);
		}
	}

	for each( auto linkage in linkLegs){
		if (linkage.size() >1){
			std::vector<LinearExpr> lengths;
			for each (auto t in linkage){
				auto bbox = t->evalLocalBoundingBox();
				lengths.push_back(bbox.max.y - bbox.min.y);
			}
			for(int i =1; i < lengths.size(); i++){
				Constraint * new_constraint = new Constraint(lengths[i] - lengths[i-1]);
				new_constraint->setConstraintType(Constraint::ConstraintType::SYMM_LEGL); 
				tmpl->addConstraint(new_constraint);		
			}
		}
	}

}

void WorkingTemplate::equalizeLegWidth(){

	std::vector<LinearExpr> widths;
	std::vector<LinearExpr> depths;
	for each (auto t in tmpl->getTemplatesByScope(TreeScope::DESCENDANTS)){
		if (t->getPartType() == Semantics::LEG){
			auto bbox = t->evalLocalBoundingBox();
			LinearExpr current_w = (bbox.max.x - bbox.min.x);
			LinearExpr current_d = (bbox.max.z - bbox.min.z);
			widths.push_back(current_w);
			depths.push_back(current_d);
		}
	}
	for(int i =1; i < widths.size(); i++){
		double eps = 0.1;
		LinearExpr constraint = widths[i] - widths[i-1];
		Constraint * new_constraint1 = new Constraint(Constraint::ConstraintRelation::INEQ, constraint -eps);
		Constraint * new_constraint2 = new Constraint(Constraint::ConstraintRelation::INEQ, (-1.0)*constraint -eps);
		new_constraint1->setConstraintType(Constraint::ConstraintType::SYMM_LEGW); 
		new_constraint2->setConstraintType(Constraint::ConstraintType::SYMM_LEGW); 
		tmpl->addConstraint(new_constraint1);		
		tmpl->addConstraint(new_constraint2);		
	}
	for(int i =1; i < depths.size(); i++){


		double eps = 0.1;
		LinearExpr constraint = depths[i] - depths[i-1];
		Constraint * new_constraint1 = new Constraint(Constraint::ConstraintRelation::INEQ, constraint -eps);
		Constraint * new_constraint2 = new Constraint(Constraint::ConstraintRelation::INEQ, (-1.0)*constraint -eps);
		new_constraint1->setConstraintType(Constraint::ConstraintType::SYMM_LEGW); 
		new_constraint2->setConstraintType(Constraint::ConstraintType::SYMM_LEGW); 
		tmpl->addConstraint(new_constraint1);		
		tmpl->addConstraint(new_constraint2);		
	}

	
}

void WorkingTemplate::makeUniformSpacing(){
	std::vector<ServoLinePatch*> servopatches;
	
	for each( auto p in tmpl->getAllPatches()){
		if(dynamic_cast<ServoLinePatch*>(p) != nullptr){
			servopatches.push_back(dynamic_cast<ServoLinePatch*>(p));
		}
	}

	for each (auto p in servopatches){
		Point3S l1 = p->getVertex1();
		Point3S l2 = p->getVertex2();
		Eigen::Vector3d l1_p = l1.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
		Eigen::Vector3d l2_p = l2.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
		Eigen::Vector3d dir = (l1_p - l2_p).normalized();
		LinearExpr paramlen =  dir.x()*(l1.x - l2.x) + dir.y()*(l1.y - l2.y) + dir.z()*(l1.z - l2.z); 
		
		std::vector<LinearExpr> alphaValues;
		for each ( auto s in p->spacings){
			alphaValues.push_back(s.alpha);
		}
		std::sort(alphaValues.begin(), alphaValues.end(), alphaValueComparitor);
		if(alphaValues.size() >0){
			if(alphaValues.size() == 1){
				LinearExpr constraint = alphaValues[0] - 0.5*paramlen;
				constraint.print(); //system("pause"); 
				Constraint * new_constraint = new Constraint(constraint);
				new_constraint->setConstraintType(Constraint::ConstraintType::SYMM_SPACING); 
				tmpl->addConstraint(new_constraint);		
			}else{
				double D = 1.0/(alphaValues.size()-1);
				for(int i = 0; i < alphaValues.size(); i++){
					LinearExpr constraint = alphaValues[i] - i*D*paramlen; 
					constraint.print();// system("pause"); 
					//Constraint * new_constraint = new Constraint(constraint);
					//new_constraint->setConstraintType(Constraint::ConstraintType::SYMM_SPACING); 
					//tmpl->addConstraint(new_constraint);		

					double eps = 0.1;
					Constraint * new_constraint1 = new Constraint(Constraint::ConstraintRelation::INEQ, constraint -eps);
					Constraint * new_constraint2 = new Constraint(Constraint::ConstraintRelation::INEQ, (-1.0)*constraint -eps);
					new_constraint1->setConstraintType(Constraint::ConstraintType::SYMM_SPACING); 
					new_constraint2->setConstraintType(Constraint::ConstraintType::SYMM_SPACING); 
					tmpl->addConstraint(new_constraint1);		
					tmpl->addConstraint(new_constraint2);		

				}
			}
		}
		
	}
	
}


static bool contactPointComparitor(ContactPoint* A, ContactPoint* B){
	return A->cPoint.y.eval(SymbolicAssignment::USE_CURRENT_VALUES) < B->cPoint.y.eval(SymbolicAssignment::USE_CURRENT_VALUES);
}

void WorkingTemplate::contrainAllContactPoints(){
	vector<ContactPoint*> contactPoints;// = tmpl->getOneContactPointsForEachPart();

	for each (auto child in this->kinchain->getRoot()->children){
		std::vector<ContactPoint*> singleContact;
		child->getAllContactPoints(singleContact);
		if(singleContact.size() > 0){
			std::sort(singleContact.begin(), singleContact.end(), contactPointComparitor); 
			contactPoints.push_back(singleContact[0]);
		}
	}

	double debug = false;
	if(debug){
		Geometry* geo  = tmpl->getGeometry();
		for each (auto c in contactPoints){
			Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));	
			sphere->applyTrans(c->cPoint.evalpoint(SymbolicAssignment::USE_CURRENT_VALUES));
			geo->add(sphere);
		}
		geo->write("..\\..\\data\\contactPointsToGround.stl");
	}
	for each (auto c in contactPoints){
//		if (!c->isConstrained){
//			c->isConstrained = true;



		double eps = 0.1;
		LinearExpr EqualilyConstraint = c->cPoint.y;
		Constraint * new_constraint1 = new Constraint(EqualilyConstraint);
		//Constraint * new_constraint1 = new Constraint(Constraint::ConstraintRelation::INEQ, EqualilyConstraint -eps);
		//Constraint * new_constraint2 = new Constraint(Constraint::ConstraintRelation::INEQ, (-1.0)*EqualilyConstraint -eps);
		new_constraint1->setConstraintType(Constraint::ConstraintType::SYMM_GROUND); 
		//new_constraint2->setConstraintType(Constraint::ConstraintType::SYMM_GROUND); 
		tmpl->addConstraint(new_constraint1);
		//tmpl->addConstraint(new_constraint2);
	}
	
}


void WorkingTemplate::updateSymmetryChoices(bool symm_ground, bool symm_legW, bool symm_legL, bool symm_spacing){
	
	bool constraintsAdded = false;
	
	// snap to ground
	if((!symm_ground) || (tmpl->symmetryChoices.symm_ground != symm_ground)){
		for each (auto t in tmpl->getTemplatesByScope(TreeScope::DESCENDANTS)){
			t->removeAllConstraintsOfType(Constraint::ConstraintType::SYMM_GROUND);
		}
		if(symm_ground == true){
			constraintsAdded = true;
			contrainAllContactPoints();
			ConstraintsEval::enforceConstraintsSymmetry(tmpl, false);	
		}
	}
	// equalize leg width
	if((!symm_legW) || (tmpl->symmetryChoices.symm_legW != symm_legW)){
		for each (auto t in tmpl->getTemplatesByScope(TreeScope::DESCENDANTS)){
			t->removeAllConstraintsOfType(Constraint::ConstraintType::SYMM_LEGW);
		}
		if(symm_legW == true){
			constraintsAdded = true;
			equalizeLegWidth();
			ConstraintsEval::enforceConstraintsSymmetry(tmpl, false);	

		}
	}

	//equalize leg length	
	if((!symm_legL) || (tmpl->symmetryChoices.symm_legL != symm_legL)){
		for each (auto t in tmpl->getTemplatesByScope(TreeScope::DESCENDANTS)){
			t->removeAllConstraintsOfType(Constraint::ConstraintType::SYMM_LEGL);
		}
		if(symm_legL == true){
			constraintsAdded = true;
			equalizeLinkLength();
			ConstraintsEval::enforceConstraintsSymmetry(tmpl, false);	

		}
	}


	// uniform spacing
	if((!symm_spacing) || (tmpl->symmetryChoices.symm_spacing != symm_spacing)){
		for each (auto t in tmpl->getTemplatesByScope(TreeScope::DESCENDANTS)){
			t->removeAllConstraintsOfType(Constraint::ConstraintType::SYMM_SPACING);
		}
		if(symm_spacing == true){
			constraintsAdded = true;
			makeUniformSpacing();
			ConstraintsEval::enforceConstraintsSymmetry(tmpl, true);	

		}
	}


	tmpl->symmetryChoices.symm_ground = symm_ground;
	tmpl->symmetryChoices.symm_legW = symm_legW;
	tmpl->symmetryChoices.symm_legL = symm_legL;
	tmpl->symmetryChoices.symm_spacing = symm_spacing;
	// to do actually

	//if(constraintsAdded){
	//	ConstraintsEval::enforceConstraints(tmpl);	
	//}
}

void WorkingTemplate::replaceShoulderJoints(){

	for each( auto j in kinchain->getRoot()->children){

		 
		bool isShoulder = false;
		std::vector<KinNode*> allDescendants;
		j->getAllDescendants(allDescendants);
		for each (auto node in allDescendants){
			if (node->getType() == KinNode::KinNodeType::JOINT){
				isShoulder = true;
			}
		}
		KinNode_Joint* joint = dynamic_cast<KinNode_Joint*>(j);
		GrammarController* controller = dynamic_cast<GrammarController*>(joint->getArticulation()->transformations[0]->controller);
		if(isShoulder){
			controller->setType(GrammarController::GrammarcontrollerType::DOUBLE_SHOULDED);
		}else{
			//if(controller->getType() != GrammarController::GrammarcontrollerType::WHEEL){
			//	controller->setType(GrammarController::GrammarcontrollerType::LEG);;
			//}
		}
	}
	

}


drawing::Drawing WorkingTemplate::getTopViewDrawing(){
	PROFILE_THIS(__FUNCTION__);


	drawing::Drawing drawing;
	TriMesh* mesh = tmpl->getGeometry()->getMesh();

	for (int i = 0; i < mesh->vertices.size(); i++){
		Eigen::Vector2d point(mesh->vertices[i][0], mesh->vertices[i][2]);
		drawing.points.push_back(point);
	}

	for (int i = 0; i < mesh->faces.size(); i++){
		drawing::Face face;
		for( int j = 0; j < 3; j++){
			drawing::Point point;
			int pointId = mesh->faces[i].v[j];
			point.id = pointId;  // not sure if it's this or the j
			point.p =  Eigen::Vector2d(mesh->vertices[pointId][0], mesh->vertices[pointId][2]);
			face.idpoints.push_back(point);
		}
		drawing.faces.push_back(face);
	}
	return drawing;
}