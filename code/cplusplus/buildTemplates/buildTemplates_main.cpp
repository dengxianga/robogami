#include <sstream>
#include <iostream>
#include "design.h"
#include "template.h"
#include "ConstraintsEval.h"
#include "workingTemplate.h"
#include <vector>
#include "TriMesh.h"
#include "component.h"
#include "../FBE_Design/geometry.h"
#include "DesignProtoConverter.h"
#include <TemplateProtoConverter.h>
#include "FoldableGraph.h"
#include "CenterOfMass.h"
#include "MyOptimizer.h"
#include "KinChain.h"
#include "ElectronicsGraph.h"
#include "metrics.h"
#include "XForm.h"
#include "TriMesh_algo.h"
#include "articulation.h"
#include "FunctionEval.h"
#include "FunctionEvalStability.h"
#include "TemplateManipulations.h"
#include "element_motion.h"
#include "controller.h"
#include <ctime>
#include <nlopt.hpp>
#include "DrivingJointSelection.h"
#include "internalMotionOptimizer.h"
#include "geometryOptimizer.h"
#include "ReducedEval.h"

using namespace FabByExample;
using namespace std;

static void initializeLogging() {
	google::InitGoogleLogging("FabByExample");

	// This sets the minimum level of logging to output. The severity level goes as:
	//  google::INFO < google::WARNING < google::ERROR.
	// So for example if you set this to google::WARNING, then LOG(INFO) messages will
	// not show up. VLOG(x) counts as INFO.
	google::SetStderrLogging(google::INFO);

	// Set the verbose logging level. Only VLOG(x) where x <= this value will show up.
	FLAGS_v = 10;
}

void testPrinting(std::string protofilename, std::string destfilename){
	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
	LOG(ERROR) << temp;
	temp->getFullQ();
	LOG(ERROR) << "ok";
	temp->updateFullQ(temp->getFullQ());

	WorkingTemplate * wt = new WorkingTemplate(temp);

	//wt->updateTempByChangingOneParam(18, -600);
	//changes the lenth of the body
	//wt->updateTempByChangingOneParam(2, 600);


	FoldableGraph * foldGraph = new FoldableGraph(wt->getTemplate()); 
	PrintableDesign pf = foldGraph->generatePrintableDesign();
	pf.generatePrint(destfilename);

}


void test3DSTLgenStationary(std::string protofilename, std::string destfilename){
	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
	WorkingTemplate * wTemp = new WorkingTemplate(temp);
	converter.AddGaitInfoToKinChain(*protoRead, wTemp->kinchain);

	wTemp->kinchain->updateControllers(0);
	wTemp->kinchain->setRounds(0);
	wTemp->kinchain->updateGait(0.0);

	FoldableGraph * foldGraph = new FoldableGraph(wTemp->getTemplate()); 
	PrintableDesign pf = foldGraph->generatePrintableDesign();
	
	std::vector<double> times;
	times.push_back(0.0);


	for (int itime = 0; itime < times.size(); ++itime) {
		wTemp->kinchain->updateTime(times[itime]);
		Geometry * geo = wTemp->kinchain->getGeometry();

		std::stringstream dest;
		dest << "..\\..\\data\\prints\\" << itime << destfilename << "_anim.stl";

		geo->write(dest.str());
	}

	pf.generatePrint(destfilename, true, wTemp->kinchain, times);
}


void test3DSTLgen(std::string protofilename, std::string destfilename, int whichgait){
	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
	WorkingTemplate * wTemp = new WorkingTemplate(temp);
	converter.AddGaitInfoToKinChain(*protoRead, wTemp->kinchain);

	wTemp->kinchain->updateControllers(whichgait);
	wTemp->kinchain->setRounds(3);
	wTemp->kinchain->updateGait(0.01);

	FoldableGraph * foldGraph = new FoldableGraph(wTemp->getTemplate()); 
	PrintableDesign pf = foldGraph->generatePrintableDesign();
	
	std::vector<double> times;
	for (double currtime = 0; currtime <= 3.005; currtime += 0.01) {
		times.push_back(currtime);
	}

	pf.generatePrint(destfilename, true, wTemp->kinchain, times);
}



void testEvalCenterOfMass(std::string protofilename){
	TriMesh * m = TriMesh::read("..\\..\\data\\sphere2.stl");

	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);


	clock_t begin_time_full = clock();
	
	clock_t begin_time_com = clock();
	CenterOfMass centerofMass = temp->computeCenterOfMass();
	std::cout << "computed center of mass in " << double( clock() - begin_time_com ) / (double)CLOCKS_PER_SEC << " seconds!" << std::endl; 


	clock_t begin_time_cpoints = clock();
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>  contactPoints;
	temp->getContactPoints(contactPoints); 
	std::cout << "computed contactPoints in " << double( clock() - begin_time_cpoints ) / (double)CLOCKS_PER_SEC << " seconds!" << std::endl; 


	clock_t begin_time_eval = clock();
	Eigen::Vector2d projectedCenter;
	projectedCenter << centerofMass.center.x(), centerofMass.center.z();
	Eigen::VectorXd f = Eigen::VectorXd::Zero(contactPoints.size());
	Eigen::MatrixXd A;
	Eigen::VectorXd B, Beq, lb, ub, sol;
	double objval;
	Eigen::MatrixXd Aeq( 2, contactPoints.size());
	for(int i = 0; i < contactPoints.size(); i++){
		Aeq.col(i) = contactPoints[i];
	}
	Beq = projectedCenter;
	lb = Eigen::VectorXd::Zero(contactPoints.size());
	ub = Eigen::VectorXd::Ones(contactPoints.size());
	MyOptimizer::linOpt(f, A, B, Aeq, Beq, lb, ub, true, sol, &objval);
	std::cout << "computed evaluation " << double( clock() - begin_time_eval ) / (double)CLOCKS_PER_SEC << " seconds!" << std::endl; 
	
	
	std::cout << "finished everything in " << double( clock() - begin_time_full ) / (double)CLOCKS_PER_SEC << " seconds!" << std::endl; 




	temp->getGeometry()->write("..\\..\\data\\test\\model.stl");
	Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\sphere2.stl"));
	sphere->applyTrans(point(centerofMass.center.x(), centerofMass.center.y(), centerofMass.center.z()));
	sphere->write("..\\..\\data\\test\\center.stl");
	Geometry contactGeo;
	for(int i = 0; i < contactPoints.size(); i++){
		Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\sphere2.stl"));
		sphere->applyTrans(point(contactPoints[i].x(), centerofMass.center.y(), contactPoints[i].y()));
		contactGeo.add(sphere);
	}
	contactGeo.write("..\\..\\data\\test\\contact.stl");
}


void testKinematics(std::string protofilename, double dT){

	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);

	//temp->rotate(Eigen::Vector3d(0,0,0), Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(1,0,0), Eigen::Vector3d(0,1,0)));
	//temp->translate(Eigen::Vector3d(100,0,0));

	
	WorkingTemplate * wt = new WorkingTemplate(temp);
	KinChain* kinchain = new KinChain(temp); 


	
	
	clock_t begin_time = clock();
	
	kinchain->setRounds(3);

	bool doSequence = false;

	//for (int i = 0; i < 20; i++)
		kinchain->updateGait(dT, doSequence);
	std::cout << "time = " << double( clock() - begin_time ) / (double)CLOCKS_PER_SEC<< " seconds." << std::endl;

	std::pair<int, int> ssr = kinchain->getSteadyStateRounds();
	std::cout << "steadystaterounds: "<< ssr.first << ", " << ssr.second << std::endl;
	kinchain->setRounds(ssr.second + 5*(ssr.second-ssr.first));

	std::cout << "Slip: " << kinchain->getSlip(doSequence) <<std::endl; // << kinchain->getSlipVector() << std::endl;
	std::cout << "Stability: " << kinchain->getStability() <<std::endl; // << kinchain->getStabilityVector() << std::endl;
	std::cout << "Speed: " << kinchain->getSpeed(doSequence) << std::endl;
	std::cout << "Forward Travel: " << kinchain->getForwardTravel() << std::endl;
	std::cout << "Forward Travel @ theta=45deg.: " << kinchain->getForwardTravel(M_PI/4) << std::endl;
	std::cout << "PerpendicularError: " << kinchain->getPerpendicularError() << std::endl;
	std::cout << "Wobbliness: " << kinchain->getToppling(doSequence) << std::endl;
	std::cout << "Curvature: " << kinchain->getMeanCurvature() << std::endl;
	std::cout << "Mean Angle: " << kinchain->getMeanAngle() << std::endl;
	
	
	std::vector<Geometry*> anim = kinchain->getAnimation(false, 1, doSequence);

	for (int t = 0; t< anim.size();  t=t+1){
		std::stringstream geoFileName;
		geoFileName << "..\\..\\data\\test\\animate" << t << ".stl";
		anim[t]->write(geoFileName.str());
		//Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\sphere2.stl"));
		//sphere->applyTrans(point(centerofMass.center.x(), centerofMass.center.y(), centerofMass.center.z()));
		//std::stringstream sphereFileName;
		//sphereFileName << "..\\..\\data\\test\\center_t" << t << ".stl";
		//sphere->write(sphereFileName.str());
	}

	//ConstraintsEval::updateTempByChangingOneParam(wt->getTemplate(), 0, 100);
	//ConstraintsEval::updateTempByChangingOneParam(wt->getTemplate(), 1, 100);
	return;

	for (double t_test = 0; t_test <= 1; t_test += .25) {
		kinchain->updateTime(t_test);

		Geometry * geo = kinchain->getGeometry();
		std::stringstream geoFileName;
		geoFileName << "..\\..\\data\\test\\transformedGeo.stl";
		geo->write(geoFileName.str());
	
		std::list<KinNode*> appendages = kinchain->getLeaves();
		int i = 0;
		for (auto it = appendages.begin(); it != appendages.end(); ++it) {
			std::stringstream geoFileName;
			geoFileName << "..\\..\\data\\test\\appendage" << i << ".stl";
			((KinNode_Part*)(*it))->getCurrentGeoRelativeToRoot()->write(geoFileName.str());
			++i;
		}
	
		//kinchain->resetControllers();

		std::vector<Eigen::Vector3d> contacts = kinchain->getAllContactPoints(t_test, KinChain::CONTACT);
		for (auto it = contacts.begin(); it != contacts.end(); ++it) {
			std::cout << (*it).x() << " " << (*it).y() << " " << (*it).z() << std::endl;
		}
	}

	

	OptimizationWeights weights( 3, 10, 1, 1);
	InternalMotionOptimizer optim(wt, weights);
	double result;
	optim.optimizeThetas(result);


}

void outputAnimation(std::string protofilename, std::string optfilename, double dT = 0.01){

	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);

	WorkingTemplate * wt = new WorkingTemplate(temp);
	KinChain* kinchain = new KinChain(temp); 

	kinchain->setRounds(3);

	kinchain->updateGait(dT);
	std::vector<Geometry*> anim = kinchain->getAnimation();

	for (int t = 0; t< anim.size();  t=t+1){
		std::stringstream geoFileName;
		geoFileName << "..\\..\\data\\test\\articulation_tNeg" << t << ".stl";
		anim[t]->write(geoFileName.str());
	}

	TemplateProtoConverter converter2;
	auto protoRead2 = converter2.loadFromFile(optfilename.c_str());
    Template* temp2 = converter2.ConvertToTemplate(*protoRead2);

	WorkingTemplate * wt2 = new WorkingTemplate(temp2);
	KinChain* kinchain2 = new KinChain(temp2); 

	kinchain2->setRounds(3);

	kinchain2->updateGait(dT);
	std::vector<Geometry*> anim2 = kinchain2->getAnimation();

	for (int t = 0; t< anim2.size();  t=t+1){
		std::stringstream geoFileName;
		geoFileName << "..\\..\\data\\test\\articulationnew_t" << t << ".stl";
		anim2[t]->write(geoFileName.str());
	}
		
}

void testAnimation(std::string protofilename){

	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
	temp->updateFullQ(temp->getFullQ());
	WorkingTemplate * wt = new WorkingTemplate(temp);
	
	int count  =0;
	for(double t = 0; t< 1.0;  t=t+0.05){
		Geometry * geo = wt->getAnimation(t);
		//geo->write("..\\..\\data\\test\\bla.stl");
		//system("pause");
		std::stringstream geoFileName;
		geoFileName << "..\\..\\data\\test\\articulation_t" << count << ".stl";
		geo->write(geoFileName.str());
		delete geo;
		count++;
	}

	/*
	wt->updateTempByChangingOneParam(0, 100);
	wt->updateTempByChangingOneParam(1, 100);

	count  =0;
	for(double t = 0; t< 1.0;  t=t+0.1){
		Geometry * geo = wt->getAnimation(t);
		std::stringstream geoFileName;
		geoFileName << "..\\..\\data\\test\\articulation_mod_t" << count << ".stl";
		geo->write(geoFileName.str());
		delete geo;
		count++;
	}
	*/

	
}

void testAnimationSimp(std::string protofilename, std::string savename){

	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
	WorkingTemplate * wt = new WorkingTemplate(temp);
	
	Geometry * geo = wt->getAnimation(0.2);
	std::stringstream geoFileName;
	geo->write(savename);
	delete geo;

	
}

void testMetrics(std::string protofilename){
	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
	WorkingTemplate * wt = new WorkingTemplate(temp);

	wt->computeMetrics(3, 0.01, false);

	if(wt->metricsInfo->isStable){
		std::cout << "is stable" << std::endl;
	}else{
		std::cout << "is *NOT* stable" << std::endl;
	}
	std::cout << "Material cost is " << wt->metricsInfo->fabCost << std::endl; 



}

void generateSTL(std::string protofilename){

	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
	temp->getGeometry()->write("..\\..\\data\\test\\crab.stl");


	
	WorkingTemplate * wt = new WorkingTemplate(temp);


	//wt->updateTempByChangingOneParam(18, -800);
	//wt->updateTempByChangingOneParam(2, -200);


	wt->getTemplate()->getGeometry()->write("..\\..\\data\\test\\crab.stl");

	wt->getTemplate()->getGeometry()->write("..\\..\\data\\test\\crab6.stl");


	/*Eigen::Matrix3d rot = q.toRotationMatrix();

	initgeo->write("..\\..\\data\\test\\before.stl");

	xform initRot(rot(0,0), rot(0,1), rot(0,2), 0,
			rot(1,0), rot(1,1), rot(1,2), 0,
			rot(2,0), rot(2,1), rot(2,2), 0,
			0,  0, 0, 1);

	apply_xform(initgeo->mesh, initRot);			

	initgeo->write("..\\..\\data\\test\\after.stl");
	
	*/
	//system("pause");




}

void showKinematicTree(std::string protofilename){


	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);

	
	//WorkingTemplate * wt = new WorkingTemplate(temp);
	KinChain* kinchain = new KinChain(temp); 
	kinchain->displayTree(); 
	kinchain->displayNodes();

}


void testNewAnim(std::string protofilename){


	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
	temp->updateFullQ(temp->getFullQ());

	
	Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
	Eigen::Vector3d z = Eigen::Vector3d::Zero(3);
	z[2] = 1;
	Eigen::Vector3d y = Eigen::Vector3d::Zero(3);
	y[1] = 1;
	Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(z, y);
	temp->rotate(center, rotation); // I don't know how this is done!!! :(
	temp->updateFullQ(temp->getFullQ());
	
	temp->recomputeCenter();
	temp->translate(-temp->getCenter());
	temp->updateFullQ(temp->getFullQ());
	temp->moveToGround();

	WorkingTemplate * wt = new WorkingTemplate(temp);


/*	std::vector<NewConnection*> allConn;
	temp->getAllConnections(allConn);
	for each(NewConnection* c in allConn){
		if (c->getArticulation() != nullptr){
			std::cout << "find some articulation " << std::endl;
			JointConnection* jc = dynamic_cast<JointConnection*> (c);
			c->getArticulation()->print();
			std::cout << jc->toString() << std::endl;
			std::cout << "center: " << c->getArticulation()->symbCenter.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES) << std::endl;
			for each (auto trans in c->getArticulation()->transformations) {
				Eigen::Vector3d axis = trans->symbAxis.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
				std::cout << "axis: " << axis << std::endl;
			}
			
		}
	}
	cin.get();

	*/
	
	// changes the length of the legs
	//wt->updateTempByChangingOneParam(18, 300);
	//changes the lenth of the body
	//wt->updateTempByChangingOneParam(2, 1000);
	//wt->updateTempByChangingOneParam(1, -200);
	std::vector<Geometry*> anim;
	std::vector<double> stabilityCost;
	double stab  = wt->getNewAnimation(false, anim, false, stabilityCost, 1); 
	std::cout << "final stability = " << stab << std::endl;


	//KinChain* kinchain = new KinChain(temp); 
	////wt->getTemplate()->updateFullQ(wt->getTemplate()->getFullQ());
	//kinchain->getAnimation(anim, 0.05);

	
	for(int i =0; i < anim.size();i++){
		std::stringstream geoFileName;
		geoFileName << "..\\..\\data\\test\\newAnim_tA" << i << ".stl";	
		anim[i]->write(geoFileName.str());
	}

}

void testRotation(std::string protofilename){
	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
	Template* temp = converter.ConvertToTemplate(*protoRead);
	WorkingTemplate * wt = new WorkingTemplate(temp);
	Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
	Eigen::Vector3d z = Eigen::Vector3d::Zero(3);
	z[2] = 1;
	Eigen::Vector3d y = Eigen::Vector3d::Zero(3);
	y[1] = 1;
	Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(z, y);
	

	wt->getAnimation(0.2)->write("..\\..\\data\\test\\newAnim_beforeRot.stl");


	//the rotation takes two arguments: a vector3d as center, a quatorniond as the rotation 
	wt->getTemplate()->rotate(center, rotation); 
	wt->getTemplate()->updateFullQ(wt->getTemplate()->getFullQ());

	// Call animate here

	wt->getTemplate()->getGeometry()->write("..\\..\\data\\test\\newpose_afterRot.stl");

	wt->getAnimation(0.2)->write("..\\..\\data\\test\\newAnim_afterRot.stl");


}


void testLoadingSub(std::string protoFilename, int id, std::string output){
	TemplateProtoConverter c;
	auto protoLoaded = c.loadFromFile(protoFilename.c_str());
	Template* temp = c.ConvertToTemplate(*protoLoaded);
	Template* child = temp->findDescendantByID(id);
	child->segregate();
	WorkingTemplate* wt = new WorkingTemplate(child);
	std::string fileName = concat() << "..\\..\\data\\test\\" << output << id << ".stl";
	wt->getTemplate()->getGeometry()->write(fileName);
}


void testLinearExprPerformance() {
	PROFILE_THIS(__FUNCTION__);
	Symbol a(nullptr, 1);
	Symbol b(nullptr, 2);
	CoeffsMap env;
	env[a] = 1;
	env[b] = 2;
	LinearExpr le(env, 3);
	std::vector<LinearExpr> v;

	for (int i = 0; i < 100000; i++) {
		v.push_back(le);
	}
	
}

void testLinearExprCopyPerformance() {
	PROFILE_THIS(__FUNCTION__);
	Symbol a(nullptr, 1);
	Symbol b(nullptr, 2);
	CoeffsMap env;
	env[a] = 1;
	env[b] = 2;
	LinearExpr le(env, 3);
	LinearExpr lp;
	int x = 0;

	for (int i = 0; i < 400000; i++) {
		lp = le;
		x += lp.getCoeffs().size();
	}
	printf("size of LE is %d", sizeof(LinearExpr));
}

namespace testFastHashmap {
	void test() {
		
		PROFILE_THIS(__FUNCTION__);
		CoeffsMap m;
		printf("Putting two items into it.\n");
		m.put(Symbol(nullptr, 1), 1);
		m.put(Symbol(nullptr, 2), 2);
		for each (auto& it in m) {
			printf("Pair: (%d, %llf)\n", it.first.id, it.second);
		}

		PROFILE_BEGIN("inline_copy");
		for (int i = 0; i < 40000; i++) {
			CoeffsMap m1 = m;
		}
		PROFILE_END;
		printf("Putting two more items into it.\n");
		m.put(Symbol(nullptr, 2), 2);
		m.put(Symbol(nullptr, 3), 3);
		m.put(Symbol(nullptr, 4), 4);
		for each (auto& it in m) {
			printf("Pair: (%d, %llf)\n", it.first.id, it.second);
		}
		printf("Putting yet another item into it.\n");
		m.put(Symbol(nullptr, 5), 5);
		for each (auto& it in m) {
			printf("Pair: (%d, %llf)\n", it.first.id, it.second);
		}

		PROFILE_BEGIN("heavy_copy");
		for (int i = 0; i < 40000; i++) {
			CoeffsMap m1 = m;
		}
		PROFILE_END;

		for each (auto& it in m) {
			printf("Pair: (%d, %llf)\n", it.first.id, it.second);
		}

		CoeffsMap n;
		n.put(Symbol(nullptr, 1), 1);
		n.put(Symbol(nullptr, 6), 6);

		{
			CoeffsMap k = m;
			printf("Correct: %b\n", k == m);


			k = n;
			printf("Correct: %b\n", k == n);
			k = m;
			printf("Correct: %b\n", k == m);
		}

	}

}
void testManipulation(std::string protofilename){

	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
	Template* temp = converter.ConvertToTemplate(*protoRead);

	WorkingTemplate* wt = new WorkingTemplate(temp);

	std::vector<TemplateElement*> elements;
	temp->getAllElementTempList(elements);

	wt->getTemplate()->getGeometry()->write("..\\..\\data\\test\\short_BEF.stl");
	PatchPair p(nullptr, nullptr, false, false);

	clock_t begin_time_eval = clock();
	// negative = regular arrow
	// zero = no arrow 
	// positive = arrow in the oposite direction
	ConstraintsEval::updateTemplateBySubElementFaceScaling(elements[0], 0, 1, 1.5, p);
	double stab = wt->getDirStability(elements[0], 0, 1, 0, 0);
	std::cout << "stab diff = " << stab << std::endl;
	std::cout << "computed evaluation " << double(clock() - begin_time_eval) / (double)CLOCKS_PER_SEC << " seconds!" << std::endl;



	wt->getTemplate()->getGeometry()->write("..\\..\\data\\test\\short_AFT.stl");

}

void testEvalCenterOfMassB(std::string protofilename){
	TriMesh * m = TriMesh::read("..\\..\\data\\sphere2.stl");

	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
	Template* temp = converter.ConvertToTemplate(*protoRead);


	CenterOfMass centerofMass = temp->computeCenterOfMass();


	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>  contactPoints;
	temp->getContactPoints(contactPoints);




	temp->getGeometry()->write("..\\..\\data\\test\\model.stl");
	Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
	sphere->applyTrans(point(centerofMass.center.x(), centerofMass.center.y(), centerofMass.center.z()));
	sphere->write("..\\..\\data\\test\\center.stl");
	Geometry contactGeo;
	for (int i = 0; i < contactPoints.size(); i++){
		Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
		sphere->applyTrans(point(contactPoints[i].x(), centerofMass.center.y(), contactPoints[i].y()));
		contactGeo.add(sphere);
	}
	contactGeo.write("..\\..\\data\\test\\contact.stl");
}

//-----------------FOR ROBIN TO CLEAN UP THIS BUNCH STUFF ------------------------------------
/*
		printf("Putting two more items into it.\n");
		m.put(Symbol(nullptr, 2), 2);
		m.put(Symbol(nullptr, 3), 3);
		m.put(Symbol(nullptr, 4), 4);
		for each (auto& it in m) {
			printf("Pair: (%d, %llf)\n", it.first.id, it.second);
		}
		printf("Putting yet another item into it.\n");
		m.put(Symbol(nullptr, 5), 5);
		for each (auto& it in m) {
			printf("Pair: (%d, %llf)\n", it.first.id, it.second);
		}

		PROFILE_BEGIN("heavy_copy");
		for (int i = 0; i < 40000; i++) {
			CoeffsMap m1 = m;
		}
		PROFILE_END;

		for each (auto& it in m) {
			printf("Pair: (%d, %llf)\n", it.first.id, it.second);
		}

		CoeffsMap n;
		n.put(Symbol(nullptr, 1), 1);
		n.put(Symbol(nullptr, 6), 6);
		
		{
			CoeffsMap k = m;
			printf("Correct: %b\n", k == m);
		

			k = n;
			printf("Correct: %b\n", k == n);
			k = m;
			printf("Correct: %b\n", k == m);
		}

	}

}
*/

void testStabilize(std:: string protofilename){
 
	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
	Template* temp = converter.ConvertToTemplate(*protoRead);

	Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
	Eigen::Vector3d z = Eigen::Vector3d::Zero(3);
	z[2] = 1;
	Eigen::Vector3d y = Eigen::Vector3d::Zero(3);
	y[1] = 1;
	Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(z, y);
	temp->rotate(center, rotation); // I don't know how this is done!!! :(
	temp->updateFullQ(temp->getFullQ());
	
	temp->recomputeCenter();
	temp->translate(-temp->getCenter());
	temp->updateFullQ(temp->getFullQ());
	temp->moveToGround();



	WorkingTemplate* wt = new WorkingTemplate(temp);


	wt->evalStabilization();

}
void savePatchesToFile(Template* temp, std::string filename){
	vector<NewPatch*> patches = temp->getAllPatches();


	Geometry * geo = temp->getGeometry();
	

	for (int i = 0 ; i < patches.size(); i++){

		if (patches[i]->getType() == NewPatch::PatchType::SERVO_LINE_PATCH){
			std::cout << "found a servo line patch" << std::endl;
			ServoLinePatch* ps = dynamic_cast<ServoLinePatch*>(patches[i]);
			Eigen::Quaterniond rot_q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),ps->getNormal());
			auto rot_mat = rot_q.toRotationMatrix();
			std::vector<double> rot(9);
			for (int i = 0; i< 9; i++){
				rot[i] = rot_mat(i);
			}
			point p = ps->getVertex1().evalpoint(SymbolicAssignment::USE_CURRENT_VALUES);
			Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\arrow.obj"));
			sphere->applyRot(rot);
			sphere->applyTrans(p);
			geo->add(sphere);
			point p2 = ps->getVertex2().evalpoint(SymbolicAssignment::USE_CURRENT_VALUES);
			Geometry * sphere2 = new Geometry(TriMesh::read("..\\..\\data\\arrow.obj"));
			sphere2->applyRot(rot);
			sphere2->applyTrans(p2);
			geo->add(sphere2);


		}
		if (patches[i]->getType() == NewPatch::PatchType::SERVO_POINT_PATCH){
			std::cout << "found a servo point patch" << std::endl;
			ServoPointPatch* ps = dynamic_cast<ServoPointPatch*>(patches[i]);
			point p = ps->center.evalpoint(SymbolicAssignment::USE_CURRENT_VALUES);
			Eigen::Quaterniond rot_q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),ps->getNormal());
			auto rot_mat = rot_q.toRotationMatrix();
			std::cout << "the normal is " << ps->getNormal().transpose() << std::endl;
			std::cout << "the rot_mat is " << rot_mat << std::endl;
			std::vector<double> rot(9);
			for (int i = 0; i< 9; i++){
				rot[i] = rot_mat(i);
			}
			Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\arrow.obj"));
			sphere->applyRot(rot);
			sphere->applyTrans(p);
			geo->add(sphere);

			
		}


		if (patches[i]->getType() == NewPatch::PatchType::PERIPHERAL_PATCH){
			std::cout << "found a peripheral patch" << std::endl;
			PeripheralPatch* ps = dynamic_cast<PeripheralPatch*>(patches[i]);
			// compute rotaion
			Eigen::Quaterniond rot_q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),ps->getNormal());
			auto rot_mat = rot_q.toRotationMatrix();
			std::vector<double> rot(9);
			for (int i = 0; i< 9; i++){
				rot[i] = rot_mat(i);
			}
			//get all points
			std::vector<Eigen::Vector3d> points;
			points.push_back(ps->getLinePatch1()->getVertex1());
			points.push_back(ps->getLinePatch1()->getVertex2());
			points.push_back(ps->getLinePatch2()->getVertex1());
			points.push_back(ps->getLinePatch2()->getVertex2());
			for each ( auto p in points){
				point p_point (p.x(), p.y(), p.z());
				Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\arrow.obj"));
				sphere->applyRot(rot);		
				sphere->applyTrans(p_point);
				geo->add(sphere);
			}			
		}
			


	}

	geo->write(filename); 


}
void showPatches(std:: string protofilename){

	std::cout << "showing patches for " << protofilename << std::endl;
	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
	Template* temp = converter.ConvertToTemplate(*protoRead);
	savePatchesToFile(temp, "..\\..\\data\\testpatches_before.stl");



	std::vector<TemplateElement*> elements;
	temp->getAllElementTempList(elements);
	PatchPair p(nullptr, nullptr, false, false);
	ConstraintsEval::updateTemplateBySubElementFaceScaling(elements[0], 0, 1, 1.5, p);
	ConstraintsEval::updateTemplateBySubElementFaceScaling(elements[0], 1, 1, 1.5, p);
	ConstraintsEval::updateTemplateBySubElementFaceScaling(elements[0], 2, 1, 1.5, p);

	savePatchesToFile(temp, "..\\..\\data\\testpatches_after.stl");
	
	std::cout << "done with saving the patches" << std::endl;

}


void showConnections(std:: string protofilename){

	std::vector<NewConnection *> connections;  
	std::cout << "showing patches for " << protofilename << std::endl;
	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
	Template* temp = converter.ConvertToTemplate(*protoRead);
	savePatchesToFile(temp, "..\\..\\data\\testpatches_before.stl");


	temp->getConnections(connections, TreeScope::DESCENDANTS);
	for each (auto c in connections){
		std::cout << "New connections " << std::endl;
		for each (auto p in c->getPatches()){
			std::cout << "-------conencting patch type: " << p->getType() << std::endl;
		}
	}

	
	std::cout << "done with showing the connecitons" << std::endl;

}



void debugWheelRotation(){



	std::string protofilename("..\\..\\data\\proto2016\\WheelBasic\\template.asciiproto");
	std::cout << "showing patches for " << protofilename << std::endl;
	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
	Template* temp = converter.ConvertToTemplate(*protoRead);
	

	if(true){
		Geometry * geo = 	temp->getGeometry();
		auto conn = temp->getAllContactPoints();
		std::cout << "found " << conn.size() << " contact points " << std::endl;
		for each ( auto c in conn){
				std::cout <<"found a contact point " << std::endl;
				Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
				sphere->applyTrans(c->cPoint.evalpoint(SymbolicAssignment::USE_CURRENT_VALUES));
				geo->add(sphere);

		}
	
		geo->write("..\\..\\data\\contatPoints_before.stl");
		std::cout << "done with saving the cpntact points" << std::endl;
	}


	Eigen::Quaterniond q0(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
	temp->rotate(Eigen::Vector3d(0,0,0), q0);
	temp->translate(Eigen::Vector3d(0,0,0));


	if(true){
		Geometry * geo = 	temp->getGeometry();
		auto conn = temp->getAllContactPoints();
		std::cout << "found " << conn.size() << " contact points " << std::endl;
		for each ( auto c in conn){
				std::cout <<"found a contact point " << std::endl;
				Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
				sphere->applyTrans(c->cPoint.evalpoint(SymbolicAssignment::USE_CURRENT_VALUES));
				geo->add(sphere);

		}
	
		geo->write("..\\..\\data\\contatPoints_after.stl");
		std::cout << "done with saving the cpntact points" << std::endl;
	}


	TemplateProtoConverter outputConverter;
	auto outputProto = outputConverter.ConvertToProto(temp);
	outputConverter.saveToFile(*outputProto, "..\\..\\data\\proto2016\\WheelBasic\\templateRot.asciiproto", true);


}


void showContactPoints(std:: string protofilename){




	std::cout << "showing patches for " << protofilename << std::endl;
	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
	Template* temp = converter.ConvertToTemplate(*protoRead);
	

	if(true){
		Geometry * geo = 	temp->getGeometry();
		auto conn = temp->getAllContactPoints();
		std::cout << "found " << conn.size() << " contact points " << std::endl;
		for each ( auto c in conn){
				std::cout <<"found a contact point " << std::endl;
				Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
				sphere->applyTrans(c->cPoint.evalpoint(SymbolicAssignment::USE_CURRENT_VALUES));
				geo->add(sphere);

		}
	
		geo->write("..\\..\\data\\contatPoints_before.stl");
		std::cout << "done with saving the cpntact points" << std::endl;
	}


	
	std::vector<TemplateElement*> elements;
	temp->getAllElementTempList(elements);
	PatchPair p(nullptr, nullptr, false, false);
	ConstraintsEval::updateTemplateBySubElementFaceScaling(elements[0], 0, 1, 1.5, p);
	ConstraintsEval::updateTemplateBySubElementFaceScaling(elements[0], 1, 1, 1.5, p);
	ConstraintsEval::updateTemplateBySubElementFaceScaling(elements[0], 2, 1, 1.5, p);

	if(true){
		Geometry * geo = 	temp->getGeometry();
		auto conn = temp->getAllContactPoints();
		std::cout << "found " << conn.size() << " contact points " << std::endl;
		for each ( auto c in conn){
				std::cout <<"found a contact point " << std::endl;
				Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
				sphere->applyTrans(c->cPoint.evalpoint(SymbolicAssignment::USE_CURRENT_VALUES));
				geo->add(sphere);

		}
	
		geo->write("..\\..\\data\\contatPoints_after.stl");
		std::cout << "done with saving the cpntact points" << std::endl;
	}


}

void showSemantics( std::string filename){
	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(filename.c_str());
	Template* temp = converter.ConvertToTemplate(*protoRead);


	for each (auto tmpl in temp->getTemplatesByScope(TreeScope::DESCENDANTS)){
		std::cout << "for template = " << tmpl->getName() << std::endl;
		std::cout << "the part type is " << tmpl->getPartType() << std::endl;
		std::cout << "the print method is " << tmpl->getPrintMethod() << std::endl;
	}

}

void openAndSaveAgain(std::string inprotofilename, std::string outprotofilenama){

	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(inprotofilename.c_str());
	Template* temp = converter.ConvertToTemplate(*protoRead);

	TemplateProtoConverter outputConverter;
	auto outputProto = outputConverter.ConvertToProto(temp);
	outputConverter.saveToFile(*outputProto, outprotofilenama.c_str(), true);



}

void orientTemplateCorrectly(std::string name, std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> > & rotations){



    std::string inFileName = "..\\..\\data\\\proto2016\\" + name;
    std::string outFileName = "..\\..\\data\\\proto2016\\" + name;

    std::cout << "doing rotations for " << name << std::endl;
    TemplateProtoConverter converter;
    auto protoRead = converter.loadFromFile(inFileName.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
    savePatchesToFile(temp, "..\\..\\data\\testpatches_before.stl");
	
	//Eigen::Quaterniond q0(Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitX()));
	temp->recomputeCenter();
	temp->translate(-temp->getCenter());
	for(int i = 0; i < rotations.size(); i++){
		temp->rotate(temp->getCenter(), rotations[i]);		
	}

	temp->moveToGround();

	TemplateProtoConverter outputConverter;
	auto outputProto = outputConverter.ConvertToProto(temp);
	outputConverter.saveToFile(*outputProto, outFileName.c_str(), true);



}

void fixOrientations(std::string name, Eigen::Quaterniond * rotation){

	std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> > rotations;
	rotations.push_back( *rotation);

	orientTemplateCorrectly(name + "\\template.asciiproto", rotations);

}

void testCompositionPeripherals(){

	TemplateProtoConverter converter1;
	auto protoRead1 = converter1.loadFromFile("..\\..\\data\\\proto2016\\tests\\BodyHex.asciiproto");
	Template* temp1 = converter1.ConvertToTemplate(*protoRead1);

	TemplateProtoConverter converter2;
	auto protoRead2 = converter2.loadFromFile("..\\..\\data\\\proto2016\\tests\\PeripheralHat.asciiproto");
	Template* temp2 = converter2.ConvertToTemplate(*protoRead2);
	
	temp2->translate(Eigen::Vector3d(0, 0, 50));

	temp1->getGeometry()->write("..\\..\\data\\combinedExample_b1.stl");
	temp2->getGeometry()->write("..\\..\\data\\combinedExample_b2.stl");

	
	Template* combinedTemplate = TemplateManipulations::ConnectWithGrammar(temp1, temp2);

	
	combinedTemplate->getGeometry()->write("..\\..\\data\\combinedExample.stl");

//	TemplateProtoConverter converter3;
//	auto protoRead3 = converter3.loadFromFile("..\\..\\data\\\proto2016\\tests\\BeamBasic.asciiproto");
//	Template* temp3 = converter3.ConvertToTemplate(*protoRead2);
	
//	combinedTemplate = TemplateManipulations::ConnectWithGrammar(combinedTemplate, temp3);
//	combinedTemplate->getGeometry()->write("..\\..\\data\\combinedExample2.stl");

	TemplateProtoConverter outputConverter;
	auto outputProto = outputConverter.ConvertToProto(combinedTemplate);
	outputConverter.saveToFile(*outputProto, "..\\..\\data\\\proto2016\\tests\\composedPeripherals.asciiproto", true);


}


void testComposition(){

	TemplateProtoConverter converter1;
	auto protoRead1 = converter1.loadFromFile("..\\..\\data\\\proto2016\\tests\\BodyHex.asciiproto");
	Template* temp1 = converter1.ConvertToTemplate(*protoRead1);

	TemplateProtoConverter converter2;
	auto protoRead2 = converter2.loadFromFile("..\\..\\data\\\proto2016\\tests\\BeamBasic.asciiproto");
	Template* temp2 = converter2.ConvertToTemplate(*protoRead2);
	
	temp1->getGeometry()->write("..\\..\\data\\combinedExample_b1.stl");
	temp2->getGeometry()->write("..\\..\\data\\combinedExample_b2.stl");

	
	Template* combinedTemplate = TemplateManipulations::ConnectWithGrammar(temp1, temp2);

	
	combinedTemplate->getGeometry()->write("..\\..\\data\\combinedExample.stl");

	TemplateProtoConverter converter3;
	auto protoRead3 = converter3.loadFromFile("..\\..\\data\\\proto2016\\tests\\BeamBasic.asciiproto");
	Template* temp3 = converter3.ConvertToTemplate(*protoRead2);
	
	combinedTemplate = TemplateManipulations::ConnectWithGrammar(combinedTemplate, temp3);
	combinedTemplate->getGeometry()->write("..\\..\\data\\combinedExample2.stl");


}

void realignProtos(){

	//first run this:

	map<std::string, Eigen::Quaterniond *> names;

	
	names["BeamBasic"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX()));
	names["BeamBasicContactless"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX()));
	names["BeamTriangle"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX()));
	names["BeamTriangleContactless"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX()));
	names["BeamHex"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX()));
	names["BeamhexContactless"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
	names["BeamStar"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
	names["BeamStarContactless"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
	names["BeamHeart"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
	names["BeamHeartContactless"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
	names["BeamRhex"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI - M_PI/6.0, Eigen::Vector3d::UnitZ()));
	names["BeamRhexContactless"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI + M_PI/12.0, Eigen::Vector3d::UnitZ()));
	names["BeamSlanted"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI + M_PI/6.0 + M_PI/12.0, Eigen::Vector3d::UnitZ()));
	names["BeamSlantedContactless"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI + M_PI/6.0 + M_PI/12.0, Eigen::Vector3d::UnitZ()));
	names["BeamConcave"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
	names["BeamConcaveContactless"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
	names["BeamConvex"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
	names["BeamConvexContactless"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

	names["BodyBasic"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-3.0/2.0*M_PI, Eigen::Vector3d::UnitX()));
	names["BodyT"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-3.0/2.0*M_PI, Eigen::Vector3d::UnitX()));
	names["BodySleek"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-3.0/2.0*M_PI, Eigen::Vector3d::UnitX()));
	names["BodyBeatle"] = new Eigen::Quaterniond(Eigen::AngleAxisd(1.0/2.0*M_PI, Eigen::Vector3d::UnitZ()));
	names["BodyTet"] = new Eigen::Quaterniond(Eigen::AngleAxisd(1.0/2.0*M_PI, Eigen::Vector3d::UnitX()));
	names["BodyTrain"] = new Eigen::Quaterniond(Eigen::AngleAxisd(1.0/2.0*M_PI, Eigen::Vector3d::UnitX()));
	names["BodyShoe"] = new Eigen::Quaterniond(Eigen::AngleAxisd(1.0/2.0*M_PI, Eigen::Vector3d::UnitX()));
	names["BodyCylinder"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-1.0/2.0*M_PI, Eigen::Vector3d::UnitX()));

	names["SnowPlow2"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/6.0 - M_PI, Eigen::Vector3d::UnitX()));
	names["PeripheralHat"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ()));
	names["Basket2"] = new Eigen::Quaterniond(Eigen::AngleAxisd(3.0*M_PI/2.0, Eigen::Vector3d::UnitZ()));
	names["HitchA"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
	names["PeripheralBunny"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX()));
	names["PeripheralCharacterFixed"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/6.0 - M_PI, Eigen::Vector3d::UnitX()));
	names["PeripheralAntenna"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ()));
	names["PeripheralWings"] = new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX()));
	names["PeripheralSiggraph"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitX()));
	names["PeripheralSiggraph"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitX()));

	

	//wheels don't need anything
	/*
	names["WheelBasic"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitX()));
	names["WheelRimless"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitX()));
	names["WheelSpoked"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitX()));
	*/
	
	/*
	names["HitchA"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitX()));
	names["Basket2"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitX()));	
	names["PeripheralHat"] = new Eigen::Quaterniond(Eigen::AngleAxisd(4.0/3.0*M_PI, Eigen::Vector3d::UnitY()));
	names["SnowPlow2"] = new Eigen::Quaterniond(Eigen::AngleAxisd(-2.0/3.0*M_PI, Eigen::Vector3d::UnitX()));
	*/
	for(auto iterator = names.begin(); iterator != names.end(); iterator++){
		std::string name = iterator->first;
		showPatches("..\\..\\data\\\proto2016\\" + name + "\\template.asciiproto");
		Eigen::Quaterniond * rotation = iterator->second;
		fixOrientations(name, rotation); 
		showPatches("..\\..\\data\\\proto2016\\" + name + "\\template.asciiproto");
	}

	//fixOrientations("PeripheralBunny", new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ())));
	fixOrientations("BodyBeatle", new Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI/10.0, Eigen::Vector3d::UnitX())));
	fixOrientations("BodyShoe", new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI + M_PI/2.0, Eigen::Vector3d::UnitY())));
	fixOrientations("BodyT", new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI + M_PI/2.0, Eigen::Vector3d::UnitY())));
	fixOrientations("BodyTrain", new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI + M_PI/2.0, Eigen::Vector3d::UnitY())));
	fixOrientations("BodyBeatle", new Eigen::Quaterniond(Eigen::AngleAxisd( M_PI/2.0, Eigen::Vector3d::UnitY())));
	fixOrientations("BodyBasic", new Eigen::Quaterniond(Eigen::AngleAxisd( M_PI/2.0, Eigen::Vector3d::UnitY())));
	fixOrientations("BodyCylinder", new Eigen::Quaterniond(Eigen::AngleAxisd( M_PI/2.0, Eigen::Vector3d::UnitY())));
	fixOrientations("BodySleek", new Eigen::Quaterniond(Eigen::AngleAxisd( M_PI/2.0, Eigen::Vector3d::UnitY())));
	fixOrientations("BodyCuboctahedra", new Eigen::Quaterniond(Eigen::AngleAxisd( M_PI/2.0, Eigen::Vector3d::UnitY())));
	fixOrientations("BodyTet", new Eigen::Quaterniond(Eigen::AngleAxisd( M_PI/2.0, Eigen::Vector3d::UnitY())));
	fixOrientations("BodyTruncated", new Eigen::Quaterniond(Eigen::AngleAxisd( M_PI/2.0, Eigen::Vector3d::UnitY())));

	fixOrientations("Basket2", new Eigen::Quaterniond(Eigen::AngleAxisd(3.0*M_PI/2.0, Eigen::Vector3d::UnitZ())));
	//fixOrientations("Basket2", new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())));
	
	fixOrientations("PeripheralAntenna", new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())));
	fixOrientations("PeripheralBox", new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())));
	
	fixOrientations("PeripheralCharacterFixed", new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())));
	fixOrientations("PeripheralHat", new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())));

	fixOrientations("SnowPlow2", new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())));

	fixOrientations("PeripheralBunny", new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ())));
	fixOrientations("PeripheralBunny", new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())));

	fixOrientations("HitchA", new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())));
	fixOrientations("BodyCylinder", new Eigen::Quaterniond(Eigen::AngleAxisd( M_PI/10.0, Eigen::Vector3d::UnitX())));

	fixOrientations("PeripheralHat", new Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/6.0, Eigen::Vector3d::UnitZ())));
	//fixOrientations("BodyCylinder", new Eigen::Quaterniond(Eigen::AngleAxisd( 2.0*M_PI/180.0, Eigen::Vector3d::UnitX())));

}

/*
void exampleForAndy(std::string name){

	//first run this:
	showPatches("..\\..\\data\\\proto2016\\" + name + "\\template.asciiproto");
	fixOrientations(name); 
	showPatches("..\\..\\data\\\proto2016\\" + name + "\\template.asciiproto");
	//showPatches("..\\..\\data\\\proto2016\\tests\\BeamBasic.asciiproto");
	

}
*/



void composeWithMotion(){

	TemplateProtoConverter converter1;
	auto protoRead1 = converter1.loadFromFile("..\\..\\data\\\proto2016\\tests\\BodyHex.asciiproto");
	Template* temp1 = converter1.ConvertToTemplate(*protoRead1);

	TemplateProtoConverter converter2;
	auto protoRead2 = converter2.loadFromFile("..\\..\\data\\\proto2016\\tests\\BeamBasic.asciiproto");
	Template* temp2 = converter2.ConvertToTemplate(*protoRead2);
	
	Template* combinedTemplate = TemplateManipulations::ConnectWithGrammar(temp1, temp2);

	std::cout << "composing the motion" << std::endl;
	combinedTemplate->getGeometry()->write("..\\..\\data\\combinedExample.stl"); 


	std::cout << "displaying the composed motion " << std::endl;
	auto elements = combinedTemplate->getAllElementList();
	for each ( auto e in elements){
		if( dynamic_cast<Element_Motion*>(e) != nullptr){
			PWLinearController *  controller = dynamic_cast<Element_Motion*>(e)->eval();
			controller->display();
		}
	}

	std::cout << "animating the final desing " << std::endl;

	auto workingTemplate = new WorkingTemplate(combinedTemplate);
	for (double i = 0; i < 1; i+=0.2){
		std::stringstream filename;
		std::cout << "for t = " << i << std::endl;
		filename << "..\\..\\data\\combinedExample_anim_t"  << i*10 <<  ".stl";
		workingTemplate->getAnimation(i)->write(filename.str());
	}

	TemplateProtoConverter outputConverter;
	auto outputProto = outputConverter.ConvertToProto(combinedTemplate);
	outputConverter.saveToFile(*outputProto, "..\\..\\data\\\proto2016\\tests\\composedWMotion.asciiproto", true);





}



void displayMotionAndAnimate(std::string filename){

	TemplateProtoConverter converter1;
	auto protoRead1 = converter1.loadFromFile(filename.c_str());
	Template* combinedTemplate = converter1.ConvertToTemplate(*protoRead1);
	

	std::cout << "composing the motion" << std::endl;
	combinedTemplate->getGeometry()->write("..\\..\\data\\combinedExample.stl"); 

	std::cout << "displaying the composed motion " << std::endl;
	auto elements = combinedTemplate->getAllElementList();
	for each ( auto e in elements){
		if( dynamic_cast<Element_Motion*>(e) != nullptr){
			PWLinearController *  controller = dynamic_cast<Element_Motion*>(e)->eval();
			controller->display();
		}
	}

	std::cout << "animating the final desing " << std::endl;

	auto workingTemplate = new WorkingTemplate(combinedTemplate);
	for (double i = 0; i < 1; i+=0.05){
		std::stringstream filename;
		std::cout << "for t = " << i << std::endl;
		filename << "..\\..\\data\\combinedExample_anim_t"  << i*10 <<  ".stl";
		workingTemplate->getAnimation(i)->write(filename.str());
	}






}


void testComposePatches(){

	TemplateProtoConverter converter1;
	auto protoRead1 = converter1.loadFromFile("..\\..\\data\\\proto2016\\PeripheralHat\\template.asciiproto");
	Template* temp1 = converter1.ConvertToTemplate(*protoRead1);

	temp1->recomputeCenter();
	temp1->translate(-temp1->getCenter());
	temp1->updateFullQ(temp1->getFullQ());
	temp1->moveToGround();


	TemplateProtoConverter converter2;
	auto protoRead2 = converter2.loadFromFile("..\\..\\data\\\proto2016\\PeripheralHat\\template.asciiproto");
	Template* temp2 = converter2.ConvertToTemplate(*protoRead2);
	
	temp2->recomputeCenter();
	temp2->translate(-temp2->getCenter());
	temp2->updateFullQ(temp2->getFullQ());
	//translate template up to the ground
	temp2->moveToGround();


	Template* combinedTemplate = TemplateManipulations::ConnectWithGrammar(temp1, temp2);
		
	BBox3S templateBBox = combinedTemplate->evalLocalBoundingBox();

	std::cout << "composing the motion" << std::endl;
	combinedTemplate->getGeometry()->write("..\\..\\data\\combinedExample.stl"); 

	savePatchesToFile(combinedTemplate, "..\\..\\data\\combinedExample_patches.stl"); 

	std::cout << "temp1 the volume is " << temp1->getGeometry()->getBBoxVolume() << std::endl;
	std::cout << "temp2 the volume is " << temp2->getGeometry()->getBBoxVolume() << std::endl;

	std::cout << "now the volume is " << combinedTemplate->getGeometry()->getBBoxVolume() << std::endl;

	TemplateProtoConverter outputConverter;
	auto outputProto = outputConverter.ConvertToProto(combinedTemplate);
	outputConverter.saveToFile(*outputProto, "..\\..\\data\\\proto2016\\tests\\testComp6.asciiproto", true);





}




void savePerpatch(){

	TemplateProtoConverter converter1;
	auto protoRead1 = converter1.loadFromFile("..\\..\\data\\\proto2016\\tests\\PeripheralHat.asciiproto");
	Template* temp1 = converter1.ConvertToTemplate(*protoRead1);

	TemplateProtoConverter outputConverter;
	auto outputProto = outputConverter.ConvertToProto(temp1);
	outputConverter.saveToFile(*outputProto, "..\\..\\data\\\proto2016\\tests\\PerHatSaved.asciiproto", true);


	TemplateProtoConverter converter2;
	auto protoRead2 = converter1.loadFromFile("..\\..\\data\\\proto2016\\tests\\PerHatSaved.asciiproto");
	Template* temp2 = converter1.ConvertToTemplate(*protoRead1);



}



void testComposeMotionAndSave(){


	TemplateProtoConverter converter1;
	auto protoRead1 = converter1.loadFromFile("..\\..\\data\\\proto2016\\BeamBasicContactless\\template.asciiproto");
	Template* temp1 = converter1.ConvertToTemplate(*protoRead1);
	temp1->recomputeCenter();
	temp1->translate(-temp1->getCenter());
	temp1->updateFullQ(temp1->getFullQ());
	temp1->moveToGround();


	TemplateProtoConverter converter2;
	auto protoRead2 = converter2.loadFromFile("..\\..\\data\\\proto2016\\BeamBasic\\template.asciiproto");
	Template* temp2 = converter2.ConvertToTemplate(*protoRead2);
	temp2->recomputeCenter();
	temp2->translate(-temp2->getCenter());
	temp2->updateFullQ(temp2->getFullQ());
	temp2->moveToGround();
	Template* combinedTemplate = TemplateManipulations::ConnectWithGrammar(temp1, temp2);

	std::cout << "composing the motion" << std::endl;
	combinedTemplate->getGeometry()->write("..\\..\\data\\combinedExample_before.stl"); 

	auto workingTemplate = new WorkingTemplate(combinedTemplate);
	for (double i = 0; i < 1; i+=0.05){
		std::stringstream filename;
		std::cout << "for t = " << i << std::endl;
		filename << "..\\..\\data\\combinedExample_anim_t"  << i*10 <<  ".stl";
		workingTemplate->getAnimation(i)->write(filename.str());
	}


	TemplateProtoConverter outputConverter;
	auto outputProto = outputConverter.ConvertToProto(combinedTemplate);
	outputConverter.saveToFile(*outputProto, "..\\..\\data\\\proto2016\\tests\\composedWMotionB.asciiproto", true);

	TemplateProtoConverter converter3;
	auto protoRead3 = converter3.loadFromFile("..\\..\\data\\\proto2016\\tests\\composedWMotionB.asciiproto");
	Template* temp3 = converter3.ConvertToTemplate(*protoRead3);
	temp3->recomputeCenter();
	temp3->translate(-temp3->getCenter());
	temp3->updateFullQ(temp3->getFullQ());
	temp3->moveToGround();	
	temp3->getGeometry()->write("..\\..\\data\\combinedExample_after.stl"); 


}



/*
void libNLopt(){

	TemplateProtoConverter converter1;
	auto protoRead1 = converter1.loadFromFile("..\\..\\data\\\proto2016\\tests\\A.BodyHex.asciiproto");
	Template* temp1 = converter1.ConvertToTemplate(*protoRead1);
	WorkingTemplate * wTemp = new WorkingTemplate(temp1);
	OptimizationWeights weights( 3, 10, 1, 1);
	RobotOptimizer* robotOptimizer = new RobotOptimizer(wTemp, weights);

	Eigen::VectorXd optimizedq;
	double result;

	robotOptimizer->optimize(optimizedq, result);

	std::cout <<"the optimal result is " << optimizedq.transpose() << " which has error " << result << std::endl;

}
*/

void testDrivingSelection(){
	TemplateProtoConverter converter1;
	auto protoRead1 = converter1.loadFromFile("..\\..\\data\\\proto2016\\MotionExamples\\quad_goodInput.asciiproto");
	Template* temp1 = converter1.ConvertToTemplate(*protoRead1);
	WorkingTemplate * wTemp = new WorkingTemplate(temp1);
	DrivingJointSelection::computeDrivingJoints(wTemp->kinchain); 


}
/*
void testRobotOptimizer(std::string filename){

	TemplateProtoConverter converter1;
	//auto protoRead1 = converter1.loadFromFile("..\\..\\data\\\proto2016\\tests\\quad_goodInput.asciiproto");
	auto protoRead1 = converter1.loadFromFile(filename.c_str());
	Template* temp1 = converter1.ConvertToTemplate(*protoRead1);
	WorkingTemplate * wTemp = new WorkingTemplate(temp1);
	OptimizationWeights weights( 3, 10, 1, 1);
	RobotOptimizer* robotOptimizer = new RobotOptimizer(wTemp, weights);
}
*/

void showContactPatchFromKinChain(std::string protfilename){
	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protfilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
	
	KinChain* kinchain = new KinChain(temp); 
	kinchain->displayNodes();


}
void testGetGaits(std::string protfilename){

	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protfilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
	
	KinChain* kinchain = new KinChain(temp); 
	auto options = kinchain->getDrawableGaitOptions();
	std::cout << "polygon = " << std::endl;
	for ( int i = 0; i < options.polygon.size(); i++){
		std::cout <<  options.polygon[i].transpose() << ",  ";
	}	
	std::cout << std::endl;
	
	for ( int i = 0; i < options.legPositions.size(); i++){
		std::cout << "leg pos = " << options.legPositions[i].transpose() << std::endl;
	}
	for each (auto l in options.gaitOptions){
		std::cout << "gait option: " ;
		for each (auto i in l){
			std::cout << i << " ";
		}
		std::cout << std::endl;
	}
	//kinchain->updateGaitOption(60, 0);
	//std:cout << "current theta = " << kinchain->getCurrentTheta() << std::endl;
	//std::cout << "current gait option = " << kinchain->getSelectedGaitOption() << std::endl;
	//kinchain->updateGait(0.01);
	//auto geoVec = kinchain->getAnimation();
	//for(int i = 0; i < geoVec.size(); i++){
	//	std::stringstream geoFileName;
	//	geoFileName << "..\\..\\data\\test\\articulationNeg_t" << i << ".stl";
	//	geoVec[i]->write(geoFileName.str());
	//}

	TemplateProtoConverter outputConverter;
	auto outputProto = outputConverter.ConvertToProto(temp);
	outputConverter.saveToFile(*outputProto, "..\\..\\data\\\proto2016\\tests\\testMotion.asciiproto", true);


}


void testOptimization(std::string protfilename, std::string optfilename){

	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protfilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
	WorkingTemplate * wTemp = new WorkingTemplate(temp);
	converter.AddGaitInfoToKinChain(*protoRead, wTemp->kinchain);

	wTemp->kinchain->updateControllers(0);
	//wTemp->kinchain->getDrawableGaitOptions();
	//wTemp->kinchain->updateGaitOption(30, 1);
	wTemp->kinchain->setRounds(3);
	wTemp->kinchain->updateGait(0.01);
	auto geoVec = wTemp->kinchain->getAnimation();
	for(int i = 0; i < geoVec.size(); i++){
		std::stringstream geoFileName;
		geoFileName << "..\\..\\data\\test\\articulation_tNeg" << i << ".stl";
		geoVec[i]->write(geoFileName.str());
	}

	double oldStability = wTemp->kinchain->getStability() ;
	double oldSlip = wTemp->kinchain->getSlip();
	double oldSpeed = wTemp->kinchain->getSpeed();
	double oldStraightness = wTemp->kinchain->getPerpendicularError();
	double oldCurvature = wTemp->kinchain->getMeanCurvature();
	double oldWobbliness = wTemp->kinchain->getToppling();
	std::cout << "the stability is " << wTemp->kinchain->getStability() << std::endl;
	std::cout << "the slip is " << wTemp->kinchain->getSlip() << std::endl;
	std::cout << "the speed is " << wTemp->kinchain->getSpeed() << std::endl;
	std::cout << "the straightness is " << wTemp->kinchain->getPerpendicularError() << std::endl;
	std::cout << "the curvature is " << wTemp->kinchain->getMeanCurvature() << std::endl;
	std::cout << "the wobbliness is " << wTemp->kinchain->getToppling() << std::endl;
	
	wTemp->stabilize(FunctionEvalStability::StabilizationObjective::WOBBLINESS);

	TemplateProtoConverter outputConverter;
	auto outputProto = outputConverter.ConvertToProto(wTemp->getTemplate());
	outputConverter.saveToFile(*outputProto, optfilename.c_str(), true);

	//while (oldStability > 0) {
	//	GeometryOptimizationWeights weights(10, 3, 1, 1);
	//	GeometryOptimizer opt(wTemp, weights);
	//	opt.optimize(); 

	std::cout << "\n\nRESULTS\n\n";
		wTemp->kinchain->updateGait(0.01);
		std::cout << "the stability is " << wTemp->kinchain->getStability() << " vs " << oldStability << std::endl;
		std::cout << "the slip is " << wTemp->kinchain->getSlip()  << " vs " << oldSlip << std::endl;
		std::cout << "the speed is " << wTemp->kinchain->getSpeed()  << " vs " << oldSpeed << std::endl;
		std::cout << "the straightness is " << wTemp->kinchain->getPerpendicularError()  << " vs " << oldStraightness << std::endl;
		std::cout << "the curvature is " << wTemp->kinchain->getMeanCurvature()  << " vs " << oldCurvature << std::endl;
		std::cout << "the wobbliness is " << wTemp->kinchain->getToppling()  << " vs " << oldWobbliness << std::endl;

	//	oldStability = wTemp->kinchain->getStability();

	//	system("pause");
	//}
	//Eigen::VectorXd temp1 = wTemp->getTemplate()->getFullQ();

	//wTemp->getTemplate()->updateFullQ(opt.origConfiguration);
	//GeometryOptimizer opt2(wTemp, weights);
	//nlopt_srand_time();
	//opt2.optimize(); 

	//wTemp->kinchain->updateGait(0.01);
	//std::cout << "the stability is " << wTemp->kinchain->getStability() << " vs " << oldStability << std::endl;
	//std::cout << "the slip is " << wTemp->kinchain->getSlip()  << " vs " << oldSlip << std::endl;
	//std::cout << "the speed is " << wTemp->kinchain->getSpeed()  << " vs " << oldSpeed << std::endl;
	
	//Eigen::VectorXd temp2 = wTemp->getTemplate()->getFullQ();


	auto newgeoVec = wTemp->kinchain->getAnimation();
	for(int i = 0; i < geoVec.size(); i++){
		std::stringstream geoFileName;
		geoFileName << "..\\..\\data\\test\\articulationnew_t" << i << ".stl";
		newgeoVec[i]->write(geoFileName.str());
	}
	
}




void saveSTL(std::string protfilename, std::string outfilename){
	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protfilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
	
	temp->getGeometry()->write(outfilename); 


}



void testGaitOptions(std::string protfilename){

	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protfilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
	
	KinChain* kinchain = new KinChain(temp); 
	auto options = kinchain->getDrawableGaitOptions();
	for each (auto l in options.gaitOptions){
		std::cout << "gait option: " ;
		for each (auto i in l){
			std::cout << i << " ";
		}
		std::cout << std::endl;
	}

}

void outputMotion(std::string protofilename, std::string outfilename){

	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);
	
	KinChain* kinchain = new KinChain(temp); 
	kinchain->outputMotionToFile(outfilename);
}

void outputMotionFab(std::string protofilename, std::string outfilename){

	TemplateProtoConverter converter;
	auto protoRead = converter.loadFromFile(protofilename.c_str());
    Template* temp = converter.ConvertToTemplate(*protoRead);

	WorkingTemplate * wt = new WorkingTemplate(temp);
	
	std::cout << protofilename << std::endl;
	std::cout << "total: " << wt->metrics->getFabricationCost() << std::endl;

	wt->elecgraph->generateCode(outfilename);
	std::cout << wt->elecgraph->getAssemblyInstr();
	
}



int main(int argc, char* argv[])
{



	initializeLogging();

	//showPatches("..\\..\\data\\proto2016\\NewBodyT2.asciiproto");


	//outputMotion("..\\..\\data\\proto2016\\examples\\print_monkey_final.asciiproto", "..\\..\\data\\code\\test\\motion.txt");
	//outputMotionFab("..\\..\\data\\proto2016\\examples\\print_monkey_final.asciiproto", "..\\..\\data\\code\\test\\");
	//test3DSTLgen("..\\..\\data\\proto2016\\examples\\3legged_barrelshort_gait1.asciiproto", "3legged_barrelshort_gait1_3d_hard");	 	

	//test3DSTLgen("..\\..\\data\\proto2016\\examples\\killerBasket_bad.asciiproto", "killerBasket_bad_3d");	 

	//test3DSTLgenStationary("E:\\Dropbox (MIT)\\DB_3DPrintableRobots\\ProtosToRender\\UserStudy_Task2\\user13_p4.asciiproto", "print_ant_final");	

	//test3DSTLgen("..\\..\\data\\proto2016\\examples\\killerBasket_bad_servospacing.asciiproto", "killerBasket_bad_topple", 0);	 	
	//test3DSTLgen("..\\..\\data\\proto2016\\examples\\killerBasket_good_servospacing.asciiproto", "killerBasket_good", 0);	 	
	//test3DSTLgen("..\\..\\data\\proto2016\\examples\\killerBasket_bad_servospacing.asciiproto", "killerBasket_bad_gait1", 1);	 	
	//test3DSTLgen("..\\..\\data\\proto2016\\examples\\opt_wobbly_walkingfish.asciiproto", "walkingfish_wobbly", 0);	 	

	
	

	//std::cout << "MODEL A:\n";
	//testKinematics("..\\..\\data\\proto2016\\examples\\mystical_videoopt_approx.asciiproto", .01);
	//std::cout << "\n\nMODEL B:\n";
	//testKinematics("..\\..\\data\\proto2016\\examples\\user_study1_modelB.asciiproto", .01);
	//std::cout << "\n\nstrangemotion3:\n";
	//testKinematics("..\\..\\data\\proto2016\\examples\\strangeMotion3.asciiproto", .01);
	//std::cout << "\n\nmouse:\n";
	//testKinematics("..\\..\\data\\proto2016\\examples\\slip.asciiproto", .01);
	//testKinematics("..\\..\\data\\proto2016\\MotionTests\\test2_6Legs.asciiproto", .05);
	//testKinematics("..\\..\\data\\proto2016\\MotionTests\\test8_2Legs2DoubleBeams.asciiproto", .01);
	//testKinematics("..\\..\\data\\proto2016\\MotionExamples\\hex_basic.asciiproto", .05);
	//testKinematics("..\\..\\data\\proto2016\\examples\\3leg.asciiproto", .01);

	//testOptimization("..\\..\\data\\proto2016\\examples\\walkingfish.asciiproto", 
	//	"..\\..\\data\\proto2016\\examples\\test_temp.asciiproto");


	//testOptimization("..\\..\\data\\proto2016\\examples\\4legged_train_nonsymmetric.asciiproto",

//		"..\\..\\data\\proto2016\\examples\\optimized_wobbly_4legged_train_nonsymmetric.asciiproto");

	//	"..\\..\\data\\proto2016\\examples\\optimized_wobbly_4legged_train_nonsymmetric.asciiproto");

	//outputAnimation("..\\..\\data\\proto2016\\examples\\4legged_train_nonsymmetric.asciiproto",
	//	"..\\..\\data\\proto2016\\examples\\optimized_wobbly_4legged_train_nonsymmetric.asciiproto");
	


	//return 0;
	//testKinematics("..\\..\\data\\Truck.asciiproto", .05);
	/*
	saveSTL("..\\..\\data\\\proto2016\\BeamConvex\\template.asciiproto", "..\\..\\data\\\meshes2016\\fixed\\BeamConvex.stl");
	saveSTL("..\\..\\data\\\proto2016\\BeamConvexContactless\\template.asciiproto", "..\\..\\data\\\meshes2016\\fixed\\BeamConvexContactless.stl");
	saveSTL("..\\..\\data\\\proto2016\\PeripheralSiggraph\\template.asciiproto", "..\\..\\data\\\meshes2016\\fixed\\PeripheralSiggraph.stl");
	saveSTL("..\\..\\data\\\proto2016\\PeripheralBunny\\template.asciiproto", "..\\..\\data\\\meshes2016\\fixed\\PeripheralBunny.stl");
	saveSTL("..\\..\\data\\\proto2016\\BodyTrain\\template.asciiproto", "..\\..\\data\\\meshes2016\\fixed\\BodyTrain.stl");
	saveSTL("..\\..\\data\\\proto2016\\BeamRhex\\template.asciiproto", "..\\..\\data\\\meshes2016\\fixed\\BeamRhex.stl");
	saveSTL("..\\..\\data\\\proto2016\\BeamRhexContactless\\template.asciiproto", "..\\..\\data\\\meshes2016\\fixed\\BeamRhexContactless.stl");
	saveSTL("..\\..\\data\\\proto2016\\BeamSlanted\\template.asciiproto", "..\\..\\data\\\meshes2016\\fixed\\BeamSlanted.stl");
	saveSTL("..\\..\\data\\\proto2016\\BeamSlantedContactless\\template.asciiproto", "..\\..\\data\\\meshes2016\\fixed\\BeamSlantedContactless.stl");
	*/
	//saveSTL("..\\..\\data\\\proto2016\\BeamConcave\\template.asciiproto", "..\\..\\data\\\meshes2016\\new\\BeamConcave.stl");
	//saveSTL("..\\..\\data\\\proto2016\\BeamConcaveContactless\\template.asciiproto", "..\\..\\data\\\meshes2016\\new\\BeamConcaveContactless.stl");
	//saveSTL("..\\..\\data\\\proto2016\\BeamRhex\\template.asciiproto", "..\\..\\data\\\meshes2016\\new\\BeamRhex.stl");
	//saveSTL("..\\..\\data\\\proto2016\\BeamRhexContactless\\template.asciiproto", "..\\..\\data\\\meshes2016\\new\\BeamRhexContactless.stl");
	//saveSTL("..\\..\\data\\\proto2016\\BeamSlanted\\template.asciiproto", "..\\..\\data\\\meshes2016\\new\\BeamSlanted.stl");
	//saveSTL("..\\..\\data\\\proto2016\\BeamSlantedContactless\\template.asciiproto", "..\\..\\data\\\meshes2016\\new\\BeamSlantedContactless.stl");
	//saveSTL("..\\..\\data\\\proto2016\\PeripheralAntenna\\template.asciiproto", "..\\..\\data\\\meshes2016\\new\\PeripheralAntenna.stl");
	//saveSTL("..\\..\\data\\\proto2016\\PeripheralCharacterFixed\\template.asciiproto", "..\\..\\data\\meshes2016\\new\\PeripheralCharacterFixed.stl");
	//saveSTL("..\\..\\data\\\proto2016\\PeripheralSpoiler\\template.asciiproto", "..\\..\\data\\\meshes2016\\new\\PeripheralSpoiler.stl");
	//saveSTL("..\\..\\data\\\proto2016\\PeripheralWings\\template.asciiproto", "..\\..\\data\\\meshes2016\\new\\PeripheralWings.stl");

	//testLibNLopt();
	//libNLopt();

	//realignProtos();
	//showPatches("..\\..\\data\\\proto2016\\BeamBasicContactlessCentered\\template.asciiproto");
	
	//testPrinting("..\\..\\data\\\proto2016\\tests\\BeamPrintTest.asciiproto", "BeamPrintTestFill");	 
	//realignProtos();
	//showPatches("..\\..\\data\\\proto2016\\PeripheralSpoiler\\template.asciiproto");
	//showContactPoints("..\\..\\data\\\proto2016\\BeamHeartContactless\\template.asciiproto");

//	testPrinting("..\\..\\data\\proto2016\\BodyT\\template.asciiproto", "BodyT");	 

	testPrinting("..\\..\\data\\protoTesting\\jeff.asciiproto", "jeff");	 

	//testPrinting("..\\..\\data\\protoTesting\\WheelCrabBasket.asciiproto", "WheelCrabBasket");	 

	
	//showPatches("..\\..\\data\\\proto2016\\tests\\BeamPrintTest.asciiproto");
	

	
	/*----------------Andy's testing code---------------------------*/
	//realignProtos();
//	exampleForAndy("BeamHex");
//	exampleForAndy("BeamBasic");
//	exampleForAndy("BeamTriangle");

	//system("pause");

	
	//testCompositionPeripherals(); 
	//testComposition(); 
	
	
	/*----------------Adriana testing code---------------------------*/
	

	//outputMotion("..\\..\\data\\proto2016\\examples\\print_movingHouse.asciiproto", "..\\..\\data\\proto2016\\examples\\movinghouse_");
	//outputMotion("..\\..\\data\\proto2016\\examples\\print_mouse.asciiproto", "..\\..\\data\\proto2016\\examples\\mouse");
	//outputMotion("..\\..\\data\\proto2016\\examples\\print_ant.asciiproto", "..\\..\\data\\proto2016\\examples\\ant");
	//outputMotion("..\\..\\data\\proto2016\\examples\\print_car.asciiproto", "..\\..\\data\\proto2016\\examples\\car");
	//outputMotion("..\\..\\data\\proto2016\\examples\\print_mystical.asciiproto", "..\\..\\data\\proto2016\\examples\\mystical");
	//outputMotion("..\\..\\data\\proto2016\\examples\\print_truck.asciiproto", "..\\..\\data\\proto2016\\examples\\truck");
	//outputMotionFab("..\\..\\data\\proto2016\\examples\\print_mouse.asciiproto", "..\\..\\data\\proto2016\\examples\\mouse");

	//debugWheelRotation();
	//testOptimization("..\\..\\data\\proto2016\\MotionTests\\test1_4Legs.asciiproto");
	//system("pause");
	//showContactPoints("..\\..\\data\\proto2016\\MotionTests\\wheel_afterRotate.asciiproto");
	//system("pause");

	//testComposeMotionAndSave();
	//testGetGaits("..\\..\\data\\proto2016\\MotionTests\\test1_4Legs.asciiproto");
	//system("pause");
	//testGetGaits("..\\..\\data\\proto2016\\MotionTests\\test11_4DoubleBeams1TripleBeam.asciiproto.asciiproto");
	//testGetGaits("..\\..\\data\\SandCrawler2.asciiproto");

	//testOptimization("..\\..\\data\\proto2016\\MotionTests\\test3_5LegsB_unstable.asciiproto");

	

	//showContactPatchFromKinChain("..\\..\\data\\\proto2016\\MotionExamples\\quad_goodInput.asciiproto");
//	system("pause");
	//openAndSaveAgain("..\\..\\data\\\proto2016\\tests\\composedWMotionB.asciiproto", "..\\..\\data\\\proto2016\\tests\\composedWMotionB2.asciiproto");
//	system("pause");
//	showContactPoints("..\\..\\data\\\proto2016\\BeamBasic\\template.asciiproto");
//	system("pause");
	



	//showConnections("..\\..\\data\\\proto2016\\tests\\BodySleekWeirdPeripherals.asciiproto");
	//system("pause"); 
	//showPatches("..\\..\\data\\\proto2016\\PeripheralHat\\template.asciiproto");

//	testComposePatches();
	//testPrinting("..\\..\\data\\\proto2016\\tests\\BodySleekWeirdPeripherals.asciiproto", "secondBug");
	//showPatches("..\\..\\data\\\proto2016\\HitchA\\template.asciiproto"); 
	//system("pause"); 
	//showPatches("..\\..\\data\\\proto2016\\SnowPlow2\\template.asciiproto"); 
	//system("pause"); 
	//showPatches("..\\..\\data\\\proto2016\\BodySleek\\template.asciiproto"); 
	//system("pause"); 
//	showPatches("..\\..\\data\\\proto2016\\BodySleek\\template.asciiproto");
	

	//	testPrinting("..\\..\\data\\\proto2016\\tests\\WontPrint.asciiproto", "BeamPrintTestFill");	
//	testRobotOptimizer("..\\..\\data\\\proto2016\\MotionExamples\\quad_goodInput.asciiproto");
	//showPatches("..\\..\\data\\\proto2016\\BodySleek\\template.asciiproto");

	//	testPrinting("..\\..\\data\\\proto2016\\tests\\WontPrint.asciiproto", "BeamPrintTestFill");	
	//testRobotOptimizer("..\\..\\data\\\proto2016\\MotionExamples\\quad_goodInput.asciiproto");
	//testRobotOptimizer();
	//testDrivingSelection();

		//savePerpatch();

	//testComposeMotionAndSave();

	//displayMotionAndAnimate("..\\..\\data\\\proto2016\\tests\\composedWMotion_working.asciiproto");
	//testComposeMotionAndSave();
	//displayMotionAndAnimate("..\\..\\data\\\proto2016\\tests\\composedWMotionB.asciiproto");


	//testCompositionPeripherals();
	//showSemantics("..\\..\\data\\\proto2016\\BeamBasic\\template.asciiproto");	
	//showPatches("..\\..\\data\\\proto2016\\tests\\BeamBasic.asciiproto");
	//openAndSaveAgain("..\\..\\data\\\proto2016\\BeamBasic\\template.asciiproto", "..\\..\\data\\\proto2016\\BeamBasic\\template2.asciiproto");
	
	//showContactPoints("..\\..\\data\\\proto2016\\BeamBasic\\template.asciiproto");	
	//showSemantics("..\\..\\data\\\proto2016\\BeamBasic\\template2.asciiproto");	
//	std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> > rotations;
//	rotations.push_back( Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())));
//	orientTemplateCorrectly("..\\..\\data\\\proto2016\\tests\\beamTest.asciiproto", rotations);
//	system("pause");
//	showContactPoints("..\\..\\data\\\proto2016\\tests\\beamTest.asciiproto");

//	openAndSaveAgain("..\\..\\data\\\proto2016\\BeamBasic\\template2.asciiproto", "..\\..\\data\\\proto2016\\BeamBasic\\template3.asciiproto");
//	showContactPoints("..\\..\\data\\\proto2016\\BeamBasic\\template3.asciiproto");


	//composeWithMotion(); 
	//system("pause");
	//openAndDisplayMotion();

	//testComposePatches();
	//system("pause");
	//showPatches("..\\..\\data\\\proto2016\\tests\\composedWMotion.asciiproto");
	//showPatches("..\\..\\data\\\proto2016\\BodySleek\\template.asciiproto");


	//
	//system("pause");


	//testLoadingSub("C:\\ResearchCode\\FabByExampleExperimental\\FabByExample\\data\\proto\\ParameciumFull.asciiproto", 0, "spider2");	 
    //testPrinting("C:\\ResearchCode\\FabByExampleExperimental\\FabByExample\\data\\protofinal\\template.asciiproto", "HalfAntRhex");	 
	//testPrinting("C:\\ResearchCode\\FabByExampleExperimental\\FabByExample\\data\\proto\\template.asciiproto", "HalfRhex");	 
	//testPrinting("C:\\ResearchCode\\FabByExampleExperimental\\FabByExample\\data\\proto\\Paramecium.asciiproto", "InchWorm");	 
	//testNewAnim("..\\..\\data\\protoTesting\\Biped.asciiproto");
	//testEvalCenterOfMassB("..\\..\\data\\protofinal\\ProtoAutomated\\(%class '__main__.CrabFull')%.asciiproto");
	 //testEvalCenterOfMassB("..\\..\\data\\protofinal\\crab_toAnimate.asciiproto");
	//testEvalCenterOfMassB("..\\..\\data\\protofinal\\ProtoAutomated\\(%class '__main__.CrabFull')%.asciiproto");
	//testNewAnim("..\\..\\data\\protofinal\\ProtoAutomated\\(%class '__main__.CrabFull')%.asciiproto");
	//testLoadingSub("..\\..\\data\\protofinal\\Ant.asciiproto", 0, "Ant");
	//testStabilize("..\\..\\data\\protofinal\\ProtoAutomated\\(%class '__main__.CrabFull')%.asciiproto");
	//testStabilize("..\\..\\data\\protofinal\\Biped_const.asciiproto");
	//testStabilize("..\\..\\data\\protofinal\\ProtoAutomated2\\(%class '__main__.BipedFullNew')%.asciiproto");

	//testLineSearch("..\\..\\data\\protofinal\\ProtoAutomated\\(%class '__main__.CrabFull')%.asciiproto");

	//testNewAnim("..\\..\\data\\protofinal\\WheelCrabBasket.asciiproto");
	
	//CrabConstrainedReal

	//testComposition();


	//testStabilize("..\\..\\data\\protofinal\\CrabConstrainedReal.asciiproto");
	//showPatches("..\\..\\data\\\proto2016\\tests\\BodyBasic.asciiproto");
	//fixOrientations();
	//showPatches("..\\..\\data\\\proto2016\\BodyBasic.asciiproto");
	//system("pause");
	//showPatches("..\\..\\data\\\proto2016\\BeamTriangle.asciiproto");
	//system("pause");
	//testStabilize("..\\..\\data\\protofinal\\CrabConstrainedReal.asciiproto");
	/*
	showPatches("..\\..\\data\\\proto2016\\tests\\CrabServoPatchFixed.asciiproto");
	system("pause");
	showPatches("..\\..\\data\\\proto2016\\tests\\BeamTriangle.asciiproto");
	system("pause");
	*/
	//showPatches("..\\..\\data\\\proto2016\\tests\\Crab2016.asciiproto");
//	showPatches("..\\..\\data\\\proto2016\\tests\\BodyT.asciiproto");
//	system("pause");
//	showPatches("..\\..\\data\\\proto2016\\tests\\BeamTriangle.asciiproto");
//	showPatches("..\\..\\data\\\proto2016\\tests\\CrabServoPatchFixed.asciiproto");
//	system("pause");



	//system("pause"); 
	//showPatches("..\\..\\data\\\proto2016\\CrabServoPatch.asciiproto");
	//testStabilize("..\\..\\data\\protofinal\\CrabConstrainedReal.asciiproto");
	//testStabilize("..\\..\\data\\protofinal\\flexCrab.asciiproto");
	//testNewAnim("..\\..\\data\\protofinal\\ProtoAutomated2\\(%class '__main__.RhexHalfAnt')%.asciiproto");

	//testLineSearch("..\\..\\data\\protofinal\\ProtoAutomated\\(%class '__main__.CrabFull')%.asciiproto");

	//testLineSearch("..\\..\\data\\protofinal\\ProtoAutomated\\(%class '__main__.CrabFull')%.asciiproto");

	/*-------------------Andy testing code------------------ */

	//testLoadingSub("C:\\Users\\Andrew Spielberg\\Dropbox\\DB_3DPrintableRobots\\CompleteDesigns\\SpiderRevoluteIncompleteControl.asciiproto", 0, "spider2");	 
	//testPrinting("C:\\Users\\Andrew Spielberg\\Dropbox\\DB_3DPrintableRobots\\TestDesigns\\HingeRevolute.asciiproto", "hinge27");	 	
	//generateSTL("..\\..\\data\\protoTesting\\Spider.asciiproto");
	// To show the kinematics tree
	//showKinematicTree("..\\..\\data\\protoTesting\\CrabArticulatedFixed.asciiproto");	
	//generateSTL("..\\..\\data\\protoTesting\\CrabArticulatedFixed_wavegait.asciiproto");


	/*----------------Cindy testing code ---------------------------*/

	//robot prototypes
	//generateSTL("..\\..\\data\\protoTesting\\CrabFull.asciiproto");
	//testPrinting("..\\..\\data\\proto2016\\examples\\print_mystical.asciiproto","beatle");
	//testPrinting("..\\..\\data\\proto2016\\BeamTriangle\\Template.asciiproto","triangle");
	//testPrinting("..\\..\\data\\protoTesting\\CrabFull.asciiproto","CrabFull");
	//testPrinting("..\\..\\data\\protoTesting\\CrabFull_tall.asciiproto","CrabFull_tall");
	//testPrinting("..\\..\\data\\protoTesting\\CrabFull_wide.asciiproto","CrabFull_wide");

	//generateSTL("..\\..\\data\\protoTesting\\BipedNew.asciiproto");
	//testNewAnim("..\\..\\data\\protoTesting\\Biped.asciiproto");
	//testManipulation("..\\..\\data\\protofinal\\ProtoAutomated\\(%class '__main__.CrabFull')%.asciiproto");
	//testPrinting("..\\..\\data\\protoTesting\\Biped.asciiproto","Biped");
	//testPrinting("..\\..\\data\\protoTesting\\BipedNew.asciiproto","BipedNew");
	//testPrinting("..\\..\\data\\protoTesting\\Biped_alljoints.asciiproto","Biped_alljoints");


	//testPrinting("..\\..\\data\\protoTesting\\Biped_alljoints.asciiproto","Biped_alljoints");

	//generateSTL("..\\..\\data\\protoTesting\\Segway.asciiproto");
	//testPrinting("..\\..\\data\\protoTesting\\Segway.asciiproto","Segway");

	
	/*---------------------Wei testing code-------------------------- */

	//testNewAnim("..\\..\\data\\protofinal\\Biped.asciiproto");
	//testMetrics("..\\..\\data\\protoTesting\\doubleBeamSimp.asciiproto");
	//testLoadingSub("..\\..\\data\\protofinal\\Ant.asciiproto", 0, "Ant");
	//testLoadingSub("..\\..\\data\\protofinal\\ProtoAutomated\\(%class '__main__.BipedFull')%.asciiproto", 0, "bipedfull");

	 /*-----------------Robin testing code----------------------------*/
	//testLinearExprCopyPerformance();
	//testFastHashmap::test();
	//FabProfiling::printCounters();

	std::cout << "done" << std::endl;
	system("pause"); 
}


