#include "geometry.h"
#include "TemplateManipulations.h"
#include "ConnectingInfo.h"
#include <ConstraintsEval.h>
#include "Template.h"
#include <templateGroup.h>
#include "templateElement.h"
#include "element_symbolic.h"
#include "element_motion.h"
#include <constraints.h>
#include <articulation.h>
#include "NewPatch.h"
#include "controller.h"
#include <queue>
#include "../FBE_Proto/TemplateProtoConverter.h"
#define DEBUG_WRITEPATCHTOFILE false
#define DEBUG_OVERLAP_CONSTRAINT false
#define DEBUG_SERVO false
#define DEBUG_PATCH false
#define ENABLE_COLLISION_DETECTION 1

int(*OverlapPreventionHack::countOverlapness)(drawing::Drawing const* drawing) = nullptr;
bool OverlapPreventionHack::reflectAllDrawings = false;

PatchPair TemplateManipulations::Snap(Template* main, Template* add, PatchPair const& current, double maxDistance, ConnectingInfo& connectingInfo) {
	NewSnapping snapping(main, add, false);
	snapping.snap(current, maxDistance, connectingInfo); 
	return connectingInfo.matchingNewPatches[0];
}

void TemplateManipulations::MultiSnap(Template* main, Template* add, double maxDistance, ConnectingInfo* connectingInfo) {
	NewSnapping snapping(main, add, false);
	snapping.multiSnap(maxDistance, connectingInfo);
}

unordered_map<Template*, int> findAllTemplateIDs(Template* tmpl) {
	unordered_map<Template*, int> result;
	for each (auto child in tmpl->getTemplatesByScope(TreeScope::DESCENDANTS)) {
		result[child] = child->getID();
	}
	return result;
}




void constrainLegOverlap(NewPatch* wpatch, Template * wTemplate, Template* addTemplate, std::vector<Constraint*> & constraints3D){

	std::pair< ServoLinePatch*, int> servoConnectionReference;
	servoConnectionReference.second = -1; 
	std::vector<ServoLinePatch*> servoLinePatches;
	for each ( auto p in wTemplate->getAllPatches()){
		if(dynamic_cast<ServoLinePatch*>(p) != nullptr){
			servoLinePatches.push_back(dynamic_cast<ServoLinePatch*>(p));
		}
	}
	for each( auto s in servoLinePatches){
		for(int i = 0; i < s->spacings.size(); i++){
			bool isRelated = false;
			for each (auto t in s->spacings[i].associatedTemplates){
				if	(wpatch->isRelated(t)){
					isRelated = true;
				}
			}
			if(isRelated){
				if(servoConnectionReference.second != -1){
					std::cout << "found more than one servo Conenctions " << std::endl;
				}
				servoConnectionReference.first = s;
				servoConnectionReference.second = i;
			}
		}
	}


	if(DEBUG_OVERLAP_CONSTRAINT){
		VLOG(3) << "in the begining the servo line patched looked like this" << std::endl;
		for each ( auto s in servoConnectionReference.first->spacings){
			s.display();
		}
	}

	if(servoConnectionReference.second != -1){
	
		Eigen::Vector3d MainDir = servoConnectionReference.first->getNormal();
		Eigen::Quaterniond aroundY90(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY()));
		Eigen::Vector3d widthDirection = aroundY90.toRotationMatrix()*MainDir;
		auto bbox = addTemplate->evalLocalBoundingBox();
		//todo the with could be x or z -> check!
		LinearExpr current_w = widthDirection.x()*(bbox.max.x - bbox.min.x) +
								widthDirection.y()*(bbox.max.y - bbox.min.y)+
								widthDirection.z()*(bbox.max.z - bbox.min.z);
		LinearExpr current_h = (bbox.max.y - bbox.min.y);


		if(DEBUG_OVERLAP_CONSTRAINT){
			VLOG(3) << "the old spacing was" << std::endl;
			servoConnectionReference.first->spacings[servoConnectionReference.second].display();
		}
		servoConnectionReference.first->spacings[servoConnectionReference.second].addInfo( current_w, current_h, addTemplate);

		if(DEBUG_OVERLAP_CONSTRAINT){
			VLOG(3) << "the new spacing is" << std::endl;
			servoConnectionReference.first->spacings[servoConnectionReference.second].display();
		}
		
		for (int i = 0; i < servoConnectionReference.first->spacings.size(); i++){
			if( i != servoConnectionReference.second){
				double order = (servoConnectionReference.first->spacings[servoConnectionReference.second].alpha.eval(SymbolicAssignment::USE_CURRENT_VALUES) < servoConnectionReference.first->spacings[i].alpha.eval(SymbolicAssignment::USE_CURRENT_VALUES))?  1 : -1;
				LinearExpr ineq2 =  order*(servoConnectionReference.first->spacings[servoConnectionReference.second].alpha -servoConnectionReference.first->spacings[i].alpha) + servoConnectionReference.first->spacings[i].separation_h + servoConnectionReference.first->spacings[servoConnectionReference.second].separation_h; 
				constraints3D.push_back(new Constraint(Constraint::ConstraintRelation::INEQ,  ineq2));
				for each ( auto w in servoConnectionReference.first->spacings[i].separation_w){
					LinearExpr ineq1 =  order*(servoConnectionReference.first->spacings[servoConnectionReference.second].alpha -servoConnectionReference.first->spacings[i].alpha) + w*0.5 + current_w*0.5; 
					constraints3D.push_back(new Constraint(Constraint::ConstraintRelation::INEQ,  ineq1));
				}
			}
		}

	
	
	}

	if(DEBUG_OVERLAP_CONSTRAINT){
		VLOG(3) << "in the end the servo line patched looked like this" << std::endl;
		for each ( auto s in servoConnectionReference.first->spacings){
			s.display();
		}
	}

}


NewConnection* ConnectServo(ServoLinePatch* p1, ServoPointPatch* p2, 
	Template* addTemplate, Template* composed,
	std::vector<Constraint*> & constraints3D,
	std::vector<Constraint*> & constraints2D,
	Element_Motion* _elementMotion){


		// step 1 rotate the servo correctly:
		Eigen::Vector3d MainDir = p1->getNormal();
		Eigen::Vector3d AddDir = (-1.0)*p2->getNormal();
		Eigen::Quaterniond qRotation = Eigen::Quaterniond::FromTwoVectors(AddDir, MainDir);
		addTemplate->rotate(addTemplate->getCenter(), qRotation);

		Eigen::Quaterniond aroundY90(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY()));
		Eigen::Vector3d widthDirection = aroundY90.toRotationMatrix()*MainDir;

	
		Point3S l1 = p1->getVertex1();
		Point3S l2 = p1->getVertex2();
		Point3S c = p2->center;
		Eigen::Vector3d l1_p = l1.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
		Eigen::Vector3d l2_p = l2.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
		Eigen::Vector3d c_p = c.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);

		Eigen::Vector3d dir = (l1_p - l2_p).normalized();

		// compute the projection onto the line

		
		double len = (l1_p - l2_p).norm();
		double origAlpha = (c_p - l2_p).dot(dir); 
		if (DEBUG_SERVO) {
			VLOG(3) <<" the orig alpha is = " << origAlpha << std::endl;	
		}
		origAlpha = (origAlpha < 0)? 0 :origAlpha;
		origAlpha = (origAlpha > len)? len :origAlpha;

		composed->addParameter("ServorPos", origAlpha);
		LinearExpr alphaLin(composed->getSymbol(composed->numSymbols() -1));

		LinearExpr paramlen =  dir.x()*(l1.x - l2.x) + dir.y()*(l1.y - l2.y) + dir.z()*(l1.z - l2.z); 
		double paramLenEval = paramlen.eval(SymbolicAssignment::USE_CURRENT_VALUES);
		if(paramLenEval < 0){
			if (DEBUG_SERVO) {
				VLOG(3) << "the paramLenEval is smaller than zero = " << paramLenEval; system("pause");
			}
		}
		double servoDistance = 9.75; 
		double servoCentering = 5.65;
		LinearExpr ineqA = alphaLin - paramlen;
		LinearExpr ineqB = (-1.0)*alphaLin; 
		LinearExpr xEqual1 = l2.x + dir.x()*alphaLin - c.x + servoDistance*MainDir.x() - servoCentering;
		LinearExpr yEqual1 = l2.y + dir.y()*alphaLin - c.y + servoDistance*MainDir.y(); 
		LinearExpr zEqual1 = l2.z + dir.z()*alphaLin - c.z + servoDistance*MainDir.z();
		LinearExpr testEqueal = alphaLin - 0.5*paramlen;

		//VLOG(3) << " the test equal equation is " << std::endl;
		//testEqueal.print();
		//VLOG(3) << "the param leg info is " << std::endl;
		//paramlen.print(); system("pause");


		//3D constraints
		constraints3D.push_back(new Constraint(Constraint::ConstraintRelation::INEQ,  ineqA));
		constraints3D.push_back(new Constraint(Constraint::ConstraintRelation::INEQ,  ineqB));
		constraints3D.push_back(new Constraint(xEqual1));
		constraints3D.push_back(new Constraint(yEqual1));
		constraints3D.push_back(new Constraint(zEqual1));
		//constraints3D.push_back(new Constraint(testEqueal));

		// add the order constraints

		auto bbox = addTemplate->evalLocalBoundingBox();
		//todo the with could be x or z -> check!
		LinearExpr current_w = widthDirection.x()*(bbox.max.x - bbox.min.x) +
								widthDirection.y()*(bbox.max.y - bbox.min.y)+
								widthDirection.z()*(bbox.max.z - bbox.min.z);
		LinearExpr current_h = (bbox.max.y - bbox.min.y);

		if(addTemplate->getName().find("Wheel" ) != std::string::npos){
			current_h = 0.6*current_h;
		}

		//VLOG(3) <<" the current w is "<< std::endl;
		//current_w.print();
		//VLOG(3) <<" the current h is "<< std::endl;
		//current_h.print();

		for each ( auto s in p1->spacings){
			double oldAlpha = s.alpha.eval(SymbolicAssignment::USE_CURRENT_VALUES);
			double order = (origAlpha < oldAlpha)?  1 : -1;
			if (DEBUG_SERVO) {
				VLOG(3) <<" the orig alpha is = " << origAlpha << std::endl;	
				VLOG(3) <<" the old alpha is = " << oldAlpha << std::endl;	
				VLOG(3) <<" the order is = " << order << std::endl;	
			}
			LinearExpr ineq2 =  order*(alphaLin -s.alpha) + s.separation_h + current_h; 
			constraints3D.push_back(new Constraint(Constraint::ConstraintRelation::INEQ,  ineq2));
			for each ( auto w in s.separation_w){
				LinearExpr ineq1 =  order*(alphaLin -s.alpha) + w*0.5 + current_w*0.5; 
				constraints3D.push_back(new Constraint(Constraint::ConstraintRelation::INEQ,  ineq1));
			}

		}
		p1->spacings.push_back(ServoSpacing(alphaLin, current_w, current_h, addTemplate)); 

		//2DConstraints: none

		// create a new connection: here we should include some logic now it is just doing the default
		vector<NewPatch*> patches;
		patches.push_back(p1);
		patches.push_back(p2);
		NewConnection* newConnection;
		Point3S axis;
		axis.x = LinearExpr(p1->getNormal().x());
		axis.y = LinearExpr(p1->getNormal().y());
		axis.z = LinearExpr(p1->getNormal().z());

		double	multi= p1->getNormal().dot(Eigen::Vector3d::Ones());
		//SymbolicController * newController = new SymbolicController(_elementMotion);
		GrammarController::GrammarcontrollerType type = GrammarController::GrammarcontrollerType::LEG;
		if(addTemplate->getName().find("Wheel" ) != std::string::npos){
			type = GrammarController::GrammarcontrollerType::WHEEL;
		}
		std::vector<NewConnection*> addedConnections = addTemplate->getAllConnections();
		for each (auto c in addedConnections){
			if(dynamic_cast<JointConnection*>(c) != nullptr){
				type = GrammarController::GrammarcontrollerType::DOUBLE_SHOULDED;
			}
		}	
		GrammarController * newController = new GrammarController(type, 30,1, 1, multi);
		//LinearController * newController = new LinearController(3.14, 0);

		Articulation * articulation = new Articulation(p2->getSymbCenter());
		SymbolicTransformation* tranf = new SymbolicTransformation(newController, axis, false);
		articulation->transformations.push_back(tranf); 
		newConnection = new JointConnection(
			patches, 90, articulation, NewConnection::ConnectionType::NONE);
		
		
		return newConnection;

}



NewConnection* ConnectServoPoint(ServoPointPatch* p1, ServoPointPatch* p2, 
	Template* addTemplate, Template* composed,
	std::vector<Constraint*> & constraints3D,
	std::vector<Constraint*> & constraints2D,
	Element_Motion* _elementMotion){


		// step 1 rotate the servo correctly:
		Eigen::Vector3d MainDir = p1->getNormal();
		Eigen::Vector3d AddDir = (-1.0)*p2->getNormal();
		Eigen::Quaterniond qRotation = Eigen::Quaterniond::FromTwoVectors(AddDir, MainDir);
		addTemplate->rotate(addTemplate->getCenter(), qRotation);

	
		Point3S l = p1->center;
		Point3S c = p2->center;
		LinearExpr xEqual1 = l.x - c.x;
		LinearExpr yEqual1 = l.y - c.y;
		LinearExpr zEqual1 = l.z  - c.z;

		//3D constraints
		constraints3D.push_back(new Constraint(xEqual1));
		constraints3D.push_back(new Constraint(yEqual1));
		constraints3D.push_back(new Constraint(zEqual1));

		// add the order constraints
		constrainLegOverlap(p1, composed, addTemplate,constraints3D);

		//2DConstraints: none

		// create a new connection: here we should include some logic now it is just doing the default
		vector<NewPatch*> patches;
		patches.push_back(p1);
		patches.push_back(p2);
		NewConnection* newConnection;
		Point3S axis;
		axis.x = LinearExpr(p1->getNormal().x());
		axis.y = LinearExpr(p1->getNormal().y());
		axis.z = LinearExpr(p1->getNormal().z());
		double	multi= p1->getNormal().dot(Eigen::Vector3d::Ones());
		//SymbolicController * newController = new SymbolicController(_elementMotion);
		//LinearController * newController = new LinearController(3.14, 0);
		GrammarController::GrammarcontrollerType type = GrammarController::GrammarcontrollerType::DOUBLE_ELBOW;
		GrammarController * newController = new GrammarController(type, 30,1, 1, multi);


		Articulation * articulation = new Articulation(p2->getSymbCenter());
		SymbolicTransformation* tranf = new SymbolicTransformation(newController, axis, false);
		articulation->transformations.push_back(tranf); 
		newConnection = new JointConnection(
			patches, 90, articulation, NewConnection::ConnectionType::NONE);
		
		
		return newConnection;

}


void getThreeDContraintsForLinePatch(NewPatchLine2D3D *p1, NewPatchLine2D3D *p2, std::vector<Constraint*> & constraints3D){


			Point3S v1 = p1->getVertexS1(); 
			Point3S v2 = p1->getVertexS2(); 

			Point3S u1 = p2->getVertexS1(); 
			Point3S u2 = p2->getVertexS2();


			Eigen::Vector3d u1p = u1.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
			Eigen::Vector3d u2p = u2.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
			Eigen::Vector3d v1p = v1.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
			Eigen::Vector3d v2p = v2.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);

			double error1 = (u1p - v1p).norm() + (u2p - v2p).norm();   
			double error2 = (u1p - v2p).norm() + (u2p - v1p).norm();   
			if( error2 < error1){
				u2 = p2->getVertexS1(); 
				u1 = p2->getVertexS2();				
			}

			bool debug = false;
			if(debug){
				Geometry * match1 = new Geometry();
				Geometry * sphere_v1 = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));	
				sphere_v1->applyTrans(v1.evalpoint(SymbolicAssignment::USE_CURRENT_VALUES));
				match1->add(sphere_v1);
				Geometry * sphere_u1 = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));	
				sphere_u1->applyTrans(u1.evalpoint(SymbolicAssignment::USE_CURRENT_VALUES));
				match1->add(sphere_u1);
				match1->write("..\\..\\data\\match1.stl");
				Geometry * match2 = new Geometry();
				Geometry * sphere_v2 = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));	
				sphere_v2->applyTrans(v2.evalpoint(SymbolicAssignment::USE_CURRENT_VALUES));
				match2->add(sphere_v2);
				Geometry * sphere_u2 = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));	
				sphere_u2->applyTrans(u2.evalpoint(SymbolicAssignment::USE_CURRENT_VALUES));
				match2->add(sphere_u2);
				match2->write("..\\..\\data\\match2.stl");
				system("pause"); 
			}


			double eps = 0.1;
			LinearExpr xEqual1 = v1.x - u1.x;
			LinearExpr yEqual1 = v1.y - u1.y;
			LinearExpr zEqual1 = v1.z - u1.z;
			LinearExpr xEqual2 = v2.x - u2.x;
			LinearExpr yEqual2 = v2.y - u2.y;
			LinearExpr zEqual2 = v2.z - u2.z;



			//3D constraints
			//constraints3D.push_back(new Constraint(xEqual1));
			//constraints3D.push_back(new Constraint(yEqual1));
			//constraints3D.push_back(new Constraint(zEqual1));

			constraints3D.push_back(new Constraint(Constraint::ConstraintRelation::INEQ, xEqual2 -eps));
			constraints3D.push_back(new Constraint(Constraint::ConstraintRelation::INEQ, (-1.0)*xEqual2 -eps));

			constraints3D.push_back(new Constraint(Constraint::ConstraintRelation::INEQ, yEqual2 -eps));
			constraints3D.push_back(new Constraint(Constraint::ConstraintRelation::INEQ, (-1.0)*yEqual2 -eps));

			constraints3D.push_back(new Constraint(Constraint::ConstraintRelation::INEQ, zEqual2 -eps));
			constraints3D.push_back(new Constraint(Constraint::ConstraintRelation::INEQ, (-1.0)*zEqual2 -eps));
	

}

void getTwoDContraintsForLinePatch(NewPatchLine2D3D *p1, NewPatchLine2D3D *p2, std::vector<Constraint*> & constraints2D, Template* addTemplate){

	vector<Element*> elements = addTemplate->getAllElementList();

	if (OverlapPreventionHack::reflectAllDrawings) {
		// mirror the 2D drawing
		for each(Element* e in elements){
			Element_Symbolic* es = dynamic_cast<Element_Symbolic*>(e);
			Drawing2S* drawing = es->getDrawing2S();
			vector<Point2S>& points2d = drawing->vertices;
			for (int i = 0; i < points2d.size(); i++){
				Point2S p = points2d[i];
				p[0] = -p[0];
				points2d[i] = p;
			}

			es->computeNewGeo();

		}
	}

	//Rotate and translate the additional Template in 2D
		Eigen::Vector2d mainDir2d = p1->getDirection2D();
		Eigen::Vector2d addDir2d = p2->getDirection2D();
		Eigen::Matrix2d rMatrix2d = getRotationMatrixFromTwoVector2d(addDir2d, mainDir2d);
		VLOG(5) << "Will transform 2D by:";
		VLOG(5) << rMatrix2d;
		for each(Element* e in elements){
			Element_Symbolic* es = dynamic_cast<Element_Symbolic*>(e);
			Drawing2S* drawing = es->getDrawing2S();
			vector<Point2S>& points2d = drawing->vertices;
			for (int i = 0; i < points2d.size(); i++){
				Point2S p = points2d[i];
				p = p.times(rMatrix2d);
				points2d[i] = p;
			}

			es->computeNewGeo();

		}

		Eigen::Vector2d mainMid2d = p1->getDrawingVertex1();
		Eigen::Vector2d addMid2d = p2->getDrawingVertex1();
		VLOG(5) << mainMid2d << "after updating mainMid2d";
		VLOG(5) << addMid2d << "after updating addMid2d";
		Eigen::Vector2d translation = getTranslationMatrixFromTwoVector2d(addMid2d, mainMid2d);
		VLOG(5) << translation;
		for each(Element* e in elements){
			Element_Symbolic* es = dynamic_cast<Element_Symbolic*>(e);
			Drawing2S* drawing = es->getDrawing2S();
			vector<Point2S>& points2d = drawing->vertices;
			for (int i = 0; i < points2d.size(); i++){
				Point2S p = points2d[i];
				p = p.translate(translation);
				points2d[i] = p;
			}

			es->computeNewGeo();
		
		}

	

		double d1 = (p1->getDrawingVertexS1().evalVector2d(SymbolicAssignment::USE_CURRENT_VALUES)
					- p2->getDrawingVertexS1().evalVector2d(SymbolicAssignment::USE_CURRENT_VALUES)).norm()
					+ (p1->getDrawingVertexS2().evalVector2d(SymbolicAssignment::USE_CURRENT_VALUES)
					- p2->getDrawingVertexS2().evalVector2d(SymbolicAssignment::USE_CURRENT_VALUES)).norm();

		double d2 = (p1->getDrawingVertexS1().evalVector2d(SymbolicAssignment::USE_CURRENT_VALUES)
					- p2->getDrawingVertexS2().evalVector2d(SymbolicAssignment::USE_CURRENT_VALUES)).norm()
					+ (p1->getDrawingVertexS2().evalVector2d(SymbolicAssignment::USE_CURRENT_VALUES)
					- p2->getDrawingVertexS1().evalVector2d(SymbolicAssignment::USE_CURRENT_VALUES)).norm();


		LinearExpr xEqual1, yEqual1 ,xEqual2, yEqual2;

		if(d1 < d2){
			xEqual1 = p1->getDrawingVertexS1().x - p2->getDrawingVertexS1().x;
			yEqual1 = p1->getDrawingVertexS1().y - p2->getDrawingVertexS1().y;
			xEqual2 = p1->getDrawingVertexS2().x - p2->getDrawingVertexS2().x;
			yEqual2 = p1->getDrawingVertexS2().y - p2->getDrawingVertexS2().y;
		}else{
			xEqual1 = p1->getDrawingVertexS1().x - p2->getDrawingVertexS2().x;
			yEqual1 = p1->getDrawingVertexS1().y - p2->getDrawingVertexS2().y;
			xEqual2 = p1->getDrawingVertexS2().x - p2->getDrawingVertexS1().x;
			yEqual2 = p1->getDrawingVertexS2().y - p2->getDrawingVertexS1().y;
		}
		constraints2D.push_back(new Constraint(xEqual1));
		constraints2D.push_back(new Constraint(yEqual1));
		//constraints2D.push_back(new Constraint(xEqual2));
		//constraints2D.push_back(new Constraint(yEqual2));


}

void ConnectPeripheral(PeripheralPatch* p1, PeripheralPatch* p2, 
	Template* addTemplate, Template* composed,
	std::vector<Constraint*> & constraints3D,
	std::vector<Constraint*> & constraints2D,
	std::vector<NewConnection*> & connections	){
		if (DEBUG_PATCH) {
			std::cout << "is connecting peripherals" << std::endl;
		}

		p1->writeToFile("..\\..\\data\\connectingPer1.stl");
		p2->writeToFile("..\\..\\data\\connectingPer2.stl");

		// step 1 rotate the servo correctly:
		Eigen::Vector3d MainDir = p1->getNormal();
		Eigen::Vector3d AddDir = (-1.0)*p2->getNormal();
		Eigen::Quaterniond qRotation = Eigen::Quaterniond::FromTwoVectors(AddDir, MainDir);
		addTemplate->rotate(addTemplate->getCenter(), qRotation);

		p1->writeToFile("..\\..\\data\\connectingPer1_rotA.stl");
		p2->writeToFile("..\\..\\data\\connectingPer2_rotA.stl");



		Eigen::Vector3d dMain = p1->getLinePatch1()->getDirection();
		Eigen::Vector3d dAdd = p2->getLinePatch1()->getDirection();
		Eigen::Vector3d dAddOpp = (-1.0)*dAdd;
		Eigen::Vector3d dAddFinal = ((dMain - dAdd).norm() < (dMain + dAdd).norm()) ? dAdd : dAddOpp;
		Eigen::Quaterniond qRotation2 = Eigen::Quaterniond::FromTwoVectors(dAddFinal, dMain);
		addTemplate->rotate(addTemplate->getCenter(), qRotation2);


		p1->writeToFile("..\\..\\data\\connectingPer1_rotB.stl");
		p2->writeToFile("..\\..\\data\\connectingPer2_rotB.stl");

		//choose the pairs
		double error_match1 = p1->getLinePatch1()->getDistanceToPatch(p2->getLinePatch1())
				+p1->getLinePatch2()->getDistanceToPatch(p2->getLinePatch2());

		double error_match2 = p1->getLinePatch1()->getDistanceToPatch(p2->getLinePatch2())
				+p1->getLinePatch2()->getDistanceToPatch(p2->getLinePatch1());

		NewConnection* newConnection1;
		NewConnection* newConnection2;
		vector<NewPatch*> patchesPairA;
		vector<NewPatch*> patchesPairB;

		if(error_match1 < error_match2){
			 getThreeDContraintsForLinePatch(p1->getLinePatch1(), p2->getLinePatch1(), constraints3D);
			 getThreeDContraintsForLinePatch(p1->getLinePatch2(), p2->getLinePatch2(),constraints3D);
			 //getTwoDContraintsForLinePatch(p1->getLinePatch1(), p2->getLinePatch1(),constraints2D, addTemplate);
			 patchesPairA.push_back(p1->getLinePatch1());
			 patchesPairA.push_back(p2->getLinePatch1());
			 patchesPairB.push_back(p1->getLinePatch2());
			 patchesPairB.push_back(p2->getLinePatch2());
		}else{
			 getThreeDContraintsForLinePatch(p1->getLinePatch1(), p2->getLinePatch2(), constraints3D);
			 getThreeDContraintsForLinePatch(p1->getLinePatch2(), p2->getLinePatch1(),constraints3D);
			 //getTwoDContraintsForLinePatch(p1->getLinePatch1(), p2->getLinePatch2(),constraints2D, addTemplate);
			 patchesPairA.push_back(p1->getLinePatch1());
			 patchesPairA.push_back(p2->getLinePatch2());
			 patchesPairB.push_back(p1->getLinePatch2());
			 patchesPairB.push_back(p2->getLinePatch1());
		}


		// add the order constraints

		// orient in 2d and create constrains
		//2DConstraints: none

		// create a new connection: here we should include some logic now it is just doing the default
		vector<NewPatch*> patches;
		patches.push_back(p1);
		patches.push_back(p2);
//		NewConnection* newConnection;
//		newConnection = new NewConnection(patches, 90, NewConnection::ConnectionType::FOLD);

		NewConnection*newConnectionA = new NewConnection(patchesPairA, 90, NewConnection::ConnectionType::FOLD);
		NewConnection*newConnectionB = new NewConnection(patchesPairB, 90, NewConnection::ConnectionType::FOLD);

		connections.push_back(newConnectionA);
		connections.push_back(newConnectionB);

}



bool findClosestPatchWithGrammar(Template* main, Template* add, std::vector<std::pair<NewPatch*, NewPatch*>>  &patchPairs){


	//filter all the patches
	std::vector<ServoPointPatch*> add_servoPatches; 
	std::vector<PeripheralPatch*> add_peripheralPatches; 
	std::vector<NewPatch*> main_servoPatches; 
	std::vector<PeripheralPatch*> main_peripheralPatches; 
	for each ( auto p in add->getAllUnusedPatches()){
		if(dynamic_cast<ServoPointPatch*>(p) != nullptr){
			add_servoPatches.push_back(dynamic_cast<ServoPointPatch*>(p));
		}
		if(dynamic_cast<PeripheralPatch*>(p) != nullptr){
			add_peripheralPatches.push_back(dynamic_cast<PeripheralPatch*>(p));
		}
	}
	for each ( auto p in main->getAllUnusedPatches()){
		if(dynamic_cast<ServoLinePatch*>(p) != nullptr){
			main_servoPatches.push_back(p);
		}
		if(dynamic_cast<ServoPointPatch*>(p) != nullptr){
			main_servoPatches.push_back(p);
		}
		if(dynamic_cast<PeripheralPatch*>(p) != nullptr){
			main_peripheralPatches.push_back(dynamic_cast<PeripheralPatch*>(p));
		}
	}

	// first try to find matching servo Patches
	if(( add_servoPatches.size() > 0) && (main_servoPatches.size() > 0)){
		std::pair<NewPatch*, NewPatch*> pair;
		double distance = std::numeric_limits<double>::max();
		double orientationDist = std::numeric_limits<double>::max();
		for each (auto pAdd in add_servoPatches){	
			for each (auto pMain in main_servoPatches){
				double new_distance = pMain->getDistanceToPoint(pAdd->center.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES));
				double new_orientationDist= 0; //(pMain->getNormal().normalized()  + pAdd->getNormal().normalized()).norm();
				bool replace = false;
				if(new_orientationDist < (orientationDist - 0.01)){
					replace  = true;
				}else{
					if(abs(new_orientationDist - orientationDist) < 0.1){
						if(new_distance < distance){
							replace = true;
						}
					}
				}
				if(replace){
					orientationDist = new_orientationDist;
					distance = new_distance;
					pair.first= pMain;
					pair.second = pAdd;
				}
			}
		}
		patchPairs.push_back(pair);
		
		
		return true; 
	}


	// then try to find matching peripherals
	if(( add_peripheralPatches.size() > 0) && (main_peripheralPatches.size() > 0)){
		std::pair<NewPatch*, NewPatch*> pair;
		double distance = std::numeric_limits<double>::max();
		double orientationDist = std::numeric_limits<double>::max();
		for each (auto pAdd in add_peripheralPatches){	
			for each (auto pMain in main_peripheralPatches){
				double new_distance = pMain->getDistanceToPatch(pAdd);
				double new_orientationDist= 0; //pMain->getSideOrientationDistanceToPatch(pAdd) + (pMain->getNormal().normalized()  + pAdd->getNormal().normalized()).norm();
				
				if (DEBUG_PATCH) {
					//std::cout << "new_dista " << new_distance << std::endl;
					//std::cout << "new orien " << new_orientationDist << std::endl;
				}
				pMain->writeToFile("..\\..\\data\\connectingPer1.stl");
				pAdd->writeToFile("..\\..\\data\\connectingPer2.stl");
				
				

				bool replace = false;
				if(new_orientationDist < orientationDist){
					if (DEBUG_PATCH) {
						std::cout << "might have found an example of orientation 0! " << new_orientationDist  << std::endl;
					}
					replace  = true;
				}else{
					if(abs(new_orientationDist - orientationDist) < 0.01){
						if (DEBUG_PATCH) {
							//std::cout << "foune an example of orientation 0! " << new_orientationDist  << std::endl;
						}
						if(new_distance < distance){
							replace = true;
						}
					}
				}
				if(replace){
					orientationDist = new_orientationDist;
					distance = new_distance;
					pair.first= pMain;
					pair.second = pAdd;
					if (DEBUG_PATCH) {
						std::cout << "replaced!" << std::endl;
					}

				}

			}
		}
		patchPairs.push_back(pair);
		
		
		return true; 
	}

	return false;

}
std::vector<NewPatch *> TemplateManipulations::findClossestElement(Template * main, Template* add){
	std::vector<std::pair<NewPatch*, NewPatch*>>  patchPairs;
	std::vector<NewPatch*> result;
	if(findClosestPatchWithGrammar(main, add, patchPairs)){
		result.push_back(patchPairs[0].first);
		result.push_back(patchPairs[0].second);

		// step 1 rotate the servo correctly:
		//Eigen::Vector3d MainDir = patchPairs[0].first->getNormal();
		//Eigen::Vector3d AddDir = (-1.0)*patchPairs[0].second->getNormal();
		//Eigen::Quaterniond qRotation = Eigen::Quaterniond::FromTwoVectors(AddDir, MainDir);
		//add->rotate(add->getCenter(), qRotation);


	}
	return result;
}


std::vector<Template *> TemplateManipulations::SnappingWithGrammar(Template* main, Template* add){
	//find the first unique id available
	unordered_map<Template*, int> ids1 = findAllTemplateIDs(main);
	unordered_map<Template*, int> ids2 = findAllTemplateIDs(add);
	int maxId1 = 0;
	for (auto it = ids1.begin(); it != ids1.end(); it++){
		maxId1 = max(it->second, maxId1);
	}
	for (auto it = ids2.begin(); it != ids2.end(); it++){
		it->first->setId(++maxId1);
	}



	TemplateGroupBuilder* tb = new TemplateGroupBuilder(++maxId1);
	tb->addChild(main);
	tb->addChild(add);

	auto mainOrigConfig = main->getFullQ(); 
	//find missing patches of add tempalte and for each of them find the closest thing on the main to add
	Template* result = tb->build();
	std::vector<Constraint*> threeDConstraints;
	std::vector<Constraint*> twoDConstraints;
	std::vector<std::pair<NewPatch*, NewPatch*>>  patchPairs;

	std::vector<Template*> connectPair;
	if(findClosestPatchWithGrammar(main, add, patchPairs)){
		connectPair.push_back(patchPairs[0].first->getElement());
		connectPair.push_back(patchPairs[0].second->getElement());
		auto pair = patchPairs[0];

			if(DEBUG_WRITEPATCHTOFILE){
				Geometry * geo = add->getGeometry();
				geo->add(main->getGeometry());
				geo->add(pair.first->getDebugGeo());
				geo->add(pair.second->getDebugGeo());
				geo->write("..\\..\\data\\debug_foundPatchPair.stl");
			}


			// only for the patches that need to be connected
			if(dynamic_cast<ServoPointPatch*>(pair.second) != nullptr){
					TemplateProtoConverter converter;
					//auto protoRead = converter.loadFromFile("..\\..\\data\\\proto2016\\GenericMotion.asciiproto");
					//Template* motionTemp = converter.ConvertToTemplate(*protoRead);
					//TemplateElement * e = dynamic_cast<TemplateElement*>(motionTemp);
					//Element_Motion* em= dynamic_cast<Element_Motion*>(e->getElement());
					//result->addChild(motionTemp);
					//Element_Motion* el_motion= dynamic_cast<Element_Motion*>((dynamic_cast<TemplateElement*>(motionTemp))->getElement());
					Element_Motion* el_motion = nullptr;
					NewConnection * newConnection;
					
					if(dynamic_cast<ServoPointPatch*>(pair.first) != nullptr){
						newConnection = ConnectServoPoint(
						dynamic_cast<ServoPointPatch*>(pair.first),
						dynamic_cast<ServoPointPatch*>(pair.second),
						add, result, threeDConstraints, twoDConstraints, el_motion);
					}else{
					newConnection = ConnectServo(
						dynamic_cast<ServoLinePatch*>(pair.first),
						dynamic_cast<ServoPointPatch*>(pair.second),
						add, result, threeDConstraints, twoDConstraints, el_motion);
					}
					result->addConnection(newConnection);
			}
			if(dynamic_cast<PeripheralPatch*>(pair.second) != nullptr){
					std::vector<NewConnection*> connections;
						ConnectPeripheral(
						dynamic_cast<PeripheralPatch*>(pair.first),
						dynamic_cast<PeripheralPatch*>(pair.second),
						add, result, threeDConstraints, twoDConstraints, connections);
						for each (auto c in connections)
							result->addConnection(c);
			}

		}

	
		//replace current MainTemplate
		//result->getGeometry()->write("..\\..\\data\\combinedExample_before.stl"); 
		//main->updateFullQ(mainOrigConfig);

		ConstraintsEval::updateTemplateForSnapping(add, main, result, threeDConstraints);

		main->parent = nullptr;
		add->parent = nullptr;
		result->clear();

		return  connectPair;
	
}



Template* TemplateManipulations::ConnectWithGrammar(Template* main, Template* add) {
	//find the first unique id available
	unordered_map<Template*, int> ids1 = findAllTemplateIDs(main);
	unordered_map<Template*, int> ids2 = findAllTemplateIDs(add);
	int maxId1 = 0;
	for (auto it = ids1.begin(); it != ids1.end(); it++){
		maxId1 = max(it->second, maxId1);
	}
	for (auto it = ids2.begin(); it != ids2.end(); it++){
		it->first->setId(++maxId1);
	}



	TemplateGroupBuilder* tb = new TemplateGroupBuilder(++maxId1);
	tb->addChild(main);
	tb->addChild(add);

	//find missing patches of add tempalte and for each of them find the closest thing on the main to add
	Template* result = tb->build();
	std::vector<Constraint*> threeDConstraints;
	std::vector<Constraint*> twoDConstraints;
	std::vector<std::pair<NewPatch*, NewPatch*>>  patchPairs;

	  if(findClosestPatchWithGrammar(main, add, patchPairs)){
		for each (auto pair in patchPairs){

			if(DEBUG_WRITEPATCHTOFILE){
				Geometry * geo = add->getGeometry();
				geo->add(main->getGeometry());
				geo->add(pair.first->getDebugGeo());
				geo->add(pair.second->getDebugGeo());
				geo->write("..\\..\\data\\debug_foundPatchPair.stl");
			}


			// only for the patches that need to be connected
			if(dynamic_cast<ServoPointPatch*>(pair.second) != nullptr){
					TemplateProtoConverter converter;
					//auto protoRead = converter.loadFromFile("..\\..\\data\\\proto2016\\GenericMotion.asciiproto");
					//Template* motionTemp = converter.ConvertToTemplate(*protoRead);
					//TemplateElement * e = dynamic_cast<TemplateElement*>(motionTemp);
					//Element_Motion* em= dynamic_cast<Element_Motion*>(e->getElement());
					//result->addChild(motionTemp);
					//Element_Motion* el_motion= dynamic_cast<Element_Motion*>((dynamic_cast<TemplateElement*>(motionTemp))->getElement());
					Element_Motion* el_motion = nullptr;
					NewConnection * newConnection;
					
					if(dynamic_cast<ServoPointPatch*>(pair.first) != nullptr){
						newConnection = ConnectServoPoint(
						dynamic_cast<ServoPointPatch*>(pair.first),
						dynamic_cast<ServoPointPatch*>(pair.second),
						add, result, threeDConstraints, twoDConstraints, el_motion);
					}else{
					newConnection = ConnectServo(
						dynamic_cast<ServoLinePatch*>(pair.first),
						dynamic_cast<ServoPointPatch*>(pair.second),
						add, result, threeDConstraints, twoDConstraints, el_motion);
					}
					result->addConnection(newConnection);
			}
			if(dynamic_cast<PeripheralPatch*>(pair.second) != nullptr){
					std::vector<NewConnection*> connections;
						ConnectPeripheral(
						dynamic_cast<PeripheralPatch*>(pair.first),
						dynamic_cast<PeripheralPatch*>(pair.second),
						add, result, threeDConstraints, twoDConstraints, connections);
						for each (auto c in connections)
							result->addConnection(c);
			}

		}
		for each( auto c in twoDConstraints){
			result->addConstraint(c);
		}
		for each( auto c in threeDConstraints){
			result->addConstraint(c);
		}
	
		//replace current MainTemplate
		//result->getGeometry()->write("..\\..\\data\\combinedExample_before.stl"); 
		ConstraintsEval::enforceConstraintsConnect(result);

		if( ENABLE_COLLISION_DETECTION && OverlapPreventionHack::countOverlapness != nullptr){
				int originalOverlapness = OverlapPreventionHack::countOverlapness(main->getAllDrawing());
				int newOverlapness = OverlapPreventionHack::countOverlapness(result->getAllDrawing());
				if (newOverlapness > originalOverlapness) {
					for each (auto c in twoDConstraints) {
						result->removeConstraint(c);
					}
					for (int i = 0; i < 20; i++) {
						add->translate2d(Eigen::Vector2d(100, 0));
						if (OverlapPreventionHack::countOverlapness(result->getAllDrawing()) <= originalOverlapness) {
							break;
						}
					}
					if (DEBUG_OVERLAP_CONSTRAINT) {
						VLOG(3) << "Collision occured; had to remove additional template away";
						cout << "Collision occured; had to remove additional template away";
					}
				}
		}
	  }
	return result;
}


Template* TemplateManipulations::Connect(Template* main, Template* add, const ConnectingInfo& connectingInfo) {
	//find the first unique id available
	unordered_map<Template*, int> ids1 = findAllTemplateIDs(main);
	unordered_map<Template*, int> ids2 = findAllTemplateIDs(add);
	int maxId1 = 0;
	for (auto it = ids1.begin(); it != ids1.end(); it++){
		maxId1 = max(it->second, maxId1);
	}
	for (auto it = ids2.begin(); it != ids2.end(); it++){
		it->first->setId(++maxId1);
	}



	TemplateGroupBuilder* tb = new TemplateGroupBuilder(++maxId1);
	tb->addChild(main);
	tb->addChild(add);
	vector<Constraint*> twoDConstraints;
	vector<PatchPair> independentPairs = ElectNonConnectingPatchPairs(connectingInfo.matchingNewPatches, add);
	LOG(INFO) << "number of patchPairs originally is : " << connectingInfo.matchingNewPatches.size();
	LOG(INFO) << "number of patchPairs elected as independent is : " << independentPairs.size();

	for (int i = 0; i < connectingInfo.matchingNewPatches.size(); i++) {
		NewPatch* p1 = connectingInfo.matchingNewPatches[i].wt_patch;
		NewPatch* p2 = connectingInfo.matchingNewPatches[i].add_patch;
		bool isOpposite = connectingInfo.matchingNewPatches[i].isOpposite;
		//bool isOpposite2D = connectingInfo.matchingNewPatches[i].isOpposite2D;

		NewPatchLine2D3D* p1l = static_cast<NewPatchLine2D3D*>(p1);
		NewPatchLine2D3D* p2l = static_cast<NewPatchLine2D3D*>(p2);
		NewPatchLine2D3D* p2lX = isOpposite ? p2l->flip() : p2l;

		LinearExpr xEqual1 = p1l->getVertexS1().x - p2lX->getVertexS1().x;
		LinearExpr yEqual1 = p1l->getVertexS1().y - p2lX->getVertexS1().y;
		LinearExpr zEqual1 = p1l->getVertexS1().z - p2lX->getVertexS1().z;
		LinearExpr xEqual2 = p1l->getVertexS2().x - p2lX->getVertexS2().x;
		LinearExpr yEqual2 = p1l->getVertexS2().y - p2lX->getVertexS2().y;
		LinearExpr zEqual2 = p1l->getVertexS2().z - p2lX->getVertexS2().z;
		VLOG(5) << "p1v1x = " << p1l->getVertexS1().x.toString();
		VLOG(5) << "p1v1y = " << p1l->getVertexS1().y.toString();
		VLOG(5) << "p1v1z = " << p1l->getVertexS1().z.toString();
		VLOG(5) << "p1v2x = " << p1l->getVertexS2().x.toString();
		VLOG(5) << "p1v2y = " << p1l->getVertexS2().y.toString();
		VLOG(5) << "p1v2z = " << p1l->getVertexS2().z.toString();
		VLOG(5) << "p2v1x = " << p2l->getVertexS1().x.toString();
		VLOG(5) << "p2v1y = " << p2l->getVertexS1().y.toString();
		VLOG(5) << "p2v1z = " << p2l->getVertexS1().z.toString();
		VLOG(5) << "p2v1x = " << p2l->getVertexS2().x.toString();
		VLOG(5) << "p2v1y = " << p2l->getVertexS2().y.toString();
		VLOG(5) << "p2v1z = " << p2l->getVertexS2().z.toString();
		VLOG(5) << "xEqual1 = " << xEqual1.eval(SymbolicAssignment::USE_CURRENT_VALUES);
		VLOG(5) << "yEqual1 = " << yEqual1.eval(SymbolicAssignment::USE_CURRENT_VALUES);
		VLOG(5) << "zEqual1 = " << zEqual1.eval(SymbolicAssignment::USE_CURRENT_VALUES);
		VLOG(5) << "xEqual2 = " << xEqual2.eval(SymbolicAssignment::USE_CURRENT_VALUES);
		VLOG(5) << "yEqual2 = " << yEqual2.eval(SymbolicAssignment::USE_CURRENT_VALUES);
		VLOG(5) << "zEqual2 = " << zEqual2.eval(SymbolicAssignment::USE_CURRENT_VALUES);

		LinearExpr xEqual11 = p1l->getDrawingVertexS1().x - p2lX->getDrawingVertexS1().x;
		LinearExpr yEqual11 = p1l->getDrawingVertexS1().y - p2lX->getDrawingVertexS1().y;
		LinearExpr xEqual22 = p1l->getDrawingVertexS2().x - p2lX->getDrawingVertexS2().x;
		LinearExpr yEqual22 = p1l->getDrawingVertexS2().y - p2lX->getDrawingVertexS2().y;

		//3D constraints
		bool adding3D = true;
		if (adding3D){
			tb->addConstraint(new Constraint(xEqual1));
			tb->addConstraint(new Constraint(yEqual1));
			tb->addConstraint(new Constraint(zEqual1));
			tb->addConstraint(new Constraint(xEqual2));
			tb->addConstraint(new Constraint(yEqual2));
			tb->addConstraint(new Constraint(zEqual2));
		}
		

		auto twoDcX = new Constraint(xEqual11),
			twoDcY = new Constraint(yEqual11);

		if (find(independentPairs.begin(), independentPairs.end(),
			connectingInfo.matchingNewPatches[i]) != independentPairs.end()) {
			twoDConstraints.push_back(twoDcX);
			twoDConstraints.push_back(twoDcY);
			for each (auto constraint in twoDConstraints) {
				tb->addConstraint(constraint);
			}
		}

		// Delete existing half-open connections and add new connection
		auto deleteConnection = [](Template* t, NewConnection* conn) {
			for each (auto temp in t->getTemplatesByScope(TreeScope::DESCENDANTS)) {
				auto connections = temp->getConnections();
				auto it = find(connections.begin(), connections.end(), conn);
				if (it != connections.end()) {
					temp->removeConnection(*it);
					for each (auto patch in conn->getPatches()) {
						if (dynamic_cast<PlaceHolderPatch*>(patch) == nullptr) {
							patch->parentConnection = nullptr;
						}
					}
				}
			}
		};

		NewConnection* connection = p1->parentConnection == nullptr ? p2->parentConnection : p1->parentConnection;
		vector<NewPatch*> patches;
		patches.push_back(p1);
		patches.push_back(p2);

		NewConnection* newConnection;
		if (dynamic_cast<JointConnection*>(connection) != nullptr){
			//this is a joint connection
			Articulation* p1Articulation = p1->parentConnection == nullptr ? nullptr : p1->parentConnection->getArticulation();
			Articulation* p2Articulation = p2->parentConnection == nullptr ? nullptr : p2->parentConnection->getArticulation();
			Articulation* articulation = p1Articulation == nullptr ? p2Articulation : p1Articulation;
			if (articulation == nullptr){
				LOG(ERROR) << "error: " << "articulation is null";
				//throw "articulation is nullptr!";
				newConnection = new NewConnection(
					patches,
					connection != nullptr ? connection->getAngle() : 90,
					connection != nullptr ? connection->getConnectionType() : NewConnection::ConnectionType::FOLD);//added fold as default connection if none exists
			}
			else{
				articulation->print();
				newConnection = new JointConnection(
					patches,
					connection != nullptr ? connection->getAngle() : 90,
					articulation,
					connection != nullptr ? connection->getConnectionType() : NewConnection::ConnectionType::BALLJOINT); //added balljoint as default jointconnection if none exists
			}
		}
		else{
			newConnection = new NewConnection(
				patches,
				connection != nullptr ? connection->getAngle() : 90,
				connection != nullptr ? connection->getConnectionType() : NewConnection::ConnectionType::FOLD);//added fold as default connection if none exists
		}

		if (p1->parentConnection != nullptr) {
			auto p1c = p1->parentConnection;
			deleteConnection(main, p1c);
		}
		if (p2->parentConnection != nullptr) {
			auto p2c = p2->parentConnection;
			deleteConnection(add, p2c);
		}

		tb->addConnection(newConnection);
	}
	//replace current MainTemplate
	Template* result = tb->build();
	ConstraintsEval::enforceConstraints(result);

#if ENABLE_COLLISION_DETECTION
		int originalOverlapness = OverlapPreventionHack::countOverlapness(main->getAllDrawing());
		int newOverlapness = OverlapPreventionHack::countOverlapness(result->getAllDrawing());
		if (newOverlapness > originalOverlapness) {
			for each (auto c in twoDConstraints) {
				result->removeConstraint(c);
			}
			for (int i = 0; i < 20; i++) {
				add->translate2d(Eigen::Vector2d(10, 0));
				if (OverlapPreventionHack::countOverlapness(result->getAllDrawing()) <= originalOverlapness) {
					break;
				}
			}
			VLOG(3) << "Collision occured; had to remove additional template away";
			cout << "Collision occured; had to remove additional template away";
		}
#endif

	return result;
}

unordered_map<TemplateElement*, int> FindConnectedComponents(
	unordered_map<TemplateElement*, vector<TemplateElement*>> const& graph) {
	
	unordered_set<TemplateElement*> notCovered;
	vector<TemplateElement*> keys;
	unordered_map<TemplateElement*, int> result;

	for each (auto pair in graph) {
		keys.push_back(pair.first);
		notCovered.insert(pair.first);
	}
	
	int groupNum = 0;
	while (!notCovered.empty()) {
		//do a BFS
		queue<TemplateElement*> q;
		unordered_set<TemplateElement*> visited;
		q.push(*notCovered.begin());
		while (!q.empty()) {
			TemplateElement* current = q.front();
			q.pop();
			visited.insert(current);
			if (notCovered.find(current) != notCovered.end()) {
				notCovered.erase(notCovered.find(current));
			}
			result[current] = groupNum;
			for each (TemplateElement* neighbor in graph.at(current)) {
				if (visited.find(neighbor) == visited.end()){
					q.push(neighbor);
				}
			}
		}
		groupNum++;
	}

	return result;
}

vector<PatchPair> TemplateManipulations::ElectNonConnectingPatchPairs(
	vector<PatchPair> const& pairs, Template* addTemplate) {

	// Build a graph of template elements connected through connections
	unordered_map<TemplateElement*, vector<TemplateElement*>> connectivity;

	vector<TemplateElement*> leafs = addTemplate->getAllElementTempList();
	for each (TemplateElement* leaf in leafs) {
		connectivity[leaf] = vector<TemplateElement*>();
	}

	vector<NewConnection*> connections = addTemplate->getConnections(TreeScope::DESCENDANTS);
	for each (auto conn in connections) {
		vector<TemplateElement*> connectedTemplates = conn->getTemplates();
		bool hasPlaceholder = false;
		for each (auto ele in connectedTemplates) {
			if (ele == nullptr) {
				hasPlaceholder = true;
			}
		}
		if (!hasPlaceholder) {
			connectivity[connectedTemplates[0]].push_back(connectedTemplates[1]);
			connectivity[connectedTemplates[1]].push_back(connectedTemplates[0]);
		}
	}

	auto componentMap = FindConnectedComponents(connectivity);

	unordered_set<int> usedComponents;
	vector<PatchPair> result;
	for each (PatchPair pp in pairs) {
		TemplateElement* ele = dynamic_cast<TemplateElement*>(pp.add_patch->getElement());
		if(ele == nullptr){
			LOG(ERROR) << "error: the template is not a template element"  << std::endl;
		}
		int component = componentMap.at(ele);
		if (usedComponents.find(component) == usedComponents.end()) {
			result.push_back(pp);
			usedComponents.insert(component);
		}
	}
	return result;
}