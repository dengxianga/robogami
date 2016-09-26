#include "constraintsEval.h"
#include "template.h"
#include "element.h"
#include "templateElement.h"
#include "myOptimizer.h"
#include <Eigen/QR>
#include "constraints.h"
#include "NewPatch.h"
#include "FriendlyLinearSystem.h"
#include <queue>
#include "element_symbolic.h"
#include "debugging.h"
#include "controller.h"
#include "element_motion.h"
#include "kinChain.h"
using namespace FabByExample;
#define USE_FRIENDLY_LINEAR_SYSTEM

Eigen::Vector3d projectPointToLine(const Eigen::Vector3d& p, const Eigen::Vector3d& l1, const Eigen::Vector3d& l2) {
	auto b = p - l1;
	auto a = l2 - l1;
	auto projBonA = a.dot(b) * a / a.squaredNorm();
	return l1 + projBonA;
}

Eigen::Vector2d projectPointToLine(const Eigen::Vector2d& p, const Eigen::Vector2d& l1, const Eigen::Vector2d& l2) {
	auto b = p - l1;
	auto a = l2 - l1;
	auto projBonA = a.dot(b) * a / a.squaredNorm();
	return l1 + projBonA;
}

// Assume that b>a, d>c, and d-c <= b-a, return the amount to move interval [c, d] by so that [c, d] is contained by [a, b], and that the amount we move is smallest possible.
double moveInterval(double a, double b, double c, double d) {
	if (c < a) return a - c;
	if (d > b) return b - d;
	return 0;
}

// p = (1-ratio)* v1 + ratio * v2
Point3S interpolate(Point3S v1, Point3S v2, double ratio){	
	VLOG(5) << "p = (1-ratio)* v1 + ratio * v2";
	return v1*(1 - ratio) + v2*ratio;
}

Point2S interpolate(Point2S v1, Point2S v2, double ratio){
	VLOG(5) << "p = (1-ratio)* v1 + ratio * v2";
	return v1*(1 - ratio) + v2*ratio;
}







double ConstraintsEval::updateTemplateForSnappingWithEdges(Template* mainTemplate, Template* addTemplate, NewPatchLine2D3D* mainEdge, 
	NewPatchLine2D3D* addEdge, bool isOpposite, bool isOpposite2D, double maxDistance,ConnectingInfo& connInfo){
	// A temporary patch that is the opposite of the current patch if needed
	NewPatchLine2D3D* addEdgeX = isOpposite ? addEdge->flip() : addEdge;
	bool optimizationNeeded;
	bool snapV1, snapV2, snapV3, splitMain, splitAdditional;
	Point3S finalAddV1S, finalAddV2S;
	Eigen::Vector3d finalMainV1, finalMainV2;
	{
		// Rotate additional template
		Eigen::Vector3d MainDir = mainEdge->getDirection();
		Eigen::Vector3d AddDir = addEdgeX->getDirection();
		Eigen::Matrix3d rotationMatrix = RotationMatrixFromTwoVectors(AddDir, MainDir);
		Eigen::Vector2d mainMid2d0 = mainEdge->getDrawingVertex1();
		Eigen::Vector2d addMid2d0 = addEdgeX->getDrawingVertex1();
		VLOG(5) << mainMid2d0 << "before updating mainMid2d0";
		VLOG(5) << addMid2d0 << "before updating addMid2d0";

		vector<Element*> elements = addTemplate->getAllElementList();
		for each(Element* e in elements){
			Element_Symbolic* es = dynamic_cast<Element_Symbolic*>(e);
			Mesh3S* mesh = es->getMesh3S();
			vector<Point3S>& points = mesh->vertices;
			for (int i = 0; i < points.size(); i++){
				points[i] = points[i].times(rotationMatrix);
			}
			es->computeNewGeo();
			
		}


		if (isOpposite == isOpposite2D) {
			
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
		Eigen::Vector2d mainDir2d = mainEdge->getDirection2D();
		Eigen::Vector2d addDir2d = addEdgeX->getDirection2D();
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

		Eigen::Vector2d mainMid2d = mainEdge->getDrawingVertex1();
		Eigen::Vector2d addMid2d = addEdgeX->getDrawingVertex1();
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

		Eigen::Vector3d mainV1 = mainEdge->getVertex1();
		Eigen::Vector3d mainV2 = mainEdge->getVertex2();
		Eigen::Vector3d addV1 = addEdgeX->getVertex1();
		Eigen::Vector3d addV2 = addEdgeX->getVertex2();
		Eigen::Vector3d projV1 = projectPointToLine(addV1, mainV1, mainV2);
		Eigen::Vector3d projV2 = projectPointToLine(addV2, mainV1, mainV2);

		Eigen::Vector3d realDir = (mainV2 - mainV1).normalized();
		double mainV1N = mainV1.dot(realDir) - mainV1.dot(realDir);
		double mainV2N = mainV2.dot(realDir) - mainV1.dot(realDir);
		double projV1N = projV1.dot(realDir) - mainV1.dot(realDir);
		double projV2N = projV2.dot(realDir) - mainV1.dot(realDir);
		double moveBy;

		if (projV2N - projV1N <= mainV2N - mainV1N) {
			moveBy = moveInterval(mainV1N, mainV2N, projV1N, projV2N);
			splitMain = true;
		}
		else {
			moveBy = -moveInterval(projV1N, projV2N, mainV1N, mainV2N);
			splitAdditional = true;
		}
		projV1 = (projV1N + moveBy) * realDir + mainV1;
		projV2 = (projV2N + moveBy) * realDir + mainV1;
		finalAddV1S = addEdgeX->getVertexS1();
		finalAddV2S = addEdgeX->getVertexS2();
		Eigen::Vector3d translateBy = projV1 - addV1;
		
		double distV1 = (projV1 - mainV1).norm();
		double distV2 = (projV2 - mainV2).norm();
		double distV3 = addTemplate->getLowestZ();
		snapV1 = distV1 <= maxDistance;
		snapV2 = distV2 <= maxDistance;
		snapV3 = abs(distV3) <= 0.1*maxDistance;

		finalMainV1 = snapV1 ? mainV1 : projV1;
		finalMainV2 = snapV2 ? mainV2 : projV2;
		optimizationNeeded = snapV1 && snapV2 || snapV3;

		if (!snapV1 || !snapV2) {
			for each(Element* e in elements) {
				Element_Symbolic* es = dynamic_cast<Element_Symbolic*>(e);
				Mesh3S* mesh = es->getMesh3S();
				vector<Point3S>& points = mesh->vertices;
				for (int i = 0; i < points.size(); i++) {
					points[i] = points[i].translate(translateBy);
				}
				es->computeNewGeo();

			}
			Point3S mainP1 = mainEdge->getVertexS1();
			Point3S mainP2 = mainEdge->getVertexS2();
			Point3S addP1 = addEdgeX->getVertexS1();
			Point3S addP2 = addEdgeX->getVertexS2();

			Point2S main2dP1 = mainEdge->getDrawingVertexS1();
			Point2S main2dP2 = mainEdge->getDrawingVertexS2();
			Point2S add2dP1 = addEdgeX->getDrawingVertexS1();
			Point2S add2dP2 = addEdgeX->getDrawingVertexS2();

			Eigen::Vector3d mainV1 = mainEdge->getVertex1();
			Eigen::Vector3d mainV2 = mainEdge->getVertex2();
			Eigen::Vector3d addV1 = addEdgeX->getVertex1();
			Eigen::Vector3d addV2 = addEdgeX->getVertex2();
			Eigen::Vector3d projV1 = projectPointToLine(addV1, mainV1, mainV2);
			Eigen::Vector3d projV2 = projectPointToLine(addV2, mainV1, mainV2);
			Eigen::Vector3d projmV1= projectPointToLine(mainV1, addV1, addV2);
			Eigen::Vector3d projmV2 = projectPointToLine(mainV2, addV1, addV2);
			Point3S pp1, pp2;
			Point2S pd1, pd2;
			
			// get the two points and either add two vertices to the additionalTemplate or the mainTemplate
			if (splitMain){
				double ratio1 = (projV1 - mainV1).norm() / (mainV2 - mainV1).norm();
				double ratio2 = (projV2 - mainV1).norm() / (mainV2 - mainV1).norm();
				pp1 = interpolate(mainP1, mainP2, ratio1);
				pp2 = interpolate(mainP1, mainP2, ratio2);
				pd1 = interpolate(main2dP1, main2dP2, ratio1);
				pd2 = interpolate(main2dP1, main2dP2, ratio2);

				//add 3d points to the mesh containing this patch
				Element_Symbolic* mainE = mainEdge->getElement();
				int endIndex = mainE->getMesh3S()->vertices.size();
				mainE->getMesh3S()->vertices.push_back(pp1);
				mainE->getMesh3S()->vertices.push_back(pp2);
		
				//add 2D points to drawing
				int endIndex1 = mainE->getDrawing2S()->vertices.size();
				mainE->getDrawing2S()->vertices.push_back(pd1);
				mainE->getDrawing2S()->vertices.push_back(pd2);
				Edge2S edge;
				edge.id = mainE->getDrawing2S()->edges.size();
				edge.name = "__split_edge";
				edge.vertice1 = endIndex;
				edge.vertice2 = endIndex + 1;
				mainE->getDrawing2S()->edges.push_back(edge);

				//add a patch to the template
				NewPatchLine2D3D* np = new NewPatchLine2D3D(mainE->getRefTemplateElement(), edge.id, endIndex, endIndex + 1);

				//update connecting information
				connInfo.addPatchPair(np, addEdge, isOpposite, isOpposite2D);
				finalAddV1S = addEdgeX->getVertexS1();
				finalAddV2S = addEdgeX->getVertexS2();

			}
			else if (splitAdditional){
				double ratio1 = (projV1 - addV1).norm() / (addV2 - addV1).norm();
				double ratio2 = (projV2 - addV1).norm() / (addV2 - addV1).norm();
				pp1 = interpolate(addP1, addP2, ratio1);
				pp2 = interpolate(addP1,addP2, ratio2);

				//add points to the face containing this patch
				Element_Symbolic* addE = addEdge->getElement();
				int endIndex = addE->getMesh3S()->vertices.size();
				addE->getMesh3S()->vertices.push_back(pp1);
				addE->getMesh3S()->vertices.push_back(pp2);

				//add 2D points to drawing
				int endIndex1 = addE->getDrawing2S()->vertices.size();
				addE->getDrawing2S()->vertices.push_back(pd1);
				addE->getDrawing2S()->vertices.push_back(pd2);
				Edge2S edge;
				edge.id = addE->getDrawing2S()->edges.size();
				edge.name = "__split_edge";
				edge.vertice1 = endIndex;
				edge.vertice2 = endIndex + 1;
				addE->getDrawing2S()->edges.push_back(edge);
				
				//add a patch to the template
				NewPatchLine2D3D* np = new NewPatchLine2D3D(addE->getRefTemplateElement(), edge.id, endIndex, endIndex + 1);

				//update connecting information
				connInfo.addPatchPair(mainEdge, np, isOpposite, isOpposite2D);
				
				//do another optimization to snap to the lowest point to the ground if necessary
				NewPatchLine2D3D* addEdgeX2 = isOpposite ? np->flip() : np;
				finalAddV1S = addEdgeX2->getVertexS1();
				finalAddV2S = addEdgeX2->getVertexS2();
				
			}
			else{
				LOG(WARNING) << "warning:: not splitting either edge";
			}



		}
		
	}

	/*
	Constraints:
		1. main template parameters should not change. I * Q[main] = q[main]
		2. add template parameters should obey their constraints. (eq and ineq)
		
	Optimize:
		1. add patch vertices = main patch vertices (3D)
		2. add template parameters should try to stay the same.
		
	*/

	double optCost = -1;
	if (optimizationNeeded) {
		FriendlyLinearSystem fls;
		ConcreteSymbolicAssignment addEnv;
		ConcreteSymbolicAssignment mainEnv;
		addTemplate->addCurrentEnvTo(addEnv);
		mainTemplate->addCurrentEnvTo(mainEnv);
		Point3S lowestZpoint = addTemplate->getLowestZPoint();
		auto constraints = addTemplate->getConstraints(TreeScope::DESCENDANTS);
		for (auto it = constraints.begin(); it != constraints.end(); it++) {
			Constraint* constraint = *it;
			LinearExpr linearExpr = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
			if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
				fls.addEqConstraint(linearExpr, "constraint");
			}
			else {
				fls.addIneqConstraint(linearExpr, "constraint");
			}
		}
		for each (auto entry in addEnv.map) {
			auto symbolLinearExpr = LinearExpr(entry.first);
			fls.addEquationToOptimize(symbolLinearExpr - entry.second, 1, "preserve add template param");
		} 
		fls.addEqConstraint(finalAddV1S.x - finalMainV1[0], "snap vertex 1 x");
		fls.addEqConstraint(finalAddV1S.y - finalMainV1[1], "snap vertex 1 y");
		fls.addEqConstraint(finalAddV1S.z - finalMainV1[2], "snap vertex 1 z");
		fls.addEqConstraint(finalAddV2S.x - finalMainV2[0], "snap vertex 2 x");
		fls.addEqConstraint(finalAddV2S.y - finalMainV2[1], "snap vertex 2 y");
		fls.addEqConstraint(finalAddV2S.z - finalMainV2[2], "snap vertex 2 z");
		if (snapV3){
			fls.addEqConstraint(lowestZpoint.y, "snap lowest point to ground");
			if (!snapV1 || !snapV2){
				// don't add patchPair, we have already added above
				//fls.print();
				optCost = fls.optimizeAndUpdate(addTemplate);
			}
			else{
				//fls.print();
				optCost = fls.optimizeAndUpdate(addTemplate);
				connInfo.addPatchPair(mainEdge, addEdge, isOpposite, isOpposite2D);
			}
			
			
		}
		else{
			//fls.print();
			optCost = fls.optimizeAndUpdate(addTemplate);
			connInfo.addPatchPair(mainEdge, addEdge, isOpposite, isOpposite2D);
		}				
	}
	return optCost;
}








double ConstraintsEval::updateTemplateForSnapping(Template* addTemplate, Template* mainTemplate, 
	Template* result, std::vector<Constraint*> snapconstraints){

		FriendlyLinearSystem fls;
		ConcreteSymbolicAssignment addEnv;
		ConcreteSymbolicAssignment mainEnv;
		ConcreteSymbolicAssignment resultMap;
		addTemplate->addCurrentEnvTo(addEnv);
		mainTemplate->addCurrentEnvTo(mainEnv);
		result->addCurrentEnvTo(resultMap);
		Point3S lowestZpoint = addTemplate->getLowestZPoint();
		//add add temp constraints
		auto constraints = addTemplate->getConstraints(TreeScope::DESCENDANTS);
		for (auto it = constraints.begin(); it != constraints.end(); it++) {
			Constraint* constraint = *it;
			LinearExpr linearExpr = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
			if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
				fls.addEqConstraint(linearExpr, "constraint");
			}
			else {
				fls.addIneqConstraint(linearExpr, "constraint");
			}
		}
		//add newConstraits as inequ
		for (auto it = snapconstraints.begin(); it != snapconstraints.end(); it++) {
			Constraint* constraint = *it;
			LinearExpr linearExpr = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
			if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
				fls.addEquationToOptimize(linearExpr, 100, "constraint");
			}
			else {
				fls.addIneqConstraint(linearExpr, "constraint");
			}
		}

/*
		for each (auto entry in addEnv.map) {
			auto symbolLinearExpr = LinearExpr(entry.first);
			fls.addEquationToOptimize(symbolLinearExpr - entry.second, 1, "preserve add template param");
		} 
		for each (auto entry in mainEnv.map) {
			auto symbolLinearExpr = LinearExpr(entry.first);
			fls.addEqConstraint(symbolLinearExpr - entry.second, "preserve add template param");
		} 

		*/
		return  fls.optimizeAndUpdate(addTemplate);
	
}

void ConstraintsEval::updateTempByChangingOneParam(Template* tmpl, int parameter,  double displacement){
	FriendlyLinearSystem fls;
	for each (auto constraint in tmpl->getConstraints(TreeScope::DESCENDANTS)) {
		LinearExpr le = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
		if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
			fls.addEqConstraint(le, "constraint");
			}
		else {
			fls.addIneqConstraint(le, "constraint");
		}
	}
	
	vector<Symbol> symbols = tmpl->getSymbols(TreeScope::DESCENDANTS);

	for each (auto symbol in symbols) {
		auto symbolLinearExpr = LinearExpr(symbol);
		fls.addEquationToOptimize(symbolLinearExpr - symbol.owner->getCurrentValue(symbol.id), 1, "preserve template param");
}
	auto changingParamLE = LinearExpr(symbols[parameter]);
	fls.addEquationToOptimize(changingParamLE - symbols[parameter].owner->getCurrentValue(symbols[parameter].id),
		100, "optimizing parameter");
	fls.optimizeAndUpdate(tmpl);
}






vector<Eigen::Vector3d> getTwoPlaneNorms(Eigen::Vector3d a, Eigen::Vector3d b) {
	double threshold = 0.8;
	vector<Eigen::Vector3d> result;
	Eigen::Vector3d ab = (b - a).normalized();
	Eigen::Vector3d xAxis, yAxis, norm1, norm2;
	xAxis << 1, 0, 0;
	yAxis << 0, 1, 0;
	if (abs(ab.dot(xAxis)) < threshold) {
		norm1 = ab.cross(xAxis).normalized();
		norm2 = ab.cross(norm1);
		result.push_back(norm1);
		result.push_back(norm2);
	}
	else {
		norm1 = ab.cross(yAxis).normalized();
		norm2 = ab.cross(norm1);
		result.push_back(norm1);
		result.push_back(norm2);
	}
	return result;
}

void ConstraintsEval::updateTemplateBySubElementFaceScaling(TemplateElement* tempElement, int subElementId, int axis, double scale, PatchPair const& snappingConstraint) {
	PROFILE_THIS(__FUNCTION__);

	PROFILE_BEGIN(__FUNCTION__ " init");
	Template* temp = tempElement->getRoot();
	FriendlyLinearSystem fls;
	ConcreteSymbolicAssignment env;
	temp->addCurrentEnvTo(env);
	BBox3S templateBBox = temp->evalLocalBoundingBox();
	Point3S templateCenter = templateBBox.center();
	
	Eigen::Vector3d templateCenterCurrent = templateCenter.evalVector3d(env);
	PROFILE_END;
	
	PROFILE_BEGIN(__FUNCTION__ " construct");

	PROFILE_BEGIN(__FUNCTION__ " construct1");
	vector<Constraint*> constraints = temp->getConstraints(TreeScope::DESCENDANTS);

	for (auto it = constraints.begin(); it != constraints.end(); it++) {
		Constraint* constraint = *it;
		LinearExpr linearExpr = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
		if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
			fls.addEqConstraint(linearExpr, "constraint");
		} else {
			fls.addIneqConstraint(linearExpr, "constraint");
		}
	}
	PROFILE_END;

	PROFILE_BEGIN(__FUNCTION__ " construct2");
	vector<Element*> elements = temp->getAllElementList();

	for (int i = 0; i <elements.size(); i++){
		Element_Symbolic* element = dynamic_cast<Element_Symbolic*>(elements[i]);
		if (element == nullptr) {
			LOG(ERROR) << "Ignoring non Element_Symbolic, not implemented.";
			continue;
		}
		PROFILE_BEGIN(__FUNCTION__ " construct3");
		auto subElements = element->getSubElements();
		PROFILE_END;
		for (int j = 0; j < subElements->faces.size(); j++) {
			const BBox2S& bbox = subElements->faces[j].bboxS;
			auto width = bbox.max.x - bbox.min.x;
			auto height = bbox.max.y - bbox.min.y;
			auto widthCurrent = width.eval(env);
			auto heightCurrent = height.eval(env);
			if (element->getRefTemplateElement() == tempElement && j == subElementId && axis == 0) {
				fls.addEquationToOptimize(width - scale * widthCurrent, 100.0, "scale face bbox width");
			}
			else {
				fls.addEquationToOptimize(width - widthCurrent, 1.0, "preserve face bbox width");
			}
			if (element->getRefTemplateElement() == tempElement && j == subElementId && axis == 1) {
				fls.addEquationToOptimize(height - scale * heightCurrent, 100.0, "scale face bbox height");
			}
			else {
				fls.addEquationToOptimize(height - heightCurrent, 1.0, "preserve face bbox height");
			}
		}
	}



	for each (auto entry in env.map) {
		std::string SymbolName = entry.first.owner->getSymbolName(entry.first.id);
		//std::cout << "symboName: " << SymbolName << std::endl;
		bool isMove = false;
		bool isMotion = false;
		if (SymbolName.find("globalx_3d" ) != std::string::npos) {
			isMove = true;
		}
		if (SymbolName.find("globaly_3d" ) != std::string::npos) {
			isMove = true;
		}
		if (SymbolName.find("globalz_3d" ) != std::string::npos) {
			isMove = true;
		}		
		if (SymbolName.find("globalx_2d" ) != std::string::npos) {
			isMove = true;
		}
		if (SymbolName.find("globaly_2d" ) != std::string::npos) {
			isMove = true;
		}
		if (SymbolName.find("ServorPos" ) != std::string::npos) {
			isMove = false;
		}
		if (SymbolName.find("motion_" ) != std::string::npos) {
			isMotion = true;
		}
		
		if(isMove){
			//std::cout << "this is a move so I dont care" << std::endl;
			auto symbolLinearExpr = LinearExpr(entry.first);
			fls.addEquationToOptimize(symbolLinearExpr - entry.second, 0.01, "preserve current template param");
		}else if(isMotion){
			auto symbolLinearExpr = LinearExpr(entry.first);			
			fls.addEqConstraint(symbolLinearExpr - entry.second, "preserve current motion param");
		}else{
			auto symbolLinearExpr = LinearExpr(entry.first);
			fls.addEquationToOptimize(symbolLinearExpr - entry.second, 0.1, "preserve current template param");
		}
	}

		
	PROFILE_END;

	//scaling right after snapping should keep the snapped edges snapped together
/*	if (snappingConstraint.wt_patch != nullptr) {
		Eigen::Vector3d A = dynamic_cast<NewPatchLine2D3D*>(snappingConstraint.wt_patch)->getVertex1();
		Eigen::Vector3d C = dynamic_cast<NewPatchLine2D3D*>(snappingConstraint.wt_patch)->getVertex2();
		auto planes = getTwoPlaneNorms(A, C);
		Point3S B1 = dynamic_cast<NewPatchLine2D3D*>(snappingConstraint.add_patch)->getVertexS1();
		LinearExpr cnst11 = (B1[0] * planes[0][0] + B1[1] * planes[0][1] + B1[2] * planes[0][2]) - A.dot(planes[0]);
		LinearExpr cnst12 = (B1[0] * planes[1][0] + B1[1] * planes[1][1] + B1[2] * planes[1][2]) - A.dot(planes[1]);
		Point3S B2 = dynamic_cast<NewPatchLine2D3D*>(snappingConstraint.add_patch)->getVertexS2();
		LinearExpr cnst21 = (B2[0] * planes[0][0] + B2[1] * planes[0][1] + B2[2] * planes[0][2]) - A.dot(planes[0]);
		LinearExpr cnst22 = (B2[0] * planes[1][0] + B2[1] * planes[1][1] + B2[2] * planes[1][2]) - A.dot(planes[1]);
		fls.addEqConstraint(cnst11, "additional template vertex 1 on snapping line (plane 1)");
		fls.addEqConstraint(cnst12, "additional template vertex 1 on snapping line (plane 2)");
		fls.addEqConstraint(cnst21, "additional template vertex 2 on snapping line (plane 1)");
		fls.addEqConstraint(cnst22, "additional template vertex 2 on snapping line (plane 2)");
		Point3S Bmid;
		for (int i = 0; i < 3; i++) {
			Bmid[i] = (B1[i] + B2[i]) / 2;
			fls.addEqConstraint(Bmid[i] - Bmid[i].eval(env), "preserve snapping edge midpoint");
		}
	}
	//otherwise, we conserve the center of the model
	else {
	*/
	fls.addEquationToOptimize(templateCenter.x - templateCenterCurrent.x(), 1.0, "template center x");
	fls.addEquationToOptimize(templateCenter.y - templateCenterCurrent.y(), 1.0, "template center y");
	fls.addEquationToOptimize(templateCenter.z - templateCenterCurrent.z(), 1.0, "template center z");

//	}
	PROFILE_END;

	//PrecomputedLinearSystem linSystem2;
	//fls.precomputeConstrains(temp, linSystem);
	//fls.optimizeAndUpdateUsingPrecomputation(temp, linSystem); 
	//fls.print();
	fls.optimizeAndUpdate(temp);

	//we also want to update kinChain once template is changed
	/*
	try{

	}
	catch (...){
		//drop these exception about animation
		cout << "warning: exception happened while creating kinchain in maintemplate - connect()" << endl;
	}*/

	constraints.clear();
	elements.clear(); 
}


void ConstraintsEval::updateTemplateBySubElementFaceScalingWithFixedAmount(TemplateElement* tempElement, int subElementId, int axis, double amount, PatchPair const& snappingConstraint) {
	PROFILE_THIS(__FUNCTION__);

	PROFILE_BEGIN(__FUNCTION__ " init");
	Template* temp = tempElement->getRoot();
	FriendlyLinearSystem fls;
	ConcreteSymbolicAssignment env;
	temp->addCurrentEnvTo(env);
	BBox3S templateBBox = temp->evalLocalBoundingBox();
	Point3S templateCenter = templateBBox.center();
	
	Eigen::Vector3d templateCenterCurrent = templateCenter.evalVector3d(env);
	PROFILE_END;
	
	PROFILE_BEGIN(__FUNCTION__ " construct");

	PROFILE_BEGIN(__FUNCTION__ " construct1");
	vector<Constraint*> constraints = temp->getConstraints(TreeScope::DESCENDANTS);

	for (auto it = constraints.begin(); it != constraints.end(); it++) {
		Constraint* constraint = *it;
		LinearExpr linearExpr = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
		if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
			fls.addEqConstraint(linearExpr, "constraint");
		} else {
			fls.addIneqConstraint(linearExpr, "constraint");
		}
	}
	PROFILE_END;

	PROFILE_BEGIN(__FUNCTION__ " construct2");
	vector<Element*> elements = temp->getAllElementList();

	for (int i = 0; i <elements.size(); i++){
		Element_Symbolic* element = dynamic_cast<Element_Symbolic*>(elements[i]);
		if (element == nullptr) {
			LOG(ERROR) << "Ignoring non Element_Symbolic, not implemented.";
			continue;
		}
		PROFILE_BEGIN(__FUNCTION__ " construct3");
		auto subElements = element->getSubElements();
		PROFILE_END;
		for (int j = 0; j < subElements->faces.size(); j++) {
			const BBox2S& bbox = subElements->faces[j].bboxS;
			auto width = bbox.max.x - bbox.min.x;
			auto height = bbox.max.y - bbox.min.y;
			auto widthCurrent = width.eval(env);
			auto heightCurrent = height.eval(env);
			if (element->getRefTemplateElement() == tempElement && j == subElementId && axis == 0) {
				fls.addEquationToOptimize(width - (widthCurrent + amount), 100.0, "scale face bbox width");
			}
			else {
				fls.addEquationToOptimize(width - widthCurrent, 1.0, "preserve face bbox width");
			}
			if (element->getRefTemplateElement() == tempElement && j == subElementId && axis == 1) {
				fls.addEquationToOptimize(height - (heightCurrent + amount), 100.0, "scale face bbox height");
			}
			else {
				fls.addEquationToOptimize(height - heightCurrent, 1.0, "preserve face bbox height");
			}
		}
	}



	for each (auto entry in env.map) {
		std::string SymbolName = entry.first.owner->getSymbolName(entry.first.id);
		//std::cout << "symboName: " << SymbolName << std::endl;
		bool isMove = false;
		bool isMotion = false;
		if (SymbolName.find("globalx_3d" ) != std::string::npos) {
			isMove = true;
		}
		if (SymbolName.find("globaly_3d" ) != std::string::npos) {
			isMove = true;
		}
		if (SymbolName.find("globalz_3d" ) != std::string::npos) {
			isMove = true;
		}		
		if (SymbolName.find("globalx_2d" ) != std::string::npos) {
			isMove = true;
		}
		if (SymbolName.find("globaly_2d" ) != std::string::npos) {
			isMove = true;
		}
		if (SymbolName.find("ServorPos" ) != std::string::npos) {
			isMove = false;
		}
		if (SymbolName.find("motion_" ) != std::string::npos) {
			isMotion = true;
		}
		
		if(isMove){
			//std::cout << "this is a move so I dont care" << std::endl;
			auto symbolLinearExpr = LinearExpr(entry.first);
			fls.addEquationToOptimize(symbolLinearExpr - entry.second, 0.01, "preserve current template param");
		}else if(isMotion){
			auto symbolLinearExpr = LinearExpr(entry.first);			
			fls.addEqConstraint(symbolLinearExpr - entry.second, "preserve current motion param");
		}else{
			auto symbolLinearExpr = LinearExpr(entry.first);
			fls.addEquationToOptimize(symbolLinearExpr - entry.second, 0.1, "preserve current template param");
		}
	}

		
	PROFILE_END;

	//scaling right after snapping should keep the snapped edges snapped together
/*	if (snappingConstraint.wt_patch != nullptr) {
		Eigen::Vector3d A = dynamic_cast<NewPatchLine2D3D*>(snappingConstraint.wt_patch)->getVertex1();
		Eigen::Vector3d C = dynamic_cast<NewPatchLine2D3D*>(snappingConstraint.wt_patch)->getVertex2();
		auto planes = getTwoPlaneNorms(A, C);
		Point3S B1 = dynamic_cast<NewPatchLine2D3D*>(snappingConstraint.add_patch)->getVertexS1();
		LinearExpr cnst11 = (B1[0] * planes[0][0] + B1[1] * planes[0][1] + B1[2] * planes[0][2]) - A.dot(planes[0]);
		LinearExpr cnst12 = (B1[0] * planes[1][0] + B1[1] * planes[1][1] + B1[2] * planes[1][2]) - A.dot(planes[1]);
		Point3S B2 = dynamic_cast<NewPatchLine2D3D*>(snappingConstraint.add_patch)->getVertexS2();
		LinearExpr cnst21 = (B2[0] * planes[0][0] + B2[1] * planes[0][1] + B2[2] * planes[0][2]) - A.dot(planes[0]);
		LinearExpr cnst22 = (B2[0] * planes[1][0] + B2[1] * planes[1][1] + B2[2] * planes[1][2]) - A.dot(planes[1]);
		fls.addEqConstraint(cnst11, "additional template vertex 1 on snapping line (plane 1)");
		fls.addEqConstraint(cnst12, "additional template vertex 1 on snapping line (plane 2)");
		fls.addEqConstraint(cnst21, "additional template vertex 2 on snapping line (plane 1)");
		fls.addEqConstraint(cnst22, "additional template vertex 2 on snapping line (plane 2)");
		Point3S Bmid;
		for (int i = 0; i < 3; i++) {
			Bmid[i] = (B1[i] + B2[i]) / 2;
			fls.addEqConstraint(Bmid[i] - Bmid[i].eval(env), "preserve snapping edge midpoint");
		}
	}
	//otherwise, we conserve the center of the model
	else {
	*/
	fls.addEquationToOptimize(templateCenter.x - templateCenterCurrent.x(), 1.0, "template center x");
	fls.addEquationToOptimize(templateCenter.y - templateCenterCurrent.y(), 1.0, "template center y");
	fls.addEquationToOptimize(templateCenter.z - templateCenterCurrent.z(), 1.0, "template center z");

//	}
	PROFILE_END;

	//PrecomputedLinearSystem linSystem2;
	//fls.precomputeConstrains(temp, linSystem);
	//fls.optimizeAndUpdateUsingPrecomputation(temp, linSystem); 
	//fls.print();
	fls.optimizeAndUpdate(temp);

	//we also want to update kinChain once template is changed
	/*
	try{

	}
	catch (...){
		//drop these exception about animation
		cout << "warning: exception happened while creating kinchain in maintemplate - connect()" << endl;
	}*/

	constraints.clear();
	elements.clear(); 
}



void ConstraintsEval::updateTemplateBySubElementFaceTranslate(KinChain * kinChain, TemplateElement* tempElement, int subElementId, int axis, double scale) {
	PROFILE_THIS(__FUNCTION__);

	PROFILE_BEGIN(__FUNCTION__ " init");
	Template* temp = tempElement->getRoot();
	FriendlyLinearSystem fls;
	ConcreteSymbolicAssignment env;
	temp->addCurrentEnvTo(env);
	BBox3S templateBBox = temp->evalLocalBoundingBox();
	Point3S templateCenter = templateBBox.center();
	
	Eigen::Vector3d templateCenterCurrent = templateCenter.evalVector3d(env);
	PROFILE_END;
	
	PROFILE_BEGIN(__FUNCTION__ " construct");

	PROFILE_BEGIN(__FUNCTION__ " construct1");
	vector<Constraint*> constraints = temp->getConstraints(TreeScope::DESCENDANTS);

	for (auto it = constraints.begin(); it != constraints.end(); it++) {
		Constraint* constraint = *it;
		LinearExpr linearExpr = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
		if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
			fls.addEqConstraint(linearExpr, "constraint");
		} else {
			fls.addIneqConstraint(linearExpr, "constraint");
		}
	}
	PROFILE_END;

	PROFILE_BEGIN(__FUNCTION__ " construct2");
	vector<Element*> elements = temp->getAllElementList();

	for (int i = 0; i <elements.size(); i++){
		Element_Symbolic* element = dynamic_cast<Element_Symbolic*>(elements[i]);
		if (element == nullptr) {
			LOG(ERROR) << "Ignoring non Element_Symbolic, not implemented.";
			continue;
		}
		PROFILE_BEGIN(__FUNCTION__ " construct3");
		auto subElements = element->getSubElements();
		PROFILE_END;
		for (int j = 0; j < subElements->faces.size(); j++) {
			const BBox2S& bbox = subElements->faces[j].bboxS;
			auto pos_width = bbox.mid.x;
			auto pos_height = bbox.mid.y;
//			std::cout << "the pos_with is " ;
//			pos_width.print();
//			std::cout << std::endl;
			auto pos_widthCurrent = pos_width.eval(env);
			auto pos_heightCurrent = pos_height.eval(env);
			if (element->getRefTemplateElement() == tempElement && j == subElementId && axis == 0) {
				fls.addEquationToOptimize(pos_width - (scale + pos_widthCurrent), 100.0, "scale face bbox pos_width");
			}
			if (element->getRefTemplateElement() == tempElement && j == subElementId && axis == 1) {
				fls.addEquationToOptimize(pos_height - (scale + pos_heightCurrent), 100.0, "scale face bbox height");
			}
		}
	}

	std::vector<Template*> fixedTemplates;
	for each ( auto e in dynamic_cast<KinNode_Part*>(kinChain->getRoot())->elements){
		fixedTemplates.push_back(e);
	}
	for each (auto n in kinChain->getNodes()){
		n->getAllTemplatesFromUnrestrictedNodes(tempElement, fixedTemplates);
	}
	

	auto tempList = tempElement->getRoot()->getTemplatesByScope(TreeScope::DESCENDANTS);
	for each (auto entry in env.map) {
		Template* owner = const_cast<Template*>(static_cast<const Template*>(entry.first.owner));
		bool shouldBeFixed = false;
		for each (auto t in fixedTemplates){
			if ( owner == t){
				shouldBeFixed = true;
			}
		}
		std::string SymbolName = entry.first.owner->getSymbolName(entry.first.id);
		//std::cout << "symboName: " << SymbolName << std::endl;
		bool isMove = false;
		bool isMotion = false;
		if (SymbolName.find("globalx_3d" ) != std::string::npos) {
			isMove = true;
		}
		if (SymbolName.find("globaly_3d" ) != std::string::npos) {
			isMove = true;
		}
		if (SymbolName.find("globalz_3d" ) != std::string::npos) {
			isMove = true;
		}
		if (SymbolName.find("ServorPos" ) != std::string::npos) {
			isMove = true;
		}
		if (SymbolName.find("motion_" ) != std::string::npos) {
			isMotion = true;
		}
		if(shouldBeFixed){
			auto symbolLinearExpr = LinearExpr(entry.first);
			fls.addEqConstraint(symbolLinearExpr - entry.second, "preserve current motion param");
			//fls.addEquationToOptimize(symbolLinearExpr - entry.second, 100, "preserve current template param");

		}else{
			if(isMove){
				//std::cout << "this is a move so I dont care" << std::endl;
				auto symbolLinearExpr = LinearExpr(entry.first);
				fls.addEquationToOptimize(symbolLinearExpr - entry.second, 1, "preserve current template param");
			}else{
				auto symbolLinearExpr = LinearExpr(entry.first);
				//fls.addEquationToOptimize(symbolLinearExpr - entry.second, 100, "preserve current template param");
				fls.addEqConstraint(symbolLinearExpr - entry.second, "preserve current motion param");
			}
		}
	}
	PROFILE_END;

	//scaling right after snapping should keep the snapped edges snapped together
	//otherwise, we conserve the center of the model

	fls.addEquationToOptimize(templateCenter.x - templateCenterCurrent.x(), 10, "template center x");
	fls.addEquationToOptimize(templateCenter.y - templateCenterCurrent.y(), 10, "template center y");
	fls.addEquationToOptimize(templateCenter.z - templateCenterCurrent.z(), 10, "template center z");

	PROFILE_END;

	//PrecomputedLinearSystem linSystem2;
	//fls.precomputeConstrains(temp, linSystem);
	//fls.optimizeAndUpdateUsingPrecomputation(temp, linSystem); 
	//fls.print();
	fls.optimizeAndUpdate(temp);

	//we also want to update kinChain once template is changed
	/*
	try{

	}
	catch (...){
		//drop these exception about animation
		cout << "warning: exception happened while creating kinchain in maintemplate - connect()" << endl;
	}*/
}

double ConstraintsEval::updateTemplateForMultiSnappingWithEdges(Template* mainTemplate, Template* addTemplate, const vector<PatchPair>& patchPairs, bool actuallyUpdate) {
	FriendlyLinearSystem fls;
	for each (auto constraint in addTemplate->getConstraints(TreeScope::DESCENDANTS)) {
		LinearExpr le = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
		if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
			fls.addEqConstraint(le, "constraint");
		}
		else {
			fls.addIneqConstraint(le, "constraint");
		}
	}

	int i = 0;
	for each (auto pp in patchPairs) {
		auto pMain = dynamic_cast<NewPatchLine2D3D*>(pp.wt_patch);
		auto pAdd = dynamic_cast<NewPatchLine2D3D*>(pp.add_patch);
		auto isOpposite = pp.isOpposite;
		auto pAddX = isOpposite ? pAdd->flip() : pAdd;
		auto v1Add = pAddX->getVertexS1();
		auto v1Main = pMain->getVertexS1();
		auto v2Add = pAddX->getVertexS2();
		auto v2Main = pMain->getVertexS2();
		fls.addEquationToOptimize(v1Add.x - v1Main.x.eval(SymbolicAssignment::USE_CURRENT_VALUES), 100.0, concat() << "patch pair " << i << " snap v1 x");
		fls.addEquationToOptimize(v1Add.y - v1Main.y.eval(SymbolicAssignment::USE_CURRENT_VALUES), 100.0, concat() << "patch pair " << i << " snap v1 y");
		fls.addEquationToOptimize(v1Add.z - v1Main.z.eval(SymbolicAssignment::USE_CURRENT_VALUES), 100.0, concat() << "patch pair " << i << " snap v1 z");
		fls.addEquationToOptimize(v2Add.x - v2Main.x.eval(SymbolicAssignment::USE_CURRENT_VALUES), 100.0, concat() << "patch pair " << i << " snap v2 x");
		fls.addEquationToOptimize(v2Add.y - v2Main.y.eval(SymbolicAssignment::USE_CURRENT_VALUES), 100.0, concat() << "patch pair " << i << " snap v2 y");
		fls.addEquationToOptimize(v2Add.z - v2Main.z.eval(SymbolicAssignment::USE_CURRENT_VALUES), 100.0, concat() << "patch pair " << i << " snap v2 z");
		i++;
	}
	
	ConcreteSymbolicAssignment env;
	addTemplate->addCurrentEnvTo(env);
	for each (auto entry in env.map) {
		auto symbolLinearExpr = LinearExpr(entry.first);
		fls.addEquationToOptimize(symbolLinearExpr - entry.second, 1, "preserve add template param");
	}
	//LOG(INFO) << "Optimization for multisnapping:";
	//fls.print();
	if (actuallyUpdate) {
		return fls.optimizeAndUpdate(addTemplate);
	}
	else {
		return fls.optimizeWithoutUpdating(addTemplate);
	}
}

void ConstraintsEval::putContactPointsOnTheGround(Template* tmpl) {
	FriendlyLinearSystem fls;
	for each (auto constraint in tmpl->getConstraints(TreeScope::DESCENDANTS)) {
		LinearExpr le = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
		if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
			fls.addEqConstraint(le, "constraint");
		}
		else {
			fls.addIneqConstraint(le, "constraint");
		}
	}

	ConcreteSymbolicAssignment env;
	tmpl->addCurrentEnvTo(env);

	for each (auto entry in env.map) {
		std::string SymbolName = entry.first.owner->getSymbolName(entry.first.id);
		//std::cout << "symboName: " << SymbolName << std::endl;
		bool isGlobalTranslation = false;
		bool isMotion = false;
		if (SymbolName.find("globalx_3d" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		if (SymbolName.find("globaly_3d" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		if (SymbolName.find("globalz_3d" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		if (SymbolName.find("motion_" ) != std::string::npos) {
			isMotion = true;
		}
		auto symbolLinearExpr = LinearExpr(entry.first);
		if(isGlobalTranslation){
			fls.addEquationToOptimize(symbolLinearExpr - entry.second, 0.1, "preserve current template param");
		}else if(isMotion){
			fls.addEqConstraint(symbolLinearExpr - entry.second, "preserve current motion param");
		}else{
			fls.addEquationToOptimize(symbolLinearExpr - entry.second, 1.0, "preserve current template param");
		}
	}


	for each ( auto c in tmpl->getAllContactPoints()){
		fls.addEquationToOptimize(c->cPoint.y, 10.0, "preserve face bbox width");
	}

	//LOG(INFO) << "Optimization for connecting:";
	//fls.print();
	fls.optimizeAndUpdate(tmpl);
}


void ConstraintsEval::enforceConstraintsSymmetry(Template* tmpl, bool uniformSpacing) {
	FriendlyLinearSystem fls;
	for each (auto constraint in tmpl->getConstraints(TreeScope::DESCENDANTS)) {
		LinearExpr le = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
		if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
			fls.addEqConstraint(le, "constraint");
		}
		else {
			fls.addIneqConstraint(le, "constraint");
		}
	}

	ConcreteSymbolicAssignment env;
	tmpl->addCurrentEnvTo(env);
	//for each (auto entry in env.map) {
	//	auto symbolLinearExpr = LinearExpr(entry.first);
	//	fls.addEquationToOptimize(symbolLinearExpr - entry.second, 1, "preserve template param");
	//}


	for each (auto entry in env.map) {
		std::string SymbolName = entry.first.owner->getSymbolName(entry.first.id);
		bool isGlobalTranslation = false;
		bool isMotion = false;
		if (SymbolName.find("globalx_3d" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		if (SymbolName.find("globaly_3d" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		if (SymbolName.find("globalz_3d" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		if (SymbolName.find("motion_" ) != std::string::npos) {
			isMotion = true;
		}
		auto symbolLinearExpr = LinearExpr(entry.first);
		if(isGlobalTranslation){
			fls.addEquationToOptimize(symbolLinearExpr - entry.second, 0.1, "preserve current template param");
		}else if(isMotion && !uniformSpacing){
			fls.addEqConstraint(symbolLinearExpr - entry.second, "preserve current motion param");
		}else{
			fls.addEquationToOptimize(symbolLinearExpr - entry.second, 1.0, "preserve current template param");
		}
	}


	//LOG(INFO) << "Optimization for connecting:";
	//fls.print();
	fls.optimizeAndUpdate(tmpl);
}


void ConstraintsEval::enforceConstraints(Template* tmpl) {
	FriendlyLinearSystem fls;
	for each (auto constraint in tmpl->getConstraints(TreeScope::DESCENDANTS)) {
		LinearExpr le = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
		if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
			fls.addEqConstraint(le, "constraint");
		}
		else {
			fls.addIneqConstraint(le, "constraint");
		}
	}

	ConcreteSymbolicAssignment env;
	tmpl->addCurrentEnvTo(env);
	for each (auto entry in env.map) {
		auto symbolLinearExpr = LinearExpr(entry.first);
		fls.addEquationToOptimize(symbolLinearExpr - entry.second, 1, "preserve template param");
	}




	//LOG(INFO) << "Optimization for connecting:";
	//fls.print();
	fls.optimizeAndUpdate(tmpl);
}



void ConstraintsEval::optimizeController(SymbolicController * symbController, PWLinearController* linearController){
	Template* tmpl = symbController->getRefTemp();
	FriendlyLinearSystem fls;
	for each (auto constraint in tmpl->getConstraints(TreeScope::DESCENDANTS)) {
		LinearExpr le = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
		if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
			fls.addEqConstraint(le, "constraint");
		}
		else {
			fls.addIneqConstraint(le, "constraint");
		}
	}


	ConcreteSymbolicAssignment env;
	tmpl->addCurrentEnvTo(env);
	for each (auto entry in env.map) {
		std::string SymbolName = entry.first.owner->getSymbolName(entry.first.id);
		bool isMotion = false;
		if (SymbolName.find("motion_" ) != std::string::npos) {
			isMotion = true;
		}
		auto symbolLinearExpr = LinearExpr(entry.first);
		if(!isMotion){
			fls.addEquationToOptimize(symbolLinearExpr - entry.second, 100.0, "preserve current template param");
		}

		int nValues = symbController->getElementMotion()->controlPointsS.size();
		int nValues_check = linearController->controllPoints.size();
		if (nValues != nValues_check){
			std::cout << "Error - number of controll point are missmatches " << std::endl;
			system("pause");
		}
		for (int i =0; i <nValues; i++){
			auto t_symb = symbController->getElementMotion()->controlPointsS[i].t;
			auto it =  std::next(linearController->controllPoints.begin(), i);
			double t = (*it).t; //t_symb.eval(SymbolicAssignment::USE_CURRENT_VALUES);
			auto val_symb = symbController->getElementMotion()->controlPointsS[i].val;
			double val =  (*it).val; //val_symb.eval(SymbolicAssignment::USE_CURRENT_VALUES);
			
			fls.addEquationToOptimize(t_symb -t, 1.0, "preserve current template param");
			fls.addEquationToOptimize(val_symb - val, 1.0, "preserve current template param");
		
		}

	}
	fls.optimizeAndUpdate(tmpl);
}


void ConstraintsEval::extractFeasibilityConstraintsFixingSomePameters(Template* tmpl, std::vector<Symbol> & fixSymbols, FlatLinearSystem* flat){

	ConcreteSymbolicAssignment env;
	tmpl->addCurrentEnvTo(env);
	BBox3S templateBBox = tmpl->evalLocalBoundingBox();
	Point3S templateCenter = templateBBox.center();

	Eigen::Vector3d templateCenterCurrent = templateCenter.evalVector3d(env);

	FriendlyLinearSystem fls;
	for each (auto constraint in tmpl->getConstraints(TreeScope::DESCENDANTS)) {
		LinearExpr le = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
		if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
			fls.addEqConstraint(le, "constraint");
			}
		else {
			fls.addIneqConstraint(le, "constraint");
		}
	}
	fls.addEqConstraint(templateCenter.x - templateCenterCurrent.x(), "template center x");
	fls.addEqConstraint(templateCenter.y - templateCenterCurrent.y(), "template center y");
	fls.addEqConstraint(templateCenter.z - templateCenterCurrent.z(), "template center z");




	vector<Element*> elements = tmpl->getAllElementList();

	for each (auto entry in fixSymbols) {
		auto symbolLinearExpr = LinearExpr(entry);
		double val = symbolLinearExpr.eval(SymbolicAssignment::USE_CURRENT_VALUES);
		fls.addEqConstraint(symbolLinearExpr - val, "preserve current template param");

	}
	for each (auto entry in env.map) {
		std::string SymbolName = entry.first.owner->getSymbolName(entry.first.id);
		//std::cout << "symboName: " << SymbolName << std::endl;
		bool isGlobalTranslation = false;
		if (SymbolName.find("globalx_2d" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		if (SymbolName.find("globaly_2d" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		auto symbolLinearExpr = LinearExpr(entry.first);
		if(isGlobalTranslation){
			fls.addEqConstraint(symbolLinearExpr - entry.second, "preserve current template param");
		}
	}


	for (int i = 0; i <elements.size(); i++){
		Element_Symbolic* element = dynamic_cast<Element_Symbolic*>(elements[i]);
		if (element == nullptr) {
			LOG(ERROR) << "Ignoring non Element_Symbolic, not implemented.";
		}
		auto subElements = element->getSubElements();
		for (int j = 0; j < subElements->faces.size(); j++) {
			const BBox2S& bbox = subElements->faces[j].bboxS;
			auto width = bbox.max.x - bbox.min.x;
			auto height = bbox.max.y - bbox.min.y;
			auto widthCurrent = width.eval(env);
			auto heightCurrent = height.eval(env);
			fls.addEquationToOptimize(width - widthCurrent, 1.0, "preserve face bbox width");
			fls.addEquationToOptimize(height - heightCurrent, 1.0, "preserve face bbox height");
			
		}
	}


	fls.flatten(tmpl, false, flat);
}

void ConstraintsEval::enforceConstraintsConnect(Template* tmpl) {
	FriendlyLinearSystem fls;
	for each (auto constraint in tmpl->getConstraints(TreeScope::DESCENDANTS)) {
		LinearExpr le = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
		if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
			fls.addEqConstraint(le, "constraint");
		}
		else {
			fls.addIneqConstraint(le, "constraint");
		}
	}

	ConcreteSymbolicAssignment env;
	tmpl->addCurrentEnvTo(env);
	for each (auto entry in env.map) {
		std::string SymbolName = entry.first.owner->getSymbolName(entry.first.id);
		//std::cout << "symboName: " << SymbolName << std::endl;
		bool isGlobalTranslation = false;
		bool isMotion = false;
		if (SymbolName.find("globalx_3d" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		if (SymbolName.find("globaly_3d" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		if (SymbolName.find("globalz_3d" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		if (SymbolName.find("globalx_2d" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		if (SymbolName.find("globaly_2d" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		if (SymbolName.find("ServorPos" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		if (SymbolName.find("patch_variable" ) != std::string::npos) {
			isGlobalTranslation = true;
		}
		if (SymbolName.find("motion_" ) != std::string::npos) {
			isMotion = true;
		}
		auto symbolLinearExpr = LinearExpr(entry.first);
		if(isGlobalTranslation){
			fls.addEquationToOptimize(symbolLinearExpr - entry.second, 0.1, "preserve current template param");
		}else if(isMotion){
			fls.addEqConstraint(symbolLinearExpr - entry.second, "preserve current motion param");
		}else{
			fls.addEquationToOptimize(symbolLinearExpr - entry.second, 100.0, "preserve current template param");
		}
	}

	//LOG(INFO) << "Optimization for connecting:";
	//fls.print();
	fls.optimizeAndUpdate(tmpl);
}

void ConstraintsEval::extractFeasibilityConstraints(Template* tmpl, FlatLinearSystem* flat){

	ConcreteSymbolicAssignment env;
	tmpl->addCurrentEnvTo(env);
	BBox3S templateBBox = tmpl->evalLocalBoundingBox();
	Point3S templateCenter = templateBBox.center();

	Eigen::Vector3d templateCenterCurrent = templateCenter.evalVector3d(env);

	FriendlyLinearSystem fls;
	for each (auto constraint in tmpl->getConstraints(TreeScope::DESCENDANTS)) {
		LinearExpr le = dynamic_cast<Constraint*>(constraint)->getLinearExpr();
		if (constraint->getRelation() == Constraint::ConstraintRelation::EQ) {
			fls.addEqConstraint(le, "constraint");
			}
		else {
			fls.addIneqConstraint(le, "constraint");
		}
	}
	fls.addEqConstraint(templateCenter.x - templateCenterCurrent.x(), "template center x");
	fls.addEqConstraint(templateCenter.y - templateCenterCurrent.y(), "template center y");
	fls.addEqConstraint(templateCenter.z - templateCenterCurrent.z(), "template center z");




	vector<Element*> elements = tmpl->getAllElementList();

	for (int i = 0; i <elements.size(); i++){
		Element_Symbolic* element = dynamic_cast<Element_Symbolic*>(elements[i]);
		if (element == nullptr) {
			LOG(ERROR) << "Ignoring non Element_Symbolic, not implemented.";
		}
		auto subElements = element->getSubElements();
		for (int j = 0; j < subElements->faces.size(); j++) {
			const BBox2S& bbox = subElements->faces[j].bboxS;
			auto width = bbox.max.x - bbox.min.x;
			auto height = bbox.max.y - bbox.min.y;
			auto widthCurrent = width.eval(env);
			auto heightCurrent = height.eval(env);
			fls.addEquationToOptimize(width - widthCurrent, 1.0, "preserve face bbox width");
			fls.addEquationToOptimize(height - heightCurrent, 1.0, "preserve face bbox height");
			
		}
	}


	fls.flatten(tmpl, false, flat);
}