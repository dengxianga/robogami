#include "element_symbolic.h"
#include "geometry.h"
#include "templateElement.h"

using namespace std;

#define DEBUG false

namespace FabByExample {

	Element_Symbolic::Element_Symbolic(Mesh3S* mesh3S, Drawing2S* drawing2S)
		: mesh3S(mesh3S), drawing2S(drawing2S) {
		tempGeometry = nullptr;
        drawing = nullptr;
		subElements = nullptr;
		computeTriangularization(); 
	}

	Element_Symbolic::~Element_Symbolic() {
		if (mesh3S != nullptr) {
			delete mesh3S;
		}
        if (drawing2S != nullptr) {
            delete drawing2S;
        }
		if (subElements != nullptr) {
			delete[] subElements;
		}
		if (tempGeometry != nullptr) {
			delete tempGeometry;
		}
	}

	void Element_Symbolic::setRefTempElement(TemplateElement* refTempElement)
	{
		this->refTempElement = refTempElement;
		computeTriangularization(); 
		computeNewGeo();
		computeSymbCenterOfMass();
	}

	static Eigen::Vector3d toVector(point const& p) {
		Eigen::Vector3d v;
		v[0] = p[0];
		v[1] = p[1];
		v[2] = p[2];
		return v;
	}

	static Vector3S multS(Eigen::Matrix3d const& mat, Vector3S const& v) {
		Vector3S result;
		result[0] = mat(0, 0) * v[0] + mat(0, 1) * v[1] + mat(0, 2) * v[2];
		result[1] = mat(1, 0) * v[0] + mat(1, 1) * v[1] + mat(1, 2) * v[2];
		result[2] = mat(2, 0) * v[0] + mat(2, 1) * v[1] + mat(2, 2) * v[2];
		return result;
	}


	point converttoPoint(Eigen::Vector3d const& v) {
		return point(static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2]));
	}

	void Element_Symbolic::computeNewGeo() {
		PROFILE_THIS(__FUNCTION__);

		if (tempGeometry != nullptr) {
			delete tempGeometry;
		}
		
		// Update numeric mesh
		
		
		//TriMesh* evaluatedMesh = mesh3S->eval(SymbolicAssignment::USE_CURRENT_VALUES, &faceIndices);
		//Adriana: I replaced this with the precomputed triangulation: 
		TriMesh* evaluatedMesh = new TriMesh();
		evaluatedMesh->faces.clear();
		evaluatedMesh->vertices.clear();
		for(int faces_i = 0; faces_i < triIndexes.size(); faces_i++){
			for (int v_i = 0; v_i < 3; v_i++) {
				evaluatedMesh->vertices.push_back(converttoPoint(
					evalSymbVertexExpr(triIndexes[faces_i][v_i]).evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES)));
			}
			evaluatedMesh->faces.push_back(TriMesh::Face(3*faces_i, 3*faces_i+1, 3*faces_i+2));
		}




		//evaluatedMesh->write("../../data/protoTesting/tempMesh.stl");
		tempGeometry = new Geometry(evaluatedMesh);
	
		// Update numeric drawing
		if (drawing != nullptr) {
			delete drawing;
		}
		if (drawing2S != nullptr) {
			drawing = drawing2S->eval(SymbolicAssignment::USE_CURRENT_VALUES);
		}

		// Update sub-elements
		if (subElements == nullptr) {			
			subElements = new SubElementsSpec();
			for (int i = 0; i < mesh3S->faces.size(); i++) {
				SubElementFace face;
				face.name = mesh3S->faces[i].name;
				face.min[0] = Eigen::VectorXd::Zero(refTempElement->numSymbols());
				face.min[1] = Eigen::VectorXd::Zero(refTempElement->numSymbols());
				face.max[0] = Eigen::VectorXd::Zero(refTempElement->numSymbols());
				face.max[1] = Eigen::VectorXd::Zero(refTempElement->numSymbols());
				subElements->faces.push_back(face);
			}
		
		}

		for (int i = 0; i < mesh3S->faces.size(); i++) {
			subElements->faces[i].trigs.clear();
		}
		for (int i = 0; i < faceIndices.size(); i++) {
			subElements->faces[faceIndices[i]].trigs.push_back(i);
		}

		for (int i = 0; i < subElements->faces.size(); i++) {
			auto& face = subElements->faces[i];

			// TODO: perhaps find a better algorithm for this

			// Motivation for this algorithm is that most faces will be axis
			// aligned in some direction, i.e. the normal will have a zero
			// component. Since the y direction is the up direction, we will
			// prefer to make the top edge of the bounding box on the xz-plane.
			// If the face is already on the xz-plane, we will make the top
			// edge on the xy-plane.
			auto p1 = toVector(evaluatedMesh->vertices[evaluatedMesh->faces[face.trigs[0]][0]]);
			auto p2 = toVector(evaluatedMesh->vertices[evaluatedMesh->faces[face.trigs[0]][1]]);
			auto p3 = toVector(evaluatedMesh->vertices[evaluatedMesh->faces[face.trigs[0]][2]]);
			auto norm = (p2 - p1).cross(p3 - p1).normalized();
			Eigen::Vector3d xznorm = Eigen::Vector3d::Zero();
			xznorm[1] = 1;
			Eigen::Vector3d up;
			if (norm.cross(xznorm).squaredNorm() < 1e-4) {
				Eigen::Vector3d xynorm = Eigen::Vector3d::Zero();
				xynorm[2] = 1;
				auto topEdgeVec = norm.cross(xynorm);
				up = topEdgeVec.cross(norm).normalized();
			}
			else {
				auto topEdgeVec = norm.cross(xznorm);
				up = topEdgeVec.cross(norm).normalized();
			}
			face.up = up;
			face.normal = norm;

			if (0){
				//other method to compute the faceup;

			}


			// calculate the bounding box of this face
			auto right = up.cross(norm);
			Eigen::Matrix3d basis;
			basis.col(0) = right;
			basis.col(1) = up;
			basis.col(2) = norm;
			Eigen::Matrix3d basisInv = basis.inverse();
			auto& firstPoint = mesh3S->vertices[mesh3S->faces[i].vertices[0]];
			auto firstPointEval = firstPoint.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
			int minIndex[2] = { 0, 0 }, maxIndex[2] = { 0, 0 };
			Eigen::Vector2d minVec = Eigen::Vector2d::Zero(), maxVec = Eigen::Vector2d::Zero();
			for (int j = 1; j < mesh3S->faces[i].vertices.size(); j++) {
				auto& p = mesh3S->vertices[mesh3S->faces[i].vertices[j]];
				Eigen::Vector3d pEval = p.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
				Eigen::Vector3d rel = pEval - firstPointEval;
				Eigen::Vector3d trans = basisInv * rel;
				for (int k = 0; k < 2; k++) {
					if (trans[k] < minVec[k]) {
						minIndex[k] = j;
						minVec[k] = trans[k];
					}
					if (trans[k] > maxVec[k]) {
						maxIndex[k] = j;
						maxVec[k] = trans[k];
					}
				}
			}

			LinearExpr minS[2], maxS[2];
			for (int k = 0; k < 4; k++) {
				int j = k < 2 ? minIndex[k] : maxIndex[k - 2];
				auto& p = mesh3S->vertices[mesh3S->faces[i].vertices[j]];
				Vector3S rel;
				rel[0] = p[0]; //- firstPoint[0];
				rel[1] = p[1]; //- firstPoint[1];
				rel[2] = p[2]; //- firstPoint[2];
				Vector3S trans = multS(basisInv, rel);
				if (k < 2) {
					minS[k] = trans[k];
				}
				else {
					maxS[k - 2] = trans[k - 2];
				}
			}

			// center the bounding box, and calculate center
			LinearExpr midS[2];
			
			
	//		std::cout << "before the minS is " ;
	//		minS[0].print();
	//		std::cout << std::endl;

	//		std::cout << "before the maxS is " ;
	//		maxS[0].print();
	//		std::cout << std::endl;

			for (int k = 0; k < 2; k++) {
				midS[k] = (minS[k] + maxS[k]) / 2;
				minS[k] = minS[k] - midS[k];
				maxS[k] = maxS[k] - midS[k];
			}

	//		std::cout << "after the minS is " ;
	//		minS[0].print();
	//		std::cout << std::endl;

	//		std::cout << "after the maxS is " ;
	//		maxS[0].print();
	//		std::cout << std::endl;


	//		std::cout << "after the midS is " ;
	//		midS[0].print();
	//		std::cout << std::endl;




			Eigen::Vector2d mid = (minVec + maxVec) / 2;
			Eigen::Vector3d midFlat = Eigen::Vector3d::Zero();
			midFlat.segment(0, 2) = mid;
			Eigen::Vector3d mid3d = basis * midFlat + firstPointEval;
			face.center = mid3d;
			face.width = maxVec.x() - minVec.x();
			face.height = maxVec.y() - minVec.y();

			// Copy the coefficients into the arrays in the face sub-element
			Eigen::VectorXd* arr[4] = { &face.min[0], &face.min[1], &face.max[0], &face.max[1] };
			double* arrConst[4] = { &face.minConst[0], &face.minConst[1], &face.maxConst[0], &face.maxConst[1] };
			LinearExpr* exprs[4] = { &minS[0], &minS[1], &maxS[0], &maxS[1] };
			for (int k = 0; k < 4; k++) {
				auto& coeffs = exprs[k]->getCoeffs();
				for (int i = 0; i < refTempElement->numSymbols(); i++) {
					Symbol paramSymbol = refTempElement->getSymbol(i);
					if (coeffs.has(paramSymbol)) {
						(*arr[k])[i] = coeffs.at(paramSymbol);
					}
					else {
						(*arr[k])[i] = 0;
					}
				}
				*arrConst[k] = exprs[k]->getConstant();
			}
			face.bboxS.min.x = minS[0];
			face.bboxS.min.y = minS[1];
			face.bboxS.max.x = maxS[0];
			face.bboxS.max.y = maxS[1];
			face.bboxS.mid.x = midS[0];
			face.bboxS.mid.y = midS[1];
		}
	}

	TriMesh::BBox Element_Symbolic::evalBoundingBox() {
		return mesh3S->evalLocalBoundingBox(SymbolicAssignment::USE_CURRENT_VALUES).eval(SymbolicAssignment::USE_CURRENT_VALUES);
	}

	bool hasEnding(std::string const &fullString, std::string const &ending)
	{
		if (fullString.length() >= ending.length()) {
			return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
		}
		else {
			return false;
		}
	}
	
	int Element_Symbolic::getNumSubElements() {
		return mesh3S->faces.size();
	}

	Element_Symbolic::SubElementsSpec const* Element_Symbolic::getSubElements() {
		return subElements;
	}

	drawing::Drawing const* Element_Symbolic::evalDrawing() {
        return drawing;
    }

	Mesh3S* Element_Symbolic::getMesh3S() {
		return mesh3S;
	}

	Drawing2S* Element_Symbolic::getDrawing2S(){
		return drawing2S;
	}

	void Element_Symbolic::get2D3Dpoints(Eigen::MatrixXd& points2D, Eigen::MatrixXd& points3D) {
		Mesh3S * meshEl = this->getMesh3S();
		//TriMesh * mesh = meshEl->eval(SymbolicAssignment::USE_CURRENT_VALUES);
		drawing::Drawing* drawing = this->getDrawing2S()->eval(SymbolicAssignment::USE_CURRENT_VALUES);
			
		//auto pointIds = face->idpoints;
		auto pointIds = drawing->points;

		int cnt = 0;

		if (DEBUG) {
			std::cout << "pointIDs size = " << pointIds.size() << std::endl;
		}

		// points in 2D
		for each (Eigen::Vector2d pointId in pointIds){
			if (DEBUG) {
				std::cout << "cnt = " << cnt << std::endl;
			}

			points2D(0, cnt) = pointId(0);
			points2D(1, cnt) = pointId(1);
			points2D(2, cnt) = 0.0;
			cnt++;
		}

		// points in 3D:
		cnt = 0;
			
		for each (auto point3DSym in this->getMesh3S()->vertices){
			auto pointCast = point3DSym.evalpoint(SymbolicAssignment::USE_CURRENT_VALUES);
			Eigen::Vector3d point3D;
			point3D(0) = pointCast[0];
			point3D(1) = pointCast[1];
			point3D(2) = pointCast[2];
			points3D.col(cnt) = point3D;
			cnt++;
		}
	}

	Eigen::MatrixXd Element_Symbolic::get2Dto3Dtransform() {
		int numPoints = this->getMesh3S()->vertices.size();
		if (DEBUG) {
			std::cout << "num points is " << numPoints << std::endl;
		}

		Eigen::MatrixXd points2D(3, numPoints);
		Eigen::MatrixXd points3D(3, numPoints);
		this->get2D3Dpoints(points2D, points3D);

		Eigen::MatrixXd matrix_out = Eigen::umeyama(points2D, points3D, false);

		if (DEBUG) {
			std::cout << "important matrices" << std::endl;
			std::cout << points3D << std::endl;
			std::cout << points2D << std::endl;
			std::cout << matrix_out << std::endl;
		}

		return matrix_out;
	}

	Eigen::MatrixXd Element_Symbolic::get3Dto2Dtransform() {
		int numPoints = this->getMesh3S()->vertices.size();
		if (DEBUG) {
			std::cout << "num points is " << numPoints << std::endl;
		}

		Eigen::MatrixXd points2D(3, numPoints);
		Eigen::MatrixXd points3D(3, numPoints);
		this->get2D3Dpoints(points2D, points3D);

		Eigen::MatrixXd matrix_out = Eigen::umeyama(points3D, points2D, false);

		if (DEBUG) {
			std::cout << "important matrices" << std::endl;
			std::cout << points3D << std::endl;
			std::cout << points2D << std::endl;
			std::cout << matrix_out << std::endl;
		}

		return matrix_out;
	}


LinearExpr Element_Symbolic::getSymbVertexExpr(Eigen::Vector3d v){

	double minError = std::numeric_limits<double>::max();
	int optId = 0;
	for(int i = 0; i < mesh3S->vertices.size(); i++){
		double error = (v - mesh3S->vertices[i].evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES)).norm();
		if(error < minError){
			minError = error;
			optId = i;
		}
		if(error < 0.5){
			return LinearExpr(Symbol(nullptr, i));
		}
	}

	//LOG(ERROR) << "has a point that is not close to a vertex ";
	//LOG(ERROR) << "minError = " << minError;
	return LinearExpr(Symbol(nullptr, optId));

	
	for(int i = 0; i < mesh3S->vertices.size(); i++){
		for(int j = i+1; i < mesh3S->vertices.size(); i++){
			Eigen::Vector3d a = mesh3S->vertices[i].evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
			Eigen::Vector3d b = mesh3S->vertices[j].evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
			double checkLineSeg = true;
			double crossproduct = (v-a).cross(b -a).norm(); 
			if(crossproduct > 0.01){
				checkLineSeg = false;
			}
			double l1 = (v-a).dot(b -a); 
			if(l1 < 0){
				checkLineSeg = false;
			}
			double l2 = (b-a).norm(); 
			if(l1 > l2){
				checkLineSeg = false;
			}
			if(checkLineSeg){
				double t = l1/l2;
				Eigen::Vector3d c = (1-t)*a + t*(b);
				if((c - v).norm() > 0.001){
					std::cout << "error on computing line seg: " << std::endl;
					system("pause");
				}
				return LinearExpr(Symbol(nullptr, i))*(1-t) + LinearExpr(Symbol(nullptr, j))*t;

			}
		}
	}


	LOG(ERROR) << "could not find a point ";
	return LinearExpr(Symbol(nullptr, 0));

}

Point3S Element_Symbolic::evalSymbVertexExpr(LinearExpr const& le) {
	Point3S result;
	for each (auto pair in le.getCoeffs()) {
		result = result + mesh3S->vertices[pair.first.id] * pair.second;
	}
	return result;
}

void Element_Symbolic::computeTriangularization(){


	TriMesh* evaluatedMesh = mesh3S->eval(SymbolicAssignment::USE_CURRENT_VALUES, &faceIndices);
	//evaluatedMesh->write("..\\..\\data\\test.stl");
	//system("pause");
	std::vector<LinearExpr> auxVertices(evaluatedMesh->vertices.size());
	for (int i = 0; i < evaluatedMesh->vertices.size(); i++) {
		Eigen::Vector3d v(evaluatedMesh->vertices[i][0], evaluatedMesh->vertices[i][1], evaluatedMesh->vertices[i][2]);
		auxVertices[i] = getSymbVertexExpr(v);
	}

	for(int i = 0; i < evaluatedMesh->faces.size(); i++){
		std::vector<LinearExpr> symbFace(3);
		symbFace[0] = auxVertices[evaluatedMesh->faces[i].v[0]];
		symbFace[1] = auxVertices[evaluatedMesh->faces[i].v[1]];
		symbFace[2] = auxVertices[evaluatedMesh->faces[i].v[2]];
		triIndexes.push_back(symbFace); 
	}


}	
	
	void Element_Symbolic::computeSymbCenterOfMass(){
//		std::vector<std::vector<Point3S>> triIndexes;
		for (int i = 0; i <triIndexes.size(); i++) {
			auto& p1 = evalSymbVertexExpr(triIndexes[i][0]);
			auto& p2 = evalSymbVertexExpr(triIndexes[i][1]);
			auto& p3 = evalSymbVertexExpr(triIndexes[i][2]);
			//auto& p1 = mesh3S->vertices[triIndexes[i](0)];
			//auto& p2 = mesh3S->vertices[triIndexes[i](1)];
			//auto& p3 = mesh3S->vertices[triIndexes[i](2)];
			//compute area of triangle
			TriCenterOfMassSymb* triMass = new TriCenterOfMassSymb();
			triMass->center.x = (1.0/3.0)*(p1.x + p2.x + p3.x);
			triMass->center.y = (1.0/3.0)*(p1.y + p2.y + p3.y);
			triMass->center.z = (1.0/3.0)*(p1.z + p2.z + p3.z);
		//	std::cout << "p1 = " << std::endl; p1.x.print(); p1.y.print(); p1.z.print(); 
		//	std::cout << "p2 = " << std::endl; p2.x.print(); p2.y.print(); p2.z.print(); 
		//	std::cout << "p3 = " << std::endl; p3.x.print(); p3.y.print(); p3.z.print(); 
		//	std::cout << "triMass = " << std::endl; triMass->x.print(); triMass->y.print(); triMass->z.print(); 
		//	system("pause");
//				std::cout << "p1 = " << p1.evalpoint(env)[0] << ", " << p1.evalpoint(env)[1] << "," << p1.evalpoint(env)[2] << std::endl;
//				std::cout << "p2 = " << p2.evalpoint(env)[0] << ", " << p2.evalpoint(env)[1] << "," << p2.evalpoint(env)[2] << std::endl;
//				std::cout << "p3 = " << p3.evalpoint(env)[0] << ", " << p3.evalpoint(env)[1] << "," << p3.evalpoint(env)[2] << std::endl;
//				std::cout << "triMass = " << triMass->x.eval(env) << ", " << triMass->y.eval(env) << "," << triMass->z.eval(env) << std::endl;
//				system("pause");
			//LinearExpr constMass(1.0);
			//compute mass u = p2 -p1 v = p3 -p1
			LinearExpr u1 = p2.x - p1.x; 
			LinearExpr u2 = p2.y - p1.y; 
			LinearExpr u3 = p2.z - p1.z; 
			LinearExpr v1 = p3.x - p1.x; 
			LinearExpr v2 = p3.y - p1.y; 
			LinearExpr v3 = p3.z - p1.z; 
			QuadExpr cross1 = u2*v3 - u3*v2;
			QuadExpr cross2 = u3*v1 - u1*v3;
			QuadExpr cross3 = u1*v2 - u2*v1;
			Eigen::Vector3d normal = (
				p2.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES) - p1.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES))
				.cross(p3.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES) - p1.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES));
			double actualmass = normal.norm();
			//std::cout << "mass should be = " << normal.norm()<< std::endl;
			normal.normalize();
			triMass->mass = normal.x()*cross1 + normal.y()*cross2 + normal.z()*cross3;  
			//std::cout << "triMass = " << triMass->mass.eval(env)<< std::endl;
			//system("pause"); 
			if(actualmass >0.000001)
				symbCenterOfMass.triangles.push_back(triMass);
			else{
				/*LOG(ERROR) << "there is a triangles of zero mass! ";
				Eigen::Vector3d points[3];
				points[0] = p1.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
				points[1] = p2.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
				points[2] = p3.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
				Geometry contactGeo;
				for (int i = 0; i < 3; i++){
					Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
					sphere->applyTrans(point(points[i].x(), points[i].y(), points[i].z()));
					contactGeo.add(sphere);
				}
				contactGeo.write("..\\..\\data\\test\\zeroMassTriangle.stl");
				*/
			}
			

		}

	}

	void Element_Symbolic::addCenterOfMass(CenterOfMass & centerOfMass){
		symbCenterOfMass.evalAndAdd(centerOfMass, SymbolicAssignment::USE_CURRENT_VALUES);
	}

	void Element_Symbolic::rotateCenterOfMass(Eigen::Vector3d const& center, Eigen::Quaterniond const& rotation){
		for each(TriCenterOfMassSymb* t in symbCenterOfMass.triangles){		
			t->center = t->center.translate(-center);
			t->center = t->center.times(rotation.toRotationMatrix());
			t->center = t->center.translate(center);
		}
	}


	void Element_Symbolic::translateCenterOfMass(Eigen::Vector3d const& trans){
		for each(TriCenterOfMassSymb* t in symbCenterOfMass.triangles){
			t->center = t->center.translate(trans);
		}
	}



}

