#include "FoldableGraph.h"
#include "template.h"
#include "templateElement.h"
#include "element_symbolic.h"
#include "symbolic.h"
#include "NewPatch.h"
#include "NewConnection.h"
#include "PrintingParameters.h"
#include "geometry.h"
#include "ReducedEval.h"
#include <Eigen/Geometry>
using namespace FabByExample;

#define DEBUG false


bool FoldableNode_Face::checkHasLinePatch(NewPatch * patch, int * newEdgeID){
	if(patch->getType() == NewPatch::PatchType::LINE2D3D){
		NewPatchLine2D3D * patch2 = dynamic_cast<NewPatchLine2D3D*>(patch);
		for(int i = 0; i< face->edges.size(); i++){
			if ((face->edges[i].vertice1 == patch2->vertexId1) && (face->edges[i].vertice2 == patch2->vertexId2)){
				(* newEdgeID) = i;
				return true;
			}
			if ((face->edges[i].vertice1 == patch2->vertexId2) && (face->edges[i].vertice2 == patch2->vertexId1)){
				(* newEdgeID) = i;
				return true;
			}
		}
	}
	return false;
}



bool FoldableNode_Mesh::checkHasLinePatch(NewPatch * patch, int * newEdgeID){
	*newEdgeID = -1;
	return (patch->isRelated(refTemp));
}


FoldableEdge_LineConn::FoldableEdge_LineConn(double _angle,FoldableNode* _node1, FoldableNode* _node2,
		int _edgeID1, int _edgeID2, FoldableEdge::FoldableEdgeType _type): FoldableEdge(_angle){

	angle = _angle;
	edgeID1 = _edgeID1;
	edgeID2 = _edgeID2;
	connectedNodes.push_back(_node1);
	connectedNodes.push_back(_node2);
	type = _type;
	
}
//-----------------------------------------------------------------------

void FoldableGraph::display(){
	VLOG(3) << "nodes";

	for (int i = 0; i < nodes.size(); i++){
		VLOG(3) << "node ref = " << nodes[i]->getRefTemp()->getID();
	}
}
bool FoldableGraph::getPatchLineCorrespondance(NewPatch * patch, FoldableNode** node, int* edgeID){
	//VLOG(3) << "patch->getElement()->getID() = " << patch->getElement()->getID();
	for (int i = 0; i < nodes.size(); i++){
		// if this is the node
		if(patch->getElement()->getID() == nodes[i]->getRefTemp()->getID()){
			//check if if has an edge
			int newEdgeID;
			if(nodes[i]->checkHasLinePatch(patch, &newEdgeID)){
				(*node) = nodes[i];
				(*edgeID) = newEdgeID;
				return true;
			}
		}
	}
	return false;
}


void createFacesFromDrawing(const Drawing2S *drawing, std::vector<Face2S*> & faces){
	
	std::vector<int>  usedInd;
	for( int i = 0; i <drawing->faces.size(); i++){
		Face2S* newFace = new Face2S;
		newFace->id = drawing->faces[i].id; 
		newFace->name = drawing->faces[i].name;
		usedInd.clear();
		for(int j =0; j <drawing->faces[i].vertices.size(); j++){
			int vertID = drawing->faces[i].vertices[j];
			usedInd.push_back(vertID);
			newFace->vertices[vertID] = drawing->vertices[vertID];
		}

		for(int j =0; j <drawing->edges.size(); j++){
			bool v1 = false;
			bool v2 = false;
			for(int u = 0; u < usedInd.size(); u++){
				if(usedInd[u] == drawing->edges[j].vertice1){
					v1 = true;
				}
				if(usedInd[u] == drawing->edges[j].vertice2){
					v2 = true;
				}
			}
			if(v1 && v2){
				Edge2S newEdge = drawing->edges[j]; 
				newFace->edges.push_back(newEdge);
			}
		}
		faces.push_back(newFace);

	}

}


FoldableGraph::FoldableGraph(Template* temp){
	//step 1: retrieve all the faces
	ProgressTracking::SetProgress("Preparing the faces...");
	std::vector<Element*> elements;
	temp->getAllElementList(elements);
	auto tempConnections = temp->getConnections(TreeScope::DESCENDANTS);
	for (int i = 0; i < elements.size(); i++) {
		auto ele = dynamic_cast<Element_Symbolic*>(elements[i]);
		if (ele != nullptr) {
		// check if it is a 3d print
			TemplateElement* tempEl = ele->getRefTemplateElement(); 
			if(tempEl->getPrintMethod() == Semantics::PrintMethod::DIRECT_3D){
				
				// find the connection
				std::vector<NewConnection*> elementConnections;
 				for each ( auto c in tempConnections){
 					if(c->isRelated(ele->getRefTemplateElement())){
 						elementConnections.push_back(c);
 					}
 				}
 				if(elementConnections.size()!= 1){
 					std::cout << "error, not exacly one connection" << std::endl;
 					system("pause");
 				}
 				auto patches = elementConnections[0]->getPatches();
				NewPatchLine2D3D* myPatch = nullptr;
				NewPatchLine2D3D* connectedPatch = nullptr;
				for each ( auto p in patches){
					if (p->isRelated(tempEl)){
						myPatch = dynamic_cast<NewPatchLine2D3D*>(p);
					}else{
						connectedPatch = dynamic_cast<NewPatchLine2D3D*>(p);
					}
				}
				if((myPatch == nullptr) || (connectedPatch == nullptr)){
 					std::cout << "error, missing a patch information" << std::endl;
 					system("pause");
				}
				FoldableNode_Mesh * meshNode = new FoldableNode_Mesh(tempEl, i, ele->getMesh3S(), myPatch, connectedPatch);
 				nodes.push_back(meshNode); 

			}else{
				if( ele->getDrawing2S() != nullptr){
					std::vector<Face2S*> faces;
					createFacesFromDrawing(ele->getDrawing2S(), faces);
					for(int j = 0; j < faces.size(); j++){
						FoldableNode_Face * faceNode = new FoldableNode_Face(ele->getRefTemplateElement(), i, faces[j]);
						nodes.push_back(faceNode); 
					}
				}else{
					LOG(ERROR) << "2D drawing does not exist";
				}
			}
		}else{
		}
	}
	display(); 
	//step 2: retrieve all the edges
	ProgressTracking::SetProgress("Preparing the edges...");
	std::vector<NewConnection *> connections;  
	temp->getConnections(connections, TreeScope::DESCENDANTS);
	for(int i = 0; i < connections.size(); i++){
		//VLOG(3) << "new Connection";
		NewConnection * conn = connections[i];
		// check if it is a line connection
		bool isLineConn = true;
		if(conn->patches.size() != 2){
			isLineConn = false;
		}else{
			if(conn->patches[0]->getType() != NewPatch::PatchType::LINE2D3D)
				isLineConn = false;
			if(conn->patches[1]->getType() != NewPatch::PatchType::LINE2D3D)
				isLineConn = false;
		}
		if(isLineConn){
			VLOG(3) << "angle = " << conn->getAngle();
			//We are doing conneciton based on two edges: this is the only type we have so far
			// so now we  have to read the patches encoded in conn and get the edgeIDs that they refer to
			FoldableNode* node1; FoldableNode* node2;
			int edgeID1, edgeID2;
			bool found1  = getPatchLineCorrespondance(conn->patches[0], &node1, &edgeID1);
			bool found2 = getPatchLineCorrespondance(conn->patches[1], &node2, &edgeID2 );
			//once we have the edgesIDs we can create a new a FoldableEdge_LineConn
			// we get the type FOLD/TEETH etc form the conn:
			FoldableEdge_LineConn* edge;
			if(!found1 || !found2){
				LOG(ERROR) << "could not find patch line correspondence " << std::endl;
			}
			if((edgeID2 <0) || (edgeID1 <0)){
				edge = new FoldableEdge_LineConn(conn->getAngle(), node1, node2, edgeID1, edgeID2, FoldableEdge::FoldableEdgeType::FILL);
			}
			else{
				NewConnection::ConnectionType connType = conn->getConnectionType();
				double angle = conn->getAngle();
				angle = (180-angle);
				switch(connType){
				case NewConnection::ConnectionType::FOLD:
					VLOG(3) << "Getting Fold in FoldableGraph.........";
					edge = new FoldableEdge_LineConn(angle, node1, node2, edgeID1, edgeID2, FoldableEdge::FoldableEdgeType::FOLD);
					break;
				case NewConnection::ConnectionType::TEETH:
					VLOG(3) << "Getting Teeth in FoldableGraph.........";
					if(angle > 90){
						std::cout << "Wea re connecting angles >90 with teeth - " << std::endl;
						//system("pause");
					}
					if ((angle == 0.0) || (angle == 180.0)){
						edge = new FoldableEdge_LineConn(angle, node1, node2, edgeID1, edgeID2, FoldableEdge::FoldableEdgeType::TOUCH);
					}else{
						edge = new FoldableEdge_LineConn(angle, node1, node2, edgeID1, edgeID2, FoldableEdge::FoldableEdgeType::TEETH);
					}
					break;
				case NewConnection::ConnectionType::TOUCH:
					VLOG(3) << "Getting touch in FoldableGraph.........";
					edge = new FoldableEdge_LineConn(conn->getAngle(), node1, node2, edgeID1, edgeID2, FoldableEdge::FoldableEdgeType::TOUCH);
					break;
				case NewConnection::ConnectionType::HINGE:
					VLOG(3) << "Getting hinge in FoldableGraph.........";
					edge = new FoldableEdge_LineConn(conn->getAngle(), node1, node2, edgeID1, edgeID2, FoldableEdge::FoldableEdgeType::HINGE);
					break;
				case NewConnection::ConnectionType::BALLJOINT:
					VLOG(3) << "Getting joint in FoldableGraph.........";
					edge = new FoldableEdge_LineConn(conn->getAngle(), node1, node2, edgeID1, edgeID2, FoldableEdge::FoldableEdgeType::BALLJOINT);
					break;
				case NewConnection::ConnectionType::PRISMATIC:
					VLOG(3) << "Getting prismatic joint in FoldableGraph.........";
					edge = new FoldableEdge_LineConn(conn->getAngle(), node1, node2, edgeID1, edgeID2, FoldableEdge::FoldableEdgeType::PRISMATIC);
					break;
				case NewConnection::ConnectionType::NONE:
					VLOG(3) << "Getting nonprinted joint in FoldableGraph.........";
					edge = new FoldableEdge_LineConn(conn->getAngle(), node1, node2, edgeID1, edgeID2, FoldableEdge::FoldableEdgeType::NONPRINTED);
					break;
				default:
					break;
				}
			}
			edges.push_back(edge);
		}
		
		
	}


}


FoldableGraph::~FoldableGraph(){
	nodes.clear();
	edges.clear(); 
}

double FoldableEdge::getTotalShrinkAmount(FoldableEdge::FoldableEdgeType type, double ang) {

	double amount = 0;
	switch (type){
		case FoldableEdge::FoldableEdgeType::FILL:{
			amount = PrintingParameters::getMinGap()/2;
		}
		case FoldableEdge::FoldableEdgeType::TOUCH:
			// 05/12/2016: REMOVE TOUCH --> only folds
			//amount = PrintingParameters::getWallThickness()/tan(ang/2)+PrintingParameters::getMinGap()/2;
			//break;
		case FoldableEdge::FoldableEdgeType::TEETH:
			// 05/12/2016: REMOVE ALL TEETH --> only folds!
			//amount = PrintingParameters::getTeethShrinkAmount(ang)	;		
			//amount = PrintingParameters::getTeethMulti()*PrintingParameters::getWallThickness() *3 / 2;
			//break;
		case FoldableEdge::FoldableEdgeType::FOLD:{
			//amount = para.getHingeShrinkAmount(); //calculated based on r1=wallThikness/3, and the other metrics			
			//double alpha = M_PI/3;
			//amount = (0.5790 + 1/tan(alpha))*PrintingParameters::getWallThickness() - PrintingParameters::getMinGap()/2; // 1.7345;
			if (abs(ang) < 0.01) {
				amount = PrintingParameters::getMinGap()/2;
			} //else if(PrintingParameters::useMultiMaterialFold()){
			//	double alpha = M_PI/3;
			//	amount = (0.5790 + 1/tan(alpha))*PrintingParameters::getWallThickness(); // 1.7345;
			//}
			else{
				amount = PrintingParameters::getFoldHingeShrinkAmount(); //calculated based on r1=wallThikness/3, and the other metrics
			}
			}break;
		case FoldableEdge::FoldableEdgeType::HINGE:{
			amount = PrintingParameters::getHingeShrinkAmount(); //calculated based on r1=wallThikness/3, and the other metrics
		}break;
		case FoldableEdge::FoldableEdgeType::BALLJOINT:
			amount = PrintingParameters::getJointShrinkAmount();
			break;
		case FoldableEdge::FoldableEdgeType::PRISMATIC:
			amount = PrintingParameters::getPrismaticShrinkAmount();
			break;
		case FoldableEdge::FoldableEdgeType::NONPRINTED:
			amount = PrintingParameters::getMinGap()/2;
			break;
		default: 
			amount = 0;
			break;
	}

	return amount;
}

PrintableDesign FoldableGraph::generatePrintableDesign() {
	PrintableDesign pd;

	ProgressTracking::SetProgress("Generating faces...");
	for (int i = 0; i < nodes.size(); i++){
		// create all faces
		FoldableNode_Face* faceNode = dynamic_cast<FoldableNode_Face*>(nodes[i]);
		FoldableNode_Mesh* meshNode = dynamic_cast<FoldableNode_Mesh*>(nodes[i]);
		if(faceNode != nullptr){
			TemplateElement* tempEl = dynamic_cast<TemplateElement*> (faceNode->getRefTemp());
			Element_Symbolic * el_symb = dynamic_cast<Element_Symbolic*> (tempEl->getElement());
			
			if (el_symb == NULL){
				throw std::string("Non symbolic elements are unsupported.");
			}
			drawing::Face* face = faceNode->face->eval(SymbolicAssignment::USE_CURRENT_VALUES);
			Mesh3S * meshEl = el_symb->getMesh3S();
			TriMesh * mesh = meshEl->eval(SymbolicAssignment::USE_CURRENT_VALUES);

			Eigen::MatrixXd matrix_out = el_symb->get3Dto2Dtransform();

			/////////////////CALCULATE HOLES HERE
			std::vector<NewPatch *> patches = tempEl->getAllPatches();
			std::vector<MotorMountLocations *> mountPatches;
			std::vector<Eigen::Vector2d *> servoPatches;

			for each (NewPatch * patch in patches){
				MotorMountLocations * mounts = new MotorMountLocations;
				NewPatch::PatchType patchType = patch->getType();
				if (patchType == NewPatch::PatchType::SERVO_LINE_PATCH){
					ServoLinePatch * servoLinePatch = dynamic_cast<ServoLinePatch *>(patch);
					std::vector<ServoSpacing> servoSpacings = servoLinePatch->spacings;
					Eigen::Vector3d point13d = servoLinePatch->getVertex1().evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
					Eigen::Vector3d point23d = servoLinePatch->getVertex2().evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
				    
					Eigen::Matrix3d rotation;
					rotation = matrix_out.block(0, 0, 3, 3);
					Eigen::Vector3d translation;
					translation(0) = matrix_out(0, 3);
					translation(1) = matrix_out(1, 3);
					translation(2) = matrix_out(2, 3);

					Eigen::Vector2d point12d = (rotation * point13d + translation).head(2);
					Eigen::Vector2d point22d = (rotation * point23d + translation).head(2);



					Eigen::Vector2d dir = point22d - point12d;
					mounts->dir = dir;
					mounts->point1 = point22d;

					double multi= patch->getNormal().dot(Eigen::Vector3d::Ones());
					mounts->orientation = (multi > 0) ? 1 : -1;

					std::vector<double> spacings;
					for (int j = 0; j < servoSpacings.size(); j++){
						ServoSpacing spacing = servoSpacings[j];
						spacings.push_back(spacing.alpha.eval(SymbolicAssignment::USE_CURRENT_VALUES));
					}
					mounts->spacings = spacings;
					mountPatches.push_back(mounts);
				}else if (patchType == NewPatch::PatchType::SERVO_POINT_PATCH){
					//if this is connected...
					auto connections = tempEl->getRoot()->getAllConnections();
					for each (NewConnection * connection in connections){
						auto connectionPatches = connection->getPatches();

						if ((connectionPatches[0] == patch) || (connectionPatches[1] == patch)){
						
					
				
							ServoPointPatch * servoPointPatch = dynamic_cast<ServoPointPatch *>(patch);
							Eigen::Vector3d point13d = servoPointPatch->getCenter();
				    
							Eigen::Matrix3d rotation;
							rotation = matrix_out.block(0, 0, 3, 3);
							std::cout << "Matrix out is now " <<  std::endl;
							std::cout << matrix_out << std::endl;

							Eigen::Vector3d translation;
							translation(0) = matrix_out(0, 3);
							translation(1) = matrix_out(1, 3);
							translation(2) = matrix_out(2, 3);

							std::cout << "translation is " << translation << std::endl;
							std::cout << "rotation is " << rotation << std::endl;
							std::cout << "point13d is " << point13d << std::endl;

							if (connectionPatches[1] == patch){ //if this is further down the list than the parent, it's holes
								Eigen::Vector2d * point12d = new Eigen::Vector2d((rotation * point13d + translation).head(2));

								std::cout << "point12d is " << point12d << std::endl;

								servoPatches.push_back(point12d);
							}else{ //otherwise a servo mount
								Eigen::Vector2d dir;
								dir(0) = 0; //dummy
								dir(1) = 1;
								mounts->dir = dir;
								Eigen::Vector2d point12d = Eigen::Vector2d((rotation * point13d + translation).head(2));
								//point12d(1) -= 10; //a hack for making them fit.
								mounts->point1 = point12d;

								std::vector<double> spacings;
								spacings.push_back(0.0);
								
								mounts->spacings = spacings;
								mountPatches.push_back(mounts);
							}
						}

					}


				}
			}


			

			////////////////END CALCULATE HOLES


			PrintableFace *pf = new PrintableFace(*face, faceNode->getID(), *mesh, mountPatches, servoPatches, faceNode);
			//(*pf).print();
			pd.addPrintablePart(pf);
		}
		if(meshNode != nullptr){
			TriMesh * mesh = meshNode->mesh->eval(SymbolicAssignment::USE_CURRENT_VALUES);

			bool isOpposite = false;
			double error = (meshNode->connectedPatch->getDrawingVertex1()  -meshNode->myPatch->getDrawingVertex2()).norm();
			if(error > 0.01){
				isOpposite = true;
			}

			Eigen::Vector3d dir_me = (meshNode->myPatch->getVertex2() - meshNode->myPatch->getVertex1());
			if(isOpposite){
				dir_me = (-1.0)*dir_me;
			}
			Eigen::Vector2d dir_connection2D = (meshNode->connectedPatch->getDrawingVertex1() - meshNode->connectedPatch->getDrawingVertex2());
			Eigen::Vector3d dir_connection(dir_connection2D.x(), dir_connection2D.y(), 0);

			Eigen::MatrixXd points2D(3, 3);
			Eigen::MatrixXd points3D(3, 3);
			
			

			points2D(0, 0) = meshNode->connectedPatch->getDrawingVertex1()(0);
			points2D(1, 0) = meshNode->connectedPatch->getDrawingVertex1()(1);
			points2D(2, 0) = 0.0;
			points2D(0, 1) = meshNode->connectedPatch->getDrawingVertex2()(0);
			points2D(1, 1) = meshNode->connectedPatch->getDrawingVertex2()(1);
			points2D(2, 1) = 0.0;
			points2D(0, 2) = points2D(0, 0);
			points2D(1, 2) = points2D(1, 0) ;
			points2D(2, 2) = points2D(2, 0) + 1.0;

			

			points3D(0, 0) = meshNode->myPatch->getVertex1()(0);
			points3D(1, 0) = meshNode->myPatch->getVertex1()(1);
			points3D(2, 0) = meshNode->myPatch->getVertex1()(2);
			points3D(0, 1) = meshNode->myPatch->getVertex2()(0);
			points3D(1, 1) = meshNode->myPatch->getVertex2()(1);
			points3D(2, 1) = meshNode->myPatch->getVertex2()(2);


			std::cout << "dir me = " << dir_me.transpose() << std::endl;
			std::cout << "dir connection = " << dir_connection.transpose() << std::endl;
			
			//rotation so that the edge aligns
			Eigen::Quaterniond rot_q = Eigen::Quaterniond::FromTwoVectors(dir_me.normalized(), dir_connection.normalized());
			
			// rotation so that the face normal aligns w me:
			Eigen::Vector3d v1 = meshNode->connectedPatch->getElement()->getMesh3S()->vertices[0].evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
			Eigen::Vector3d v2 = meshNode->connectedPatch->getElement()->getMesh3S()->vertices[1].evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
			Eigen::Vector3d v3 = meshNode->connectedPatch->getElement()->getMesh3S()->vertices[2].evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
			Eigen::Vector3d faceNormal = ((v2 - v1).cross(v3 - v1)).normalized();
			std::cout << "the face normal before rotation " << faceNormal.transpose() << std::endl;
			//faceNormal = rot_q.toRotationMatrix()*faceNormal;
			//std::cout << "the face normal after rotation " << faceNormal.transpose() << std::endl;
			//Eigen::Quaterniond rot_q2 = Eigen::Quaterniond::FromTwoVectors(faceNormal, Eigen::Vector3d::UnitZ());
			
			bool debugPoints = false;
			if(debugPoints){
				Geometry* geo = new Geometry();
				for( int d = 0; d<3; d++){
					Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\uniteSphereSimp.obj"));
					Eigen::Vector3d v = meshNode->connectedPatch->getElement()->getMesh3S()->vertices[d].evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
					sphere->applyTrans(point(v.x(), v.y(),v.z()));
					geo->add(sphere);
				}
				geo->write("..\\..\\data\\pointsonMesh.obj");
			}


			

			points3D(0, 2) = points3D(0, 0) -faceNormal(0);
			points3D(1, 2) = points3D(1, 0) - faceNormal(1);
			points3D(2, 2) = points3D(2, 0) - faceNormal(2);

			if(debugPoints){
				Geometry* geo = new Geometry();
				for( int d = 0; d<3; d++){
					Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\arrow.obj"));
					sphere->applyTrans(point(points2D(0,d),points2D(1,d),points2D(2,d)));
					geo->add(sphere);
				}
				geo->write("..\\..\\data\\points2D_untransformed.obj");
			}
			if(debugPoints){
				Geometry* geo = new Geometry();
				for( int d = 0; d<3; d++){
					Geometry * sphere = new Geometry(TriMesh::read("..\\..\\data\\arrow.obj"));
					Eigen::Quaterniond rot_q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),faceNormal);
					auto rot_matt = rot_q.toRotationMatrix();
					std::vector<double> rottt(9);
					for (int i = 0; i< 9; i++){
						rottt[i] = rot_matt(i);
					}
					sphere->applyRot(rottt);
					sphere->applyTrans(point(points3D(0,d),points3D(1,d),points3D(2,d)));
					geo->add(sphere);
				}
				geo->write("..\\..\\data\\points3D_untransformed.obj");
			}
	
			auto transformation = Eigen::umeyama(points3D, points2D, false);
			//std::cout <<"The transformation is \n" << transformation << std::endl;

/*			auto rot_mat = rot_q.toRotationMatrix();
			std::vector<double> rot(9);
			for (int i = 0; i< 9; i++){
				rot[i] = rot_mat(i);
			}

			auto rot_mat2 = rot_q2.toRotationMatrix();
			std::vector<double> rot2(9);
			for (int i = 0; i< 9; i++){
				rot2[i] = rot_mat2(i);
			}

			Eigen::VectorXd centerTrans = -1.0*(meshNode->myPatch->getVertex2()) ;
			if(isOpposite){
				centerTrans = -1.0*(meshNode->myPatch->getVertex1()) ;
			}
			Eigen::Vector2d posPatch = meshNode->connectedPatch->getDrawingVertex1();
			Eigen::VectorXd moveTrans = Eigen::Vector3d(posPatch.x(), posPatch.y(), PrintingParameters::getWallThickness());
*/
			Geometry * geo = new Geometry();
			geo->addMesh(mesh);
			//geo->write("..\\..\\data\\original_Mesh.stl");

			
			Eigen::Matrix3d rotTemp = transformation.block(0, 0, 3, 3);
			std::vector<double> rotNew(9);
			for (int i = 0; i< 9; i++){
				rotNew[i] = rotTemp(i);
			}
			
			Eigen::Vector3d transNew;
			transNew(0) = transformation(0, 3);
			transNew(1) = transformation(1, 3);
			transNew(2) = transformation(2, 3);
			geo->applyRot(rotNew);
			geo->applyTrans(point(transNew(0), transNew(1), transNew(2)));

			//geo->write("..\\..\\data\\Transfolrmed_Mesh.stl");
			//system("pause"); 

			PrintableMesh *pf = new PrintableMesh(meshNode->getID(), *geo->getMesh(), meshNode);
			//(*pf).print();
			pd.addPrintablePart(pf);

		}

	}

	//TODO: add mounts here.
	/*
	for (int i = 0; i < nodes.size(); i++){
		// create all faces
		FoldableNode_Face* faceNode = dynamic_cast<FoldableNode_Face*>(nodes[i]);
		FoldableNode_Mesh* meshNode = dynamic_cast<FoldableNode_Mesh*>(nodes[i]);
		if(faceNode != nullptr){
			TemplateElement* tempEl = dynamic_cast<TemplateElement*> (faceNode->getRefTemp());
			Element_Symbolic * el_symb = dynamic_cast<Element_Symbolic*> (tempEl->getElement());
			if (el_symb == NULL){
				throw std::string("Non symbolic elements are unsupported.");
			}
			

			//TODO: calculate 2D direction and location

			drawing::Face* face = faceNode->face->eval(SymbolicAssignment::USE_CURRENT_VALUES);
			drawing::Drawing* drawing = el_symb->getDrawing2S()->eval(SymbolicAssignment::USE_CURRENT_VALUES);

			
			//auto pointIds = face->idpoints;
			auto pointIds = drawing->points;

			Eigen::Matrix3d points2D;
			Eigen::Matrix3d points3D;


			int cnt = 0;


			//get first 3 points in 2D
			for each (auto pointId in pointIds){
				points2D(0, cnt) = pointId(0);
				points2D(1, cnt) = pointId(1);
				points2D(2, cnt) = 0.0;
				cnt++;
				if (cnt > 2){
					break;
				}
			}

			//get first 3 points in 3D:
			cnt = 0;
			int numPoints = el_symb->getMesh3S()->vertices.size();
			for each (auto point3DSym in el_symb->getMesh3S()->vertices){
				auto pointCast = point3DSym.evalpoint(SymbolicAssignment::USE_CURRENT_VALUES);
				Eigen::Vector3d point3D;
				point3D(0) = pointCast[0];
				point3D(1) = pointCast[1];
				point3D(2) = pointCast[2];
				points3D.col(cnt) = point3D;
				cnt++;
				if (cnt > 2){
					break;
				}
			}

			Eigen::Vector3d translation2 =  points2D.col(0);
			Eigen::Vector3d translation1 =  points3D.col(0);
			Eigen::Vector3d diff2D1 = points2D.col(1) - points2D.col(0);
			Eigen::Vector3d diff3D1 = points3D.col(1) - points3D.col(0);

			Eigen::Quaterniond rot1 = Eigen::Quaterniond::FromTwoVectors(diff3D1, diff2D1);
			Eigen::Vector3d residual = rot1.matrix() * (points3D.col(2) - points3D.col(0));
			Eigen::Vector3d diff2D2 = points2D.col(2) - points2D.col(0);
			Eigen::Quaterniond rot2 = Eigen::Quaterniond::FromTwoVectors(residual, diff2D2);
			Eigen::Matrix3d rotation = rot2.matrix() * rot1.matrix();
			//Eigen::Matrix3d rotation = rot1.matrix();

			std::vector<NewPatch *> patches = tempEl->getAllPatches();
			for each (NewPatch * patch in patches){
				NewPatch::PatchType patchType = patch->getType();
				if (patchType == NewPatch::PatchType::SERVO_LINE_PATCH){
					ServoLinePatch * servoLinePatch = dynamic_cast<ServoLinePatch *>(patch);
					std::vector<ServoSpacing> servoSpacings = servoLinePatch->spacings;
					for (int j = 0; j < servoSpacings.size(); j++){



						ServoSpacing spacing = servoSpacings[j];

						Eigen::Vector3d point13d = servoLinePatch->getVertex1().evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
						Eigen::Vector3d point23d = servoLinePatch->getVertex2().evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
						
						Eigen::Vector2d point12d = (translation2 + (rotation * (point13d - translation1))).head(2);
						Eigen::Vector2d point22d = (translation2 + (rotation * (point23d - translation1))).head(2);
						Eigen::Vector2d dir = point22d - point12d;
						

						MotorMount * mm = new MotorMount(i, &dir, &point12d, spacing.alpha.eval(SymbolicAssignment::USE_CURRENT_VALUES)); //TODO: fill in displacement value here. TODO: should i be here?
						pd.addPrintablePart(mm);

						//std::cout << "point1 is " << point12d << std::endl;
					}
				}
			}
			
			
		}

	}
	*/


	//shrink all vertices
	//bool testJustFaces = true;

	pd.shrinkDesign(PrintingParameters::getMinGap()/2); 
	
	//edges.clear(); 
	
	
	//shrink all edges

	ProgressTracking::SetProgress("Generating edges...");
	for (int i = 0; i <edges.size(); i++){ //edges.size()

		double amount = FoldableEdge::getTotalShrinkAmount(edges[i]->getType(), edges[i]->getAngle()) - PrintingParameters::getMinGap()/2;
		if (DEBUG) {
			std::cout <<"amout = " << amount << std::endl;
		}
		if(amount > 0){
			FoldableEdge* edge = edges[i];
			FoldableEdge_LineConn * foldEdge = dynamic_cast<FoldableEdge_LineConn*>(edge);
			//get faces
			PrintableFace* pf_1 = pd.getPrintableFaceByIndex(foldEdge->connectedNodes[0]->getID());
			PrintableFace* pf_2 = pd.getPrintableFaceByIndex(foldEdge->connectedNodes[1]->getID());
			// shrink faces
			if (DEBUG) {
				std::cout << "edge is " << edges[i] << std::endl;
				std::cout << "shrink amount is " << amount << std::endl;
				std::cout << "ids are " << foldEdge->edgeID1 << std::endl;
				std::cout << "and " << foldEdge->edgeID2 << std::endl;
			}

			if(pf_1 != nullptr){
				pf_1->shrinkFace(foldEdge->edgeID1, amount);
			}
			if(pf_2 != nullptr){
				pf_2->shrinkFace(foldEdge->edgeID2, amount);
			}
			//system("pause"); 
		}

	}

	//TODO: this is where stuff needs to be commented out.
	
	//system("pause");
	
	// simplify all faces of repeat vertices / 0-length edges
	pd.simplifyFaces();


	ProgressTracking::SetProgress("Generating connections...");
	//Add all connections	
	//edges.clear();
	for (int i = 0; i <edges.size(); i++){ // edges.size()
		//get the info
		FoldableEdge* edge = edges[i];
		FoldableEdge_LineConn * foldEdge = dynamic_cast<FoldableEdge_LineConn*>(edge);
		PrintableFace* pf_1 = pd.getPrintableFaceByIndex(foldEdge->connectedNodes[0]->getID());
		PrintableFace* pf_2 = pd.getPrintableFaceByIndex(foldEdge->connectedNodes[1]->getID());

		double amount = FoldableEdge::getTotalShrinkAmount(edges[i]->getType(), edges[i]->getAngle());

		if((pf_1 == nullptr) || (pf_2 ==  nullptr)){
			std::cout << "this edge does not connect 2 faces!" << std::endl;
			Edge2dp * edge = nullptr;
			Edge2dp *auxedge;
			FoldableNode *fn;
			if(pf_1 != nullptr){
					edge = pf_1->getEdge(foldEdge->edgeID1);
					auxedge = pf_1->getUnshrinkFace(foldEdge->edgeID1, amount);
					fn = foldEdge->connectedNodes[0];
			}
			if(pf_2 != nullptr){
					edge = pf_2->getEdge(foldEdge->edgeID2);
					auxedge = pf_2->getUnshrinkFace(foldEdge->edgeID2, amount);
					fn = foldEdge->connectedNodes[1];
			}
			if (edge != nullptr){
				PrintableFill* pfill = new PrintableFill(i, edge, auxedge, fn);
				pd.addPrintablePart(pfill);
			}

		} else{
			Edge2dp * edge1 = pf_1->getEdge(foldEdge->edgeID1);
			Edge2dp * edge2 = pf_2->getEdge(foldEdge->edgeID2);

			
			Edge2dp * auxedge1 = pf_1->getUnshrinkFace(foldEdge->edgeID1, amount);
			Edge2dp * auxedge2 = pf_2->getUnshrinkFace(foldEdge->edgeID2, amount);

			FoldableNode* fn = foldEdge->connectedNodes[0];

			//add the edges
			switch (edges[i]->getType()){
			case FoldableEdge::FoldableEdgeType::FOLD:{
				PrintableFold* pfold = new PrintableFold(i, edges[i]->getAngle(), edge1, edge2, auxedge1, auxedge2, fn);
				pd.addPrintablePart(pfold);
			}break;
			case FoldableEdge::FoldableEdgeType::TEETH:{
				//amount = PrintingParameters::getTeethMulti()*PrintingParameters::getWallThickness()/tan(edges[i]->getAngle()/2)  - PrintingParameters::getMinGap()/2;
				//amount = PrintingParameters::getTeethMulti()*PrintingParameters::getWallThickness() * 3 / 2; //shannon's version of teeth design
			
				//amount = PrintingParameters::getTeethShrinkAmount(edges[i]->getAngle())	;					
				//Edge2dp * auxedge1 = pf_1->getUnshrinkFace(foldEdge->edgeID1, amount);
				//Edge2dp * auxedge2 = pf_2->getUnshrinkFace(foldEdge->edgeID2, amount);


				// 05/12/2016: REMOVING TEETH: EVERYTHING IS A FOLD
				//PrintableTeeth* pteeth = new PrintableTeeth(i, edges[i]->getAngle(), edge1, edge2, auxedge1, auxedge2, fn);
				//pd.addPrintablePart(pteeth);
				PrintableFold* pfold = new PrintableFold(i, edges[i]->getAngle(), edge1, edge2, auxedge1, auxedge2, fn);
				pd.addPrintablePart(pfold);

				}break;
			case FoldableEdge::FoldableEdgeType::TOUCH:{
				//amount = PrintingParameters::getWallThickness()/tan(edges[i]->getAngle()/2);
				//Edge2dp * auxedge1 = pf_1->getUnshrinkFace(foldEdge->edgeID1, amount);
				//Edge2dp * auxedge2 = pf_2->getUnshrinkFace(foldEdge->edgeID2, amount);

				// 05/12/2016: REMOVING TEETH: EVERYTHING IS A FOLD
				//PrintableTouch* ptouch = new PrintableTouch(i, edges[i]->getAngle(), edge1, edge2, auxedge1, auxedge2, fn);
				//pd.addPrintablePart(ptouch);
				PrintableFold* pfold = new PrintableFold(i, edges[i]->getAngle(), edge1, edge2, auxedge1, auxedge2, fn);
				pd.addPrintablePart(pfold);
				}break;
			case FoldableEdge::FoldableEdgeType::HINGE:{
				PrintableHinge* phinge = new PrintableHinge(i, edges[i]->getAngle(), edge1, edge2, auxedge1, auxedge2, fn);
				pd.addPrintablePart(phinge);
				}break;
			case FoldableEdge::FoldableEdgeType::BALLJOINT:{
				PrintableBallJoint* pjoint = new PrintableBallJoint(i, edges[i]->getAngle(), edge1, edge2, auxedge1, auxedge2, fn);
				pd.addPrintablePart(pjoint);
				}break;
			case FoldableEdge::FoldableEdgeType::PRISMATIC:{
				PrintablePrismatic* pjoint = new PrintablePrismatic(i, edges[i]->getAngle(), edge1, edge2, auxedge1, auxedge2, fn);
				pd.addPrintablePart(pjoint);
				}break;
			case FoldableEdge::FoldableEdgeType::NONPRINTED:{
				}break;
			default: 
				break;
			}
		}
	}
	
		return pd;
} 