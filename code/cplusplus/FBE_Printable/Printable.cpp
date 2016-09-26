#include "Printable.h"
#include "simple_svg_1.0.0.h"
#include <drawing.h>
#include <string>
#include "primitives.h"
#include "geometry.h"
#include "PrintingParameters.h"
#include <math.h>
#include "KinChain.h"
#include "element_symbolic.h"
#include "FoldableGraph.h"
#include "templateElement.h"
#include "element_symbolic.h"
#include <iomanip>

#define DEBUG false

//#define pathdir "C:\\Users\\Jacob\\Code\\FabByExample\\data\\prints\\part"
//#define pathdir "C:\\ResearchCode\\FabByExampleExperimental\\FabByExample\\data\\prints\\part"

//#define pathdir "C:\\Users\\Wei\\Desktop\\FabByExample\\data\\prints\\"
//#define pathdir "C:\\Users\\Wei\\Desktop\\FabByExample\\code\\cplusplus\\FBE_Printable\\part"
//#define pathdir "D:\\Adriana\\projects\\PrintableRobots\\FabByExample\\data\\prints\\part"
//#define pathdir "F:\\Projects\\PPR\\FabByExample\\data\\prints\\part"

///#define pathdir "D:\\Adriana\\projects\\PrintableRobots\\FabByExample\\data\\prints\\part"

#define pathdir "..\\..\\data\\prints\\part" 
#define stldir pathdir
//#define stldir "C:\\Users\\Cindy\\Dropbox (MIT)\\DB_3DPrintableRobots\\full STLs\\"
//#define stldir "E:\\Dropbox (MIT)\\DB_3DPrintableRobots\\ProtosToRender\\UserStudy_Task2\\"
using namespace FabByExample; 


Eigen::Matrix4d PrintablePart::get2Dto3Dtransform(){
	if (this->foldnode != NULL) {
		TemplateElement* refTemp = dynamic_cast<TemplateElement*> (this->foldnode->getRefTemp());
		Element_Symbolic* elem_S = dynamic_cast<Element_Symbolic*>(refTemp->getElement());
		
		Eigen::Matrix4d transform =  elem_S->get2Dto3Dtransform();

		return transform;
	}
	else
	{
		return Eigen::Matrix4d::Identity();
	}
}

void PrintableDesign::generatePrint(std::string destfilename /*= "result"*/, bool is3D /*= false*/, KinChain* kinchain /*=NULL*/, std::vector<double> times) {
	std::map<int, std::string> hardSTLs;
	std::map<int, std::string> softSTLs;
	std::map<std::string, int> extraSTLs;
	std::string hardfile, softfile;
	
	PROCESS_INFORMATION *pid = new PROCESS_INFORMATION[10];

	int idx = 0;
	int numextra = 0;
	for(int i = 0; i < printableParts.size(); i++) {

		 //for jeff bezzos asciiproto
		if ((i > 11 && i < 16) || (i > 87 && i < 92)) {
			continue;
		}

		if (idx < i) {
			// wait for the process to complete
			// Wait until child process exits.
			WaitForSingleObject( pid[idx].hProcess, INFINITE );

			// Close process and thread handles. 
			CloseHandle( pid[idx].hProcess );
			CloseHandle( pid[idx].hThread );
		}
		printableParts[i]->convertToMesh(hardfile, softfile, pid[idx], is3D);
		hardSTLs[i] = hardfile;
		softSTLs[i] = softfile;

		// hack for fish
		double translateamount = 0;
		if (false) {// (i == 50 || i == 42 || i == 34 || i==26) {
			switch(i) {
			case 26:
				//translateamount = -62; break; // fish orig
				//translateamount = -52; break; // fish speed
				translateamount = -76; break; // fish wobbly
			case 34: 
				//translateamount = -62; break; // fish orig
				//translateamount = -90; break; // fish speed
				translateamount = -52; break; // fish wobbly
			case 42:
				//translateamount = -62; break; // fish orig
				//translateamount = -90; break; // fish speed
				translateamount = -77; break; // fish wobbly
			case 50:
				//translateamount = -62; break; // fish orig
				//translateamount = -66; break; // fish speed
				translateamount = -52; break; // fish wobbly
			}

			WaitForSingleObject( pid[idx].hProcess, INFINITE );

			TriMesh* mesh = TriMesh::read(hardSTLs[i].c_str()); 

			KinRigidTransform kT;
			kT.trans.x() = translateamount;
			kT.apply(mesh);

			mesh->write(hardSTLs[i-4].c_str());

		}

		if (printableParts[i]->getType() == PrintablePart::PrintablePartType::FACE) {
			if (is3D) {
			for each (MotorMountLocations* mml in (dynamic_cast<PrintableFace*>(printableParts[i]))->mountPatches) {
				for each (double spacing in mml->spacings){
					TriMesh* mesh = TriMesh::read("..\\..\\data\\fabSTLs\\Turnigy_1370A.STL"); 

					if (mml->orientation < 0) {
						KinRigidTransform kT;
						Eigen::Vector3d z(0,0,1);
						Eigen::AngleAxisd aa(M_PI, z);
						kT.rot = aa.toRotationMatrix();
						kT.apply(mesh);
					}

					Eigen::Vector3d dir = Eigen::Vector3d(mml->dir.x(),  mml->dir.y(), 0.0).normalized();
					Eigen::Vector3d xhat = Eigen::Vector3d(1.0, 0.0, 0.0);
					Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(xhat, dir);

					KinRigidTransform kT;
					kT.rot = q.toRotationMatrix();
					kT.apply(mesh);

					KinRigidTransform kT2;
					kT2.trans.x() = mml->point1.x()-dir.x()*spacing;
					kT2.trans.y() = mml->point1.y()-dir.y()*spacing;
					kT2.apply(mesh);

					std::stringstream extraname;
					extraname << "..\\..\\data\\prints\\motor_" << numextra << ".stl";
					mesh->write(extraname.str().c_str());

					extraSTLs[extraname.str()] = i;
					++numextra;
				}
			}
		}
		}

		idx = (idx + 1) % 10;
	}

	for (int i = 0; i < 10; ++i) {
		// Wait until child process exits.
		WaitForSingleObject( pid[i].hProcess, INFINITE );

		// Close process and thread handles. 
		CloseHandle( pid[i].hProcess );
		CloseHandle( pid[i].hThread );
	}

	delete[] pid;
	
	ProgressTracking::SetProgress("Combining the generated files...");

	for (int itime = 0; itime < times.size(); ++itime) {
		if (kinchain != NULL) {
			kinchain->updateTime(times[itime]);
		}

		Geometry * geo_hard = new Geometry();
		Geometry * geo_soft = new Geometry();
		Geometry * geo_extra = new Geometry();
		for(int i = 0; i < printableParts.size(); i++){
			if (!hardSTLs[i].empty()) {
				TriMesh* mesh = TriMesh::read(hardSTLs[i].c_str());  
				if (mesh != nullptr){
					if (is3D) {
						// transform geometry to be in 3d
						std::cout << hardSTLs[i] << ": " << mesh->vertices.size() << std::endl;
				
						Eigen::Matrix4d T = printableParts[i]->get2Dto3Dtransform();
				
						KinRigidTransform transform;
						transform.rot = T.block(0,0,3,3);
						transform.trans(0) = T(0,3);
						transform.trans(1) = T(1,3);
						transform.trans(2) = T(2,3);
						transform.apply(mesh, false);
					
						// Kinematic chain transformation
						TemplateElement* refTemp = dynamic_cast<TemplateElement*> (printableParts[i]->getFoldNode()->getRefTemp());
						kinchain->find(refTemp)->transformToRootFrame(mesh);
						kinchain->getCurrentRootPose().apply(mesh, false);
					}
					geo_hard->addMesh(mesh);
					delete mesh;
				}
			}
		
			if (!softSTLs[i].empty()) {
				TriMesh* mesh = TriMesh::read(softSTLs[i].c_str());

				if (mesh != nullptr){
					if (is3D) {
						// transform geometry to be in 3d
						std::cout << hardSTLs[i] << ": " << mesh->vertices.size() << std::endl;
				
						Eigen::Matrix4d T = printableParts[i]->get2Dto3Dtransform();
				
						KinRigidTransform transform;
						transform.rot = T.block(0,0,3,3);
						transform.trans(0) = T(0,3);
						transform.trans(1) = T(1,3);
						transform.trans(2) = T(2,3);
						transform.apply(mesh, false);
					
						// KinRigidTransform
					}
				
					geo_soft->addMesh(mesh);
					delete mesh;
				}
			}
		}

		for (std::map<std::string,int>::iterator it = extraSTLs.begin(); it != extraSTLs.end(); ++it) {
			TriMesh* mesh = TriMesh::read(it->first.c_str());  

			if (mesh != nullptr && is3D) {
				// transform geometry to be in 3d
				std::cout << "face " << it->second << "contains" << it->first << ": " << mesh->vertices.size() << std::endl;
				
				int partid = it->second;

				Eigen::Matrix4d T = printableParts[partid]->get2Dto3Dtransform();
				
				KinRigidTransform transform;
				transform.rot = T.block(0,0,3,3);
				transform.trans(0) = T(0,3);
				transform.trans(1) = T(1,3);
				transform.trans(2) = T(2,3);
				transform.apply(mesh, false);
					
				// Kinematic chain transformation
				TemplateElement* refTemp = dynamic_cast<TemplateElement*> (printableParts[partid]->getFoldNode()->getRefTemp());
				kinchain->find(refTemp)->transformToRootFrame(mesh);
				kinchain->getCurrentRootPose().apply(mesh, false);
				
				geo_extra->addMesh(mesh);
				delete mesh;
			}
		}

		if (is3D) {
			std::stringstream desthard;
			desthard << stldir << destfilename << "\\hard\\" << setfill('0') << setw(3) << itime << ".obj";
			std::stringstream destsoft;
			destsoft << stldir << destfilename << "\\soft\\" << setfill('0') << setw(3) << itime  <<".obj";
			std::stringstream destextra;
			destextra << stldir << destfilename << "\\extra\\" << setfill('0') << setw(3) << itime  <<".obj";
			geo_hard->write(desthard.str());
			geo_soft->write(destsoft.str());
			geo_extra->write(destextra.str());
		} else {
			std::stringstream desthard;
			desthard << stldir << destfilename <<"_hard.stl";
			std::stringstream destsoft;
			destsoft << stldir << destfilename << "_soft.stl";
			geo_hard->write(desthard.str());
			geo_soft->write(destsoft.str());
		}

		delete geo_hard; delete geo_soft; delete geo_extra;
	}
}


void PrintableDesign::addPrintablePart(PrintablePart *pb) {
	printableParts.push_back(pb);
}

//void PrintableDesign::addPrintableFace(PrintableFace pf) {
//	printableFaces.push_back(pf);
//}

PrintableFace* PrintableDesign::getPrintableFaceByIndex(int index) {
	for(int i = 0; i < printableParts.size(); i++){
		if((printableParts[i]->getType() ==  PrintablePart::PrintablePartType::FACE) && (printableParts[i]->getId() == index)){
			PrintableFace* face = dynamic_cast<PrintableFace*>(printableParts[i]);
			return face; 
		}
	}
	return nullptr;

}
void PrintableDesign::generateSVG(int width, int height, std::string path) {
/*	VLOG(3) << "Generating PrintableDesign SVG";
	svg::Dimensions dimensions(width, height);
	svg::Document doc(path, svg::Layout(dimensions, svg::Layout::TopLeft));

	//for(std::list<PrintablePart*>::iterator p = printableParts.begin(); p != printableParts.end(); ++p) {
	for(int i = 0; i < printableParts.size(); i++) {
		printableParts[i]->generateSVG(doc);
	}
    
	doc.save();
	*/

}


PrintableDesign::~PrintableDesign() {
	//for(std::vector<PrintablePart*>::iterator p = printableParts.begin(); p != printableParts.end(); ++p) {
	//for(int i = 0; i < printableParts.size(); i++) {
	//		delete printableParts[i];
	//}
}

void PrintableDesign::shrinkDesign(double delta) {
	for(std::vector<PrintablePart*>::iterator p = printableParts.begin(); p != printableParts.end(); ++p) {
		(*p)->shrinkAllVertices(delta); 
	}
}

void PrintableDesign::simplifyFaces() {
	for(std::vector<PrintablePart*>::iterator p = printableParts.begin(); p != printableParts.end(); ++p) {
		(*p)->simplify(); 
	}
}


namespace {
	Eigen::Vector2d find_direction(drawing::Face face, int edgeID) {

		//orientation dependent

		Eigen::Vector2d p0 = face.getPointFromId(face.edges[edgeID].vertice1_id)->p;
		Eigen::Vector2d p1 = face.getPointFromId(face.edges[edgeID].vertice2_id)->p;

		Eigen::Vector2d p = PrintingParameters::getOrientInd()*( p1 - p0);

		Eigen::Vector2d d = Eigen::Vector2d(-p.y(), p.x());

		d.normalize();
		//VLOG(3) << "D: " << " (" << d.x() << ", " << d.y() << ")";
		return d;

	}

	Eigen::Vector2d find_normal(const Eigen::Vector2d& p) {
		
		Eigen::Vector2d d = Eigen::Vector2d(-p.y(), p.x());

		d.normalize();
		//VLOG(3) << "D: " << " (" << d.x() << ", " << d.y() << ")";
		return d;

	}

	Eigen::Vector2d reduce_p(const Eigen::Vector2d& direction, double delta, const Eigen::Vector2d& p, const Eigen::Vector2d& c) {
		// normalize direction (just in case)
		Eigen::Vector2d ldirection(direction);
		ldirection.normalize();
		//VLOG(3) << "LD: " << " (" << ldirection.x() << ", " << ldirection.y() << ")";
		// get target unit
		Eigen::Vector2d t = c - p;
		
		t.normalize();
		//VLOG(3) << "T: " << " (" << t.x() << ", " << t.y() << ")";

		double step = delta / t.dot(ldirection);
		//VLOG(3) << "Tdot: " << " (" << t.x() << ", " << t.y() << ")";
		//VLOG(3) << "Step: " << step;

		Eigen::Vector2d newp = p + t*step;
	    return newp;

	}

	drawing::Point::point_vector_type::iterator find_sucessor(const Eigen::Vector2d& direction, 
		const drawing::Point::point_vector_type::iterator e2,  drawing::Face & face) {
		drawing::Point::point_vector_type::iterator n;
		
		
		drawing::Point::point_vector_type::iterator lastOne = face.idpoints.end();
		--lastOne;
		if(e2 == lastOne){
			n = face.idpoints.begin();
		}else{
			n = e2;
			++n;
		}
		//VLOG(3) << "sending = " << n->id;
		/*
		if (e2->id >= face.getMaxID()) {
			n = face.getPointFromId(face.getMinID());
		} else {
			n = face.getPointFromId(e2->id+1);
		}
		*/
		return n;
	}
	drawing::Point::point_vector_type::iterator find_predecessor(const Eigen::Vector2d& direction, 
		drawing::Point::point_vector_type::iterator e1, drawing::Face & face) {
    
		drawing::Point::point_vector_type::iterator n;

		drawing::Point::point_vector_type::iterator b =  face.idpoints.begin();
		if(e1->id == b->id){
			n = face.idpoints.end();
			--n;
		}else{
			n = e1;
			--n; 
		}
		/*
		if (e1->id <= face.getMinID()) {
			n = face.getPointFromId(face.getMaxID());
		} else {
			n = face.getPointFromId(e1->id-1);
		}
		*/
		return n;
	}

	Edge2dp* getreduce_pol(const Eigen::Vector2d& direction, double delta, int edgeID, drawing::Face& face) {

		drawing::Point::point_vector_type::iterator e1 = face.getPointFromId(face.edges[edgeID].vertice1_id);
		drawing::Point::point_vector_type::iterator e2 = face.getPointFromId(face.edges[edgeID].vertice2_id);


		Eigen::Vector2d p1 = e1->p;
		Eigen::Vector2d p2 = e2->p;

		if (DEBUG) {
			VLOG(3) << "Current 1: " << e1->id << ": " << p1.transpose();
			VLOG(3) << "Current 2: " << e2->id << ": " << p2.transpose();
		}

		drawing::Point::point_vector_type::iterator n1 = find_predecessor(direction, e1, face );
		drawing::Point::point_vector_type::iterator n2 = find_sucessor(direction, e2, face );
		if (DEBUG) {
			VLOG(3) << "Predecessor: " << n1->id << ": " << n1->p.transpose();
			VLOG(3) << "Sucessor: " << n2->id << ": " << n2->p.transpose();
		}    
		Eigen::Vector2d c1 = n1->p;

		Eigen::Vector2d newp1;
		if(abs((c1-p1).normalized().dot(direction)) < 0.01 ) { 
			//std::cout << "Direction: " << direction.transpose() << std::endl;
			newp1 = p1 + direction*delta; //c1 + direction*delta;
		}else{
			newp1 = reduce_p (direction, delta, p1, c1);
		}
		
		
		Eigen::Vector2d c2 = n2->p;
		Eigen::Vector2d newp2;
		if(abs((c2-p2).normalized().dot(direction)) < 0.01 ) { 
			newp2 = p2 + direction*delta; //c2 + direction*delta;
		}else{
			newp2 = reduce_p (direction, delta, p2, c2);
		}

		Edge2dp* edge = new Edge2dp();
		edge->v1 = newp1;
		edge->v2 = newp2;
		return edge;

	}
	void reduce_pol(const Eigen::Vector2d& direction, double delta, int edgeID, drawing::Face& face) {
		
		drawing::Point::point_vector_type::iterator e1 = face.getPointFromId(face.edges[edgeID].vertice1_id);
		drawing::Point::point_vector_type::iterator e2 = face.getPointFromId(face.edges[edgeID].vertice2_id);

		Eigen::Vector2d p1 = e1->p;
		Eigen::Vector2d p2 = e2->p;

		if (DEBUG) {
			VLOG(3) << "e1: " << e1->id << " x: " << p1.x() << " y: " << p1.y();
			VLOG(3) << "e2: " << e2->id << " x: " << p2.x() << " y: " << p2.y();
		}
		
		drawing::Point::point_vector_type::iterator n1 = find_predecessor(direction, e1, face );
		//VLOG(3) << "Predecessor: " << n1->id << " x: " << n1->p.x() << " y: " << n1->p.y();
		drawing::Point::point_vector_type::iterator n2 = find_sucessor(direction, e2, face );
		//VLOG(3) << "Sucessor: " << n2->id << " x: " << n2->p.x() << " y: " << n2->p.y();
    
		Eigen::Vector2d c1 = n1->p;

		if(abs((c1-p1).normalized().dot(direction)) < 0.4) { // CORRECT? original was abs((c1-p1).dot(direction))  < 0.4){
			//VLOG(3) << "shrinking colinear face";
			Eigen::Vector2d newp1 = p1 + direction*delta;
			// create new point
			drawing::Point newPoint;
			newPoint.id = face.getMaxID() +1;
			newPoint.p = newp1;
			face.idpoints.insert(std::next(e1), newPoint);
			// add new edge
			drawing::Edge newEdge;
			face.edges.push_back(newEdge);
			face.edgeMap[newPoint.id] = face.edgeMap[e1->id];
			face.edgeMap[e1->id] = face.edges.size()-1;
			// fix endpoint of original edge
			face.edges[edgeID].vertice1_id = newPoint.id;
			//change the edge 
			// insert between n1 and e1
		}else{
			Eigen::Vector2d newp1 = reduce_p (direction, delta, p1, c1);
			//VLOG(3) << "P1: " << " (" << p1.x() << ", " << p1.y() << ")";
			//VLOG(3) << "NewP1: " << " (" << newp1.x() << ", " << newp1.y() << ")";
			e1->p = newp1;
			// record edge reduction info in edges
			face.edges[edgeID].trimFront += (newp1-p1).dot((p2-p1).normalized());
			face.edges[face.edgeMap[n1->id]].trimBack += (newp1-p1).dot((c1-p1).normalized());
			//std::cout << "trimfront "<< edgeID << " " << face.edges[edgeID].trimFront << "back " << face.edgeMap[n1->id] << " "<< face.edges[face.edgeMap[n1->id]].trimBack << std::endl;
		}
		
		
		Eigen::Vector2d c2 = n2->p;

		if(abs((c2-p2).normalized().dot(direction)) < 0.4){ // CORRECT? original was abs((c1-p1).dot(direction))  < 0.4){
			VLOG(3) << "shrinking colinear face2"; 
			Eigen::Vector2d newp2 = p2 + direction*delta;
			// create new point
			drawing::Point newPoint;
			newPoint.id = face.getMaxID() +1;
			newPoint.p = newp2;
			face.idpoints.insert(e2, newPoint);
			// add new edge
			drawing::Edge newEdge;
			face.edges.push_back(newEdge);
			face.edgeMap[newPoint.id] = face.edges.size()-1;
			// fix endpoint info
			face.edges[edgeID].vertice2_id = newPoint.id;
		}else{
			Eigen::Vector2d newp2 = reduce_p (direction, delta, p2, c2);
			e2->p = newp2;
			// record edge reduction info in edges
			face.edges[edgeID].trimBack += (newp2-p2).dot((p1-p2).normalized());
			face.edges[face.edgeMap[e2->id]].trimFront += (newp2-p2).dot((c2-p2).normalized());
			//std::cout << "trimback " << edgeID << " " << face.edges[edgeID].trimBack << "front " << << face.edgeMap[e2->id] << " "<< face.edges[face.edgeMap[e2->id]].trimFront << std::endl;
		}
	}

}

/*********************   FILL     *********************************/

PrintableFill::PrintableFill(int id, Edge2dp* _edge1, Edge2dp* _auxedge1, FoldableNode* fn): PrintablePart(id, fn){
	edge1 = _edge1;
	auxedge1 = _auxedge1;
}

PrintableFill::~PrintableFill() {
}

void PrintableFill::print() {
}


void PrintableFill::generateSVG(svg::Document& doc) {
}


void PrintableFill::convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled) {
	
	ThickExtrude le(edge1, auxedge1);
	hardStls = pathdir  + std::to_string(long long(this->partid)) + "_fold_hard.off";
	softStls = pathdir + std::to_string(long long(this->partid)) + "_fold_soft.off";

	//hardStls.push_back(path);
	//softStls.push_back(path1);
	le.generate_stl(hardStls,softStls, pi,isAssembled);
}
/**************** MOTOR MOUNT ******************************/

MotorMount::MotorMount(int id, Eigen::Vector2d * dir, Eigen::Vector2d * point, double displacement) : PrintablePart(id, NULL) {
	this->dir = *dir;
	this->point = *point;
	this->displacement = displacement;
}

MotorMount::~MotorMount(){

}

void MotorMount::print(){

}

void MotorMount::generateSVG(svg::Document& doc) {

}

void MotorMount::convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled){
	Eigen::Vector2d normDir = this->dir.normalized();
	Eigen::Vector2d position = this->point + this->displacement * normDir;
	Eigen::Vector2d xhat = Eigen::Vector2d(1.0, 0.0);
	double angle = acos(xhat.dot(normDir));
	HoleCreation le(&position, angle);
	hardStls = pathdir  + std::to_string(long long(this->partid)) + "_motor_mount_hard.off";
	softStls = ""; //pathdir + std::to_string(long long(this->partid)) + "_motor_mount_soft.off";
	//hardStls.push_back(path);
	le.generate_stl(hardStls,softStls, pi,isAssembled);
}
	

/*********************   FOLD     *********************************/

PrintableFold::PrintableFold(int id, double _angle, Edge2dp* _edge1, Edge2dp* _edge2, Edge2dp* _auxedge1, Edge2dp* _auxedge2, FoldableNode* fn): PrintablePart(id, fn){
	edge1 = _edge1;
	edge2 = _edge2;
	angle = _angle;
	auxedge1 = _auxedge1;
	auxedge2 = _auxedge2;
}

PrintableFold::~PrintableFold() {
}


void PrintableFold::print() {
}

//void PrintableDesign::addPrintableFold(PrintableFold* pf) {
//	printableFolds.push_back(pf);
//}

void PrintableFold::generateSVG(svg::Document& doc) {
	/*
	VLOG(3) << "Generating SVG for PrintableFold";
    // Blue
	svg::Polygon border(svg::Stroke(5, svg::Color::Blue));

	for(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>::iterator p = auxFoldFace.points.begin(); p != auxFoldFace.points.end(); ++p) {
		VLOG(3) << "Point #: " << p - auxFoldFace.points.begin() << " (" << p->x() << ", " << p->y() << ")";
		border << svg::Point(p->x(), p->y());
	}

	doc << border;
	*/
}


void PrintableFold::convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled) {

/*	drawing::Face face;
	Eigen::Vector2d p0 = edge1->v1;
	Eigen::Vector2d p1 = edge1->v2;
	Eigen::Vector2d p2 = edge2->v1;
	Eigen::Vector2d p3 = edge2->v2;
	face.points.push_back(p0);
	face.points.push_back(p1);
	face.points.push_back(p2);
	face.points.push_back(p3);
	auxFoldFace = face;
	*/
	
	FoldEdge le(angle, edge1, edge2, auxedge1, auxedge2);
	hardStls = pathdir  + std::to_string(long long(this->partid)) + "_fold_hard.off";
	softStls = ""; //pathdir + std::to_string(long long(this->partid)) + "_fold_soft.off";

	//hardStls.push_back(path);
	//softStls.push_back(path1);
	le.generate_stl(hardStls,softStls, pi,isAssembled);
}



/*********************   TEETH     *********************************/

PrintableTeeth::PrintableTeeth(int id, double _angle, Edge2dp* _edge1, Edge2dp* _edge2, Edge2dp* _auxedge1, Edge2dp* _auxedge2, FoldableNode* fn):PrintablePart(id, fn){
	edge1 = _edge1;
	edge2 = _edge2;
	auxedge1 = _auxedge1;
	auxedge2 = _auxedge2;
	angle = _angle;
}

void PrintableTeeth::convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled) {
	FoldEdge le(angle, edge1, edge2, auxedge1, auxedge2);	
	hardStls = pathdir  + std::to_string(long long(this->partid)) + "_teeth_hard.off";
	softStls = ""; //pathdir  + std::to_string(long long(this->partid)) + "_teeth_soft.off";
	//hardStls.push_back(path);
	//softStls.push_back(path1);
	le.generate_teeth_stl(hardStls, softStls, pi,isAssembled);
}

void PrintableTeeth::generateSVG(svg::Document& doc) {
	/*
	VLOG(3) << "Generating SVG for PrintableFold";
	// Blue
	svg::Polygon border(svg::Stroke(5, svg::Color::Blue));

	for (std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>::iterator p = auxFoldFace.points.begin(); p != auxFoldFace.points.end(); ++p) {
		VLOG(3) << "Point #: " << p - auxFoldFace.points.begin() << " (" << p->x() << ", " << p->y() << ")";
		border << svg::Point(p->x(), p->y());
	}

	doc << border;
	*/

}

void PrintableTeeth::print() {

}


/*********************   TOUCH    *********************************/

PrintableTouch::PrintableTouch(int id, double _angle, Edge2dp* _edge1, Edge2dp* _edge2, Edge2dp* _auxedge1, Edge2dp* _auxedge2, FoldableNode* fn) : PrintablePart(id, fn){
	edge1 = _edge1;
	edge2 = _edge2;
	auxedge1 = _auxedge1;
	auxedge2 = _auxedge2;
	angle = _angle;
}

void PrintableTouch::convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled) {
	FoldEdge le(angle, edge1, edge2, auxedge1, auxedge2);
	hardStls = pathdir + std::to_string(long long(this->partid)) + "_touch_hard.off";
	softStls = ""; //pathdir + std::to_string(long long(this->partid)) + "_touch_soft.off";
	//hardStls.push_back(path);
	le.generate_touch_stl(hardStls, softStls, pi,isAssembled);


}

void PrintableTouch::generateSVG(svg::Document& doc) {
	/*
	VLOG(3) << "Generating SVG for PrintableFold";
	// Blue
	svg::Polygon border(svg::Stroke(5, svg::Color::Blue));

	for (std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>::iterator p = auxFoldFace.points.begin(); p != auxFoldFace.points.end(); ++p) {
		VLOG(3) << "Point #: " << p - auxFoldFace.points.begin() << " (" << p->x() << ", " << p->y() << ")";
		border << svg::Point(p->x(), p->y());
	}

	doc << border;
	*/
}
void PrintableTouch::print() {

}


/*********************   HINGE     *********************************/

PrintableHinge::PrintableHinge(int id, double _angle, Edge2dp* _edge1, Edge2dp* _edge2, Edge2dp* _auxedge1, Edge2dp* _auxedge2, FoldableNode* fn) : PrintablePart(id, fn){
	edge1 = _edge1;
	edge2 = _edge2;
	angle = _angle;
	auxedge1 = _auxedge1;
	auxedge2 = _auxedge2;
}


PrintableHinge::~PrintableHinge() {
}

void PrintableHinge::print() {
}

void PrintableHinge::generateSVG(svg::Document& doc) {
	/*
	VLOG(3) << "Generating SVG for PrintableHinge";
	// Blue
	svg::Polygon border(svg::Stroke(5, svg::Color::Blue));

	for (std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>::iterator p = auxFoldFace.points.begin(); p != auxFoldFace.points.end(); ++p) {
		VLOG(3) << "Point #: " << p - auxFoldFace.points.begin() << " (" << p->x() << ", " << p->y() << ")";
		border << svg::Point(p->x(), p->y());
	}

	doc << border;
	*/
}


void PrintableHinge::convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled) {
	FoldEdge le(angle, edge1, edge2, auxedge1, auxedge2);
	hardStls = pathdir + std::to_string(long long(this->partid)) + "_hinge_hard.off";
	softStls = ""; //pathdir + std::to_string(long long(this->partid)) + "_hinge_soft.off";
	//hardStls.push_back(path);
	le.generate_hinge_stl(hardStls, softStls, pi,isAssembled);
}



/*********************  BALL JOINT     *********************************/

PrintableBallJoint::PrintableBallJoint(int id, double _angle, Edge2dp* _edge1, Edge2dp* _edge2, Edge2dp* _auxedge1, Edge2dp* _auxedge2, FoldableNode* fn) : PrintablePart(id, fn){
	edge1 = _edge1;
	edge2 = _edge2;
	angle = _angle;
	auxedge1 = _auxedge1;
	auxedge2 = _auxedge2;
}


PrintableBallJoint::~PrintableBallJoint() {
}

void PrintableBallJoint::print() {
}

void PrintableBallJoint::generateSVG(svg::Document& doc) {
}


void PrintableBallJoint::convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled) {
	FoldEdge le(angle, edge1, edge2, auxedge1, auxedge2);
	hardStls = pathdir + std::to_string(long long(this->partid)) + "_joint_hard.off";
	softStls = ""; //pathdir + std::to_string(long long(this->partid)) + "_joint_soft.off";
	//hardStls.push_back(path);
	le.generate_balljoint_stl(hardStls, softStls, pi,isAssembled);
}


/*********************   PRISMATIC     *********************************/

PrintablePrismatic::PrintablePrismatic(int id, double _angle, Edge2dp* _edge1, Edge2dp* _edge2, Edge2dp* _auxedge1, Edge2dp* _auxedge2, FoldableNode* fn) : PrintablePart(id, fn){
	edge1 = _edge1;
	edge2 = _edge2;
	angle = _angle;
	auxedge1 = _auxedge1;
	auxedge2 = _auxedge2;
}


PrintablePrismatic::~PrintablePrismatic() {
}

void PrintablePrismatic::print() {
}

void PrintablePrismatic::generateSVG(svg::Document& doc) {
}


void PrintablePrismatic::convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled) {
	FoldEdge le(angle, edge1, edge2, auxedge1, auxedge2);
	hardStls = pathdir + std::to_string(long long(this->partid)) + "_prismatic_hard.off";
	softStls = ""; //pathdir + std::to_string(long long(this->partid)) + "_prismatic_soft.off";
	//hardStls.push_back(path);
	le.generate_prismatic_stl(hardStls, softStls, pi,isAssembled);
}


/***********************     FACE    **********************************/

void PrintableFace::print() {
	VLOG(3) << "Hello Print from Face";

	VLOG(3) << "NEW vertices:::::::::::::::::::::: ";
	drawing::Point::point_vector_type::iterator it;
	int count = 0;
	for(it = face.idpoints.begin(); it!= face.idpoints.end(); ++it){
		VLOG(3) << count << ": id: " << it->id << "pos: " << it->p.transpose();
		//VLOG(3) << count << ": id: " << it->id << "edge: " << face.edgeMap[it->id];
		++count;
	}
	//for(int i=face.getMinID();i<=face.getMaxID(); i++) {
	//	VLOG(3) << i<< ": " << face.getPointFromId(i)->p.transpose();
	//}
	//system("pause");
}
PrintableFace::PrintableFace(drawing::Face f, int id, TriMesh mesh, std::vector<MotorMountLocations *> mountPatches, std::vector<Eigen::Vector2d *> servoPatches, FoldableNode* te): PrintablePart(id, te) {
	face = f;
	triMesh = mesh;
	this->mountPatches = mountPatches;
	this->servoPatches = servoPatches;
}


void PrintableFace::generateSVG(svg::Document& doc) {
/*	VLOG(3) << "Generating SVG for PrintableFace";
    // Red image border.
	svg::Polygon border(svg::Stroke(5, svg::Color::Green));

	for(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>::iterator p = face.points.begin(); p != face.points.end(); ++p) {
		VLOG(3) << "Point #: " << p - face.points.begin() << " (" << p->x() << ", " << p->y() << ")";
		border << svg::Point(p->x(), p->y());
	}

	doc << border;
	*/
}


Edge2dp * PrintableFace::getEdge(int edgeID){

//	std::cout << "I am looking at a face id " << this->getId() << std::endl;
	drawing::Edge edge = face.edges[edgeID]; 
	Edge2dp * res  = new Edge2dp();
	res->v1= face.getPointFromId(edge.vertice1_id)->p;
	res->v2= face.getPointFromId(edge.vertice2_id)->p;
	res->offset = (edge.trimBack - edge.trimFront)/2;

//	std::cout << "res->v1 = " << res->v1.transpose() << std::endl;
//	std::cout << "res->v2 = " << res->v2.transpose() << std::endl;
//	system("pause"); 
	return res;
}

void PrintableFace::shrinkFace(int edgeID, double amount) {

	Eigen::Vector2d direction = find_direction(face, edgeID);

	//Eigen::Vector2d direction = find_normal(points[vertice_id1] - pointsvertice_id2);
	//VLOG(3) << "Direction: " << " (" << direction.x() << ", " << direction.y() << ")";
	reduce_pol(direction,  amount , edgeID, face);
	
	//VLOG(3) << "Reducing Face";
	//for(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>::iterator p = face.points.begin(); p != face.points.end(); ++p) {
	//	VLOG(3) << "Point #: " << p - face.points.begin() << " (" << p->x() << ", " << p->y() << ")";
	//}
	
	
}	

Edge2dp* PrintableFace::getUnshrinkFace(int edgeID, double amount) {

	Eigen::Vector2d direction = find_direction(face, edgeID);
	return getreduce_pol(-1*direction,  amount , edgeID, face);
	
	
}
PrintableFace::~PrintableFace() {
}

void PrintableFace::convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled) {
	if (this->face.idpoints.size() > 0) {
		LinearExtrude le(this->face, this->mountPatches, this->servoPatches);
		hardStls = pathdir  + std::to_string(long long(this->partid)) + "face_hard.off";
		le.generate_stl(hardStls,softStls, pi,isAssembled);
	} else {
		hardStls = "";
	}
	softStls = ""; //pathdir + std::to_string(long long(this->partid)) + "face_soft.off";
	//hardStls.push_back(path); 

}


void PrintableFace::shrinkAllVertices(double delta) {

	drawing::Point::point_vector_type::iterator vertex;
	Eigen::Vector2d dir1; 
	Eigen::Vector2d dir2; 
	VLOG(3) << "new face";
	//system("pause"); 
	for(vertex = face.idpoints.begin(); vertex != face.idpoints.end(); vertex++){
		drawing::Point::point_vector_type::iterator vertex_prev = find_predecessor(dir1, vertex, face );
		drawing::Point::point_vector_type::iterator vertex_next = find_sucessor(dir2, vertex, face );

		dir1 = PrintingParameters::getOrientInd()*( vertex->p - vertex_prev->p);
		dir1 = Eigen::Vector2d(-dir1.y(), dir1.x());
		dir1.normalize();
	
		dir2 = PrintingParameters::getOrientInd()*( vertex_next->p - vertex->p);
		dir2 = Eigen::Vector2d(-dir2.y(), dir2.x());
		dir2.normalize();

		//VLOG(3) << "dir1 = " << dir1;
		//VLOG(3) << "dir2 = " << dir2;
		//VLOG(3) << "abs((vertex_next->p-vertex->p).normalized().dot(dir1))" << abs((vertex_next->p-vertex->p).normalized().dot(dir1)) <<std::endl;
		Eigen::Vector2d change1;
		if(abs((vertex_next->p-vertex->p).normalized().dot(dir1)) < 0.05){ 
			change1 =  dir1*delta;
		}else{
			change1 = reduce_p (dir1, delta, vertex->p, vertex_next->p)- vertex->p;
		}

		Eigen::Vector2d change2;
		if(abs((vertex_prev->p-vertex->p).normalized().dot(dir2)) < 0.05){ 
			change2 = dir2*delta;
		}else{
			change2 = reduce_p (dir2, delta, vertex->p, vertex_prev->p)- vertex->p;
		}

		//VLOG(3) << "change1 = " << change1;
		//VLOG(3) << "change2 = " << change2;
			
//			= reduce_p (dir1, delta, vertex->p, vertex_next->p) - vertex->p;
//		Eigen::Vector2d change2 = reduce_p (dir2, delta, vertex->p, vertex_prev->p) - vertex->p;

		Eigen::Vector2d totchange = change1;
		if(abs((dir1.dot(dir2) -1)) > 0.001){
			totchange = totchange + change2;
		}

		Eigen::Vector2d newp = vertex->p + totchange;

		// store the changes in length
		face.edges[face.edgeMap[vertex->id]].trimFront += totchange.dot((vertex_next->p-vertex->p).normalized());
		face.edges[face.edgeMap[vertex_prev->id]].trimBack += totchange.dot((vertex_prev->p-vertex->p).normalized());

		VLOG(3) << "old vertex = " << vertex->p.transpose();
		VLOG(3) << "new vertex = " << newp.transpose();
		VLOG(3) << "trim = " << face.edges[face.edgeMap[vertex->id]].trimFront << "back " << face.edges[face.edgeMap[vertex_prev->id]].trimBack;
		//system("pause"); 
		vertex->p = newp; 


	}
	//system("pause");
}

void PrintableFace::simplify() {
	drawing::Point::point_vector_type::iterator vertex = face.idpoints.begin();
	drawing::Point::point_vector_type::iterator vertex_next;
	Eigen::Vector2d dir; 
	std::cout << "simplifying face" << partid << std::endl;

	while (vertex != face.idpoints.end()) {
		vertex_next = find_sucessor(dir, vertex, face );

		if ((vertex->p - vertex_next->p).norm() < 0.001) {
			std::cout << "deleting vertex " << vertex_next->id << ": " << vertex_next->p.transpose() << std::endl;
			std::cout << "is the same as vertex " << vertex->id << ": " << vertex->p.transpose() << std::endl;
	//system("pause");

			// vertices are the same

			// change edge information
			face.edges[face.edgeMap[vertex_next->id]].vertice1_id = vertex->id;
			face.edgeMap[vertex->id] = face.edges[face.edgeMap[vertex_next->id]].id;
			face.edgeMap.erase(vertex_next->id);

			// delete the vertex
			face.idpoints.erase(vertex_next);

			vertex_next = find_sucessor(dir, vertex, face );
		} else {
			vertex = vertex_next;
		}

		if (vertex_next == face.idpoints.begin()) {
			break;
		}
	}

	/*
	// TODO: This can probably be combined with above loop
	// loop around for last vertex 
	vertex = face.idpoints.begin();
	drawing::Point::point_vector_type::iterator vertex_prev = find_predecessor(dir, vertex, face );
	drawing::Point::point_vector_type::iterator vertex_prevprev = find_predecessor(dir, vertex_prev, face );
	if ((vertex->p - vertex_prev->p).norm() < 0.001) {
		// vertices are the same
		std::cout << "deleting vertex " << vertex_prev->id << ": " << vertex_prev->p.transpose() << std::endl;
		std::cout << "is the same as vertex " << vertex->id << ": " << vertex->p.transpose() << std::endl;
			
		// change edge information
		face.edges[face.edgeMap[vertex_prevprev->id]].vertice2_id = vertex->id;
		face.edgeMap.erase(vertex_prev->id);

		// delete the last vertex
		face.idpoints.erase(vertex_prev);
	}
	*/

}


/***********************     MESH    **********************************/

void PrintableMesh::print() {
	VLOG(3) << "This is a mesh ";

}
PrintableMesh::PrintableMesh(int id, TriMesh mesh, FoldableNode* te): PrintablePart(id, te) {
	triMesh = mesh;
}


void PrintableMesh::generateSVG(svg::Document& doc) {
}



PrintableMesh::~PrintableMesh() {
}

void PrintableMesh::convertToMesh(std::string & hardStls, std::string & softStls,PROCESS_INFORMATION &pi, bool isAssembled) {
	//ANDYTODO: change this to create the contact point

	hardStls = ""; //pathdir  + std::to_string(long long(this->partid)) + "face_hard.off";
	softStls = pathdir  + std::to_string(long long(this->partid)) + "face_soft.off";
	//hardStls.push_back(path); 
	//softStls.push_back(path1); 
	triMesh.write(softStls.c_str());

}


void PrintableMesh::shrinkAllVertices(double delta) {


}

void PrintableMesh::simplify() {

}
