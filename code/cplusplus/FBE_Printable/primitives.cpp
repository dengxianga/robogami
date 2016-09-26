#include "primitives.h"
#include <shlobj.h>
#include <iostream>
#include <vector>
#include <string>
#include <windows.h>
#include<Eigen/StdVector>
#include <Eigen/Dense>
#include "drawing.h"
#include "PrintingParameters.h"
#include "FoldableGraph.h"
using namespace std;

namespace FabByExample {
#define PI 3.14159265
PrintingParameters para;

#define DEBUG false

namespace {
	void openscad_generate_stl(string input, string output, string constants,PROCESS_INFORMATION &pi) {

		ProgressTracking::SetProgress("Generating " + output + " from " + input + "...");
		std::string delimiter = ";";
		vector<string> const_arr;

		size_t pos = 0;
		std::string token;
		while ((pos = constants.find(delimiter)) != std::string::npos) {
			token = constants.substr(0, pos);
			const_arr.push_back(token);
			constants.erase(0, pos + delimiter.length());
		}

		
		// std::string args = "bscad.bat \"-o "+output+" -D "+constants+" "+input + "\"";

		std::string args = "openscad.exe -o " + output;

		for (int i = 0; i < const_arr.size(); i++) {
			args += " -D " + const_arr[i];
		}

		args += " FBE_Printable\\" + input;

		if (DEBUG) {
			LOG(ERROR) << "Executing: " << args;
		}
		LPSTR sargs = const_cast<char *>(args.c_str());

		

		//VLOG(3) << "Pos: " << position;

		STARTUPINFOA si;
		si.cb = sizeof(si);
		si.dwFlags = STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
		si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
		si.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
		si.hStdError = GetStdHandle(STD_ERROR_HANDLE);
		si.wShowWindow = SW_HIDE;  // No black window
		//PROCESS_INFORMATION pi;
		
		ZeroMemory( &si, sizeof(si) );
		si.cb = sizeof(si);
		ZeroMemory( &pi, sizeof(pi) );

		//VLOG(3) << args;
		// Start the child process. 
		if( !CreateProcessA( NULL,   // No module name (use command line)
			//"bscad.bat \"cube.scad -o cube.stl -D abc=[2,3,4]\"",        // Command line
			sargs,
			NULL,           // Process handle not inheritable
			NULL,           // Thread handle not inheritable
			FALSE,          // Set handle inheritance to FALSE
			CREATE_NO_WINDOW,              // No creation flags
			NULL,           // Use parent's environment block
			NULL,           // Use parent's starting directory 
			&si,            // Pointer to STARTUPINFO structure
			&pi )           // Pointer to PROCESS_INFORMATION structure
		) 
		{
			LOG(FATAL) << "CreateProcess failed (%d).\n" << GetLastError();
		    return;
		}
		
		// Wait until child process exits.
		//WaitForSingleObject( pi.hProcess, INFINITE );

		// Close process and thread handles. 
		//CloseHandle( pi.hProcess );
		//CloseHandle( pi.hThread );

		ProgressTracking::SetProgress("");

		//HINSTANCE rval = ShellExecute(NULL, "open", "bscad.bat", "'-o cube.stl cube.scad'", "C:\Users\dutra\workspace\FabByExample\code\cplusplus\FBE_Printable", 1);
		//system("openscad -o cube.stl cube.scad");
		//VLOG(3) << "Return value: " << rval;
}

}

// ----------------------------------------------------------------------------------------------------
// LINEAR EXTRUDE
// ----------------------------------------------------------------------------------------------------
LinearExtrude::LinearExtrude(drawing::Face face, std::vector<MotorMountLocations *> mounts, std::vector<Eigen::Vector2d *> servoMounts) {
	vertices.clear();
	drawing::Point::point_vector_type::iterator it;
	
	for(it = face.idpoints.begin(); it!= face.idpoints.end(); ++it){
		vertices.push_back(it->p);
	}
	this->mounts = mounts;
	this->servoMounts = servoMounts;
	/*
	for(int i=face.getMinID();i<=face.getMaxID(); i++) {
		vertices.push_back(face.getPointFromId(i)->p);
	}
	*/
}
LinearExtrude::~LinearExtrude() {
}

void LinearExtrude::unrepeatedVertices(std::vector<int> &indices){
	/*
	if( vertices.size() != 4){
		VLOG(3) << "vertices: ";
		for (int i = 0; i < vertices.size(); i++){
			VLOG(3) << vertices[i].transpose();
		}
		//system("pause");
	}
	*/

//	indices.push_back(0);
//	for (int i = 1; i < vertices.size(); i++){
//		int previ = i-1;
//		int posi = i+1;
//		if(posi == vertices.size())
//			posi = 0;
//		Eigen::Vector2d dir1 = (vertices[posi] - vertices[previ]).normalized();
//		Eigen::Vector2d dir2 = (vertices[i] - vertices[previ]).normalized();
//		if(abs(dir1.dot(dir2) -1) > 0.00001){
//			indices.push_back(i);
//		}else{
//			VLOG(3) <<"removing vertex = " << i;
//		}
//	}
	for (int i = 0; i < vertices.size(); i++){

		//if(vertices[i].norm() <500){
			indices.push_back(i);
		/*}else{
			VLOG(3) << "removing vertex = " << i;
			//system("pause");
		}*/
	}
	

	

}
void LinearExtrude::generate_stl(std::string path,std::string path1,PROCESS_INFORMATION &pi, bool isAssembled) {
	VLOG(3) << "Generating LinearExtrude STL: " << path;
	VLOG(3) << "Generating LinearExtrude STL: " << path1;
	std::ostringstream args;

	args << "resolution=" << PrintingParameters::getResolution() << ";";

	args << "wallThickness=" << PrintingParameters::getWallThickness() << ";";
	args << "isAssembled=";
	if (isAssembled) {
		args << "true;";
	} else {
		args << "false;";
	}
	args << "points=[";	
	std::vector<int> indices;
	unrepeatedVertices(indices);
	for(auto p = indices.begin(); p != indices.end(); ++p) {
		string constants;
		double x = vertices[*p].x();
		double y = vertices[*p].y();
		//VLOG(3) << "(" << x << "," << y << ")";
		if(p+1==indices.end())
			args << "[" << x << "," << y << "]";
		else
			args << "[" << x << "," << y << "],";
	}
	args << "];";
	args << "motorMountLocs=[";
	bool remove = false;
	for each (MotorMountLocations * mml in this->mounts){
		Eigen::Vector2d point1 = mml->point1;
		Eigen::Vector2d dir = mml->dir.normalized();
		Eigen::Vector2d xhat = Eigen::Vector2d(1.0, 0.0);
		double angle = acos(xhat.dot(dir)) * 180.0/PI;
		for each (double spacing in mml->spacings){
			remove = true;
			Eigen::Vector2d point = point1 - dir*spacing;
			args << "[" << point(0) << "," << point(1) << "," << angle << "],";
		}
	}

	if (remove){
		args.seekp(-1,args.cur);
	}
	args << "];";

	remove = false;
	args << "servoMountLocs=[";
	for each (Eigen::Vector2d * mountPoses in this->servoMounts){
		double angle = 0.0;
		args << "[" << (*mountPoses)(0) << "," << (*mountPoses)(1) << "," << angle << "],";
		remove = true;
	}

	if (remove){
		args.seekp(-1,args.cur);
	}


	args << "];";

	//std::cout << args.str() << std::endl;
	openscad_generate_stl("polygon_motors.scad", path, args.str(), pi);
}
// THICK EXTRUDE ----------------------------


// ----------------------------------------------------------------------------------------------------
// LINEAR EXTRUDE
// ----------------------------------------------------------------------------------------------------
ThickExtrude::ThickExtrude(Edge2dp*  _e1, Edge2dp* _e2) {
	vertices.clear();
	
	vertices.push_back(_e1->v1);
	vertices.push_back(_e1->v2);
	vertices.push_back(_e2->v2);
	vertices.push_back(_e2->v1);
}

ThickExtrude::~ThickExtrude() {
}

void ThickExtrude::unrepeatedVertices(std::vector<int> &indices){
	for (int i = 0; i < vertices.size(); i++){

			indices.push_back(i);
	}
	

}
void ThickExtrude::generate_stl(std::string path,std::string path1,PROCESS_INFORMATION &pi, bool isAssembled) {
	VLOG(3) << "Generating LinearExtrude STL: " << path;
	VLOG(3) << "Generating LinearExtrude STL: " << path1;
	std::ostringstream args;

	args << "resolution=" << PrintingParameters::getResolution() << ";";

	args << "wallThickness=" << PrintingParameters::getWallThickness() << ";";
	args << "points=[";	
	std::vector<int> indices;
	unrepeatedVertices(indices);
	for(auto p = indices.begin(); p != indices.end(); ++p) {
		string constants;
		double x = vertices[*p].x();
		double y = vertices[*p].y();
		//VLOG(3) << "(" << x << "," << y << ")";
		if(p+1==indices.end())
			args << "[" << x << "," << y << "]";
		else
			args << "[" << x << "," << y << "],";
	}
	args << "];";

	args << "isAssembled=";
	if (isAssembled) {
		args << "true;";
	} else {
		args << "false;";
	}
	
	openscad_generate_stl("polygon_thick.scad", path, args.str(), pi);
}
//----------------------------------------------------------------------

Cube::Cube(Eigen::Vector3d pos, Eigen::Vector3d dim) {
	position = pos;
	dimensions = dim;

}
Cube::~Cube() {
}

void Cube::generate_stl(std::string path,PROCESS_INFORMATION &pi) {
	openscad_generate_stl("cube.scad", "cube.stl", "size=[2,3,4]", pi);
}

FoldEdge::FoldEdge(double _angle, Edge2dp*  _e1, Edge2dp* _e2){
	this->e1 = _e1;
	this->e2 = _e2;
	angle = _angle;
}

FoldEdge::FoldEdge(double _angle, Edge2dp*  _e1, Edge2dp* _e2, Edge2dp*  _auxe1, Edge2dp* _auxe2){
	this->e1 = _e1;
	this->e2 = _e2;
	this->auxe1 = _auxe1;
	this->auxe2 = _auxe2;
	angle = _angle;
}

FoldEdge::~FoldEdge(){
	delete e1;
	delete e2;
}

double cropSides (Edge2dp * e1, Edge2dp * e2, Eigen::Vector2d & center,
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> & triangle1,
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> & triangle2){

	//VLOG(3) << "croping sides ";
	//VLOG(3) << e1->v1.transpose() << " -- " << e1->v2.transpose();
	//VLOG(3) << e2->v1.transpose() << " -- " << e2->v2.transpose();

	Eigen::Vector2d a = e1->v1;
	Eigen::Vector2d b = e1->v2;
	Eigen::Vector2d c = e2->v2;
	Eigen::Vector2d d = e2->v1;

	//compute ther triangles

	Eigen::Vector2d dir1 = a - d;
	dir1.normalize();
	Eigen::Vector2d ap = a + dir1*0.1;
	Eigen::Vector2d dp = d - dir1*0.1;

	Eigen::Vector2d dir2 = b - c;
	dir2.normalize();
	Eigen::Vector2d bp = b + dir2*0.1;
	Eigen::Vector2d cp = c - dir2*0.1;

	center = Eigen::Vector2d::Zero();

	Eigen::Vector2d dir = b - a; dir.normalize();
	Eigen::Vector2d c1;
	if((d-a).dot(dir) < 0){
		c1 =  ap + ((dp-ap).dot(dir))*dir;
	//	center = center + a + ((d-a).dot(dir))*dir;
	}
	else{
		c1 =  dp - ((dp-ap).dot(dir))*dir;
	}
	center = center + a;
	Eigen::Vector2d c2;
	if((c-b).dot(dir) > 0){
		c2 =  bp + ((cp-bp).dot(dir))*dir;
	//	center = center +  b + ((c-b).dot(dir))*dir;
	}
	else{
		c2 =  cp - ((cp-bp).dot(dir))*dir;
	}
	center = center + b;
	
	triangle1[0] = ap;
	triangle1[1] = dp;
	triangle1[2] = c1;
	triangle2[0] = cp;
	triangle2[1] = bp;
	triangle2[2] = c2;

	//VLOG(3) << "tri1 = ";
	//VLOG(3) << triangle1[0].transpose();
	//VLOG(3) << triangle1[1].transpose();
	//VLOG(3) << triangle1[2].transpose();
	//VLOG(3) << "tri2 = ";
	//VLOG(3) << triangle2[0].transpose();
	//VLOG(3) << triangle2[1].transpose();
	//VLOG(3) << triangle2[2].transpose();

 
	//center = a + (center -a).dot(dir)*dir; 
	center = center*0.5;
	//VLOG(3) << "center = " << center.transpose();
	double length = (abs((c1 - c2).dot(dir)));
	//VLOG(3) << "length = " << length;	
	length = 0.999*length; 
	return length;
}

void FoldEdge::get_stl_edge_args(std::ostringstream& args, bool isAssembled, bool checkisfold /*=false*/) {
	bool fold = PrintingParameters::useMultiMaterialFold(); 

	// taken from FoldEdge::generate_stl
	Eigen::Vector2d a = this->e1->v2;
	Eigen::Vector2d b = this->e1->v1;
	Eigen::Vector2d c = this->e2->v2;
	Eigen::Vector2d d = this->e2->v1;

	// rotation
	Eigen::Vector3d rot1, rot2;
	Eigen::Vector2d dir1 = PrintingParameters::getOrientInd()*(a - b) ;
	dir1.normalize();
	double rotAngle1 = std::atan2(dir1[1],dir1[0]) * 180 / PI;
	rot1.x() = 0;
	rot1.y() = 0;
	rot1.z() = rotAngle1;
	
	Eigen::Vector2d dir2 = PrintingParameters::getOrientInd()*(d - c) ;
	dir2.normalize();
	double rotAngle2 = std::atan2(dir2[1],dir2[0]) * 180 / PI;
	rot2.x() = 0;
	rot2.y() = 0;
	rot2.z() = rotAngle2;

	// triangles
	Eigen::Vector2d center1;
	Eigen::Vector2d center2;
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> triangle1A(3);
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> triangle1B(3);
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> triangle2A(3);
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> triangle2B(3);

	double L1 =  cropSides(this->e1, this->auxe1, center1, triangle1A, triangle1B);
	double L2 =  cropSides(this->e2, this->auxe2, center2, triangle2A, triangle2B);
	
	Eigen::Vector3d trans1, trans2;
	trans1.x() = center1.x();
	trans1.y() = center1.y();
	trans2.x() = center2.x();
	trans2.y() = center2.y();
	trans2.z() = trans1.z() = 0;
	
	//VLOG(3) << "transR = " << trans1.transpose();
	//VLOG(3) << "transL = " << trans2.transpose();
	//system("pause");
	
	// Center the edges
	double L = min(L1, L2);
	// get the offset
	double delta1 = (e1->offset) + (e2->offset);
	double delta2 = 0;
	//VLOG(3) << "delta = " << (e1->offset) << "+" << (e2->offset) << "="<< delta1;

	// constants
	args << "isAssembled=";
	if (isAssembled) {
		args << "true;";
	} else {
		args << "false;";
	}
	args << "r1=" << PrintingParameters::getHingeR1() << ";";
	args << "mingap=" << PrintingParameters::getMinGap() << ";";
	args << "resolution=" << PrintingParameters::getResolution() << ";";
	args << "wallThickness=" << PrintingParameters::getWallThickness() << ";";

	// angles
	args << "inangle=" << this->angle << ";";
	args << "rot1=[" << rot1[0] << "," << rot1[1] << "," << rot1[2] << "];";
	args << "rot2=[" << rot2[0] << "," << rot2[1] << "," << rot2[2] << "];";

	// centers
	args << "transR=[" << trans1[0] << "," << trans1[1] << "," << trans1[2] << "];";
	args << "transL=[" << trans2[0] << "," << trans2[1] << "," << trans2[2] << "];";

	args << "length=" << L << ";";
	args << "length1=" << L1 << ";";
	args << "length2=" << L2 << ";";
	args << "delta1=" << delta1 << ";";
	args << "delta2=" << delta2 << ";";

	// boundaries
	args << "points1=[";	
	for(auto p = triangle1A.begin(); p != triangle1A.end(); ++p) {
		args << "[" << p->x() << "," << p->y() << "]";
		if(p+1!=triangle1A.end())
			args << ",";
	}
	args << "];";
	args << "points2=[";	
	for(auto p = triangle1B.begin(); p != triangle1B.end(); ++p) {
		args << "[" << p->x() << "," << p->y() << "]";
		if(p+1!=triangle1B.end())
			args << ",";
	}
	args << "];";
	args << "points3=[";	
	for(auto p = triangle2A.begin(); p != triangle2A.end(); ++p) {
		args << "[" << p->x() << "," << p->y() << "]";
		if(p+1!=triangle2A.end())
			args << ",";
	}
	args << "];";
	args << "points4=[";	
	for(auto p = triangle2B.begin(); p != triangle2B.end(); ++p) {
		args << "[" << p->x() << "," << p->y() << "]";
		if(p+1!=triangle2B.end())
			args << ",";
	}
	args << "];";

}

void HoleCreation::generate_stl(std::string path, std::string path1, PROCESS_INFORMATION &pi, bool isAssembled){
	std::string scadfile = "stencil.scad";
	std::ostringstream args;

	double angleDegrees = angle * 180.0/PI;

	args << "x=" << this->location.x() << ";";
	args << "y=" << this->location.y() << ";";
	args << "angle=" << angleDegrees << ";";
	std::cout << args.str() <<std::endl;
	openscad_generate_stl(scadfile, path, args.str(), pi);
}

HoleCreation::HoleCreation(Eigen::Vector2d * location, double angle){
	this->location = *location;
	this->angle = angle;
}

HoleCreation::~HoleCreation(){

}

void FoldEdge::generate_stl(std::string path,std::string path1,PROCESS_INFORMATION &pi, bool isAssembled) {
	bool fold = PrintingParameters::useMultiMaterialFold(); 

	VLOG(3) << "Generating EdgeFold STL: " << path;
	VLOG(3) << "Generating EdgeFold STL: " << path1;
	
	std::ostringstream args;
	this->get_stl_edge_args(args, isAssembled, true);
	args << "foldZOffset="<< -(0.68065)*PrintingParameters::getWallThickness() <<";";
	args << "r1s=" << FoldableEdge::getTotalShrinkAmount(FoldableEdge::FoldableEdgeType::FOLD, this->angle) << ";";
	
	// divisions
	//int chunks = std::floor(length/5.0);
	//if(chunks < 5){
	//	chunks = 5;
	//}
	//args << "chunks=" <<chunks << ";";

	std::string scadFile;
	
	if(fold){
		// NOTE: update these for 2-sided arguments
		openscad_generate_stl("joint.scad", path1, args.str(),pi);
		// Wait until child process exits.
		WaitForSingleObject( pi.hProcess, INFINITE );

		// Close process and thread handles. 
		CloseHandle( pi.hProcess );
		CloseHandle( pi.hThread );
		
		openscad_generate_stl("slantedEdge.scad", path, args.str(),pi);
	}else{
		if (abs(this->angle) < 0.01) {
			std::string scadfile;
			if(PrintingParameters::isPrint()){
				scadfile = "mergeface.scad";
			}else{
				scadfile = "mergeface.scad";
			}
			openscad_generate_stl(scadfile, path, args.str(),pi);
		} else {
			args << "r1=" << 1.5 << ";";

			std::string scadfile;
			if(PrintingParameters::isPrint()){
				scadfile = "hinge.scad";
			}else{
				scadfile = "hinge_lowRes.scad";
			}
			openscad_generate_stl(scadfile, path, args.str(),pi);
		}
	}

}


void FoldEdge::generate_teeth_stl(std::string path,std::string path1, PROCESS_INFORMATION &pi, bool isAssembled) {
	
	VLOG(3) << "Generating Teeth STL: " << path;
	VLOG(3) << "Generating Teeth STL: " << path1;
	
	std::ostringstream args;
	this->get_stl_edge_args(args, isAssembled);
	
	VLOG(3) << "args passed in: " << args;
	std::string scadFile;

	if(abs(this->angle -3.14) < 0.1){
		args << "wallThickness=" << PrintingParameters::getWallThickness() << ";";
		if(PrintingParameters::isPrint()){
			scadFile = "teeth180_dov.scad";
		}else{
			scadFile = "teeth180_dov_lowRes.scad";
		}
	}
	else{
		args << "wallThickness=" << PrintingParameters::getTeethMulti()*PrintingParameters::getWallThickness() << ";";

		if(PrintingParameters::isPrint()){
			scadFile = "teeth_small_dov.scad";
		}else{
			scadFile = "teeth_small_dov.scad";
		}
	}
	openscad_generate_stl(scadFile, path, args.str(),pi);
}


void FoldEdge::generate_touch_stl(std::string path,std::string path1,PROCESS_INFORMATION &pi, bool isAssembled) {

	VLOG(3) << "Generating Teeth STL: " << path;
	VLOG(3) << "Generating Teeth STL: " << path1;
	
	std::ostringstream args;
	this->get_stl_edge_args(args, isAssembled);
	args << "wallThickness=" << 1.5 << ";";
	args << "r1s=" << FoldableEdge::getTotalShrinkAmount(FoldableEdge::FoldableEdgeType::TOUCH, this->angle) << ";";
	
	VLOG(3) << "args passed in: " << args;
	openscad_generate_stl("touch.scad", path, args.str(),pi);	
}


void FoldEdge::generate_hinge_stl(std::string path, std::string path1,PROCESS_INFORMATION &pi, bool isAssembled){
	VLOG(3) << "Generating EdgeHinge STL: " << path;
	VLOG(3) << "Generating EdgeHinge STL: " << path1;
	
	std::ostringstream args;
	this->get_stl_edge_args(args, isAssembled);
	args << "r1s=" << FoldableEdge::getTotalShrinkAmount(FoldableEdge::FoldableEdgeType::HINGE, this->angle) << ";";
	//args << "delta1=" << 0<<";"<<std::endl;


	//int chunks = std::floor(length);
	//if(chunks > 3){
	//	chunks = 3;
	//}
	//args << "chunks=" <<chunks << ";";
	//system("pause");


	VLOG(3) << "args passed in: " << args;
	std::string scadfile;
	//if(PrintingParameters::isPrint()){
	//	scadfile = "hinge_big_lowRes.scad"; //Remove this hardcode
	//}else{
	//	scadfile = "hinge_big_lowRes.scad";
	//}

	// NEW FILES
	if(PrintingParameters::isPrint()){
		scadfile = "hinge_big_snap.scad";
	}else{
		scadfile = "hinge_big_snap.scad";
	}
	openscad_generate_stl(scadfile, path, args.str(),pi);
}

void FoldEdge::generate_balljoint_stl(std::string path, std::string path1,PROCESS_INFORMATION &pi, bool isAssembled){
	VLOG(3) << "Generating BallJoint STL: " << path;
	VLOG(3) << "Generating BallJoint STL: " << path1;
	
	std::ostringstream args;
	this->get_stl_edge_args(args, isAssembled);
	args << "r1s=" << FoldableEdge::getTotalShrinkAmount(FoldableEdge::FoldableEdgeType::BALLJOINT, this->angle) << ";";

	//int chunks = std::floor(length);
	//if(chunks > 3){
	//	chunks = 3;
	//}
	VLOG(3) << "args passed in: " << args;

	std::string scadfile;
	if(PrintingParameters::isPrint()){
		scadfile = "ballJoint_angled.scad";
	}else{
		scadfile = "ballJoint_angled.scad";
	}
	openscad_generate_stl(scadfile, path, args.str(),pi);
}

void FoldEdge::generate_prismatic_stl(std::string path, std::string path1,PROCESS_INFORMATION &pi, bool isAssembled){
	VLOG(3) << "Generating EdgePrismatic STL: " << path;
	VLOG(3) << "Generating EdgePrismatic STL: " << path1;
	
	std::ostringstream args;
	this->get_stl_edge_args(args, isAssembled);
	args << "r1s=" << FoldableEdge::getTotalShrinkAmount(FoldableEdge::FoldableEdgeType::PRISMATIC, this->angle) << ";";
	VLOG(3) << "args passed in: " << args;

	std::string scadfile;
	if(PrintingParameters::isPrint()){
		scadfile = "prismatic2.scad";
	}else{
		scadfile = "prismatic2.scad";
	}
	openscad_generate_stl(scadfile, path, args.str(),pi);
}

}