#pragma once

#ifndef _PRIMITIVES_H
#define _PRIMITIVES_H

#include <Eigen/Dense>
#include "drawing.h"
#include <vector>
#include <string>
#include "Printable.h"
#include "PrintingParameters.h"

namespace FabByExample {
	struct MotorMountLocations; //TODO: forward declaration okay?

class PrimitiveNode {
public:
	
	virtual void generate_stl(std::string path,std::string path1,PROCESS_INFORMATION &pi, bool isAssembled) =0;
};

class Cube : PrimitiveNode {
public:
	Cube(Eigen::Vector3d pos = Eigen::Vector3d(0,0,0), Eigen::Vector3d dim = Eigen::Vector3d(1,1,1)	);
	~Cube();

	void generate_stl(std::string path,PROCESS_INFORMATION &pi);

private:
	Eigen::Vector3d position;
	Eigen::Vector3d dimensions;

};	

class LinearExtrude : PrimitiveNode {
public:
	LinearExtrude(drawing::Face face, std::vector<MotorMountLocations *> mounts, std::vector<Eigen::Vector2d *> servoMounts);
	~LinearExtrude();

	void generate_stl(std::string path, std::string path1,PROCESS_INFORMATION &pi, bool isAssembled);
	void unrepeatedVertices(std::vector<int> &indices);
private:
	Eigen::Vector3d position;
	std::vector<Eigen::Vector2d *> servoMounts;
	std::vector<MotorMountLocations *> mounts;
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> vertices;

};

class HoleCreation : PrimitiveNode {
public:
	HoleCreation(Eigen::Vector2d * location, double angle);
	~HoleCreation();
	void generate_stl(std::string path, std::string path1,PROCESS_INFORMATION &pi, bool isAssembled);
private:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector2d location;
	double angle;
};


class ThickExtrude : PrimitiveNode {
public:
	ThickExtrude(Edge2dp*  _e1, Edge2dp* _e2);
	~ThickExtrude();

	void generate_stl(std::string path, std::string path1,PROCESS_INFORMATION &pi, bool isAssembled);
	void unrepeatedVertices(std::vector<int> &indices);
private:
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> vertices;

};


class FoldEdge : PrimitiveNode {
public:
	FoldEdge(double angle, Edge2dp*  _e1, Edge2dp* _e2);
	FoldEdge(double angle, Edge2dp*  _e1, Edge2dp* _e2, Edge2dp*  _auxe1, Edge2dp* _auxe2);
	~FoldEdge();
	void generate_stl(std::string path, std::string path1,PROCESS_INFORMATION &pi, bool isAssembled = false);
	void generate_teeth_stl(std::string path, std::string path1,PROCESS_INFORMATION &pi, bool isAssembled = false);
	void generate_touch_stl(std::string path, std::string path1,PROCESS_INFORMATION &pi, bool isAssembled = false);
	void generate_hinge_stl(std::string path, std::string path1,PROCESS_INFORMATION &pi, bool isAssembled = false);
	void generate_balljoint_stl(std::string path, std::string path1,PROCESS_INFORMATION &pi, bool isAssembled = false);
	void generate_prismatic_stl(std::string path, std::string path1,PROCESS_INFORMATION &pi, bool isAssembled = false);


private:
	/*
	Eigen::Vector3d trans;
	Eigen::Vector3d rot;
	double length;*/
	Edge2dp*  e1;
	Edge2dp* e2;
	Edge2dp*  auxe1;
	Edge2dp* auxe2;
	double angle;

	void get_stl_edge_args(std::ostringstream& args, bool isAssembled = false, bool checkisfold=false);
};

}


#endif