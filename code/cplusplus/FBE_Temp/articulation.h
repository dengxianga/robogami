#ifndef _ARTICULATION_H
#define _ARTICULATION_H

#include <Eigen\Dense>
#include <Eigen/Geometry>
#include <vector>
#include <list>
#include<Eigen/StdVector>
#include "symbolic.h"
namespace FabByExample{
		

	class Geometry; 
	class Controller;


	class SymbolicTransformation : public Debuggable {
	public:
		SymbolicTransformation();
		SymbolicTransformation(Controller * _controller, Point3S & _symbAxis, bool isTrans);
		void updateParameters(SymbolicAssignment const& env);
		void updateTime(double t);
		void updateTransform();

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Eigen::Affine3d transformation;
		Controller * controller;
		Point3S symbAxis;
		Eigen::Vector3d axis;
		double val;
		bool isTrans;
		FabDebugging::DebugInfo* getDebugInfo();
		
	};
	


	class Articulation : public Debuggable {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Articulation(); 
		Articulation(Point3S & _symbCenter); 
		void updateParameters(SymbolicAssignment const& env);
		void updateTime(double t);
		void transformMesh(TriMesh* mesh, bool hasnormals = true);
		void transformGeo(Geometry* geo, bool hasnormals = true);
		void transformVec(Eigen::Vector3d & vec);
		void transformPoints(std::vector<point> &pts);
		void updateTransform();
		// this is temporary, remove after we stop hardcoding the axis and center
		void transformSelf(const Eigen::Matrix4d& xform);
		void addTranformation(SymbolicTransformation* trans );
		void rotate(Eigen::Vector3d const& center, Eigen::Quaterniond const& rotation);
		void translate(Eigen::Vector3d const& trans);
		Point3S symbCenter;
		std::vector<SymbolicTransformation*> transformations;
		void print(){
			VLOG(3) << "center: ( " << center[0] << ", " << center[1] << ", " << center[2] << " )";
			VLOG(3) << "transformation: [ " << transformation(0, 0) << ", " << transformation(0, 1) << ", " << transformation(0, 2) << " " << " , " << transformation(0, 3);
			VLOG(3) << transformation(1, 0) << ", " << transformation(1, 1) << ", " << transformation(1, 2) << " " << " , " << transformation(1, 3);
			VLOG(3) << transformation(2, 0) << ", " << transformation(2, 1) << ", " << transformation(2, 2) << " , " << transformation(2, 3) << " ]";
		}
		FabDebugging::DebugInfo* getDebugInfo();
		Eigen::Vector3d getCenter() {return center;}
	private:
		Eigen::Vector3d center;
		Eigen::Affine3d transformation;
	};
}


#endif