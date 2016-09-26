#pragma once

#include <Eigen/Dense>
#include "symbolic.h"

namespace FabByExample
{
	class CenterOfMass{
	public:
		CenterOfMass(){
			mass = 0;
			center = Eigen::Vector3d::Zero();
		}
		Eigen::Vector3d center;
		double mass;
		void add (CenterOfMass const & p2){
			center = p2.center*p2.mass + center*mass;
			mass = p2.mass + mass;
			center = center/mass;
		}
	};

	class TriCenterOfMassSymb{
	public:
		TriCenterOfMassSymb(){}
		Point3S center;
		QuadExpr mass;
		CenterOfMass eval (SymbolicAssignment const& env){
			CenterOfMass result;
			result.mass = mass.eval(env);
			result.center = center.evalVector3d(env);
			return result;
		}
	};


	class CenterOfMassSymb{
	public:
		CenterOfMassSymb(){};
		std::vector<TriCenterOfMassSymb*> triangles;
		void evalAndAdd(CenterOfMass & centerOfMass, SymbolicAssignment const& env){
			double mass = 0;
			Eigen::Vector3d center = Eigen::Vector3d::Zero();
			for (int i = 0; i< triangles.size(); i++){
				CenterOfMass newCenter = triangles[i]->eval(env);
				//std::cout << "center tri " << i << " = " << newCenter.center.transpose() << std::endl;
				center = center + newCenter.mass* newCenter.center;
				mass = mass + newCenter.mass;
			}
			center = center/mass;
			//std::cout << "center part = " << center.transpose() << std::endl;
			centerOfMass.center = centerOfMass.center*centerOfMass.mass + center*mass;
			centerOfMass.mass = centerOfMass.mass + mass;
			centerOfMass.center = centerOfMass.center/centerOfMass.mass;
		}
	};





}