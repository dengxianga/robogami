#ifndef GeometricOperations_h
#define GeometricOperations_f
#include <vector>
#include <Eigen/Dense>
#include<Eigen/StdVector>

namespace FabByExample{

	class CenterOfMass;

	class IdPoint {
	public:
		int id;
		Eigen::Vector2d point;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		IdPoint(){}
		IdPoint(int _id, Eigen::Vector2d & _point){
			id = _id;
			point = _point;
		}
	};
	typedef std::vector<IdPoint, Eigen::aligned_allocator<IdPoint>> IdPoints;


	class GeometricOperations
	{
	public:
		static bool getDistanceToConvexHull(){return true;}
		static void computeConvexHull(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> & inPoints, 
					std::vector<int> & hull);
		static double mycross(const Eigen::Vector2d &O, const Eigen::Vector2d &A, const Eigen::Vector2d &B);

		static double getStabilityCost( CenterOfMass & centerofMass,
			std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>  contactPoints,
			int & nearestVert, int & edgeinfo );
		static double getStabilityCost( CenterOfMass & centerofMass,
			std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>  contactPoints);
		static double calcPlane(std::vector<Eigen::Vector3d> points, Eigen::Vector3d & pt, Eigen::Vector3d & norm);

	};

}



#endif