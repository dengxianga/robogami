#pragma once
#include <string>
#include <Eigen/Dense>
#include <vector>
#include <Eigen/StdVector>
#include <unordered_map>

namespace FabByExample {
namespace drawing {
	struct Edge {
		int id;
		std::string name;
		int vertice1_id;
		int vertice2_id;
		std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		typedef std::vector<Edge, Eigen::aligned_allocator<Edge>> vector_type;
		double trimFront; // positive = decrease length
		double trimBack;  // positive = decrease length
	};

	struct Point{
		Eigen::Vector2d p;
		int id;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		typedef std::list<Point, Eigen::aligned_allocator<Point>> point_vector_type;
	};

	struct Face {
		int id;
		std::string name;
		//std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points;
		Point::point_vector_type idpoints; 
		//std::unordered_map<int, int> mapping;
		std::vector<Edge> edges;
		std::unordered_map<int, int> edgeMap; // HACK for backing out edge from vertex id FIXME
		Point::point_vector_type::iterator getPointFromId(int _id){
			Point::point_vector_type::iterator it;
			for(it = idpoints.begin(); it!= idpoints.end(); ++it){
				if((*it).id == _id){
					return it;
				}
			}
			return idpoints.begin();
		}
		int getMaxID(){
			int maxID = 0;
			Point::point_vector_type::iterator it;
			for(it = idpoints.begin(); it!= idpoints.end(); ++it){
				if((*it).id >maxID){
					maxID = it->id;
				}
			}
			return maxID;
		}
		int getMinID(){
			int minID = -1;
			Point::point_vector_type::iterator it;
			for(it = idpoints.begin(); it!= idpoints.end(); ++it){
				if(minID == -1 || (*it).id < minID){
					minID = it->id;
				}
			}
			return minID;
		}
	};

	struct Drawing {
		//Edge::vector_type edges;
		std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points;
		std::vector<Face> faces;
		std::vector<Edge> edges;
		
	};
}
}
