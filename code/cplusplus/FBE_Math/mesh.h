#pragma once
#include <string>
#include <Eigen/Dense>
#include <vector>
#include <Eigen/StdVector>
#include <unordered_map>

namespace FabByExample {
namespace mesh {

	struct Point{
		Eigen::Vector3d p;
	};

	struct Face {
		Face(){};
		std::vector<int> points;
	};

	struct SubElementFace {
		// normal of the face
		Eigen::Vector3d normal;
		// up direction of the bounding box (normalized, perpendicular to normal)
		Eigen::Vector3d up;
		// Center position of the face
		Eigen::Vector3d center;

		double width;
		double height;

	};
	
	struct Edge {
		int edgeid;
		std::vector<Eigen::Vector3d> vertices;
	};

	struct Mesh {
		std::vector<Eigen::Vector3d> points;
		std::vector<Face> faces;
		std::vector<Edge> edges;
		std::vector<SubElementFace> subElementFaces;
	};
}
}
