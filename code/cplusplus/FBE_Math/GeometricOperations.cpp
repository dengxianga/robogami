#include "GeometricOperations.h"
#include "CenterOfMass.h"
using namespace FabByExample;






double GeometricOperations::mycross(const Eigen::Vector2d &O, const Eigen::Vector2d &A, const Eigen::Vector2d &B)
{
	return (A(0) - O(0)) * (B(1) - O(1)) - (A(1) - O(1)) * (B(0) - O(0));
}


static bool myCompoarator(IdPoint &A, IdPoint &B){
	return A.point(0) < B.point(0) || (A.point(0) == B.point(0) && A.point(1) < B.point(1));
}


double GeometricOperations::getStabilityCost( CenterOfMass & centerofMass,
			std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>  contactPoints,
			int & nearestVert, int & edgeinfo){

	int N = contactPoints.size();
	nearestVert = -1; edgeinfo = -1;

	// Special case #1: no points
	if (N == 0) {
		return 0;
	}

	Eigen::Vector2d center;
	center.x() = centerofMass.center.x();
	center.y() = centerofMass.center.z();

	// Special case #2: 1 point
	if (N == 1) {
		nearestVert = 0;
		return (center-contactPoints[0]).norm();
	}
	// Special case #3: 2 points <-- does this need its own special case?
	if (N == 2) {
		// which direction?
		double newDist = GeometricOperations::mycross(contactPoints[0], center, contactPoints[1]);

		double distance;
		Eigen::Vector2d dir =  (contactPoints[1] - contactPoints[0]);
		double proj = (center - contactPoints[0]).dot(dir.normalized());
		if((proj < 0 )) {
			nearestVert = 0;
			return (center - contactPoints[0]).norm();
		}else if ((proj > dir.norm())){
			nearestVert = 1;
			return (center - contactPoints[1]).norm();
		}else{
			if (newDist > 0) {
				nearestVert = 0;
				edgeinfo = 1;
			} else {
				nearestVert = 1;
				edgeinfo = 0;
			}
			return ((center - contactPoints[0]) - proj*dir.normalized()).norm();
		}
	}

	// if there are 3+ points
	std::vector<int> hull;
	computeConvexHull(contactPoints, hull);

	int thisclosestPoint = -1, closestPoint = -1;
	bool thisisedge = false, isedge = false;

	bool isStable = true;
	double stabDist = std::numeric_limits<double>::max();

	for(int i = 0; i < hull.size(); i++){
		int next_i = (i+1) % (hull.size());

		// Is the CoM outside or inside the convex hull?
		double newDist = GeometricOperations::mycross(contactPoints[hull[i]], center, contactPoints[hull[next_i]]);
		//std::cout << "p1 = " << contactPoints[hull[i]].transpose() << std::endl;
		//std::cout << "p2 = " << contactPoints[hull[next_i]].transpose() << std::endl;
		//std::cout << "c0 = " << center.transpose() << std::endl;
		//std::cout << "newDist = " << newDist << std::endl;
		
		// The point is outside of the convex hull --> unstable
		if(newDist > 0){
			//std::cout << "is out!! " << newDist << std::endl;
			isStable = false;
			//compute the distance
			double distance;
			Eigen::Vector2d dir =  (contactPoints[hull[next_i]] - contactPoints[hull[i]]);
			double proj = (center - contactPoints[hull[i]]).dot(dir.normalized());
			if((proj < 0 )) {
				distance =  (center - contactPoints[hull[i]]).norm(); 
				thisclosestPoint = i;
				thisisedge = false;
			}else if ((proj > dir.norm())){
				distance = (center - contactPoints[hull[next_i]]).norm();
				thisclosestPoint = next_i;
				thisisedge = false;
			}else{
				distance = ((center - contactPoints[hull[i]]) - proj*dir.normalized()).norm();	
				thisclosestPoint = i;
				thisisedge = true;
			}
			// Find the minimum distance to the convex polygon edges
			if(stabDist > distance){
				stabDist = distance;
				closestPoint = thisclosestPoint;
				isedge = thisisedge;
			}
		}else{
			//std::cout << "is in!" << std::endl;
		}
		//std::cout << "----------------------------------" << std::endl;
	}

	if(isStable){
		stabDist = std::numeric_limits<double>::max();
		for(int i = 0; i < hull.size(); i++){
			int next_i = (i+1) % (hull.size());

			double distance;
			Eigen::Vector2d dir =  (contactPoints[hull[next_i]] - contactPoints[hull[i]]);
			double proj = (center - contactPoints[hull[i]]).dot(dir.normalized());
			if((proj < 0 ) || (proj > dir.norm())){
				distance =  std::min( (center - contactPoints[hull[i]]).norm(), (center - contactPoints[hull[next_i]]).norm()); 
			}else{
				distance = ((center - contactPoints[hull[i]]) - proj*dir.normalized()).norm();	
			}
			//std::cout << "distance " << distance << std::endl;
			if(stabDist > distance){
				stabDist = distance;
			}
		}

		stabDist = -stabDist; 
		//std::cout << " is stable and stabDist " << stabDist << std::endl;
	} else {
		// output edgeinfo
		nearestVert = hull[closestPoint];

		int nextPoint = (closestPoint + 1) % (hull.size());
		if (isedge) {
			edgeinfo = hull[nextPoint];
		}
	}

	return stabDist;
}

double GeometricOperations::getStabilityCost( CenterOfMass & centerofMass,
			std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>  contactPoints){
	int closestVert, edgeinfo;
	return getStabilityCost(centerofMass, contactPoints, closestVert, edgeinfo);
}

void GeometricOperations::computeConvexHull(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> & inPoints, 
					std::vector<int> & hull){

	int n = inPoints.size();
	int k = 0;
	IdPoints P(n);
	IdPoints H(2*n);
	for(int i =0; i < n; i++){
		P[i] = IdPoint(i, inPoints[i]);
	}
 
	// Sort points lexicographically
	sort(P.begin(), P.end(), myCompoarator);
	// Build lower hull
	for (int i = 0; i < n; ++i) {
		while (k >= 2 && mycross(H[k-2].point, H[k-1].point, P[i].point) <= 0) 
			k--;
		H[k++] = P[i];
	}
 
	// Build upper hull
	for (int i = n-2, t = k+1; i >= 0; i--) {
		while (k >= t && mycross(H[k-2].point, H[k-1].point, P[i].point) <= 0) k--;
		H[k++] = P[i];
	}
 
	//H.resize(k);
	for(int i = 0; i < k-1; i++)
		hull.push_back(H[i].id);

	P.clear();
	H.clear();
}


double GeometricOperations::calcPlane(std::vector<Eigen::Vector3d> points, Eigen::Vector3d & pt, Eigen::Vector3d & norm) {

	if (points.size()<3) {return 0;}

	Eigen::MatrixXf A(points.size(), 3);
	Eigen::VectorXf b(points.size());
	
	for (int pidx = 0; pidx < points.size(); ++pidx) {
		A.row(pidx) << points[pidx].x(), points[pidx].z(), 1;
		b(pidx) = points[pidx].y();
	}

	//cout << "Here is the matrix A:\n" << A << endl;
	//cout << "Here is the vector b:\n" << b << endl;

	Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);
	//cout << "The solution is:\n" << x << endl;

	//cout << "Here is the matrix A:\n" << A << endl;
	//cout << "Here is the vector b:\n" << b << endl;

	double SOSerror = (A*x-b).norm();
	pt(0) = 0; pt(1) = x(2); pt(2) = 0;
	norm(0) = x(0); norm(1) = 1; norm(2) = x(1);
	norm.normalize();
    
	/*
	pt = points[0];
	Eigen::Vector3d bdir1 = points[1] - pt;
	Eigen::Vector3d bdir2 = points[2] - pt;
	norm = bdir1.cross(bdir2);
	norm.normalize();
	*/

	// check direction, we want the plane normal to face in +z direction
	Eigen::Vector3d plusZ (0,1,0);
	if (plusZ.dot(norm) < 0) {
		norm = -norm;
	}

	return SOSerror;
}