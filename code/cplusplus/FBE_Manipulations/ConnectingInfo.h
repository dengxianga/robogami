#ifndef _CONNECTING_INFO_
#define _CONNECTING_INFO_


#include<vector>
#include "NewPatch.h"

namespace FabByExample{

	class ConnectingInfo{
	public:
		ConnectingInfo();
		~ConnectingInfo();
		void addPatchPair(NewPatch* wt_patch, NewPatch * add_patch, bool isOpposite, bool isOpposite2D);
		void addPoint3S(Point3S p){ connectingPoints.push_back(p); }
		void clear();
		std::vector<PatchPair> matchingNewPatches;
		std::vector<Point3S> connectingPoints;
	};

}

#endif