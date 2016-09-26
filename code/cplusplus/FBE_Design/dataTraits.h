#ifndef __DATATRAITS__
#define __DATATRAITS__

#include<list>
#include<vector>
#include<sstream>

namespace FabByExample{

class RangeConstraint{
public:
	double max;
	char type;
	std::string getString(){
		std::stringstream name;
		name << type << " " << max*25.4;
		return name.str();
	}
};


struct PartInfo{
	RangeConstraint range[3]; // for x, y and z
	int material;
	std::string name;
};

struct ExtraPatch{
	int ax;
	double delta;
	int sign;
};


struct PatchInfo{
	std::vector<ExtraPatch> patches; // for x, y and z
	int partId;
	std::string modelName;
};


class DataTraits{

public:

	DataTraits();
	PartInfo* getSpecificationForComponentName(std::string name);
	bool checkIfHasSpecification(std::string name);
	bool getMaterialForComponentName(std::string name, int* material);
	void read();
	void readPatches();
	PatchInfo* getPatchInfoForComponent(std::string _modelName, int _partId);


private:
	std::list<PartInfo> partInfoList;
	std::vector<PatchInfo> patchList;
};

}
#endif