#include "rapidxml.hpp"
#include <string>
#include <vector>

namespace FabByExample{

class Tree;

class Parser{
	Tree* tree;
	std::string fileName;
	std::vector<int> leafComps;
public:
	Parser(	Tree* _tree, std::string _fileName){
		tree = _tree;
		fileName = _fileName;
	}

	void parse_old();
	void parse();
	void parse_new(); 
	void parseNewVersion();
	bool checkIsLeaf(int id);

};

}

