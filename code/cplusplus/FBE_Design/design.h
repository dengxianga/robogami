#ifndef DATA_H
#define DATA_H

#include <list>
#include "functionality.h"
#include "component.h"

class TriMesh;

namespace FabByExample{
class DataTraits;
class Tree;
class Component;
class Graph;

class Design{
private:
	std::string name;
	Tree* tree;
	Graph* graph; 
	std::vector<FuncState> funcSates;
	//Graph* reducedComponentsGraph;

		
public:


	Design(std::string dataName, DataTraits* dataTraits, bool userNew=false);
	Design(std::string dataName);
	void addPartInfo( DataTraits* dataTraits);
	~Design();
	std::string getName() { return name;}
	Tree* getTree () { return tree;}
	Graph* getGraph() { return graph;}	
//	Graph* getGraphRC() { 
//		if(reducedComponentsGraph == NULL)
//			getReducedComponentsGraph();
//		return reducedComponentsGraph;}
	void read();

	//Tree
	int getNumFuncs();
	Component * getComponentFromIndex(int index);
	void simplify();

	//Connectivity
	void saveDefaultConnectivitytoFile(std::string filename);
	void saveConnectivitytoFile(std::string filename);
	void saveConnectivityWNames();
	void saveConnectivityWNamestoFile(std::string filename);	
	void saveConnectivity();
	void saveDefaultConnectivity();
	void removeAllGraphEdges();
	void writeGraphToFile(std::string filename);
	void removeGraphEdgeFromIds(int name1, int name2);
	void addGraphEdgeFromIds(int agg_name, int ist_name);
	void reloadDefaulIntersections(double err);
	std::string addEdgesFromFileWNames(std::string filename);
	std::string addOldGraphConnections();
	bool addNewGraphConnections(double err);
	void saveNewGraphConnections();
	void saveGraphConnectionsToGraphFile(std::string graphFilename);

	// Functionality
	void saveFuncStates();
	void loadSavedFuncStates();
	bool hasNotShownUPYet(int istate, int id, double alpha);
	void addFuncState(std::string _name, std::vector<double> _alphas);
	std::vector<double> getFuncState(int i);	
	std::vector<FuncState> getFuncStates(){ return funcSates;}
	void getReducedComponentsGraph();





	// get Meshes
	std::vector<TriMesh*> getOriginalMeshes(Component::ComponentSet cs);
	std::vector<TriMesh*> getMeshsToBePlottedForFuncs(std::vector<double> alphas, std::vector<Functionality*> funcs);
	

	//writeMesh
	void writeMeshTofile(std::string filename);
	void writeMeshTofileUsingGlobalCoords(std::string filename);
	
};

}


  
#endif
