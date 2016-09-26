#ifndef __GRAPH__
#define __GRAPH__

#include <list>
#include <vector>

namespace FabByExample{

class GEdge;
class Component;
class Geometry;

class GNode{
private:
	std::list<Component*> components;
	std::list<GEdge*> edges;


public:
	GNode(Component* _component){components.push_back(_component);}
	~GNode();
	Component* getComponent(){ return (*components.begin());}
	std::list<GEdge*> getEdges(){ return edges;}
	std::list<Component*> getComponents(){ return components;}
	Geometry* getGeometryInGlobalCoord();

	void printNode();
	void simplePrintNode();
	void addEdge(GNode* node, bool isDirect);
	void removeAllEdges();
	int removeEdge(GNode* node);
	bool isConnected(int id2);
	bool isOneOfTheComponenst(Component* comp);    
	bool isOneOfTheEdges(Component* comp);
	void merge(GNode* node2);
	bool replaceEdge(GNode* node1, GNode* node2);
	bool GNode::checkIfEdgeExists(GNode* node_2);
	bool hasAnEdgeTo(GNode* node);
	bool inConenctorGroup();
	GNode* getFirtsConnectedConnectorGroup();
	bool isthesame(GNode* node); 
	void replaceGroupByComponent(Component* comp);
	void display(bool showEdges);


};



class GEdge{
private:
	GNode* child;
	bool isDirect;
public:
	GEdge(GNode* _child, bool _isDirect){ child = _child; isDirect = _isDirect;}
	void printEdge();
	GNode* getChild(){	return child; }
	bool getIsDirect() {return isDirect;}
	void replaceChild (GNode* newChild) {child = newChild;}

};



class Graph{
private:
	std::list<GNode*> nodes;
	std::string objectName;
	std::vector<std::string> repetedNames;
public:
	Graph(std::string _objectName);
	~Graph(){
		nodes.clear();
		repetedNames.clear(); 
	}
	std::list<GNode*> getNodes(){ return  nodes;}
	void addNode(Component* _component);
	void read();
	void simpleRead();
	GNode* getNodeFromId (int id);
	//GNode* getNodeFromName (std::string name);
	GNode* getFirstNode ();
	GNode* getNextNode (GNode* currentNode);
	GNode* getPrevNode (GNode* currentNode);
	bool addEdgeFromIds(int agg_id, int ist_id);
	//void addEdgeFromNames(std::string agg_name, std::string ist_name);
	bool addEdgesFromFile(std::string filename);
	bool addEdgesFromNewFile(std::string filename);
	void write(std::string filename);
	void writeNewConnections(std::string filename);
	void writeWNames(std::string filename);
	void removeAllEdges();
	//void removeEdgeFromNames(std::string name1, std::string name2);
	void removeEdgeFromIds(int id1, int id2);
	void removeEdge(GNode* edg1, GNode* edg2);
	void reloadDefaulIntersections(double err);
	void remove(GNode* node);
	int removeNode(GNode* node);
	void reduceToConnectors();
	void clearNodes(){		
		for (std::list<GNode*>::iterator it=nodes.begin() ; it != nodes.end(); ++it ){
			delete (*it);
		}
	}
	void getRepetedNames();
	bool checkIsRepetedName(std::string name); 
	bool Graph::getNodeFromStr(std::string stringNode, GNode* & node); 
	void Graph::convertAllConnctorGroupsToComp();

	void display();


};


}

#endif