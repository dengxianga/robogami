#include "graph.h"
#include "tree.h"
#include "component.h"
#include "geometry.h"


using namespace std;
using namespace FabByExample;


GNode::~GNode(){
	for (std::list<GEdge*>::iterator it=edges.begin() ; it != edges.end(); ++it ){
		delete (*it);
	}
	edges.clear();
	//std::cout << " in the destructor " << std::endl;
}

Geometry* GNode::getGeometryInGlobalCoord(){
	Geometry * nodeGeo = new Geometry();
	std::list<Component*>::iterator it;
	for (it = components.begin(); it != components.end(); ++it){
		nodeGeo->addMesh((*it)->getGeometryInGlobalCoord()->getMesh());
	}
	return nodeGeo;
}

void GNode::printNode(){
	std::cout << "Component = " << getComponent()->getName() << std::endl;
	std::cout << "Edges: " <<  std::endl;
	std::list<GEdge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
			(*it)->printEdge();
	}
}

void GNode::simplePrintNode(){
	std::cout << "COMPONENT: " << getComponent()->getId() << " - " << getComponent()->getName() << std::endl;
	std::list<GEdge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
		std::cout << "       -> " << (*it)->getChild()->getComponent()->getId() << " - " << (*it)->getChild()->getComponent()->getName() << std::endl;
		;
	}
}

void GNode::addEdge(GNode* node, bool isDirect){
	GEdge* newedge = new GEdge(node, isDirect);
	edges.push_back(newedge);
}

int GNode::removeEdge(GNode* node){
	std::list<GEdge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
		if((*it)->getChild()->getComponent()->getId() == (node->getComponent()->getId())){
			//std:: cout << "!!! comparing " << (*it)->getChild()->getComponent()->getName() << " and " <<node->getComponent()->getName()<< endl;
			edges.erase(it);
			return 0;
		}
	}
	return 1;
}

void GNode::removeAllEdges(){
	std::list<GEdge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
			delete (*it);
	}
	edges.clear();
}


bool GNode::isConnected(int id2){
	if(getComponent()->getId() == id2)
		return true;
	std::list<GEdge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
		if((*it)->getChild()->getComponent()->getId() == id2){
			return true;
		}
	}
	return false;
}

bool GNode::isOneOfTheComponenst(Component* comp){
	std::list<Component*>::iterator itc;
	for (itc = components.begin(); itc!= components.end(); itc++){
		if((*itc)->getId() == comp->getId())
			return true;
	}
	return false;
}

void GNode::merge(GNode* node2){

	std::list<Component*> comps2 = node2->getComponents();

	//std::cout<< "add comps2 to component list" << std::endl;
	std::list<Component*>::iterator itc;
	for (itc = comps2.begin(); itc!= comps2.end(); itc++){
		components.push_back(*itc);
	}

	//std::cout<< "remove all edges to comps2" << std::endl;
	std::list<GEdge*>::iterator it, it2;
	for ( it=edges.begin() ; it != edges.end(); ){
		it2 = it;
		it2++;
		if(isOneOfTheComponenst((*it)->getChild()->getComponent())){
			//std::cout<< "removing edge to" << (*it)->getChild()->getComponent()->getName() << std::endl;
			(*it)->getChild()->removeEdge(this);
			edges.erase(it);
		}
		it = it2;
	}

	std::list<GEdge*> edges2 = node2->getEdges();
	//std::cout<< "adding edges2" << std::endl;
	for ( it=edges2.begin() ; it != edges2.end(); it++ ){
		GNode * nodeChild = (*it)->getChild();

		//there is an edge from node 2 to nodeChild and therefore thereshould be an edge from node child to node2
		if(!nodeChild->checkIfEdgeExists(node2)){
			std::cout << "big error! " << std::endl;
			//system("pause");
			nodeChild->checkIfEdgeExists(node2);
		}
		

		if(hasAnEdgeTo(nodeChild)){
			// this node already has a edge to what the new part is pointing to 
			// so all you  have to do is remove the edge
			nodeChild->removeEdge(node2);
			//std::cout<< "removing an edge to node2 from" << (*it)->getChild()->getComponent()->getName() << std::endl;
		}	
		else{
			//std::cout<< "pushing back:" << std::endl;
			bool ok = nodeChild->replaceEdge(node2, this);
			if(!ok){
				std::cout<< "did not find child!" << std::endl;
				system("pause");
			}
			edges.push_back(*it);
			//(*it)->getChild()->simplePrintNode();
		}
	}



	//std::cout<< "adding edges2" << std::endl;

}

void GNode::display(bool showEdges){
	std::list<Component*>::iterator it;
	for(it = components.begin(); it!= components.end(); ++it){
		std::cout << " ----> " << (*it)->getName() << "----" << (*it)->getId() << std::endl;
	}
	std::list<GEdge*>::iterator it2;
	for ( it2=edges.begin() ; it2 != edges.end(); ++it2 ){
		std::cout << " --------->  edge to: " << (*it2)->getChild()->getComponent()->getName() << "---" <<  (*it2)->getChild()->getComponent()->getId() << std::endl;
		if(!(*it2)->getChild()->checkIfEdgeExists(this)){
			std::cout << "error in the display!" << std::endl;
			system("pause");
		}
	}
	system("pause");
}


bool GNode::checkIfEdgeExists(GNode* node_2){
	bool ok = false;
	std::list<GEdge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
		if((*it)->getChild()->getComponent()->getId() == node_2->getComponent()->getId()){
			ok = true;
		}
	}
	return ok;
}

bool GNode::replaceEdge(GNode* node_2, GNode* this_node){
	bool ok = false;
	std::list<GEdge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
		if((*it)->getChild()->getComponent()->getId() == node_2->getComponent()->getId()){
			(*it)->replaceChild(this_node);
			ok = true;
		}
	}
	return ok;
}

bool GNode::hasAnEdgeTo(GNode* node){
	std::list<GEdge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
		if((*it)->getChild()->getComponent()->getId() == node->getComponent()->getId()){
			return true;
		}
	}
	return false;

}

bool GNode::inConenctorGroup(){
	if (getComponent()->isConnector())
		return true;
	return false;
}
GNode* GNode::getFirtsConnectedConnectorGroup(){
	std::list<GEdge*>::iterator it;
	for ( it=edges.begin() ; it != edges.end(); ++it ){
		GNode * connComp = (*it)->getChild();
		if(connComp->inConenctorGroup()){
			if(connComp->getComponent()->getParent() == this->getComponent()->getParent())
			return connComp;
		}
	}

	return NULL;

}

bool GNode::isthesame(GNode* node2){
	std::list<Component*>::iterator itc, itc2;
	std::list<Component*> components = node2->getComponents();
	if (components.size() != components.size())
			return false;
	int nonMatches = components.size() ;

	for (itc = components.begin(); itc!= components.end(); itc++){
		for (itc2 = components.begin(); itc2!= components.end(); itc2++){
			if((*itc)->getGenName().compare((*itc2)->getGenName()) == 0){
				nonMatches --;
			}
		}
	}
	if(nonMatches > 0)
		return false;
	//if all is well return true
	return true;

}



//---------------------------------------------------------------------------------------------------------


void GEdge::printEdge(){ 
	if(isDirect)
		std::cout << " Direct Edge to = " << child->getComponent()->getName() << std::endl;
	else
		std::cout << " Indirect Edge to = " << child->getComponent()->getName() << std::endl;
}


//---------------------------------------------------------------------------------------------------------



Graph::Graph(std::string _objectName){
	objectName = _objectName;
}


void Graph::addNode(Component* _component){
	GNode * newNode = new GNode(_component);
	nodes.push_back(newNode);
}


void Graph::read(){
	std::list<GNode*>::iterator it;
	cout << "READ" << endl; 
	for ( it=nodes.begin() ; it != nodes.end(); ++it ){
		std::cout << "=== New Node in Graph === " << std::endl;
		(*it)->printNode();
	}
}

void Graph::simpleRead(){
	std::list<GNode*>::iterator it;
	cout << "================ GRAPH ================" << endl; 
	for ( it=nodes.begin() ; it != nodes.end(); ++it ){
		(*it)->simplePrintNode();
	}
	cout << "================ END GRAPH ================" << endl; 
}

GNode* Graph::getNodeFromId (int id){
	std::list<GNode*>::iterator it;
	for ( it=nodes.begin() ; it != nodes.end(); ++it ){
		if((*it)->getComponent()->getId() == id)
			return *it;

	}
	return NULL;

}


//GNode* Graph::getNodeFromName (std::string name){
//	GNode* node;
//	std::list<GNode*>::iterator it;
//	//cout << "Name = " << name << endl;
//	for ( it=nodes.begin() ; it != nodes.end(); ++it ){
//		//cout << "name = " << (*it)->getComponent()->getName() <<endl; 
//		if(name.compare((*it)->getComponent()->getName()) == 0){
//			node = *it;
//			return node;
//		}
//	}
//
//	return NULL;
//}


GNode* Graph::getFirstNode (){
//	GNode* node;
	if (nodes.empty())
			return NULL;
	return *(nodes.begin());	
}


GNode* Graph::getPrevNode (GNode* currentNode){

	std::list<GNode*>::iterator it, finalIt;
	int pos = 0;
	int finalpos = 0;
	for ( it=nodes.begin() ; it != nodes.end(); ++it ){
		//cout << "name = " << (*it)->getComponent()->getName() <<endl; 
		//alt2212
		if( currentNode->getComponent()->getId() == (*it)->getComponent()->getId()){ 	
			finalIt = it;
			finalpos = pos;
		}
		pos++;
	}

	if (finalpos != 0){
		--finalIt;
		return(*finalIt);
	}

	return NULL;
	
}

GNode* Graph::getNextNode (GNode* currentNode){

	std::list<GNode*>::iterator it, finalIt;
	int pos = nodes.size();
	int finalpos = 0;
	for ( it=nodes.begin() ; it != nodes.end(); ++it ){
		pos--;
		//cout << "name = " << (*it)->getComponent()->getName() <<endl; 
		//alt2212
		if( currentNode->getComponent()->getId() == (*it)->getComponent()->getId()){ 	
			finalIt = it;
			finalpos = pos;
		}
	}

	if (finalpos != 0){
		++finalIt;
		return(*finalIt);
	}

	return NULL;
	
}

bool Graph::addEdgeFromIds(int id1, int id2){
	GNode* node1 = getNodeFromId(id1);
	GNode* node2 = getNodeFromId(id2);

	if(node1 == NULL){
		//std::cout << "Node 1 === null" << std::endl;
		return false;
	}
	if(node2 == NULL){
		//std::cout << "Node 2 === null" << std::endl;
		return false;
	}


	node1->addEdge(node2, true);
	node2->addEdge(node1, false);
	return true;
}

//void Graph::addEdgeFromNames(string name1, string name2){
//	GNode* node1 = getNodeFromName(name1);
//	GNode* node2 = getNodeFromName(name2);
//	node1->addEdge(node2, true);
//	node2->addEdge(node1, false);
//}

bool Graph::addEdgesFromFile(std::string filename){
	std::string line, node1, node2;
	std::ifstream myfile (filename);
	if (!myfile.is_open())
	{
		//std::cout << "problem opening file addEdgesFromFile" << std::endl;
		return false;
	}
	while ( myfile.good() )
	{
		getline (myfile,line);
		getline (myfile,node1);
		getline (myfile,node2);

		if(line.compare("New Intersection") == 0){
			//std::cout << "new = " << line << std::endl;
			//std::cout << "1 = " << line << std::endl;
			//std::cout << "2 = " << line << std::endl;
			bool allset = addEdgeFromIds(atoi(node1.c_str())*10000, atoi(node2.c_str())*10000);
			if (!allset){
				//std::cout << "did not find intersection from ids" << std::endl;
				return false;
			}
		}
	}
	myfile.close();
	return true;
}




bool Graph::addEdgesFromNewFile(std::string filename){
	std::string line, strnode1, strnode2;
	bool emptyFile = true;
	std::ifstream myfile (filename);
	if (!myfile.is_open())
	{
		std::cout << "Filename is " << filename << std::endl;
		std::cout << "problem opening file addEdgesFromFile" << std::endl;
		return false;
	}
	while ( myfile.good() )
	{
		getline (myfile,line);
		getline (myfile,strnode1);
		getline (myfile,strnode2);
		if(line.compare("New Edge") == 0){
			emptyFile = false;
			GNode* node1 = NULL;
			GNode* node2 = NULL;
			if (!getNodeFromStr(strnode1, node1))
				return false;	
			if (!getNodeFromStr(strnode2, node2))
				return false;	
			node1->addEdge(node2, true);
			node2->addEdge(node1, false);
	
		}
	}
	myfile.close();
	if (emptyFile)
		return false;

	return true;
}

void Graph::write(std::string filename){
	ofstream myfile (filename);
	if (!myfile.is_open())
	{
		cout << "Unable to open file" << endl;
	}
	std::list<GNode*>::iterator it;
	for ( it=nodes.begin() ; it != nodes.end(); ++it ){
		std::list<GEdge*> nodeEdges = (*it)->getEdges();
		std::list<GEdge*>::iterator it2;
		//cout << "at1 = " << (*it)->getComponent()->getName()  << endl;
		for ( it2=nodeEdges.begin() ; it2 != nodeEdges.end(); ++it2 ){
			//cout << "at2 = " << (*it2)->getChild()->getComponent()->getName() << endl;
			if ((*it2)->getIsDirect() == true){
				myfile  << "New Intersection" << endl;
				myfile <<  (*it)->getComponent()->getId() << endl;
				myfile <<  (*it2)->getChild()->getComponent()->getId() << endl;
			}
		}		
	}
	myfile.close();
}



void Graph::getRepetedNames(){
	std::list<GNode*>::iterator it, it2;
	for ( it=nodes.begin() ; it != nodes.end(); ++it ){
		for ( it2=nodes.begin() ; it2 != it; ++it2 ){
			std::string name1 = (*it)->getComponent()->getName();
			std::string name2 = (*it2)->getComponent()->getName();
			if(name1.compare(name2) == 0){
				repetedNames.push_back(name1);
			}
		}
	}

//	for(int i = 0; i <repetedNames.size(); i++){
//		std::cout << "name = " << repetedNames[i] << std::endl;
//	}
}

bool Graph::checkIsRepetedName(std::string name){
	for(int i = 0; i <repetedNames.size(); i++){
		if(repetedNames[i].compare(name) == 0){
			return true;
		}
	}
	return false;
}

bool Graph::getNodeFromStr(std::string stringNode, GNode* & node){
	std::istringstream line(stringNode);
	std::string repetedStr;
	bool repeted = false;
	int idNumber;
	std::string name;
	std::string parentName;

	line >> repetedStr; 
	if(repetedStr.compare("R") == 0)
		repeted = true;
	line >> idNumber;
	line >> name;
	line >> parentName;

/*	std::list<GNode*>::iterator it;
	for ( it=nodes.begin() ; it != nodes.end(); ++it ){
		std::string i_name = (*it)->getComponent()->getName();
		if(name.compare(i_name) == 0){
			if(!repeted){
				node = (*it);
				return true;
			}else{
				std::string i_parentName = (*it)->getComponent()->getParent()->getName();
				if(parentName.compare(i_parentName) == 0){
					node = (*it);
					return true;
				}
			}
		}
	}
*/
	std::list<GNode*>::iterator it;
	for ( it=nodes.begin() ; it != nodes.end(); ++it ){
		int i_id = (*it)->getComponent()->getId();
		if(i_id == idNumber){
			node = (*it);
			return true;
		}
	}

	return false;  
	

}

void Graph::writeNewConnections(std::string filename){


	ofstream myfile (filename);
	if (!myfile.is_open())
	{
		cout << "Unable to open file" << endl;
	}
	std::list<GNode*>::iterator it;
	for ( it=nodes.begin() ; it != nodes.end(); ++it ){
		std::list<GEdge*> nodeEdges = (*it)->getEdges();
		std::list<GEdge*>::iterator it2;
		//cout << "at1 = " << (*it)->getComponent()->getName()  << endl;
		for ( it2=nodeEdges.begin() ; it2 != nodeEdges.end(); ++it2 ){
			//cout << "at2 = " << (*it2)->getChild()->getComponent()->getName() << endl;
			if ((*it2)->getIsDirect() == true){
				myfile  << "New Edge" << endl;
				Component * comp1 = (*it)->getComponent();
				Component * comp2 = (*it2)->getChild()->getComponent();
				if (checkIsRepetedName(comp1->getName()))
					myfile <<  "R ";
				else
					myfile <<  "U ";
				myfile <<  comp1->getId()  << " " << comp1->getName() << " " << comp1->getParent()->getName() << endl;
				if (checkIsRepetedName(comp2->getName()))
					myfile <<  "R ";
				else
					myfile <<  "U ";
				myfile <<  comp2->getId()  << " " << comp2->getName() << " " << comp2->getParent()->getName() << endl;

			}
		}		
	}
	myfile.close();
}


void Graph::writeWNames(std::string filename){
	ofstream myfile (filename);
	if (!myfile.is_open())
	{
		cout << "Unable to open file" << endl;
	}
	std::list<GNode*>::iterator it;
	for ( it=nodes.begin() ; it != nodes.end(); ++it ){
		std::list<GEdge*> nodeEdges = (*it)->getEdges();
		std::list<GEdge*>::iterator it2;
		//cout << "at1 = " << (*it)->getComponent()->getName()  << endl;
		for ( it2=nodeEdges.begin() ; it2 != nodeEdges.end(); ++it2 ){
			//cout << "at2 = " << (*it2)->getChild()->getComponent()->getName() << endl;
			if ((*it2)->getIsDirect() == true){
				myfile  << "New Intersection" << endl;
				myfile <<  (*it)->getComponent()->getParent()->getName() << endl;
				myfile <<  (*it)->getComponent()->getName() << endl;
				myfile <<  (*it2)->getChild()->getComponent()->getParent()->getName() << endl;
				myfile <<  (*it2)->getChild()->getComponent()->getName() << endl;

			}
		}		
	}
	myfile.close();
}


void Graph::removeAllEdges(){
	std::list<GNode*>::iterator it;
	for ( it=nodes.begin() ; it != nodes.end(); ++it ){
		(*it)->removeAllEdges();
	}
}



//void Graph::removeEdgeFromNames(string name1, string name2){
//	//cout << "is here !!!!!" << endl;
//	GNode* node1 = getNodeFromName(name1);
//	GNode* node2 = getNodeFromName(name2);
//	if((node1 != NULL) && (node2 != NULL)){
//		node1->printNode();
//		node2->printNode();
//		removeEdge(node1, node2);
//	}
//}


void Graph::removeEdgeFromIds(int id1, int id2){
	//cout << "is here !!!!!" << endl;
	GNode* node1 = getNodeFromId(id1);
	GNode* node2 = getNodeFromId(id2);
	if((node1 != NULL) && (node2 != NULL)){
		//node1->printNode();
		//node2->printNode();
		removeEdge(node1, node2);
	}
}

void Graph::removeEdge(GNode* edg1, GNode* edg2){
	edg1->removeEdge(edg2);
	edg2->removeEdge(edg1);
}



void Graph::reloadDefaulIntersections(double err){

	std::list<GNode*>::iterator it, it2;
	int count = 0;
	for ( it=nodes.begin() ; it != nodes.end(); ++it ){
		it2 = it; it2++;
		cout << "Comparing = " << count << " out of  " << nodes.size() << endl;
		for ( 0; it2 != nodes.end(); ++it2 ){
			if (((*it)->getComponent()->isPart() + (*it2)->getComponent()->isPart()) <= 1){
				//cout << "Comparing = " << (*it)->getComponent()->getName() << " and " << (*it2)->getComponent()->getName() << endl;
				bool intersection;
				if((*it)->getComponent()->isConnector() && (*it2)->getComponent()->isConnector())
					intersection = (*it)->getComponent()->getGeometryInGlobalCoord()->checkIntersectionWithResolution((*it2)->getComponent()->getGeometryInGlobalCoord());
				else
					intersection = (*it)->getComponent()->getGeometryInGlobalCoord()->checkIntersection((*it2)->getComponent()->getGeometryInGlobalCoord(), err);			
				if ( intersection == true){	
					//std::cout << "found intersection! " << std::endl; 
					(*it)->addEdge((*it2), true);
					(*it2)->addEdge((*it), false);
				}
			}
		}
		count++; 
	}
}


void Graph::remove(GNode* node){

	Component * comp = node->getComponent();

	std::list<GEdge*> thisEdges = node->getEdges();
	std::list<GEdge*>::iterator it, it2;
	for ( it=thisEdges.begin() ; it != thisEdges.end(); ++it ){
		it2 = it;
		it2++;
		for ( ; it2 != thisEdges.end(); ++it2 ){
			bool areConnected = (*it)->getChild()->isConnected((*it2)->getChild()->getComponent()->getId());
			if(!areConnected){
				(*it)->getChild()->addEdge((*it2)->getChild(), true);
				(*it2)->getChild()->addEdge((*it)->getChild(), false);
			}
		}
	}
	for ( it=thisEdges.begin() ; it != thisEdges.end(); ++it ){
		removeEdge(node, (*it)->getChild());
	}


	removeNode(node);

	//std::cout << "here" << std::endl; 



}






int Graph::removeNode(GNode* node){
	std::list<GNode*>::iterator it3;
	for ( it3=nodes.begin() ; it3 != nodes.end(); ++it3 ){
		if((*it3)->getComponent()->getId() == (node->getComponent()->getId())){
			nodes.erase(it3);
			//std::cout << "erased" << std::endl; 
			return 0;

		}
	}
	return 1;
}

void Graph::display(){
	std::list<GNode*>::iterator it3;
	for ( it3=nodes.begin() ; it3 != nodes.end();  it3++){
		std::cout << "-----------------------------------:" << std::endl;
		(*it3)->display(true);
	}
}



void Graph::reduceToConnectors(){
	std::list<GNode*>::iterator it3,it4;
	for ( it3=nodes.begin() ; it3 != nodes.end();  ){
		it4 = it3;
		++it4;
		if((*it3)->inConenctorGroup()){
			GNode* conn = (*it3)->getFirtsConnectedConnectorGroup();
			if (conn != NULL){
				//std::cout << "merging:" << std::endl;
				//std::cout << "node1:" << std::endl;
				//conn->display(false);
				//std::cout << "mode2:" << std::endl;
				//(*it3)->display(false); 
				//std::cout << "-----------------------------------:" << std::endl;
				conn->merge(*it3);
				nodes.erase(it3);
			}
		}
		it3=it4;
	}
}


void GNode::replaceGroupByComponent(Component* comp){
	components.clear();
	components.push_back(comp);
}

void Graph::convertAllConnctorGroupsToComp(){
	int i = 0;
	std::list<GNode*>::iterator it;
	for ( it=nodes.begin() ; it != nodes.end();  it++){
		if(((*it)->inConenctorGroup()) && ((*it)->getComponents().size() > 1)){
			Component * comp = new ComponentNode(*it, i, "C.groupComp");
			i++;
			(*it)->getComponent()->getTree()->pushBackComp(comp);
			(*it)->replaceGroupByComponent(comp);
		}
	}
}


