#include <iostream>
#include <fstream>
#include "TriMesh_algo.h"
#include "Design.h"
#include "geometry.h"
#include "params.h"
#include "tree.h"
#include "graph.h"
#include "parser.h"
#include "dataTraits.h"


using namespace FabByExample;
Design::Design(std::string dataName){
	name = dataName;
	tree = new Tree(this, dataName);
	graph = new Graph(dataName);	
}

Design::Design(std::string dataName,  DataTraits* dataTraits, bool userNew){
	
	name = dataName;
	tree = new Tree(this, dataName);
	//reducedComponentsGraph = NULL;
	std::stringstream inFile;
	if(userNew)
		inFile<< SHARED_DATA_DIR << "cleanDataBase\\3DXML\\" << dataName <<".3dxml" ; 
	else
		inFile<< SHARED_DATA_DIR << "3DXML\\" << dataName <<".3dxml" ; 

	//if (MODEL_TYPE == 1){
	//	inFile = "..\\SW2010\\Test Chairs\\3DXML\\";
	//}


	//std::cout<< "DataName: " << dataName << std::endl;
	//std::cout<< "inFile: " << inFile << std::endl;
	//system("pause");
	Parser parser(tree, inFile.str());
	parser.parse_new();
	//tree->simpleRead();
	matrix9f rot; 	rot.LoadIdentity();
	vector3f trans;	trans.LoadZero();
	//tree->simpleRead();
	tree->updateGlobalTrans(tree->getRoot(), rot, trans);

	//system("pause");
	//tree->simpleRead();
	//system("pause");
	tree->setFunctionality();
	//system("pause");
	//tree->removeAllConnectors();
	//tree->simpleRead();
	//tree->simpleRead();
	//system("pause");
	graph = new Graph(dataName);

	//tree->simpleRead();
	//system("pause");
	tree->getRoot()->removeConnectorGroup();
	//tree->simpleRead();
	//system("pause");

	
	std::list<Component*> leafComponents = tree->getLeafComponents();
	std::list<Component*>::iterator it;
	//std::cout << "adding to graph" << std::endl; 
	for ( it=leafComponents.begin() ; it != leafComponents.end(); ++it ){
		//std::cout << "adding : " << (*it)->getName() << std::endl;
		graph->addNode((*it));
	}
		
	graph->getRepetedNames(); 
	//addGraphConnections();
	//initializeCurrentComponent();

	loadSavedFuncStates();

	//graph->simpleRead();
	//system("PAUSE");

	//dataTraits->read();
	//system("pause");
	addPartInfo(dataTraits);
	
}


void Design::addPartInfo( DataTraits* dataTraits){
	std::list<Component*> leafComponents = tree->getLeafComponents();
	std::list<Component*>::iterator it;
	//std::cout << "adding to graph" << std::endl; 
	for ( it=leafComponents.begin() ; it != leafComponents.end(); ++it ){
		ComponentLeaf* comp = dynamic_cast<ComponentLeaf*>(*it);
		int material;
		if(comp->isPart()){
			if(dataTraits->getMaterialForComponentName(comp->getName(), &material)){
				comp->setMaterial(material);
				//std::cout << " ----- ok to find the part info for component:" << comp->getName() << std::endl;
			
			}else{
				comp->setMaterial(2);
				std::cout << "error: could not find the part info for component:" << comp->getName() << std::endl;
			}
		}
	}

}


Design::~Design(){

	//std::cout<< "Destroing the data" << std::endl; system("pause");
	//delete tempInfo;
	//std::cout<< "done the tempinfo" << std::endl; system("pause");
	//for (std::list<ConnectorsGroup*>::iterator it=connectorGroups.begin() ; it != connectorGroups.end(); ++it ){
	//	delete (*it);
	//}
	//connectorGroups.clear();
	//std::cout<< "Done connectorGroups" << std::endl; system("pause");
	//delete reducedComponentsGraph;
	//std::cout<< "done the reducedcompgraph" << std::endl; system("pause");
	graph->clearNodes();
	delete graph;
	//std::cout<< "destroid the graph" << std::endl; system("pause");
	delete tree;
	//std::cout<< "Done the tree" << std::endl; system("pause");
}


void Design::read(){
	tree->simpleRead();
	system("pause"); 
	graph->simpleRead();
}


int Design::getNumFuncs(){
	return tree->getNumFuncts();
}

void Design::saveDefaultConnectivitytoFile(std::string filename){
	tree->checkAllIntersections(filename);
}

void Design::saveConnectivitytoFile(std::string filename){
	graph->write(filename);
}

void Design::saveConnectivity(){
	std::cout << "Saving ... ";
	std::stringstream filename;
	filename<< SHARED_DATA_DIR << "connections\\" << name <<".txt" ; 
	saveConnectivitytoFile(filename.str());
	std::cout << "Done." << std::endl;

}



void Design::saveConnectivityWNamestoFile(std::string filename){
	graph->writeWNames(filename);
}

void Design::saveConnectivityWNames(){
	std::cout << "Saving ... ";
	std::stringstream filename;
	filename<< SHARED_DATA_DIR << "connectionsWithNames\\" << name <<".txt" ; 
	saveConnectivityWNamestoFile(filename.str());
	std::cout << "Done." << std::endl;

}

void Design::saveFuncStates(){
	std::cout << "Saving ... ";
	std::stringstream filename;
	filename<< SHARED_DATA_DIR << "funcStates\\" << name <<".txt" ; 
	std::ofstream myfile (filename.str());
	if (!myfile.is_open())
	{
		std::cout << "Unable to open file" << std::endl;
		system("pause");
	}
	else{
		for (unsigned int i = 0 ; i < funcSates.size(); i++ ){
			myfile  << funcSates[i].name << std::endl;
			myfile  << funcSates[i].alphas[0];
			for (unsigned int j = 1 ; j< funcSates[i].alphas.size(); j++ ){
				myfile  << " " << funcSates[i].alphas[j];
			}
			myfile << std::endl;
		}	
	}
	myfile.close();
	std::cout << "Done." << std::endl;

}


void Design::loadSavedFuncStates(){
	//std::cout << "Loading Func States ... ";
	std::stringstream filename;
	filename<< SHARED_DATA_DIR << "funcStates\\" << name <<".txt" ; 
	std::ifstream myfile (filename.str());
	std::string line;
	if (!myfile.is_open())
	{
		//std::cout << "Unable to open file of func states" << std::endl;
	}
	else{
		getline (myfile,line);
		while ( myfile.good() )
		{
			FuncState fs;
			//std::cout << line << std::endl;
			fs.name = line;
			getline (myfile,line);
			size_t found;
			found=line.find(" ");
			while(found!=std::string::npos){
				fs.alphas.push_back(atof(line.substr(0,found).c_str()));
				line = line.substr(found+1);
				found=line.find(" ");
			}  
			fs.alphas.push_back(atof(line.substr(0,found).c_str()));
			for (unsigned int j = 0 ; j< fs.alphas.size(); j++ ){
				//std::cout  << fs.alphas[j] << " ";
			}
			//std::cout << std::endl;
			funcSates.push_back(fs);
			getline (myfile,line);
		}
		myfile.close();
	}
	//std::cout << "Done." << std::endl;

}

void Design::saveDefaultConnectivity(){
	std::stringstream filename;
	filename<< SHARED_DATA_DIR << "connections\\" << name <<".txt" ; 
	saveDefaultConnectivitytoFile(filename.str());

}

void Design::writeMeshTofile(std::string filename){
	tree->write(filename);
}

void Design::writeMeshTofileUsingGlobalCoords(std::string filename){
	tree->saveAllLeafComponentsInGlobalCoords(filename);
		
}



std::string Design::addEdgesFromFileWNames(std::string filename){
	std::string errorMessage("Count find conections: ");
	std::string line, node1, node2, nodep1, nodep2;
	int n1, n2;
	bool allset = true;

	std::ifstream myfile (filename);
	if (!myfile.is_open())
	{
		std::cout << "problem opening file with names" << std::endl;
		return "Could not get Connectivity. Names don't match or nothing saved.";
	}
	while ( myfile.good() )
	{
		getline (myfile,line);
		getline (myfile,nodep1);
		getline (myfile,node1);
		getline (myfile,nodep2);
		getline (myfile,node2);

		if(line.compare("New Intersection") == 0){
			n1 = tree->getIdFromNames(nodep1, node1);
			n2 = tree->getIdFromNames(nodep2, node2);
			if ((n1 > 0) && (n2>0))
			graph->addEdgeFromIds(n1, n2);
			else{
				std::cout << "could not find part " << std::endl;
				errorMessage.append("(");
				errorMessage.append(node1);
				errorMessage.append(" <> ");
				errorMessage.append(node2);
				errorMessage.append(") ");
				allset = false;
			}
		}
	}
	myfile.close();
	if (allset)
		return "Added all Connections from Names!";
	return errorMessage;
}



void Design::removeAllGraphEdges(){
	graph->removeAllEdges();

}

void Design::writeGraphToFile(std::string filename){
	graph->write(filename);
}




void Design::removeGraphEdgeFromIds(int name1, int name2){
	graph->removeEdgeFromIds( name1, name2);
}

void Design::addGraphEdgeFromIds(int agg_name, int ist_name){
	graph->addEdgeFromIds( agg_name, ist_name);
}

void Design::reloadDefaulIntersections(double err){
	std::cout << "loading default connections ...";
	graph->removeAllEdges();
	graph->reloadDefaulIntersections(err);
	std::cout << "Done." << std::endl;

}


void Design::simplify(){
	//tree->simpleRead();
	//graph->simpleRead();
	//system("pause");
	std::cout << "Simplifying data... " ;

	std::list<GNode*> nodes = graph->getNodes();
	std::list<GNode*>::iterator it, it2;
	for ( it=nodes.begin() ; it != nodes.end();  ){
		it2 = it;
		it2++;
		Component * comp = (*it)->getComponent();
		if(comp->isConnector()){
			//std::cout<< (*it)->getComponent()->getName() << std::endl; 
			graph->remove(*it);
			//std::cout << "component" << comp->getName() << std::endl; 
			tree->removeComponentfromTree(comp);
			//system("pause");
			//tree->simpleRead();
			//graph->simpleRead();
		}
		it = it2;
	}
	std::cout << "Done." << std::endl;
	//system("pause");
	tree->simpleRead();
	//graph->simpleRead();

		


}




Component* Design::getComponentFromIndex(int index){
	return (tree->getComponentFromID(index));
}



void Design::getReducedComponentsGraph(){
	/*reducedComponentsGraph = new Graph(name);

	std::list<Component*> leafComponents = tree->getLeafComponents();
	std::list<Component*>::iterator it;
	//std::cout << "adding to graph" << std::endl; 
	for ( it=leafComponents.begin() ; it != leafComponents.end(); ++it ){
		//std::cout << "adding : " << (*it)->getName() << std::endl;
		reducedComponentsGraph->addNode((*it));
	}
	
	std::stringstream filename;
	filename<< SHARED_DATA_DIR << "connections\\" << name <<".txt" ; 
	reducedComponentsGraph->addEdgesFromFile(filename.str());

	*/
	//reducedComponentsGraph->simpleRead();
	graph->reduceToConnectors();
	graph->convertAllConnctorGroupsToComp();


	

	//reducedComponentsGraph->simpleRead();
}



bool Design::hasNotShownUPYet(int istate, int id, double alpha){
	bool result = true;
	for (int i =0; i < istate; i++){
		if(funcSates[i].alphas[id] == alpha)
			result = false;
	}
	return result;
}


void Design::addFuncState(std::string _name, std::vector<double> _alphas){
	FuncState fs;
	fs.name = _name;
	fs.alphas = _alphas;
	funcSates.push_back(fs);
}
std::vector<double> Design::getFuncState(int i){
	return funcSates[i].alphas;
}




std::vector<TriMesh*> Design::getMeshsToBePlottedForFuncs(std::vector<double> alphas, std::vector<Functionality*> funcs){
	std::vector<TriMesh*>meshes;

	//std::list<Component*>::iterator it;
	//std::list<Component*> comps = tree->getLeafComponents();
	//for ( it=comps.begin() ; it != comps.end(); ++it ){ if((*it)->getIsLeaf() == true){
	//	meshes.push_back((*it)->getGeometryMinFunc(alpha)->getMesh());
	//}}

	std::list<GNode*> nodes = getGraph()->getNodes();
	std::list<GNode*>::iterator it;
	for ( it=nodes.begin() ; it != nodes.end(); ++it ){	
		meshes.push_back((*it)->getComponent()->getGeometryMinFuncs(alphas, funcs)->getMesh());
	}

	return meshes;
}



std::vector<TriMesh*> Design::getOriginalMeshes(Component::ComponentSet cs){
	
	std::vector<TriMesh*>meshes;

	std::list<GNode*> nodes = graph->getNodes();
	std::list<GNode*>::iterator it;


	for ( it=nodes.begin() ; it != nodes.end(); it++ ){
		Component * comp = (*it)->getComponent();
		if((comp->isConnector() && (cs == Component::ComponentSet::PARTS)) ||
			(!comp->isConnector() && (cs == Component::ComponentSet::CONNS))){
		}else{
		//std::cout << "comp:: = " << comp->getId() << " - " << comp->getName() << std::endl;

			//system("pause");
			Geometry *m = comp->getGeometryInGlobalCoord();
			//m->getMesh()->need_tstrips();
	
			meshes.push_back(m->getMesh());
		}

	}

	return meshes;
}



std::string Design::addOldGraphConnections(){
	std::stringstream filename;
	filename<< SHARED_DATA_DIR << "connections\\" << name <<".txt" ; 
	bool added = graph->addEdgesFromFile(filename.str());
	if (added){
		std::cout << "The previously saved connectivity is ok." << std::endl;
	}
	else{
		graph->removeAllEdges();
		std::cout << "The previously saved connectivity has ERRORS!" << std::endl; //system("pause");
	}
	//std::stringstream filename2;
	//filename2<< SHARED_DATA_DIR << "connectionsWithNames\\" << name <<".txt" ; 
	//return addEdgesFromFileWNames(filename2.str());
	return("nothinig");

}


bool Design::addNewGraphConnections(double err){
	std::stringstream filename;
	filename<< SHARED_DATA_DIR << "cleanDataBase\\connectivityGraph\\" << name <<".txt" ; 
	bool added = graph->addEdgesFromNewFile(filename.str());
	if (added){
		std::cout << "Was able to transfer connectivity from existent data. :)" << std::endl;
	}
	else{
		graph->removeAllEdges();
		graph->reloadDefaulIntersections(err);
		std::cout << "Was NOT able to transfer connectivity from existent data! :(" << std::endl; //system("pause");
	}
	return added; 
}

void Design::saveNewGraphConnections(){
	std::stringstream filename;
	filename<< SHARED_DATA_DIR << "cleanDataBase\\connectivityGraph\\" << name <<".txt" ; 
	graph->writeNewConnections(filename.str());

}


void Design::saveGraphConnectionsToGraphFile(std::string graphFilename){

}
