#include "parser.h"
#include "tree.h"

using namespace rapidxml;
using namespace std;
using namespace FabByExample;


struct Ref3D{
	int id;
	std::string name;
	bool isLeaf;
	int repeted;
};


struct RepAssembly{
	Component *repComponenet;
	int	compId;
	int parentId;
	std::string name;
	int val;
	std::string relativeMatrixValue;
}; 

void Parser::parse_new(){

// Read file to string
	std::string input_xml;
	ifstream InFile; 
	InFile.open( fileName, ifstream::in );

	if( !InFile ) {
		cout << "Couldn´t open input file::" << fileName <<  endl;
		system("PAUSE"); 
	}

	istream_iterator<string> DataBegin( InFile );
	istream_iterator<string> DataEnd;
	while( DataBegin != DataEnd ) {
		//cout << *DataBegin << endl;
		input_xml += *DataBegin;
		input_xml += " ";
		DataBegin++;
	}
	//std::cout << input_xml << std::endl;


    // make a safe-to-modify copy of input_xml
    std::vector<char> xml_copy(input_xml.begin(), input_xml.end());
    xml_copy.push_back('\0');
    xml_document<> doc;
    //doc.parse<parse_declaration_node | parse_no_data_nodes>(&xml_copy[0]);
	doc.parse<parse_full>(&xml_copy[0]); // parses everything (slowest)


    // Getting to productStructure
    xml_node<>* productStructure_node = doc.first_node("Model_3dxml")->first_node("ProductStructure");





	leafComps.clear();

	//Getting All InstanceRep
    xml_node<>* instanceRep_node = productStructure_node->first_node("InstanceRep");
	while (instanceRep_node){
		string isleaf = instanceRep_node->first_node("IsAggregatedBy")->value();
		leafComps.push_back(atoi(isleaf.c_str()));
	    instanceRep_node = instanceRep_node->next_sibling("InstanceRep");
	}



	std::vector<Ref3D> ref3ds;
	//Getting All Reference3D
    xml_node<>* reference3D_node = productStructure_node->first_node("Reference3D");
    while (reference3D_node){
		string id = reference3D_node->first_attribute("id")->value();
		string name = reference3D_node->first_attribute("name")->value();
		//tree->addComponent(atoi(id.c_str()), name, checkIsLeaf(atoi(id.c_str())));
		reference3D_node = reference3D_node->next_sibling("Reference3D");
		Ref3D ref3d;
		ref3d.id = atoi(id.c_str());
		ref3d.name = name;
		ref3d.isLeaf = checkIsLeaf(ref3d.id);
		ref3d.repeted = 0;
		ref3ds.push_back(ref3d);
	}


	std::vector<RepAssembly> repetedAssemblies;

	//Getting All Instance3D
	Component* root  = tree->addComponent(10000, ref3ds[0].name, ref3ds[0].isLeaf);
	tree->addSubstructure(root); 
    // get the first Instance3D
    xml_node<>* instance3D_node = productStructure_node->first_node("Instance3D");
		while (instance3D_node){
		string name = instance3D_node->first_attribute("name")->value();
		string istId = instance3D_node->first_attribute("id")->value();
		string isAggregatedByValue = instance3D_node->first_node("IsAggregatedBy")->value();
		string isInstanceOfValue = instance3D_node->first_node("IsInstanceOf")->value();
		string relativeMatrixValue = instance3D_node->first_node("RelativeMatrix")->value();
		int iref;
		for (int i = 0; i < ref3ds.size(); i++){
			if(ref3ds[i].id == atoi(isInstanceOfValue.c_str()))
				iref = i;
			}
		int compId = (ref3ds[iref].id*1000 + ref3ds[iref].repeted)*10;
		int parentId = (atoi(isAggregatedByValue.c_str()))*10000;
		if( (ref3ds[iref].repeted > 0) && (ref3ds[iref].isLeaf == false)){
			//std::cout << "has a repeted subassembly " << std::endl;
			RepAssembly repAssembly;
			repAssembly.repComponenet = tree->getComponentFromID(ref3ds[iref].id*10000);
			repAssembly.compId = compId;
			repAssembly.parentId = parentId;
			repAssembly.name = name;
			repAssembly.val = ref3ds[iref].repeted;
			repAssembly.relativeMatrixValue = relativeMatrixValue;
			repetedAssemblies.push_back(repAssembly);
		}else{
			int compId = (ref3ds[iref].id*1000 + ref3ds[iref].repeted)*10;
			Component* comp = tree->addComponent(compId, name, ref3ds[iref].isLeaf);
			if(!ref3ds[iref].isLeaf){
				tree->addSubstructure(comp); 
			}
			tree->addEdge(parentId, compId, relativeMatrixValue);
		}
		ref3ds[iref].repeted = ref3ds[iref].repeted +1;
		instance3D_node = instance3D_node->next_sibling("Instance3D");
	}

	for (int i = 0; i < repetedAssemblies.size(); i ++){
		RepAssembly repAssembly = repetedAssemblies[i];
		Component* parentComp = tree->addComponent(repAssembly.compId, repAssembly.name, false);
		tree->addEdge(repAssembly.parentId, repAssembly.compId, repAssembly.relativeMatrixValue);

		tree->addRepetedSubassembly(parentComp, repAssembly.repComponenet, repAssembly.val);
	}


}


void Parser::parse_old(){
/*
// Read file to string
	std::string input_xml;
	ifstream InFile; 
	InFile.open( fileName, ifstream::in );

	if( !InFile ) {
		cout << "Couldn´t open input file::" << fileName <<  endl;
		system("PAUSE"); 
	}

	istream_iterator<string> DataBegin( InFile );
	istream_iterator<string> DataEnd;
	while( DataBegin != DataEnd ) {
		//cout << *DataBegin << endl;
		input_xml += *DataBegin;
		input_xml += " ";
		DataBegin++;
	}
	//std::cout << input_xml << std::endl;


    // make a safe-to-modify copy of input_xml
    std::vector<char> xml_copy(input_xml.begin(), input_xml.end());
    xml_copy.push_back('\0');
    xml_document<> doc;
    //doc.parse<parse_declaration_node | parse_no_data_nodes>(&xml_copy[0]);
	doc.parse<parse_full>(&xml_copy[0]); // parses everything (slowest)


    // Getting to productStructure
    xml_node<>* productStructure_node = doc.first_node("Model_3dxml")->first_node("ProductStructure");


	//Getting All Reference3D
    // get the first Reference3D
    xml_node<>* reference3D_node = productStructure_node->first_node("Reference3D");
    string id = reference3D_node->first_attribute("id")->value();
	string name = reference3D_node->first_attribute("name")->value();
	//cout << "first ref = " << id << " - "<< name  << endl;
	tree->addComponent(atoi(id.c_str()), name);
    // and then to the others
    reference3D_node = reference3D_node->next_sibling("Reference3D");
	while (reference3D_node){
		id = reference3D_node->first_attribute("id")->value();
		name = reference3D_node->first_attribute("name")->value();
	    reference3D_node = reference3D_node->next_sibling("Reference3D");
		//cout << "sib ref = " << id << " - "<< name  << endl;
		tree->addComponent(atoi(id.c_str()), name);
	}

	//Getting All InstanceRep
    // get the first InstanceRep
    xml_node<>* instanceRep_node = productStructure_node->first_node("InstanceRep");
    //string id3 = instance3D_node->first_attribute("id")->value();
	//string repIsAggregatedByValue = instanceRep_node->first_node("IsAggregatedBy")->value();
	string isleaf = instanceRep_node->first_node("IsAggregatedBy")->value();
	//cout << "isleaf = " << isleaf << endl;
	tree->addReference(atoi(isleaf.c_str()));
	// and then to the others
    instanceRep_node = instanceRep_node->next_sibling("InstanceRep");
	while (instanceRep_node){
		isleaf = instanceRep_node->first_node("IsAggregatedBy")->value();
		//cout << "isleaf = " << isleaf << endl;
		tree->addReference(atoi(isleaf.c_str()));
	    instanceRep_node = instanceRep_node->next_sibling("InstanceRep");
	}


	//Getting All Instance3D
    // get the first Instance3D
    xml_node<>* instance3D_node = productStructure_node->first_node("Instance3D");
    string id2 = instance3D_node->first_attribute("id")->value();
	string isAggregatedByValue = instance3D_node->first_node("IsAggregatedBy")->value();
	string isInstanceOfValue = instance3D_node->first_node("IsInstanceOf")->value();
	string relativeMatrixValue = instance3D_node->first_node("RelativeMatrix")->value();
	//cout << "first inst = " << id2 << endl;
	//cout << "isAggregatedByValue  = " << isAggregatedByValue <<endl;
	//cout << "isInstanceOfValue  = " << isInstanceOfValue <<endl;
	//cout << "relativeMatrixValue  = " << relativeMatrixValue <<endl;
	tree->addEdge(atoi(isAggregatedByValue.c_str()), atoi(isInstanceOfValue.c_str()), relativeMatrixValue);
	// and then to the others
    instance3D_node = instance3D_node->next_sibling("Instance3D");
	while (instance3D_node){
		id2 = instance3D_node->first_attribute("id")->value();
		isAggregatedByValue = instance3D_node->first_node("IsAggregatedBy")->value();
		isInstanceOfValue = instance3D_node->first_node("IsInstanceOf")->value();
		string relativeMatrixValue = instance3D_node->first_node("RelativeMatrix")->value();
		//cout << "sib inst = " << id2 << endl;
		//cout << "isAggregatedByValue  = " << isAggregatedByValue <<endl;
		//cout << "isInstanceOfValue  = " << isInstanceOfValue <<endl;
		//cout << "relativeMatrixValue  = " << relativeMatrixValue <<endl;
		tree->addEdge(atoi(isAggregatedByValue.c_str()), atoi(isInstanceOfValue.c_str()), relativeMatrixValue);
	    instance3D_node = instance3D_node->next_sibling("Instance3D");
	}

	*/

}


bool Parser::checkIsLeaf(int id){
	bool isLeaf = false;
	for(int i = 0; i< leafComps.size(); i++){
		if(leafComps[i] == id){
			isLeaf = true;
		}
	}
	return isLeaf;

}


void Parser::parse(){

// Read file to string
	std::string input_xml;
	ifstream InFile; 
	InFile.open( fileName, ifstream::in );

	if( !InFile ) {
		cout << "Couldn´t open input file::" << fileName <<  endl;
		system("PAUSE"); 
	}

	istream_iterator<string> DataBegin( InFile );
	istream_iterator<string> DataEnd;
	while( DataBegin != DataEnd ) {
		//cout << *DataBegin << endl;
		input_xml += *DataBegin;
		input_xml += " ";
		DataBegin++;
	}
	//std::cout << input_xml << std::endl;


    // make a safe-to-modify copy of input_xml
    std::vector<char> xml_copy(input_xml.begin(), input_xml.end());
    xml_copy.push_back('\0');
    xml_document<> doc;
    //doc.parse<parse_declaration_node | parse_no_data_nodes>(&xml_copy[0]);
	doc.parse<parse_full>(&xml_copy[0]); // parses everything (slowest)


    // Getting to productStructure
    xml_node<>* productStructure_node = doc.first_node("Model_3dxml")->first_node("ProductStructure");





	leafComps.clear();

	//Getting All InstanceRep
    // get the first InstanceRep
    xml_node<>* instanceRep_node = productStructure_node->first_node("InstanceRep");
    //string id3 = instance3D_node->first_attribute("id")->value();
	//string repIsAggregatedByValue = instanceRep_node->first_node("IsAggregatedBy")->value();
	string isleaf = instanceRep_node->first_node("IsAggregatedBy")->value();
	//cout << "isleaf = " << isleaf << endl;
	leafComps.push_back(atoi(isleaf.c_str()));
	// and then to the others
    instanceRep_node = instanceRep_node->next_sibling("InstanceRep");
	while (instanceRep_node){
		isleaf = instanceRep_node->first_node("IsAggregatedBy")->value();
		//cout << "isleaf = " << isleaf << endl;
		leafComps.push_back(atoi(isleaf.c_str()));
	    instanceRep_node = instanceRep_node->next_sibling("InstanceRep");
	}




	//Getting All Reference3D
    // get the first Reference3D
    xml_node<>* reference3D_node = productStructure_node->first_node("Reference3D");
    string id = reference3D_node->first_attribute("id")->value();
	string name = reference3D_node->first_attribute("name")->value();
	//cout << "first ref = " << id << " - "<< name  << endl;
	tree->addComponent(atoi(id.c_str()), name, checkIsLeaf(atoi(id.c_str())));
	// and then to the others
    reference3D_node = reference3D_node->next_sibling("Reference3D");
		while (reference3D_node){
			id = reference3D_node->first_attribute("id")->value();
		name = reference3D_node->first_attribute("name")->value();
	    reference3D_node = reference3D_node->next_sibling("Reference3D");
		//cout << "sib ref = " << id << " - "<< name  << endl;
		tree->addComponent(atoi(id.c_str()), name, checkIsLeaf(atoi(id.c_str())));
		//if(checkIsLeaf(atoi(id.c_str()))){
		//	tree->addReference(atoi(id.c_str()));
		//}
	}



	//Getting All Instance3D
    // get the first Instance3D
    xml_node<>* instance3D_node = productStructure_node->first_node("Instance3D");
    string id2 = instance3D_node->first_attribute("id")->value();
	string isAggregatedByValue = instance3D_node->first_node("IsAggregatedBy")->value();
	string isInstanceOfValue = instance3D_node->first_node("IsInstanceOf")->value();
	string relativeMatrixValue = instance3D_node->first_node("RelativeMatrix")->value();
	//cout << "first inst = " << id2 << endl;
	//cout << "isAggregatedByValue  = " << isAggregatedByValue <<endl;
	//cout << "isInstanceOfValue  = " << isInstanceOfValue <<endl;
	//cout << "relativeMatrixValue  = " << relativeMatrixValue <<endl;
	tree->addEdge(atoi(isAggregatedByValue.c_str()), atoi(isInstanceOfValue.c_str()), relativeMatrixValue);
	// and then to the others
    instance3D_node = instance3D_node->next_sibling("Instance3D");
	while (instance3D_node){
		id2 = instance3D_node->first_attribute("id")->value();
		isAggregatedByValue = instance3D_node->first_node("IsAggregatedBy")->value();
		isInstanceOfValue = instance3D_node->first_node("IsInstanceOf")->value();
		string relativeMatrixValue = instance3D_node->first_node("RelativeMatrix")->value();
		//cout << "sib inst = " << id2 << endl;
		//cout << "isAggregatedByValue  = " << isAggregatedByValue <<endl;
		//cout << "isInstanceOfValue  = " << isInstanceOfValue <<endl;
		//cout << "relativeMatrixValue  = " << relativeMatrixValue <<endl;
		tree->addEdge(atoi(isAggregatedByValue.c_str()), atoi(isInstanceOfValue.c_str()), relativeMatrixValue);
	    instance3D_node = instance3D_node->next_sibling("Instance3D");
	}



}



void Parser::parseNewVersion(){

// Read file to string
	std::string input_xml;
	ifstream InFile; 
	InFile.open( fileName, ifstream::in );

	if( !InFile ) {
		cout << "Couldn´t open input file::" << fileName <<  endl;
		system("PAUSE"); 
	}

	istream_iterator<string> DataBegin( InFile );
	istream_iterator<string> DataEnd;
	while( DataBegin != DataEnd ) {
		//cout << *DataBegin << endl;
		input_xml += *DataBegin;
		input_xml += " ";
		DataBegin++;
	}
	//std::cout << input_xml << std::endl;


    // make a safe-to-modify copy of input_xml
    std::vector<char> xml_copy(input_xml.begin(), input_xml.end());
    xml_copy.push_back('\0');
    xml_document<> doc;
    //doc.parse<parse_declaration_node | parse_no_data_nodes>(&xml_copy[0]);
	doc.parse<parse_full>(&xml_copy[0]); // parses everything (slowest)


    // Getting to productStructure
    xml_node<>* productStructure_node = doc.first_node("Model_3dxml")->first_node("ProductStructure");





	leafComps.clear();

	//Getting All InstanceRep
    // get the first InstanceRep
    xml_node<>* instanceRep_node = productStructure_node->first_node("InstanceRep");
    //string id3 = instance3D_node->first_attribute("id")->value();
	//string repIsAggregatedByValue = instanceRep_node->first_node("IsAggregatedBy")->value();
	string isleaf = instanceRep_node->first_node("IsAggregatedBy")->value();
	//cout << "isleaf = " << isleaf << endl;
	leafComps.push_back(atoi(isleaf.c_str()));
	// and then to the others
    instanceRep_node = instanceRep_node->next_sibling("InstanceRep");
	while (instanceRep_node){
		isleaf = instanceRep_node->first_node("IsAggregatedBy")->value();
		//cout << "isleaf = " << isleaf << endl;
		leafComps.push_back(atoi(isleaf.c_str()));
	    instanceRep_node = instanceRep_node->next_sibling("InstanceRep");
	}




	//Getting All Reference3D
    // get the first Reference3D
    xml_node<>* reference3D_node = productStructure_node->first_node("Reference3D");
    string id = reference3D_node->first_attribute("id")->value();
	string name = reference3D_node->first_attribute("name")->value();
	//cout << "first ref = " << id << " - "<< name  << endl;
	tree->addComponent(atoi(id.c_str()), name, checkIsLeaf(atoi(id.c_str())));
	// and then to the others
    reference3D_node = reference3D_node->next_sibling("Reference3D");
	while (reference3D_node){
		id = reference3D_node->first_attribute("id")->value();
		name = reference3D_node->first_attribute("name")->value();
	    reference3D_node = reference3D_node->next_sibling("Reference3D");
		//cout << "sib ref = " << id << " - "<< name  << endl;
		tree->addComponent(atoi(id.c_str()), name, checkIsLeaf(atoi(id.c_str())));
		//if(checkIsLeaf(atoi(id.c_str()))){
		//	tree->addReference(atoi(id.c_str()));
		//}
	}



	//Getting All Instance3D
    // get the first Instance3D
    xml_node<>* instance3D_node = productStructure_node->first_node("Instance3D");
    string id2 = instance3D_node->first_attribute("id")->value();
	string isAggregatedByValue = instance3D_node->first_node("IsAggregatedBy")->value();
	string isInstanceOfValue = instance3D_node->first_node("IsInstanceOf")->value();
	string relativeMatrixValue = instance3D_node->first_node("RelativeMatrix")->value();
	//cout << "first inst = " << id2 << endl;
	//cout << "isAggregatedByValue  = " << isAggregatedByValue <<endl;
	//cout << "isInstanceOfValue  = " << isInstanceOfValue <<endl;
	//cout << "relativeMatrixValue  = " << relativeMatrixValue <<endl;
	tree->addEdge(atoi(isAggregatedByValue.c_str()), atoi(isInstanceOfValue.c_str()), relativeMatrixValue);
	// and then to the others
    instance3D_node = instance3D_node->next_sibling("Instance3D");
	while (instance3D_node){
		id2 = instance3D_node->first_attribute("id")->value();
		isAggregatedByValue = instance3D_node->first_node("IsAggregatedBy")->value();
		isInstanceOfValue = instance3D_node->first_node("IsInstanceOf")->value();
		string relativeMatrixValue = instance3D_node->first_node("RelativeMatrix")->value();
		//cout << "sib inst = " << id2 << endl;
		//cout << "isAggregatedByValue  = " << isAggregatedByValue <<endl;
		//cout << "isInstanceOfValue  = " << isInstanceOfValue <<endl;
		//cout << "relativeMatrixValue  = " << relativeMatrixValue <<endl;
		tree->addEdge(atoi(isAggregatedByValue.c_str()), atoi(isInstanceOfValue.c_str()), relativeMatrixValue);
	    instance3D_node = instance3D_node->next_sibling("Instance3D");
	}



}


