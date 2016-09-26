#include "dataTraits.h"
#include "params.h"
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <iostream>
using namespace FabByExample;
DataTraits::DataTraits(){


	
	std::string line, node1, node2;
	std::stringstream myfilename;
	myfilename << SHARED_DATA_DIR << "ranges.txt";

	std::ifstream myfile (myfilename.str());
	if (!myfile.is_open())
	{
		std::cout << "problem opening file makePartInfoList" << std::endl;
	}

	getline (myfile,line);
	while ( myfile.good() )
	{
		getline (myfile,line);
		std::istringstream  isLine(line);
		PartInfo partInfo;
		isLine >> partInfo.name;

		for (int i = 0; i < 3; i++){
				isLine >> partInfo.range[i].type;
				isLine >> partInfo.range[i].max;
		}

		isLine >> partInfo.material;

		partInfoList.push_back(partInfo);

	}
	myfile.close();
	
	readPatches(); 
	//std::list<PartInfo> ::iterator it; 
	//for(it = partInfoList.begin(); it!= partInfoList.end(); it++){
	//	std::cout << (*it).name << std::endl;
	//}
	//system("pause"); 



}



void DataTraits::readPatches(){


	
	std::string line, node1, node2;
	std::stringstream myfilename;
	myfilename << "..\\..\\data\\UI_testing\\extraPatches.txt" ; 
	std::ifstream myfile (myfilename.str());
	if (!myfile.is_open())
	{
		std::cout << "problem opening file extra patches" << std::endl;
	}

	getline (myfile,line);
	while ( myfile.good() )
	{		
		std::istringstream  isLine(line);
		PatchInfo info;
		isLine >> info.modelName;
		isLine >> info.partId;
		int NPatches;
		isLine >> NPatches;
		for (int i = 0; i < NPatches; i++){
				getline (myfile,line);
				std::istringstream  isLine2(line);
				ExtraPatch ep;
				isLine2 >> ep.ax;
				isLine2 >> ep.sign;
				isLine2 >> ep.delta;
				info.patches.push_back(ep);
		}
		patchList.push_back(info);
		getline (myfile,line);
	}
	myfile.close();
	
	/*
	for(int i = 0; i < patchList.size(); i++){
		std::cout << "Patch for: " << patchList[i].modelName << " --- " << patchList[i].partId << std::endl;
		for(int j = 0 ; j < patchList[i].patches.size() ; j++){
			std::cout << "-----" << patchList[i].patches[j].ax << " <-> " << patchList[i].patches[j].delta << std::endl;
		}
	}
	system("pause"); 

	*/

}


PatchInfo* DataTraits::getPatchInfoForComponent(std::string _modelName, int _partId){

	//std::cout << "Looking for: " << _modelName << " --- " <<  _partId << std::endl;


	for(int i = 0; i < patchList.size(); i++){
		if((patchList[i].modelName.compare(_modelName) == 0) && (patchList[i].partId == _partId)){
			//std::cout << "Found patch for: " << patchList[i].modelName << " --- " << patchList[i].partId << std::endl;
			//for(int j = 0 ; j < patchList[i].patches.size() ; j++){
			//	std::cout << "-----" << patchList[i].patches[j].ax << " <-> " << patchList[i].patches[j].delta << std::endl;
			//}
			return & patchList[i];
		}
	}
	

	return NULL;

}


 PartInfo* DataTraits::getSpecificationForComponentName(std::string name){

	std::string orgGeofileName(name);
	//cout << "filename = " << fileName << endl;
	size_t found= orgGeofileName.find(".", 2);
	if (found > 100)
		found = orgGeofileName.find("-", 1);
	orgGeofileName.resize(found);
	orgGeofileName.append(".STL"); 
	
	std::string orgGeofileName2(name);
	found = orgGeofileName2.find("-", 1);
	if (found < 200)
		orgGeofileName2.resize(found);
	orgGeofileName2.append(".STL"); 
	std::list<PartInfo>::iterator itaux;

	bool foundRangeAnnot = false;

	for ( itaux = partInfoList.begin() ; itaux != partInfoList.end(); itaux++ ){
		//std::cout << "orgGeofileName = " << (*it)->name << std::endl;
		if(orgGeofileName2.compare((*itaux).name) == 0){
			//std::cout << "orgGeofileName = " << orgGeofileName << std::endl;
			//std::cout << "orgGeofileName2 = " << orgGeofileName2 << std::endl;			
			//std::cout << "(*itaux)->name = " << (*itaux)->name << std::endl;
			return &(*itaux);
			
		}
	}
	for ( itaux = partInfoList.begin() ; itaux != partInfoList.end(); itaux++ ){
		//std::cout << "orgGeofileName = " << (*it)->name << std::endl;
		if( orgGeofileName.compare((*itaux).name) == 0){
			//std::cout << "orgGeofileName = " << orgGeofileName << std::endl;
			//std::cout << "orgGeofileName2 = " << orgGeofileName2 << std::endl;			
			//std::cout << "(*itaux)->name = " << (*itaux)->name << std::endl;
			return &(*itaux);
			
		}
	}

	return NULL;

}

bool DataTraits::checkIfHasSpecification(std::string name){

	std::string orgGeofileName(name);
	//cout << "filename = " << fileName << endl;
	size_t found= orgGeofileName.find(".", 2);
	if (found > 100)
		found = orgGeofileName.find("-", 1);
	orgGeofileName.resize(found);
	orgGeofileName.append(".STL"); 
	
	std::string orgGeofileName2(name);
	found = orgGeofileName2.find("-", 1);
	orgGeofileName2.resize(found);
	orgGeofileName2.append(".STL"); 
	std::list<PartInfo>::iterator itaux;

	bool foundRangeAnnot = false;

	for ( itaux = partInfoList.begin() ; itaux != partInfoList.end(); itaux++ ){
		//std::cout << "orgGeofileName = " << (*it)->name << std::endl;
		if((orgGeofileName2.compare((*itaux).name) == 0) || ( orgGeofileName.compare((*itaux).name) == 0)){
			return true;
			
		}
	}

	return false;

}


bool DataTraits::getMaterialForComponentName(std::string name, int* material){

    PartInfo* partInfo = getSpecificationForComponentName(name);
	if(partInfo == NULL)
		return false;

	*material = partInfo->material;

	return true;

}

void DataTraits::read(){

	std::list<PartInfo>::iterator itaux;

	bool foundRangeAnnot = false;
	int counter = 0;
	for ( itaux = partInfoList.begin() ; itaux != partInfoList.end(); itaux++ ){
		counter++;
		std::cout << "(" << counter << ") " << (*itaux).name << std::endl;
		std::cout << "----> material = " << (*itaux).material << std::endl;
		std::cout << "----> constraints = (" << (*itaux).range[0].type << ", " << (*itaux).range[1].type << ", " << (*itaux).range[2].type << ")"<< std::endl;
	}	

}