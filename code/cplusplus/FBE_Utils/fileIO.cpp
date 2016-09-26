#include "fileIO.h"


MyInFile::MyInFile(std::string _filename):file(_filename){
	filename = _filename;
}

int MyInFile::open(){
	if (!file.is_open())
	{
		std::cout << " could not open file" << filename  << std::endl;
		system("pause");
		return -1;
	}

	return 0;
}

bool MyInFile::openIfPossible(){
	if (!file.is_open())
	{
		return false;
	}

	return true;
}

void MyInFile::close(){
	file.close();
}

void MyInFile::readLine(std::string & line){

}
void MyInFile::readAllLines(std::vector<std::string> &lines){
	std::string line;
	getline (file,line);
	while ( file.good() )
	{
		lines.push_back(std::string(line));
		getline (file,line);
	}

}
void MyInFile::readStruct(std::vector< std::vector<std::string>> & filearray, int N){
	std::string line;
	getline (file,line);
	while ( file.good() )
	{
		std::vector<std::string> parts(N);
		for(int i = 0; i<  N-1; i++){
			size_t found=line.find(" ");
			parts[i] = line.substr(0,found);
			line = line.substr(found+1);
		}
		parts[N-1] = line;
		filearray.push_back(parts);
		getline (file,line);
	}

}


// ---------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------


MyOutFile::MyOutFile(std::string _filename){
	filename = _filename;
	append = false;
}

MyOutFile::MyOutFile(std::string _filename, bool _append){
	filename = _filename;
	append = _append; 
	
}



int MyOutFile::open(){
	if (append){
		file.open(filename, std::ios_base::app);
	} else{
		file.open(filename);
	}

	if (!file.is_open())
	{
		std::cout << " could not open file" << filename  << std::endl;
		system("pause");
		return -1;
	}

	return 0;
}

void MyOutFile::close(){
	file.close();
}
	
void MyOutFile::writeLine(std::string line){

		file << line << std::endl;

}

void MyOutFile::writeAllLines(std::vector<std::string> &lines){

}

void MyOutFile::writeStruct(std::vector< std::vector<std::string>> & filearray){

}




