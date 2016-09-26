#ifndef fileIO_H
#define fileIO_H

#include<vector>
#include <string>
#include <iostream>
#include <fstream>

class MyInFile{

public:
	
	MyInFile(std::string _filename);
	int open();
	void close();
	bool openIfPossible();
	
	//reading functions
	void readLine(std::string & line);
	void readAllLines(std::vector<std::string> &lines);
	void readStruct(std::vector< std::vector<std::string>> & filearray, int N);

private:
	std::string filename;
	std::ifstream  file;

};


class MyOutFile{

public:
	

	MyOutFile(std::string _filename);
	MyOutFile(std::string _filename, bool _append);
	int open();
	void close();
	
	//writing functions 
	void writeLine(std::string line);
	void writeAllLines(std::vector<std::string> &lines);
	void writeStruct(std::vector< std::vector<std::string>> & filearray);

private:
	std::string filename;
	std::ofstream  file;
	bool append;

};

class ErrorFile{
public:
	MyOutFile * file; 

	ErrorFile(){
		file =  new MyOutFile("..\\..\\data\\UI_testing\\log.txt", true);
		file->open();
	}

	void write(std::string line){
		file->writeLine(line);
	}

	~ErrorFile(){
		file->close();
		delete file;
	}

	static void openAndWrite(std::string line){
		MyOutFile fileaux("..\\..\\data\\UI_testing\\log.txt", true);
		fileaux.open();
		fileaux.writeLine(line);
		fileaux.close();
	
	}
	
	static void clear(){
		MyOutFile fileaux("..\\..\\data\\UI_testing\\log.txt");
		fileaux.open();
		fileaux.close();
	}


};


  
#endif
