#ifndef __ELECTRONICS_GRAPH_H__
#define __ELECTRONICS_GRAPH_H__

#include "KinChain.h"
#include "controller.h"
#include <map>

namespace FabByExample{

class ElecEdge;

struct PortType{
public:
	enum TYPE {NONE, POWER_PLUS, POWER_NEG, DIGITAL, ANALOG, PWM};
	
	TYPE type;
	bool isrequired;
	bool isonetoone;

	PortType() {
		type = TYPE::NONE;
		isrequired = false;
		isonetoone = true;
	}

	PortType(TYPE _type, bool _isrequired, bool _isonetoone = true) {
		type = _type;
		isrequired = _isrequired;
		isonetoone = _isonetoone;
	}
};

class ElecNode {

public:
	enum NodeType {NONE, MICROCONTROLLER, BATTERY, ACTUATOR};

protected:
	int id;
	NodeType type;
	KinNode* kinnode;
	std::string name;
	std::map<std::string, PortType> portData;
	std::map<std::string, std::vector<ElecEdge*>> ports; // possibly not one-to-one

public: 
	ElecNode(int _id = 0, std::string _name = "Node");

	void clearPorts();
	int getId() {return id;}
	std::string getName() {return name;}
	NodeType getType() {return type;}
	PortType getPortType(std::string portname) {return portData[portname];}
	bool getConnection(std::string portname, std::vector<ElecEdge*>&);
	void addConnection(std::string portname, ElecEdge*);
	std::vector<std::string> getRequiredPorts();
	std::vector<std::string> getOpenPorts(PortType::TYPE t);
};

class Microcontroller : public ElecNode {
public:
	Microcontroller(int id = 0, std::string _name = "Microcontroller");

	virtual std::string getSoftware(std::string codedir = "..\\..\\data\\code\\sample\\") = 0;
};

class Arduino : public Microcontroller {
public:
	Arduino(int id = 0, std::string _name = "Arduino");

	void writeGaitInfo(std::ofstream& outfile, Arduino* arduino);
	std::string getSoftware(std::string codedir = "..\\..\\data\\code\\sample\\");

};

class Actuator : public ElecNode {
protected:
	std::vector<Controller*> controllers;
public:
	Actuator(int id = 0, std::string _name = "Actuator");

	std::vector<Controller*> getControllers() {return controllers;}
};

class Motor : public Actuator {

public:
	Motor(int id = 0, std::string _name = "Motor");
};

class Servo : public Actuator {

public:
	Servo(int id = 0, std::string _name = "Servo");
};

class ServoPos : public Servo {

public:
	ServoPos(int id = 0, std::string _name = "Servo");
};

class Battery : public ElecNode {

public:
	Battery(int id = 0, std::string _name = "Battery");
};


//------- Electronics Edge ---------//

class ElecEdge {
private:
	std::pair<ElecNode*, ElecNode*> nodes;

	std::vector< std::pair<std::string, std::string> > ports;

public:
	ElecEdge();
	ElecEdge(ElecNode* node1, ElecNode* node2);

	std::pair<ElecNode*, ElecNode*> getNodes() {return nodes;}
	std::vector< std::pair<std::string, std::string> > getPorts() {return ports;}

	bool isValid();
	bool assignPort(std::string, std::string);
	bool removeAssign(std::string, std::string);
	int removeAllAssign(std::string);
	void clearPorts();
};


//------- Electronics Graph ---------//

class ElectronicsGraph {
private:
	KinChain* kinchain;
	std::vector<ElecNode*> nodes;
	std::vector<ElecEdge*> connections;
	std::string codedir;

public:
	// constructors
	ElectronicsGraph();
	ElectronicsGraph(KinChain* _kinchain);

	// methods
	void update();
	std::string getAssemblyInstr();
	void generateCode(std::string codedir = "..\\..\\data\\code\\sample\\");

	int getNumServos();  // get number of servos needed for the robot
	std::vector<Microcontroller*> getAllMicrocontrollers();

	void assignPorts();

	bool isValid();
	void clearPorts();

	void clear();
};

}


#endif // __ELECTRONICS_GRAPH_H__