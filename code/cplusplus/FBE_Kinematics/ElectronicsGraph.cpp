#include "ElectronicsGraph.h"
#include <map>
#include <Windows.h>
#include <stdlib.h>
#include <fstream>

#define mincodeloc "..\\arduino\\minimal_code\\"
#define numcodefiles 5
std::string mincodefiles[numcodefiles] = {"minimal_code.ino", "gaitinfo.h", "robotLibrary.cpp", "robotLibrary.h", "string_functions.h"};

using namespace FabByExample;

//-------- Electronics Edge ----------//

ElecNode::ElecNode(int _id, std::string _name) : name(_name), id(_id) {type = NodeType::NONE;}

bool ElecNode::getConnection(std::string portname, std::vector<ElecEdge *>& portConnection) {
	if (ports.find(portname) != ports.end()) {
		portConnection = ports[portname];
		return true;
	}
	return false;
}

void ElecNode::addConnection(std::string portname, ElecEdge* e) {
	if (ports.find(portname) != ports.end()) {
		for each (ElecEdge* echeck in ports[portname]) {
			if (echeck == e) {
				return;
			}
		}
		ports[portname].push_back(e);
	} else {
		std::vector<ElecEdge *> v;
		v.push_back(e);
		ports[portname] = v;
	}
}

void ElecNode::clearPorts() {
	for (std::map<std::string, std::vector<ElecEdge*>>::iterator port_it = ports.begin();
		port_it != ports.end(); ++port_it) {
		for each (ElecEdge* e in port_it->second) {
			e->clearPorts();
		}
	}
}

std::vector<std::string> ElecNode::getOpenPorts(PortType::TYPE t) {
	std::vector<std::string> open_ports;
	std::vector<ElecEdge*> v;

	for each (auto p in portData) {
		if (p.second.type == t) {
			if (!(p.second.isonetoone) || !(this->getConnection(p.first,v))) {
				open_ports.push_back(p.first);
			}
		}
	}

	return open_ports;
}

std::vector<std::string> ElecNode::getRequiredPorts() {
	std::vector<std::string> req_ports;

	for each (auto p in portData) {
		if (p.second.isrequired) {
			req_ports.push_back(p.first);
		}
	}

	return req_ports;
}

Microcontroller::Microcontroller(int _id, std::string _name) : ElecNode(_id, _name) {
	portData["+"] = PortType(PortType::TYPE::POWER_PLUS, true, false);
	portData["-"] = PortType(PortType::TYPE::POWER_NEG, true, false);
	type = ElecNode::NodeType::MICROCONTROLLER;
}

Arduino::Arduino(int _id, std::string _name) : Microcontroller(_id, _name) {
	// Arduino Pro Mini
	
	// Digital ports
	portData["D02"] = PortType(PortType::TYPE::DIGITAL, false);
	portData["D03"] = PortType(PortType::TYPE::PWM, false, true); // one-to-one not necessary, but easier for software gen
	portData["D04"] = PortType(PortType::TYPE::DIGITAL, false);
	portData["D05"] = PortType(PortType::TYPE::PWM, false, true); // one-to-one not necessary, but easier for software gen
	portData["D06"] = PortType(PortType::TYPE::PWM, false, true); // one-to-one not necessary, but easier for software gen
	portData["D07"] = PortType(PortType::TYPE::DIGITAL, false);
	portData["D08"] = PortType(PortType::TYPE::DIGITAL, false);
	portData["D09"] = PortType(PortType::TYPE::PWM, false, true); // one-to-one not necessary, but easier for software gen
	portData["D10"] = PortType(PortType::TYPE::PWM, false, true); // one-to-one not necessary, but easier for software gen
	portData["D11"] = PortType(PortType::TYPE::PWM, false, true); // one-to-one not necessary, but easier for software gen
	portData["D12"] = PortType(PortType::TYPE::DIGITAL, false);
	portData["D13"] = PortType(PortType::TYPE::DIGITAL, false);

	// Analog input ports
	portData["A0"] = PortType(PortType::TYPE::ANALOG, false, true);
	portData["A1"] = PortType(PortType::TYPE::ANALOG, false, true);
	portData["A2"] = PortType(PortType::TYPE::ANALOG, false, true);
	portData["A3"] = PortType(PortType::TYPE::ANALOG, false, true);
	portData["A4"] = PortType(PortType::TYPE::ANALOG, false, true);
	portData["A5"] = PortType(PortType::TYPE::ANALOG, false, true);
	portData["A6"] = PortType(PortType::TYPE::ANALOG, false, true);
	portData["A7"] = PortType(PortType::TYPE::ANALOG, false, true);
}

void Arduino::writeGaitInfo(std::ofstream& outfile, Arduino* arduino) {

	int nServos = 0;
	int nPhase = 2;
	int maxWheelPose = 167, minWheelPose = 23;
	int wheelPoseStart = 155;
	std::vector<Controller*> controllers;

	// iterate through connections and grab servos
	for (std::map<std::string, std::vector<ElecEdge*>>::iterator portit = arduino->ports.begin();
		portit != arduino->ports.end(); ++portit) {
			switch (getPortType(portit->first).type) {
			case PortType::TYPE::NONE: {
				} break;
			case PortType::TYPE::POWER_PLUS: {
				} break;
			case PortType::TYPE::POWER_NEG: {
				} break;
			case PortType::TYPE::DIGITAL: {
				} break;
			case PortType::TYPE::ANALOG: {
				} break;
			case PortType::TYPE::PWM: {
				std::vector<ElecEdge*> pwmconnects = portit->second;
				for (int iconnect = 0; iconnect < pwmconnects.size(); ++iconnect) {
					ElecNode* othernode;
					if (pwmconnects[iconnect]->getNodes().first == arduino) {
						othernode = pwmconnects[iconnect]->getNodes().second;
					} else {
						othernode = pwmconnects[iconnect]->getNodes().first;
					}

					std::cout << "FOUND a PWM ";
					if (othernode->getType() == ElecNode::NodeType::ACTUATOR) {
						std::cout << "connected to an actuator ";
						std::vector<Controller*> control = (static_cast<Actuator*>(othernode))->getControllers();
						std::cout << "with " << control.size() << " controllers, ";
						for each (Controller* c in control) {
							std::cout << " which is > 0 ";
							++nServos;
							controllers.push_back(c);

							GrammarController* gc = dynamic_cast<GrammarController*>(c);
							if (gc != NULL) {
								nPhase = gc->getNintervals();
							}
						}
					}
					std::cout << std::endl;
				}
				} break;
			}
	}
	
	outfile << "// GAIT-SPECIFIC VARS" << std::endl;
	outfile << "#define numUsedServos " << nServos << std::endl;
	outfile << "#define numPhase      " << nPhase << std::endl;
	outfile << "#define minWheelPose  " << minWheelPose << std::endl;
	outfile << "#define maxWheelPose  " << maxWheelPose << std::endl;
	outfile << std::endl;

	outfile << "const int TYPE[numUsedServos]     = {";
	for (int i = 0; i < nServos; ++i) {
		if (i > 0) {
			outfile << ", ";
		}
		switch (dynamic_cast<GrammarController*>(controllers[i])->getType()) {
		case GrammarController::GrammarcontrollerType::LEG: {
			outfile << "SINGLE";
			} break;
		case GrammarController::GrammarcontrollerType::WHEEL: {
			outfile << "WHEEL";
			} break;
		case GrammarController::GrammarcontrollerType::DOUBLE_ELBOW: {
			outfile << "DOUBLE";
			} break;
		case GrammarController::GrammarcontrollerType::DOUBLE_SHOULDED: {
			outfile << "DOUBLE";
			} break;
		}
	}
	outfile << "};" << std::endl;

	outfile << "const int phases[numUsedServos]   = {";
	for (int i = 0; i < nServos; ++i) {
		if (i > 0) {
			outfile << ", ";
		}
		outfile << dynamic_cast<GrammarController*>(controllers[i])->getIinterval();
	}
	outfile << "};" << std::endl;

	outfile << "const int wheelposediff[numUsedServos] = {";
	for (int i = 0; i < nServos; ++i) {
		if (i > 0) {
			outfile << ", ";
		}
		GrammarController* c = dynamic_cast<GrammarController*>(controllers[i]);
		switch (c->getType()) {
		case GrammarController::GrammarcontrollerType::LEG: {
			outfile << 0;
			} break;
		case GrammarController::GrammarcontrollerType::DOUBLE_ELBOW: {
			outfile << 0;
			} break;
		case GrammarController::GrammarcontrollerType::DOUBLE_SHOULDED: {
			outfile << 0;
			} break;
		case GrammarController::GrammarcontrollerType::WHEEL: {
			double theta_prop = 2*c->getTheta()/360.;
			int wheeldiff = (int)( theta_prop * (maxWheelPose - minWheelPose) );
			outfile << wheeldiff;
			} break;
		}
	}
	outfile << "};" << std::endl;

	outfile << std::endl;

	outfile << "volatile int STATE[numUsedServos] = {";
	for (int i = 0; i < nServos; ++i) {
		if (i > 0) {
			outfile << ", ";
		}
		outfile << "BACKMOVE";
	}
	outfile << "};" << std::endl;

	outfile << "volatile int valback[numUsedServos];" << std::endl;

	//const int forpos[numUsedServos]  = {100, 150, 100, 100};
	//const int retrpos[numUsedServos] = {100,  80, 200, 100};
	//const int midpos[numUsedServos]  = {125, 175, 200, 125};
	//const int backpos[numUsedServos] = {150, 200, 100, 150};

	outfile << "volatile int wheelposes[numUsedServos] = {";
	for (int i = 0; i < nServos; ++i) {
		if (i > 0) {
			outfile << ", ";
		}
		outfile << wheelPoseStart;
	}
	outfile << "};" << std::endl;

}

std::string Arduino::getSoftware(std::string codedir) {
	/*
	char fname[_MAX_FNAME];
	_splitpath(codedir.c_str(), NULL, NULL, fname, NULL );
	std::cout << codedir << std::endl;
	std::cout << fname << std::endl;

	TCHAR pwd[MAX_PATH];
	GetCurrentDirectory(MAX_PATH,pwd);
	std::wcout << pwd << std::endl;
	*/

	std::stringstream minfile;
	std::stringstream destfile;

	for (int i = 0; i < numcodefiles; ++i) {
		minfile.clear();
		minfile.str("");
		minfile << mincodeloc << mincodefiles[i].c_str();
		
		destfile.clear();
		destfile.str("");
		destfile << codedir.c_str() << mincodefiles[i].c_str();
		
		//std::cout << mincodefiles[i] << std::endl;
		//std::cout << minfile.str().c_str() << std::endl;
		//std::cout << destfile.str().c_str() << std::endl;

		// Copy necessary files:
		CopyFileA(minfile.str().c_str(), destfile.str().c_str(), false);
	}

	destfile.clear();
	destfile.str("");
	destfile << codedir.c_str() << "gaitdef.h";

	std::cout << "Writing gait info to: " << destfile.str().c_str() << std::endl;
	std::ofstream gaitfile;
	gaitfile.open(destfile.str().c_str(), std::ofstream::out);
	writeGaitInfo(gaitfile, this);
	gaitfile.close();

	// RETURN CODEDIR CONTENTS
	return codedir;
}

Actuator::Actuator(int _id, std::string _name) : ElecNode(_id, _name) {
	portData["+"] = PortType(PortType::TYPE::POWER_PLUS, true, false);
	portData["-"] = PortType(PortType::TYPE::POWER_NEG, true, false);
	type = ElecNode::NodeType::ACTUATOR;
}

Motor::Motor(int _id, std::string _name) : Actuator(_id, _name) {
}

Servo::Servo(int _id, std::string _name) : Actuator(_id, _name) {
	portData["in"] = PortType(PortType::TYPE::PWM, true, false);
}
ServoPos::ServoPos(int _id, std::string _name) : Servo(_id, _name) {
	portData["out"] = PortType(PortType::TYPE::ANALOG, true, false);
}
Battery::Battery(int _id, std::string _name) : ElecNode(_id, _name) {
	portData["+"] = PortType(PortType::TYPE::POWER_PLUS, true, false);
	portData["-"] = PortType(PortType::TYPE::POWER_NEG, true, false);
	type = ElecNode::NodeType::BATTERY;
}


//-------- Electronics Edge ----------//

ElecEdge::ElecEdge() {}

ElecEdge::ElecEdge(ElecNode* node1,  ElecNode* node2): nodes(node1, node2), ports() { ports.clear(); }

bool ElecEdge::isValid() {
	ElecNode* node1 = nodes.first;
	ElecNode* node2 = nodes.second;

	// check all port connection types are valid
	for each (std::pair<std::string, std::string> p in ports) {
		if (node1->getPortType(p.first).type != node2->getPortType(p.second).type) {
			return false;
		}
	}

	return true;
}

bool ElecEdge::assignPort(std::string p1, std::string p2) {	
	this->ports.push_back(std::pair<std::string,std::string>(p1,p2));
	return true;
}

bool ElecEdge::removeAssign(std::string p1, std::string p2) {
	// remove connection between port p1 and port p2
	int i = 0;
	bool isfound = false;
	while (i < ports.size()) {
		if (strcmp(ports[i].first.c_str(), p1.c_str()) && strcmp(ports[i].second.c_str(), p2.c_str())) {
			ports.erase(ports.begin()+i);
			isfound = true;
		} else {
			++i;
		}
	}
	return isfound;
}

int ElecEdge::removeAllAssign(std::string p1) {
	// remove all port assignments to port p1
	int i = 0;
	int count = 0;
	while (i < ports.size()) {
		if (strcmp(ports[i].first.c_str(), p1.c_str())) {
			ports.erase(ports.begin()+i);
			++count;
		} else {
			++i;
		}
	}
	return count;
}

void ElecEdge::clearPorts() {
	ports.clear();
}


//-------- Electronics Graph ----------//

ElectronicsGraph::ElectronicsGraph() {}

ElectronicsGraph::ElectronicsGraph(KinChain* _kinchain) {
	kinchain = _kinchain;
	
	// traverse the kinchain to extract all the actuator data
	this->update();
}

void ElectronicsGraph::update() {
	int nServos = 0;
	int nBattery = 0;
	int nController = 0;

	// check if kinchain has changed

	// count parts
	for each (ElecNode* n in nodes) {
		switch (n->getType()) {
		case ElecNode::NodeType::MICROCONTROLLER: {
			++nController;
			} break;
		case ElecNode::NodeType::BATTERY: {
			++nBattery;
			} break;
		case ElecNode::NodeType::ACTUATOR: {
			++nServos;
			} break;
		}
	}

	int reqServos = kinchain->totalDegreesOfFreedom();
	int reqController = 1; // ceil((double)(reqServos)/6);
	int reqBattery = 1;

	if (reqServos != nServos) {
		clear();

		// call me when changing the controller: update the electronics
		for (int i = 0; i < reqController; ++i) {
			this->nodes.push_back(new Arduino(i));
		}
		for (int i = 0; i < reqBattery; ++i) {
			this->nodes.push_back(new Battery(i));
		}
		for (int i = 0; i < reqServos; ++i) {
			this->nodes.push_back(new ServoPos(i));
		}
	}

	if (!isValid()) {
		clearPorts();
		assignPorts();
	}
}

bool ElectronicsGraph::isValid() {
	std::vector<ElecEdge*> edgeToCheck;

	std::vector<std::pair<ElecNode*, std::string>> usedPorts;

	for each (ElecNode* n in nodes) {
		std::vector<std::string> req_ports = n->getRequiredPorts();

		for each (std::string portname in req_ports) {
			edgeToCheck.clear();
			if (!(n->getConnection(portname, edgeToCheck))) {
				// if all node's required ports are assigned
				return false;
			} else if (!(edgeToCheck[0]->isValid())) {
				// and assignments are valid
				return false;
			}
		
			// check for one-to-one connections
			if (n->getPortType(portname).isonetoone && edgeToCheck.size() > 1) {
				return false;
			}
		}
	}

	return true;
}

void ElectronicsGraph::clearPorts() {
	for each (ElecEdge* e in connections) {
		e->clearPorts();
	}
}

void ElectronicsGraph::assignPorts() {

	// greedy alg:

	std::vector<Microcontroller*> allM = this->getAllMicrocontrollers();

	for each (ElecNode* n in nodes) {
		if (n->getType() == ElecNode::NodeType::MICROCONTROLLER) {
			continue;
		}
		//std::cout << "Assigning to " << n->getName() << ", ports : ";
		std::vector<std::string> ptoconnect = n->getRequiredPorts();
		for each (std::string s in ptoconnect) {
			PortType::TYPE pt = n->getPortType(s).type;

			//std::cout << s << ", ";

			bool connected = false;

			// connect it to the microcontroller
			for each (Microcontroller* m in allM) {
				std::vector<std::string> openports = m->getOpenPorts(pt);

				if (!openports.empty()) {
					// find out if this ElecEdge already exists
					ElecEdge* econnect = NULL;
					for each (ElecEdge* e in this->connections) {
						// it exists!
						if ((e->getNodes().first == n && e->getNodes().second == m) ||
							(e->getNodes().second == n && e->getNodes().first == m)) {
								econnect = e;
								break;
						}
					}
					if (econnect == NULL) {
						// create a new edge
						econnect = new ElecEdge(n, m);
						this->connections.push_back(econnect);
					}

					int i = 0;
					while (i<openports.size()) {
						if (econnect->assignPort(s, openports[i])) {
							n->addConnection(s, econnect);
							m->addConnection(openports[i], econnect);
							connected = true;
							break;
						} else {
							++i;
						}
					}
					if (connected) {
						break;
					}
				}
			}
		}
		//std::cout << std::endl;
	}

}

void ElectronicsGraph::clear() {

	this->clearPorts();

	for each (ElecNode* n in nodes) {
		n->clearPorts();
	}
	this->nodes.clear();

}

std::string ElectronicsGraph::getAssemblyInstr() {

	this->update();

	std::stringstream info;

	info << "PARTS LIST:\n";
	
	// traverse the graph for nodes : combine like parts into a dictionary
	std::map<std::string, int> partslist;
	for each (ElecNode* node in nodes) {
		partslist[node->getName()] = partslist[node->getName()] + 1;
	}

	// output parts list
	for each (auto part in partslist) {
		info << "  " << part.second << " x  " << part.first << "\n";
	}

	info << "\n\nCONNECTIONS:\n" ;
	// output connections
	for each (ElecEdge* connect in connections) {
		std::string node1type = connect->getNodes().first->getName();
		int node1id = connect->getNodes().first->getId();
		std::string node2type = connect->getNodes().second->getName();
		int node2id = connect->getNodes().second->getId();
		for each (std::pair<std::string, std::string> p in connect->getPorts()) {
			info << "Connect " << node1type << " " << node1id << ":" << p.first;
			info << " to " << node2type << " " << node2id << ":" << p.second << std::endl;
		}
	}

	info << "\n\n Code location: ";
	info << codedir;

	return info.str();
}

int ElectronicsGraph::getNumServos() {
	int totalServos = 0;

	for (std::vector<ElecNode*>::iterator it = nodes.begin(); it != nodes.end(); ++it) {
		if ((*it)->getType() == ElecNode::NodeType::ACTUATOR) {
			totalServos++;
		}
	}

	return totalServos;
	
}


std::vector<Microcontroller*> ElectronicsGraph::getAllMicrocontrollers() {
	std::vector<Microcontroller*> allMicros;

	for (std::vector<ElecNode*>::iterator it = nodes.begin(); it != nodes.end(); ++it) {
		if ((*it)->getType() == ElecNode::NodeType::MICROCONTROLLER) {
			Microcontroller* it_m = static_cast<Microcontroller*>(*it);
			allMicros.push_back(it_m);
		}
	}

	return allMicros;
	
}

void ElectronicsGraph::generateCode(std::string codedir)
{
	this->codedir = codedir;

	if (CreateDirectoryA(codedir.c_str(), NULL) ||
		ERROR_ALREADY_EXISTS == GetLastError())
	{
		for (std::vector<ElecNode*>::iterator it = nodes.begin(); it != nodes.end(); ++it) {
			if ((*it)->getType() == ElecNode::NodeType::MICROCONTROLLER) {
				((Microcontroller*)(*it))->getSoftware(codedir);
			}
		}
	}
}