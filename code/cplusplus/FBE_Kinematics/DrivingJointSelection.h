#pragma once

#include <vector>
#include <Eigen/Dense>

namespace FabByExample{

class KinNode; 
class KinChain;
class KinNode_Part;
class KinNode_Joint;

class KinDriverAbstraction{
protected:
	
	std::vector<KinDriverAbstraction*> connectedNodes;
	int id; 
public:
	
	KinDriverAbstraction(int _id){ id = _id;}
	std::vector<KinDriverAbstraction*> getFirstNeighbours();
	std::vector<KinDriverAbstraction*> getSecondNeighbours();
	void addConnection(KinDriverAbstraction* abs){connectedNodes.push_back(abs);}
	int getId(){return id;}
	virtual bool discoverInformation() =0;
	virtual bool isFullyDetermined() = 0;
	virtual bool iAmAPartAndHavePosition() =0;
	virtual bool iAmAJointAndHavePosition() =0;
	virtual bool iAmAJointAndHaveAngle() =0;
	virtual bool makeMeADriver() = 0;
	virtual void fixInitialPosition() = 0; 
	virtual void printConnections() =0;
	virtual void printInfo()= 0;
};


class KinDriverAbstraction_Part: public KinDriverAbstraction{
private:

	bool isDetermined_position;
	KinNode_Part * node;

public:	

	KinDriverAbstraction_Part(KinNode_Part * _node, int _id);
	bool discoverInformation();
	bool isFullyDetermined();


	bool iAmAPartAndHavePosition() {return isDetermined_position;}
	bool iAmAJointAndHavePosition() {return false;}
	bool iAmAJointAndHaveAngle() {return false;}
	bool makeMeADriver() {return false;}
	void fixInitialPosition(){isDetermined_position = true;}
	void printConnections();
	void printInfo();
};


class KinDriverAbstraction_Joint: public KinDriverAbstraction{

private:
	KinNode_Joint * node;
	bool isDetermined_angle;
	bool isDetermined_position;
	bool isDriving;
	bool canBeSetToDriving;
public:


	KinDriverAbstraction_Joint(KinNode_Joint* _node, bool _canBeSetToDriving, int _id);
	bool discoverInformation();
	bool isFullyDetermined();

	bool iAmAPartAndHavePosition() {return false;}
	bool iAmAJointAndHavePosition() {return isDetermined_position;}
	bool iAmAJointAndHaveAngle() {return isDetermined_angle;}
	bool makeMeADriver();
	void fixInitialPosition(){throw "Cannot fix for joint";}
	void printConnections();
	void printInfo();
};


class DrivingJointSelection{
public:

	static int computeDrivingJoints(KinChain* _initialChain);
};



}





