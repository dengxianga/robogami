#ifndef __KIN_CHAIN_H__
#define __KIN_CHAIN_H__

#include <vector>
#include <list>
#include <Eigen/Dense>
#include "TriMesh.h" 
namespace FabByExample{

class TemplateElement;
class Template;
class Geometry;
class Articulation;
class NewPatch;
class CenterOfMass;
class Geometry; 
class ContactPoint;



struct DrawableGaitOptions{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> polygon;
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>  legPositions;
	std::vector<std::vector<std::pair<int, double>>> gaitOptions;
};







class KinRigidTransform{
public:
	Eigen::Vector3d normal;
	Eigen::Vector3d trans;
	Eigen::Matrix3d rot;
	KinRigidTransform();
	void clear();
	void apply( Geometry * geo, bool updatenormals = true);
	void apply( TriMesh * mesh, bool updatenormals = true);
	void transform(Eigen::Vector3d & point);
	void transform(std::vector<Eigen::Vector3d> & point);
	void transformPoint(point & p);
	void combine(KinRigidTransform& newTransf);
	void display();
};


class KinNode{
public:

	std::vector<KinNode*> children;
	KinNode* parent;

	enum  KinNodeType {PART, JOINT};
	
	//constructors
	KinNode(){
		parent = nullptr;
	}
	~KinNode(){
		children.clear(); 
	}
	
	//virtual methods
	virtual KinNodeType getType() =0;
	virtual void display(int N, bool doTree) = 0;
	virtual void addGeometry(Geometry* geo, bool withnormals = true) =0;
	virtual void addCenterOfMass(CenterOfMass & centerOfMass) = 0;
	virtual void updateTime(double t) = 0;
	virtual bool isSame(KinNode * node) = 0;
	virtual CenterOfMass getCenter() = 0;
	virtual std::vector<Eigen::Vector3d> getAllAttachmentPoints() = 0;
	virtual void collectControlThetas(std::vector<double>& allthetas, std::vector<bool>& iscontact, std::vector<bool>& ismoving, double t) = 0;
	virtual void broadcastControlThetas(std::list<double>& allthetas, double t) = 0;
	virtual void transformToRootFrame(std::vector<point>& pts) = 0;
	virtual void transformToRootFrame(Geometry* geo) = 0;
	virtual void transformToRootFrame(TriMesh* mesh) = 0;
	virtual void getAllTemplatesFromUnrestrictedNodes(Template* restrictedTemp, std::vector<Template*> templates) = 0; 
	virtual void getAllContactPoints(std::vector<ContactPoint*> & contactPointsOut) = 0;
	void getAllDescendants(std::vector<KinNode*> & allDescendents){
		for each (auto c in children){
			allDescendents.push_back(c);
			c->getAllDescendants(allDescendents);
		}
	}
	//other methods
	void addChild(KinNode* child);
	void replaceChild(KinNode* child_old, KinNode* child_new);
	void removeChild(KinNode * child);
	bool hasNoParent(){
		if (parent == nullptr){
			return true;
		}
		return false; 
	}
	std::vector<KinNode*> getNeighbors();

};


class KinNode_Part: public KinNode{
public:

	std::vector<TemplateElement*> elements;
	std::vector<ContactPoint*> contactpoints;
	//constructors
	KinNode_Part();
	KinNode_Part(TemplateElement* el);
	~KinNode_Part(){elements.clear();}

	//virtual methods
	KinNodeType getType() {return KinNode::KinNodeType::PART;}
	void display(int N, bool doTree);	
	void addGeometry(Geometry* geo, bool withnormals = true);
	void addCenterOfMass(CenterOfMass & centerOfMass);
	CenterOfMass getCenter();
	std::vector<Eigen::Vector3d> getAllAttachmentPoints();
	void collectControlThetas(std::vector<double>& allthetas, std::vector<bool>& iscontact, std::vector<bool>& ismoving, double t);
	void broadcastControlThetas(std::list<double>& allthetas, double t);
	void transformToRootFrame(std::vector<point>& pts);
	void transformToRootFrame(Geometry* geo);
	void transformToRootFrame(TriMesh* mesh);
	void getAllTemplatesFromUnrestrictedNodes(Template* restrictedTemp, std::vector<Template*> templates); 
	void getAllContactPoints(std::vector<ContactPoint*> & outContacts) {
		for each (auto c in contactpoints){
			outContacts.push_back(c);
		}
		for each (auto c in children){
			c->getAllContactPoints(outContacts);
		}
	}
	void updateTime(double t){};
	//other methods
	bool checkContainsElement(int elID);
	void add(KinNode_Part* part);
	bool isSame(KinNode * node){return false;}
	
	Geometry* getCurrentGeoRelativeToRoot();

};

class KinNode_Joint: public KinNode{
private:
	Articulation* articulation;

public:
	bool isDriving;

	//constructors
	KinNode_Joint();
	KinNode_Joint(Articulation* _articulation);

	void reset();
	void resetControllers();

	//virtual methods
	KinNodeType getType() {return KinNode::KinNodeType::JOINT;}
	void display(int N, bool doTree);	
	void addGeometry(Geometry* geo, bool withnormals = true);
	CenterOfMass getCenter();
	std::vector<Eigen::Vector3d> getAllAttachmentPoints();
	void collectControlThetas(std::vector<double>& allthetas, std::vector<bool>& iscontact, std::vector<bool>& ismoving, double t);
	void broadcastControlThetas(std::list<double>& allthetas, double t);
	void transformToRootFrame(std::vector<point>& pts);
	void transformToRootFrame(Geometry* geo);
	void transformToRootFrame(TriMesh* mesh);
	void getAllTemplatesFromUnrestrictedNodes(Template* restrictedTemp, std::vector<Template*> templates){} 

	void addCenterOfMass(CenterOfMass & centerOfMass);
	void updateTime(double t);
	Articulation* getArticulation() { return articulation; }
	bool isSame(KinNode * node);
	void getAllContactPoints(std::vector<ContactPoint*> & outContacts) {
		for each (auto c in children){
			c->getAllContactPoints(outContacts);
		}
	}
	//other methods
	
};

struct GaitInfo{
public:
	std::vector<std::pair<int, double>> jointInfo;
	std::string name;
	double desiredDirection;
	GaitInfo(){
		name = "gait";
		desiredDirection = 0; 
	}
};


struct GaitInterpInfo{
public:
	bool isInterp;
	int gait1;
	int gait2;
	int gait;
	GaitInterpInfo(){
		gait = -1;
		gait1 = -1;
		gait2 = -1;
	}
	GaitInterpInfo (int id){
		isInterp = false;
		gait = id;
	}
	GaitInterpInfo (int id1, int id2){
		isInterp = true;
		gait1 = id1;
		gait2 = id2;
	}
};

class KinChain{
private:
	KinNode * root;
	
	double dt;
	int currentGaitID;
	int rounds; // input from user 
	int nrounds; // how many rounds needed for steady state computation
	std::pair<int, int> steadyStateRounds;
	std::list<KinNode*> nodes;
	std::vector<double> slipCost;
	std::vector<double> stabilityCost;
	std::vector<KinRigidTransform> rootPose;
	KinRigidTransform currentRootPose;
	void resetDrivers();
	void getTranformation(Geometry * geo, KinRigidTransform & rtrans, std::vector<Eigen::Vector3d> & points);
	bool rotateToGround(Geometry * geo, CenterOfMass & com, KinRigidTransform & rtrans, std::vector<Eigen::Vector3d> & points);
	//gait options
	DrawableGaitOptions gaitOptions;
	std::vector<GaitInfo*> savedGaits;
	std::vector<int> gaitSequence; 
	std::vector<GaitInterpInfo> gaitInterpolationSequence;
public:
	enum WhichContacts {CONTACT, NONCONTACT, ALL, LEGS, WHEELS};

	void resetControllers();
	void setRounds(int rounds=1);
	std::pair<int,int> getSteadyStateRounds() {return this->steadyStateRounds;}
	std::pair<int, int> getSteadyStateIdx();

	KinChain(Template* temp);
	KinNode* find(TemplateElement* tempEl);
	
	void updateTime(double t);
	void updateCurrentPose(double t);
	KinRigidTransform getPoseAtT(double t);
	std::list<KinNode*> & getNodes() {return nodes;}
	void clearGaitInfo();
	void updateGait(double delta, bool doSequence=false); // linkage version
	
	Geometry * getGeometry(bool withnormals = true);
	CenterOfMass getCenterOfMass();
	KinNode* getRoot() { return root; }
	KinRigidTransform getSSTransform();
	KinRigidTransform getCurrentRootPose() {return currentRootPose;}
	std::pair<double, int> adjustTimeToSS(double t);
	double getSSRotation();       // angle turned in one gate cycle in steadu state
	Eigen::Vector3d getSSTranslation();    // steady state velocity vector
	
	// METRICS
	double getMeanCurvature(); // mean curvature of path in steady state (in 1/m)
	Eigen::Vector2d getCenterOfRotation();
	double getMeanAngle();     // mean angle of path in steady state (in rad)
	double getStability();     // steady state worst-case stability
	double getSlip(bool doSequence = false);          // steady state average slip over time (in mm)
	double getSpeed(bool doSequence =false);         // steady state speed along curved path (in mm/s)
	double getTimeOfOneCycle(); 
	double getPerpendicularError();  // steady state average perpendular SOS error from circular path (in mm) over time
	double getForwardTravel(double angle = 0); // forward travel during one steady state gait cycle, projected onto path at user-inputted angle
	double getToppling(bool doSequence =false);      // steady time average wobbliness over time
	double getObjectiveParam(int id, bool doSequence, double desiredCurvature);
	std::vector<std::pair<double,double>> getPath();   // get path from 0 to rounds
	std::vector<std::pair<double,double>> getSSPath(bool doSequence); // get path in steady state
	std::vector<Geometry*> getAnimation( bool useSState=false, int ratio=1, bool doSequence=false);  // get animation from 0 to rounds
	std::vector<double> getStabilityVector() {return stabilityCost;}
	std::vector<double> getSlipVector() {return slipCost;}
	int totalDegreesOfFreedom();

	void getNodeFromPatch(NewPatch * patch, KinNode_Part** node);
	KinNode_Part * getNodeFromTemplate(Template* temp); 
	//KinNode* getNodeThatBelongsToPoint(p1);
	void changeRoot(KinNode* newRoot);
	void removeReduntantJoints();
	void fixLockedJoints();

	std::list<KinNode*> getLeaves();
	std::vector<Eigen::Vector3d> getAllContactPoints(double t, WhichContacts incontact);
	std::vector<double> getTimeStamps();
	void collectControlThetas(std::vector<double>& allthetas, std::vector<bool>& iscontact, std::vector<bool>& ismoving, double t);
	void broadcastControlThetas(std::list<double>& allthetas, double t);
	void displayNodes();
	void displayTree();
	
	//gait specification functions
	void computeGaitOptions();
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> getJointPositiont();
	DrawableGaitOptions & getDrawableGaitOptions() {return gaitOptions;}
	std::vector<std::pair<int, double>> getGaitSpecification( int gaitID){ return savedGaits[gaitID]->jointInfo;}
	std::vector<std::string> getListofSavedGaits();
	double getDesiredDirection(int gaitId){ return savedGaits[gaitId]->desiredDirection;}
	void setDesiredDirection(int gaitId, double dir){ savedGaits[gaitId]->desiredDirection =dir;}
	void updateGaitName(int gaitId, std::string name){savedGaits[gaitId]->name = name;}
	void createNewGaitOptionFromSuggestion( int id);
	void updateControllers(int gaitID);
	void outputMotionToFile(std::string filename); 
	void updateGaitSingleJoint(int gaitID, int i , bool isup); 
	void updateGaitSingleAngle(int gaitID, int i,  double theta); 
	GaitInfo * getGaitInfo(){ return savedGaits[currentGaitID];}
	GaitInfo * getGaitInfo(int id){ return savedGaits[id];}
	void addNewGaitInfo(GaitInfo* gaitInfo){savedGaits.push_back(gaitInfo);} 
	int getNSavedGaits() {return savedGaits.size();}
	void updateSequence(std::vector<int> _sequence);
	std::vector<int> getSequence(){return this->gaitSequence;}
	void updateControllersForInterpolationCase(int gait1, int gait2); 

	void clearSavedGaits(){savedGaits.clear();}
	void updateControllersWithInterpolation(GaitInterpInfo gaitInfo);
	std::vector<double> getTimesPerCycle(bool doSequence);
	std::vector<int> getFramesPerCycle(bool doSequence);
	double getTotalTimeSequence(); 
	std::vector<int> getInterpGaitSequenceForDrawingPath(); 

};

}





#endif // __SINGLE_SAMPLE_H__