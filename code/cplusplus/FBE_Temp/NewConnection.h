#ifndef _NEWCONNECTION_
#define _NEWCONNECTION_

#include <Eigen/Dense>
#include <vector>
#include "symbolic.h"
#include "debugging.h"

namespace FabByExample{

	class Articulation;
	class NewPatch;
	class TemplateElement;
	class Template;

	class NewConnection : public FabDebugging::Debuggable //eventually this becomes a subclass of something else.  By default, this is currently a fold connection.
	{
		
	public:
		enum ConnectionType {FOLD, TEETH, TOUCH, HINGE, BALLJOINT, PRISMATIC, NONE};
		static bool isJoint(ConnectionType testConn) {
			return ((testConn == NewConnection::ConnectionType::HINGE) || 
				(testConn == NewConnection::ConnectionType::BALLJOINT) || 
				(testConn == NewConnection::ConnectionType::PRISMATIC) ||
				(testConn == NewConnection::ConnectionType::NONE));
		}

		NewConnection(std::vector<NewPatch *> patches, double angle, ConnectionType connectionType); 
		~NewConnection(void);
		std::vector<NewPatch *> patches;
		double getAngle(){return angle;}
		void setAngle(double angle){ this->angle = angle; }
		void setType(ConnectionType type){ this->connectionType = type; }
		std::vector<NewPatch *> getPatches(){return patches;}
		std::vector<TemplateElement *> getTemplates();
		ConnectionType getConnectionType(){return connectionType;}
		//ConnectionType getConnectionType(){ return ConnectionType::HINGE; }  //using this hack to test hinges
		virtual void updateParameters(SymbolicAssignment const& env) {}
		virtual Articulation * getArticulation(){return nullptr;}
		std::string toString(){
			std::string res = "connection type: ";
			switch (connectionType){
			case FOLD:
				res += "fold";
				break;
			case TEETH:
				res += "teeth";
				break;
			case TOUCH:
				res += "touch";
				break;
			case HINGE:
				res += "hinge";
				break;
			case BALLJOINT:
				res += "balljoint";
				break;
			case PRISMATIC:
				res += "prismatic";
				break;
			case NONE:
				res += "none";
				break;
			default:
				res += "type not specified";
				break;
			}

			int convertedAngle = int (angle);
			res += " with angle: " ;//+ std::to_string(convertedAngle);
			return res;
		}
		bool isIncomplete();
		FabDebugging::DebugInfo* getDebugInfo();
		bool is2dSeparate;
		bool isRelated(Template * temp);
		bool hasPatch(NewPatch * patch); 
	private:
		double angle;
		ConnectionType connectionType;
	
		DISALLOW_COPY_AND_ASSIGN(NewConnection);
	};


	class JointConnection : public NewConnection{
	public:
		JointConnection(std::vector<NewPatch *> patches, double angle, Articulation * _articulation, NewConnection::ConnectionType connType);
		Articulation * getArticulation(){return articulation;}
		void setArticulation(Articulation* articulation);
		void updateParameters(SymbolicAssignment const& env);
	private:
		Articulation * articulation;

		DISALLOW_COPY_AND_ASSIGN(JointConnection);
	};


}

#endif