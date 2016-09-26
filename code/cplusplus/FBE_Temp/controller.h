#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include <iostream>
#include <list>
namespace FabByExample{
		
	class Element_Motion;
	class Template;

	class Controller{
		

	public:
		Controller(){}
		virtual double getVal(double t) =0;
		virtual void display() = 0;
		virtual bool isSymbolic() = 0;
		virtual std::string getMotionInfo() = 0; 
	};


	struct ControllPoint{
		double t;
		double val;
		bool contact;
		bool moving;
		ControllPoint(double _t, double _val, bool _contact, bool _moving){
			t = _t;
			val = _val;
			contact = _contact; 
			moving = _moving;
		}
		ControllPoint(double _t, double _val, bool _contact){
			t = _t;
			val = _val;
			contact = _contact; 
			moving = false;
		}
		ControllPoint(double _t, double _val){
			t = _t;
			val = _val;
			contact = false; 
			moving = false;
		}
		std::string getMotionInfo(){return "";}
	};


	class LinearController: public Controller{
	public:
		LinearController(double _coeff, double _constant);
		double getVal(double t);
		void display();
		bool isSymbolic() {return false;}
		std::string getMotionInfo(){return "";}

	private:
		double coeff;
		double constant;
	};

	class PWLinearController: public Controller{
	public:
		PWLinearController();
		void addPair(double t , double val);
		void pushBackPair(double t , double val);
		void pushBackWithContactInfo(double t , double val, bool contact, bool moving);
		double getVal(double t);
		bool inContact(double t);
		bool isMoving(double t);
		void display();
		bool isSymbolic() {return false;}
		void clear(){controllPoints.clear();}
		std::list<double> getTimeStamps();
		std::list<ControllPoint> controllPoints;
		std::string getMotionInfo();
		double getMaxCycleSpeed();

	private:
		friend class PartialConverters;
	};

	class SymbolicController: public Controller{
	public:
		SymbolicController(Element_Motion* elementMotion);
		void updateSymbols();
		double getVal(double t);
		void display() {};
		Template* getRefTemp();
		bool isSymbolic() {return true;}
		void clearLinearController();
		
		std::list<double> getTimeStamps() {return elementVals->getTimeStamps();}
		PWLinearController* getElementVals() {return elementVals;}
		PWLinearController* getLinearController(){return linearcontroller;}
		Element_Motion * getElementMotion(){return elementMotion;}
		std::string getMotionInfo(){return "";}
		
	private:
		PWLinearController* linearcontroller;
		PWLinearController* elementVals;
		Element_Motion* elementMotion;
	};
	


	class GrammarController: public Controller{
	public:
		enum GrammarcontrollerType{WHEEL, LEG, DOUBLE_SHOULDED, DOUBLE_ELBOW, NONE};
		GrammarController(GrammarcontrollerType _type, double theta, int N_intervals, int i_interval, double multiFactor);
		double getVal(double t){return linearcontroller->getVal(t);}
		void display(){}
		bool isSymbolic() {return false;}
		void updateParameters(double theta, int N_intervals, int i_interval, double multi);
		void updateParametersForInterpolation(double theta1, double theta2);
		void setType(GrammarcontrollerType _type){type = _type;}
		double getTheta(){return theta;}
		int getNintervals(){return N_intervals;}
		int getIinterval(){return i_interval;}
		double getMulti(){return multiFactor;}
		GrammarcontrollerType getType(){return type;}
		PWLinearController* getLinearController(){return linearcontroller;}
		std::string getMotionInfo(){return "";}

	private:
		PWLinearController* linearcontroller;
		GrammarcontrollerType type;
		double theta;
		int N_intervals;
		int i_interval;
		double multiFactor; 

	};

	


}






#endif