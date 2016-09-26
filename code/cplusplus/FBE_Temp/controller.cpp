#include "controller.h"
#include "element_motion.h"
#include "templateElement.h"
#include "debugging.h"

using namespace FabByExample;


LinearController::LinearController(double _coeff, double _constant){
	coeff = _coeff;
	constant = _constant;
}
double LinearController::getVal(double t){
	return coeff * t + constant;
}


void LinearController::display(){
	VLOG(3) << "linear Controller: " << coeff << "*t + " << constant;
}
//--------------------------------------------------------------------------------


PWLinearController::PWLinearController(){
//	ControllPoint initPoint(0, 0);
//	controllPoints.push_back(initPoint);
//	ControllPoint endPoint(1.0, 0);
//	controllPoints.push_back(endPoint);
}

void PWLinearController::addPair(double t , double val){
	std::list<ControllPoint>::iterator it, it1, it2;
	//step1: wheck if is part of existent point;
	bool pointExists = false;
	for(it = controllPoints.begin(); it != controllPoints.end(); ++it){
		if(abs(it->t - t) < 1e-6){ // tolerance for equality
			it->val = val;
			pointExists = true;
		}
	}
	if(!pointExists){
		it = controllPoints.begin();
		if (it->t > t) 
		{
			std::list<ControllPoint>::iterator itlast = controllPoints.end(); --itlast;
			ControllPoint newPoint(t, val, itlast->contact, itlast->moving);
			controllPoints.insert(it2, newPoint);
			pointExists = true;
		}

		++it;
		for(; it != controllPoints.end() && !pointExists; ++it){
			it1 = it; --it1;
			it2 = it;
			if((it1->t <= t) && (it2->t > t)){
				ControllPoint newPoint(t, val, it1->contact, it1->moving);
				controllPoints.insert(it2, newPoint);
				pointExists = true;
			}
		}
	}
	if(!pointExists){
		// add to end
		ControllPoint newPoint(t, val, it2->contact, it2->moving);
		controllPoints.insert(controllPoints.end(), newPoint);
		pointExists = true;
			//LOG(ERROR) << "error: could not find insert a point at t = " << t;
	}
}

void PWLinearController::pushBackPair(double t , double val){
	controllPoints.push_back(ControllPoint(t,val));	
}


std::string PWLinearController::getMotionInfo(){
	
	std::stringstream info;
	for each (auto c in controllPoints){
		info <<"t =  " << c.t << "     pos " << c.val <<"\n" ;
	}
	return info.str();
}


void PWLinearController::pushBackWithContactInfo(double t , double val, bool _contact, bool _moving){
	controllPoints.push_back(ControllPoint(t,val, _contact, _moving));	
}

double PWLinearController::getVal(double t){


	if( t <= 0.005){
		return controllPoints.begin()->val;
	}
	std::list<ControllPoint>::iterator it, it1, it2;
	it = controllPoints.begin();
	if (it->t > t) 
	{
		return it->val;
	}
		
	++it;
	for(; it != controllPoints.end(); ++it){
		it1 = it; --it1;
		it2 = it;
		if((it1->t <= t) && (it2->t > t)){
			double alpha = (t - it1->t)/(it2->t - it1->t); 
			double val = it1->val +alpha*(it2->val - it1->val); 
			return val;
		}
	}
	
	// no linear interpolation because t_begin=0 should be equal to t_end=1
	//LOG(ERROR) << "error: could not find a point to interp for t = " << t;
	it2 = controllPoints.end();
	it2--;
	return  it2->val;
}

bool PWLinearController::inContact(double t){
	std::list<ControllPoint>::iterator it, it1, it2;
	it = controllPoints.begin();
	
	++it;
	for(; it != controllPoints.end(); ++it){
		it1 = it; --it1;
		it2 = it;
		if((it1->t <= t) && (it2->t > t)){
			return it1->contact;
		}
	}
	// out of range --> between end and beginning looparound
	return controllPoints.begin()->contact;
	//LOG(ERROR) << "error: could not find a point to interp for t = " << t;
	return 0.0;
}

bool PWLinearController::isMoving(double t){
	std::list<ControllPoint>::iterator it, it1, it2;
	it = controllPoints.begin();
	
	++it;
	for(; it != controllPoints.end(); ++it){
		it1 = it; --it1;
		it2 = it;
		if((it1->t <= t) && (it2->t > t)){
			return it1->moving;
		}
	}
	// out of range --> between end and beginning looparound
	return it2->moving;
	//LOG(ERROR) << "error: could not find a point to interp for t = " << t;
	return 0.0;
}

void PWLinearController::display(){
	VLOG(3) << "piecewise linear Controller: ";
	std::list<ControllPoint>::iterator it;
	for(it = controllPoints.begin(); it != controllPoints.end(); ++it){
		VLOG(3) << "t = " << it->t << " val = " << it->val << " contact = " << it->contact << " val = " << it->moving;
	}
}

std::list<double> PWLinearController::getTimeStamps(){
	std::list<double> timestamps;
	std::list<ControllPoint>::iterator it;
	for(it = controllPoints.begin(); it != controllPoints.end(); ++it){
		timestamps.push_back((*it).t);
	}
	return timestamps;
}


double PWLinearController::getMaxCycleSpeed(){

	std::list<ControllPoint>::iterator it1, it2;
	double  maxSpeed = 0;
	for(it1 = controllPoints.begin(); it1 != controllPoints.end(); ++it1){
		it2 = it1;
		it2++;
		if(it2 != controllPoints.end()){
			double delta_time = it2->t - it1->t;
			double delta_angle = abs(it2->val - it1->val);
			if(abs(delta_angle) > 0.001){
				double thisSpeed = delta_angle/delta_time;
				if(thisSpeed > maxSpeed){
					maxSpeed = thisSpeed;
				}
			}
		}
	}

	return maxSpeed;

}

//-------------------------------------------------------------------------------------------


SymbolicController::SymbolicController(Element_Motion* _elementMotion){
	elementMotion = _elementMotion;
	linearcontroller = elementMotion->eval();
	elementVals = elementMotion->eval();
}

void SymbolicController::clearLinearController() {
	linearcontroller = elementMotion->eval();
}


void SymbolicController::updateSymbols(){
	delete elementVals;
	elementVals = elementMotion->eval();
	delete linearcontroller;
	linearcontroller = elementMotion->eval();
}

double SymbolicController:: getVal(double t){
	return linearcontroller->getVal(t); 
}
	


Template* SymbolicController::getRefTemp(){
	return elementMotion->getRefTemplateElement();
}


GrammarController::GrammarController(GrammarcontrollerType _type, double _theta, int _N_intervals, int _i_interval, double _multiFactor){

	type = _type;
	theta = _theta;
	N_intervals = _N_intervals;
	i_interval = _i_interval;
	multiFactor = _multiFactor; 
	linearcontroller = new PWLinearController();
	updateParameters(theta, N_intervals, i_interval, multiFactor);

}


void GrammarController::updateParameters(double _theta, int _N_intervals, int _i_interval, double multi){





	multiFactor = sgn(multi);
	theta = _theta;
	N_intervals = _N_intervals;
	i_interval = _i_interval;


	double usingMultiFactor = multiFactor; 

	if (_theta < 0){
		theta = -1.0 *theta;
		usingMultiFactor = -1.0 * usingMultiFactor; 
	}


	linearcontroller->clear(); 

	if(abs(theta) < 0.001){
		linearcontroller->pushBackWithContactInfo(0, 0, true, false);
		linearcontroller->pushBackWithContactInfo(1, 0, true, true);
	}
	else{



	double trans_time = theta/(N_intervals*(180-theta) + theta);
	double reset_time = (180.0 - theta)/(N_intervals*(180.0-theta) + theta);
	

	double 	theta_r = M_PI/180.0*theta;
	
	double theta1 = theta_r;
	double theta2 = 2*M_PI- theta_r;
	double theta3 = 2*M_PI + theta_r;
	if(usingMultiFactor <0){
		theta1 = 2*M_PI -theta_r;
		theta2 = theta_r;
		theta3 = -theta_r;
	}
	double reset_time_init = i_interval*reset_time;
	double reset_time_end = (i_interval+1)*reset_time;
	double reset_time_mid1 = reset_time_init + (reset_time_end - reset_time_init)/3.0;
	double reset_time_mid2 = reset_time_init + 2.0*(reset_time_end - reset_time_init)/3.0;

	switch(type){
	case (WHEEL):
		linearcontroller->pushBackWithContactInfo(0, 0, true, false);
		linearcontroller->pushBackWithContactInfo(1-trans_time, 0, true, true);
		linearcontroller->pushBackWithContactInfo(1, 2*(usingMultiFactor)/(abs(usingMultiFactor))*theta_r, true, false);
		break;
	case (LEG):
		linearcontroller->pushBackWithContactInfo(0, theta1, true, false);
		linearcontroller->pushBackWithContactInfo(i_interval*reset_time, theta1, false, true);
		linearcontroller->pushBackWithContactInfo((i_interval+1)*reset_time, theta2, true, false);
		linearcontroller->pushBackWithContactInfo(1-trans_time, theta2, true, true);
		linearcontroller->pushBackWithContactInfo(1, theta3, true, false);

		//std::cout << theta1 << " " << theta2 << " " << theta3 << std::endl;
		break;
	case (DOUBLE_SHOULDED):
		linearcontroller->pushBackWithContactInfo(0, theta_r*usingMultiFactor, true, false);
		linearcontroller->pushBackWithContactInfo(reset_time_init, theta_r*usingMultiFactor, false, true);
		linearcontroller->pushBackWithContactInfo(reset_time_mid1, (1.2/2.0)*M_PI*sgn(usingMultiFactor), false, true);
		linearcontroller->pushBackWithContactInfo(reset_time_mid2, 0, false, true);
		linearcontroller->pushBackWithContactInfo(reset_time_end, (-1)*theta_r*sgn(usingMultiFactor), true, false);
		linearcontroller->pushBackWithContactInfo(1-trans_time, (-1)*theta_r*sgn(usingMultiFactor), true, true);
		linearcontroller->pushBackWithContactInfo(1, theta_r*usingMultiFactor, true, false);
		break;
	case (DOUBLE_ELBOW):
		linearcontroller->pushBackWithContactInfo(0, 0, true, false);
		linearcontroller->pushBackWithContactInfo(reset_time_init, 0, false, true);
		linearcontroller->pushBackWithContactInfo(reset_time_mid1, -1*(2*theta_r + M_PI*0.05)*usingMultiFactor, false, true);
		linearcontroller->pushBackWithContactInfo(reset_time_mid2, -1*(2*theta_r + M_PI*0.05)*usingMultiFactor, false, true);
		linearcontroller->pushBackWithContactInfo(reset_time_end, 0, true, false);
		linearcontroller->pushBackWithContactInfo(1-trans_time, 0, true, true);
		linearcontroller->pushBackWithContactInfo(1, 0, true, false);
		break;
	default:
		linearcontroller->pushBackWithContactInfo(0, 0, true, false);
		linearcontroller->pushBackWithContactInfo(1-trans_time, 0, true, true);
		linearcontroller->pushBackWithContactInfo(1, 2*theta_r, true, false);

	break;
	}

	}
}




void GrammarController::updateParametersForInterpolation(double _theta1, double _theta2){

	linearcontroller->clear(); 


	double 	theta1_r = multiFactor*(M_PI/180.0*_theta1);
	double 	theta2_r = multiFactor*(M_PI/180.0*_theta2);
	
//	if(theta1_r <0){
//		theta1_r = 2*M_PI -theta1_r;
//	}
//	if(theta2_r <0){
//		theta2_r = 2*M_PI -theta2_r;
//	}

	//std::cout <<"-------------------------------- interpolating  t1= " << theta1_r << std::endl;
	//std::cout <<"-------------------------------- interpolating t2 = " << theta2_r << std::endl;

	

	switch(type){
	case (WHEEL):
		linearcontroller->pushBackWithContactInfo(0, 0, true, true);
		linearcontroller->pushBackWithContactInfo(1, 0, true, false);
		break;
	case (LEG):
		linearcontroller->pushBackWithContactInfo(0, theta1_r, true, true);
		//linearcontroller->pushBackWithContactInfo(0.25, theta1_r, true, true);
		//linearcontroller->pushBackWithContactInfo(0.75, theta2_r, true, true);
		//linearcontroller->pushBackWithContactInfo(0.5, theta2_r, true, false);
		linearcontroller->pushBackWithContactInfo(1, theta2_r, true, false);

		//std::cout << theta1 << " " << theta2 << " " << theta3 << std::endl;
		break;
	case (DOUBLE_SHOULDED):
		linearcontroller->pushBackWithContactInfo(0, theta1_r, true, true);
		linearcontroller->pushBackWithContactInfo(1, theta2_r, true, false);
		break;
	case (DOUBLE_ELBOW):
		linearcontroller->pushBackWithContactInfo(0, 0, true, true);
		linearcontroller->pushBackWithContactInfo(1, 0, true, false);
		break;
	default:
		linearcontroller->pushBackWithContactInfo(0, 0, true, true);
		linearcontroller->pushBackWithContactInfo(1, 0, true, false);

	break;
	}

	
}