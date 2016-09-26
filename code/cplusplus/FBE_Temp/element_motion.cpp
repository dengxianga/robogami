#include "element_motion.h"
#include "controller.h"
#include "geometry.h"
#include <Windows.h>
#include <fstream>
#include <iostream>

using namespace FabByExample; 

PWLinearController * Element_Motion::eval(){
	PWLinearController* controller = new PWLinearController();
	for each (auto p in controlPointsS){
		controller->pushBackWithContactInfo(p.t.eval(SymbolicAssignment::USE_CURRENT_VALUES),
			p.val.eval(SymbolicAssignment::USE_CURRENT_VALUES), p.contact, p.ismoving);

	}
	return controller;
}


void Element_Motion::addNewControPointS(LinearExpr & t, LinearExpr & val, bool _contact, bool _ismoving){
	ControlPointS newControlPointS(t,val, _contact, _ismoving);
	controlPointsS.push_back(newControlPointS);
}



Element_Motion::Element_Motion(){
	tempGeometry = new Geometry();
	drawing = new drawing::Drawing();

}