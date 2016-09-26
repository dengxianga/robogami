#ifndef _CONTACT_POINT_
#define _CONTACT_POINT_

#include "symbolic.h"
#include "Debugging.h"

namespace FabByExample{


class ContactPoint : public Debuggable {
public:

	Point3S cPoint;  // The linear expression for this constraint
	bool isConstrained;  // Whether it's equality constraint (linearExpr = 0) or inequality constraint (linearExpr <= 0)

	// Construct a constraint of linearExpr = 0.
	ContactPoint(const Point3S& _cPoint) : cPoint(_cPoint){ isConstrained = false;}

	ContactPoint(const Point3S& _cPoint, bool _isConstrained) : cPoint(_cPoint){ isConstrained = _isConstrained;}

	void roate(Eigen::Vector3d const& center, Eigen::Quaterniond const& rotation){
				cPoint = cPoint.translate(-center);
				cPoint = cPoint.times(rotation.toRotationMatrix());
				cPoint = cPoint.translate(center);
	}


	void translate(Eigen::Vector3d const& trans){
		cPoint = cPoint.translate(trans);
	}
	virtual DebugInfo* getDebugInfo(){
		auto info = new DebugInfo();
		info->setTypeName("ContactPoint");
		info->setShortDescription(concat() << cPoint.y.toString() << " = 0 ");
		info->putStringProperty("they y cordinate is ", concat() << cPoint.y.toString() << " = 0");
		return info;
	}

	DISALLOW_COPY_AND_ASSIGN(ContactPoint);
};
}
#endif