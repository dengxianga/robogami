#ifndef _CONSTRAINTS_
#define _CONSTRAINTS_

#include "symbolic.h"
#include "Debugging.h"

namespace FabByExample{
class Template;

class Constraint : public Debuggable {
public:
	enum ConstraintRelation { EQ, INEQ };
	enum ConstraintType{GENERAL, SYMM_GROUND, SYMM_LEGW, SYMM_LEGL, SYMM_SPACING};
private:
	LinearExpr linearExpr;  // The linear expression for this constraint
	ConstraintRelation relation;  // Whether it's equality constraint (linearExpr = 0) or inequality constraint (linearExpr <= 0)
	ConstraintType type;
public:
	// Construct a constraint of linearExpr = 0.
	Constraint(const LinearExpr& linearExpr) : linearExpr(linearExpr), relation(ConstraintRelation::EQ) , type(ConstraintType::GENERAL){}

	// Construct a constraint of linearExpr = 0 or linearExpr <= 0.
	Constraint(ConstraintRelation relation, const LinearExpr& linearExpr) : linearExpr(linearExpr), relation(relation) , type(ConstraintType::GENERAL){}

	// Get the linear expression on the left hand side of the constraint.
	LinearExpr getLinearExpr() { return linearExpr; }

	// Get the relation, = or <=, for the constraint
	ConstraintRelation getRelation() { return relation; }

	ConstraintType getType() {return type;}
	void setConstraintType(ConstraintType _type){type = _type;}
	// whether the constraint involves any symbols defined directly by the given template
	bool isRelated(Template* _temp);
	

	virtual DebugInfo* getDebugInfo();

	DISALLOW_COPY_AND_ASSIGN(Constraint);
};
}
#endif