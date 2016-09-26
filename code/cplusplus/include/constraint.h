#ifndef __CONSTRAINT__
#define __CONSTRAINT__


#include "primitives.h"
#include "context.h"


namespace objrep {

	enum ConstraintType {
		CONSTRAINT_POINT2POINT,
		CONSTRAINT_HINGE,
		CONSTRAINT_D6,
		CONSTRAINT_SLIDER
	};


	class Constraint {
	protected:
		vec         m_lowerLimit, m_upperLimit;

		Geometry        *m_geoA, *m_geoB;
		ConstraintType   m_ConstraintType;
	public:
		Constraint( ConstraintType type, Geometry *geoA ) 
			: m_ConstraintType(type), m_geoA(geoA), m_geoB(NULL) 
		{
			m_lowerLimit = vec(0,0,0);
			m_upperLimit = vec(0,0,0);
		}
		Constraint( ConstraintType type, Geometry *geoA, Geometry *geoB ) 
			: m_ConstraintType(type), m_geoA(geoA), m_geoB(geoB) 
		{
			m_lowerLimit = vec(0,0,0);
			m_upperLimit = vec(0,0,0);
		}

		Geometry *getGeometryA() {
			return m_geoA;
		}

		Geometry *getGeometryB() {
			return m_geoB;
		}

		ConstraintType getConstraintType() const {
			return m_ConstraintType;
		}
		// User Add justable value
		vec &getLowerLimit() {
			return m_lowerLimit;
		}
		vec &getUpperLimit() {
			return m_upperLimit;
		}
	};


	class Point2PointConstraint: public Constraint {
	protected:
		vec m_pivotInA,  m_pivotInB;
		
	public:
		Point2PointConstraint( Geometry *geoA, const vec &pivotInA)
			: Constraint( CONSTRAINT_POINT2POINT, geoA ), 
			  m_pivotInA(pivotInA) {
				  m_ConstraintType = CONSTRAINT_POINT2POINT;
		}
		Point2PointConstraint( Geometry *geoA, const vec &pivotInA,
			                   Geometry *geoB, const vec &pivotInB)
			: Constraint( CONSTRAINT_POINT2POINT, geoA, geoB ), 
			  m_pivotInA(pivotInA),
			  m_pivotInB(pivotInB){
				  m_ConstraintType = CONSTRAINT_POINT2POINT;
		}

		vec &getPivotInA() { return m_pivotInA; }
		vec &getPivotInB() { return m_pivotInB; }
	};

	class HingeConstraint: public Constraint {
	protected:
		vec m_pivotInA,  m_pivotInB;
		vec m_axisInA ,  m_axisInB;

	public:
		HingeConstraint( Geometry *geoA, const vec &pivotInA, const vec &axisInA)
			: Constraint( CONSTRAINT_POINT2POINT, geoA ), 
			  m_pivotInA(pivotInA),
		      m_axisInA(axisInA)
		{
			m_ConstraintType = CONSTRAINT_HINGE;
		}

		HingeConstraint( Geometry *geoA, const vec &pivotInA, const vec &axisInA,
	                     Geometry *geoB, const vec &pivotInB, const vec &axisInB)
			: Constraint( CONSTRAINT_POINT2POINT, geoA, geoB ), 
			  m_pivotInA(pivotInA),
			  m_pivotInB(pivotInB),
			  m_axisInA(axisInA),
			  m_axisInB(axisInB)
		{
			m_ConstraintType = CONSTRAINT_HINGE;
		}

		vec &getPivotInA() { return m_pivotInA; }
		vec &getPivotInB() { return m_pivotInB; }

		vec &getAxisInA() { return m_axisInA; }
		vec &getAxisInB() { return m_axisInB; }
	};

	class SliderConstraint: public Constraint {
	protected:
		vec m_axisInA,  m_axisInB;
		
	public:
		SliderConstraint( Geometry *geoA, const vec &axisInA)
			: Constraint( CONSTRAINT_POINT2POINT, geoA ), 
			  m_axisInA(axisInA) {
			m_ConstraintType = CONSTRAINT_SLIDER;
		}
		SliderConstraint( Geometry *geoA, const vec &axisInA,
			                   Geometry *geoB, const vec &axisInB)
			: Constraint( CONSTRAINT_POINT2POINT, geoA, geoB ), 
			  m_axisInA(axisInA),
			  m_axisInB(axisInB){
			m_ConstraintType = CONSTRAINT_SLIDER;
		}

		vec &getAxisInA() { return m_axisInA; }
		vec &getAxisInB() { return m_axisInB; }
	};

}

#endif