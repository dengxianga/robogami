#ifndef __UNIT__
#define __UNIT__
#include <string>
#include <vector>
#include <utility>
#include <map>
#include "Vec.h"
#include "XForm.h"

namespace objrep {
	class Unit;
	class Context;

	enum VALUE_TYPE {
		VAL_PTR,
		VAL_INT,
		VAL_FLOAT,
		VAL_DOUBLE,
		VAL_CHAR,
		VAL_FPTR,
		VAL_ERR,
	};

	typedef void * Pointer;

	struct Value {
		VALUE_TYPE type;
		union {
			Pointer * ptrVal;
			int iVal;
			float fVal;
			double dVal;
			char tVal[8];
		} ;
		Value() : dVal(0) { };
		Value(float x) : fVal(x) { type = VAL_FLOAT;  };
		Value(int   x) : iVal(x) { type = VAL_INT;  };
		Value(double x) : dVal(x) { type = VAL_DOUBLE;  };
	};


	class Vector {
		Value * val;
		int size;
	};


	enum SET_TYPE {
		SET_RANGE,
		SET_UNION,
		SET_ITER,
	};


	struct Set {
		SET_TYPE type;
		union {
			std::vector< struct Set > *m_sets;
			std::vector< Value > *m_values;
		} ;
		Set( std::vector< Value > &values ) {   
			type = SET_ITER; 
			m_values = new std::vector< Value >; 
			*m_values = values;
		};
		Set( std::vector< struct Set > &sets) {
			type = SET_UNION; 
			m_sets = new std::vector< struct Set >; 
			*m_sets = sets;
		}
		Set( Value min, Value max) {
			type = SET_RANGE;
			m_values = new std::vector< Value >(2); 
			(*m_values)[0] = min;
			(*m_values)[1] = max;
		}

		~Set() {
			if (type == SET_UNION) {
				delete m_sets;
			} else {
				delete m_values;
			}
		}


	};


	enum UNIT_TYPE {
		PHYSICAL,
		AUXILARY,
		VECTOR,
		COMPOSIT_MUL,
		COMPOSIT_ADD,
		FUNCTION,
	};

	enum BASE_SI_UNIT{
		SI_m,
		SI_kg,
		SI_s,
		SI_A,
		SI_K,
		SI_cd,
		SI_mol,
	};

	struct PowerBase {
		int power : 14;
		int base  : 2;  // 0 -> normal, 1-> log, 2->exp
		PowerBase(int _power, int _base) : power(_power), base(_base) {}; 
	} ;

	// Unit tree
	class Unit {
	public:
		// Unit() : m_dim(0) { };
		typedef std::pair<BASE_SI_UNIT, PowerBase> PowerBaseUnit;

		// Physical unit
		Unit(std::string symbol, const std::vector< Unit::PowerBaseUnit > &physical);
		Unit(std::string symbol,  Unit::PowerBaseUnit physical );
		// Auxilary
		Unit(std::string symbol, int nDim, VALUE_TYPE m_auxtype);
		// Vector
		Unit(std::string symbol, Vector* basis, Unit *unit);
		// Composit
		Unit(UNIT_TYPE type, const std::vector<Unit *>&unitslist);
		Unit(UNIT_TYPE type, Unit ** unitslist);
		// Function
		Unit(Unit *fin, Unit *fout);

		int dimensions();
		const std::string &description();
		const std::string &symbol();


	protected:
		UNIT_TYPE m_type;
		std::vector<Unit *> m_unitslist;
		// Physical Description
		std::vector< Unit::PowerBaseUnit > si_pairs;
		Vector * m_basis;

		VALUE_TYPE m_auxtype;

		int m_dim;
		std::string m_description;
		std::string m_symbol;
	};


	extern std::map< std::string, Unit *> globalUnitMap; 

}

#endif
