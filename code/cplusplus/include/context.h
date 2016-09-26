#ifndef __CONTEXT__
#define __CONTEXT__

#include <vector>
#include <string>
#include <stdio.h>
#include <map>
#include "atom.h"

// adriana was here

namespace objrep {

	class Context;
	class Primitive;
	class Unit;
	class Scope;
	class SymbolTable;
	class Atoms;

	class Pose;
	class Function;
	class Geometry ;
	class SurfaceProperty;
	class VolumeProperty;
	class Property;

	class Constrain;



	enum SYMBOL_TYPE {
		SYMBOL_PRIMITIVE,
		SYMBOL_UNIT,
		SYMBOL_NONE,        // In case you remove some primitive out
	};
	/*
	Symbol -- Multiple type of material in 

	*/
	struct  Symbol {
		SYMBOL_TYPE Type;
		int Atom;
		union {
			Primitive *prim;
			Unit *unit;
		};
	};

	/* 
	Context is used to store all information neccessary for each
	representation/document. All the memory allocation for the
	representation is declared within the context, and removed when it
	is done.
	*/
	class Context {
	public:
		Context();
		~Context();


		int AddPrimitive( Primitive * ); 
		void RemovePrimitive( Primitive *);
		Primitive *FindPrimitive( std::string Name);

		void RegisterUnit(Unit *unit);
		Unit *FindUnit(std::string symbol);

		//typedef iterator 
		typedef  Primitive* Iterator;

		void SetProperty( Property *prop);

		// For the sake of getting this done
	//private:

		void AddSymbol( Primitive *);
		void AddSymbol( Unit *);

		Property * m_property;

		std::vector< SurfaceProperty *> m_surfaceProperties;
		std::vector< VolumeProperty *>  m_volumeProperties;
		std::vector< Function *> m_funcs;
		std::vector< Pose *>    m_poses;
		std::vector< Geometry *> m_geometries;
		
		// Symbol Table
		std::vector< Symbol >  m_symboltab;
		Atom                   m_atomTable;

	};


};



#endif
