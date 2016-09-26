#ifndef __PRIMITIVE__
#define __PRIMITIVE__
#include <vector>
#include "Vec.h"
#include <string.h>
#include <map>
#include "context.h"
#include "unit.h"

// This will replace libobject.h

namespace objrep {
	//  Primitive type 
	class Pose;
	class Alteration;
	class Constraint;

	class Primitive {
	public:
		 Primitive(Context *, const std::string *name = NULL) ;
		void rename(std::string name);
		
		const std::string getName() { return m_name; }
		 Context *getContext()  { return m_context; } 
		//Primitive *Clone() = 0;
		 virtual std::string desc() { return "PRIMITIVE!!!"; }


	protected:
		Context *m_context;
		std::string m_name;
		int m_havename;
	};

	class Function : public Primitive{
	public:
		Function(Context *context, const std::string *name) : Primitive(context, name) {} ; 
		~Function() {};

		virtual void callFunc(const std::vector<Value > &functionParams,
			std::vector<Value > &outparams) = 0;
		virtual Unit *argUnit() = 0;
		virtual Unit *outUnit() = 0;

		virtual const std::string language() = 0;

		virtual std::string desc() { return "VFunction"; };

		virtual int indexSize(int dim) = 0;
	};

#if 1 // supose to be WIN32 but would not compile for Adriana so she changed it.
#define EXPORTTYPE extern "C" __declspec(dllexport)
#else
#define EXPORTTYPE extern "C"
#endif 
	/*
	EXPORTTYPE void callFunction(Function *callFunc, const std::vector<Value > &functionParams,
			std::vector<Value > &outparams) {
		callFunc->callFunc(functionParams, outparams);
	}
	*/

	enum SURFACEPROP_PARAMETERIZE {
		VERTICE_INDEX ,   // For each vertice
		FACE_INDEX,       // For each face
		CANONICAL,        // X, Y, Z coordinate
	};

	class SurfaceProperty : public Primitive{
	private:
		SurfaceProperty() : Primitive(NULL, NULL)  { }
	public:
		SurfaceProperty(Context *context, Unit *unitType, std::string *name, Function *function,
						SURFACEPROP_PARAMETERIZE parameterization);

		Value *getVerticeProperty(int index); 
		Value *getFaceProperty(int index); 
		Value *getProperty(float x,float y, float z); 

		Function *getFunction();
		const Unit *getUnit(); 
	private:
		Unit *m_unit;

		Function *m_function;
		SURFACEPROP_PARAMETERIZE m_parameterization;
	
		std::string desc()     { return "SurfaceProperties"; };
	};

	
	class VolumeProperty : public Primitive{
	public:
		virtual Value *getProperties() = 0; 
		virtual Unit  *outUnit()       = 0;

		virtual std::string desc() = 0;
	};

}

#endif