#ifndef __POSE__
#define __POSE__
#include "primitives.h"
#include "context.h"
#include "constraint.h"


namespace objrep {



	class Alteration {
	public:
		enum ALTER_TYPE {
			ALTER_PRIMITIVE,
			ALTER_SEQUENCE // Assume floating sequence
		} ;
		struct AlterStruct {
			ALTER_TYPE type;
			union {
				Primitive *primitive;
				std::vector< float > *sequence;
			};
			AlterStruct(Primitive *primitive) {
				type = ALTER_PRIMITIVE;
				this->primitive = primitive;
			}
			AlterStruct(std::vector< float > *seq) {
				type = ALTER_SEQUENCE;
				this->sequence = seq;
			}
		};
		void AddConstraint( const std::string &key, Primitive *prim) {
			alterationSet.push_back( std::pair< std::string, struct AlterStruct>( key, struct AlterStruct(prim)) );
		}
		void AddConstraint( const std::string &key, std::vector<float>* f) {
			alterationSet.push_back( std::pair< std::string, struct AlterStruct>( key, struct AlterStruct(f)) );
		}
		std::vector< std::pair< std::string, AlterStruct> > alterationSet;
		// Clear memory if remove this out

		~Alteration() {
			for (unsigned int i=0;i<alterationSet.size();i++) {
				if (alterationSet[i].second.type == ALTER_SEQUENCE)
					delete alterationSet[i].second.sequence;
			}
		}
	};

	class Pose: public Primitive {
	public:
		Pose(Context *context, 
			const std::string *name) : Primitive(context, name) 
		{

		}
		// This work only on Quad and Triangle, Mainly for displaying
		void  getTriangles(std::vector<vec > &vertices, 
			std::vector< Vec<3, int> > &faces);


		void  getTriangles(std::vector<vec > &vertices, 
			std::vector< Vec<3, int> > &faces, 
			Pose *pose, 
			std::string trail);

		Primitive *matchPrimitive( const std::string trail, Primitive *originalPrimitive );

		void matchSequence( const std::string trail, std::vector<Value > &originalPrimitive );
		void matchSequence( const std::string trail, vec &originalPrimitive,  int ndim);

		void AddLocalConstraint( Constraint* constraint) {
			m_constraints.push_back( constraint);
		}

		int		getNumConstraints() const {	return m_constraints.size();	}
		Constraint* getConstraint(int index)		{	return m_constraints[index];		}


		std::vector< Constraint* >   m_constraints; 
		Alteration                   m_alterationSet;
		std::vector< Geometry *>     m_geometries;

		Pose                        *m_reference;
		virtual std::string desc() { return "Pose"; }

		friend class YAMLPoseIO;
	};
};
#endif