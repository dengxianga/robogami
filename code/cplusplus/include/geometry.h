#ifndef __GEOMETRY__
#define __GEOMETRY__

#include "primitives.h"


namespace objrep {
	class Geometry : public Primitive{
	public:
		enum PRIMITIVE_TYPE {
			QUAD,
			TRIANGLE,
			TETRAHEDRAL,
			VOLUME,
			GROUP,
		};

//		vec position, rotation, scale;
		Geometry(Context *context, 
			     const std::string *name,
				 PRIMITIVE_TYPE _type,
				 Function * _vertices,
				 Function * _faces) : Primitive(context, name) {
				 m_type = _type;
				 m_vertices = _vertices;
				 m_faces = _faces;

				 Init();
		}

		Geometry(Context *context, 
			     const std::string *name,
				 Function * volumes) : Primitive(context, name) {
				 m_type = VOLUME;
				 m_volumes = volumes;

				 Init();
		}

		Geometry(Context *context,
			     const std::string *name,
				 std::vector< Geometry *> geometries) : Primitive(context, name){
			m_type = GROUP;
			m_geometryGroups = geometries;

			Init();
		}

		static Geometry *FromMeshFile( Context *context, const char * filename);

		SurfaceProperty *getSurfaceProperty(int index);
		VolumeProperty  *getVolumeProperty(int index);

		SurfaceProperty *getSurfaceProperty(const char *SurfaceType);
		VolumeProperty  *getVolumeProperty(const char *VolumeType);



		PRIMITIVE_TYPE  getType();
		Function       *getVerticesFunc();
		Function       *getFacesFunc();
		Function       *getVolumesFunc();

		int             getnVertices();
		int             getnFaces();
		
		// This work only on Quad and Triangle, Mainly for displaying
		void            getTriangles(std::vector<vec > &vertices, 
			                         std::vector< Vec<3, int> > &faces);

		void            getTriangles(std::vector<vec > &vertices, 
			                         std::vector< Vec<3, int> > &faces, Pose *, std::string trail);
		bool            isTrianglable(bool partial = false) ;
		bool            isVolumeObject(bool partial = false);

		// Respect to its parent, operate in the order of scale, rotate then translate
		virtual std::string desc() {return "Geometry"; }

	private:
		void Init() {
			m_isFixed = 0;
			m_scale = vec(1,1,1);
			m_rotation = vec(0,0,0);
			m_position = vec(0,0,0);

		}
	protected:


		int            m_isFixed;

		vec            m_scale;
		vec            m_rotation;
		vec            m_position;

		PRIMITIVE_TYPE m_type;
		Function     * m_vertices;
		Function     * m_faces;
		Function     * m_volumes;
		
		std::map< std::string, SurfaceProperty *> m_SurfaceProperties;
		std::map< std::string, VolumeProperty  *> m_VolumeProperties;
		std::vector< Geometry *>         m_geometryGroups;
		
		friend class YAMLGeometryIO;
	};
}

#endif 