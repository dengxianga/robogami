#pragma once
#include "symbolic.h"
#include "element.h"
namespace FabByExample {
	class Geometry;
	class TemplateElement;

	// Corresponds to a proto::OpenscadParameter
	struct OpenscadParameter {
		string name;
		double initValue;
		// represents an optional double: second element is true iff min is specified.
		pair<double, bool> min;
		pair<double, bool> max;
	};

	class Element_Openscad : public Element {
	public:
		Element_Openscad(const vector<OpenscadParameter>& params, const string& code) : params(params), code(code) {
			// TODO: Figure out where to get the 2D drawing from an OpenSCAD design.
			drawing = new drawing::Drawing();
		}

		virtual ~Element_Openscad() {}

		// Recompute the geometry by invoking OpenSCAD.
		virtual void computeNewGeo();
		// Compute the bounding box of this element.
		virtual ::TriMesh::BBox evalBoundingBox();

		// TODO: implement.
		virtual drawing::Drawing const* evalDrawing() {
			return drawing;
		}

		void setCode(const string& code) {
			this->code = code;
			computeNewGeo();
		}

		const string& getCode() { return code; }
		const vector<OpenscadParameter>& getParams() { return params; }

		// Modify this Element_Openscad to be the same as the given Element_Openscad. This is useful
		// when C# side sends down a new Element_OpenSCAD and we want to update this element.
		void modify(Element_Openscad* as);

	private:
		vector<OpenscadParameter> params;
		string code;
		DISALLOW_COPY_AND_ASSIGN(Element_Openscad);
	};

}