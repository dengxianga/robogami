#pragma once
#include "symbolic.h"
#include "element.h"
namespace FabByExample {
	class Geometry;
	class TemplateElement;
	class PWLinearController;

	// Corresponds to a proto::OpenscadParameter
	struct ControlPointS {
		LinearExpr t;
		LinearExpr val;
		bool contact;
		bool ismoving;
		ControlPointS(LinearExpr & _t, LinearExpr & _val, bool _contact, bool _ismoving){
			t = _t;
			val= _val;
			contact = _contact;
			ismoving = _ismoving;
		}
	};

	class Element_Motion : public Element {
	public:
		Element_Motion();

		virtual ~Element_Motion() {
		}

		// The virtual methods that are not implemented
		void computeNewGeo(){ }
		::TriMesh::BBox evalBoundingBox() { throw "Not Implemented"; }
		drawing::Drawing const* evalDrawing(){return drawing;}

		PWLinearController * eval();
		void addNewControPointS(LinearExpr & t, LinearExpr & val, bool _contact, bool _ismoving);


		// made this public so this are easier!
		vector<ControlPointS> controlPointsS;
	private:
		DISALLOW_COPY_AND_ASSIGN(Element_Motion);
	};

}