#pragma once
#include "symbolic.h"
namespace FabByExample {
	class Geometry;
	class TemplateElement;

	class Element {
	protected:
		Element() : drawing(nullptr), tempGeometry(nullptr), refTempElement(nullptr), isStable(false) {}

	public:
		virtual ~Element() {}

		// Latently set the referring template element.
		virtual void setRefTempElement(TemplateElement* refTempElement) {
			this->refTempElement = refTempElement;
			computeNewGeo();
		}
		// Get the referring TemplateElement.
		TemplateElement* getRefTemplateElement() { return refTempElement; }

		// Recompute the geometry, as well as the sub element specifications.
		virtual void computeNewGeo() = 0;
		// Get the last computed geometry.
		Geometry* getGeo(){ return tempGeometry; }
		// Compute the bounding box of this element.
		virtual ::TriMesh::BBox evalBoundingBox() = 0;

		// Evaluate the symbolic drawing with the current environment, and return the
		// evaluated numeric drawing.
		virtual drawing::Drawing const* evalDrawing() = 0;
		
		// Getter and setter for whether this element is stable
		bool getIsStable(){ return this->isStable; }
		void setIsStable(bool _isStable){ this->isStable = _isStable; }

	protected:
		drawing::Drawing* drawing;
		Geometry * tempGeometry;
		TemplateElement * refTempElement;
		bool isStable;

		DISALLOW_COPY_AND_ASSIGN(Element);
	};

}