#ifndef _NEWPATCH_
#define _NEWPATCH_


#include <Eigen/Dense>
#include "NewConnection.h"
#include <symbolic.h>

namespace FabByExample{

	class ParamVectorMap; 
	class TemplateElement; 
	class Template;
	class Element_Symbolic;
	class Geometry;

	class ServoSpacing{
	public:
		LinearExpr alpha;
		std::vector<LinearExpr> separation_w;
		LinearExpr separation_h;
		std::vector<Template*> associatedTemplates;
		ServoSpacing(){}
		ServoSpacing(LinearExpr _alpha, LinearExpr _separation_w, LinearExpr _separation_h, Template* initTemp){
			alpha =_alpha;
			separation_w.push_back(_separation_w);
			separation_h = _separation_h;
			associatedTemplates.push_back(initTemp); 
		}
		void addInfo(LinearExpr _separation_w, LinearExpr _separation_h, Template* newTemp){
			separation_w.push_back(_separation_w);
			separation_h = separation_h + _separation_h;
			associatedTemplates.push_back(newTemp);
		}
		void display();
	};


	class NewPatch : public FabDebugging::Debuggable {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		NewPatch(Template * _element){ element = _element; parentConnection = nullptr; }
		NewPatch::~NewPatch();
		enum PatchType {LINE2D3D, PLANE, PatchPlaceHolder, SERVO_LINE_PATCH, SERVO_POINT_PATCH, PERIPHERAL_PATCH};
		virtual double compare(NewPatch* _reference) = 0;
		virtual NewPatch::PatchType getType() = 0;
		Template * getElement(){return element;}
		virtual double getDistanceToPoint(const Eigen::Vector3d & refPoint) = 0;
		virtual bool isTheSame(NewPatch * patch) = 0;
		NewConnection * parentConnection; //gets set when this gets added to a connection.  Should only ever be set to one connection
		virtual Eigen::Vector3d getReferencePoint() = 0;
		virtual Eigen::Vector2d getReferencePoint2D() = 0;
		virtual FabDebugging::DebugInfo* getDebugInfo() = 0;
		virtual void rotate(Eigen::Vector3d const& center_rot, Eigen::Quaterniond const& rotation) = 0;
		virtual void translate(Eigen::Vector3d const& trans) = 0;
		virtual Geometry* getDebugGeo() = 0;
		bool isRelated(Template * temp);
		virtual Eigen::Vector3d getNormal() = 0;

	protected:
		Template * element;

		DISALLOW_COPY_AND_ASSIGN(NewPatch);
	};

	class NewPatchLine2D3D : public NewPatch {

	public:
		NewPatchLine2D3D(Template * _element, int edgeId, int vertexId1, int vertexId2);
		void rotate(Eigen::Vector3d const& center_rot, Eigen::Quaterniond const& rotation){;}
		void translate(Eigen::Vector3d const& trans){;}
		Eigen::Vector3d getNormal(){throw "NOt implemented";}


		Eigen::Vector3d getDirection();

		Eigen::Vector2d getDirection2D();

		virtual Eigen::Vector3d getReferencePoint();
		virtual Eigen::Vector2d getReferencePoint2D();
		bool isTheSame(NewPatch * patch);

		NewPatch::PatchType getType(){return PatchType::LINE2D3D; }

		double getDistanceToPoint(const Eigen::Vector3d & refPoint);
		double getDistanceToPatch(NewPatchLine2D3D* refPatch);
		virtual Eigen::Vector3d getVertex1();
		virtual Eigen::Vector3d getVertex2();
		virtual Eigen::Vector2d getDrawingVertex1();
		virtual Eigen::Vector2d getDrawingVertex2();

		double compare(NewPatch* _reference);

		int edgeId;
		int vertexId1, vertexId2;

		Element_Symbolic* getElement();
		Point3S getVertexS1();
		Point3S getVertexS2();
		Point2S getDrawingVertexS1();
		Point2S getDrawingVertexS2();

		// Returns whether the drawing edge goes counterclockwise around the face.
		bool isDrawingEdgeCounterClockwise();

		FabDebugging::DebugInfo* getDebugInfo();
		Geometry* getDebugGeo(){ throw "not implemented" ;}

		NewPatchLine2D3D* flip() {
			return new NewPatchLine2D3D(this->element, this->edgeId, this->vertexId2, this->vertexId1);
		}
		
		enum DirectionResult {
			SAME,
			OPPOSITE,
			DIFFERENT,
		};

		DirectionResult matchDirection(NewPatchLine2D3D* otherPatch);

		DISALLOW_COPY_AND_ASSIGN(NewPatchLine2D3D);
	};



	class ServoLinePatch : public NewPatch {

	public:
		ServoLinePatch(Template * _element, Point3S _vertex1, Point3S _vertex2,Eigen::Vector3d normal );
		ServoLinePatch::~ServoLinePatch(){}
		void rotate(Eigen::Vector3d const& center_rot, Eigen::Quaterniond const& rotation);
		void translate(Eigen::Vector3d const& trans);
		Eigen::Vector3d getDirection() { throw "NotImplemented"; }
		Eigen::Vector2d getDirection2D() { throw "NotImplemented"; }
		bool isTheSame(NewPatch * patch) { throw "NotImplemented"; }
		double compare(NewPatch* _reference) { throw "NotImplemented"; }
		NewPatch::PatchType getType(){return PatchType::SERVO_LINE_PATCH; }
		double getDistanceToPoint(const Eigen::Vector3d & refPoint);
		Eigen::Vector3d getReferencePoint() { throw "NotImplemented"; }
		Eigen::Vector2d getReferencePoint2D() { throw "NotImplemented"; }
		FabDebugging::DebugInfo* getDebugInfo(){ throw "NotImplemented"; }

		Point3S getVertex1(){ return vertex1;}
		Point3S getVertex2() {return vertex2;}
		Eigen::Vector3d getNormal(){return normal;}
		Geometry* getDebugGeo();
		Eigen::Vector3d getEvalVertex1(){return vertex1.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);}
		Eigen::Vector3d getEvalVertex2(){return vertex2.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);}
		std::vector<ServoSpacing> spacings;
		DISALLOW_COPY_AND_ASSIGN(ServoLinePatch);
	private:
		Point3S vertex1;
		Point3S vertex2;
		Eigen::Vector3d normal;


	};


	
	class ServoPointPatch : public NewPatch {

	public:
		ServoPointPatch(Template * _element, Point3S _center, LinearExpr _separtion, Eigen::Vector3d _normal );

		ServoPointPatch::~ServoPointPatch(){}
		void rotate(Eigen::Vector3d const& center_rot, Eigen::Quaterniond const& rotation);
		void translate(Eigen::Vector3d const& trans);
		Eigen::Vector3d getDirection() { throw "NotImplemented"; }
		Eigen::Vector2d getDirection2D() { throw "NotImplemented"; }
		bool isTheSame(NewPatch * patch) { throw "NotImplemented"; }
		double compare(NewPatch* _reference)  { throw "NotImplemented"; }
		NewPatch::PatchType getType(){return PatchType::SERVO_POINT_PATCH; }
		double getDistanceToPoint(const Eigen::Vector3d & refPoint);
		Eigen::Vector3d getReferencePoint() { throw "NotImplemented"; }
		Eigen::Vector2d getReferencePoint2D() { throw "NotImplemented"; }
		FabDebugging::DebugInfo* getDebugInfo(){ throw "NotImplemented"; }
		Geometry* getDebugGeo();
		

		DISALLOW_COPY_AND_ASSIGN(ServoPointPatch);

		Eigen::Vector3d getNormal(){return normal;}
		Point3S getSymbCenter() {return center;}
		LinearExpr getSeparation(){ return separation; }
		Eigen::Vector3d getCenter();
		Point3S center;
		
	private:
		Eigen::Vector3d normal;
		LinearExpr separation; 


	};


	class PeripheralPatch : public NewPatch {

	public:
		PeripheralPatch(Template * _element, NewPatchLine2D3D *p1, NewPatchLine2D3D *p2, Eigen::Vector3d _normal );

		PeripheralPatch::~PeripheralPatch(){}
		void rotate(Eigen::Vector3d const& center_rot, Eigen::Quaterniond const& rotation);
		void translate(Eigen::Vector3d const& trans){}
		Eigen::Vector3d getDirection() { throw "NotImplemented"; }
		Eigen::Vector2d getDirection2D() { throw "NotImplemented"; }
		bool isTheSame(NewPatch * patch) { throw "NotImplemented"; }
		double compare(NewPatch* _reference)  { throw "NotImplemented"; }
		NewPatch::PatchType getType(){return PatchType::PERIPHERAL_PATCH; }
		double getDistanceToPoint(const Eigen::Vector3d & refPoint);
		double getDistanceToPatch(PeripheralPatch* refPatch);
		double getSideOrientationDistanceToPatch( PeripheralPatch*  refPatch);
		
		void writeToFile(std::string filename); 

		Eigen::Vector3d getReferencePoint() { throw "NotImplemented"; }
		Eigen::Vector2d getReferencePoint2D() { throw "NotImplemented"; }
		FabDebugging::DebugInfo* getDebugInfo(){ throw "NotImplemented"; }
		Geometry* getDebugGeo();

		DISALLOW_COPY_AND_ASSIGN(PeripheralPatch);

		Eigen::Vector3d getNormal(){return normal;}
		NewPatchLine2D3D *getLinePatch1(){return p1;}
		NewPatchLine2D3D *getLinePatch2(){return p2;}

	private:
		Eigen::Vector3d normal;
		NewPatchLine2D3D *p1;
		NewPatchLine2D3D *p2;

	};

	

	class PlaceHolderPatch : public NewPatch {
	public:
		bool isOpposite;
		PlaceHolderPatch::PlaceHolderPatch(bool isOpposite) : NewPatch(nullptr), isOpposite(isOpposite) {}
		PlaceHolderPatch::~PlaceHolderPatch() {}
		void rotate(Eigen::Vector3d const& center_rot, Eigen::Quaterniond const& rotation){;}
		void translate(Eigen::Vector3d const& trans){;}
		double compare(NewPatch* _reference){ throw "NotImplemented"; }
		NewPatch::PatchType getType(){return PatchType::PatchPlaceHolder;}
		double getDistanceToPoint(const Eigen::Vector3d & refPoint){ throw "NotImplemented"; }
		bool isTheSame(NewPatch * patch) { throw "NotImplemented"; }
		Eigen::Vector3d getReferencePoint() { throw "NotImplemented"; }
		Eigen::Vector2d getReferencePoint2D() { throw "NotImplemented"; }
		FabDebugging::DebugInfo* getDebugInfo() { throw "NotImplemented"; }
		Geometry* getDebugGeo(){ throw "NotImplemented"; }
		Eigen::Vector3d getNormal(){throw "NOt implemented";}

		DISALLOW_COPY_AND_ASSIGN(PlaceHolderPatch);
	};


	struct PatchPair {
		NewPatch* wt_patch;
		NewPatch* add_patch;
		bool isOpposite;
		bool isOpposite2D;
		PatchPair(NewPatch* _wt_patch, NewPatch * _add_patch, bool isOpposite, bool isOpposite2D) {
			wt_patch = _wt_patch;
			add_patch = _add_patch;
			this->isOpposite = isOpposite;
			this->isOpposite2D = isOpposite2D;
		}
		bool operator==(const PatchPair& rhs) const
		{
			return wt_patch == rhs.wt_patch && add_patch == rhs.add_patch && isOpposite == rhs.isOpposite && isOpposite2D == rhs.isOpposite2D;
		}
	};

}

#endif