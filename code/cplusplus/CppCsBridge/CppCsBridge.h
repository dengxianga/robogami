// CppCsBridge.h

#pragma once
#include "abstractUI.h"
#include "drawing.h"
#include "workingTemplate.h"
#include <msclr\marshal_cppstd.h>
#include "mesh.h"
#include "element.h"
#include "element_symbolic.h"
#include "WorkingTemplate.h"
#include "template.h"
#include "templateElement.h"
#include "TriMesh.h"
#include "../FBE_Temp/geometry.h"
#include "metrics.h"
#include "NewPatch.h"
#include "util.h"
#include "articulation.h"
#include "TemplateManipulations.h"
#include "PartialConverters.h"
#include "TemplateProtoConverter.h"
#include "KinChain.h"
#include "FunctionEvalStability.h"

#using <mscorlib.dll>
#using <System.dll>
#using <PresentationCore.dll>
#using <System.Core.dll>

using namespace System::Collections::Generic;
using namespace System::Windows;
using namespace System::Windows::Input;
using namespace System::Windows::Media::Media3D;
using namespace System::Windows::Media;
using namespace msclr::interop;
using namespace System;

namespace CppCsBridge {
    
    public value struct Edge {
    public:
        int id;
        String^ name;
        cli::array<Point>^ points;
        Edge(const FabByExample::drawing::Edge& native, const FabByExample::drawing::Drawing& native_parent) {
            this->id = native.id;
            this->name = marshal_as<String^>(native.name);		
            this->points = gcnew cli::array<Point>(2);
            this->points[0] = Point(native_parent.points[0][0], native_parent.points[0][1]);
            this->points[1] = Point(native_parent.points[1][0], native_parent.points[1][1]);
        }
    };

	

    public value struct Face {
    public:
        int id;
        String^ name;
        cli::array<Point>^ points;
        Face(const FabByExample::drawing::Face& native) {
            this->id = native.id;
            this->name = marshal_as<String^>(native.name);
            this->points = gcnew cli::array<Point>(native.idpoints.size());
			//FabByExample::drawing::Point::point_vector_type::iterator it;
			int i = 0;
			for(auto& it = native.idpoints.begin(); it!= native.idpoints.end(); ++it){
				double p1 = it->p[0];
				double p2 = it->p[1];
				this->points[i] = Point(p1, p2);
				i++;
			}
            //for(int i=0;i<native.points.size();i++) {
            //    this->points[i] = Point(native.points[i][0], native.points[i][1]);
            //}
        }
    };

	public value struct Face3 {
	public:
		cli::array<int>^ points;
		Face3(const FabByExample::mesh::Face& native) {
			this->points = gcnew cli::array<int>(native.points.size());
			for (int i = 0; i < this->points->Length; i++){
				this->points[i] = native.points[i];
			}
		}
		Face3(const TriMesh::Face native) {
			this->points = gcnew cli::array<int>(3);
			for (int i = 0; i < 3; i++){
				this->points[i] = native.v[i];
			}
		}
	};

	public value struct Edge3 {
	public:
		int edgeid;
		cli::array<Point3D>^ vertices;
		Edge3(const FabByExample::mesh::Edge& native){
			this->edgeid = native.edgeid;
			this->vertices = gcnew cli::array<Point3D>(native.vertices.size());
			for (int i = 0; i < this->vertices->Length; i++){
				Vector3D p = Vector3D(native.vertices[i][0], native.vertices[i][1], native.vertices[i][2]);
				this->vertices[i]= Point3D(p.X, p.Y, p.Z);				 				
			}
		}
	};



	public value struct SubElementFace {
	public:
		Point3D center;
		Vector3D up;
		Vector3D normal;
		double width;
		double height;
		SubElementFace(const FabByExample::mesh::SubElementFace& native) {
			this->center = Point3D(native.center[0], native.center[1], native.center[2]);
			this->up = Vector3D(native.up[0], native.up[1], native.up[2]);
			this->normal = Vector3D(native.normal[0], native.normal[1], native.normal[2]);

			this->width = native.width;
			this->height = native.height;

		}
		SubElementFace(const FabByExample::Element_Symbolic::SubElementFace& native) {
			this->center = Point3D(native.center[0], native.center[1], native.center[2]);
			this->up = Vector3D(native.up[0], native.up[1], native.up[2]);
			this->normal = Vector3D(native.normal[0], native.normal[1], native.normal[2]);

			this->width = native.width;
			this->height = native.height;
		}

	};

	value struct TemplateRef;

	public value struct PatchRef{
	private:
		double _length;
	public:
		NewPatch* patch;
		PatchRef(NewPatch* patch)
		{
			this->patch = patch;
		}
		property double length {
			double get() {
				return _length;
			}
			void set(double l) {
				_length = l;
			}
		}
		property Point3D Vertex1 {
			Point3D get(){
				if( dynamic_cast<NewPatchLine2D3D*>(patch) !=nullptr){
					NewPatchLine2D3D* p = dynamic_cast<NewPatchLine2D3D*>(patch);
					Eigen::Vector3d v1 = p->getVertex1();
					return Point3D(v1[0], v1[1], v1[2]);
				}
				if( dynamic_cast<ServoPointPatch*>(patch) !=nullptr){
					ServoPointPatch* p = dynamic_cast<ServoPointPatch*>(patch);
					Eigen::Vector3d v2 = p->getCenter();
					return Point3D(v2[0], v2[1], v2[2]);
				}
				if( dynamic_cast<ServoLinePatch*>(patch) !=nullptr){
					ServoLinePatch* p = dynamic_cast<ServoLinePatch*>(patch);
					Eigen::Vector3d v2 = p->getEvalVertex1();
					return Point3D(v2[0], v2[1], v2[2]);
				}
				return Point3D(0,0,0);
			}
		}
		property Point3D Vertex2 {
			Point3D get(){
				if( dynamic_cast<NewPatchLine2D3D*>(patch) !=nullptr){
					NewPatchLine2D3D* p = dynamic_cast<NewPatchLine2D3D*>(patch);
					Eigen::Vector3d v2 = p->getVertex2();
					return Point3D(v2[0], v2[1], v2[2]);
				}
				if( dynamic_cast<ServoPointPatch*>(patch) !=nullptr){
					ServoPointPatch* p = dynamic_cast<ServoPointPatch*>(patch);
					Eigen::Vector3d v2 = p->getCenter() - 1*p->getNormal(); 
					return Point3D(v2[0], v2[1], v2[2]);
				}
				if( dynamic_cast<ServoLinePatch*>(patch) !=nullptr){
					ServoLinePatch* p = dynamic_cast<ServoLinePatch*>(patch);
					Eigen::Vector3d v2 = p->getEvalVertex2();
					return Point3D(v2[0], v2[1], v2[2]);
				}
				return Point3D(0,0,0);
			}
		}

		property bool IsNull {
			bool get()
			{
				return patch == nullptr;
			}
		}
		property TemplateRef OwningTemplate {
			TemplateRef get();
		}
		virtual bool Equals(Object^ o) override {
			if (dynamic_cast<PatchRef^>(o) == nullptr) return false;
			PatchRef^ r = dynamic_cast<PatchRef^>(o);
			return r->patch == patch;
		}
		virtual int GetHashCode() override {
			return (int)patch;
		}
		static property PatchRef Null {
			PatchRef get(){
				return PatchRef(nullptr);
			}
		}
		property IntPtr RawPtr {
			IntPtr get() {
				return (IntPtr)(patch);
			}
		}
	};

	public enum class ConnectionType {
		FOLD,
		TEETH,
		TOUCH,
		HINGE,
		BALLJOINT, 
		PRISMATIC, 
		NONE
	};


	ref class UIInstance;

	public value struct ConnectionRef{
	public:
		NewConnection* conn;
		ConnectionRef(NewConnection* conn)
		{
			this->conn = conn;
		}

		property double Angle{
			void set(double value){
				conn->setAngle(value);
			}
			double get(){
				return conn->getAngle();
			}
		}

		property ConnectionType Type {
			void set(ConnectionType type) {
				conn->setType(static_cast<NewConnection::ConnectionType>(type));
			}
			ConnectionType get() {
				return static_cast<ConnectionType>(conn->getConnectionType());
			}
		}

		property bool IsNull {
			bool get()
			{
				return conn == nullptr;
			}
		}

		List<PatchRef>^ GetPatches() {
			List<PatchRef>^ patches = gcnew List<PatchRef>();
			for each (auto patch in conn->patches) {
				if((dynamic_cast<ServoPointPatch*>(patch) != nullptr)
					|| (dynamic_cast<NewPatchLine2D3D*>(patch) != nullptr)){
					patches->Add(PatchRef(patch));
				}
			}
			return patches;
		}

		virtual bool Equals(Object^ o) override {
			if (dynamic_cast<ConnectionRef^>(o) == nullptr) return false;
			ConnectionRef^ r = dynamic_cast<ConnectionRef^>(o);
			return r->conn == conn;
		}
		virtual int GetHashCode() override {
			return (int)conn;
		}
		static property ConnectionRef Null {
			ConnectionRef get(){
				return ConnectionRef(nullptr);
			}
		}

		ConnectionRef Copy() {
			NewConnection* copied;
			if (dynamic_cast<JointConnection*>(conn) != nullptr) {
				copied = new JointConnection(conn->patches, conn->getAngle(), conn->getArticulation(), conn->getConnectionType());
			}
			else {
				copied = new NewConnection(conn->patches, conn->getAngle(), conn->getConnectionType());
			}
			return ConnectionRef(copied);
		}

		static ConnectionRef New(List<PatchRef>^ patches, double angle, ConnectionType connectionType) {
			vector<NewPatch*> nativePatches;
			for each (PatchRef ref in patches) {
				nativePatches.push_back(ref.patch);
			}
			return ConnectionRef(new NewConnection(nativePatches, angle, static_cast<NewConnection::ConnectionType>(connectionType)));
		}

		void SetArticulation(cli::array<Byte>^ binary, UIInstance^ instance);

		cli::array<Byte>^ GetArticulation() {
			if (dynamic_cast<JointConnection*>(conn) == nullptr) {
				return nullptr;
			}
			Articulation* real = dynamic_cast<JointConnection*>(conn)->getArticulation();
			if(dynamic_cast<SymbolicController*>(real->transformations[0]->controller) != nullptr){
				Articulation* copy = new Articulation(real->symbCenter);
				SymbolicController * copyController = (dynamic_cast<SymbolicController*>(real->transformations[0]->controller));
				copyController->updateSymbols();
				copy->addTranformation( new SymbolicTransformation(copyController->getLinearController(), real->transformations[0]->symbAxis, 
					real->transformations[0]->isTrans)); 
				real = copy;
			}
			proto::Articulation pro;
			ProtoConversionContext context;
			Template* nonNullTemplate = nullptr;
			for each (Template* t in conn->getTemplates()) {
				if (t != nullptr) {
					nonNullTemplate = t;
				}
			}
			if (nonNullTemplate == nullptr) {
				LOG(WARNING) << "Connection does not have a non null template";
				return nullptr;
			}
			context.populateFromTemplate(nonNullTemplate->getRoot());
			PartialConverters(context).toProto(real, &pro);
			
			auto pair = PartialConverters::SerializeToArray(pro);
			void* arr = pair.first;
			int size = pair.second;
			cli::array<Byte>^ result = gcnew cli::array<Byte>(size);
			pin_ptr<Byte> ptr = &result[0];
			memcpy(ptr, arr, size);
			free(arr);
			return result;
		}
	};


	public value struct Patch3{
	public:
		cli::array<Point3D>^ vertices;
		PatchRef patchRef;
		Patch3(PatchRef patch){
			this->patchRef = patch;
			vertices = gcnew cli::array<Point3D>(2);
			vertices[0] = patch.Vertex1;
			vertices[1] = patch.Vertex2;
		}

	};

	inline vector<NewPatch*> getAllPatches(Template* t) {
		return TemplateUtil::getForAllDescendants<NewPatch*>(t, [](TemplateElement* tele, vector<NewPatch*>& result) {
			copy(tele->getPatches().begin(), tele->getPatches().end(), back_inserter(result));
		}, [](TemplateGroup* tele, vector<NewPatch*>& result) {});
	}

	public ref class DebugInfo {
	public:
		List<String^>^ propNames;
		Dictionary<String^, Object^>^ props;
		String^ typeName;
		String^ shortDescription;
		HashSet<String^>^ aggregationPropNames;
		DebugInfo(const FabDebugging::DebugInfo& native);
	};

	public value struct DebuggableRef {
	public:
		FabDebugging::Debuggable* native;
		DebuggableRef(FabDebugging::Debuggable* native) {
			this->native = native;
		}
		property bool IsNull {
			bool get()
			{
				return native == nullptr;
			}
		}
		virtual bool Equals(Object^ o) override {
			if (dynamic_cast<DebuggableRef^>(o) == nullptr) return false;
			DebuggableRef^ r = dynamic_cast<DebuggableRef^>(o);
			return r->native == native;
		}
		virtual int GetHashCode() override {
			return (int)native;
		}
		static property DebuggableRef Null {
			DebuggableRef get(){
				return DebuggableRef(nullptr);
			}
		}
		DebugInfo^ getDebugInfo() {
			auto nativeInfo = native->getDebugInfo();
			return gcnew DebugInfo(*nativeInfo);
			delete nativeInfo;
		}
		property TemplateRef AsTemplate {
			TemplateRef get();
		}
		property PatchRef AsPatch {
			PatchRef get() {
				if (dynamic_cast<NewPatch*>(native) != nullptr) {
					return PatchRef(dynamic_cast<NewPatch*>(native));
				}
				return PatchRef::Null;
			}
		}
	};

	DebugInfo::DebugInfo(const FabDebugging::DebugInfo& native) {
		propNames = gcnew List<String^>();
		props = gcnew Dictionary<String^, Object^>();
		aggregationPropNames = gcnew HashSet<String^>();

		for each (const auto& pair in native.getProperties()) {
			auto name = marshal_as<String^>(pair.first);
			propNames->Add(name);
			switch (pair.second) {
			case FabDebugging::DebugInfo::PropertyType::INT:
				props[name] = native.getInt(pair.first);
				break;
			case FabDebugging::DebugInfo::PropertyType::DOUBLE:
				props[name] = native.getDouble(pair.first);
				break;
			case FabDebugging::DebugInfo::PropertyType::BOOL:
				props[name] = native.getBool(pair.first);
				break;
			case FabDebugging::DebugInfo::PropertyType::STRING:
				props[name] = marshal_as<String^>(native.getString(pair.first));
				break;
			case FabDebugging::DebugInfo::PropertyType::AGGREGATION:
				props[name] = DebuggableRef(native.getAggregationOrReference(pair.first));
				aggregationPropNames->Add(name);
				break;
			case FabDebugging::DebugInfo::PropertyType::REFERENCE:
				props[name] = DebuggableRef(native.getAggregationOrReference(pair.first));
				break;
			}
		}
		typeName = marshal_as<String^>(native.getTypeName());
		shortDescription = marshal_as<String^>(native.getShortDescription());
	}

	Point3D toPoint3D(const Eigen::Vector3d& v) {
		return Point3D(v[0], v[1], v[2]);
	}
	Vector3D toVector3D(const Eigen::Vector3d& v) {
		return Vector3D(v[0], v[1], v[2]);
	}

	public ref class JointLabeling {
	public:
		List<int>^ JointLabels;
		List<int>^ JointAngles;
		
		JointLabeling() {
			JointLabels = gcnew List<int>();
			JointAngles = gcnew List<int>();
		}
	};

	public ref class JointLayout {
	public:
		List<Point>^ RobotShape;
		List<Point>^ Joints;

		JointLayout() {
			RobotShape = gcnew List<Point>();
			Joints = gcnew List<Point>();
		}
	};

	public ref class JointChoices {
	public:
		//JointLayout^ Layout;
		List<JointLabeling^>^ Labelings;
		//List<int>^ CurrentChoice;
		//double Theta;

		JointChoices() {
			Labelings = gcnew List<JointLabeling^>();
		}
	};

	public ref class SingleGait {
	public:
		List<int>^ JointLabels;
		List<double>^ JointAngles;
		String^ name;
		double desiredAngle;
		int id;
		SingleGait( FabByExample::GaitInfo * gaitInfo, int _id) {
			JointLabels = gcnew List<int>(gaitInfo->jointInfo.size());
			JointAngles = gcnew List<double>(gaitInfo->jointInfo.size());
			for (int i = 0; i < gaitInfo->jointInfo.size(); i++){
				JointLabels->Add( gaitInfo->jointInfo[i].first);
				JointAngles->Add( gaitInfo->jointInfo[i].second);
			}
			this->name = marshal_as<String^>(gaitInfo->name);
			desiredAngle = gaitInfo->desiredDirection;
			id = _id;
		}
	};

	public ref class GaitInfo {
	public:
		List<SingleGait^>^ savedGaits;
		List<int>^ Sequence;

		GaitInfo() {
			savedGaits = gcnew List<SingleGait^>();
			Sequence = gcnew List<int>();
		}
	};
	


	public ref class SymmetryChoices {
	public:
		bool symm_ground;
		bool symm_legW;
		bool symm_legL;
		bool symm_spacing;

		SymmetryChoices() {
		}
	};


	public value struct TemplateRef {
	public:
		Template* tmpl;
		TemplateRef(Template* tmpl)
		{
			this->tmpl = tmpl;
		}
		
		property Point3D Center {
			Point3D get(){
				Eigen::Vector3d center = tmpl->getCenter();
				return Point3D(center[0], center[1], center[2]);
			}
		}

		property Int32 Id{
			Int32 get(){
				return this->tmpl->getID();
			}
		}

		property List<TemplateRef>^ Children {
			List<TemplateRef>^ get()
			{
				List<TemplateRef>^ result = gcnew List<TemplateRef>();
				for each (Template* child in tmpl->getChildren())
				{
					result->Add(TemplateRef(child));
				}
				return result;
			}
		}
		property bool IsLeaf {
			bool get()
			{
				return tmpl->isLeaf();
			}
		}
		property DebuggableRef Debuggable {
			DebuggableRef get()
			{
				return DebuggableRef(tmpl);
			}
		}
		property TemplateRef Parent {
			TemplateRef get()
			{
				return TemplateRef(tmpl->getParent());
			}
		}
		property TemplateRef Root {
			TemplateRef get()
			{
				return TemplateRef(tmpl->getRoot());
			}
		}
		property double LowestZ {
			double get() {
				return tmpl->getLowestZ();
			}
		}
		bool IsAncestorOf(TemplateRef other)
		{
			while (other.tmpl != nullptr && other.tmpl != tmpl)
			{
				other = other.Parent;
			}
			return other.tmpl != nullptr;
		}
		property bool IsNull {
			bool get()
			{
				return tmpl == nullptr;
			}
		}
		virtual bool Equals(Object^ o) override {
			if (dynamic_cast<TemplateRef^>(o) == nullptr) return false;
			TemplateRef^ r = dynamic_cast<TemplateRef^>(o);
			return r->tmpl == tmpl;
		}

		virtual int GetHashCode() override {
			return (int)tmpl;
		}
		static property TemplateRef Null {
			TemplateRef get(){
				return TemplateRef(nullptr);
			}
		}
		Nullable<SubElementFace> getSubElementFace(int id) {
			if (dynamic_cast<TemplateElement*>(tmpl) != nullptr) {
				TemplateElement* tmplEle = dynamic_cast<TemplateElement*>(tmpl);
				auto ele = dynamic_cast<Element_Symbolic*>(tmplEle->getElement());
				if (ele == nullptr) {
					return Nullable<SubElementFace>();
				}
				auto subElements = ele->getSubElements();
				SubElementFace face(subElements->faces[id]);
				return Nullable<SubElementFace>(face);
			}
			else {
				return Nullable<SubElementFace>();
			}
		}

		List<PatchRef>^ getAllPatches(){
			List<PatchRef>^ res = gcnew List<PatchRef>();
			auto patches = CppCsBridge::getAllPatches(tmpl);
			for (int i = 0; i < patches.size(); i++){
				if((dynamic_cast<ServoPointPatch*>(patches[i]) != nullptr)
					|| (dynamic_cast<NewPatchLine2D3D*>(patches[i]) != nullptr)){
					res->Add(PatchRef(patches[i]));
				}
			}
			return res;
			
		}
		
		List<ConnectionRef>^ getAllConnectionsForPatch(PatchRef patch){
			List<ConnectionRef>^ res = gcnew List<ConnectionRef>();
			if (!IsNull) {
				vector<NewConnection*> conn;
				tmpl->getConnections(conn, TreeScope::DESCENDANTS);

				for (int i = 0; i < conn.size(); i++){
					auto listOfPatches = conn[i]->getPatches();
					for (int j = 0; j < listOfPatches.size(); j++){
						if (patch.Equals(PatchRef(listOfPatches[j]))){
							res->Add(ConnectionRef(conn[i]));
							break;
						}
					}

				}
			}
			return res;

		}

		List<ConnectionRef>^ getConnections() {
			List<ConnectionRef>^ res = gcnew List<ConnectionRef>();
			if (!IsNull) {
				const vector<NewConnection*>& conns = tmpl->getConnections();
				for each (NewConnection* conn in conns) {
					res->Add(ConnectionRef(conn));
				}
			}
			return res;
		}

		void moveToGround(){
			tmpl->moveToGround();
		}

		List<Tuple<Point3D, Vector3D>^>^ getArticulationAxes() {
			List<Tuple<Point3D, Vector3D>^>^ results = gcnew List < Tuple<Point3D, Vector3D>^ >();
			vector<NewConnection*> connections;
			tmpl->getConnections(connections, TreeScope::DESCENDANTS);
			for each (auto nc in connections) {
				if (nc->getArticulation() != nullptr) {
					auto art = nc->getArticulation();
					//printf("Has articulation");
					for each (auto trans in art->transformations) {
						//printf("Has articulation transform");
						Point3D center = toPoint3D(art->symbCenter.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES));
						Vector3D axis = toVector3D(trans->symbAxis.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES));
						results->Add(gcnew Tuple<Point3D, Vector3D>(center, axis));
					}
				}
			}
			if (results->Count == 0){
				//printf("There is no articulation axis");
			}
			return results;
		}

		void Segregate() {
			Root.tmpl->findDescendantByID(tmpl->getID())->segregate();
		}

		void Remove() {
			tmpl->remove();
		}




		property bool IsIndependent {
			bool get() {
				return tmpl->isIndependent;
			}
			void set(bool value) {
				tmpl->isIndependent = value;
			}
		}

		// Get the serialized byte[] of OpenscadDesign, if the Template encloses an Element_Openscad.
		// Otherwise, return null.
		cli::array<Byte>^ GetOpenscadDesign() {
			if (!IsLeaf ) {
				return nullptr;
			}
			Element_Openscad* scad = dynamic_cast<Element_Openscad*>(dynamic_cast<TemplateElement*>(tmpl)->getElement());
			if (scad == nullptr) {
				return nullptr;
			}
			proto::OpenscadDesign pro;
			PartialConverters(ProtoConversionContext()).toProto(scad, &pro);

			auto pair = PartialConverters::SerializeToArray(pro);
			void* arr = pair.first;
			int size = pair.second;
			cli::array<Byte>^ result = gcnew cli::array<Byte>(size);
			pin_ptr<Byte> ptr = &result[0];
			memcpy(ptr, arr, size);
			free(arr);
			return result;
		}

		// Modify the Element_Openscad under this Template to be the same as the OpenscadDesign encoded in this byte[].
		void SetOpenscadDesign(cli::array<Byte>^ binary) {
			pin_ptr<Byte> ptr = &binary[0];
			const void* native = ptr;
			proto::OpenscadDesign pro;
			PartialConverters::ParseFromArray(native, binary->Length, &pro);
			Element_Openscad* real = PartialConverters(ProtoConversionContext()).fromProto(pro);
			dynamic_cast<Element_Openscad*>(dynamic_cast<TemplateElement*>(tmpl)->getElement())->modify(real);
		}

		void AddConstraint(cli::array<Tuple<double, int>^>^ coeffs, double constant, bool isEquality) {
			CoeffsMap coeffsMap;
			for each(Tuple<double, int>^ coeff in coeffs) {
				coeffsMap.put(Symbol(tmpl, coeff->Item2), coeff->Item1);
				if (coeff->Item2 < 0 || coeff->Item2 >= tmpl->numSymbols()) {
					throw gcnew ArgumentException("Symbol ID out of bounds.");
				}
			}
			LinearExpr le(coeffsMap, -constant);
			Constraint* con = new Constraint(isEquality ? Constraint::ConstraintRelation::EQ : Constraint::ConstraintRelation::INEQ, le);
			tmpl->addConstraint(con);
		}
	};

	TemplateRef DebuggableRef::AsTemplate::get() {
		if (dynamic_cast<Template*>(native) != nullptr) {
			return TemplateRef(dynamic_cast<Template*>(native));
		}
		return TemplateRef::Null;
	}




	TemplateRef PatchRef::OwningTemplate::get() {
		return TemplateRef(patch->getElement());
	}

	public ref class Mesh {
	public:
		List<Point3D>^ points;
		List<Vector3D>^ normals;
		List<Face3>^ faces;
		List<Edge3>^ edges;
		List<Patch3>^ patches;
		List<SubElementFace>^ subElementFaces;
		TemplateRef tmpl;
		Mesh(const FabByExample::mesh::Mesh& native, Template* tmpl) {
			points = gcnew List<Point3D>();
			normals = gcnew List<Vector3D>();
			faces = gcnew List<Face3>();
			edges = gcnew List<Edge3>();
			patches = gcnew List<Patch3>();
			subElementFaces = gcnew List<SubElementFace>();
			for (int i = 0; i<native.points.size(); i++) {
				Point3D v;
				v.X = native.points[i][0];
				v.Y = native.points[i][1];
				v.Z = native.points[i][2];
				points->Add(v);
			}
			for (int i = 0; i<native.faces.size(); i++) {
				faces->Add(Face3(native.faces[i]));
			}
			for (int i = 0; i<native.edges.size(); i++) {
				edges->Add(Edge3(native.edges[i]));
			}
			/*
			TemplateElement* tEle = dynamic_cast<TemplateElement*> (tmpl);
			for (int i = 0; i < tEle->patches.size(); i++) {
				patches->Add(Patch3(PatchRef(tEle->patches[i])));
			}*/
			for (int i = 0; i<native.subElementFaces.size(); i++) {
				subElementFaces->Add(SubElementFace(native.subElementFaces[i]));
			}
			this->tmpl = TemplateRef(tmpl);
			List<PatchRef>^ patchRefs = this->tmpl.getAllPatches();
			for (int i = 0; i < patchRefs->Count; i++) {
				patches->Add(Patch3(patchRefs[i]));
			}

		}
		
		Mesh(const TriMesh& native, Template* tmpl){
			points = gcnew List<Point3D>();
			normals = gcnew List<Vector3D>();
			faces = gcnew List<Face3>();
			edges = gcnew List<Edge3>();
			subElementFaces = gcnew List<SubElementFace>();
			patches = gcnew List<Patch3>();
			this->tmpl = TemplateRef(tmpl);
			List<PatchRef>^ patchRefs = this->tmpl.getAllPatches();
			for (int i = 0; i < patchRefs->Count; i++) {
				patches->Add(Patch3(patchRefs[i]));
			}
			for (int i = 0; i < native.faces.size(); i++){
				TriMesh::Face f = native.faces[i];
				faces->Add(Face3(f));
			}
			for (int i = 0; i < native.vertices.size(); i++){
				Point3D v;
				v.X = native.vertices[i][0];
				v.Y = native.vertices[i][1];
				v.Z = native.vertices[i][2];
				points->Add(v);			
			}
			for (int i = 0; i < native.normals.size(); i++){
				Vector3D v;
				v.X = native.normals[i][0];
				v.Y = native.normals[i][1];
				v.Z = native.normals[i][2];
				normals->Add(v);
			}
		}
	};
    public ref class Drawing {
    public:
        List<Edge>^ edges;
        List<Face>^ faces;
        Drawing(const FabByExample::drawing::Drawing& native) {
            edges = gcnew List<Edge>();
            faces = gcnew List<Face>();
            for(int i=0;i<native.edges.size();i++) {
                edges->Add(Edge(native.edges[i], native));
            }
            for(int i=0;i<native.faces.size();i++) {
                faces->Add(Face(native.faces[i]));
            }
        }
    };

	public ref class PointList2D{
	public:
		cli::array<Point>^ points;
		PointList2D(){}
        PointList2D(const std::vector<std::pair<double,double>>& native) {
            this->points = gcnew cli::array<Point>(native.size());
			for(int i = 0; i < native.size(); i++){
				double p1 = native[i].first;
				double p2 = native[i].second;
				this->points[i] = Point(p1, p2);
			}
		}
	};



	public ref class PatchPair
	{
	public:
		PatchRef MainTemplatePatch;
		PatchRef AddTemplatePatch;
		bool IsOpposite;
		bool IsOpposite2D;
		PatchPair(FabByExample::PatchPair& native) {
			MainTemplatePatch = PatchRef(native.wt_patch);
			AddTemplatePatch = PatchRef(native.add_patch);
			IsOpposite = native.isOpposite;
		}
		static FabByExample::PatchPair ToNative(PatchPair^ pp) {
			if (pp == nullptr) {
				return FabByExample::PatchPair(nullptr, nullptr, false, false);
			}
			else {
				return FabByExample::PatchPair(pp->MainTemplatePatch.patch, pp->AddTemplatePatch.patch, pp->IsOpposite, pp->IsOpposite2D);
			}
		}
	};

	public ref class UIInstance {
	private:
		abstractUI* ui;
		bool DEBUG_ANIMATION;

	public:
		UIInstance() {
			ui = new abstractUI();
			DEBUG_ANIMATION = false;
		}
		~UIInstance() {
			delete ui;
		}

		abstractUI* getUI() {
			return ui;
		}


		void Save(String^ filename) {
			if (ui->workingTemplates.size() == 0) return;
			string cppFilename = marshal_as<string>(filename);
			ui->saveProto(cppFilename); 
		}

		void HandleClear() { ui->handleClear(); }
		PatchPair^ HandleSnap(PatchPair^ patchPair, double maxDistance) {
			FabByExample::PatchPair newPatchPair = ui->handleSnap(PatchPair::ToNative(patchPair), maxDistance);
			return gcnew PatchPair(newPatchPair);
		}
		void MultiSnap(double maxDistance) {
			ui->handleMultiSnap(maxDistance);
		}
		void HandleConnect() { ui->handleConnect(); }
		void HandleSnapToGround() { 
			//ui->snapToGround(); 
			ui->workingTemplates[0]->updateSymmetryChoices(true, false, false, false);
		}
		
		List<TemplateRef>^ getClosesetElements() {
			List<TemplateRef>^ results =  gcnew List<TemplateRef>();
			for each ( NewPatch* t in ui->getClossestPatch()){
				results->Add( TemplateRef(t->getElement()));
			}
			return results;
			//results TemplateRef(ui->getClossestPatch());
		}


	   List<List<Patch3>^>^ getClosesetPatch() {
		   List<List<Patch3>^>^ allresults = gcnew List<List<Patch3>^>();
			for each ( NewPatch* patch in ui->getClossestPatch()){
				List<Patch3>^ results =  gcnew List<Patch3>();
				if( dynamic_cast<PeripheralPatch*>(patch) !=nullptr){
					PeripheralPatch* p = dynamic_cast<PeripheralPatch*>(patch);
					results->Add(Patch3( PatchRef(p->getLinePatch1())));
					results->Add(Patch3( PatchRef(p->getLinePatch2())));
				}else{
					results->Add(Patch3 (PatchRef(patch)));
				}
				allresults->Add(results);
			}
			return allresults;
			//results TemplateRef(ui->getClossestPatch());
		}


	   double getTotalSequenceTime(){
	   	if (ui->workingTemplates.size() == 0) {
				return 0;
			}
			return ui->workingTemplates[0]->kinchain->getTotalTimeSequence();	
		}

		List<Mesh^>^ getAnimationMeshes(double t) {
			//just the gait, not the actual anim
			if (ui->workingTemplates.size() == 0) {
				LOG(ERROR) << "Cannot get animation meshes without a template present.";
				return gcnew List<Mesh^>();
			}
			List<Mesh^>^ results = gcnew List<Mesh^>();		
			FabByExample::Geometry* geo = ui->workingTemplates[0]->getAnimation(t);		
			TriMesh* tMesh = geo->getMesh();
			Mesh^ m = gcnew Mesh(*tMesh, ui->workingTemplates[0]->getTemplate());
			results->Add(m);
			return results;
		}

		List<Mesh^>^ getAllPoseMeshes(){
			List<Mesh^>^ results = gcnew List<Mesh^>();
			vector<FabByExample::Geometry*> anim = ui->workingTemplates[0]->kinchain->getAnimation(false, 10);	
			for each (FabByExample::Geometry* geo in anim){
				TriMesh* tMesh = geo->getMesh();
				Mesh^ m = gcnew Mesh(*tMesh, ui->workingTemplates[0]->getTemplate());
				results->Add(m);
			}	
			return results;
		}


		List<Mesh^>^ getAllAnimationMeshes(bool useSteadyState, bool doSequence){
			if (ui->workingTemplates.size() == 0) {
				LOG(ERROR) << "Cannot get all animation meshes without a template present.";
				return gcnew List<Mesh^>();
			}
			List<Mesh^>^ results = gcnew List<Mesh^>();
			vector<FabByExample::Geometry*> anim;
			vector<double> stabilityCost;
			ui->workingTemplates[0]->getNewAnimation(useSteadyState, anim, doSequence, stabilityCost);	
					
			
			// debugging animation/stability
			if (DEBUG_ANIMATION){
				ofstream myfile;
				myfile.open("..\\..\\data\\test\\stabilityUI.txt");
				for (int i = 0; i < anim.size(); i++){
					stringstream geoFileName;
					geoFileName << "..\\..\\data\\test\\newAnimUI_tAt" << i << ".stl";
					anim[i]->write(geoFileName.str());
					myfile << stabilityCost[i] << "\n";
				}
				myfile.close();
			}

			if (anim.size() != stabilityCost.size()){
				cout << "Error: animation mesh list size doesn't match stability list size " << endl;
			}
			for each (FabByExample::Geometry* geo in anim){
				TriMesh* tMesh = geo->getMesh();
				Mesh^ m = gcnew Mesh(*tMesh, ui->workingTemplates[0]->getTemplate());
				results->Add(m);
			}	
			return results;
		}	

		Mesh^ getObstacle(){
			TriMesh* tMesh = TriMesh::read("..\\..\\data\\obstacle.obj");
			Mesh^ m = gcnew Mesh(*tMesh, ui->workingTemplates[0]->getTemplate());
			return m;
		}

		List<Mesh^>^ getAllViewGaitMeshes(){
			if (ui->workingTemplates.size() == 0) {
				LOG(ERROR) << "Cannot get all animation meshes without a template present.";
				return gcnew List<Mesh^>();
			}
			List<Mesh^>^ results = gcnew List<Mesh^>();
			vector<FabByExample::Geometry*> anim;
			ui->workingTemplates[0]->getViewGait(anim);	
					
			
			for each (FabByExample::Geometry* geo in anim){
				TriMesh* tMesh = geo->getMesh();
				Mesh^ m = gcnew Mesh(*tMesh, ui->workingTemplates[0]->getTemplate());
				results->Add(m);
			}	
			return results;
		}
		List<bool>^ getAllAnimationStabilityCosts(bool useSteadyState, bool doSequence){
			if (ui->workingTemplates.size() == 0) {
				LOG(ERROR) << "Cannot get all animation stability costs without a template present.";
				return gcnew List<bool>();
			}
			List<bool>^ results = gcnew List<bool>();
			vector<FabByExample::Geometry*> anim;
			vector<double> stabilityCost;
			ui->workingTemplates[0]->getNewAnimation(useSteadyState, anim, doSequence, stabilityCost);
			const double epsilon = 0.5;
			for each (auto cost in stabilityCost){
				if (cost < epsilon){
					results->Add(true);
				}
				else{
					results->Add(false);
				}
			}
			return results;

		}
		List<bool>^ getAllAnimationObstacleColision(bool doSequence){
			if (ui->workingTemplates.size() == 0) {
				LOG(ERROR) << "Cannot get all animation stability costs without a template present.";
				return gcnew List<bool>();
			}
			List<bool>^ results = gcnew List<bool>();
			vector<FabByExample::Geometry*> anim;
			vector<double> stabilityCost;
			vector<bool> obstacleColision;
			ui->workingTemplates[0]->getNewAnimation(false, anim, doSequence, stabilityCost, true, obstacleColision);
			const double epsilon = 0.5;
			for each (auto cost in obstacleColision){
				if (cost){
					results->Add(false);
				}
				else{ 
					results->Add(true);
				}
			}
			return results;

		}

		void updateCurrentSequenceInWt(List<int>^ sequence){
			if (ui->workingTemplates.size() == 0) return;
			std::vector<int> seq;
			for (int i = 0; i < sequence->Count; i++){
				seq.push_back(sequence[i]); 
			}
			ui->workingTemplates[0]->kinchain->updateSequence(seq);
		
		}


		List<int>^ getSequence(){
			if (ui->workingTemplates.size() == 0) return nullptr;
			List<int>^ sequence = gcnew List<int>();
			for each (auto s in ui->workingTemplates[0]->kinchain->getSequence()){
				sequence->Add(s);
			}
			return sequence;
		}


			List<int>^ getInterpSequence(){
			if (ui->workingTemplates.size() == 0) return nullptr;
			List<int>^ sequence = gcnew List<int>();
			for each (auto s in ui->workingTemplates[0]->kinchain->getInterpGaitSequenceForDrawingPath()){
				sequence->Add(s);
			}
			return sequence;
		}

		

		void updateMetrics(int Nrounds, double delta, bool doSequence) {
			if (ui->workingTemplates.size() == 0) return;
			ui->workingTemplates[0]->computeMetrics(Nrounds, delta, doSequence);
		}

		double getCurvature(){
			if (ui->workingTemplates.size() == 0) return 0;
			return ui->workingTemplates[0]->metricsInfo->curvature;
		}

		double getMeanAngle(){
			if (ui->workingTemplates.size() == 0) return 0;
			return ui->workingTemplates[0]->metricsInfo->meanangle;
		}

		double getRotation(){
			if (ui->workingTemplates.size() == 0) return 0;
			return ui->workingTemplates[0]->metricsInfo->rotation;
		}

		double getFabCost(){
			if (ui->workingTemplates.size() == 0) return 0;
			return ui->workingTemplates[0]->metricsInfo->fabCost;
		}

		double getTotalMass(){
			if (ui->workingTemplates.size() == 0) return 0;
			return ui->workingTemplates[0]->metricsInfo->totalMass;
		}

		double getElectronicsCost(){
			if (ui->workingTemplates.size() == 0) return 0;
			return ui->workingTemplates[0]->metricsInfo->electronicsCost;
		}

		double getToppling(){
			if (ui->workingTemplates.size() == 0) return 0;
			return ui->workingTemplates[0]->metricsInfo->toppling;
		}
		bool isStable(){
			if (ui->workingTemplates.size() == 0) return true;
			/*vector<FabByExample::Geometry*> anim;
			vector<double> stabilityCost;
			double cost = 0;
			cost = ui->workingTemplates[0]->getNewAnimation(anim, stabilityCost, 1);
			return cost == 0;*/
			return ui->workingTemplates[0]->metricsInfo->isStable;

		}

		PointList2D ^ getGaitPath(){
			if (ui->workingTemplates.size() == 0) 
				return gcnew PointList2D();
			PointList2D^ gaitPath = gcnew PointList2D(ui->workingTemplates[0]->metricsInfo->path);
			return gaitPath; 
		}

		PointList2D^ getTopViewShape(){
		if (ui->workingTemplates.size() == 0) 
				return gcnew PointList2D();
			PointList2D^ topView = gcnew PointList2D(ui->workingTemplates[0]->getTopView());
			return topView; 
		}

		double getSpeed(){
			if (ui->workingTemplates.size() == 0) return 0;
			return ui->workingTemplates[0]->metricsInfo->speed;
		}
		
		double getSlip(){
			if (ui->workingTemplates.size() == 0) return 0;
			return ui->workingTemplates[0]->metricsInfo->slip;
		}

		double getPerpendicularError() {
			if (ui->workingTemplates.size() == 0) return 0;
			return ui->workingTemplates[0]->metricsInfo->perperror;
		}
		
		double getForwardTravel() {
			if (ui->workingTemplates.size() == 0) return 0;
			return ui->workingTemplates[0]->metricsInfo->travel;
		}
		
		double getError(){
			if (ui->workingTemplates.size() == 0) return 0;
			return ui->workingTemplates[0]->metricsInfo->error;
		}

		bool getStabilityAnalysis(){
			if (ui->workingTemplates.size() == 0) return true;
			MetricsInfo* metricInfo = ui->workingTemplates[0]->metricsInfo;
			return metricInfo->isStable;
		}

		List<Mesh^>^ getMeshes(){
			List<Mesh^>^ results = gcnew List<Mesh^>();
			unordered_map<Template*, ::TriMesh*> meshes = ui->getMeshes();
			for each (pair<Template*, ::TriMesh*> pair in meshes)
			{
				Mesh^ m = gcnew Mesh(*(pair.second), pair.first);
				results->Add(m);
			}
			return results;
		}

		Drawing^ getTopViewDrawing(){
			if (ui->workingTemplates.size() == 0) return nullptr;
			FabByExample::drawing::Drawing d = ui->getTopViewDrawing();
				Drawing^ results = gcnew Drawing(d);
				return results;
		}
		TemplateRef AddProtoTemplate(String^ filename, int id, System::Windows::Point point) {

			string std_filename = marshal_as<string>(filename);
			cout << std_filename << endl;
			WorkingTemplate* wt = ui->addTemplateByFilename(std_filename, id,(int)point.X, (int)point.Y);
			wt->getTemplate()->isIndependent = true;
			return TemplateRef(wt->getTemplate());
		}

		void AddProtoTemplateWithPointCloud(String^ filename, int id, System::Windows::Point point) {
			string std_filename = marshal_as<string>(filename);
			ui->addTemplateByFilename(std_filename, id, (int)point.X, (int)point.Y);
		}

		void SaveGeometry(String^ path) {
			ui->generateStationary3DStl();

			LOG(INFO) << "Saving geometry to: " << marshal_as<string>(path);
			if (ui->workingTemplates.size() < 1) {
				LOG(WARNING) << "Not saving geometry due to no templates";
				return;
			}
			if (ui->workingTemplates.size() > 1) {
				LOG(WARNING) << "Multiple templates are present. Saving geometry of only the oldest";
			}
			ui->workingTemplates[0]->getTemplate()->getGeometry()->write(marshal_as<string>(path));
		}

        Dictionary<TemplateRef, Drawing^>^ getDrawings() {
			Dictionary<TemplateRef, Drawing^>^ result = gcnew Dictionary<TemplateRef, Drawing^>;
            auto drawings = ui->getDrawings();
			for each (auto drawing in drawings) {
				result[TemplateRef(drawing.first)] = gcnew Drawing(*drawing.second);
			}
			return result;
        }

		bool hasMainWorkignTemp(){
			if (ui->workingTemplates.size() >0) {
				return true;
			}
			return false;
		}

		void generateFoldableSTL() {
			ui->generateFoldableSTL();
		}
		
		void Rotate(TemplateRef ref, System::Windows::Media::Media3D::Quaternion q, System::Windows::Media::Media3D::Point3D center)
		{
			ui->rotate(ref.tmpl, q.X, q.Y, q.Z, q.W, center.X, center.Y, center.Z);
		}
		void Scale(TemplateRef ref, int axis, double amount, PatchPair^ snappingConstraint, bool preventCollisions){
			ui->scale(dynamic_cast<TemplateElement*>(ref.tmpl), 0, axis, amount, PatchPair::ToNative(snappingConstraint), preventCollisions);
		}
		void translatePart(TemplateRef ref, int axis, double amount){
			ui->translatePart(dynamic_cast<TemplateElement*>(ref.tmpl), 0, axis, amount);
		}
		void Translate(TemplateRef ref, Vector3D by)
		{
			ui->translate(ref.tmpl, by.X, by.Y, by.Z);
		}

		double GetStabilityDir(TemplateRef ref, int id, int axis, int gaitId, int objective){
			return ui->getStabilityDir(dynamic_cast<TemplateElement*>(ref.tmpl), id, axis,  gaitId, objective);
		}


		void generateStationary3DStl(){

			ui->generateStationary3DStl();
		}

		// FOR DEBUGGING ONLY: Only works for template 0
		List<double>^ getTemplate0Parameters(){
			List<double>^ result = gcnew List<double>;
			if (ui->workingTemplates.size() > 0) {
				Template* tmpl = ui->workingTemplates[0]->getTemplate();
				Eigen::VectorXd parameters = tmpl->getFullQ();

				for (int i = 0; i < parameters.size(); i++){
					result->Add(parameters[i]);
				}
			}

			return result;
		}

		// FOR DEBUGGING ONLY: Only works for template 0
		void updateTemp0ForIndividualParam(int id, double newVal){
			double oldVal = ui->workingTemplates[0]->getTemplate()->getFullQ()[id];
			ConstraintsEval::updateTempByChangingOneParam(ui->workingTemplates[0]->getTemplate(), id, newVal - oldVal);
		}

		List<TemplateRef>^ getTemplates() {
			List<TemplateRef>^ result = gcnew List<TemplateRef>();
			for each (auto workingTemplate in ui->workingTemplates) {
				result->Add(TemplateRef(workingTemplate->getTemplate()));
			}
			return result;
		}

		void setTemplate(int index, TemplateRef tmpl) {
			ui->workingTemplates[index] = new WorkingTemplate(tmpl.tmpl);
		}

		void removeTemplate(int index) {
			ui->workingTemplates.erase(ui->workingTemplates.begin() + index);
		}
		
		void exportSTL(Model3DGroup^ models, String^ path) {
			FabByExample::Geometry geo;
			for each (Model3D^ model in models->Children) {
				GeometryModel3D^ geoModel = dynamic_cast<GeometryModel3D^>(model);
				MeshGeometry3D^ mesh = dynamic_cast<MeshGeometry3D^>(geoModel->Geometry);
				TriMesh tri;
				for each (Point3D p in mesh->Positions) {
					tri.vertices.push_back(point(p.X, p.Y, p.Z));
				}
				for (int i = 0; i < mesh->TriangleIndices->Count; i+=3) {
					tri.faces.push_back(TriMesh::Face(mesh->TriangleIndices[i], mesh->TriangleIndices[i + 1], mesh->TriangleIndices[i + 2]));
				}
				geo.addMesh(&tri);
			}
			geo.write(marshal_as<string>(path));
		}

		void exportWorkingTempMesh(String^ path) {
			ui->workingTemplates[0]->writeMeshToFile(marshal_as<string>(path));
		}

		void optimizeSpeed() {
			ui->workingTemplates[0]->stabilize(FunctionEvalStability::StabilizationObjective::SPEED);
		}
		void optimizeGeometry() {
			ui->workingTemplates[0]->stabilize(FunctionEvalStability::StabilizationObjective::GEOMETRY);
		}

		void optimizeCost() {
			ui->workingTemplates[0]->stabilize(FunctionEvalStability::StabilizationObjective::FABCOST);
		}

		void optimizeNone() {
			ui->workingTemplates[0]->stabilize(FunctionEvalStability::StabilizationObjective::NONE);
		}

		void evalStablization() {
			ui->workingTemplates[0]->evalStabilization();

		}
		
		// Make a hard-coded example SCAD template and add it to the UI.
		void makeExampleScadTemplate() {
			vector<OpenscadParameter> params;
			OpenscadParameter width = { "width", 10, make_pair(0, false), make_pair(0, false) };
			OpenscadParameter height = { "height", 8, make_pair(0, false), make_pair(0, false) };
			OpenscadParameter depth = { "depth", 6, make_pair(0, false), make_pair(0, false) };
			params.push_back(width);
			params.push_back(height);
			params.push_back(depth);

			Element_Openscad* scad = new Element_Openscad(params, "cube([width, height, depth], center=false);");
			TemplateElementBuilder builder(0);
			builder.setElement(scad);
			builder.setName("example_scad");
			vector<string> paramNames;
			paramNames.push_back("width");
			paramNames.push_back("height");
			paramNames.push_back("depth");
			Eigen::VectorXd paramValues(3);
			paramValues[0] = 10;
			paramValues[1] = 8;
			paramValues[2] = 6;

			builder.defineParameters(paramNames, paramValues);
			TemplateElement* element = builder.build();
			WorkingTemplate* wt = new WorkingTemplate(element);
			ui->workingTemplates.push_back(wt);
		}

		void updateGaitSingleLable(int gaitId, int i, bool isup){
			std::cout << "is updating gait " << i << "with val " << isup << std::endl;		
			if(ui->workingTemplates.size() >0){
 				auto wt = ui->workingTemplates[0];
				wt->kinchain->updateGaitSingleJoint(gaitId, i, isup);
			}
		}

		void updateGaitSingleAngle(int gaitId, int i, double theta){
			if(ui->workingTemplates.size() >0){
 				auto wt = ui->workingTemplates[0];
				wt->kinchain->updateGaitSingleAngle(gaitId, i, theta);
			}
		}

		void updateGaitSingleDirection(int gaitId, double dir){
			if(ui->workingTemplates.size() >0){
 				auto wt = ui->workingTemplates[0];
				wt->kinchain->setDesiredDirection(gaitId, dir);
			}
		}

		void updateGaitSingleName(int gaitId, String^ newName ){
			string cppNewName = marshal_as<string>(newName);

			if(ui->workingTemplates.size() >0){
 				auto wt = ui->workingTemplates[0];
				wt->kinchain->updateGaitName(gaitId, cppNewName);
			}
		}


		List<String^>^ getAllGaitNames(){
			List<String^>^ names = gcnew List<String^>();
			if(ui->workingTemplates.size() > 0){
				auto wt = ui->workingTemplates[0];
				for each (auto s in wt->kinchain->getListofSavedGaits()){
					String^ name = marshal_as<String^>(s);
					names->Add(name);
				}
							
			}
			return names;
		}

		SingleGait^ getSingleGait(int id){
			if(ui->workingTemplates.size() >0){
 				auto wt = ui->workingTemplates[0];
				return gcnew SingleGait(wt->kinchain->getGaitInfo(id), id);
			}
		}

		void updateJointGaitChoice(int choice) {
				auto gait = ui->workingTemplates[0]->kinchain;
				gait->createNewGaitOptionFromSuggestion(choice);
		}

		JointChoices^ GetJointChoices() {
			if(ui->workingTemplates.size() >0){
 				auto wt = ui->workingTemplates[0];
				const auto& gait = wt->kinchain->getDrawableGaitOptions();
				JointChoices^ choices = gcnew JointChoices();
				for each (const auto& choice in gait.gaitOptions) {
					JointLabeling^ labeling = gcnew JointLabeling();
					for each (auto num in choice) {
						labeling->JointLabels->Add(num.first);
						labeling->JointAngles->Add(num.second);
					}
					choices->Labelings->Add(labeling);
				}
				return choices;
			}
			return nullptr;
		}
		
	//	void SetJointChoice(int choice, double theta) {
	//			auto gait = ui->workingTemplates[0]->kinchain;
	//			gait->updateGaitOption(theta, choice);
	//	}
		

		List<Point>^ getJointLocations(){
			List<Point>^ jointLocations = gcnew List<Point>();

			if(ui->workingTemplates.size() >0){
				auto wt = ui->workingTemplates[0];
				for each (const auto& point in wt->kinchain->getJointPositiont()) {
					jointLocations->Add(Point(point.x(), point.y()));
				}
			}
			return jointLocations;
		}

		void updateControllers(int id){
			ui->workingTemplates[0]->kinchain->updateControllers(id);
		}

		

	//	List<int>^ getListOfTimeIntervals(bool doSequence){
	//		List<int>^ timeinter = gcnew List<int>();
	//		std::vector<int> timevec= ui->workingTemplates[0]->kinchain->getTimePerCycle(doSequence);
	//		for(int i = 0 ; i < timevec.size(); i++){
	//			timeinter->Add(timevec[i]);
	//		}
	//		return timeinter; 
	//	}
		
		 SymmetryChoices^ GetSymmetryChoices() {
			if(ui->workingTemplates.size() >0){
				auto wt = ui->workingTemplates[0];
				SymmetryChoices^ choices = gcnew SymmetryChoices();
				choices->symm_ground = wt->tmpl->symmetryChoices.symm_ground;
				choices->symm_legW = wt->tmpl->symmetryChoices.symm_legW;
				choices->symm_legL = wt->tmpl->symmetryChoices.symm_legL;
				choices->symm_spacing = wt->tmpl->symmetryChoices.symm_spacing;
				return choices;
			}
			return nullptr;
		}

		void SetSymmetryChoice(bool symm_ground, bool symm_legW, bool symm_legL, bool symm_spacing) {
			ui->workingTemplates[0]->updateSymmetryChoices(symm_ground, symm_legW, symm_legL, symm_spacing);
		}
	};

	void ConnectionRef::SetArticulation(cli::array<Byte>^ binary, UIInstance^ instance) {
		pin_ptr<Byte> ptr = &binary[0];
		const void* native = ptr;
		proto::Articulation pro;
		PartialConverters::ParseFromArray(native, binary->Length, &pro);
		ProtoConversionContext context;
		Template* root = conn->getTemplates()[0]->getRoot();
		context.populateFromTemplate(root);
		Articulation* real = PartialConverters(context).fromProto(pro);
		dynamic_cast<JointConnection*>(conn)->setArticulation(real);
		conn->updateParameters(SymbolicAssignment::USE_CURRENT_VALUES);
		for each (auto workingTemplate in instance->getUI()->workingTemplates) {
			workingTemplate->kinchain = new KinChain(root);
		}
	}

	static int CountOverlapness(drawing::Drawing const* drawing);
	public ref class OverlapPreventionHack {
		public:
			static Func<Drawing^, int>^ countOverlapnessDelegate;
			static OverlapPreventionHack() {
				FabByExample::OverlapPreventionHack::countOverlapness = &CountOverlapness;
			}
			static void SetAlwaysReflect(bool reflect) {
				FabByExample::OverlapPreventionHack::reflectAllDrawings = reflect;
			}
			static bool IsAlwaysReflect() {
				return FabByExample::OverlapPreventionHack::reflectAllDrawings;
			}
	};
	static int CountOverlapness(FabByExample::drawing::Drawing const* drawing) {
		return OverlapPreventionHack::countOverlapnessDelegate(gcnew Drawing(*drawing));		
	}

	public ref class ProgressRetrieval {
	public:
		static String^ GetProgressString() {
			return marshal_as<String^>(_global_progress_string);
		}
	};
}
