#pragma once
#include "template.pb.h"
#include "symbolic.h"
#include <articulation.h>
#include <controller.h>
#include "template.h"
#include <google\protobuf\text_format.h>
#include "templateElement.h"
#include "templateGroup.h"
#include "element_symbolic.h"
#include "element_openscad.h"
#include "element_motion.h"
#include "NewPatch.h"
#include "constraints.h"


namespace FabByExample {


template<typename Key, typename Value>
class Bijection {
private:
	unordered_map<Key, Value> forward;
	unordered_map<Value, Key> reverse;

public:
	void put(const Key& key, const Value& value) {
		forward[key] = value;
		reverse[value] = key;
	}

	const Value& getByKey(const Key& key) const {
		return forward.at(key);
	}

	const Key& getByValue(const Value& value) const {
		return reverse.at(value);
	}

	bool hasKey(const Key& key) const {
		return forward.find(key) != forward.end();
	}

	bool hasValue(const Value& value) const {
		return reverse.find(value) != reverse.end();
	}

	int size() const {
		return forward.size();
	}
};

template<typename Key>
class IndexMap : public Bijection<Key, int> {
public:
	void add(const Key& key) {
		put(key, size());
	}

	const Key& getByIndex(int index) const { return getByValue(index); }
	bool hasIndex(int index) const { return hasValue(index); }

};


	class ProtoConversionContext {
	private:
		IndexMap<Symbol> symbols;
		IndexMap<NewPatch*> patches;
		IndexMap<Template*> templates;

	public:

		int getSymbolIndex(Symbol symbol) const {
			if (!symbols.hasKey(symbol)) {
				LOG(ERROR) << "Cannot find symbol in symbol map!";
				return -1;
			}
			return symbols.getByKey(symbol);
		}

		Symbol getSymbol(int index) const {
			if (!symbols.hasIndex(index)) {
				LOG(ERROR) << "Cannot find index in symbol map!";
				return Symbol(nullptr, -1);
			}
			return symbols.getByIndex(index);
		}

		int getPatchIndex(NewPatch* patch) const {
			if (!patches.hasKey(patch)) {
				LOG(ERROR) << "Cannot find patch in patch map!";
				return -1;
			}
			return patches.getByKey(patch);
		}

		NewPatch* getPatch(int index) const {
			if (!patches.hasIndex(index)) {
				LOG(ERROR) << "Cannot find index in patch map!";
				return nullptr;
			}
			return patches.getByIndex(index);
		}

		int getTemplateIndex(Template* tmpl) const {
			if (!templates.hasKey(tmpl)) {
				LOG(ERROR) << "Cannot find template in template map!";
				return -1;
			}
			return templates.getByKey(tmpl);
		}

		Template* getTemplate(int index) const {
			if (!templates.hasIndex(index)) {
				LOG(ERROR) << "Cannot find index in template map!";
				return nullptr;
			}
			return templates.getByIndex(index);
		}

		void populateFromTemplate(Template* root) {
			for each (auto symbol in root->getSymbols(TreeScope::DESCENDANTS)) {
				symbols.add(symbol);
			}
			for each (auto patch in root->getAllPatches()) {
				patches.add(patch);
			}
			for each (auto tmpl in root->getTemplatesByScope(TreeScope::DESCENDANTS)) {
				templates.add(tmpl);
			}
		}
	};

	class PartialConverters
	{
	private:
		const ProtoConversionContext* context;
	public:
		PartialConverters(const ProtoConversionContext& context) {
			this->context = &context;
		}

		LinearExpr fromProto(const proto::symbolic::LinearExpr& proto) {
			CoeffsMap map;
			for (int i = 0; i < proto.parameter_id_size(); i++) {
				map[context->getSymbol(proto.parameter_id(i))] = proto.coeff(i);
			}
			return LinearExpr(map, proto.const_());
		}

		void toProto(const LinearExpr& real, proto::symbolic::LinearExpr* proto) {
			for each (auto pair in real.getCoeffs()) {
				proto->add_parameter_id(context->getSymbolIndex(pair.first));
				proto->add_coeff(pair.second);
			}
			proto->set_const_(real.getConstant());
		}

		Point3S fromProto(const proto::symbolic::Point3S& proto) {
			Point3S real;
			real.x = fromProto(proto.x());
			real.y = fromProto(proto.y());
			real.z = fromProto(proto.z());
			return real;
		}

		void toProto(const Point3S& real, proto::symbolic::Point3S* proto) {
			toProto(real.x, proto->mutable_x());
			toProto(real.y, proto->mutable_y());
			toProto(real.z, proto->mutable_z());
		}

		Point2S fromProto(const proto::symbolic::Point2S& proto) {
			Point2S real;
			real.x = fromProto(proto.x());
			real.y = fromProto(proto.y());
			return real;
		}

		void toProto(const Point2S& real, proto::symbolic::Point2S* proto) {
			toProto(real.x, proto->mutable_x());
			toProto(real.y, proto->mutable_y());
		}

		void toProto(const Drawing2S& real, proto::symbolic::Drawing2S* proto) {
			for (int i = 0; i < real.vertices.size();i++) {
				const auto& vertex = real.vertices[i];
				auto* protoVertex = proto->add_vertex();
				protoVertex->set_id(i);
				toProto(vertex, protoVertex->mutable_point());
			}
			for (int i = 0; i < real.edges.size(); i++) {
				const auto& edge = real.edges[i];
				auto* protoEdge = proto->add_edge();
				protoEdge->set_id(edge.id);
				protoEdge->set_name(edge.name);
				protoEdge->set_vertex1_id(edge.vertice1);
				protoEdge->set_vertex2_id(edge.vertice2);
			}
			for (int i = 0; i < real.faces.size(); i++) {
				const auto& face = real.faces[i];
				auto* protoFace = proto->add_face();
				protoFace->set_id(face.id);
				protoFace->set_name(face.name);
				for each (int vertex in face.vertices) {
					protoFace->add_vertex_id(vertex);
				}
			}
		}

		void toProto(const Mesh3S& real, proto::symbolic::Mesh3S* proto) {
			for (int i = 0; i < real.vertices.size(); i++) {
				const auto& vertex = real.vertices[i];
				auto* protoVertex = proto->add_vertex();
				protoVertex->set_id(i);
				toProto(vertex, protoVertex->mutable_point());
			}
			for (int i = 0; i < real.faces.size(); i++) {
				const auto& face = real.faces[i];
				auto* protoFace = proto->add_face();
				protoFace->set_id(face.id);
				protoFace->set_name(face.name);
				for each (int vertex in face.vertices) {
					protoFace->add_vertex_id(vertex);
				}
			}
		}

		Controller* fromProto(const proto::symbolic::Control& proto) {
			if (proto.has_inputs()) {
				const auto& inputs = proto.inputs();
				if (inputs.has_piecewiselinear()) {
					PWLinearController* pwController = new PWLinearController();
					const auto& controlProto = inputs.piecewiselinear();
					for (int pw = 0; pw < controlProto.timesandvalues_size(); pw++){
						double t = controlProto.timesandvalues(pw).timestamp();
						double v = controlProto.timesandvalues(pw).value();
						pwController->pushBackPair(t, v);
					}
					return pwController;
				}
				else {
					LOG(ERROR) << "Nothing is supported yet except piecewiselinear";
					return nullptr;
				}
			}
			else {
				LOG(ERROR) << "No inputs in Control proto";
				return nullptr;
			}
		}

		void toProto(Controller* real, proto::symbolic::Control* proto) {
			if (dynamic_cast<PWLinearController*>(real) != nullptr) {
				PWLinearController* realPW = dynamic_cast<PWLinearController*>(real);
				auto* protoPW = proto->mutable_inputs()->mutable_piecewiselinear();
				for each (const auto& pair in realPW->controllPoints) {
					auto* pairProto = protoPW->add_timesandvalues();
					pairProto->set_timestamp(pair.t);
					pairProto->set_value(pair.val);
				}
			}
			else if (dynamic_cast<SymbolicController*>(real) != nullptr) {
				SymbolicController* realPW = dynamic_cast<SymbolicController*>(real);
				proto->mutable_inputs()->set_symboliccontroller(context->getTemplateIndex(realPW->getRefTemp()));
			}else if (dynamic_cast<GrammarController*>(real) != nullptr) {
				GrammarController* realController = dynamic_cast<GrammarController*>(real);
				auto* protoController = proto->mutable_inputs()->mutable_grammarfunction();
				protoController->set_i_interval(realController->getIinterval());
				protoController->set_theta(realController->getTheta());
				protoController->set_n_intervals(realController->getNintervals());
				protoController->set_multi(realController->getMulti());
				proto::symbolic::GrammarControllerMappingFunction_GrammarControllerType protoType;
				protoType= static_cast<proto::symbolic::GrammarControllerMappingFunction_GrammarControllerType>(static_cast<int>(realController->getType())+1);
				protoController->set_type(protoType); 
			}
			else {
				LOG(ERROR) << "Not a PW linear controller. Not supported.";
			}
		}

		SymbolicTransformation* fromProto(const proto::symbolic::Transform& proto) {
			if (!proto.has_axis()) {
				LOG(ERROR) << "Transformation does not have axis";
				return nullptr;
			}
			if (!proto.has_control()) {
				LOG(ERROR) << "Transformation does not have control";
				return nullptr;
			}
			Point3S axis = fromProto(proto.axis());
			Controller* controller = fromProto(proto.control());
			auto type = proto.has_type() ? proto.type() :
				proto::symbolic::Transform::TransformType::Transform_TransformType_REVOLUTE_TRANSFORM;
			auto isTranslation =
				type == proto::symbolic::Transform::TransformType::Transform_TransformType_PRISMATIC_TRANSFORM;
			return new SymbolicTransformation(controller, axis, isTranslation);
		}

		void toProto(SymbolicTransformation* real, proto::symbolic::Transform* proto) {
			toProto(real->symbAxis, proto->mutable_axis());
			toProto(real->controller, proto->mutable_control());
			proto->set_type(real->isTrans ?
				proto::symbolic::Transform::TransformType::Transform_TransformType_PRISMATIC_TRANSFORM :
				proto::symbolic::Transform::TransformType::Transform_TransformType_REVOLUTE_TRANSFORM);
		}

		Articulation* fromProto(const proto::Articulation& proto) {
			if (!proto.has_center()) {
				LOG(ERROR) << "Articulation does not have center";
				return nullptr;
			}

			Point3S center = fromProto(proto.center());
			Articulation* real = new Articulation(center);
			for (int k = 0; k < proto.transforms_size(); k++) {
				SymbolicTransformation* realTrans = fromProto(proto.transforms(k));
				real->addTranformation(realTrans);
			}
			return real;
		}

		void toProto(Articulation* real, proto::Articulation* proto) {
			toProto(real->symbCenter, proto->mutable_center());
			for (int i = 0; i < real->transformations.size(); i++) {
				toProto(real->transformations[i], proto->add_transforms());
			}
		}

		NewConnection* fromProto(const proto::Connection& proto) {
			const proto::ConnectionMode& mode = proto.connectionmode();
			vector<NewPatch*> patches;
			NewPatch* parentPatch = context->getPatch(proto.parentpatchref());
			patches.push_back(parentPatch);
			for (int i = 0; i < proto.patchref_size(); i++) {
				patches.push_back(context->getPatch(proto.patchref(i)));
			}
			if (mode.has_jointconnection()) {
				Articulation* articulation = fromProto(mode.jointconnection().articulations());
				const auto& printType = mode.jointconnection().printtype();
				NewConnection::ConnectionType type;
				if (printType.has_ballandsocket()) {
					type = NewConnection::ConnectionType::BALLJOINT;
				}
				else if (printType.has_prismatic()) {
					type = NewConnection::ConnectionType::PRISMATIC;
				}
				else if (printType.has_revolute()) {
					type = NewConnection::ConnectionType::HINGE;
				}
				else {
					type = NewConnection::ConnectionType::BALLJOINT;
				}
				return new JointConnection(patches, mode.jointconnection().angle(), articulation, type);
			}
			else if (mode.has_bendconnection()) {
				return new NewConnection(patches, mode.bendconnection().angle(), NewConnection::ConnectionType::TEETH);
			}
			else if (mode.has_flexconnection()) {
				return new NewConnection(patches, mode.flexconnection().angle(), NewConnection::ConnectionType::HINGE);
			}
			else if (mode.has_foldconnection()) {
				return new NewConnection(patches, mode.foldconnection().angle(), NewConnection::ConnectionType::FOLD);
			}
			LOG(ERROR) << "Unsupported connection type";
			return nullptr;
		}

		void toProto(NewConnection* real, proto::Connection* proto) {
			for each (auto patch in real->getPatches()) {
				if (proto->has_parentpatchref()) {
					proto->add_patchref(context->getPatchIndex(patch));
				}
				else {
					proto->set_parentpatchref(context->getPatchIndex(patch));
				}
			}
			if (dynamic_cast<JointConnection*>(real) != nullptr) {
				JointConnection* conn = dynamic_cast<JointConnection*>(real);
				auto* joint = proto->mutable_connectionmode()->mutable_jointconnection();
				joint->set_angle(conn->getAngle());
				toProto(conn->getArticulation(), joint->mutable_articulations());
				auto* type = joint->mutable_printtype();
				switch (conn->getConnectionType()) {
				case NewConnection::ConnectionType::BALLJOINT:
					type->set_ballandsocket(true);
					break;
				case NewConnection::ConnectionType::HINGE:
					type->set_revolute(true);
					break;
				case NewConnection::ConnectionType::PRISMATIC:
					type->set_prismatic(true);
					break;
				case NewConnection::ConnectionType::NONE:
					type->set_none(true);
					break;
				default:
					LOG(WARNING) << "Unknown connection type for joint: " << conn->getConnectionType();
					type->set_none(true);
					break;
				}
			}
			else {
				switch (real->getConnectionType()) {
				case NewConnection::ConnectionType::TEETH:
					proto->mutable_connectionmode()->mutable_bendconnection()->set_angle(real->getAngle());
					break;
				case NewConnection::ConnectionType::HINGE:
					proto->mutable_connectionmode()->mutable_flexconnection()->set_angle(real->getAngle());
					break;
				case NewConnection::ConnectionType::FOLD:
					proto->mutable_connectionmode()->mutable_foldconnection()->set_angle(real->getAngle());
					break;
				default:
					LOG(ERROR) << "Unknown connection type: " << real->getConnectionType();
					break;
				}
			}
		}

		void toProto(Constraint* real, proto::Constraint* proto) {
			auto* lc = proto->mutable_linear_constraint();
			lc->set_type(real->getRelation() == Constraint::ConstraintRelation::EQ ?
				proto::LinearConstraint_Type::LinearConstraint_Type_EQUALITY :
				proto::LinearConstraint_Type::LinearConstraint_Type_LESS_THAN_ZERO);
			toProto(real->getLinearExpr(), lc->mutable_expr());
			proto::LinearConstraint::ConstraintSemantics protoConstraintType = static_cast<proto::LinearConstraint::ConstraintSemantics>(static_cast<int>(real->getType())+1);
			lc->set_constraintsemantics(protoConstraintType);
		}

		void toProto(const OpenscadParameter& real, proto::OpenscadParameter* proto) {
			proto->set_name(real.name);
			proto->set_initial_value(real.initValue);
			if (real.min.second) {
				proto->set_min(real.min.first);
			}
			if (real.max.second) {
				proto->set_max(real.max.first);
			}
		}

		void toProto(Element_Openscad* real, proto::OpenscadDesign* proto) {
			for each (const auto& param in real->getParams()) {
				toProto(param, proto->add_parameter());
			}
			proto->set_code(real->getCode());
		}

		
		void toProto(Element_Motion* real, proto::SymbolicWayPointsMappingFunction* proto) {
			for each (const auto& wayPoint in real->controlPointsS) {
				auto protoWayPoint = proto->add_symbolicwaypoint();
				toProto(wayPoint.t, protoWayPoint->mutable_time());
				toProto(wayPoint.val, protoWayPoint->mutable_value());
				protoWayPoint->set_contact(wayPoint.contact);
				protoWayPoint->set_ismoving(wayPoint.ismoving);
			}
		}

		OpenscadParameter fromProto(const proto::OpenscadParameter& proto) {
			OpenscadParameter param;
			param.name = proto.name();
			param.initValue = proto.initial_value();
			param.min = make_pair(proto.min(), proto.has_min());
			param.max = make_pair(proto.max(), proto.has_max());
			return param;
		}

		Element_Openscad* fromProto(const proto::OpenscadDesign& proto) {
			vector<OpenscadParameter> params;
			for (int i = 0; i < proto.parameter_size(); i++) {
				params.push_back(fromProto(proto.parameter(i)));
			}
			return new Element_Openscad(params, proto.code());
		}

		private:
		void toProtoTemplateCommon(Template* tmpl, proto::Template* proto) {
			proto->set_id(context->getTemplateIndex(tmpl));
			proto->set_name(tmpl->getName());


			proto::Semantics* semantics(new proto::Semantics); 
			proto::Semantics_PartType protoPartType = static_cast<proto::Semantics_PartType>(static_cast<int>(tmpl->getPartType())+1);
			proto::Semantics_PrintMethod protoPrintMethod = static_cast<proto::Semantics_PrintMethod>(static_cast<int>(tmpl->getPrintMethod())+1);
			semantics->set_printmethod(protoPrintMethod);
			semantics->set_parttype(protoPartType);
			proto->set_allocated_semantics(semantics);
			//Semantics::PartType partType= static_cast<Semantics::PartType>(static_cast<int>(tmplProto.semantics().parttype())-1);


			proto::SymmetryChoices* symmetrychoices(new proto::SymmetryChoices);
			symmetrychoices->set_symm_ground(tmpl->symmetryChoices.symm_ground);
			symmetrychoices->set_symm_legl(tmpl->symmetryChoices.symm_legL);
			symmetrychoices->set_symm_legw(tmpl->symmetryChoices.symm_legW);
			symmetrychoices->set_symm_spacing(tmpl->symmetryChoices.symm_spacing);
			proto->set_allocated_symmetrychoices(symmetrychoices);

			for (int i = 0; i < tmpl->numSymbols(); i++) {
				Symbol symbol = tmpl->getSymbol(i);
				auto* paramProto = proto->add_parameter();
				paramProto->set_id(context->getSymbolIndex(symbol));
				paramProto->set_name(tmpl->describeSymbol(i));
				paramProto->set_default_(tmpl->getCurrentValue(i));
			}
			for each (auto conn in tmpl->getConnections()) {
				toProto(conn, proto->add_connection());
			}
			for each (auto conn in tmpl->contactPoints) {
				auto* paramProto = proto->add_contactinfo();
				toProto(conn->cPoint, paramProto->mutable_point());
				paramProto->set_hasbeenconstraint(conn->isConstrained); 
			}
			for each (auto patch in tmpl->getPatches()) {
				if (dynamic_cast<NewPatchLine2D3D*>(patch) != nullptr) {
					NewPatchLine2D3D* linePatch = dynamic_cast<NewPatchLine2D3D*>(patch);
					auto* protoPatch = proto->add_patch();
					protoPatch->set_id(context->getPatchIndex(patch));
					protoPatch->mutable_edge2spatch()->set_edgeid(linePatch->edgeId);
				}
				if (dynamic_cast<ServoPointPatch*>(patch) != nullptr) {
					ServoPointPatch* tempPatch = dynamic_cast<ServoPointPatch*>(patch);
					auto* protoPatch = proto->add_patch();
					protoPatch->set_id(context->getPatchIndex(patch));
					proto::ServoPointPatch* servoPointPatch(new proto::ServoPointPatch);
					toProto(tempPatch->center, servoPointPatch->mutable_center());
					toProto(tempPatch->getSeparation(), servoPointPatch->mutable_separation());
					Eigen::Vector3d normalVector = tempPatch->getNormal();
					servoPointPatch->add_normal(normalVector.x());
					servoPointPatch->add_normal(normalVector.y());
					servoPointPatch->add_normal(normalVector.z());
					protoPatch->set_allocated_servopointpatch(servoPointPatch);

				}
				if (dynamic_cast<ServoLinePatch*>(patch) != nullptr) {
					ServoLinePatch* tempPatch = dynamic_cast<ServoLinePatch*>(patch);
					auto* protoPatch = proto->add_patch();
					protoPatch->set_id(context->getPatchIndex(patch));
					proto::ServoLinePatch* specialProtoPatch(new proto::ServoLinePatch);
					toProto(tempPatch->getVertex1(), specialProtoPatch->mutable_startpoint());
					toProto(tempPatch->getVertex2(), specialProtoPatch->mutable_endpoint());
					Eigen::Vector3d normalVector = tempPatch->getNormal();
					specialProtoPatch->add_normal(normalVector.x());
					specialProtoPatch->add_normal(normalVector.y());
					specialProtoPatch->add_normal(normalVector.z());
					protoPatch->set_allocated_servolinepatch(specialProtoPatch);
					for each (auto servoSpacing in tempPatch->spacings){
						proto::ServoSpacingInfo * protoSpacing = specialProtoPatch->add_servospacing();
						toProto(servoSpacing.alpha, protoSpacing->mutable_alpha());
						toProto(servoSpacing.separation_h, protoSpacing->mutable_separation_h());
						for each (auto sw in servoSpacing.separation_w){
							toProto(sw, protoSpacing->add_separation_w());
						}
						for each (auto t in servoSpacing.associatedTemplates){
							protoSpacing->add_associatedtemplates(context->getTemplateIndex(t)); 
						}
					}
				}
				if (dynamic_cast<PeripheralPatch*>(patch) != nullptr) {
					PeripheralPatch* tempPatch = dynamic_cast<PeripheralPatch*>(patch);
					auto* protoPatch = proto->add_patch();
					protoPatch->set_id(context->getPatchIndex(patch));
					proto::PeripheralPatch* specialProtoPatch(new proto::PeripheralPatch);
					Eigen::Vector3d normalVector = tempPatch->getNormal();
					specialProtoPatch->add_normal(normalVector.x());
					specialProtoPatch->add_normal(normalVector.y());
					specialProtoPatch->add_normal(normalVector.z());
					specialProtoPatch->set_edgeid1(context->getPatchIndex(tempPatch->getLinePatch1()));
					specialProtoPatch->set_edgeid2(context->getPatchIndex(tempPatch->getLinePatch2()));
					protoPatch->set_allocated_peripheralpatch(specialProtoPatch);
				}
			}


			auto* cl = proto->mutable_feasible_set()->mutable_constraint_list();
			for each (auto constraint in tmpl->getConstraints()) {
				toProto(constraint, cl->add_constraint());
			}
			for each (auto child in tmpl->getChildren()) {
				cl->add_inherited_template_id(context->getTemplateIndex(child));
			}
			proto->set_independent(tmpl->isIndependent);
		}

		void toProto(TemplateElement* ele, proto::Template* proto) {
			toProtoTemplateCommon(ele, proto);
			auto* mf = proto->mutable_mapping_function();
			Element* element = ele->getElement();
			if (dynamic_cast<Element_Symbolic*>(element) != nullptr) {
				Element_Symbolic* symbEle = dynamic_cast<Element_Symbolic*>(element);
				toProto(*symbEle->getMesh3S(), mf->mutable_linear_3()->mutable_mesh());
				toProto(*symbEle->getDrawing2S(), mf->mutable_linear_2()->mutable_drawing());
			}
			else if (dynamic_cast<Element_Openscad*>(element) != nullptr) {
				Element_Openscad* scadEle = dynamic_cast<Element_Openscad*>(element);
				toProto(scadEle, mf->mutable_scad()->mutable_design());
			}
			else if (dynamic_cast<Element_Motion*>(element) != nullptr) {
				Element_Motion* motionEle = dynamic_cast<Element_Motion*>(element);
				toProto(motionEle, mf->mutable_symbolicwaypointsmappingfunction());
			}
		}

		void toProto(TemplateGroup* group, proto::Template* proto) {
			toProtoTemplateCommon(group, proto);
			auto* mf = proto->mutable_mapping_function()->mutable_composition();
			for each (auto child in group->getChildren()) {
				mf->add_template_id(context->getTemplateIndex(child));
				proto->add_child_template_id(context->getTemplateIndex(child));
			}
		}


		public:
		void toProto(Template* tmpl, proto::Template* proto) {
			if (dynamic_cast<TemplateElement*>(tmpl) != nullptr) {
				toProto(dynamic_cast<TemplateElement*>(tmpl), proto);
			}
			else {
				toProto(dynamic_cast<TemplateGroup*>(tmpl), proto);
			}
		}

		void toProto(Template* root, proto::TemplateSet* proto) {
			proto->set_root_template_id(context->getTemplateIndex(root));
			for each (Template* tmpl in root->getTemplatesByScope(TreeScope::DESCENDANTS)) {
				toProto(tmpl, proto->add_template_());
			}
		}

		template<class T>
		static void ParseFromString(const string& str, T* proto) {
			bool success = google::protobuf::TextFormat::ParseFromString(str, proto);
			if (!success) {
				LOG(ERROR) << "Protobuf parsing failure";
			}
		}

		template<class T>
		static string SerializeToString(const T& proto) {
			string output;
			google::protobuf::TextFormat::PrintToString(proto, &output);
			return output;
		}

		static void ParseFromArray(const void* arr, int size, google::protobuf::Message* proto) {
			proto->ParseFromArray(arr, size);
		}

		static pair<void*, int> SerializeToArray(google::protobuf::Message const& proto) {
			int size = proto.ByteSize();
			void* arr = malloc(size);
			proto.SerializeToArray(arr, size);
			return make_pair(arr, size);
		}
	};

}