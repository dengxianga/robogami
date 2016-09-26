#include "TemplateProtoConverter.h"

#include "template.h"
#include <vector>
#include <unordered_map>
#include "templateElement.h"
#include "templateGroup.h"
#include "NewPatch.h"
#include "symbolic.h"
#include "constraints.h"
#include <element_symbolic.h>
#include <element_motion.h>
#include "articulation.h"
#include "controller.h"
#include "element.h"
#include "PartialConverters.h"
#include "..\\FBE_Kinematics\\KinChain.h"

#define PARSER_CORRECT true

using namespace std;
namespace FabByExample {

	TemplateProtoConverter::TemplateProtoConverter() {
	}


	TemplateProtoConverter::~TemplateProtoConverter() {
	}

	void TemplateProtoConverter::AddGaitInfoToProto(KinChain* kinChain, proto::TemplateSet* proto) {
		proto::GaitInformation* gaitInfo = new proto::GaitInformation();
		for each (auto  s in kinChain->getSequence()){
			gaitInfo->add_gaitsequence(s);
		}
		for (int i = 0; i < kinChain->getNSavedGaits(); i++){
			auto g = kinChain->getGaitInfo(i);
			proto::SavedGait *savedGait = gaitInfo->add_savedgaits();
			string* name = savedGait->mutable_name();
			*name = g->name; 
			savedGait->set_desireddirection(g->desiredDirection);
			for each (auto j in g->jointInfo){
				savedGait->add_jointinfo_id(j.first);
				savedGait->add_jointinfo_angle(j.second);
			}
		}
		
		proto->set_allocated_gaitinfo(gaitInfo); 
	}

	proto::TemplateSet* TemplateProtoConverter::ConvertToProto(Template* tmpl) {
		ProtoConversionContext context;
		context.populateFromTemplate(tmpl);
		PartialConverters converter(context);
		proto::TemplateSet* result = new proto::TemplateSet();
		converter.toProto(tmpl, result);
		return result;
	}

	namespace _ProtoToTemplate {
		struct defer {
			std::function<void()> func;
			defer(std::function<void()> func) {
				this->func = func;
			}
			~defer() { func(); }
		};
		struct ConversionHelper {
			std::unordered_map<int, const proto::Template*> templateProtoMap;
			std::unordered_map<int, TemplateBuilder*> builderMap;
			std::unordered_map<int, Symbol> paramSymbolMap;
			int rootTemplate;
			std::string error;
		};

		LinearExpr protoToLinearExpr(proto::symbolic::LinearExpr const& in, ConversionHelper* helper) {
			CoeffsMap coeffs;
			for (int i = 0; i < in.coeff_size(); i++) {
				Symbol symbol = helper->paramSymbolMap.at(in.parameter_id(i));
				coeffs[symbol] = in.coeff(i);
			}
			return LinearExpr(coeffs, in.const_());
		}

		Mesh3S* protoToMesh3S(proto::symbolic::Mesh3S const& in, ConversionHelper* helper) {
			Mesh3S* result = new Mesh3S();
			for (int i = 0; i < in.vertex_size(); i++) {
				auto& vertex = in.vertex(i);
				Point3S p;
				p.x = protoToLinearExpr(vertex.point().x(), helper);
				p.y = protoToLinearExpr(vertex.point().y(), helper);
				p.z = protoToLinearExpr(vertex.point().z(), helper);
				result->vertices.push_back(p);
			}
			for (int i = 0; i < in.face_size(); i++) {
				auto& face = in.face(i);
				Face3Sp face3sp;
				for (int j = 0; j < face.vertex_id_size(); j++) {
					face3sp.vertices.push_back(face.vertex_id(j));
				}
				face3sp.id = face.has_id() ? face.id() : -1;
				face3sp.name = face.has_name() ? face.name() : "";
				result->faces.push_back(face3sp);
			}
			return result;
		}

		Drawing2S* protoToDrawing2S(proto::symbolic::Drawing2S const& in, ConversionHelper* helper) {
			Drawing2S* result = new Drawing2S();
			for (int i=0;i<in.vertex_size();i++) {
				auto& vertex = in.vertex(i);
				Point2S p;
				p.x = protoToLinearExpr(vertex.point().x(), helper);
				p.y = protoToLinearExpr(vertex.point().y(), helper);
				result->vertices.push_back(p);
			}
			for(int i=0;i<in.face_size();i++) {
				auto& face = in.face(i);
				Face2Sp face2sp;
				for(int j=0;j<face.vertex_id_size();j++) {
					face2sp.vertices.push_back(face.vertex_id(j));
				}
				face2sp.id = face.has_id() ? face.id() : -1;
				face2sp.name = face.has_name() ? face.name() : "";
				result->faces.push_back(face2sp);
			}
			for(int i=0; i<in.edge_size(); i++) {
				auto& edge = in.edge(i);
				Edge2S edge2s;

				if(edge.has_id())
					edge2s.id = edge.id();
				if(edge.has_name())
					edge2s.name = edge.name();
				if(edge.has_vertex1_id())
					edge2s.vertice1 = edge.vertex1_id();
				if(edge.has_vertex2_id())
					edge2s.vertice2 = edge.vertex2_id();
				result->edges.push_back(edge2s);

			}
			return result;
		}

		bool isZero(const proto::symbolic::LinearExpr& expr)
		{
			return expr.coeff_size() == 0 && expr.const_() == 0;
		};

		bool approx(double a, double b)
		{
			return abs(a - b) < 1e-6 || abs((b - a) / a) < 1e-6;
		};


		Element_Symbolic* tryConvertingToElementSymbolic(proto::Template const& tmpl,
			std::vector<int>* leftOverConstraints, ConversionHelper* helper) {

			if (!(tmpl.has_mapping_function() && tmpl.mapping_function().has_linear_3())) {
				helper->error = "No LinearMappingFunction3 mapping function";
				return nullptr;
			}
			if (tmpl.parameter_size() < 1) {
				helper->error = "There must be at least 3 parameters for Element_Symbolic";
				return nullptr;
			}

			auto& mapFunc = tmpl.mapping_function().linear_3();
			Mesh3S* mesh = protoToMesh3S(mapFunc.mesh(), helper);
			Drawing2S* drawing = nullptr;
			if (tmpl.mapping_function().has_linear_2()) {
				auto& mapFunc2 = tmpl.mapping_function().linear_2();
				drawing = protoToDrawing2S(mapFunc2.drawing(), helper);
			}

			//ANDYTODO: set patches and connections





			//go into the element symbolic
			//get the edge


			if (tmpl.feasible_set().has_constraint_list()) {
				auto& constraints = tmpl.feasible_set().constraint_list();
				for (int i = 0; i < constraints.constraint_size(); i++) {
					leftOverConstraints->push_back(i);
				}
			}

			return new Element_Symbolic(mesh, drawing);
		}

		Element_Openscad* tryConvertingToElementOpenscad(proto::Template const& tmpl,
			std::vector<int>* leftOverConstraints, ConversionHelper* helper) {

			if (!(tmpl.has_mapping_function() && tmpl.mapping_function().has_scad())) {
				helper->error = "No LinearMappingFunction3 mapping function";
				return nullptr;
			}
			auto& mapFunc = tmpl.mapping_function().scad();
			if (!mapFunc.has_design()) {
				helper->error = "There must be a design in scad mapping function.";
			}
			if (tmpl.parameter_size() != mapFunc.design().parameter_size()) {
				helper->error = "Number of parameters must match in template and in scad parameters.";
				return nullptr;
			}

			if (tmpl.feasible_set().has_constraint_list()) {
				auto& constraints = tmpl.feasible_set().constraint_list();
				for (int i = 0; i < constraints.constraint_size(); i++) {
					leftOverConstraints->push_back(i);
				}
			}

			return PartialConverters(ProtoConversionContext()).fromProto(mapFunc.design());
		}



		Element_Motion* tryConvertingToElementMotion(proto::Template const& tmpl,
			std::vector<int>* leftOverConstraints, ConversionHelper* helper) {

				if (!(tmpl.has_mapping_function() && tmpl.mapping_function().has_symbolicwaypointsmappingfunction())) {
				helper->error = "No control mapping function";
				return nullptr;
			}
			if (tmpl.parameter_size() < 1) {
				helper->error = "There must be at least 1 parameters for Element_Symbolic";
				return nullptr;
			}

			auto& mapFunc = tmpl.mapping_function().symbolicwaypointsmappingfunction();

			Element_Motion* el_motion = new Element_Motion();
			//std::cout << "the number of way points " << mapFunc.symbolicwaypoint_size() << std::endl;
			for(int pw = 0; pw < mapFunc.symbolicwaypoint_size(); pw++){
				LinearExpr t = protoToLinearExpr(mapFunc.symbolicwaypoint(pw).time(), helper);
				LinearExpr val = protoToLinearExpr(mapFunc.symbolicwaypoint(pw).value(), helper);
				bool contact =mapFunc.symbolicwaypoint(pw).contact();
				bool ismoving =mapFunc.symbolicwaypoint(pw).ismoving();
				el_motion->addNewControPointS(t, val, contact, ismoving);
			}

			if (tmpl.feasible_set().has_constraint_list()) {
				auto& constraints = tmpl.feasible_set().constraint_list();
				for (int i = 0; i < constraints.constraint_size(); i++) {
					leftOverConstraints->push_back(i);
				}
			}

			return el_motion;
		}


		Constraint* convertProtoToConstraint(proto::Constraint const& constraintProto,
			ConversionHelper* helper) {
				if (!constraintProto.has_linear_constraint()) {
					helper->error = "Constraint is non-linear";
					return nullptr;
				}
				auto& linearConstraint = constraintProto.linear_constraint();
				auto& exprProto = linearConstraint.expr();
				/*
				std::unordered_map<Template*, Eigen::VectorXd> coeffs;
				for (int i = 0; i < exprProto.coeff_size(); i++) {
					int param = exprProto.parameter_id(i);
					double coeff = exprProto.coeff(i);
					auto symbol = helper->paramSymbolMap.at(param);
					Template* tmpl = const_cast<Template*>(static_cast<const Template*>(symbol.owner));
					if (coeffs.find(tmpl) == coeffs.end()) {
						int size = helper->templateProtoMap.at(tmpl->getID())->parameter_size();
						coeffs[tmpl] = Eigen::VectorXd::Zero(size);
					}
					coeffs[tmpl](symbol.id) = coeff;
				}
				int bVal = -exprProto.const_();
				Constraint::CostraintRelation relation =
					linearConstraint.type() == proto::LinearConstraint::Type::LinearConstraint_Type_EQUALITY ?
					Constraint::CostraintRelation::EQ : Constraint::CostraintRelation::INEQ;

				ConstraintGeneric* constraint = new ConstraintGeneric(coeffs, bVal, relation);
				return constraint;
				*/
				LinearExpr linearExpr = protoToLinearExpr(exprProto, helper);
				Constraint::ConstraintRelation relation =
					linearConstraint.type() == proto::LinearConstraint::Type::LinearConstraint_Type_EQUALITY ?
					Constraint::ConstraintRelation::EQ : Constraint::ConstraintRelation::INEQ;
				Constraint::ConstraintType constraintType= static_cast<Constraint::ConstraintType>(static_cast<int>(linearConstraint.constraintsemantics())-1);
				auto c = new Constraint(relation, linearExpr);
				c->setConstraintType(constraintType);
				return c;
		}


		Articulation* convertProtoToArticulation(proto::Articulation const& articulationProto,
			ConversionHelper* helper, map<int, Element * > & elements) {
				if (!articulationProto.has_center()) {
					helper->error = "Articulation does not have center";
					return nullptr;
				}
				//TODO: implement the constuctor for the articulation 
				Point3S center;
				center.x = protoToLinearExpr(articulationProto.center().x(), helper);
				center.y = protoToLinearExpr(articulationProto.center().y(), helper);
				center.z = protoToLinearExpr(articulationProto.center().z(), helper);
				Articulation* articulation = new Articulation(center);
				for (int k = 0; k < articulationProto.transforms_size(); k++){
					auto new_transform = articulationProto.transforms(k);
					// get axis of tranformation
					if (!new_transform.has_axis()) {
						helper->error = "transformation does not have axis";
						return nullptr;
					}
					Point3S axis;
					axis.x = protoToLinearExpr(new_transform.axis().x(), helper);
					axis.y = protoToLinearExpr(new_transform.axis().y(), helper);
					axis.z = protoToLinearExpr(new_transform.axis().z(), helper);
					Controller * newController = nullptr;
					// create the controller
					if(new_transform.has_control()){
						if(new_transform.control().has_inputs()){
							//--------------- Addign Parametric piecewise linear funciton -------------------------------
							if(new_transform.control().inputs().has_piecewisefunction()) {
								PWLinearController * pwController = new PWLinearController();
								auto pwLinearControl = new_transform.control().inputs().piecewisefunction();
								if (pwLinearControl.pairs_size() > 0) {
									//std::cout << "there is a piecewise function control" << std::endl;
									for(int pw = 0; pw < pwLinearControl.pairs_size(); pw++){
										double t, v;
											
										//time
										if (pwLinearControl.pairs(pw).time(0).has_const_()) {
											t = pwLinearControl.pairs(pw).time(0).const_();
										}
										if (pwLinearControl.pairs(pw).time(0).coeff_size() > 0){
											//TODO
											helper->error = "not implemented: time is a function of parameters";
											return nullptr;
										}	

										//values
										if (pwLinearControl.pairs(pw).value(0).has_const_()) {
											v = pwLinearControl.pairs(pw).value(0).const_();
										}
										if (pwLinearControl.pairs(pw).value(0).coeff_size() > 0){
											//TODO
											helper->error = "not implemented: value is a function of parameters";
											return nullptr;
										}	

										//std::cout << "new time and values: t = " << t << "v = "  << v << std::endl;
										pwController->pushBackPair(t,v);
									}
									newController = (Controller*) pwController; 
									VLOG(3) << "reated pwcomtroller";
								}
							}
							//--------------- Addign piecewise linear funciton -------------------------------
							if(new_transform.control().inputs().has_piecewiselinear()){
								PWLinearController * pwController = new PWLinearController();
								auto pwLinearControl = new_transform.control().inputs().piecewiselinear();
								//std::cout << "there is a piecewise linear control" << std::endl;
								for(int pw = 0; pw < pwLinearControl.timesandvalues_size(); pw++){
									double t = pwLinearControl.timesandvalues(pw).timestamp();
									double v = pwLinearControl.timesandvalues(pw).value();
									//std::cout << "new time and values: t = " << t << "v = "  << v << std::endl;
									pwController->addPair(t,v);
								}
								newController = (Controller*) pwController; 
							}
							//--------------- Addign linear funciton -------------------------------
							if(new_transform.control().inputs().has_linear_1()){
								auto linearControl = new_transform.control().inputs().linear_1();
								//std::cout << "there is a linear control" << std::endl;
								double coeff = 0.0;
								double constant = 0.0;
								if(linearControl.has_lineartimemap()){
									if(linearControl.lineartimemap().has_const_()){
										constant = linearControl.lineartimemap().const_();
									}
									if(linearControl.lineartimemap().coeff_size() > 0){
										coeff = linearControl.lineartimemap().coeff(0);
									}		
								}
								newController = (Controller*) new LinearController(coeff, constant);
							}
							//--------------- Addign Grammar Controller -------------------------------
							if(new_transform.control().inputs().has_grammarfunction()){
								auto grammarControl = new_transform.control().inputs().grammarfunction();
								//std::cout << "there is a linear control" << std::endl;
								double theta = grammarControl.theta();
								int N_intervals = grammarControl.n_intervals();
								int i_interval = grammarControl.i_interval();
								double multi = grammarControl.multi();
								GrammarController::GrammarcontrollerType controllerType= static_cast<GrammarController::GrammarcontrollerType>(static_cast<int>(grammarControl.type())-1);

								newController = (Controller*) new GrammarController(controllerType, theta, N_intervals, i_interval,multi);
							}
							//--------------- Addign linear funciton -------------------------------
							if(new_transform.control().inputs().has_symboliccontroller()){
								Element * refElement = elements[new_transform.control().inputs().symboliccontroller()];
								Element_Motion* elMotion = dynamic_cast<Element_Motion*> (refElement);
								newController = (Controller*) new SymbolicController(elMotion);
							}

						}else{
							return nullptr;
						}
					}
					SymbolicTransformation * tranf = nullptr;
					if(new_transform.has_type()){						
						if (new_transform.type() == proto::symbolic::Transform::TransformType::Transform_TransformType_PRISMATIC_TRANSFORM){
							if (newController == nullptr) {
								printf("WARNING: Not adding transform due to null controller.");
								
							}
							else {
								tranf = new SymbolicTransformation(newController, axis, true);

							}
						}
						else if (new_transform.type() == proto::symbolic::Transform::TransformType::Transform_TransformType_REVOLUTE_TRANSFORM){
							if (newController == nullptr) {
								printf("WARNING: Not adding transform due to null controller.");

							}
							else {

								tranf = new SymbolicTransformation(newController, axis,  false);
							}
						}
						else {
							printf("WARNING: Not adding transform type %d", new_transform.type());
						}
					}else{
						tranf = new SymbolicTransformation(newController, axis, false);	
					}
					//get the type
					//Constraint::CostraintRelation relation =
					//linearConstraint.type() == proto::LinearConstraint::Type::LinearConstraint_Type_EQUALITY ?
					//Constraint::CostraintRelation::EQ : Constraint::CostraintRelation::INEQ;
					if (tranf != nullptr) {
						articulation->addTranformation(tranf);
					}
					else{
						printf("WARNING::: symbolicranformation is null");
					}
				}

				return articulation;
		}


	}


	void TemplateProtoConverter::AddGaitInfoToKinChain(const proto::TemplateSet& input, KinChain * kinchain) {
		
		if(input.has_gaitinfo()){
			kinchain->clearSavedGaits();
			proto::GaitInformation gaitInfo = input.gaitinfo();
			std::vector<int> seq;
			for (int s = 0; s <  gaitInfo.gaitsequence_size(); s++){
				seq.push_back(gaitInfo.gaitsequence(s));
			}
			kinchain->updateSequence(seq);

			for (int s = 0; s <  gaitInfo.savedgaits_size(); s++){
				auto protoGait = gaitInfo.savedgaits(s); 
				GaitInfo * gaitInfo = new GaitInfo();
				gaitInfo->name = protoGait.name();
				gaitInfo->desiredDirection = protoGait.desireddirection();
				for ( int i = 0; i < protoGait.jointinfo_angle_size(); i++){
					int pos = protoGait.jointinfo_id(i);
					double angle = protoGait.jointinfo_angle(i);
					std::pair<int, double> jointPair(pos, angle); 
					gaitInfo->jointInfo.push_back(jointPair);
				}

				kinchain->addNewGaitInfo(gaitInfo); 
			}

			

		}
	}

	Template* TemplateProtoConverter::ConvertToTemplate(const proto::TemplateSet& input) {
		using namespace _ProtoToTemplate;
		ConversionHelper helper;
		Template* root = nullptr;

		map<int, NewPatch * > patches;
		map<int, Element * > elements;

		helper.rootTemplate = input.root_template_id();

		// First build a template instance for every template, and populate the param->tmpl map
		for (int i = 0; i < input.template__size(); i++) {
			auto& tmplProto = input.template_(i);
			int tmplId = tmplProto.id();
			vector<string> parameterNames;
			Eigen::VectorXd initialValues(tmplProto.parameter_size());

			helper.templateProtoMap[tmplId] = &tmplProto;
			TemplateBuilder* builder;
			if (tmplProto.child_template_id_size() == 0) {
				builder = new TemplateElementBuilder(tmplId);
			}
			else {
				builder = new TemplateGroupBuilder(tmplId);
			}

			for (int j = 0; j < tmplProto.parameter_size(); j++) {
				parameterNames.push_back(tmplProto.parameter(j).name());
				initialValues[j] = tmplProto.parameter(j).default_();
			}
			builder->defineParameters(parameterNames, initialValues);
			for (int j = 0; j < tmplProto.parameter_size(); j++) {
				helper.paramSymbolMap[tmplProto.parameter(j).id()] = builder->getResultingPointer()->getSymbol(j);
			}

			helper.builderMap[tmplId] = builder;

			if (tmplProto.has_name()) {
				builder->setName(tmplProto.name());
			}
		}

		// then go through each template again and connect them up
		for (int i = 0; i < input.template__size(); i++) {

			auto& tmplProto = input.template_(i);
			int tmplId = tmplProto.id();
			auto builder = helper.builderMap[tmplId];
			vector<int> leftOverConstraints;
			auto constraintList = tmplProto.feasible_set().constraint_list();
			if (builder->isLeaf()) {
				Element* element;
				if ((element = tryConvertingToElementSymbolic(tmplProto, &leftOverConstraints, &helper)) != nullptr) {
					builder->asElement()->setElement(element);
				}
				else if ((element = tryConvertingToElementOpenscad(tmplProto, &leftOverConstraints, &helper)) != nullptr) {
					builder->asElement()->setElement(element);
				}
				else if ((element = tryConvertingToElementMotion(tmplProto, &leftOverConstraints, &helper)) != nullptr) {
					builder->asElement()->setElement(element);
				}
				else {
					throw std::string("Unsupported leaf template type: ") + helper.error;
				}
					elements[element->getRefTemplateElement()->getID()] = element;
			}
			else {
				// add all the children for non-leaf nodes
				auto groupBuilder = builder->asGroup();
				for (int j = 0; j < tmplProto.child_template_id_size(); j++) {
					groupBuilder->addChild(helper.builderMap[tmplProto.child_template_id(j)]->getResultingPointer());
				}
				for (int j = 0; j < constraintList.constraint_size(); j++) {
					leftOverConstraints.push_back(j);
				}
			}
			for (int j = 0; j < leftOverConstraints.size(); j++) {
				auto constraintProto = constraintList.constraint(j);
				Constraint* constraint = convertProtoToConstraint(constraintProto, &helper);
				if (constraint == nullptr) {
					std::cerr << "Constraint #" << j << " for template " << tmplId << " unsupported: " << helper.error << std::endl;
					continue;
				}
				builder->addConstraint(constraint);
			}


			

			for (int j = 0; j < tmplProto.patch_size(); j++){
				//TODO: maybe check that this isn't a group; i.e. that it has patches?
				if(tmplProto.patch(j).has_edge2spatch()){
					NewPatch * patch = builder->asElement()->createAndAddPatch(tmplProto.patch(j).edge2spatch().edgeid());
					patches[tmplProto.patch(j).id()] = patch;
				} else if(tmplProto.patch(j).has_servopointpatch()){
					Point3S center;
					center.x = protoToLinearExpr(tmplProto.patch(j).servopointpatch().center().x(), &helper);
					center.y = protoToLinearExpr(tmplProto.patch(j).servopointpatch().center().y(), &helper);
					center.z = protoToLinearExpr(tmplProto.patch(j).servopointpatch().center().z(), &helper);
					LinearExpr separation = protoToLinearExpr(tmplProto.patch(j).servopointpatch().separation(), &helper);
					Eigen::Vector3d normal;
					normal.x() = tmplProto.patch(j).servopointpatch().normal(0);
					normal.y() = tmplProto.patch(j).servopointpatch().normal(1);
					normal.z() = tmplProto.patch(j).servopointpatch().normal(2);
					NewPatch * patch =  builder->asElement()->createAndAddServoPointPatch(center, separation, normal);
					patches[tmplProto.patch(j).id()] = patch;
				} else if(tmplProto.patch(j).has_servolinepatch()){
					Point3S v1;
					v1.x = protoToLinearExpr(tmplProto.patch(j).servolinepatch().startpoint().x(), &helper);
					v1.y = protoToLinearExpr(tmplProto.patch(j).servolinepatch().startpoint().y(), &helper);
					v1.z = protoToLinearExpr(tmplProto.patch(j).servolinepatch().startpoint().z(), &helper);
					Point3S  v2;
					v2.x = protoToLinearExpr(tmplProto.patch(j).servolinepatch().endpoint().x(), &helper);
					v2.y = protoToLinearExpr(tmplProto.patch(j).servolinepatch().endpoint().y(), &helper);
					v2.z = protoToLinearExpr(tmplProto.patch(j).servolinepatch().endpoint().z(), &helper);
					Eigen::Vector3d normal;
					normal.x() = tmplProto.patch(j).servolinepatch().normal(0);
					normal.y() = tmplProto.patch(j).servolinepatch().normal(1);
					normal.z() = tmplProto.patch(j).servolinepatch().normal(2);
					NewPatch * patch = builder->asElement()->createAndAddServoLinePatch(v1, v2, normal);
					patches[tmplProto.patch(j).id()] =  patch;
					for(int s = 0; s < tmplProto.patch(j).servolinepatch().servospacing_size(); s++){
						ServoSpacing * servoSpacing = new ServoSpacing();
						auto protoServoSpacing = tmplProto.patch(j).servolinepatch().servospacing(s);
						LinearExpr alpha = protoToLinearExpr(protoServoSpacing.alpha(), &helper);
						servoSpacing->alpha = alpha;
						servoSpacing->separation_h = protoToLinearExpr(protoServoSpacing.separation_h(), &helper);
						for(int sw = 0; sw < protoServoSpacing.separation_w_size(); sw++){
							servoSpacing->separation_w.push_back(protoToLinearExpr(protoServoSpacing.separation_w(sw), &helper));
						}
						for(int tid = 0; tid < protoServoSpacing.associatedtemplates_size(); tid++){
							Template* t = helper.builderMap.at(protoServoSpacing.associatedtemplates(tid))->getResultingPointer();
							servoSpacing->associatedTemplates.push_back(t);
						}
						ServoLinePatch* linepatch = dynamic_cast<ServoLinePatch*>(patch);
						linepatch->spacings.push_back(*servoSpacing);
					}
				}
			}
			
			
			for (int j = 0; j < tmplProto.contactpoints_size(); j++){
				Point3S  p;
				p.x = protoToLinearExpr(tmplProto.contactpoints(j).x(), &helper);
				p.y = protoToLinearExpr(tmplProto.contactpoints(j).y(), &helper);
				p.z = protoToLinearExpr(tmplProto.contactpoints(j).z(), &helper);
				ContactPoint* contactPoint = new ContactPoint(p);
				builder->addContactPoint(contactPoint);
			}
			for (int j = 0; j < tmplProto.contactinfo_size(); j++){
				Point3S  p;
				p.x = protoToLinearExpr(tmplProto.contactinfo(j).point().x(), &helper);
				p.y = protoToLinearExpr(tmplProto.contactinfo(j).point().y(), &helper);
				p.z = protoToLinearExpr(tmplProto.contactinfo(j).point().z(), &helper);
				ContactPoint* contactPoint = new ContactPoint(p, tmplProto.contactinfo(j).hasbeenconstraint());
				builder->addContactPoint(contactPoint);
			}



		}




		//we need one mid pass for the peripheral patchess
		for (int i = 0; i < input.template__size(); i++) {

			auto& tmplProto = input.template_(i);
			int tmplId = tmplProto.id();
			auto builder = helper.builderMap[tmplId];


			// only add the peripheral connections in the second step
			for (int j = 0; j < tmplProto.patch_size(); j++){
				//TODO: maybe check that this isn't a group; i.e. that it has patches?
				if(tmplProto.patch(j).has_peripheralpatch()){
					int id1 = tmplProto.patch(j).peripheralpatch().edgeid1();
					int id2 = tmplProto.patch(j).peripheralpatch().edgeid2();
					Eigen::Vector3d normal;
					normal.x() = tmplProto.patch(j).peripheralpatch().normal(0);
					normal.y() = tmplProto.patch(j).peripheralpatch().normal(1);
					normal.z() = tmplProto.patch(j).peripheralpatch().normal(2);
					NewPatchLine2D3D * patchLine1 = dynamic_cast<NewPatchLine2D3D *> (patches[id1]);
					NewPatchLine2D3D * patchLine2 = dynamic_cast<NewPatchLine2D3D *> (patches[id2]);
					if(patchLine1 == nullptr){
						std::cout << "could not find patch line from id " << std::endl; system("pause");
					}
					if(patchLine2 == nullptr){
						std::cout << "could not find patch line from id " << std::endl; system("pause");
					}
					NewPatch * patch =  builder->createAndAddPeripheralPatch(patchLine1, patchLine2, normal);
					patches[tmplProto.patch(j).id()] = patch;
				}
			}

		}



		//we need one last pass to build stuff and also to add connections
		for (int i = 0; i < input.template__size(); i++) {

			auto& tmplProto = input.template_(i);
			int tmplId = tmplProto.id();
			auto builder = helper.builderMap[tmplId];




			//and now we have the connection so let's add it.
			for (int j = 0; j < tmplProto.connection_size(); j++){
				vector<NewPatch *> local_patches;
				if(tmplProto.connection(j).has_parentpatchref()){
					int patch_id = tmplProto.connection(j).parentpatchref();
					local_patches.push_back(patches[patch_id]);
				}
				for (int k = 0; k < tmplProto.connection(j).patchref_size(); k++){
					int patch_id = tmplProto.connection(j).patchref(k);
					local_patches.push_back(patches[patch_id]);
				}
				double angle = tmplProto.connection(j).connectionmode().foldconnection().angle();
				//only groups have connections
				//TODO: maybe some sort of check here as well?
				if (tmplProto.connection(j).connectionmode().has_foldconnection()){
					angle = tmplProto.connection(j).connectionmode().foldconnection().angle();
					NewConnection * conn = new NewConnection(local_patches, angle, NewConnection::ConnectionType::FOLD);
					builder->asGroup()->addConnection(conn);
				}
				else if(tmplProto.connection(j).connectionmode().has_bendconnection()){
					//VLOG(3) << "Adding teeth from PROTO......................";
					angle = tmplProto.connection(j).connectionmode().bendconnection().angle();
					NewConnection * conn = new NewConnection(local_patches, angle, NewConnection::ConnectionType::TEETH);
					builder->asGroup()->addConnection(conn);
				}
				else if(tmplProto.connection(j).connectionmode().has_flexconnection()){
					angle = tmplProto.connection(j).connectionmode().flexconnection().angle();
					NewConnection * conn = new NewConnection(local_patches, angle, NewConnection::ConnectionType::HINGE);
					builder->asGroup()->addConnection(conn);
				}
				else if(tmplProto.connection(j).connectionmode().has_jointconnection()){
					Articulation* articulation = nullptr;
					if( PARSER_CORRECT && tmplProto.connection(j).connectionmode().jointconnection().has_articulations()) {
						auto articulationProto = tmplProto.connection(j).connectionmode().jointconnection().articulations(); 
						//FabByExample::proto::Articulation
						articulation = convertProtoToArticulation(articulationProto, &helper, elements);
					}
					
					angle = tmplProto.connection(j).connectionmode().jointconnection().angle();
					
					if(articulation == nullptr){
						//TODO: add it somehow - this is a problem with the biped
						//builder->asGroup()->addConnection(new NewConnection(local_patches, angle, NewConnection::ConnectionType::FOLD));
					}
					else{	
						JointConnection * conn;
						if (tmplProto.connection(j).connectionmode().jointconnection().printtype().has_ballandsocket()){
							conn = new JointConnection(local_patches, angle, articulation, NewConnection::ConnectionType::BALLJOINT);
						}else if (tmplProto.connection(j).connectionmode().jointconnection().printtype().has_revolute()){
							conn = new JointConnection(local_patches, angle, articulation, NewConnection::ConnectionType::HINGE);
						}else if (tmplProto.connection(j).connectionmode().jointconnection().printtype().has_prismatic()){
							conn = new JointConnection(local_patches, angle, articulation, NewConnection::ConnectionType::PRISMATIC);
						}else if (tmplProto.connection(j).connectionmode().jointconnection().printtype().has_none()){
							conn = new JointConnection(local_patches, angle, articulation, NewConnection::ConnectionType::NONE);
						}else {
							LOG(WARNING) << "unknown articulation type; using default NONE";
							conn = new JointConnection(local_patches, angle, articulation, NewConnection::ConnectionType::NONE);
						}
						builder->asGroup()->addConnection((NewConnection*)conn);
					}

				}
				else {
					builder->asGroup()->addConnection(new NewConnection(local_patches, 90, NewConnection::ConnectionType::FOLD));
				}
			}

			Template* result = builder->build();
			if (tmplProto.has_independent()) {
				result->isIndependent = tmplProto.independent();
			}


			if(tmplProto.has_semantics()){
				Semantics::PartType partType= static_cast<Semantics::PartType>(static_cast<int>(tmplProto.semantics().parttype())-1);
				Semantics::PrintMethod printMethod= static_cast<Semantics::PrintMethod>(static_cast<int>(tmplProto.semantics().printmethod())-1);
				result->setPartType(partType);
				result->setPrintMethod(printMethod);
			}
			if(tmplProto.has_symmetrychoices()){
				result->symmetryChoices.symm_ground = tmplProto.symmetrychoices().symm_ground();
				result->symmetryChoices.symm_legL = tmplProto.symmetrychoices().symm_legl();
				result->symmetryChoices.symm_legW = tmplProto.symmetrychoices().symm_legw();
				result->symmetryChoices.symm_spacing = tmplProto.symmetrychoices().symm_spacing();
			}

			if (tmplId == helper.rootTemplate) {
				root = result;
			}

		}
		if (root == nullptr){
			LOG(FATAL) << "error: the tempalte converted from proto is null";
		}
		return root;

	}
}