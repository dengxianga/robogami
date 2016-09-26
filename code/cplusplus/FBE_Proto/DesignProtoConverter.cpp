#include "DesignProtoConverter.h"

#include "design.h"
#include "template.h"
//#include "ConstraintsEval.h"
//#include "workingTemplate.h"
//#include "tempDatabse.h"
#include <vector>
//#include "hierarchyGraphs.h"
//#include "mainTemplate.h"
//#include "templateFactory.h"
//#include "TriMesh.h"
#include "component.h"
//#include "fileIO.h"
//#include "computeConnGraph.h"
#include "graph.h"
#include "tree.h"
#include "../FBE_Design/geometry.h"
//#include "google/protobuf/text_format.h"

using namespace std;

namespace FabByExample {
	DesignProtoConverter::DesignProtoConverter()
	{
	}


	DesignProtoConverter::~DesignProtoConverter()
	{
	}

	namespace{
		// assume v is 3x4 matrix
		void makeTransformation(proto::Transformation* t, const vector<double>& v){
			proto::Matrix* identity = t->mutable_rotation_matrix();
			proto::Vector* zero = t->mutable_translation_vector();
			for (int i = 0; i < 9; i++){
				identity->add_values(v[i]);
			}

			for (int j = 0; j < 3; j++){
				zero->add_values(v[9 + j]);
			}

		}

		void makeIdentity(proto::Transformation* t, int n){
			proto::Matrix* identity = t->mutable_rotation_matrix();
			proto::Vector* zero = t->mutable_translation_vector();
			for (int i = 0; i < n; i++){
				for (int j = 0; j < n; j++){
					if (i == j){
						identity->add_values(1);
					}
					else{
						identity->add_values(0);
					}
				}
				zero->add_values(0);
			}
		}


		void makeMatrix(const proto::Transformation& t, vector<double>* v){
			const proto::Matrix& rotMatrix = t.rotation_matrix();
			const proto::Vector& transVector = t.translation_vector();
			for (int i = 0; i < rotMatrix.values_size(); i++){
				v->push_back(rotMatrix.values(i));
			}
			for (int j = 0; j < transVector.values_size(); j++){
				v->push_back(transVector.values(j));
			}

		}
		void convertVectorToProto(const vector3f& v3f, proto::Vector* v){
			for (int i = 0; i < 3; i++){
				v->add_values(v3f.vertex[i]);
			}
		}

		vector3f convertToVector3f(const proto::Vector& v){
			vector3f v3f;
			for (int i = 0; i < 3; i++){
				v3f.vertex[i] = v.values(i);
			}
			return v3f;

		}

		void translateComponentIntoNode(proto::Node* node, Component* comp){

			std::list<Edge*> edges = comp->edges;
			node->set_id(comp->getId());
			node->set_name(comp->getName());
			Functionality* func = comp->getFunc();
			if (func != nullptr) {
				proto::Functionality* protoFunc = node->mutable_func();
				proto::Vector* center = protoFunc->mutable_center();
				convertVectorToProto(func->getCenter(), center);
				protoFunc->set_name(func->getName());
				protoFunc->set_axis((proto::Functionality_Axis) (int)func->getAxis());
				protoFunc->set_type((proto::Functionality_Type) (int)func->getType());
			}
			if (edges.size() == 0) {
				proto::LeafNode* childLeafNode = node->MutableExtension(proto::LeafNode::leaf_node);
				childLeafNode->set_geometry(comp->getGeometry()->getSourceFileName());
				if (comp->isConnector()){
					childLeafNode->set_type(proto::LeafNode::Type::LeafNode_Type_CONNECTOR);

				}
			}

			for (auto it = edges.begin(); it != edges.end(); it++){
				Component* p = (*it)->getChild();
				vector<double> matrix = (*it)->getRelativeMatrix();
				proto::Node* child = node->add_children();
				proto::Transformation* transformation = child->mutable_transformation();
				makeTransformation(transformation, matrix);
				translateComponentIntoNode(child, p);
			}
		}

		Component* translateNodeIntoComponent(Tree* tree, const proto::Node& node){

			Component* comp = tree->addComponent(node.id(), node.name(), node.HasExtension(proto::LeafNode::leaf_node));
			/*if (node.HasExtension(proto::LeafNode::leaf_node)) {
			Geometry* geometry = new Geometry(node.GetExtension(proto::LeafNode::leaf_node).geometry());
			//static_cast<ComponentLeaf*>(comp)->
			}*/
			if (node.has_func()) {
				const proto::Functionality& pFunc = node.func();
				Functionality* func0 = new Functionality(convertToVector3f(pFunc.center()), pFunc.name(), pFunc.id());
				comp->addFunctionality(func0);
			}

			int size = node.children_size();
			for (int i = 0; i < size; i++){
				const proto::Node& child = node.children(i);
				Component* childComp = translateNodeIntoComponent(tree, child);
				vector<double> relativeMatrix;
				makeMatrix(child.transformation(), &relativeMatrix);
				comp->addEdge(childComp, relativeMatrix);
			}
			return comp;

		}

	}
	proto::Design* DesignProtoConverter::ConvertToProto(Design* design){
		proto::Design* designProto = new proto::Design();
		designProto->set_name(design->getName());
		proto::Node* rootNode = designProto->mutable_root();
		Component* root = design->getTree()->getRoot();
		proto::Transformation* transformation = rootNode->mutable_transformation();
		makeIdentity(transformation, 3);
		//recursively add children		
		translateComponentIntoNode(rootNode, root);
		return designProto;

	}

	Design* DesignProtoConverter::ConvertToDesign(proto::Design* protoDesign){
		Design * design = new Design(protoDesign->name());
		Tree* tree = design->getTree();
		const proto::Node& loadedRoot = protoDesign->root();
		Component* treeRoot = translateNodeIntoComponent(tree, loadedRoot);
		return design;

	}
}