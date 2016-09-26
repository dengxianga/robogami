#include "template.h"
#include "templateElement.h"
#include "NewPatch.h"
#include "element.h"
#include "constraints.h"
#include "geometry.h"
#include "CenterOfMass.h"
#include "element_symbolic.h"
#include "element_motion.h"
#include "articulation.h"
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <sstream>

#define DEBUG true

namespace FabByExample{
class NewPatchLine2D3D;
}

using namespace FabByExample;
using namespace std;



Template::Template(int id): symmetryChoices() {
	parent = nullptr;
	tempID = id;
	tempGeo = NULL; 
	numParams = 0;
	center.setZero();
	pauseRecalculation = false;
	isIndependent = false;
}

void Template::addParameter(std::string newName, double q0)
{
	parameterNames.push_back(newName);
	numParams = parameterNames.size();
	Eigen::VectorXd new_q (q.size() +1);
	new_q.head(q.size()) = q;
	new_q(q.size()) = q0;
	q = new_q; 
	
	Eigen::VectorXd new_Origq (origQ.size() +1);
	new_Origq.head(origQ.size()) = origQ;
	new_Origq(origQ.size()) = q0;
	origQ = new_Origq;
}


Template::~Template(){
	vector<Constraint*>::iterator it;
	for (it = constraints.begin(); it != constraints.end(); it++){
		delete(*it);
	}
	constraints.clear();
	// Delete children
	vector<Template*>::iterator itc;
	for (itc = children.begin(); itc != children.end(); itc++){
		delete(*itc);	
	}
	children.clear();
}

void Template::clear(){
	vector<Constraint*>::iterator it;
	for (it = constraints.begin(); it != constraints.end(); it++){
		delete(*it);
	}
	constraints.clear();
	children.clear();

}

Geometry * Template::getGeometry(bool withnormals){
	if (tempGeo != NULL)
		delete tempGeo;
	tempGeo = new Geometry();
	vector<Geometry*>   geometries;
	getGeometryList(geometries);
	for (unsigned int i = 0; i < geometries.size(); i++){
		tempGeo->add(geometries[i], withnormals);
	}
	return tempGeo;
}

void Template::addChild(Template * child){
	children.push_back(child);
	child->parent = this; 
}

void Template::getConstraints(vector<Constraint*>& constraints, TreeScope scope) {
	for each (auto tmpl in getTemplatesByScope(scope)) {
		for each (auto constraint in tmpl->constraints) {
			constraints.push_back(constraint);
		}
	}
}

void Template::getAssociatedConstraints(vector<Constraint*>& constraints, TreeScope scope) {
	for each (auto tmpl in getTemplatesByScope(scope)) {
		for each (auto constraint in tmpl->constraints) {
			if (constraint->isRelated(this)) {
				constraints.push_back(constraint);
			}
		}
	}
}

void Template::resetInitialParamValues() {
	if (parent != nullptr) {
		LOG(WARNING) << "resetInitialParamValues called on non-root!";
	}
	updateFullQ(getOrigFullQ());
}

BBox3S Template::evalLocalBoundingBox() {
	vector<Element*> elements;
	getAllElementList(elements);

	vector<BBox3S> bboxes;
	for (int i = 0; i < elements.size(); i++) {
		if(dynamic_cast<Element_Symbolic*>(elements[i]) != nullptr){
			bboxes.push_back(dynamic_cast<Element_Symbolic*>(elements[i])->getMesh3S()->evalLocalBoundingBox(SymbolicAssignment::USE_CURRENT_VALUES));
		}
	}

	return BBox3S::unionOf(bboxes, SymbolicAssignment::USE_CURRENT_VALUES);
}

Eigen::Vector3d Template::getCenter(){
	return center;
}

Template * Template::getLowestCommonAncestor(Template* temp2){
	// choose where to place in the tree
	Template* parent = this;
	bool fathersEveryone = parent->isAncestorOf(temp2);
	while(!fathersEveryone){
		parent = parent->getParent();
		if(parent == NULL){
			LOG(ERROR) << "error finging parent that includes all ";
			return NULL;
		}
		fathersEveryone = parent->isAncestorOf(temp2);
	}
	
	return parent;

}

void Template::getTemplatesByScope(vector<Template*>& elements, TreeScope scope) {
	switch (scope) {
	case TreeScope::SELF:
		elements.push_back(this);
		return;
	case TreeScope::ANCESTORS:
		elements.push_back(this);
		// NOTE: Intentional fall-through
	case TreeScope::STRICT_ANCESTORS:
		if (parent != nullptr) {
			parent->getTemplatesByScope(elements, TreeScope::ANCESTORS);
		}
		return;
	case TreeScope::DESCENDANTS:
		elements.push_back(this);
		// NOTE: Intentional fall-through
	case TreeScope::STRICT_DESCENDANTS:
		for each (Template* child in children) {
			child->getTemplatesByScope(elements, TreeScope::DESCENDANTS);
		}
		return;
	}
	LOG(ERROR) << "Invalid tree scope: " << scope;
}

Template* Template::findDescendantByID(int id) {
	queue<Template*> myQueue;
	myQueue.push(this);
	while (!myQueue.empty()){
		Template* ele = myQueue.front();
		myQueue.pop();
		if (ele->getID() == id){
			return ele;
		}
		vector<Template*> children = ele->getChildren();
		vector<Template*>::iterator it;
		for (it = children.begin(); it != children.end(); it++){
			myQueue.push((*it));
		}
	}
	VLOG(5) << "Did not find the descendant template with ID: " << id;
	return nullptr;
}

Template * Template::getRoot(){
	Template* p = this;
	while(p->getParent() != NULL){
		p = p->getParent();
	}
	return p;
}

void Template::addConnection(NewConnection * conn) {
	this->connections.push_back( conn );
	for (int i = 0; i < conn->getPatches().size(); i++){
		conn->getPatches()[i]->parentConnection = conn;
	}
}

void Template::removeConnection(NewConnection* conn) {
	auto it = find(connections.begin(), connections.end(), conn);
	if (it != connections.end()) {
		connections.erase(it);
		for (int i = 0; i < conn->getPatches().size(); i++){
			conn->getPatches()[i]->parentConnection = nullptr;
		}
	}
}

void Template::getConnections(vector<NewConnection *> &connections, TreeScope scope){
	for each (auto tmpl in getTemplatesByScope(scope)) {
		for each (auto conn in tmpl->connections) {
			connections.push_back(conn);
		}
	}
}

CenterOfMass Template::computeCenterOfMass(){
	CenterOfMass centerOfMass;
	vector<Element*>  elements;
	getAllElementList(elements);
	TriMesh::BBox bbox;
	for (int i = 0; i < elements.size(); i++)
	{
		//cout << "doing element " << i << endl;
		if (dynamic_cast<Element_Symbolic*>(elements[i]) == nullptr) {
			LOG(ERROR) << "Adding center of mass to non Element_Symbolic is not implemented.";
		}
		else {
			dynamic_cast<Element_Symbolic*>(elements[i])->addCenterOfMass(centerOfMass);
		}
		//cout << "............. updated center = " << centerOfMass.center.transpose() << endl;

	}

	return centerOfMass;
}


void Template::getContactPoints(vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>  & contpoints){
	vector<Geometry*>   geometries;
	vector<point> points;
	getGeometryList(geometries);
	for (unsigned int i = 0; i < geometries.size(); i++){
		geometries[i]->addAllPoints(points);
	}
	double minP = numeric_limits<double>::max();
	for(int i = 0; i < points.size(); i++){
		if (points[i][1] < minP)
			minP = points[i][1];
	}
	double eps = 0.00001;
	for(int i = 0; i < points.size(); i++){
		if (points[i][1] < (minP + eps)){
			Eigen::Vector2d cPoint;
			cPoint << points[i][0], points[i][2];
			contpoints.push_back(cPoint);
		}
	}
	//cout << "found " << contpoints.size() << " contact points" << endl;

	geometries.clear();
}


void Template::rotate(Eigen::Vector3d const& center, Eigen::Quaterniond const& rotation){
	vector<Element*> elements;
	this->getAllElementList(elements);
	for each(Element* e in elements){
		Element_Symbolic* es = dynamic_cast<Element_Symbolic*>(e);
		if (es != nullptr) {
			Mesh3S* mesh = es->getMesh3S();
			vector<Point3S>& points = mesh->vertices;
			for (int i = 0; i < points.size(); i++){
				points[i] = points[i].translate(-center);
				points[i] = points[i].times(rotation.toRotationMatrix());
				points[i] = points[i].translate(center);
			}
			//es->computeTriangularization(); 
			es->computeNewGeo();
			es->rotateCenterOfMass( center,  rotation);
		}
	}
	this->center = center + rotation.toRotationMatrix() * (this->center - center);
	vector<NewConnection *> connections = getConnections(TreeScope::DESCENDANTS);
	for each(NewConnection* c in connections){
		Articulation * a = c->getArticulation();
		if(a != nullptr){
			a->rotate(center, rotation);
			a->updateParameters(SymbolicAssignment::USE_CURRENT_VALUES);
			a->updateTransform();
		}
	}

	vector<NewPatch *> patches = this->getAllPatches();
	for each(NewPatch* p in patches){
			p->rotate(center, rotation);
		
	}
	//for all contact points
	for each ( auto c in getAllContactPoints()){
		c->roate(center,rotation);
	}

}


vector<ContactPoint*> Template::getAllContactPoints(){
	std::vector <ContactPoint*> contactPoints;
	for each (auto tmpl in getTemplatesByScope(TreeScope::DESCENDANTS)) {
		for each ( auto c in tmpl->contactPoints){
			contactPoints.push_back(c);
		}
	}
	return contactPoints;
}


vector<NewConnection*> Template::getAllConnections(){
	std::vector <NewConnection*> allconnections;
	for each (auto tmpl in getTemplatesByScope(TreeScope::DESCENDANTS)) {
		for each ( auto c in tmpl->connections){
			allconnections.push_back(c);
		}
	}
	return allconnections;
}


vector<ContactPoint*> Template::getOneContactPointsForEachPart(){
	std::vector <ContactPoint*> contactPoints;
	for each (auto tmpl in getTemplatesByScope(TreeScope::DESCENDANTS)) {
		if(tmpl->contactPoints.size() >0){		
			if(tmpl->contactPoints.size() ==1){
				contactPoints.push_back(tmpl->contactPoints[0]);
			}else{
				int minCon = 0;
				double minConPos = std::numeric_limits<double>::max();
				for(int i = 0; i < tmpl->contactPoints.size(); i++){
					double this_minConPos = tmpl->contactPoints[i]->cPoint.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES).y();
					if(this_minConPos <minConPos){
						minConPos = this_minConPos;
						minCon = i;
					}
				}
				contactPoints.push_back(tmpl->contactPoints[minCon]);
			}
		}
	}
	return contactPoints;
}


void Template::translate(Eigen::Vector3d const& trans){
	vector<Element*> elements;
	this->getAllElementList(elements);
	for each(Element* e in elements){
		Element_Symbolic* es = dynamic_cast<Element_Symbolic*>(e);
		if (es != nullptr) {
			Mesh3S* mesh = es->getMesh3S();
			vector<Point3S>& points = mesh->vertices;
			for (int i = 0; i < points.size(); i++){
				points[i].x = points[i].x + trans[0];
				points[i].y = points[i].y + trans[1];
				points[i].z = points[i].z + trans[2];
				//points[i].z.print();
			}
			//es->computeTriangularization(); 
			es->computeNewGeo();
			es->translateCenterOfMass( trans);
		}
	}

	this->center += trans;
	//also translate the connections
	vector<NewConnection *> connections = getConnections(TreeScope::DESCENDANTS);
	for each(NewConnection* c in connections){
		Articulation * a = c->getArticulation();
		if(a != nullptr){
			a->translate(trans);
			a->updateParameters(SymbolicAssignment::USE_CURRENT_VALUES);
			a->updateTransform();
		}
	}

	vector<NewPatch *> patches = this->getAllPatches();
	for each(NewPatch* p in patches){
			p->translate(trans);
		
	}

	for each ( auto c in getAllContactPoints()){
		c->translate(trans);
	}

}

void Template::translate2d(Eigen::Vector2d const& trans){
	vector<Element*> elements;
	this->getAllElementList(elements);
	for each(Element* e in elements){
		Element_Symbolic* es = dynamic_cast<Element_Symbolic*>(e);
		if (es == nullptr) {
			LOG(ERROR) << "Translating a non Element_Symbolic is not implemented.";
			continue;
		}
		Drawing2S* drawing = es->getDrawing2S();
		vector<Point2S>& points = drawing->vertices;
		for (int i = 0; i < points.size(); i++){
			points[i].x = points[i].x + trans[0];
			points[i].y = points[i].y + trans[1];
		}
		es->computeNewGeo();
	}
}

void Template::getAllPatches(vector<NewPatch*>& patches) {

	for each (auto tmpl in getTemplatesByScope((TreeScope::DESCENDANTS))) {
		for each (auto patch in tmpl->getPatches()) {
				patches.push_back(patch);
		}
	}
}


void Template::getAllUnusedPatches(vector<NewPatch*>& patches) {


	vector<NewConnection*>  connections;
	vector<NewPatch*> allpatches;

	getConnections(connections, TreeScope::DESCENDANTS);
	getAllPatches(allpatches);

	for each (auto patch in allpatches) {
		double isUsed = false;
		if (patch->getType() != NewPatch::PatchType::SERVO_LINE_PATCH){
			for each (auto connection in connections){
				if (connection->hasPatch(patch)){
					isUsed = true;
				}
			}
		}
		if(!isUsed){					
			patches.push_back(patch);
		}
	}
}


TriMesh::BBox Template::evalBoundingBox(){
	std::vector<Element*>  elements = getAllElementList();
	TriMesh::BBox bbox;
	for (int i = 0; i < elements.size(); i++)
	{
		if(dynamic_cast<Element_Motion*>(elements[i]) == nullptr){
			bbox = bbox + elements[i]->evalBoundingBox();
		}
	}
	return bbox;
}

drawing::Drawing* Template::getAllDrawing() {
	// TODO(robin): think about memory leak
	drawing::Drawing* pcomposed = new drawing::Drawing();
	drawing::Drawing& composed = *pcomposed;
	auto addDrawing = [&composed](const drawing::Drawing& sub)
	{
		int pointOffset = composed.points.size();
		int edgeOffset = composed.edges.size();
		int faceOffset = composed.faces.size();

		for (int i = 0; i < sub.points.size(); i++) {
			composed.points.push_back(sub.points[i]);
		}
		for (int i = 0; i < sub.edges.size(); i++) {
			auto edge = sub.edges[i];
			auto newEdge = edge;
			newEdge.id += edgeOffset;
			newEdge.vertice1_id += pointOffset;
			newEdge.vertice2_id += pointOffset;
			composed.edges.push_back(newEdge);
		}
		for (int j = 0; j < sub.faces.size(); j++) {
			auto face = sub.faces[j];
			auto newFace = face;
			face.id += faceOffset;
			for (int i = 0; i < face.edges.size(); i++) {
				face.edges[i] = composed.edges[face.edges[i].id + edgeOffset];
			}
			composed.faces.push_back(newFace);
		}
	};
	for each (auto ele in getAllElementList()) {
		auto symb = dynamic_cast<Element_Symbolic*>(ele);
		if (symb != nullptr) {
			addDrawing(*(symb->evalDrawing()));
		}
	}
	return pcomposed;
}

void Template::recomputeCenter() {
	point bboxcenter = evalBoundingBox().center();
	for (int ax = 0; ax< 3; ax++){
		center[ax] = bboxcenter[ax];
	}
}

int Template::numSymbols() const
{
	return numParams;
}

string Template::describeSymbol(int i) const
{
	ostringstream ss;
	ss << tempID << ":";
	ss << parameterNames[i];
	return ss.str();
}

double Template::getCurrentValue(int i) const
{
	return q[i];
}


void Template::setCurrentValue(int i, double _val)
{
	q[i] = _val;
}


std::string Template::getSymbolName(int i) const
{
	return parameterNames[i];
}

void TemplateBuilder::defineParameters(const vector<string>& parameterNames, const Eigen::VectorXd& initialValues)
{
	auto tmpl = getResultingPointer();
	tmpl->numParams = parameterNames.size();
	tmpl->parameterNames = parameterNames;
	tmpl->q = initialValues;
	tmpl->origQ = initialValues;
}


void Template::getSymbols(vector<Symbol>& symbols, TreeScope scope) {
	for each (auto tmpl in getTemplatesByScope(scope)) {
		for (int i = 0; i < tmpl->numSymbols();i++) {
			symbols.push_back(tmpl->getSymbol(i));
		}
	}
}

vector<LinearExpr> inferSubsetConstraints(const vector<LinearExpr>& constraints, const unordered_set<Symbol>& subset) {
	unordered_set<Symbol> allSymbols = subset;
	for each (const auto& le in constraints) {
		for each (const auto& entry in le.getCoeffs()) {
			allSymbols.insert(entry.first);
		}
	}
	int nextPos = allSymbols.size() - 1;
	unordered_map<Symbol, int> symbolPositions;
	Symbol* positionToSymbol = new Symbol[allSymbols.size()];
	for each (const auto& symbol in subset) {
		auto pos = nextPos--;
		symbolPositions[symbol] = pos;
		positionToSymbol[pos] = symbol;
	}
	for each (const auto& symbol in allSymbols) {
		if (symbolPositions.find(symbol) == symbolPositions.end()) {
			auto pos = nextPos--;
			symbolPositions[symbol] = pos;
			positionToSymbol[pos] = symbol;
		}
	}
	Eigen::MatrixXd A(constraints.size(), allSymbols.size());
	A.setZero();
	Eigen::VectorXd b(constraints.size());
	b.setZero();
	for (int i = 0; i < constraints.size();i++) {
		for each (const auto& entry in constraints[i].getCoeffs()) {
			A(i, symbolPositions[entry.first]) = entry.second;
		}
		b[i] = -constraints[i].getConstant();
	}


	// gaussian elimination.... any better way??
	auto swapRows = [&](int i, int j) {
		A.row(i).swap(A.row(j));
		auto c = b[i];
		b[i] = b[j];
		b[j] = c;
	};

	auto eliminate = [&](int row, int col) {
		for (int i = row + 1; i < A.rows(); i++) {
			if (abs(A(i, col)) > 1e-4) {
				auto ratio = A(i, col) / A(row, col);
				A.row(i) -= A.row(row) * ratio;
				b[i] -= b[row] * ratio;
			}
		}
	};

	auto findPivot = [&](int row, int col) -> bool {
		if (abs(A(row, col)) > 1e-4) return true;
		for (int i = row + 1; i < A.rows(); i++) {
			if (abs(A(i, col)) > 1e-4) {
				swapRows(row, i);
				return true;
			}
		}
		return false;
	};

	int i = 0;
	for (int j = 0; j < A.cols(); j++) {
		if (i == A.rows()) break;
		if (findPivot(i, j)) {
			eliminate(i, j);
			i++;
		}
	}

	vector<LinearExpr> inferred;
	for (int i = 0; i < A.rows(); i++) {
		bool goodRow = true;
		for (int j = 0; j < allSymbols.size() - subset.size(); j++) {
			if (abs(A(i, j)) > 1e-4) {
				goodRow = false;
			}
		}
		if (goodRow) {
			CoeffsMap coeffs;
			for (int j = allSymbols.size() - subset.size(); j < A.cols(); j++) {
				if (abs(A(i, j)) > 1e-4) {
					coeffs[positionToSymbol[j]] = A(i, j);
				}
			}
			inferred.push_back(LinearExpr(coeffs, -b[i]));
		}
	}
	return inferred;
}

namespace {
	bool equals(double a, double b) {
		return abs(a - b) < 1e-6;
	}

	Symbol findSubstituteInSubtemplate(Symbol toSub, vector<Constraint*> const& constraints, Template* sub) {
		unordered_map<Symbol, unordered_set<Symbol>> adj;
		for each (auto cnst in constraints) {
			if (cnst->getRelation() == Constraint::ConstraintRelation::INEQ) {
				continue;
			}
			auto le = dynamic_cast<Constraint*>(cnst)->getLinearExpr();
			auto const& coeffs = le.getCoeffs();
			if (coeffs.size() == 2 && equals(0, le.getConstant())) {
				auto it = coeffs.begin();
				Symbol sA = it->first;
				double cA = it->second;
				++it;
				Symbol sB = it->first;
				double cB = it->second;
				if (equals(cA, -cB)) {
					adj[sA].insert(sB);
					adj[sB].insert(sA);
				}
			}
		}

		// do a bfs
		queue<Symbol> q;
		q.push(toSub);
		unordered_set<Symbol> visited;
		while (!q.empty()) {
			Symbol s = q.front();
			q.pop();
			visited.insert(s);
			if (const_cast<Template*>(dynamic_cast<const Template*>(s.owner))->isDescendentOf(sub)) {
				return s;
			}
			for each (Symbol other in adj[s]) {
				if (visited.find(other) == visited.end()) {
					q.push(other);
				}
			}
		}
		auto error = (string)(concat() << "Cannot find substitute of symbol " << toSub.owner->describeSymbol(toSub.id) <<
			" in subtemplate " << sub->getID() << "!");
		LOG(ERROR) << error;
		throw error;
	}

	LinearExpr findSubstituteInSubtemplate(LinearExpr const& toSub, vector<Constraint*> const& constraints, Template* sub) {
		CoeffsMap coeffs = toSub.getCoeffs();
		CoeffsMap result;
		for each (auto const& pair in coeffs) {
			Symbol s = findSubstituteInSubtemplate(pair.first, constraints, sub);
			result[s] = pair.second;
		}
		return LinearExpr(result, toSub.getConstant());
	}

	Point3S findSubstituteInSubtemplate(Point3S const& toSub, vector<Constraint*> const& constraints, Template* sub) {
		Point3S p;
		p.x = findSubstituteInSubtemplate(toSub.x, constraints, sub);
		p.y = findSubstituteInSubtemplate(toSub.y, constraints, sub);
		p.z = findSubstituteInSubtemplate(toSub.z, constraints, sub);
		return p;
	}
}

void replacePatchesByPlaceholders(Template* tmpl) {
	vector<NewConnection*> connections = tmpl->getConnections(TreeScope::DESCENDANTS);
	for each (auto conn in connections) {
		Eigen::Vector3d masterDirection;
		for (int i = 0; i < conn->patches.size(); i++) {
			auto patch = conn->patches[i];
			if (patch->getElement()->isDescendentOf(tmpl)) {
				masterDirection = dynamic_cast<NewPatchLine2D3D*>(patch)->getDirection();
				break;
			}
		}
		for (int i = 0; i < conn->patches.size(); i++) {
			auto patch = conn->patches[i];
			if (!patch->getElement()->isDescendentOf(tmpl)) {
				Eigen::Vector3d thisDirection = dynamic_cast<NewPatchLine2D3D*>(patch)->getDirection();
				auto dot = masterDirection.dot(thisDirection);
				conn->patches[i] = new PlaceHolderPatch(dot < 0);
			}
		}
	}
}

// Deduce some obvious constants (such as x = 0) from constraints.
CoeffsMap deduceConstants(vector<Constraint*> const& constraints) {
	CoeffsMap sa;
	for each (Constraint* c in constraints) {
		if (c->getRelation() == Constraint::ConstraintRelation::INEQ) {
			continue;
		}
		LinearExpr ex = dynamic_cast<Constraint*>(c)->getLinearExpr();
		if (ex.getCoeffs().size() == 1) {
			sa[ex.getCoeffs().begin()->first] = -ex.getConstant() / ex.getCoeffs().begin()->second;
		}
	}
	return sa;
}

void substituteSymbolsInArticulations(Template* child, vector<Constraint*> const& constraints) {
	CoeffsMap deduced = deduceConstants(constraints);
	ConcreteSymbolicAssignment deducedEnv;
	deducedEnv.map = deduced;
	vector<NewConnection*> connections = child->getConnections(TreeScope::DESCENDANTS);
	for each (NewConnection* nc in connections) {
		if (nc->getArticulation() != nullptr) {
			auto art = nc->getArticulation();
			art->symbCenter = findSubstituteInSubtemplate(art->symbCenter.evalPartial(deducedEnv), constraints, child);
			for each (auto xform in art->transformations) {
				xform->symbAxis = findSubstituteInSubtemplate(xform->symbAxis.evalPartial(deducedEnv), constraints, child);
			}
		}
	}
}

void Template::segregate() {
	Template* root = getRoot();
	if (this == root) {
		return;
	}
	vector<Constraint*> constraints = root->getConstraints(TreeScope::DESCENDANTS);
	vector<NewConnection*> connections = root->getConnections(TreeScope::DESCENDANTS);
	vector<NewConnection*> connectionsToKeep;
	for each (NewConnection* nc in connections) {
		auto patches = nc->getPatches();
		bool relevant = false;
		for each (auto patch in patches) {
			if (patch->getElement()->isDescendentOf(this)) {
				relevant = true;
				break;
			}
		}
		if (relevant) {
			connectionsToKeep.push_back(nc);
		}
	}


#if 0
	
	// get all constraints of all ancestors, not including this child itself.
	vector<Constraint*> constraintsUp = getConstraints(TreeScope::ANCESTORS);

	// get all the symols defined by the child as well as all its descendent templates.
	vector<Symbol> allSymbolsBelow = getSymbols(TreeScope::DESCENDANTS);

	// infer constraints for these symbols
	vector<Constraint*> inferredConstraints;

	vector<LinearExpr> originalExprs;
	for (auto it = constraintsUp.begin(); it != constraintsUp.end(); it++){
		ConstraintLinearExpr* linearConstraint = dynamic_cast<ConstraintLinearExpr*> (*it);
		originalExprs.push_back(linearConstraint->getLinearExpr());
	}
	auto inferredExprs = inferSubsetConstraints(originalExprs, unordered_set<Symbol>(allSymbolsBelow.begin(), allSymbolsBelow.end()));

	// add the resulting constraints to the child we are going to segregate.
	for (int i = 0; i < inferredExprs.size(); i++){
		inferredExprs[i].print();
		printf("\n");
		addConstraint(new ConstraintLinearExpr(inferredExprs[i]));
	}
#endif

	vector<NewConnection*> existingConnections = getConnections(TreeScope::DESCENDANTS);
	unordered_set<NewConnection*> existingConnectionsSet;
	for each (auto nc in existingConnections) {
		existingConnectionsSet.insert(nc);
	}
	for each (auto nc in connectionsToKeep) {
		if (existingConnectionsSet.find(nc) == existingConnectionsSet.end()) {
			addConnection(nc);
		}
	}

	replacePatchesByPlaceholders(this);
	substituteSymbolsInArticulations(this, constraints);

	parent = nullptr;
	recomputeCenter();
}

void Template::remove() {
	if (this == getRoot()) {
		LOG(ERROR) << "Cannot remove root template by calling Template::remove";
		return;
	}

	Template* root = getRoot();

	// Remove this from parent
	parent->children.erase(find(parent->children.begin(), parent->children.end(), this));
	parent = nullptr;

	vector<Template*> remaining = root->getTemplatesByScope(TreeScope::DESCENDANTS);
	for each (Template* rem in remaining) {
		auto connections = rem->connections;
		for each (NewConnection* c in connections) {
			auto templates = c->getTemplates();
			for each (auto temp in templates) {
				if (temp->isDescendentOf(this)) {
					rem->removeConnection(c);
					LOG(INFO) << "Removed connection from " << rem->getName();
					for each( auto p in c->patches){
						ServoLinePatch* linePatch = dynamic_cast<ServoLinePatch*>(p);
						if(linePatch!= nullptr){
							int associatedSpacing = -1;
							for(int s = 0; s < linePatch->spacings.size(); s++){
								bool isRelated = false;
								for each (auto t in linePatch->spacings[s].associatedTemplates){
									if(t->isDescendentOf(this)){
										isRelated = true;
									}
								}
								if(isRelated){
									if(associatedSpacing >=0){
										VLOG(3) << "found more than one spacing" ; system("pause");
									}
									associatedSpacing = s;
								}
							}
							if(associatedSpacing >= 0){
								linePatch->spacings.erase(linePatch->spacings.begin() + associatedSpacing);
							}else{
								VLOG(3) << "could not fine line spacing " ; system("pause");
							}
						}
					}
				}
			}
		}

		auto constraints = rem->constraints;
		for each (Constraint* c in constraints) {
			const auto& le = c->getLinearExpr();
			for (auto it = le.getCoeffs().begin(); it != le.getCoeffs().end(); ++it)
			{
				if (const_cast<Template*>(static_cast<const Template*>(it->first.owner))->isDescendentOf(this)) {
					rem->removeConstraint(c);
					LOG(INFO) << "Removed constraint from " << rem->getName();
					break;
				}
			}
		}
	}
}

bool Template::isDescendentOf(Template* other) {
	Template* t = this;
	while (t != nullptr) {
		if (t == other) return true;
		t = t->getParent();
	}
	return false;
}


void Template::moveToGround(){
	double lowestZ = getLowestZ();
	Eigen::Vector3d up;
	up << 0, -lowestZ, 0;
	this->translate(up);
	if (DEBUG) {
		VLOG(3) << "Move template mesh up in z by " << -lowestZ << " to the ground";
	}
}



Point3S Template::getLowestZPoint(){
	double lowestZvalue = 10000;
	Point3S lowestZpoint;
	auto elements = this->getAllElementList();
	for each (auto e in elements){
			Element_Symbolic* es = dynamic_cast<Element_Symbolic*> (e);
			if (es != nullptr) {
				Mesh3S* mesh3S = es->getMesh3S();
				//FabByExample::mesh::Mesh* mesh = es->getMesh3S()->evalMesh(this->getCurrentEnv());;
				for each(auto const& p3S in mesh3S->vertices){
					Eigen::Vector3d p = p3S.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
					if (p[1] < lowestZvalue){
						lowestZvalue = p[1];
						lowestZpoint = p3S;
					}
				}
			}
	}
	return lowestZpoint;
}


/*Point3S Template::getLowestZPoint(){
	queue<Template*> myQueue;
	double lowestZvalue = 10000;
	Point3S lowestZpoint;
	myQueue.push(this);
	while (!myQueue.empty()){
		auto toVisit = myQueue.front();
		myQueue.pop();
		if (toVisit->isLeaf()){
			TemplateElement* te = dynamic_cast<TemplateElement*> (toVisit);
			Element* e = te->getElement();
			Element_Symbolic* es = dynamic_cast<Element_Symbolic*> (e);
			if (es == nullptr) {
				LOG(WARNING) << "Not a symbolic element. Going to return a constant instead";
				double minZ = e->getGeo()->getMesh()->bbox.min[1];
				Point3S result;
				result.x = LinearExpr(0.0);
				result.y = LinearExpr(minZ);
				result.z = LinearExpr(0.0);
				return result;
			}
			Mesh3S* mesh3S = es->getMesh3S();
			//FabByExample::mesh::Mesh* mesh = es->getMesh3S()->evalMesh(this->getCurrentEnv());;
			for each(auto const& p3S in mesh3S->vertices){
				Eigen::Vector3d p = p3S.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES);
				if (p[1] < lowestZvalue){
					lowestZvalue = p[1];
					lowestZpoint = p3S;
				}
			}
		}
		else{
			for each (auto child in toVisit->getChildren()){
				myQueue.push(child);
			}
		}
	}

	return lowestZpoint;
} */

double Template::getLowestZ(){
	Point3S lowestZpoint = this->getLowestZPoint();
	return lowestZpoint.evalVector3d(SymbolicAssignment::USE_CURRENT_VALUES)[1];
}

void Template::removeConstraint(Constraint* c) {
	auto it = find(constraints.begin(), constraints.end(), c);
	if (it != constraints.end()) {
		constraints.erase(it);
	}
}

void Template::removeAllConstraintsOfType(Constraint::ConstraintType type){
	

	std::vector<Constraint*> removingConstraints;
	for each (auto c in constraints){
		if(c->getType() == type){
			removingConstraints.push_back(c);
		}
	}

	for each(auto c in removingConstraints){
		removeConstraint(c);
	}

	removingConstraints.clear();

		
}


void Template::forceRedefineParams(const vector<string>& paramNames, const Eigen::VectorXd& q) {
	int newNumParams = paramNames.size();
	for each (auto cst in this->getRoot()->getConstraints(TreeScope::DESCENDANTS)) {
		for each (const auto& pair in cst->getLinearExpr().getCoeffs()) {
			if (pair.first.owner == this && pair.first.id >= newNumParams) {
				LOG(FATAL) << "Constraint involves more parameters than would be newly defined.";
			}
		}
	}
	if (newNumParams != numParams) {
		LOG(WARNING) << "Redefining parameters from " << numParams << " to " << newNumParams << " parameters";
	}
	numParams = paramNames.size();
	this->parameterNames = paramNames;
	this->origQ = this->q = q;
	this->updateLocalQ(q);
	this->getRoot()->updateFullQ(this->getRoot()->getFullQ());
}

void Template::addPatch(NewPatch* p){
	
		this->patches.push_back( p );
}
