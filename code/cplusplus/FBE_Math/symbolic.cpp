#include "symbolic.h"
#include "poly2tri\poly2tri.h" // for triangulation
#include <Eigen/StdVector>
#include <unordered_set>
#include <sstream>
#include <cmath>
#include <ctime>

#define I_KNOW_HOW_TO_INCLUDE_FAST_HASHMAP
#define FAST_HASHMAP_IMPL
namespace _CoeffsMapImpl {
	#include "fast_hashmap.h"
}
#undef FAST_HASHMAP_IMPL
#undef I_KNOW_HOW_TO_INCLUDE_FAST_HASHMAP

using namespace std;
static bool approxZero(double x)
{
	return abs(x) < 1e-7;
}
namespace FabByExample {
	static UseCurrentValuesSymbolicAssignment __USE_CURRENT_VALUES;
	const SymbolicAssignment& SymbolicAssignment::USE_CURRENT_VALUES = __USE_CURRENT_VALUES;

	LinearExpr::LinearExpr() : constant(0)
	{
	}

	LinearExpr::LinearExpr(double constant) : constant(constant)
	{
	}

	LinearExpr::LinearExpr(Symbol variable) : constant(0)
	{
		coeffs[variable] = 1;
	}

	LinearExpr::LinearExpr(LinearExpr const& from)
	{
		PROFILE_THIS(__FUNCTION__);
		coeffs = from.coeffs;
		constant = from.constant;
	}

	LinearExpr::LinearExpr(CoeffsMap const& coeffs, double constant) : constant(round(constant * 1000000)/1000000)
	{
		for (auto it = coeffs.begin(); it != coeffs.end(); ++it)
		{
			if (!approxZero(it->second))
			{
				this->coeffs[it->first] = round(it->second * 1000000) / 1000000;
			}
		}
	}

	double& LinearExpr::operator[](Symbol index) {
		if(index == ConstSymbol())
			return constant;
		return coeffs[index];

	}

	LinearExpr& LinearExpr::operator=(LinearExpr const& that)
	{
		PROFILE_THIS(__FUNCTION__);
		coeffs = that.coeffs;
		constant = that.constant;
		return *this;
	}

	bool LinearExpr::operator==(LinearExpr const& that) const {
		return coeffs == that.coeffs && constant == that.constant;
	}

	LinearExpr LinearExpr::operator+(LinearExpr const& that) const
	{
		CoeffsMap coeffs;
		for (auto it = this->coeffs.begin(); it != this->coeffs.end(); ++it)
		{
			coeffs[it->first] += it->second;
		}
		for (auto it = that.coeffs.begin(); it != that.coeffs.end(); ++it)
		{
			coeffs[it->first] += it->second;
		}
		return LinearExpr(coeffs, this->constant + that.constant);
	}

	LinearExpr LinearExpr::operator-(LinearExpr const& that) const
	{
		CoeffsMap coeffs;
		for (auto it = this->coeffs.begin(); it != this->coeffs.end(); ++it)
		{
			coeffs[it->first] += it->second;
		}
		for (auto it = that.coeffs.begin(); it != that.coeffs.end(); ++it)
		{
			coeffs[it->first] -= it->second;
		}
		return LinearExpr(coeffs, this->constant - that.constant);
	}

	LinearExpr LinearExpr::operator+(double toadd) const
	{
		return LinearExpr(this->coeffs, this->constant + toadd);
	}
	
	LinearExpr LinearExpr::operator-(double tosub) const
	{
		return LinearExpr(this->coeffs, this->constant - tosub);
	}

	QuadExpr LinearExpr::operator*(LinearExpr const& that) const
	{
		std::unordered_map<Symbol, LinearExpr> coeffs;

		for (auto it = this->coeffs.begin(); it != this->coeffs.end(); ++it)
		{
			coeffs[it->first] = it->second * that;
		}
		coeffs[ConstSymbol()] = this->constant * that;

		return QuadExpr(coeffs);
	}

	LinearExpr LinearExpr::operator*(double factor) const
	{
		CoeffsMap coeffs;
		for (auto it = this->coeffs.begin(); it != this->coeffs.end(); ++it)
		{
			coeffs[it->first] = it->second * factor;
		}
		return LinearExpr(coeffs, this->constant * factor);
	}

	LinearExpr LinearExpr::operator/(double factor) const
	{
		CoeffsMap coeffs;
		for (auto it = this->coeffs.begin(); it != this->coeffs.end(); ++it)
		{
			coeffs[it->first] = it->second / factor;
		}
		return LinearExpr(coeffs, this->constant / factor);
	}

	CoeffsMap const& LinearExpr::getCoeffs() const {
		return coeffs;
	}

	double LinearExpr::getConstant() const {
		return constant;
	}

	bool LinearExpr::isConstant() const {
		return coeffs.size() == 0;
	}

	std::array<double, 1> LinearExpr::match(Symbol param, bool* result) const {
		std::vector<Symbol> params;
		params.push_back(param);
		std::vector<double> res = match(params, result);
		std::array<double, 1> ret;
		ret[0] = res[0];
		return ret;
	}

	std::array<double, 2> LinearExpr::match(Symbol param1, Symbol param2, bool* result) const {
		std::vector<Symbol> params;
		params.push_back(param1);
		params.push_back(param2);
		std::vector<double> res = match(params, result);
		std::array<double, 2> ret;
		ret[0] = res[0];
		ret[1] = res[1];
		return ret;
	}

	std::array<double, 3> LinearExpr::match(Symbol param1, Symbol param2, Symbol param3, bool* result) const {
		std::vector<Symbol> params;
		params.push_back(param1);
		params.push_back(param2);
		params.push_back(param3);
		std::vector<double> res = match(params, result);
		std::array<double, 3> ret;
		ret[0] = res[0];
		ret[1] = res[1];
		ret[2] = res[2];
		return ret;
	}

	std::array<double, 4> LinearExpr::match(Symbol param1, Symbol param2, Symbol param3, Symbol param4, bool* result) const {
		std::vector<Symbol> params;
		params.push_back(param1);
		params.push_back(param2);
		params.push_back(param3);
		params.push_back(param4);
		std::vector<double> res = match(params, result);
		std::array<double, 4> ret;
		ret[0] = res[0];
		ret[1] = res[1];
		ret[2] = res[2];
		ret[3] = res[3];
		return ret;
	}

	double LinearExpr::eval(SymbolicAssignment const& env) const {
		double result = constant;
		for (auto it = coeffs.begin(); it != coeffs.end(); ++it) {
			result += it->second * env.get(it->first);
		}
		return result;
	}

	LinearExpr LinearExpr::evalPartial(SymbolicAssignment const& env) const {
		double newConst = constant;
		decltype(coeffs) newCoeffs;
		for (auto it = coeffs.begin(); it != coeffs.end(); ++it) {
			if (env.has(it->first)) {
				newConst += env.get(it->first) * it->second;
			}
			else {
				newCoeffs[it->first] = it->second;
			}
		}
		return LinearExpr(newCoeffs, newConst);
	}

	std::vector<double> LinearExpr::match(std::vector<Symbol> const& params, bool* result) const {
		std::vector<double> ret(params.size());
		for (int i = 0; i < params.size(); i++) {
			if (coeffs.has(params[i])) {
				*result = false;
				return ret;
			}
			ret[i] = coeffs.at(params[i]);
		}
		// intentionally not setting result to true, so caller can coalesce calls
		return ret;
	}

	LinearExpr operator-(LinearExpr const& expr)
	{
		CoeffsMap coeffs;
		for (auto it = expr.coeffs.begin(); it != expr.coeffs.end(); ++it)
		{
			coeffs[it->first] = -it->second;
		}
		return LinearExpr(coeffs, -expr.constant);
	}

	LinearExpr operator*(double factor, LinearExpr const& expr)
	{
		return expr * factor;
	}
	
	QuadExpr::QuadExpr()
	{
		this->coeffs[ConstSymbol()] = LinearExpr();
	}

	QuadExpr::QuadExpr(double constant)
	{
		this->coeffs[ConstSymbol()] = LinearExpr(constant);
	}

	QuadExpr::QuadExpr(Symbol variable)
	{
		this->coeffs[ConstSymbol()] = LinearExpr(variable);
	}
	
	QuadExpr::QuadExpr(const LinearExpr& from)
	{
		this->coeffs[ConstSymbol()] = LinearExpr(from);
	}
	
	QuadExpr::QuadExpr(LinearExpr&& from)
	{
		this->coeffs[ConstSymbol()] = LinearExpr(from);
	}

	QuadExpr::QuadExpr(QuadExpr const& from) : coeffs(from.coeffs)
	{
	}

	QuadExpr::QuadExpr(QuadExpr&& from) : coeffs(std::move(from.coeffs))
	{
	}

	QuadExpr::QuadExpr(std::unordered_map<Symbol, LinearExpr> const& coeffs)
	{
		for (auto it = coeffs.begin(); it != coeffs.end(); ++it)
		{
			if (!(it->second.isConstant() && approxZero(it->second.getConstant())))
			{
				this->coeffs[it->first] = it->second;
			}
		}
	}

	LinearExpr& QuadExpr::operator[](Symbol index) {
		return coeffs[index];
	}

	QuadExpr& QuadExpr::operator=(QuadExpr const& that)
	{
		coeffs = that.coeffs;
		return *this;
	}

	QuadExpr& QuadExpr::operator=(QuadExpr&& that)
	{
		coeffs = std::move(that.coeffs);
		return *this;
	}

	QuadExpr QuadExpr::operator+(QuadExpr const& that) const
	{
		std::unordered_map<Symbol, LinearExpr> coeffs;
		for (auto it = this->coeffs.begin(); it != this->coeffs.end(); ++it)
		{
			coeffs[it->first] = coeffs[it->first] + it->second;
		}
		for (auto it = that.coeffs.begin(); it != that.coeffs.end(); ++it)
		{
			coeffs[it->first] = coeffs[it->first] + it->second;
		}
		return QuadExpr(coeffs);
	}

	QuadExpr QuadExpr::operator-(QuadExpr const& that) const
	{
		std::unordered_map<Symbol, LinearExpr> coeffs;
		for (auto it = this->coeffs.begin(); it != this->coeffs.end(); ++it)
		{
			coeffs[it->first] = coeffs[it->first] + it->second;
		}
		for (auto it = that.coeffs.begin(); it != that.coeffs.end(); ++it)
		{
			coeffs[it->first] = coeffs[it->first] - it->second;
		}
		return QuadExpr(coeffs);
	}

	QuadExpr QuadExpr::operator+(const LinearExpr& toadd) const
	{
		std::unordered_map<Symbol, LinearExpr> coeffs(this->coeffs);
		coeffs[ConstSymbol()] = coeffs[ConstSymbol()] + toadd;
		return QuadExpr(coeffs);
	}

	QuadExpr QuadExpr::operator+(double toadd) const
	{
		std::unordered_map<Symbol, LinearExpr> coeffs(this->coeffs);
		coeffs[ConstSymbol()] = coeffs[ConstSymbol()] + toadd;
		return QuadExpr(coeffs);
	}
	
	QuadExpr QuadExpr::operator-(const LinearExpr& tosub) const
	{
		std::unordered_map<Symbol, LinearExpr> coeffs(this->coeffs);
		coeffs[ConstSymbol()] = coeffs[ConstSymbol()] - tosub;
		return QuadExpr(coeffs);
	}
	
	QuadExpr QuadExpr::operator-(double tosub) const
	{
		std::unordered_map<Symbol, LinearExpr> coeffs(this->coeffs);
		coeffs[ConstSymbol()] = coeffs[ConstSymbol()] - tosub;
		return QuadExpr(coeffs);
	}

	QuadExpr QuadExpr::operator*(double factor) const
	{
		std::unordered_map<Symbol, LinearExpr> coeffs;
		for (auto it = this->coeffs.begin(); it != this->coeffs.end(); ++it)
		{
			coeffs[it->first] = it->second * factor;
		}
		return QuadExpr(coeffs);
	}

	QuadExpr QuadExpr::operator/(double factor) const
	{
		std::unordered_map<Symbol, LinearExpr> coeffs;
		for (auto it = this->coeffs.begin(); it != this->coeffs.end(); ++it)
		{
			coeffs[it->first] = it->second / factor;
		}
		return QuadExpr(coeffs);
	}

	std::unordered_map<Symbol, LinearExpr> const& QuadExpr::getCoeffs() const {
		return coeffs;
	}

	double QuadExpr::getConstant() const {
		return coeffs.at(ConstSymbol()).getConstant();
	}
	
	bool QuadExpr::isConstant() const {
		return ((coeffs.size() == 1) && (coeffs.find(ConstSymbol()) != coeffs.end()) && coeffs.at(ConstSymbol()).isConstant());
	}

	QuadExpr QuadExpr::simplify() const {
		std::unordered_map<Symbol, LinearExpr> coeffs;
		LinearExpr newConst = this->coeffs.at(ConstSymbol());

		for (auto it = this->coeffs.begin(); it != this->coeffs.end(); ++it)
		{
			if (it->first == ConstSymbol()) {
				continue;
			}

			LinearExpr tostore = it->second;

			if (tostore.getConstant() != 0) {
				newConst[it->first] += tostore.getConstant();
				tostore = tostore - tostore.getConstant();
			}

			for (auto lowerit = this->coeffs.begin(); lowerit != it; ++lowerit) {
				CoeffsMap lincoeff = tostore.getCoeffs();
				if (lincoeff.has(lowerit->first)) {
					coeffs[lowerit->first] = coeffs[lowerit->first] + lincoeff[lowerit->first] * LinearExpr(it->first);
					tostore = tostore - lincoeff[lowerit->first] * LinearExpr(lowerit->first);
				}
			}

			coeffs[it->first] = tostore;
		}

		coeffs[ConstSymbol()] = newConst;

		return QuadExpr(coeffs);
	}

	double QuadExpr::eval(SymbolicAssignment const& env) const {
		double result = 0;
		for (auto it = coeffs.begin(); it != coeffs.end(); ++it) {
			if (it->first == ConstSymbol())
			{
				result += it->second.eval(env);
			} else {
				result += env.get(it->first) * it->second.eval(env);
			}
		}
		return result;
	}

	QuadExpr QuadExpr::evalPartial(SymbolicAssignment const& env) const {
		decltype(coeffs) newCoeffs;
		LinearExpr newConst;
		for (auto it = coeffs.begin(); it != coeffs.end(); ++it) {
			if (env.has(it->first)) {
				newConst = newConst + it->second.evalPartial(env) * env.get(it->first);
			} else if (it->first == ConstSymbol()) {
				newConst = newConst + it->second.evalPartial(env);
			} else {
				LinearExpr tempRow = it->second.evalPartial(env);
				newConst[it->first] += tempRow.getConstant();
				tempRow = tempRow - tempRow.getConstant();
				newCoeffs[it->first] = tempRow;
			}
		}
		newCoeffs[ConstSymbol()] = newConst;
		return QuadExpr(newCoeffs);
	}

	QuadExpr operator-(QuadExpr const& expr)
	{
		std::unordered_map<Symbol, LinearExpr> coeffs;
		for (auto it = expr.coeffs.begin(); it != expr.coeffs.end(); ++it)
		{
			coeffs[it->first] = -it->second;
		}
		return QuadExpr(coeffs);
	}

	QuadExpr operator*(double factor, QuadExpr const& expr)
	{
		return expr * factor;
	}
	
	void LinearExpr::print() const
	{
		for (auto it = coeffs.begin(); it != coeffs.end(); ++it)
		{
			if (it->first.owner == nullptr) {  // for testing
				VLOG(3) << it->second << "*#(" << it->first.id << ")+";
			}
			else {
				VLOG(3) << it->second << "*" << it->first.owner->describeSymbol(it->first.id) << " + ";
			}
		}
		cout << constant;
	}

	string LinearExpr::toString() const
	{
		ostringstream ss;
		for (auto it = coeffs.begin(); it != coeffs.end(); ++it)
		{
			ss << it->second << "*" << it->first.owner->describeSymbol(it->first.id) << " + ";
		}
		ss << constant;
		return ss.str();
	}

	bool LinearExpr::onlyContains(std::unordered_set<Symbol> symbols){
		for (auto it = coeffs.begin(); it != coeffs.end(); ++it){
			if (symbols.find(it->first) == symbols.end()){
				return false;
			}
		}
		return true;
	}

	void QuadExpr::print() const
	{
		for (auto it = coeffs.begin(); it != coeffs.end(); ++it)
		{
			if (it->first.owner == nullptr) {  // for testing
				cout << "#(" << it->first.id << ")*(";
			}
			else {
				cout << it->first.owner->describeSymbol(it->first.id) << "*(";
			}
			coeffs.at(it->first).print();
			cout << ") + ";
		}
		cout << endl;
	}

	TriMesh::BBox BBox3S::eval(SymbolicAssignment const& env) const {
		TriMesh::BBox result;
		point minPoint = min.evalpoint(env);
		point maxPoint = max.evalpoint(env);
		result.min = minPoint;
		result.max = maxPoint;
		return result;
	}

	BBox3S BBox3S::extend(BBox3S const& with, SymbolicAssignment const& env) const {
		auto bbox1 = eval(env);
		auto bbox2 = with.eval(env);
		BBox3S result;
		result.min.x = bbox1.min[0] < bbox2.min[0] ? this->min.x : with.min.x;
		result.min.y = bbox1.min[1] < bbox2.min[1] ? this->min.y : with.min.y;
		result.min.z = bbox1.min[2] < bbox2.min[2] ? this->min.z : with.min.z;

		result.max.x = bbox1.max[0] > bbox2.max[0] ? this->max.x : with.max.x;
		result.max.y = bbox1.max[1] > bbox2.max[1] ? this->max.y : with.max.y;
		result.max.z = bbox1.max[2] > bbox2.max[2] ? this->max.z : with.max.z;
		return result;
	}

	BBox3S BBox3S::unionOf(std::vector<BBox3S> const& boxes, SymbolicAssignment const& env) {
		if (boxes.empty()) return BBox3S();
		// TODO(robin): not very efficient, but let's keep it simple for now
		BBox3S bbox = boxes[0];
		for (int i = 1; i < boxes.size(); i++) {
			bbox = bbox.extend(boxes[i], env);
		}
		return bbox;
	}

	Point3S BBox3S::center() const {
		Point3S p;
		p.x = (min.x + max.x) / 2;
		p.y = (min.y + max.y) / 2;
		p.z = (min.z + max.z) / 2;
		return p;
	}

	namespace _Mesh3SImpl {
		struct trig {
			Eigen::Vector2d vertex[3];
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
			typedef vector<trig, Eigen::aligned_allocator<trig>> vector_type;
		};
		void triangulate(vector<p2t::Point*> const& face, trig::vector_type* out) {			
			PROFILE_THIS(__FUNCTION__);

			p2t::CDT cdt(face);
			cdt.Triangulate();
			auto result = cdt.GetTriangles();
			for (int i = 0; i < result.size(); i++) {
				trig t;
				p2t::Triangle* triangle = result[i];
				t.vertex[0][0] = triangle->GetPoint(0)->x;
				t.vertex[0][1] = triangle->GetPoint(0)->y;
				t.vertex[1][0] = triangle->GetPoint(1)->x;
				t.vertex[1][1] = triangle->GetPoint(1)->y;
				t.vertex[2][0] = triangle->GetPoint(2)->x;
				t.vertex[2][1] = triangle->GetPoint(2)->y;
				out->push_back(t);
			}
		}
		// out = list of three columns of triangle vertices (not rows)
		void triangulate3d(vector<Eigen::Vector3d> const& face, vector<Eigen::Matrix3d>* out) {
			PROFILE_THIS(__FUNCTION__);

			if( face.size() == 3){
				Eigen::Matrix3d trig3d;
				for (int j = 0; j < 3; j++) {
					trig3d.col(j) = face[j];
				}
				out->push_back(trig3d);
			}else{
				auto a = face[0];
				auto b = face[1];
				auto ab = b - a;

				// if the edges chosen are collinear
				int whichface = 2;
				bool isparallel = true;
				while (isparallel  && whichface < face.size()) {
					auto ctemp = face[whichface];
					auto actemp = ctemp - a;
					auto acdotab = ab.normalized().dot(actemp.normalized());
					isparallel = (abs(acdotab) == 1);

					++whichface;
				}

				auto c = face[whichface-1];
				auto ac = c - a;

				// make the rotation matrix that takes the face to one parallel to z-plane
				auto normal = ab.cross(ac).normalized();
				Eigen::Vector3d z = Eigen::Vector3d::Zero(3);
				z[2] = 1;
				Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(normal, z);
				Eigen::Matrix3d mat = q.toRotationMatrix();
				Eigen::Matrix3d invmat = q.conjugate().toRotationMatrix();

				{
					double area = 0;
					for (int i = 0; i < face.size(); i++) {
						int j = (i + 1) % face.size();
						Eigen::Vector2d v1 = (mat*face[i]).head(2);
						Eigen::Vector2d v2 = (mat*face[j]).head(2);
						area += (v2[0] - v1[0]) * (v2[1] + v1[1]);
					}
					if (area > 0) {
						Eigen::Matrix3d flip = Eigen::AngleAxisd(atan(1.0) * 4, Eigen::Vector3d(0, 1, 0)).toRotationMatrix();
						mat = flip * mat;
						invmat = invmat * flip.inverse();
					}
				}

				vector<p2t::Point*> face2d;
				trig::vector_type out2d;
				double zOffset;

				//std::cout << "found a face that has " << face.size() << std::endl;
				//for (int ii = 0; ii < face.size(); ii++) {
				//	std::cout << face[ii].transpose() << std::endl;
				//}

				for (int i = 0; i < face.size(); i++) {
					Eigen::Vector3d flat = mat * face[i];
					Eigen::Vector2d flat2 = flat.segment(0, 2);

					//std::cout << "flat = " << flat.transpose() << std::endl;
					//std::cout << "i = " << i << std::endl;
					//std::cout << "zOffset = " << zOffset << std::endl;

					if (i == 0) {
						zOffset = flat[2];
					}
					else {
						//assert(abs(zOffset - flat[2]) < 1e-6 || abs((zOffset - flat[2]) / zOffset) < 1e-6);
					}
					face2d.push_back(new p2t::Point(flat2[0], flat2[1]));
				}
				//std::cout << "will try to triangulate" << std::endl;
				triangulate(face2d, &out2d);
				for (int i = 0; i < out2d.size(); i++) {
					Eigen::Matrix3d trig3d;
					for (int j = 0; j < 3; j++) {
						Eigen::Vector3d v3d;
						v3d.segment(0,2) = out2d[i].vertex[j];
						v3d[2] = zOffset;
						v3d = invmat * v3d;
						trig3d.col(j) = v3d;
					}
					out->push_back(trig3d);
				}
			}
		}
	}

	point toPoint(Eigen::Vector3d const& v) {
		return point(static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2]));
	}

	TriMesh* Mesh3S::eval(SymbolicAssignment const& env, vector<int>* faceIndices) const {
		PROFILE_THIS(__FUNCTION__);
		using namespace _Mesh3SImpl;
		TriMesh* result = new TriMesh();


		if (faceIndices != nullptr) {
			faceIndices->clear();
		}

		bool facesNeedTriag = false;
		for (int i = 0; i < faces.size(); i++) {
			if(faces[i].vertices.size() != 3){
				facesNeedTriag = true;
			}
		}
		if (facesNeedTriag){
			vector<Eigen::Vector3d> evaluatedVertices;
			for (int i = 0; i < vertices.size(); i++) {
				evaluatedVertices.push_back(vertices[i].evalVector3d(env));
			}
			// we need to triangulate the faces
			for (int i = 0; i < faces.size(); i++) {
				vector<Eigen::Vector3d> auxvertices;
				auto& face = faces[i];
				//cout << "For face #" << i << endl;
				for (int j = 0; j < face.vertices.size(); j++) {
					auxvertices.push_back(evaluatedVertices[face.vertices[j]]);
					//cout << "Vertex " << j << " = (" << auxvertices[j][0] << ", " << auxvertices[j][1] << ", " << auxvertices[j][2] << ')' << endl;
				}
				vector<Eigen::Matrix3d> trigs;
				triangulate3d(auxvertices, &trigs);
				for (int j = 0; j < trigs.size(); j++) {
					int index0 = result->vertices.size();
					auto& trig = trigs[j];
					result->vertices.push_back(toPoint(trig.col(0)));
					result->vertices.push_back(toPoint(trig.col(1)));
					result->vertices.push_back(toPoint(trig.col(2)));
					result->faces.push_back(TriMesh::Face(index0, index0 + 1, index0 + 2));
					if (faceIndices != nullptr) {
						faceIndices->push_back(i);
					}
				}
			}
		}
		else{
			for (int i = 0; i < vertices.size(); i++) {
				result->vertices.push_back(toPoint(vertices[i].evalVector3d(env)));
			}
			for (int i = 0; i < faces.size(); i++) {
				result->faces.push_back(TriMesh::Face(faces[i].vertices[0], 
						faces[i].vertices[1], faces[i].vertices[2]));
				if (faceIndices != nullptr){
					faceIndices->push_back(i);
				}
			}
		}
		return result;
	}

	mesh::Mesh* Mesh3S::evalMesh(SymbolicAssignment const& env) const {
		using namespace _Mesh3SImpl;
		mesh::Mesh* result = new mesh::Mesh();

		vector<Eigen::Vector3d> evaluatedVertices;
		for (int i = 0; i < vertices.size(); i++) {
			evaluatedVertices.push_back(vertices[i].evalVector3d(env));
		}
		// we need to triangulate the faces
		for (int i = 0; i < faces.size(); i++) {
			vector<Eigen::Vector3d> auxvertices;
			auto& face = faces[i];
			for (int j = 0; j < face.vertices.size(); j++) {
				auxvertices.push_back(evaluatedVertices[face.vertices[j]]);
				mesh::Edge edge;
				edge.vertices.push_back(evaluatedVertices[face.vertices[j]]);
				edge.vertices.push_back(evaluatedVertices[face.vertices[(j + 1) % face.vertices.size()]]);
				edge.edgeid = i * 10000 + j;
				result->edges.push_back(edge);
			}
			vector<Eigen::Matrix3d> trigs;
			triangulate3d(auxvertices, &trigs);
			for (int j = 0; j < trigs.size(); j++) {
				int index0 = result->points.size();
				auto& trig = trigs[j];
				result->points.push_back(trig.col(0));
				result->points.push_back(trig.col(1));
				result->points.push_back(trig.col(2));
				mesh::Face f;
				f.points.push_back(index0);
				f.points.push_back(index0+1);
				f.points.push_back(index0+2);
				result->faces.push_back(f);


			}
		}
		return result;
	}

	DebugInfo* Mesh3S::getDebugInfo() {
		DebugInfo* info = new DebugInfo();
		info->setTypeName("Mesh3S");
		info->setShortDescription("3D symbolic mesh");
		for (int i = 0; i < this->vertices.size(); i++) {
			info->putStringProperty(concat() << "vertices[" << i << "].x", this->vertices[i].x.toString());
			info->putStringProperty(concat() << "vertices[" << i << "].y", this->vertices[i].y.toString());
			info->putStringProperty(concat() << "vertices[" << i << "].z", this->vertices[i].z.toString());
		}
		return info;
	}

	BBox3S Mesh3S::evalLocalBoundingBox(SymbolicAssignment const& env) const {
		Eigen::Vector3i minIndex, maxIndex;
		Eigen::Vector3d minValue, maxValue;
		for (int i = 0; i < vertices.size(); i++) {
			Eigen::Vector3d evaluated = vertices[i].evalVector3d(env);
			for (int j = 0; j < 3; j++) {
				if (i == 0 || evaluated[j] < minValue[j]) {
					minValue[j] = evaluated[j];
					minIndex[j] = i;
				}
				if (i == 0 || evaluated[j] > maxValue[j]) {
					maxValue[j] = evaluated[j];
					maxIndex[j] = i;
				}
			}
		}
		BBox3S result;
		for (int j = 0; j < 3; j++) {
			result.min[j] = vertices[minIndex[j]][j];
			result.max[j] = vertices[maxIndex[j]][j];
		}
		return result;
	}

	vector3f Point3S::evalvector3f(SymbolicAssignment const& env) const {
		return vector3f(x.eval(env), y.eval(env), z.eval(env));
	}

	point Point3S::evalpoint(SymbolicAssignment const& env) const {
		point result;
		result[0] = x.eval(env);
		result[1] = y.eval(env);
		result[2] = z.eval(env);
		return result;
	}

	LinearExpr& Point3S::operator[](int index) {
		switch (index) {
		case 0:
			return x;
		case 1:
			return y;
		case 2:
			return z;
		}
		throw std::string("Invalid index");
	}
	
	Point3S Point3S::operator*(double coeff){
		Point3S res;
		res.x = coeff*x;
		res.y = coeff*y;
		res.z = coeff*z;
		return res;
	}
	
	Point3S Point3S::operator+(Point3S other){
		Point3S res;
		res.x = x + other.x;
		res.y = y + other.y;
		res.z = z + other.z;
		return res;
	}

	Eigen::Vector3d Point3S::evalVector3d(SymbolicAssignment const& env) const {
		Eigen::Vector3d result;
		result[0] = x.eval(env);
		result[1] = y.eval(env);
		result[2] = z.eval(env);
		return result;
	}

	LinearExpr const& Point3S::operator[](int index) const {
		switch (index) {
		case 0:
			return x;
		case 1:
			return y;
		case 2:
			return z;
		}
		throw std::string("Invalid index");
	}
	
	Point3S Point3S::times(Eigen::Matrix3d matrix){
		Point3S res;
		res.x = matrix(0, 0) * x + matrix(0, 1) * y + matrix(0, 2) * z;
		res.y = matrix(1, 0) * x + matrix(1, 1) * y + matrix(1, 2) * z;
		res.z = matrix(2, 0) * x + matrix(2, 1) * y + matrix(2, 2) * z;
		return res;
	}

	Point3S Point3S::translate(Eigen::Vector3d trans){
		Point3S p;
		p.x = x + trans[0];
		p.y = y + trans[1];
		p.z = z + trans[2];
		return p;
	}
	
	Point3S Point3S::evalPartial(SymbolicAssignment const& env) const {
		Point3S p;
		p.x = x.evalPartial(env);
		p.y = y.evalPartial(env);
		p.z = z.evalPartial(env);
		return p;
	}

	 drawing::Face* Face2S::eval(SymbolicAssignment const& env) const {
        drawing::Face* res = new drawing::Face();

		// TODO: make this cleaner by combining with iteration over edges below
		for (int edgeit = 0; edgeit < edges.size(); ++edgeit) {
			res->edgeMap[edges[edgeit].vertice1] = edgeit;
		}

		//std::unordered_map<int, Point2S>::iterator it;
		int count = 0;
		//for (auto& it = vertices.begin(); it != vertices.end(); ++it){
		auto& it = vertices.begin();

		// store the first vertex
			auto& point = (*it).second;
			drawing::Point dPoint;
			dPoint.p = point.evalVector2d(env);
			dPoint.id = (*it).first;
			res->idpoints.push_back(dPoint); 
			//res->points.push_back(point.evalVector2d(env));
			//res->mapping[(*it).first] = count;
			//count++;
		//}

		int nextEdge = res->edgeMap[dPoint.id];
		int nextid = edges[nextEdge].vertice2;
		
		while (nextid != dPoint.id) {
			// store the next vertex
			drawing::Point nextPoint;
			nextPoint.id = nextid;
			auto& point = vertices.at(nextid);
			nextPoint.p = point.evalVector2d(env);
			res->idpoints.push_back(nextPoint);

			nextEdge = res->edgeMap[nextid];
			nextid = edges[nextEdge].vertice2;
		}

		for(auto it = edges.begin(); it != edges.end(); ++it) {
			drawing::Edge edge;
			edge.id = it->id;
			edge.name = it->name;
			
			edge.vertice1_id = it->vertice1;
			edge.vertice2_id = it->vertice2;

			edge.trimFront = 0;
			edge.trimBack = 0;

			res->edges.push_back(edge);

		}

		return res;
    }


    drawing::Drawing* Drawing2S::eval(SymbolicAssignment const& env) const {
        drawing::Drawing* res = new drawing::Drawing();

		for(int i=0; i < this->vertices.size(); i++) {
			auto& point = this->vertices[i];
			res->points.push_back(point.evalVector2d(env));
			
		}

        for(int i=0;i<this->faces.size();i++) {
            drawing::Face face;
            face.id = this->faces[i].id;
            face.name = this->faces[i].name;
            for(int j=0;j<this->faces[i].vertices.size();j++) {
                auto& point = this->vertices[this->faces[i].vertices[j]];
				drawing::Point dPoint;
				dPoint.p = point.evalVector2d(env);
				dPoint.id = this->faces[i].vertices[j];
				face.idpoints.push_back(dPoint); 
				//face.points.push_back(point.evalVector2d(env));
				//face.mapping[this->faces[i].vertices[j]] = j;
            }
			
            res->faces.push_back(face);
        }

		for(auto it = edges.begin(); it != edges.end(); ++it) {
			drawing::Edge edge;
			edge.id = it->id;
			edge.name = it->name;
			
			edge.vertice1_id = it->vertice1;
			
			
			edge.vertice2_id = it->vertice2;

			res->edges.push_back(edge);

		}
        return res;
    }

	LinearExpr const& Point2S::operator[](int index) const {
		switch (index) {
		case 0:
			return x;
		case 1:
			return y;
		}
		throw std::string("Invalid index");
	}

	LinearExpr& Point2S::operator[](int index) {
		switch (index) {
		case 0:
			return x;
		case 1:
			return y;
		}
		throw std::string("Invalid index");
	}

	Point2S Point2S::operator*(double coeff){
		Point2S res;
		res.x = coeff*x;
		res.y = coeff*y;
		return res;
	}

	Point2S Point2S::operator+(Point2S other){
		Point2S res;
		res.x = x + other.x;
		res.y = y + other.y;
		return res;
	}

	Eigen::Vector2d Point2S::evalVector2d(SymbolicAssignment const& env) const {
		Eigen::Vector2d result;
		result[0] = x.eval(env);
		result[1] = y.eval(env);
		return result;
    }

	Point2S Point2S::times(const Eigen::Matrix2d& m){
		Point2S p;
		p.x = m(0, 0)*x + m(0, 1)*y;
		p.y = m(1, 0)*x + m(1, 1)* y;
		return p;
	}
	Point2S Point2S::translate(const Eigen::Vector2d& v){
		Point2S p;
		p.x = x + v[0];
		p.y = y + v[1];
		return p;
	}

	Eigen::Matrix2d getRotationMatrixFromTwoVector2d(const Eigen::Vector2d& from, const Eigen::Vector2d& to){
		///double alpha = acos(to.dot(from) / (to.norm()*from.norm()));
		Eigen::Vector2d f = from.normalized();
		Eigen::Vector2d t = to.normalized();
		Eigen::Matrix2d m1, m2;
		m1(0, 0) = f[0];
		m1(0, 1) = -f[1];
		m1(1, 0) = f[1];
		m1(1, 1) = f[0];
		m2(0, 0) = t[0];
		m2(0, 1) = -t[1];
		m2(1, 0) = t[1];
		m2(1, 1) = t[0];
		std::cout << m2.inverse()*m1;
		return (m2.inverse()*m1).inverse();
	}
	
	Eigen::Vector2d getTranslationMatrixFromTwoVector2d(const Eigen::Vector2d& from, const Eigen::Vector2d& to){
		return to - from;
	}

	Eigen::Matrix3d RotationMatrixFromTwoVectors(const Eigen::Vector3d from, const Eigen::Vector3d to){
		Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(from, to);
		return q.toRotationMatrix();
	}
}