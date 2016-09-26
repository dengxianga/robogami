#pragma once

#include <unordered_map>
#include <array>
#include "..\include\TriMesh.h"
#include "mathVec.h"
#include <Eigen/Dense>
#include "drawing.h"
#include "mesh.h"
#include <unordered_set>
#include <string>
#include "../FBE_Utils/debugging.h"
using namespace FabDebugging;

// Very useful macro from Google coding style.
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
TypeName(const TypeName&);                 \
void operator=(const TypeName&)


#pragma region Define Symbol, which instead of being an absolute offset, is an owner (most likely an Element*) and the offset into that owner.
namespace FabByExample
{
	class IDefinesSymbols;
	template<typename T>
	struct const_symbol {};
	struct SymbolByOwnerAndId {
		IDefinesSymbols* owner;
		int id;
		bool operator==(const SymbolByOwnerAndId& other) const {
			return this->owner == other.owner && this->id == other.id;
		}
		SymbolByOwnerAndId(IDefinesSymbols* owner, int id) {
			this->owner = owner;
			this->id = id;
		}
		SymbolByOwnerAndId() {
			this->owner = nullptr;
			this->id = -1;
		}
	};
	template<>
	struct const_symbol<SymbolByOwnerAndId> {
		static SymbolByOwnerAndId value() {
			return SymbolByOwnerAndId(nullptr, -1);
		}
	};
}
namespace std{
	template<>
	struct hash<FabByExample::SymbolByOwnerAndId> {
		size_t operator()(const FabByExample::SymbolByOwnerAndId& s) const
		{
			return hash<const FabByExample::IDefinesSymbols*>()(s.owner) ^ hash<int>()(s.id);
		}
	};
}

namespace FabByExample {
	typedef SymbolByOwnerAndId Symbol;
	static Symbol ConstSymbol() {
		return const_symbol<Symbol>::value();
	}
}

namespace FabByExample {
	class IDefinesSymbols {
	public:
		virtual int numSymbols() const = 0;
		virtual std::string describeSymbol(int id) const = 0;
		virtual SymbolByOwnerAndId getSymbol(int id) {
			return SymbolByOwnerAndId(this, id);
		}
		virtual double getCurrentValue(int id) const = 0;
		virtual void setCurrentValue(int id, double _val) = 0;
		virtual std::string getSymbolName(int id) const = 0;
	};
}
#pragma endregion


#pragma region Adapt fast_hashmap

#define I_KNOW_HOW_TO_INCLUDE_FAST_HASHMAP
#define FAST_HASHMAP_HEADERS
namespace _CoeffsMapImpl {
	typedef FabByExample::Symbol Key;
	typedef double Value;
	#include "fast_hashmap.h"
}
namespace FabByExample {
	typedef _CoeffsMapImpl::fast_hashmap CoeffsMap;
}
#undef FAST_HASHMAP_HEADERS
#undef I_KNOW_HOW_TO_INCLUDE_FAST_HASHMAP

#pragma endregion


namespace FabByExample {

	class SymbolicAssignment {
	public:
		virtual double get(const Symbol& symbol) const = 0;
		virtual bool has(const Symbol& symbol) const = 0;
		DISALLOW_COPY_AND_ASSIGN(SymbolicAssignment);

		static const SymbolicAssignment& USE_CURRENT_VALUES;
	protected:
		SymbolicAssignment() {}
	};

	class ConcreteSymbolicAssignment : public SymbolicAssignment {
	public:
		ConcreteSymbolicAssignment() : map(){}
		virtual double get(const Symbol& symbol) const {
			return map.at(symbol);
		}
		virtual bool has(const Symbol& symbol) const {
			return map.has(symbol);
		}
		CoeffsMap map;

		DISALLOW_COPY_AND_ASSIGN(ConcreteSymbolicAssignment);
	};

	class UseCurrentValuesSymbolicAssignment : public SymbolicAssignment {
	public:
		UseCurrentValuesSymbolicAssignment() {}
		virtual double get(const Symbol& symbol) const {
			return symbol.owner->getCurrentValue(symbol.id);
		}
		virtual bool has(const Symbol& symbol) const {
			return symbol.owner == nullptr;
		}
		DISALLOW_COPY_AND_ASSIGN(UseCurrentValuesSymbolicAssignment);
	};

	class QuadExpr;

	class LinearExpr
	{
	private:
		CoeffsMap coeffs;
		double constant;
	public:
		LinearExpr();
		LinearExpr(double constant);
		LinearExpr(Symbol variable);
		LinearExpr(const LinearExpr& from);
		LinearExpr(const CoeffsMap& coeffs, double constant);
		LinearExpr& operator=(const LinearExpr& that);
		bool operator==(LinearExpr const& that) const;

		LinearExpr operator+(const LinearExpr& that) const;
		LinearExpr operator+(double toadd) const;
		LinearExpr operator-(const LinearExpr& that) const;
		LinearExpr operator-(double tosub) const;
		LinearExpr operator*(double factor) const;
		QuadExpr   operator*(const LinearExpr& that) const;
		LinearExpr operator/(double factor) const;

		const CoeffsMap& getCoeffs() const;
		double getConstant() const;
		bool isConstant() const;
		std::array<double, 1> match(Symbol param, bool* result) const;
		std::array<double, 2> match(Symbol param1, Symbol param2, bool* result) const;
		std::array<double, 3> match(Symbol param1, Symbol param2, Symbol param3, bool* result) const;
		std::array<double, 4> match(Symbol param1, Symbol param2, Symbol param3, Symbol param4, bool* result) const;

		double& operator[](Symbol index);

		double eval(SymbolicAssignment const& env) const;
		LinearExpr evalPartial(SymbolicAssignment const& env) const;

		friend LinearExpr operator-(const LinearExpr&);
		friend LinearExpr operator*(double factor, const LinearExpr&);

		void print() const;
		std::string toString() const;
		bool onlyContains(std::unordered_set<Symbol> symbols);

	private:
		std::vector<double> match(std::vector<Symbol> const& params, bool* result) const;
		LinearExpr(int DO_NOT_USE_THIS_CONSTRUCTOR);
	};
}

namespace std{
	using namespace FabByExample;
	template<>
	struct hash<CoeffsMap> {
		size_t operator()(const CoeffsMap& m) const {
			size_t h = 0;
			for each (auto const& pair in m) {
				h ^= hash<FabByExample::Symbol>()(pair.first);
				h ^= hash<double>()(pair.second);
			}
			return h;
		}
	};
	template<>
	struct hash<FabByExample::LinearExpr> {
		size_t operator()(const FabByExample::LinearExpr& s) const
		{
			return hash<CoeffsMap>()(s.getCoeffs()) ^ hash<double>()(s.getConstant());
		}
	};
}

namespace FabByExample {
	
	class QuadExpr
	{
	private:
		std::unordered_map<Symbol, LinearExpr> coeffs;
	
	public:
		QuadExpr();
		QuadExpr(double constant);
		QuadExpr(Symbol variable);
		QuadExpr(const LinearExpr& from);
		QuadExpr(LinearExpr&& from);
		QuadExpr(const QuadExpr& from);
		QuadExpr(QuadExpr&& from);
		QuadExpr(const std::unordered_map<Symbol, LinearExpr>& coeffs);
		
		QuadExpr& operator=(const QuadExpr& that);
		QuadExpr& operator=(QuadExpr&& that);
		
		QuadExpr operator+(const QuadExpr& that) const;
		QuadExpr operator+(const LinearExpr& toadd) const;
		QuadExpr operator+(double toadd) const;
		QuadExpr operator-(const QuadExpr& that) const;
		QuadExpr operator-(const LinearExpr& tosub) const;
		QuadExpr operator-(double tosub) const;
		QuadExpr operator*(double factor) const;
		QuadExpr operator/(double factor) const;
		
		const std::unordered_map<Symbol, LinearExpr>& getCoeffs() const;
		double getConstant() const;
		bool isConstant() const;
		QuadExpr simplify() const;
		
		LinearExpr& operator[](Symbol index);
		
		double eval(SymbolicAssignment const& env) const;
		QuadExpr evalPartial(SymbolicAssignment const& env) const;
		
		friend QuadExpr operator-(const QuadExpr&);
		friend QuadExpr operator*(double factor, const QuadExpr&);
		
		void print() const;
	private:
		//std::vector<LinearExpr> match(std::vector<Symbol> const& params, bool* result) const;
	};

	typedef struct Point3S {
		LinearExpr x, y, z;
		vector3f evalvector3f(SymbolicAssignment const& env) const;
		point evalpoint(SymbolicAssignment const& env) const;
		Eigen::Vector3d evalVector3d(SymbolicAssignment const& env) const;
		LinearExpr const& operator[](int index) const;
		LinearExpr& operator[](int index);
		Point3S operator*(double coeff);
		Point3S operator+ (Point3S other);
		Point3S times(Eigen::Matrix3d matrix);
		Point3S translate(Eigen::Vector3d trans);
		Point3S evalPartial(SymbolicAssignment const& env) const;
	} Vector3S;

	struct Face3Sp {
		int id;
		std::string name;
		std::vector<int> vertices;
	};

	struct BBox3S {
		Point3S min;
		Point3S max;
		::TriMesh::BBox eval(SymbolicAssignment const& env) const;
		BBox3S extend(const BBox3S& with, SymbolicAssignment const& env) const;
		static BBox3S unionOf(const std::vector<BBox3S>& boxes, SymbolicAssignment const& env);
		Point3S center() const;
	};

	struct Mesh3S : public Debuggable {
		std::vector<Point3S> vertices;
		std::vector<Face3Sp> faces;

		// Substitute the values given in env to return a concrete numeric mesh.
		// This process will triangulate the faces.
		//
		// faceIndices is an optional output (nullptr if not needed) that will contain a mapping
		// from the index of face in the output TriMesh to the index of face in the Mesh3S.
		::TriMesh* eval(SymbolicAssignment const& env, std::vector<int>* faceIndices=nullptr) const;
		mesh::Mesh* evalMesh(SymbolicAssignment const& env) const;

		// Compute the bounding box as symbolic expressions, in the currently assigned
		// values. Theoretically the expressions for the bounding box may change
		// depending on the values they actually take, but having such expressions
		// locally for the current values will be helpful for manipulation purposes.
		BBox3S evalLocalBoundingBox(SymbolicAssignment const& env) const;

		virtual DebugInfo* getDebugInfo();
	};

	typedef struct Point2S {
		LinearExpr x, y;
		Eigen::Vector2d evalVector2d(SymbolicAssignment const& env) const;
		LinearExpr const& operator[](int index) const;
		LinearExpr& operator[](int index);
		Point2S times(const Eigen::Matrix2d& m);
		Point2S translate(const Eigen::Vector2d& v);
		Point2S operator*(double coeff);
		Point2S operator+ (Point2S other);
	} Vector2S;

	struct Face2Sp {
		int id;
		std::string name;
		std::vector<int> vertices;
	};



	struct BBox2S {
		Point2S min;
		Point2S max;
		Point2S mid;
		// return a matrix whose first column is the min and second column is the max
		Eigen::Matrix2d eval(SymbolicAssignment const& env) const;
	};

	struct Edge2S {
		int id;
		std::string name;
		int vertice1;
		int vertice2;
	};
	struct Face2S {
		int id;
		std::string name;
		std::unordered_map<int, Point2S> vertices;
		std::vector<Edge2S> edges;
		drawing::Face* eval(SymbolicAssignment const& env) const;

	};
	struct Drawing2S {
		std::vector<Point2S> vertices;
		std::vector<Face2Sp> faces;
		std::vector<Edge2S> edges;

		// Substitute the values given in env to return a concrete numeric mesh
		drawing::Drawing* eval(SymbolicAssignment const& env) const;
	};

	Eigen::Matrix3d RotationMatrixFromTwoVectors(const Eigen::Vector3d from, const Eigen::Vector3d to);
	
	Eigen::Matrix2d getRotationMatrixFromTwoVector2d(const Eigen::Vector2d& from, const Eigen::Vector2d& to);
	Eigen::Vector2d getTranslationMatrixFromTwoVector2d(const Eigen::Vector2d& from, const Eigen::Vector2d& to);
	
	template<typename SomeRowVectorXd>
	void flattenSymbolic(const vector<Symbol>& symbols, LinearExpr const& expr, const Eigen::MatrixBase<SomeRowVectorXd>& out_coeffs, double& constant) {
		Eigen::MatrixBase<SomeRowVectorXd>& coeffs = const_cast<Eigen::MatrixBase<SomeRowVectorXd>&>(out_coeffs);
		const auto& exprCoeffs = expr.getCoeffs();
		for (int i = 0; i < symbols.size(); i++) {
			coeffs[i] = exprCoeffs.has(symbols[i]) ? exprCoeffs.at(symbols[i]) : 0;
		}
		constant = expr.getConstant();
	}

	template<typename SomeMatrix2Xd, typename SomeVector2d>
	void flattenSymbolic(const vector<Symbol>& symbols, Point2S const& expr, const Eigen::MatrixBase<SomeMatrix2Xd>& out_coeffs, const Eigen::MatrixBase<SomeVector2d>& out_constants) {
		Eigen::MatrixBase<SomeMatrix2Xd>& coeffs = const_cast<Eigen::MatrixBase<SomeMatrix2Xd>&>(out_coeffs);
		Eigen::MatrixBase<SomeVector2d>& constants = const_cast<Eigen::MatrixBase<SomeVector2d>&>(out_constants);
		for (int i = 0; i < 2; i++) {
			const auto& exprCoeffs = expr[i].getCoeffs();
			for (int j = 0; j < symbols.size(); j++) {
				coeffs.row(i)[j] = exprCoeffs.has(symbols[i]) ? exprCoeffs.at(symbols[i]) : 0;
			}
			constants[i] = expr[i].getConstant();
		}
		
	}

	template<typename SomeMatrix3Xd, typename SomeVector3d>
	void flattenSymbolic(const vector<Symbol>& symbols, Point3S const& expr, const Eigen::MatrixBase<SomeMatrix3Xd>& out_coeffs, const Eigen::MatrixBase<SomeVector3d>& out_constants) {
		Eigen::MatrixBase<SomeMatrix3Xd>& coeffs = const_cast<Eigen::MatrixBase<SomeMatrix3Xd>&>(out_coeffs);
		Eigen::MatrixBase<SomeVector3d>& constants = const_cast<Eigen::MatrixBase<SomeVector3d>&>(out_constants);
		for (int i = 0; i < 3; i++) {
			const auto& exprCoeffs = expr[i].getCoeffs();
			for (int j = 0; j < symbols.size(); j++) {
				coeffs.row(i)[j] = exprCoeffs.has(symbols[i]) ? exprCoeffs.at(symbols[i]) : 0;
			}
			constants[i] = expr[i].getConstant();
		}
	}
}
