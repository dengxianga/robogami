#pragma once

#include <vector>
#include <unordered_map>
#include "symbolic.h"
#include "template.h"

template<>
struct hash<Eigen::MatrixXd>
{
public:
	size_t operator()(Eigen::MatrixXd const& s) const
	{
		return 0;
	}
};

namespace FabByExample{
	class Constraint;

	// A flattened version of FriendlyLinearSystem.
	class FlatLinearSystem {
	public:
		FlatLinearSystem(){}
		// The reduced (independent rows) equation system Ax = b, representing the
		// constraints.
		Eigen::MatrixXd Aeq_red;
		Eigen::VectorXd Beq_red;

		// The inequation system Ax <= b, representing the inequality constraints.
		Eigen::MatrixXd Aineq;
		Eigen::VectorXd Bineq;

		// The system to optimize.
		Eigen::MatrixXd Aopt;
		Eigen::VectorXd Bopt;

		// The parameter array. Corresponds to the columns of the above matrices.
		// When created by FriendlyLinearSystem::flatten, this vector contains the
		// current parameter values. One may modify this vector, and then call
		// FriendlyLinearSystem::updateFromFlattened, which will use this vector to
		// assign the new values.
		Eigen::VectorXd q;

		// Metadata added by FriendlyLinearSystem representing the symbol to index
		// map. For example, if [A, B, C] were the symbols, and B was eliminated
		// due to being equal to A, and suppose the initial values were A=1, B=1,
		// C=3, then
		//   q = [1, 3]
		//   symbolIndices = [0, 0, 1]
		// This is only used by FriendlyLinearSystem::updateFromFlattened.
		Eigen::VectorXi symbolIndices;
	};

	class FriendlyLinearSystem {
	private:
		vector<LinearExpr> eqConstraints;
		vector<string> eqcLabel;
		vector<LinearExpr> ineqConstraints;
		vector<string> ineqcLabel;
		vector<pair<double, LinearExpr>> equationsToOptimize;
		vector<string> etoLabel;
		static unordered_map<Eigen::MatrixXd, pair<Eigen::MatrixXd, int> > qrCache;
	public:
		FriendlyLinearSystem() {}
		~FriendlyLinearSystem() {
			eqConstraints.clear();
			eqcLabel.clear();
			ineqConstraints.clear();
			ineqcLabel.clear();
			equationsToOptimize.clear();
			etoLabel.clear();
			qrCache.clear();
		}
		void addEqConstraint(const LinearExpr& eq, string label="");
		void addIneqConstraint(const LinearExpr& ineq, string label="");
		void addEquationToOptimize(const LinearExpr& eq, double weight=1.0, string label="");
		double optimizeAndUpdate(Template* target);
		double optimizeWithoutUpdating(Template* target);
		double optimizeAndUpdateInefficient(Template* target);

		// Return a flattened version of this linear system.
		// args:
		//   target - The Template* that this linear system concerns.
		//   eliminateVars - if true, equal variables are collapsed which reduces the matrix sizes
		//   flat - A FlatLinearSystem to output to. It is passed in instead of returned because
		//          copying matrix objects may be expensive.
		void flatten(Template* target, boolean eliminateVars, FlatLinearSystem* flat);

		// Given a FlatLinearSystem with the optimized q vector, update the target template with
		// these updated parameters.
		void updateFromFlattened(Template* target, FlatLinearSystem const& system);

		pair<Eigen::MatrixXd, int> doCachedQRDecomposition(const Eigen::MatrixXd& matrix);
		void print();

		DISALLOW_COPY_AND_ASSIGN(FriendlyLinearSystem);
	};
}
