#include "FriendlyLinearSystem.h"
#include <myOptimizer.h>
#include <unordered_set>

#define DEBUG true

unordered_map<Eigen::MatrixXd, pair<Eigen::MatrixXd, int> > FabByExample::FriendlyLinearSystem::qrCache;

namespace FabByExample {
	void FriendlyLinearSystem::addEqConstraint(LinearExpr const& eq, string label) {
		PROFILE_THIS(__FUNCTION__);
		eqConstraints.push_back(eq);
		eqcLabel.push_back(label);
	}

	void FriendlyLinearSystem::addIneqConstraint(LinearExpr const& ineq, string label) {
		PROFILE_THIS(__FUNCTION__);
		ineqConstraints.push_back(ineq);
		ineqcLabel.push_back(label);
	}

	void FriendlyLinearSystem::addEquationToOptimize(LinearExpr const& ineq, double weight, string label) {
		PROFILE_THIS(__FUNCTION__);
		equationsToOptimize.push_back(make_pair(weight, ineq));
		etoLabel.push_back(label);
	}

	namespace {
		bool equals(double a, double b) {
			return abs(a - b) < 1e-6;
		}

		void eliminate(Symbol const& to, Symbol const& from, unordered_map<Symbol, Symbol>* replacements) {
			if (to == from) return;
			if (replacements->find(to) != replacements->end()) {
				eliminate(Symbol(replacements->at(to)), from, replacements);  // copy needed to avoid obscure bug with constness!
				return;
			}
			if (replacements->find(from) != replacements->end()) {
				eliminate(to, Symbol(replacements->at(from)), replacements);
				return;
			}
			for (auto it = replacements->begin(); it != replacements->end(); it++) {
				if (it->second == from) {
					it->second = to;
				}
			}
			(*replacements)[from] = to;
		}

		void eliminateVariables(vector<LinearExpr> const& eqConstraints, unordered_map<Symbol, Symbol>* replacements) {
			replacements->clear();
			for each (auto const& le in eqConstraints) {
				auto const& coeffs = le.getCoeffs();
				if (coeffs.size() == 2 && equals(0, le.getConstant())) {
					auto it = coeffs.begin();
					Symbol sA = it->first;
					double cA = it->second;
					++it;
					Symbol sB = it->first;
					double cB = it->second;
					if (equals(cA, -cB)) {
						eliminate(sA, sB, replacements);
					}
				}
			}
		}
		LinearExpr replaceSymbols(LinearExpr const& le, unordered_map<Symbol, Symbol> const& replacements) {
			CoeffsMap newCoeffs;
			for each (auto const& pair in le.getCoeffs()) {
				if (replacements.find(pair.first) != replacements.end()) {
					Symbol replaced = replacements.at(pair.first);
					newCoeffs[replaced] += pair.second;
				}
				else {
					newCoeffs[pair.first] += pair.second;
				}
			}
			return LinearExpr(newCoeffs, le.getConstant());
		}

	}

	double FabByExample::FriendlyLinearSystem::optimizeAndUpdateInefficient(Template* target) {
		PROFILE_THIS(__FUNCTION__);

		FlatLinearSystem flat;
		flatten(target, false, &flat);

		Eigen::VectorXd lb, ub;
		int status = MyOptimizer::leastSquaresWithConstraints(
			flat.Aopt, flat.Bopt, flat.Aineq, flat.Bineq,
			flat.Aeq_red, flat.Beq_red, lb, ub, flat.q);

		updateFromFlattened(target, flat);
		return (flat.Aopt * flat.q - flat.Bopt).squaredNorm();
	}

	double FriendlyLinearSystem::optimizeAndUpdate(Template* target) {
		PROFILE_THIS(__FUNCTION__);

		FlatLinearSystem flat;
		flatten(target, true, &flat);

		Eigen::VectorXd lb, ub;
		int status = MyOptimizer::leastSquaresWithConstraints(
			flat.Aopt, flat.Bopt, flat.Aineq, flat.Bineq,
			flat.Aeq_red, flat.Beq_red, lb, ub, flat.q);

		double error = 0;
		if(status && DEBUG){
			//VLOG(3) << "status = " << status << std::endl;
			//system("pause"); 
		}else
		{		
			updateFromFlattened(target, flat);
			error =  (flat.Aopt * flat.q - flat.Bopt).squaredNorm();
		}
		return error;
	}

	double FriendlyLinearSystem::optimizeWithoutUpdating(Template* target) {
		PROFILE_THIS(__FUNCTION__);

		FlatLinearSystem flat;
		flatten(target, true, &flat);

		Eigen::VectorXd lb, ub;
		int status = MyOptimizer::leastSquaresWithConstraints(
			flat.Aopt, flat.Bopt, flat.Aineq, flat.Bineq,
			flat.Aeq_red, flat.Beq_red, lb, ub, flat.q);

		if (status != 0) {
			return -1;
		}
		return (flat.Aopt * flat.q - flat.Bopt).squaredNorm();
	}

	void FriendlyLinearSystem::flatten(Template* target, boolean eliminateVars, FlatLinearSystem* flat) {
		PROFILE_THIS(__FUNCTION__);

		if (eliminateVars) {

			PROFILE_BEGIN("variable reduction in flatten");
			auto symbols = target->getSymbols(TreeScope::DESCENDANTS);
			unordered_map<Symbol, int> originalSymbolToId;
			for each (auto const& symbol in symbols) {
				originalSymbolToId[symbol] = originalSymbolToId.size();
			}

			unordered_map<Symbol, Symbol> replacements;
			eliminateVariables(eqConstraints, &replacements);
			unordered_set<Symbol> eliminatedSymbols;
			for each (auto const& symbol in symbols) {
				if (replacements.find(symbol) == replacements.end()) {
					eliminatedSymbols.insert(symbol);
				}
			}

			unordered_map<Symbol, int> symbolToId;
			for each (auto const& symbol in eliminatedSymbols) {
				symbolToId[symbol] = symbolToId.size();
			}

			decltype(eqConstraints) eqConstraintsR;
			decltype(ineqConstraints) ineqConstraintsR;
			unordered_map<LinearExpr, double> equationsToOptimizeRSet;
			decltype(equationsToOptimize) equationsToOptimizeR;
			for each (auto const& le in eqConstraints) {
				auto replaced = replaceSymbols(le, replacements);
				if (replaced.getCoeffs().empty()) continue;
				eqConstraintsR.push_back(replaced);
			}
			for each (auto const& le in ineqConstraints) {
				auto replaced = replaceSymbols(le, replacements);
				if (replaced.getCoeffs().empty()) continue;
				ineqConstraintsR.push_back(replaced);
			}
			for each (auto const& le in equationsToOptimize) {
				auto replaced = replaceSymbols(le.second, replacements);
				if (replaced.getCoeffs().empty()) continue;
				equationsToOptimizeRSet[replaced] += le.first * le.first;
			}
			for each (auto const& pair in equationsToOptimizeRSet) {
				equationsToOptimizeR.push_back(make_pair(sqrt(pair.second), pair.first));
			}
			PROFILE_END;

#if 0
			using namespace std;
			cout << "Reduced friendly linear system:" << endl;
			for each (auto const& rep in replacements) {
				cout << " Reduced symbol " << rep.first.owner->describeSymbol(rep.first.id) << " to " << rep.second.owner->describeSymbol(rep.second.id) << endl;
			}
			cout << "  Optimize:" << endl;
			for (int i = 0; i < equationsToOptimizeR.size(); i++) {
				const LinearExpr& le = equationsToOptimizeR[i].second;
				double weight = equationsToOptimizeR[i].first;
				string label;
				cout << "    ";
				if (!label.empty()) {
					cout << "[" << label << "] ";
				}
				if (weight != 1.0) {
					cout << "[weight=" << weight << "] ";
				}
				le.print();
				cout << " = 0" << endl;
			}
			cout << "  Subject to equalities:" << endl;

			for (int i = 0; i < eqConstraintsR.size(); i++) {
				const LinearExpr& le = eqConstraintsR[i];
				string label;
				cout << "    ";
				if (!label.empty()) {
					cout << "[" << label << "] ";
				}
				le.print();
				cout << " = 0" << endl;
			}
			cout << "  Subject to inequalities:" << endl;

			for (int i = 0; i < ineqConstraintsR.size(); i++) {
				const LinearExpr& le = ineqConstraintsR[i];
				string label;
				cout << "    ";
				if (!label.empty()) {
					cout << "[" << label << "] ";
				}
				le.print();
				cout << " = 0" << endl;
			}

			cout << "Matrix sizes used to be: " << "eq = " << eqConstraints.size() << " x " << symbols.size() <<
				", ineq = " << ineqConstraints.size() << " x " << symbols.size() <<
				", opt = " << equationsToOptimize.size() << " x " << symbols.size() << endl;


			cout << "New sizes are: " << "eq = " << eqConstraintsR.size() << " x " << symbolToId.size() <<
				", ineq = " << ineqConstraintsR.size() << " x " << symbolToId.size() <<
				", opt = " << equationsToOptimizeR.size() << " x " << symbolToId.size() << endl;
#endif

			PROFILE_BEGIN("Matrix construction in flatten");
			Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(eqConstraintsR.size(), symbolToId.size());
			Eigen::VectorXd Beq = Eigen::VectorXd::Zero(eqConstraintsR.size());
			Eigen::MatrixXd Aineq = Eigen::MatrixXd::Zero(ineqConstraintsR.size(), symbolToId.size());
			Eigen::VectorXd Bineq = Eigen::VectorXd::Zero(ineqConstraintsR.size());
			Eigen::MatrixXd Aopt = Eigen::MatrixXd::Zero(equationsToOptimizeR.size(), symbolToId.size());
			Eigen::VectorXd Bopt = Eigen::VectorXd::Zero(equationsToOptimizeR.size());
			for (int i = 0; i < eqConstraintsR.size(); i++) {
				for each (auto const& pair in eqConstraintsR[i].getCoeffs()) {
					Aeq(i, symbolToId[pair.first]) = pair.second;
				}
				Beq[i] = eqConstraintsR[i].getConstant();
			}
			Beq = -Beq;
			for (int i = 0; i < ineqConstraints.size(); i++) {
				for each (auto const& pair in ineqConstraintsR[i].getCoeffs()) {
					Aineq(i, symbolToId[pair.first]) = pair.second;
				}
				Bineq[i] = ineqConstraintsR[i].getConstant();
			}
			Bineq = -Bineq;
			for (int i = 0; i < equationsToOptimizeR.size(); i++) {
				for each (auto const& pair in equationsToOptimizeR[i].second.getCoeffs()) {
					Aopt(i, symbolToId[pair.first]) = pair.second;
				}
				Bopt[i] = equationsToOptimizeR[i].second.getConstant();
				Aopt.row(i) *= equationsToOptimizeR[i].first;
				Bopt[i] *= equationsToOptimizeR[i].first;
			}
			Bopt = -Bopt;
			Eigen::VectorXd lb, ub;

			auto qAndDim = doCachedQRDecomposition(Aeq);
			Eigen::MatrixXd Q = qAndDim.first;
			int dim = qAndDim.second;
			Eigen::MatrixXd Beq_red = (Q.transpose()*Beq).topRows(dim);
			Eigen::MatrixXd Aeq_red = (Q.transpose()*Aeq).topRows(dim);
			PROFILE_END;

			Eigen::VectorXd q(eliminatedSymbols.size());
			Eigen::VectorXi symbolIndices(symbols.size());
			for (int i = 0; i < symbols.size(); i++) {
				Symbol symbol = symbols[i];
				int index;
				if (replacements.find(symbol) != replacements.end()) {
					index = symbolToId.at(replacements[symbol]);
				}
				else {
					index = symbolToId.at(symbol);
				}
				q[index] = symbol.owner->getCurrentValue(symbol.id);
				symbolIndices[i] = index;
			}

			flat->Aeq_red = Aeq_red;
			flat->Beq_red = Beq_red;
			flat->Aineq = Aineq;
			flat->Bineq = Bineq;
			flat->Aopt = Aopt;
			flat->Bopt = Bopt;
			flat->q = q;
			flat->symbolIndices = symbolIndices;
		} else {
			vector<Symbol> symbols = target->getSymbols(TreeScope::DESCENDANTS);
			int qlen = symbols.size();
			Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(eqConstraints.size(), qlen);
			Eigen::VectorXd Beq = Eigen::VectorXd::Zero(eqConstraints.size());
			Eigen::MatrixXd Aineq = Eigen::MatrixXd::Zero(ineqConstraints.size(), qlen);
			Eigen::VectorXd Bineq = Eigen::VectorXd::Zero(ineqConstraints.size());
			Eigen::MatrixXd Aopt = Eigen::MatrixXd::Zero(equationsToOptimize.size(), qlen);
			Eigen::VectorXd Bopt = Eigen::VectorXd::Zero(equationsToOptimize.size());
			for (int i = 0; i < eqConstraints.size(); i++) {
				flattenSymbolic(symbols, eqConstraints[i], Aeq.row(i), Beq[i]);
			}
			Beq = -Beq;
			for (int i = 0; i < ineqConstraints.size(); i++) {
				flattenSymbolic(symbols, ineqConstraints[i], Aineq.row(i), Bineq[i]);
			}
			Bineq = -Bineq;
			for (int i = 0; i < equationsToOptimize.size(); i++) {
				flattenSymbolic(symbols, equationsToOptimize[i].second, Aopt.row(i), Bopt[i]);
				Aopt.row(i) *= equationsToOptimize[i].first;
				Bopt[i] *= equationsToOptimize[i].first;
			}
			Bopt = -Bopt;
			auto q = target->getFullQ();
			Eigen::VectorXd lb, ub;

			Eigen::FullPivHouseholderQR<Eigen::MatrixXd> qr(Aeq);
			Eigen::MatrixXd Q = qr.matrixQ();
			int dim = qr.rank();
			//cout << "dimention of eq const = " << dim << endl;
			Eigen::MatrixXd Beq_red = (Q.transpose()*Beq).topRows(dim);
			Eigen::MatrixXd Aeq_red = (Q.transpose()*Aeq).topRows(dim);
			Eigen::VectorXi symbolIndices(symbols.size());
			for (int i = 0; i < symbols.size(); i++) {
				symbolIndices[i] = i;
			}

			flat->Aeq_red = Aeq_red;
			flat->Beq_red = Beq_red;
			flat->Aineq = Aineq;
			flat->Bineq = Bineq;
			flat->Aopt = Aopt;
			flat->Bopt = Bopt;
			flat->q = q;
			flat->symbolIndices = symbolIndices;
		}
	}

	void FriendlyLinearSystem::updateFromFlattened(Template* target, FlatLinearSystem const& system) {
		PROFILE_THIS(__FUNCTION__);
		Eigen::VectorXd fullQ(system.symbolIndices.size());
		for (int i = 0; i < system.symbolIndices.size(); i++) {
			fullQ[i] = system.q[system.symbolIndices[i]];
		}
		target->updateFullQ(fullQ);
	}

	pair<Eigen::MatrixXd, int> FriendlyLinearSystem::doCachedQRDecomposition(const Eigen::MatrixXd& matrix) {
		PROFILE_THIS(__FUNCTION__);
		/*if (qrCache.find(matrix) != qrCache.end()) {
			cout << "Cache hit!" << endl;
			return qrCache.at(matrix);
		}*/   //disable cache 
		Eigen::FullPivHouseholderQR<Eigen::MatrixXd> qr(matrix);
		Eigen::MatrixXd Q = qr.matrixQ();
		int dim = qr.rank();
		//qrCache[matrix] = make_pair(Q, dim);
		// cout << "Cache miss!" << endl;
		//return qrCache.at(matrix);
		return make_pair(Q, dim);
	}

	void FriendlyLinearSystem::print() {
		using namespace std;
		cout << "Friendly linear system of:" << endl;
		cout << "  Optimize:" << endl;
		for (int i = 0; i < equationsToOptimize.size(); i++) {
			const LinearExpr& le = equationsToOptimize[i].second;
			double weight = equationsToOptimize[i].first;
			string label = etoLabel[i];
			cout << "    ";
			if (!label.empty()) {
				cout << "[" << label << "] ";
			}
			if (weight != 1.0) {
				cout << "[weight=" << weight << "] ";
			}
			le.print();
			cout << " = 0" << endl;
		}
		cout << "  Subject to equalities:" << endl;

		for (int i = 0; i < eqConstraints.size(); i++) {
			const LinearExpr& le = eqConstraints[i];
			string label = eqcLabel[i];
			cout << "    ";
			if (!label.empty()) {
				cout << "[" << label << "] ";
			}
			le.print();
			cout << " = 0" << endl;
		}
		cout << "  Subject to inequalities:" << endl;

		for (int i = 0; i < ineqConstraints.size(); i++) {
			const LinearExpr& le = ineqConstraints[i];
			string label = ineqcLabel[i];
			cout << "    ";
			if (!label.empty()) {
				cout << "[" << label << "] ";
			}
			le.print();
			cout << " = 0" << endl;
		}
	}
}