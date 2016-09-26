#ifndef __MyOptimizer_H__
#define __MyOptimizer_H__

#include <Eigen/Dense>
#include "FunctionEval.h"

class MyOptimizer {

 public:
	MyOptimizer(){}

 	static int linOpt( const Eigen::VectorXd & f, 
		const Eigen::MatrixXd & A, Eigen::VectorXd & b, 
		const Eigen::MatrixXd & Aeq, const Eigen::VectorXd & beq, 
		const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, 
		bool isMin,
		Eigen::VectorXd & sol, double* objval);

	static int quadOpt( const Eigen::MatrixXd & H, const Eigen::VectorXd & f, 
		const Eigen::MatrixXd & A, Eigen::VectorXd & b, 
		const Eigen::MatrixXd & Aeq, const Eigen::VectorXd & beq, 
		const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, 
		bool isMin,
		Eigen::VectorXd & sol, double* objval);

	static int simpleQuadOpt( const Eigen::MatrixXd & H, const Eigen::VectorXd & f, 
		const Eigen::MatrixXd & A, Eigen::VectorXd & b, 
		const Eigen::MatrixXd & Aeq, const Eigen::VectorXd & beq, 
		Eigen::VectorXd & sol);

	// solve min|| Ax - b||2 st equality and inequality contraints
	static int leastSquaresWithConstraints( const Eigen::MatrixXd & A,  const Eigen::VectorXd & b, 
											const Eigen::MatrixXd & A_ineq, Eigen::VectorXd & b_inq, 
											const Eigen::MatrixXd & Aeq, const Eigen::VectorXd & beq, 
											const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, 
											Eigen::VectorXd & sol);


	static void lineSearch(FunctionEval * f, double xu, 
		double xl, double delta, double * resultVal, double * sol);
 };
 






#endif // __MyOptimizer_H__