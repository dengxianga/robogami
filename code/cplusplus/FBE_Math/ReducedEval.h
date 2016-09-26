#ifndef _REDUCED_EVAl_
#define _REDUCED_EVAl_

#include <Eigen/Dense>
#include <vector>

namespace FabByExample{

class EigenHelper{
public:
	static std::vector<double> eigen2std(const Eigen::VectorXd & v){
		std::vector<double>  result;
		for (int i =0; i <  v.size(); i++){
			result.push_back(v(i));
		}
		return result;
	}
	static Eigen::VectorXd std2eigen(const std::vector<double> & v){
		Eigen::VectorXd  result = Eigen::VectorXd::Zero(v.size());
		for (int i =0; i <  v.size(); i++){
			result(i) = v[i];
		}
		return result;
	}
};




class ReducedEval{
public:
	ReducedEval(Eigen::MatrixXd & A_eq, Eigen::VectorXd & B_eq, 
		Eigen::MatrixXd & A_ineq, Eigen::VectorXd & B_ineq, Eigen::VectorXd & q_orig);
	
	bool evalFeasibility(const Eigen::VectorXd & t);
	bool getQ(const Eigen::VectorXd & t, Eigen::VectorXd & q);
	void getT(const Eigen::VectorXd & q, Eigen::VectorXd & t);
	void getClosestFeasiblePoint(const Eigen::VectorXd & origt, Eigen::VectorXd & newt);
	void getEquationFromQ2T(const Eigen::VectorXd & Aq, const double & Bq, Eigen::VectorXd & At, double & Bt);
	void getMatEquationFromQ2T(const Eigen::MatrixXd & Aq, const Eigen::VectorXd & Bq, Eigen::MatrixXd & At, Eigen::VectorXd & Bt);

	void getNewQ(Eigen::MatrixXd & Mat_S, Eigen::VectorXd & Vec_S, Eigen::VectorXd & newQ);

	void getFeasibleRange(const Eigen::VectorXd & origt,const  Eigen::VectorXd & dir, double * minRange, double * maxRange);


	Eigen::MatrixXd Vt; 
	Eigen::MatrixXd C;
	Eigen::VectorXd D;
	Eigen::VectorXd q_0;
	int sizeOfT;

};

}
	
#endif