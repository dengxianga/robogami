#include "ReducedEval.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include "myOptimizer.h"


using namespace FabByExample;


ReducedEval::ReducedEval(Eigen::MatrixXd & A_eq, Eigen::VectorXd & B_eq, Eigen::MatrixXd & A_ineq, Eigen::VectorXd & B_ineq, Eigen::VectorXd & q_orig){


	// we have already done qr in A and therefore:
	int n_var = A_eq.cols();
	Eigen::JacobiSVD<Eigen::MatrixXd> svdOfA(A_eq, Eigen::ComputeFullV );
	Eigen::MatrixXd V = svdOfA.matrixV();
	//std::cout << "V = (" << V.rows() << " , " << V.cols() << " ) " << std::endl;
	
	//system("pause");
	//std::cout << V.transpose()*V << std::endl;
	//system("pause");
	//std::cout << V*V.transpose() << std::endl;
	//system("pause");

	Eigen::VectorXd S = svdOfA.singularValues();	
	//std::cout << "S " << S.transpose() << std::endl;
	int zerosInS = 0;
	for( int i = 0; i < S.size(); i++){
		if(S(i)< 0.000001){
			zerosInS++;
		}
	}
	int n_const = S.size() - zerosInS;
	int n_f = n_var - n_const;

	//std::cout << "n_var = " << n_var << std::endl;
	//std::cout << "nf = " << n_f << std::endl;
	// we get that q = q_0 + Vt * t
	Eigen::MatrixXd V0 = V.leftCols(n_var- n_f);
	Vt = V.rightCols(n_f);
	//std::cout << "V = (" << V.rows() << " , " << V.cols() << " ) " << std::endl;
	//std::cout << "V0 = (" << V0.rows() << " , " << V0.cols() << " ) " << std::endl;
	//std::cout << "Vt = (" << Vt.rows() << " , " << Vt.cols() << " ) " << std::endl;

	//std::cout << V.transpose()*V << std::endl;

	Eigen::VectorXd z_orig = (V.transpose())*q_orig;
	Eigen::VectorXd z0 = z_orig.head(n_var- n_f);
	Eigen::VectorXd t_org = z_orig.tail(n_f);
	//std::cout << "t_orig = "  << t_org.transpose() << std::endl;
	q_0 = V0*z0;
	
	//std::cout << "error1  = " << (V*z_orig - q_orig).norm() << std::endl; 
	//std::cout << "error = " << ((q_0 + Vt*t_org) - q_orig).norm() << std::endl; 

	std::cout << "the size of A_ineq is " << A_ineq.rows() << " " << A_ineq.cols() << std::endl;
	std::cout << "the size of B_ineq is " << B_ineq.size() << std::endl;
	std::cout << "the size of Vt is " << Vt.rows() << " " << Vt.cols() << std::endl;

	// we get that C*t <= D
	C = A_ineq*Vt;
	D = B_ineq - A_ineq * q_0;
	
	//std::cout << "nvar  = " << n_var << std::endl;
	//std::cout << "n_f = " << n_f << std::endl;
	//std::cout << "A_eq = (" << A_eq.rows() << " , " << A_eq.cols() << " ) " << std::endl;
	//std::cout << "V = (" << V.rows() << " , " << V.cols() << " ) " << std::endl;
	//std::cout << "V0 = (" << V0.rows() << " , " << V0.cols() << " ) " << std::endl;
	//std::cout << "Vt = (" << Vt.rows() << " , " << Vt.cols() << " ) " << std::endl;
	//std::cout << "C = (" << C.rows() << " , " << C.cols() << " ) " << std::endl;
	//std::cout << "D = (" << D.rows() << " , " << D.cols() << " ) " << std::endl;


	sizeOfT = n_f;

	//system("pause"); 

}
bool ReducedEval::evalFeasibility(const Eigen::VectorXd & t){
	double maxC = (C*t - D).maxCoeff();
	//std::cout << "MaxC = " << maxC << std::endl;
	return ( maxC <= 0.00001);
}

bool ReducedEval::getQ(const Eigen::VectorXd & t, Eigen::VectorXd & q){
	q = q_0 + Vt*t;
		
	if (evalFeasibility(t)){
		return true;
	}
	
	return false;
}


void ReducedEval::getEquationFromQ2T(const Eigen::VectorXd & Aq, const double & Bq, Eigen::VectorXd & At, double & Bt){
	// q = q_0 + Vt*t;
	// Aq*(q) + Bq = Aq*(q_0 + Vt*t) + Bq
	// Aq*q_0 + Aq*Vt*t + Bq
	// Aq*Vt*t + Aq*q_0 + Bq

	At = Aq.transpose()*Vt;
	Bt = Aq.dot(q_0) + Bq;

}


void ReducedEval::getMatEquationFromQ2T(const Eigen::MatrixXd & Aq, const Eigen::VectorXd & Bq, Eigen::MatrixXd & At, Eigen::VectorXd & Bt){
	// q = q_0 + Vt*t;
	// Aq*(q) + Bq = Aq*(q_0 + Vt*t) + Bq
	// Aq*q_0 + Aq*Vt*t + Bq
	// Aq*Vt*t + Aq*q_0 + Bq

	At = Aq*Vt;
	Bt = Aq*q_0 + Bq;

}



void ReducedEval::getT(const Eigen::VectorXd & q, Eigen::VectorXd & t){
	t = Vt.transpose()*q;

}


void ReducedEval::getNewQ(Eigen::MatrixXd & Mat_S, Eigen::VectorXd & Vec_S, Eigen::VectorXd & newQ){

	// argmin_q ||Mat_S * q - Vec_S ||

	std::cout << "Mat_S = (" << Mat_S.rows() << " , " << Mat_S.cols() << " ) " << std::endl;
	std::cout << "Vt = (" << Vt.rows() << " , " << Vt.cols() << " ) " << std::endl;
	// optimize for t
	Eigen::MatrixXd A = Mat_S*Vt;
	std::cout << "A = (" << A.rows() << " , " << A.cols() << " ) " << std::endl;
	std::cout << "Vec_S = " << Vec_S.rows()  << std::endl;
	std::cout << "q_0 = " << q_0.rows()  << std::endl;
	Eigen::VectorXd b = Vec_S - Mat_S*q_0;
	std::cout << "b = " << b.rows()  << std::endl;
	Eigen::VectorXd newt;
	Eigen::MatrixXd Aeq;
	Eigen::VectorXd beq, lu, gu;
	MyOptimizer::leastSquaresWithConstraints(A, b , C, D, Aeq, beq, lu, gu, newt);

	std::cout << "newt = " << newt.rows()  << std::endl;
	// compute newQ
	newQ = q_0  + Vt*newt;


}

void ReducedEval::getClosestFeasiblePoint(const Eigen::VectorXd & origt, Eigen::VectorXd & newt){


	Eigen::MatrixXd A = Eigen::MatrixXd::Identity(origt.size(),origt.size());
	Eigen::MatrixXd Aeq;
	Eigen::VectorXd beq, lu, gu;
	MyOptimizer::leastSquaresWithConstraints(A, origt , C, D, Aeq, beq, lu, gu, newt);


}

void ReducedEval::getFeasibleRange(const Eigen::VectorXd & origt,const  Eigen::VectorXd & dir, double * minRange, double * maxRange){

	// C(origT + alpha*dir) <= D
	// A*alpha < = B
	// B = D - C*origT
	// A = C*dir




	Eigen::VectorXd A = C*dir;
	Eigen::VectorXd B = D - C*origt;

	//std::cout << "A =" << A.transpose() << std::endl;
	//std::cout << "B =" << B.transpose() << std::endl;
	//std::cout << "D =" << D.transpose() << std::endl;
	//std::cout << "C*origt=" << (C*origt).transpose() << std::endl;
	//std::cout << "origt =" << origt.transpose() << std::endl;

	double minAlpha = -1*std::numeric_limits<double>::max();
	double maxAlpha = std::numeric_limits<double>::max();

	for(int i = 0; i < A.size() ; i++){
		//std::cout << A(i)<< "x <= " << B(i) << std::endl;

		if(A(i) > 0){
			double maxVal = B(i)/A(i);
			//std::cout << "maxVal = " << maxVal << std::endl;
			if (maxAlpha > maxVal){
				maxAlpha = maxVal;
			}
		}else if(A(i) < 0){
			double minVal = B(i)/A(i);
			//std::cout << "minVal = " << minVal << std::endl;
			if (minAlpha < minVal){
				minAlpha = minVal;
			}

		}

	}

	//std::cout << "final minAlpha = " << minAlpha << std::endl;
	//std::cout << "final maxAlpha = " << maxAlpha << std::endl;

	if(maxAlpha == std::numeric_limits<double>::max()){
		maxAlpha = 5000;
	}
	if(minAlpha == std::numeric_limits<double>::min()){
		minAlpha = -5000;
	}

	*minRange = minAlpha;
	*maxRange = maxAlpha;

}


