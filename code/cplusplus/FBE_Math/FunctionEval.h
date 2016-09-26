#ifndef __FunctionEVal_H__
#define __FunctionEVal_H__

#include <Eigen/Dense>


class FunctionEval {

 public:
	FunctionEval(){}
	virtual double eval(double x) =0;
 	
 };
 

class TestFunctionEval: public FunctionEval{
	public:
	TestFunctionEval(){}
	double eval(double x){
		return  -5*pow(x,5) + 4*pow(x,4) - 12*pow(x,3) + 11*pow(x,2) - 2*x + 1;
	}

};


 





#endif // __MyOptimizer_H__