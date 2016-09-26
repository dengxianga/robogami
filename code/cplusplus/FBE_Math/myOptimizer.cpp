
#include <limits>
#include "myOptimizer.h"
#include <ilcplex/ilocplex.h>
#include <stdio.h>
#include "debugging.h"

// solve min|| Ax - b||2 st equality and inequality contraints
int MyOptimizer::leastSquaresWithConstraints( const Eigen::MatrixXd & A,  const Eigen::VectorXd & b, 
											const Eigen::MatrixXd & A_ineq, Eigen::VectorXd & b_ineq, 
											const Eigen::MatrixXd & A_eq, const Eigen::VectorXd & b_eq, 
											const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, 
											Eigen::VectorXd & sol)
{
	PROFILE_THIS(__FUNCTION__);
	Eigen::MatrixXd H = A.transpose()*A;
	Eigen::VectorXd f = -b.transpose()*A;


//	std::cout << "H = \n" << H << std::endl;
//	std::cout << "f = \n" << f << std::endl;

//	std::cout << "A_ineq = \n" << A_ineq<< std::endl;
//	std::cout << "b_ineq = \n" << b_ineq << std::endl;

//	std::cout << "A_eq = \n" << A_eq<< std::endl;
//	std::cout << "b_eq = \n" << b_eq << std::endl;
	
	
//	std::cout << "lb = \n" << lb << std::endl;
//	std::cout << "ub = \n" << ub << std::endl;
	
//	system("pause"); 


	double objval = 0;
	int status = quadOpt( H,  f, A_ineq, b_ineq, A_eq, b_eq, lb, ub, true, sol, &objval);
	
	return status;
}


int MyOptimizer::quadOpt( const Eigen::MatrixXd & H, const Eigen::VectorXd & f, 
	const Eigen::MatrixXd & A, Eigen::VectorXd & b, 
	const Eigen::MatrixXd & Aeq, const Eigen::VectorXd & beq, 
	const Eigen::VectorXd & _lb, const Eigen::VectorXd & _ub, 
		bool isMin,
		Eigen::VectorXd & sol, double* _objval)
{

	int nVar = f.rows();
	int nConstr = A.rows();
	int nConstrEq = Aeq.rows();

	
   int      solstat;
   double   objval;
   double   *x = NULL;
   double   *pi = NULL;
   double   *slack = NULL;
   double   *dj = NULL;
   CPXENVptr     env = NULL;
   CPXLPptr      lp = NULL;
   int           status;
   int           i, j;
   int           cur_numrows, cur_numcols;

   /* Initialize the CPLEX environment */
   env = CPXopenCPLEX (&status);
   if ( env == NULL ) {
   char  errmsg[CPXMESSAGEBUFSIZE];
      fprintf (stderr, "Could not open CPLEX environment.\n");
      CPXgeterrorstring (env, status, errmsg);
      fprintf (stderr, "%s", errmsg);
	  return (status);
   }

   lp = CPXcreateprob (env, &status, "Dada");
   if ( lp == NULL ) { fprintf (stderr, "Failed to create problem.\n"); return (status);}

   
   // Optimize
   	double* obj = (double *) malloc (nVar * sizeof(double));
	double* lb = (double *) malloc (nVar * sizeof(double));
	double* ub = (double *) malloc (nVar * sizeof(double));

	//set objective
	if(isMin)
		CPXchgobjsen (env, lp, CPX_MIN);  
	else
		CPXchgobjsen (env, lp, CPX_MAX);  
	for(int i= 0; i<nVar; i++){ 
		obj[i] = f(i);
		if(_lb.size() == 0)
			lb[i] = -CPX_INFBOUND;
		else
			lb[i] = _lb(i);
		if(_ub.size() == 0)
			ub[i] = CPX_INFBOUND;
		else
			ub[i] = _ub(i);
		
	}
	status = CPXnewcols (env, lp, nVar, obj, lb, ub, NULL, NULL);
	if ( status )  { std::cout << "Cplex error 2" << std::endl; return (status);}
	int NUMQNZ = nVar*nVar;
	int *zqmatbeg  = (int *) malloc (nVar * sizeof(int));
	int *zqmatcnt  = (int *) malloc (nVar * sizeof(int));
	int *zqmatind  = (int *) malloc (NUMQNZ * sizeof(int));
	double *zqmatval  = (double *) malloc (NUMQNZ * sizeof(double));
	int index = 0;
	for(int i= 0; i<nVar; i++){ 
		zqmatbeg[i] = index;
		zqmatcnt[i] = nVar; 
		for(int j= 0; j<nVar; j++){ 
			zqmatind[index] = j;  			
			zqmatval[index] = H(i,j);
			index++;
		}
	}
	status = CPXcopyquad (env, lp, zqmatbeg, zqmatcnt, zqmatind, zqmatval);
	if ( status ) { fprintf (stderr, "Failed to copy quadratic matrix.\n"); return (status);}
	free(zqmatbeg);
	free(zqmatcnt);
	free(zqmatind);
	free(zqmatval);


	if(nConstr + nConstrEq > 0){
		//Set linear constraints

		int NUMROWS = nConstr + nConstrEq;
		int NUMCOLS  = nVar;
		int NUMNZ = NUMCOLS*NUMROWS;
		int* rowlist = (int *) malloc (NUMNZ * sizeof(double));
		int* collist = (int *) malloc (NUMNZ * sizeof(double));
		double* vallist = (double *) malloc (NUMNZ * sizeof(double));
		double* rhs = (double *) malloc (NUMROWS * sizeof(double));
		char*    sense = (char *) malloc (NUMROWS * sizeof(char));
		char** rowname = (char**) malloc (NUMROWS * sizeof(char*));


	   /* Adding Template Constraints  */
	   index =0;
	   for(int i= 0; i<nConstr; i++){ 
			sense[i] = 'L'; 
			rhs[i]   = b(i);
			for(int j= 0; j<NUMCOLS; j++){
				rowlist[index] = i;   
				collist[index] = j;  
				vallist[index] = A(i,j);
				index++;
			}
	   }
	   for(int i=nConstr; i<NUMROWS; i++){ 
			sense[i] = 'E'; 
			rhs[i]   = beq(i - nConstr) + 0.0001;
			for(int j= 0; j<NUMCOLS; j++){
				rowlist[index] = i;   
				collist[index] = j;  
				vallist[index] = Aeq(i - nConstr,j);
				index++;
			}
	   }
	   status = CPXnewrows (env, lp, NUMROWS, rhs, sense, NULL, NULL);
	   if ( status ) { std::cout << "Cplex error 6" << std::endl;  return (status);}
	   status = CPXchgcoeflist (env, lp, NUMNZ, rowlist, collist, vallist);
	   if ( status ) { std::cout << "Cplex error 7" << std::endl; return (status);}

		free(rhs) ;
		free(sense);
		free(rowname);
		free(rowlist); 
		free(collist); 
		free(vallist);

	}
	// Optimize
	status = CPXqpopt (env, lp);
	if ( status ) {fprintf (stderr, "Failed to optimize QP.\n"); return (status);}

	free(obj );
	free(lb );
	free(ub );

   //get solution

   	cur_numrows = CPXgetnumrows (env, lp);
	cur_numcols = CPXgetnumcols (env, lp);

	x = (double *) malloc (cur_numcols * sizeof(double));
	slack = (double *) malloc (cur_numrows * sizeof(double));
	dj = (double *) malloc (cur_numcols * sizeof(double));
	pi = (double *) malloc (cur_numrows * sizeof(double));

	if ( x     == NULL ||
		slack == NULL ||
		dj    == NULL ||
		pi    == NULL   ) {
		status = CPXERR_NO_MEMORY;
		fprintf (stderr, "Could not allocate memory for solution.\n");
		return (status);
	}

	
   status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
   if ( status ) {
	   std::cout << "status = " << status << std::endl; 
       fprintf (stderr, "Failed to obtain solution.\n");
	   if ( lp != NULL ) {
		  if ( CPXfreeprob (env, &lp) ) {
			 fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status);
		  }
	   }
	   if ( env != NULL ) {
		  if ( CPXcloseCPLEX (&env) ) {
			 char  errmsg[CPXMESSAGEBUFSIZE];
			 fprintf (stderr, "Could not close CPLEX environment.\n");
			 CPXgeterrorstring (env, status, errmsg);
			 fprintf (stderr, "%s", errmsg);
		  }
	   }

	   free(x);
	   free(slack);
	   free(dj);
	   free(pi);
	   return (status);
   }


   // save output

 //  printf ("\nSolution status = %d\n", solstat);
 //  printf ("Solution value  = %f\n\n", objval);

 //   for(int i = 0; i< nVar; i++){
	//	printf ("x[%d]  = %f\n", i, x[i]);
	//}
	sol = Eigen::VectorXd::Zero(nVar);
    for(int i = 0; i< nVar; i++){
		sol(i) = x[i];
	}

   // Free things

   if ( lp != NULL ) {
      status = CPXfreeprob (env, &lp);
      if ( status ) {
         fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status);
		 return (status);
      }
   }
   if ( env != NULL ) {
      status = CPXcloseCPLEX (&env);
	  if ( status ) {
         char  errmsg[CPXMESSAGEBUFSIZE];
         fprintf (stderr, "Could not close CPLEX environment.\n");
         CPXgeterrorstring (env, status, errmsg);
         fprintf (stderr, "%s", errmsg);
		 return (status);
      }
   }



   free(x);
   free(slack);
   free(dj);
   free(pi);


	*_objval = objval;


	return (status);

}


int MyOptimizer::simpleQuadOpt( const Eigen::MatrixXd & H, const Eigen::VectorXd & f, 
	const Eigen::MatrixXd & A, Eigen::VectorXd & b, 
	const Eigen::MatrixXd & Aeq, const Eigen::VectorXd & beq, 
	Eigen::VectorXd & sol){


	int nVar = f.rows();
	int nConstr = A.rows();
	int nConstrEq = Aeq.rows();

	
   int      solstat;
   double   objval;
   double   *x = NULL;
   double   *pi = NULL;
   double   *slack = NULL;
   double   *dj = NULL;
   CPXENVptr     env = NULL;
   CPXLPptr      lp = NULL;
   int           status;
   int           i, j;
   int           cur_numrows, cur_numcols;

   /* Initialize the CPLEX environment */
   env = CPXopenCPLEX (&status);
   if ( env == NULL ) {
   char  errmsg[CPXMESSAGEBUFSIZE];
      fprintf (stderr, "Could not open CPLEX environment.\n");
      CPXgeterrorstring (env, status, errmsg);
      fprintf (stderr, "%s", errmsg);
	  return (status);
   }

   lp = CPXcreateprob (env, &status, "Dada");
   if ( lp == NULL ) { fprintf (stderr, "Failed to create problem.\n"); return (status);}

   
   // Optimize
   	double* obj = (double *) malloc (nVar * sizeof(double));
	double* lb = (double *) malloc (nVar * sizeof(double));
	double* ub = (double *) malloc (nVar * sizeof(double));

	//set objective
	CPXchgobjsen (env, lp, CPX_MIN);  
	for(int i= 0; i<nVar; i++){ 
		obj[i] = f(i);
		lb[i] = -CPX_INFBOUND;
		ub[i] = CPX_INFBOUND;
	}
	status = CPXnewcols (env, lp, nVar, obj, lb, ub, NULL, NULL);
	if ( status )  { std::cout << "Cplex error 2" << std::endl; return (status);}
	int NUMQNZ = nVar*nVar;
	int *zqmatbeg  = (int *) malloc (nVar * sizeof(int));
	int *zqmatcnt  = (int *) malloc (nVar * sizeof(int));
	int *zqmatind  = (int *) malloc (NUMQNZ * sizeof(int));
	double *zqmatval  = (double *) malloc (NUMQNZ * sizeof(double));
	int index = 0;
	for(int i= 0; i<nVar; i++){ 
		zqmatbeg[i] = index;
		zqmatcnt[i] = nVar; 
		for(int j= 0; j<nVar; j++){ 
			zqmatind[index] = j;  			
			zqmatval[index] = H(i,j);
			index++;
		}
	}
	status = CPXcopyquad (env, lp, zqmatbeg, zqmatcnt, zqmatind, zqmatval);
	if ( status ) { fprintf (stderr, "Failed to copy quadratic matrix.\n"); return (status);}
	free(zqmatbeg);
	free(zqmatcnt);
	free(zqmatind);
	free(zqmatval);
	//Set linear constraints

	int NUMROWS = nConstr + nConstrEq;
	int NUMCOLS  = nVar;
	int NUMNZ = NUMCOLS*NUMROWS;
	int* rowlist = (int *) malloc (NUMNZ * sizeof(double));
	int* collist = (int *) malloc (NUMNZ * sizeof(double));
	double* vallist = (double *) malloc (NUMNZ * sizeof(double));
	double* rhs = (double *) malloc (NUMROWS * sizeof(double));
	char*    sense = (char *) malloc (NUMROWS * sizeof(char));
	char** rowname = (char**) malloc (NUMROWS * sizeof(char*));


   /* Adding Template Constraints  */
   index =0;
   for(int i= 0; i<nConstr; i++){ 
		sense[i] = 'L'; 
		rhs[i]   = b(i);
	    for(int j= 0; j<NUMCOLS; j++){
			rowlist[index] = i;   
			collist[index] = j;  
			vallist[index] = A(i,j);
			index++;
		}
   }
   for(int i=nConstr; i<NUMROWS; i++){ 
		sense[i] = 'E'; 
		rhs[i]   = beq(i - nConstr) + 0.0001;
	    for(int j= 0; j<NUMCOLS; j++){
			rowlist[index] = i;   
			collist[index] = j;  
			vallist[index] = Aeq(i - nConstr,j);
			index++;
		}
   }
   status = CPXnewrows (env, lp, NUMROWS, rhs, sense, NULL, NULL);
   if ( status ) { std::cout << "Cplex error 6" << std::endl;  return (status);}
   status = CPXchgcoeflist (env, lp, NUMNZ, rowlist, collist, vallist);
   if ( status ) { std::cout << "Cplex error 7" << std::endl;  return (status);}

	free(rhs) ;
	free(sense);
	free(rowname);
	free(rowlist); 
	free(collist); 
	free(vallist);


	// Optimize
	status = CPXqpopt (env, lp);
	if ( status ) {fprintf (stderr, "Failed to optimize QP.\n"); return (status);}

	free(obj );
	free(lb );
	free(ub );

   //get solution

   	cur_numrows = CPXgetnumrows (env, lp);
	cur_numcols = CPXgetnumcols (env, lp);

	x = (double *) malloc (cur_numcols * sizeof(double));
	slack = (double *) malloc (cur_numrows * sizeof(double));
	dj = (double *) malloc (cur_numcols * sizeof(double));
	pi = (double *) malloc (cur_numrows * sizeof(double));

	if ( x     == NULL ||
		slack == NULL ||
		dj    == NULL ||
		pi    == NULL   ) {
		status = CPXERR_NO_MEMORY;
		fprintf (stderr, "Could not allocate memory for solution.\n");
		return (NULL);
	}

	
   status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
   if ( status ) {
       fprintf (stderr, "Failed to obtain solution.\n");
	   if ( lp != NULL ) {
		  if ( CPXfreeprob (env, &lp) ) {
			 fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status);
		  }
	   }
	   if ( env != NULL ) {
		  if ( CPXcloseCPLEX (&env) ) {
			 char  errmsg[CPXMESSAGEBUFSIZE];
			 fprintf (stderr, "Could not close CPLEX environment.\n");
			 CPXgeterrorstring (env, status, errmsg);
			 fprintf (stderr, "%s", errmsg);
		  }
	   }

	   free(x);
	   free(slack);
	   free(dj);
	   free(pi);
	   return (status);
   }


   // save output

 //  printf ("\nSolution status = %d\n", solstat);
 //  printf ("Solution value  = %f\n\n", objval);

 //   for(int i = 0; i< nVar; i++){
	//	printf ("x[%d]  = %f\n", i, x[i]);
	//}
	sol = Eigen::VectorXd::Zero(nVar);
    for(int i = 0; i< nVar; i++){
		sol(i) = x[i];
	}

   // Free things

   if ( lp != NULL ) {
      status = CPXfreeprob (env, &lp);
      if ( status ) {
         fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status);
		 return (status);
      }
   }
   if ( env != NULL ) {
      status = CPXcloseCPLEX (&env);
	  if ( status ) {
         char  errmsg[CPXMESSAGEBUFSIZE];
         fprintf (stderr, "Could not close CPLEX environment.\n");
         CPXgeterrorstring (env, status, errmsg);
         fprintf (stderr, "%s", errmsg);
		 return (status);
      }
   }



   free(x);
   free(slack);
   free(dj);
   free(pi);


	


	return (status);

}



int MyOptimizer::linOpt( const Eigen::VectorXd & f, 
	const Eigen::MatrixXd & A, Eigen::VectorXd & b, 
	const Eigen::MatrixXd & Aeq, const Eigen::VectorXd & beq, 
	const Eigen::VectorXd & _lb, const Eigen::VectorXd & _ub, 
	bool isMin,
	Eigen::VectorXd & sol, double* _objval)
{

	int nVar = f.rows();
	int nConstr = A.rows();
	int nConstrEq = Aeq.rows();

	
   int      solstat;
   double   objval;
   double   *x = NULL;
   double   *pi = NULL;
   double   *slack = NULL;
   double   *dj = NULL;
   CPXENVptr     env = NULL;
   CPXLPptr      lp = NULL;
   int           status;
   int           i, j;
   int           cur_numrows, cur_numcols;

   /* Initialize the CPLEX environment */
   env = CPXopenCPLEX (&status);
   if ( env == NULL ) {
   char  errmsg[CPXMESSAGEBUFSIZE];
      fprintf (stderr, "Could not open CPLEX environment.\n");
      CPXgeterrorstring (env, status, errmsg);
      fprintf (stderr, "%s", errmsg);
	  return (status);
   }

   lp = CPXcreateprob (env, &status, "Dada");
   if ( lp == NULL ) { fprintf (stderr, "Failed to create problem.\n"); return (status);}

   
   // Optimize
   	double* obj = (double *) malloc (nVar * sizeof(double));
	double* lb = (double *) malloc (nVar * sizeof(double));
	double* ub = (double *) malloc (nVar * sizeof(double));

	//set objective
	if(isMin)
		CPXchgobjsen (env, lp, CPX_MIN);  
	else
		CPXchgobjsen (env, lp, CPX_MAX);  
	for(int i= 0; i<nVar; i++){ 
		obj[i] = f(i);
		if(_lb.size() == 0)
			lb[i] = -CPX_INFBOUND;
		else
			lb[i] = _lb(i);
		if(_ub.size() == 0)
			ub[i] = CPX_INFBOUND;
		else
			ub[i] = _ub(i);
		
	}
	status = CPXnewcols (env, lp, nVar, obj, lb, ub, NULL, NULL);
	

	if(nConstr + nConstrEq > 0){
		//Set linear constraints

		int NUMROWS = nConstr + nConstrEq;
		int NUMCOLS  = nVar;
		int NUMNZ = NUMCOLS*NUMROWS;
		int* rowlist = (int *) malloc (NUMNZ * sizeof(double));
		int* collist = (int *) malloc (NUMNZ * sizeof(double));
		double* vallist = (double *) malloc (NUMNZ * sizeof(double));
		double* rhs = (double *) malloc (NUMROWS * sizeof(double));
		char*    sense = (char *) malloc (NUMROWS * sizeof(char));
		char** rowname = (char**) malloc (NUMROWS * sizeof(char*));


	   /* Adding Template Constraints  */
	   int index =0;
	   for(int i= 0; i<nConstr; i++){ 
			sense[i] = 'L'; 
			rhs[i]   = b(i);
			for(int j= 0; j<NUMCOLS; j++){
				rowlist[index] = i;   
				collist[index] = j;  
				vallist[index] = A(i,j);
				index++;
			}
	   }
	   for(int i=nConstr; i<NUMROWS; i++){ 
			sense[i] = 'E'; 
			rhs[i]   = beq(i - nConstr) + 0.0001;
			for(int j= 0; j<NUMCOLS; j++){
				rowlist[index] = i;   
				collist[index] = j;  
				vallist[index] = Aeq(i - nConstr,j);
				index++;
			}
	   }
	   status = CPXnewrows (env, lp, NUMROWS, rhs, sense, NULL, NULL);
	   if ( status ) { std::cout << "Cplex error 6" << std::endl;  return (status);}
	   status = CPXchgcoeflist (env, lp, NUMNZ, rowlist, collist, vallist);
	   if ( status ) { std::cout << "Cplex error 7" << std::endl;  return (status);}

		free(rhs) ;
		free(sense);
		free(rowname);
		free(rowlist); 
		free(collist); 
		free(vallist);

	}
	// Optimize
	status = CPXlpopt (env, lp);
	if ( status ) {fprintf (stderr, "Failed to optimize LP.\n"); return (status);}

	free(obj );
	free(lb );
	free(ub );

   //get solution

   	cur_numrows = CPXgetnumrows (env, lp);
	cur_numcols = CPXgetnumcols (env, lp);

	x = (double *) malloc (cur_numcols * sizeof(double));
	slack = (double *) malloc (cur_numrows * sizeof(double));
	dj = (double *) malloc (cur_numcols * sizeof(double));
	pi = (double *) malloc (cur_numrows * sizeof(double));

	if ( x     == NULL ||
		slack == NULL ||
		dj    == NULL ||
		pi    == NULL   ) {
		status = CPXERR_NO_MEMORY;
		fprintf (stderr, "Could not allocate memory for solution.\n");
		return (NULL);
	}

	
   status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
   if ( status ) {
       fprintf (stderr, "Failed to obtain solution.\n");
	   if ( lp != NULL ) {
		  if ( CPXfreeprob (env, &lp) ) {
			 fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status);
		  }
	   }
	   if ( env != NULL ) {
		  if ( CPXcloseCPLEX (&env) ) {
			 char  errmsg[CPXMESSAGEBUFSIZE];
			 fprintf (stderr, "Could not close CPLEX environment.\n");
			 CPXgeterrorstring (env, status, errmsg);
			 fprintf (stderr, "%s", errmsg);
		  }
	   }

	   free(x);
	   free(slack);
	   free(dj);
	   free(pi);
	   return (status);
   }


   // save output

 //  printf ("\nSolution status = %d\n", solstat);
 //  printf ("Solution value  = %f\n\n", objval);

 //   for(int i = 0; i< nVar; i++){
	//	printf ("x[%d]  = %f\n", i, x[i]);
	//}
	sol = Eigen::VectorXd::Zero(nVar);
    for(int i = 0; i< nVar; i++){
		sol(i) = x[i];
	}

   // Free things

   if ( lp != NULL ) {
      status = CPXfreeprob (env, &lp);
      if ( status ) {
         fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status);
		 return (status);
      }
   }
   if ( env != NULL ) {
      status = CPXcloseCPLEX (&env);
	  if ( status ) {
         char  errmsg[CPXMESSAGEBUFSIZE];
         fprintf (stderr, "Could not close CPLEX environment.\n");
         CPXgeterrorstring (env, status, errmsg);
         fprintf (stderr, "%s", errmsg);
		 return (status);
      }
   }



   free(x);
   free(slack);
   free(dj);
   free(pi);


	
   *_objval = objval;

	return (status);
	

}



void MyOptimizer:: lineSearch(FunctionEval * f, double xL, 
		double xU, double delta, double * resultVal, double * sol){



double I0 = xU - xL;

std::vector<double> F;
F.push_back(1.0); 
F.push_back(2.0);
int n= 1;
double In = I0/F[1];
//std::cout << "In " << In << std::endl;
while(In > delta){
    n= n+1;
    F.push_back(F[n-1] + F[n-2]);
    In = I0/F[n];
}

int nIter = n -2;

//VLOG(3)  << "nIter = " << nIter << std::endl;
//system("pause"); 
std::vector<double> x;

// Para k = 1 - inicializando
double Ik = F[n - 1]*In;
//std::cout << "int ik = " << Ik << std::endl;
double xa = xU - Ik;
double xb = xL + Ik;
x.push_back(xa);
    
for (int k = 2; k <= nIter; k++){
    Ik = F[n - k]*In;
	//std::cout << "ik = " << Ik << std::endl;
    if (f->eval(xa) > f->eval(xb)) { // a derivada ?negativa
        xL  = xa;
        xa = xb;
        xb = xL + Ik;
	}else{
        xU = xb;
        xb = xa;
        xa = xU - Ik;
	}
	x.push_back(xa);
}

*sol = xa;

}
