#include <vector>
#include <math.h>
#include <stdlib.h>
#include <cstdlib>

#define debug(S, ...) fprintf(stderr, S, __VA_ARGS__);

using namespace std;

class Params {
 public:
  virtual int nDim() = 0;
  virtual void Copy(Params * params) = 0;
};

class Objective {
 public:
  virtual float func(const Params *params) = 0;
  virtual void nextSolution(const Params *paramin, 
			    Params *paramout) = 0;

  virtual void initSolution(Params *initSol) = 0;
  virtual Params* allocParams() = 0;
};

class Optimizer {
 public:
  virtual Params * optimize(Objective &obj, float *minEnergy = NULL) = 0;
}; 
 
