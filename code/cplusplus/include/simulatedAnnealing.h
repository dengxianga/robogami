#ifndef __SIMULATED__
#define __SIMULATED__

#include "optimizer.h"


class SimulatedAnnealingCoolSched {
 public:
  virtual float newT( float T) { return 0.9f*T; };
};


SimulatedAnnealingCoolSched _DefaultCooling;
class SimulatedAnnealing : public Optimizer {
 private:
  float m_InitTemp;
  int m_MaxConsRef;
  int m_MaxSuccess;
  int m_MaxTries;
  float m_StopTemp;
  float m_StopVal;
  SimulatedAnnealingCoolSched *m_sched;
  float m_BoltzmannConst;
  
 public:
  SimulatedAnnealing(float InitTemp = 1,
		     int MaxConsRef = 1000,
		     int MaxSuccess = 100,
		     int MaxTries = 300,
		     float StopTemp = 1e-8,
		     float BoltzmannConst = 1.0f,
		     float StopVal = -INFINITY,
		     SimulatedAnnealingCoolSched *sched = &_DefaultCooling) 
    : m_InitTemp(InitTemp),
    m_MaxConsRef(MaxConsRef),
    m_MaxSuccess(MaxSuccess),
    m_MaxTries(MaxTries),
    m_StopTemp(StopTemp),
    m_BoltzmannConst(BoltzmannConst),
    m_StopVal(StopVal),
    m_sched(sched)
    {
      
    }
  Params *optimize(Objective &obj, float *minEnergyOut = NULL) {
    int nsample = 0;
    int itry = 0;
    int success = 0;
    int finished = 0;
    int consec = 0;
    float T = m_InitTemp;

    Params *current = obj.allocParams();
    Params *newParam = obj.allocParams();
    Params *minParam = obj.allocParams();
    Params *tmpParam = NULL;
    obj.initSolution(current);
    float initEnergy = obj.func( current);
    float oldEnergy = initEnergy;
    float minEnergy = INFINITY;
    float newEnergy = 0.0f;
    float total = 0.0f;
    while (!finished) {
      // iteration counter;
      itry ++;
      nsample ++;

      if ( itry >=m_MaxTries || success >= m_MaxSuccess) {
	total += itry;
	if (T < m_StopTemp || consec  >= m_MaxConsRef) {
	  finished = 1;
	  break;
	} else {
	  T = m_sched->newT(T);
	  debug("T = %7.5f, loss = %10.5f\n", T, oldEnergy);
	  itry = 1;
	  success = 1;
	}
      }
#define SWAP(A, B) { tmpParam = A; A = B; B=tmpParam; };
      // Get new solution && enery
      obj.nextSolution(current, newParam);
      newEnergy = obj.func(newParam);

      if (newEnergy < minEnergy) {
	minEnergy = newEnergy;
	newParam->Copy(minParam);
      }

      if (newEnergy < m_StopVal) {
	break;
      }
      if (oldEnergy - newEnergy > 1e-6) {
	oldEnergy = newEnergy;
	SWAP(newParam, current);
	success ++;
	consec = 0;
      } else {
	float t = (float)rand()/RAND_MAX;
	if (t < exp( (oldEnergy - newEnergy)/(m_BoltzmannConst*T))) {
	  oldEnergy = newEnergy;
	  SWAP(newParam, current);
	  success ++;
	} else {
	  consec ++;
	}
      }
    }
    delete newParam;
    delete current;
    if (minEnergyOut)
      *minEnergyOut = minEnergy;
    printf("%d %f\n", nsample, minEnergy);
    return minParam;
  }
};







#endif
