#ifndef __FRAMEWORK__
#define __FRAMEWORK__

#include "optimizer.h"
#include "libobject.h"
#include <vector>


using namespace std;




class Metric{ 
 public:
  virtual float Measure( Document *optimized) = 0;
};


// Partial implementation of Objective in optimizer.h
class Reducer {
 public:
  
  virtual void initSolution(Params * param) = 0;
  virtual Params * allocParams() = 0;
  virtual void nextSolution( const Params * paramin, Params *paramout);

  //  virtual 

};

class Simulator{
 public:
  virtual Document *simulate( const Params * reducer) = 0;
  virtual void *deviceObject( const Params * reducer) = 0;
};

class FrameWork {

 public:
  FrameWork();
  ~FrameWork();
  void RegisterSimulator( Simulator * s);
  void RegisterMetric( Metric * s);
  void RegisterOptimizer( Optimizer * s);
  void RegisterReducer( Reducer * s);

  void Run(); 

  void SetGoal( Document * goal);

  void SetLogLevel(int level);
  
  Params * GetResults();

 private:
  Document *m_goal;
  vector< Simulator* > simulators;
  vector< Metric * >   metrics;
  vector< Reducer * > reducers;
  // We may have more than one optimizer
  vector< Optimizer *>   optimizers;
  
  Params * m_result;
};



#endif
