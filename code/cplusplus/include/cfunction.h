#ifndef  __CFUNC__
#define  __CFUNC__

#include "context.h"
#include "primitives.h"
#include "loadfunc.h"


class CFunction : public Function {
 private:
  int length;
  string code;
  string functionname;
  Unit *inputUnit;
  Unit *outputUnit;

  LoadCPP *cpp;

 public:

  CFunction();
  ~CFunction();


  void callFunc(const std::vector<Value > &functionParams,
		std::vector<Value > &outparams);
  Unit *argUnit() { return inputUnit; }
  Unit *outUnit() { return outputUnit; }

  const string language() { return "C"; }

  Primitive * Read(const YAML::Node &node, Document *document) ;
  void Write(YAML::Emitter &emitter) ;

  const char *TAG() { return "CFunction"; }
  

};

#endif
