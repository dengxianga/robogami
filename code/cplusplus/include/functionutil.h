#ifndef _FUNCTION_UTIL
#define _FUNCTION_UTIL

#include "libobject.h"
#include "loadfunc.h"
#include <vector>



int callFunction(void *f, float x, std::vector<objrep::Value > &tout) {
  std::vector< objrep::Value > t;
  t.push_back(objrep::Value(x));
  return ((FUNC)f)(t, tout);
}

int callFunction(void *f, float x, float y, std::vector<objrep::Value > &tout) {
  std::vector< objrep::Value > t;
  t.push_back(objrep::Value(x));
  t.push_back(objrep::Value(y));

  return ((FUNC)f)(t, tout);
}


#endif
