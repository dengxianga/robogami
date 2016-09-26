#ifndef __REPIO__
#define __REPIO__

#include "context.h"
#include <string>

namespace objrep {

	void ReadRep( Context *context, const char * filename);
	void WriteRep( Context *context, const char * filename);

}
#endif