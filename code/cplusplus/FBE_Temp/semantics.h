#ifndef _SEMANTICS_H
#define _SEMANTICS_H

#include <iostream>
#include <list>
namespace FabByExample{
		

	class Semantics{
	public:
		enum PartType {LEG, BODY, PERIPHERAL, FACE, GENERAL, WHEEL};
		enum PrintMethod {PRINT_AND_FOLD, DIRECT_3D};

		PartType partType;
		PrintMethod printMethod;

		Semantics(){
			partType = PartType::GENERAL;
			printMethod = PrintMethod::PRINT_AND_FOLD;
		}
		Semantics(PartType _partType, PrintMethod _printMethod){
			partType = _partType;
			printMethod = _printMethod;
		}

	};
	

}






#endif