#ifndef __UTILS__
#define __UTILS__


#include "libobject.h"
#include "yaml-cpp/yaml.h"
#include <vector>
#include "Vec.h"
#include <string.h>
#include <fstream>
#include <iostream> 
#include "context.h"
using namespace std;


// Helper Function
class Primitive;
class Document;


template<class TYPE>
int readYAMLMapScalar(const YAML::Node &node, const char * TAG, TYPE &val) {
  int success = 1;
  if (node.FindValue(TAG) == NULL)
    return 0;

  if (node[TAG].Type() == YAML::NodeType::Scalar) {
    try {
      node[TAG] >> val;
    } catch(exception &e) {
      success = 0;
    }
  }
  return success;
}

template<class TYPE>
TYPE *readYAMLMapSequence(const YAML::Node &node, const char *TAG, int *size = 0) {
	TYPE *array = NULL;
	int rsize = 0;
	if (node.FindValue(TAG) == NULL)
		return 0;
	if (node[TAG].Type() == YAML::NodeType::Sequence) {

		if (size && *size > 0) {
			rsize = *size;
			if ((signed)node[TAG].size() < *size)
				return NULL;
		}
		if (size && *size == 0) {
			*size = node[TAG].size();
			rsize = node[TAG].size();
		}
		array = new TYPE[rsize];
		try {
			//node[TAG] >> val;
			for(unsigned int i=0;i< rsize;i++) {
				node[TAG][i] >> array[i];
			}
		} catch(exception &e) {
			delete array;
		}
	}
	return array;
}

template<class TYPE>
void readYAMLSequence(const YAML::Node &node, std::vector<TYPE> &vec) {
	int rsize = 0;
	if (node.Type() == YAML::NodeType::Sequence) {
		rsize = node.size();
		vec.resize( rsize);
		try {
			//node[TAG] >> val;
			for(unsigned int i=0;i< rsize;i++) {
				node[i] >> vec[i];
			}
		} catch(exception &e) {
	
		}
	}
}

 

template<class ENUM>
int readYAMLMapEnum(const YAML::Node &node, const char *TAG, 
	const char **enums_text,
	const ENUM *enums, 
	int nEnums,
	ENUM &out) {
		if (node.FindValue(TAG) == NULL)
			return 0;
		if (node[TAG].Type() == YAML::NodeType::Scalar) {
			string val;
			node[TAG] >> val;
			for(int i=0;i<nEnums;i++) {
				if (val == enums_text[i]) {
					out = enums[i];
					return 1;
				}
			}
		}
		return 0;

}



objrep::Unit *readUnit(objrep::Context *context, const YAML::Node &node, const char *TAG);




template<class ENUM>
void writeYAMLMapEnum(YAML::Emitter &emitter, const char *TAG,
			const char **enums_text,
			const ENUM *enums, 
			int nEnums,
			const ENUM &val) {

  for(int i=0;i<nEnums;i++) {
    if (val == enums[i]) {
      emitter << YAML::Key << TAG;
      emitter << YAML::Value << enums_text[i];
      break;
    }
  }
}
			


template<class TYPE>
void writeYAMLMapScalar(YAML::Emitter &emitter, const char *TAG, const TYPE &val, int literal = 0) {
  emitter << YAML::Key << TAG;
  //  emitter << 
  if (literal)
    emitter << YAML::Value << YAML::Literal << val;
  else
    emitter << YAML::Value << val;
}


template<class TYPE>
void writeYAMLMapSequence(YAML::Emitter &emitter, const char *TAG, TYPE * vals, int size) {

  emitter << YAML::Key << TAG;
  emitter << YAML::Value << YAML::Flow<< YAML::BeginSeq;
  for(int i=0;i<size;i++) {
    emitter << vals[i];
  }
  emitter << YAML::EndSeq;
}


//void writeMapPrimitive(YAML::Emitter &emitter, const char *TAG, Primitive *primitive);
void writeUnit(YAML::Emitter &emitter, const char *TAG, objrep::Unit *unit);




#endif
