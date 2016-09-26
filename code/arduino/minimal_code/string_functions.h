#ifndef INCL_STRING_FUNCTIONS
#define INCL_STRING_FUNCTIONS

#include <stdlib.h>

inline int length(const char* source);
inline int indexOf(const char* source, const char* target);
inline bool contains(const char* source, const char* target);

//============================================================================================
// Gets the length of the character array
//============================================================================================
int length(const char* source)
{
  int length = 0;
  for(; source[length] != '\0'; length++);
  return length;
}

//============================================================================================
// Gets the index of the given string, or -1 if not found
//============================================================================================
int indexOf(const char* source, const char* target)
{
  int targetLength = length(target);
  int sourceLength = length(source);
  int index = -1;
  for(int i = 0; i <= sourceLength - targetLength && index == -1; i++)
  {
    bool foundTarget = true;
    for(int n = 0; n < targetLength && i+n < sourceLength; n++)
    {
      if(source[i+n] != target[n])
        foundTarget = false;
    }
    if(foundTarget)
      index = i;
  }
  return index;
}

//============================================================================================
// Checks if the given string is contained in the source string
//============================================================================================
bool contains(const char* source, const char* target)
{
  return indexOf(source, target) >= 0;
}

#endif
