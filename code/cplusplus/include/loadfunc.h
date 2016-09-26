#ifndef __LOAD_FUNC
#define __LOAD_FUNC
#ifdef WIN32

#else
#include <dlfcn.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#endif

#include <time.h>
#include <string.h>
#include <unit.h>
#include <vector>

#ifdef LUA
#include "lua.hpp"
#include "unit.h"
#include <vector>


// Lua helper

class Lua_State
{

 public:
  lua_State *L;
 Lua_State() : L(lua_open()) {  luaL_openlibs(L); fprintf(stderr,"Open lua\n");}

  ~Lua_State() {
    lua_close(L);
  }

  // implicitly act as a lua_State pointer
  inline operator lua_State*() {
    return L;
  }
};

// C++ compiler

#endif

#define COMPILEFLAG   "-g", "-Wall", "-I/usr/include/lua5.1", "-I.", "-I./include"

typedef int (*FUNC)( const std::vector<struct objrep::Value > &functionParams,
		     std::vector<struct objrep::Value > &outparams);

class DynamicFunc {
 public:
  DynamicFunc( const char *library, const char *name) {
    function = NULL;
#ifdef WIN32
#else
    handle = dlopen(library, RTLD_LOCAL | RTLD_LAZY);
    if (handle) {
      function = (FUNC)dlsym(handle, name);
      if (function == NULL)
	fprintf(stderr,"Error loading function\n");
    } else {
      fprintf(stderr,"Error loading linking\n");
    }
#endif	

  }
  ~DynamicFunc() {
#ifdef WIN32       
#else
    if (handle) 
      dlclose(handle);
#endif
  }

  FUNC function;


 private:
#ifdef WIN32


#else
  void *handle;

#endif




};

// Will need to make it compile on windows

#define CC "/usr/bin/g++"

class LoadCPP {
 public:
#ifdef WIN32
  static char * mktemp(char * name) {
    for(int i=0;i<(int)strlen(name);i++) 
      if (name[i]=='X') {
	name[i] = time(NULL);
      }
    return name;
  }
#endif

  LoadCPP( const char * filename, const char *name) {
    func = NULL;


#ifdef WIN32

#else
    //	char *ftemplate = "/tmp/fileXXXXXX";
    tempfile = new char[30];
    strcpy(tempfile, "/tmp/file.XXXXXX");

    mktemp(tempfile);

    pid_t pid = fork ();
    int status;
    if (pid == 0)   {
      execl(CC, CC, "-shared", "-fPIC", COMPILEFLAG, "-o", tempfile, filename, (char*) 0);
    } else {
      if (waitpid (pid, &status, 0) != pid)
	status = -1;
    }
    func = new DynamicFunc(tempfile, name);
#endif
  }
  LoadCPP( const char *source, int len, const char *name) {
    tempfile = NULL;
    func = NULL;
    char tname[30];
    char fname[30];
    strcpy(tname, "/tmp/source.XXXXXX");       
    if (mktemp(tname) != NULL) {
      sprintf(fname, "%s.c", tname);
      FILE *fo=fopen(fname, "wb");
      if (fo) {
	fwrite(source, sizeof(char), len, fo);
	fclose(fo);
	LoadCPP(fname, name);
      }
    }
    //	unlink(tname);

  }

  ~LoadCPP() {
    if (tempfile) {
      //	    unlink(tempfile);
      delete tempfile;
    }
  }
  DynamicFunc *getFunction() { return func; }
 private:
  char *tempfile;
  DynamicFunc *func;


};




#endif
