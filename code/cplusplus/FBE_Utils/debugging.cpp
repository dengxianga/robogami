#include "debugging.h"


namespace FabProfiling {
	ProfileCounterObj* Impl::_callStack[MaxProfilingDepth];
	bint Impl::_startTimes[MaxProfilingDepth];
	bint Impl::_innerTimes[MaxProfilingDepth];
	int Impl::_callDepth;
	std::vector<ProfileCounterObj*> Impl::__allCounters;
	std::map<ProfileCounterObj*, Counters> Impl::__savedCounters;

	ProfileCounterObj::ProfileCounterObj(const char* name) {
		this->name = name;
		Impl::__allCounters.push_back(this);
		totalTimes = totalSelfTimes = totalCalls = 0;
	}

	ProfileCounterObj::~ProfileCounterObj() {
		Counters c;
		c.name = name;
		c.totalCalls = totalCalls;
		c.totalSelfTimes = totalSelfTimes;
		c.totalTimes = totalTimes;
		Impl::__savedCounters[this] = c;
	}
#ifdef ENABLE_PROFILING
	static class _init
	{
	public:
		_init() {
			Impl::_callDepth = 0;
			for (int i = 0; i < MaxProfilingDepth; i++) {
				Impl::_callStack[i] = nullptr;
				Impl::_startTimes[i] = 0;
				Impl::_innerTimes[i] = 0;
			}
			recordBegin(nullptr);
		}
		~_init() {
			printCounters();
		}
	} _initializer;
#endif
}
string _global_progress_string = "";
int _global_progress_total = 0;
int _global_progress_done = 0;