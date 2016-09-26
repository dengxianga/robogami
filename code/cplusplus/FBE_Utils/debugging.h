#pragma once
#include <vector>
#include <string>
#include <map>
#include <sstream>
#include <Windows.h>
#include <iostream>
#include <iomanip>
#include "glog/logging.h"
#include "glog/stl_logging.h"
#include "glog/raw_logging.h"

#define ENABLE_PROFILING

using namespace std;
namespace FabDebugging {
	class Debuggable;
	class DebugInfo {
	public:
		enum PropertyType {
			INT,
			DOUBLE,
			BOOL,
			STRING,
			AGGREGATION,  // owner
			REFERENCE,  // reference
		};

		virtual const vector<pair<string, PropertyType>>& getProperties() const { return properties; }
		virtual int getInt(const string& name) const { return intData.at(name); }
		virtual double getDouble(const string& name) const { return doubleData.at(name); }
		virtual bool getBool(const string& name) const { return boolData.at(name); }
		virtual string getString(const string& name) const { return stringData.at(name); }
		virtual Debuggable* getAggregationOrReference(const string& name) const { return pointerData.at(name); }
		virtual string getTypeName() const { return typeName; }
		virtual string getShortDescription() const { return shortDescription; }

		void putIntProperty(const string& name, int value) { intData[name] = value; properties.push_back(make_pair(name, PropertyType::INT)); }
		void putDoubleProperty(const string& name, double value) { doubleData[name] = value; properties.push_back(make_pair(name, PropertyType::DOUBLE)); }
		void putBoolProperty(const string& name, bool value) { boolData[name] = value; properties.push_back(make_pair(name, PropertyType::BOOL)); }
		void putStringProperty(const string& name, string value) { stringData[name] = value; properties.push_back(make_pair(name, PropertyType::STRING)); }
		void putAggregationProperty(const string& name, Debuggable* value) { pointerData[name] = value; properties.push_back(make_pair(name, PropertyType::AGGREGATION)); }
		void putReferenceProperty(const string& name, Debuggable* value) { pointerData[name] = value; properties.push_back(make_pair(name, PropertyType::REFERENCE)); }
		void setTypeName(const string& value) { typeName = value; }
		void setShortDescription(const string& value) { shortDescription = value; }

	private:
		vector<pair<string, PropertyType>> properties;
		map<string, int> intData;
		map<string, double> doubleData;
		map<string, bool> boolData;
		map<string, string> stringData;
		map<string, Debuggable*> pointerData;
		string typeName;
		string shortDescription;
	};


	class Debuggable {
	public:
		virtual ~Debuggable() {}

		virtual DebugInfo* getDebugInfo() = 0;
	};
}

class StringConcater {
private:
	ostringstream s;

public:
	template<class T>
	StringConcater& operator << (const T& thing) {
		s << thing;
		return *this;
	}

	operator string() {
		return s.str();
	}
};

inline StringConcater concat() {
	return StringConcater();
}

namespace FabProfiling {
	const int MaxProfilingDepth = 200;

	typedef long long bint;

	struct ProfileCounterObj {
		bint totalTimes;
		bint totalSelfTimes;
		bint totalCalls;
		const char* name;
		ProfileCounterObj(const char* name);
		~ProfileCounterObj();
	};

	struct Counters {
		bint totalTimes;
		bint totalSelfTimes;
		bint totalCalls;
		const char* name;
	};

	struct Impl {
		static ProfileCounterObj* _callStack[MaxProfilingDepth];
		static bint _startTimes[MaxProfilingDepth];
		static bint _innerTimes[MaxProfilingDepth];
		static int _callDepth;
		static std::vector<ProfileCounterObj*> __allCounters;
		static std::map<ProfileCounterObj*, Counters> __savedCounters;
	};



#ifdef ENABLE_PROFILING
	inline void recordBegin(ProfileCounterObj* counter) {
		LARGE_INTEGER startTime;
		::QueryPerformanceCounter(&startTime);
		Impl::_startTimes[Impl::_callDepth] = startTime.QuadPart;
		Impl::_innerTimes[Impl::_callDepth] = 0;
		Impl::_callStack[Impl::_callDepth] = counter;
		++Impl::_callDepth;
	}

	inline void recordEnd() {
		LARGE_INTEGER endTime;
		::QueryPerformanceCounter(&endTime);
		--Impl::_callDepth;
		bint time = endTime.QuadPart - Impl::_startTimes[Impl::_callDepth];
		auto counter = Impl::_callStack[Impl::_callDepth];
		counter->totalTimes += time;
		bint selfTime = time - Impl::_innerTimes[Impl::_callDepth];
		counter->totalSelfTimes += selfTime;
		++counter->totalCalls;
		Impl::_innerTimes[Impl::_callDepth - 1] += time;
	}

	struct __ProfileObject {
		__ProfileObject(ProfileCounterObj* counter) {
			recordBegin(counter);
		}
		
		~__ProfileObject() {
			recordEnd();
		}
	};

#define TOKENPASTE(x, y) x ## y
#define TOKENPASTE2(x, y) TOKENPASTE(x, y)
#define UNIQUE_VAR TOKENPASTE2(__unique_var_, __LINE__)

#define __PROFILE_VAR_NAME __must_not_use_two_PROFILE_THIS_macros_in_the_same_scope
#define PROFILE_THIS(name) static FabProfiling::ProfileCounterObj __PROFILE_VAR_NAME(name); FabProfiling::__ProfileObject __instrumenter(&__PROFILE_VAR_NAME);
#define PROFILE_BEGIN(name) static FabProfiling::ProfileCounterObj UNIQUE_VAR(name); FabProfiling::recordBegin(&UNIQUE_VAR);
#define PROFILE_END FabProfiling::recordEnd();
#define PROFILED

	inline void printCounters() {
		printf("=========== Profile Counters =============\n");
		printf("%80s%10s%15s%15s%15s%15s\n", "Counter", "Calls", "Total Time", "Self Time", "Avg Time", "Avg Self");
		for each (ProfileCounterObj* counter in Impl::__allCounters) {

			const char* name;
			double totalCalls, totalSelfTimes, totalTimes;

			if (Impl::__savedCounters.find(counter) != Impl::__savedCounters.end()) {
				auto saved = Impl::__savedCounters[counter];
				name = saved.name;
				totalCalls = saved.totalCalls;
				totalSelfTimes = saved.totalSelfTimes;
				totalTimes = saved.totalTimes;
			}
			else {
				name = counter->name;
				totalCalls = counter->totalCalls;
				totalSelfTimes = counter->totalSelfTimes;
				totalTimes = counter->totalTimes;
			}
			
			std::cout << std::setprecision(3)
				<< std::setw(80) << name
				<< std::setw(10) << totalCalls;
			cout.setf(ios::fixed, ios::floatfield);
			cout.setf(ios::showpoint);
			cout<< std::setw(15) << (double)totalTimes / 1000
				<< std::setw(15) << (double)totalSelfTimes / 1000
				<< std::setw(15) << (double)totalTimes / totalCalls / 1000
				<< std::setw(15) << (double)totalSelfTimes / totalCalls / 1000
				<< std::endl;
		}
	}

#else
#define PROFILE_THIS(counter)
#define PROFILED
#define PROFILE_BEGIN(name)
#define PROFILE_END
#endif
}

extern string _global_progress_string;
extern int _global_progress_total;
extern int _global_progress_done;

class ProgressTracking {
public:
	static void SetProgress(string const& name) {
		_global_progress_string = name;
	}
	static void StartIntensiveWork(int totalSteps) {
		_global_progress_total = totalSteps;
		_global_progress_done = 0;
		_global_progress_string = "";
	}
	static void NextStep() {
		_global_progress_done++;
	}
};