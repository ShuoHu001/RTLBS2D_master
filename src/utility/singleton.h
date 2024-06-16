#ifndef RTLBS_SINGLETON
#define RTLBS_SINGLETON

#include "rtlbs.h"
#include "utility/define.h"

template<typename T>
class Singleton {
public:
	static T& getInstance() {
		static T instance;
		return instance;
	}

protected:
	Singleton() {}
	~Singleton() {}

private:
	Singleton(const Singleton&) = delete;
	Singleton& operator=(const Singleton&) = delete;
};

#endif
