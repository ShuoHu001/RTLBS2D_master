#ifndef RTLBS_GRIDID
#define RTLBS_GRIDID

#include "utility/define.h"
#include <stdexcept>


class GridId {
public:
	int64_t x;
	int64_t y;
public:
	GridId();
	GridId(int64_t _x, int64_t _y);
	~GridId();
	GridId& operator = (const GridId& id);
	bool operator == (const GridId& id) const;
	bool operator != (const GridId& id) const;
	int64_t operator [] (unsigned id) const;
	int64_t& operator [] (unsigned id);
};

#endif
