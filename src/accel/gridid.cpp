#include "gridid.h"


GridId::GridId()
	: x(-1)
	, y(-1)
{
}

GridId::GridId(int64_t _x, int64_t _y) :x(_x), y(_y)
{

}

GridId::~GridId()
{
}

GridId& GridId::operator=(const GridId& id)
{
	x = id.x;
	y = id.y;
	return *this;
}

bool GridId::operator==(const GridId& id) const
{
	return (x == id.x) && (y == id.y);
}

bool GridId::operator!=(const GridId& id) const
{
	return !(*this == id);
}

int64_t GridId::operator[](unsigned id) const
{
	switch (id) {
		case 0: return x;
		case 1:return y;
		default: return 0;
	}
}

int64_t& GridId::operator[](unsigned id)
{
	switch (id) {
		case 0: return x;
		case 1:return y;
		default:  throw std::out_of_range("Invalid index");
	}
}

