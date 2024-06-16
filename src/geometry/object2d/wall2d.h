#ifndef RTLBS_WALL2D
#define RTLBS_WALL2D

#include "rtlbs.h"
#include "utility/define.h"
#include "object2d.h"

class Wall2D :public Object2D {
public:
	int m_wallId;				/** @brief	«ΩÃÂID	*/

public:
	Wall2D();
	~Wall2D();
};

bool LoadWallsFromFile(const std::string& geometryFile, const std::string& attributeFile, std::vector<Wall2D*>& outWalls);

#endif
