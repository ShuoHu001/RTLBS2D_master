#ifndef RTLBS_BUILDING2D
#define RTLBS_BUILDING2D

#include "rtlbs.h"
#include "utility/define.h"
#include "object2d.h"
#include "managers/logmanager.h"

class Building2D :public Object2D {
public:
	int m_buildingId;					/** @brief	������ID	*/

public:
	Building2D();
	~Building2D();
};

//���ļ��г�ʼ��������
bool LoadBuildingsFromFile(const std::string& geometryFile, const std::string& attributeFile, std::vector<Building2D*>& outBuildings);

#endif
