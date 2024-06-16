#ifndef RTLBS_VEGETATION2D
#define RTLBS_VEGETATION2D

#include "rtlbs.h"
#include "utility/define.h"
#include "object2d.h"

class Vegetation2D :public Object2D {
public:
	int m_vegetationId;

public:
	Vegetation2D();
	~Vegetation2D();
};

//���ļ��ж�ȡ�̻�ֲ��
bool LoadVegetationsFromFile(const std::string& geometryFile, const std::string& attributeFile, std::vector<Vegetation2D*>& outVegetations);

#endif
