#ifndef RTLBS_LOCATIONBASEDSERVICE
#define RTLBS_LOCATIONBASEDSERVICE

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "equipment/sensor.h"
#include "equipment/receiver.h"
#include "tree/raypath3d.h"
#include "tree/lbstreenode.h"
#include "localization/generalsource.h"

//LBS中的广义源版本数据
class LBSResultGS {
public:
	Sensor* m_sensor;
	std::vector<LBSTreeNode*> m_nodes;
	std::vector<GeneralSource*> m_sources;

public:
	LBSResultGS();
	~LBSResultGS();
	void SetNodes(std::vector<LBSTreeNode*>& nodes);
	void CalculateBaseInfo(LOCALIZATION_METHOD lbsMethod);								//计算基本信息
};

inline void EraseRepeatGeneralSources(std::vector<GeneralSource*>& sources);

#endif
