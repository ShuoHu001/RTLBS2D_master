#ifndef RTLBS_LBSINFO
#define RTLBS_LBSINFO

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "equipment/sensor/sensor.h"
#include "equipment/transceiver/receiver.h"
#include "radiowave/raypath/raypath3d.h"
#include "lbstreenode.h"
#include "generalsource.h"

//LBS中的广义源版本数据
class LBSInfo {
public:
	Sensor* m_sensor;
	std::vector<LBSTreeNode*> m_nodes;
	std::vector<GeneralSource*> m_sources;

public:
	LBSInfo();
	~LBSInfo();
	void SetSensorData(const SensorData& data);											//设置传感器数据
	void SetNodes(std::vector<LBSTreeNode*>& nodes);
	void CalculateBaseInfo(LOCALIZATION_METHOD lbsMethod);								//计算基本信息
	void CalculateBaseInfo(std::vector<SensorData>& sensorDatas, LOCALIZATION_METHOD lbsMethod = LBS_METHOD_RT_TOA);						//计算基本信息-TOA单站多数据定位算法
};

inline void EraseRepeatGeneralSources(std::vector<GeneralSource*>& sources);

#endif
