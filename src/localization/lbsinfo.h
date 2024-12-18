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

//LBS�еĹ���Դ�汾����
class LBSInfo {
public:
	Sensor* m_sensor;
	std::vector<LBSTreeNode*> m_nodes;
	std::vector<GeneralSource*> m_sources;

public:
	LBSInfo();
	~LBSInfo();
	void SetSensorData(const SensorData& data);											//���ô���������
	void SetNodes(std::vector<LBSTreeNode*>& nodes);
	void CalculateBaseInfo(LOCALIZATION_METHOD lbsMethod);								//���������Ϣ
	void CalculateBaseInfo(std::vector<SensorData>& sensorDatas, LOCALIZATION_METHOD lbsMethod = LBS_METHOD_RT_TOA);						//���������Ϣ-TOA��վ�����ݶ�λ�㷨
};

inline void EraseRepeatGeneralSources(std::vector<GeneralSource*>& sources);

#endif
