#ifndef RTLBS_SENSORDATACOLLECTION
#define RTLBS_SENSORDATACOLLECTION

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "sensordata.h"
#include "sensordatacluster.h"

const std::string KEY_SENSORDATACOLLECTION = "SensorDataCollection";
const std::string KEY_SENSORDATACOLLECTION_SENSORID = "SensorId";
const std::string KEY_SENSORDATACOLLECTION_SENSORDATAS = "SensorDatas";

class SensorDataCollection {
public:
	int m_sensorId;
	std::vector<SensorData> m_data;

public:
	SensorDataCollection();
	SensorDataCollection(const SensorDataCollection& collection);
	~SensorDataCollection();
	SensorDataCollection& operator = (const SensorDataCollection& collection);
	void ReClusterByAOAError(RtLbsType phiError);										//按照角度误差进行聚类				
	void SortByPower();																	//按照能量的大小进行排序
	void CalculateTimeDiff();															//计算时延差值
	std::vector<RtLbsType> GetPowerDiffMatrix() const;									//计算功率差矩阵
	void AddSimulationError(RtLbsType phiErrorSigma, RtLbsType timeErrorSigma, RtLbsType powerErrorSigma);			//增加仿真误差
	bool Init(const std::string& filename);
	void Write2Json(const std::string& filename) const;
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) const;
	bool Deserialize(const rapidjson::Value& value);
	
};




#endif
