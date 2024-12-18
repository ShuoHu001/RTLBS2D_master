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
	std::vector<SensorData> m_datas;

public:
	SensorDataCollection();
	SensorDataCollection(const SensorDataCollection& collection);
	~SensorDataCollection();
	SensorDataCollection& operator = (const SensorDataCollection& collection);
	RtLbsType CalculateRMSAngularSpread() const;										//计算RMS角度扩展
	RtLbsType CalculateRMSDelaySpread() const;											//计算RMS时延扩展
	void ReClusterByAOAError(RtLbsType phiError);										//按照角度误差进行聚类	
	void ReClusterByTOAError(RtLbsType timeError);										//按照时间误差进行聚类
	void ReClusterByTDOAError(RtLbsType timeDiffError);									//按照时间差误差进行聚类
	void SortByPower();																	//按照能量的大小进行排序
	void SortByTime();																	//按照时延大小进行排序
	void SortByTimeDifference();														//按照时间差的大小进行排序
	void CalculateTimeDiff();															//计算时延差值
	RtLbsType GetMaxPropagationTime() const;											//获取最大传播时延值
	std::vector<RtLbsType> GetPowerDiffMatrix() const;									//计算功率差矩阵
	void AddSimulationError(RtLbsType phiErrorSigma, RtLbsType timeErrorSigma, RtLbsType powerErrorSigma);			//增加仿真误差
	bool Init(const std::string& filename);
	void Write2Json(const std::string& filename) const;
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) const;
	bool Deserialize(const rapidjson::Value& value);

private:
	RtLbsType CalculateMeanArrivedAngle() const;										//计算平均到达角度
	RtLbsType CalculateMeanArrivedDelay() const;										//计算平均到达时延
	
};




#endif
