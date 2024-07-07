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
	void ReClusterByAOAError(RtLbsType phiError);										//���սǶ������о���				
	void SortByPower();																	//���������Ĵ�С��������
	void CalculateTimeDiff();															//����ʱ�Ӳ�ֵ
	std::vector<RtLbsType> GetPowerDiffMatrix() const;									//���㹦�ʲ����
	void AddSimulationError(RtLbsType phiErrorSigma, RtLbsType timeErrorSigma, RtLbsType powerErrorSigma);			//���ӷ������
	bool Init(const std::string& filename);
	void Write2Json(const std::string& filename) const;
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) const;
	bool Deserialize(const rapidjson::Value& value);
	
};




#endif
