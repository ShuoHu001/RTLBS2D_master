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
	RtLbsType CalculateRMSAngularSpread() const;										//����RMS�Ƕ���չ
	RtLbsType CalculateRMSDelaySpread() const;											//����RMSʱ����չ
	void ReClusterByAOAError(RtLbsType phiError);										//���սǶ������о���	
	void ReClusterByTOAError(RtLbsType timeError);										//����ʱ�������о���
	void ReClusterByTDOAError(RtLbsType timeDiffError);									//����ʱ��������о���
	void SortByPower();																	//���������Ĵ�С��������
	void SortByTime();																	//����ʱ�Ӵ�С��������
	void SortByTimeDifference();														//����ʱ���Ĵ�С��������
	void CalculateTimeDiff();															//����ʱ�Ӳ�ֵ
	RtLbsType GetMaxPropagationTime() const;											//��ȡ��󴫲�ʱ��ֵ
	std::vector<RtLbsType> GetPowerDiffMatrix() const;									//���㹦�ʲ����
	void AddSimulationError(RtLbsType phiErrorSigma, RtLbsType timeErrorSigma, RtLbsType powerErrorSigma);			//���ӷ������
	bool Init(const std::string& filename);
	void Write2Json(const std::string& filename) const;
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) const;
	bool Deserialize(const rapidjson::Value& value);

private:
	RtLbsType CalculateMeanArrivedAngle() const;										//����ƽ������Ƕ�
	RtLbsType CalculateMeanArrivedDelay() const;										//����ƽ������ʱ��
	
};




#endif
