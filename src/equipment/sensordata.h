#ifndef RTLBS_SENSORDATA
#define RTLBS_SENSORDATA

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"
#include "managers/randomanager.h"
#include "geometry/vector2d.h"


const std::string KEY_SENSORDATA_TIME = "Time";
const std::string KEY_SENSORDATA_TIMEDIFF = "TimeDiff";
const std::string KEY_SENSORDATA_PHI_DEGREE = "PhiDegree";
const std::string KEY_SENSORDATA_POWER = "Power";

class SensorData {
public:
	int m_id;								/** @brief	���������ݱ��,ȫ��Ψһ	*/
	int m_sensorId;							/** @brief	������ID	*/
	RtLbsType m_time;						/** @brief	���������յ���ʱ��, ��λ ns	*/
	RtLbsType m_timeDiff;					/** @brief	���������յ���ʱ����λ ns	*/
	RtLbsType m_phi;						/** @brief	���������յ��ķ�λ��,��λ ����	*/
	RtLbsType m_power;						/** @brief	���������յ��Ĺ��ʣ���λ dBm	*/
	RtLbsType m_powerLin;					/** @brief	���������յ������Թ��ʣ���λdBm	*/
	RtLbsType m_phiDegree;					/** @brief	���������յ��ķ�λ�� ��λ ��	*/

public:
	SensorData();
	SensorData(const SensorData& data);
	~SensorData();
	SensorData operator = (const SensorData& data);
	bool operator < (const SensorData& data);									//����С�ڷ��ţ����չ��ʴ�С������������
	RtLbsType DistanceAOA2D(const SensorData& data) const;						//��ά�ǶȾ���
	RtLbsType DistanceDelay(const SensorData& data) const;						//ʱ��
	Vector2D GetDirection() const;												//�����յ��ĽǶ�ת��Ϊ����ʸ��
	void AddSimulationError(RtLbsType phiErrorSigma, RtLbsType timeErrorSigma, RtLbsType powerErrorSigma);			//���ӷ������
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
	
};

#endif
