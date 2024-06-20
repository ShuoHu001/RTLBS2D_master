#ifndef RTLBS_SENSORDATA
#define RTLBS_SENSORDATA

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"
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
	RtLbsType m_phiDegree;					/** @brief	���������յ��ķ�λ�� ��λ ��	*/

public:
	SensorData();
	SensorData(const SensorData& data);
	~SensorData();
	SensorData operator = (const SensorData& data);
	bool operator < (const SensorData& data);									//����С�ڷ��ţ����չ��ʴ�С������������
	Vector2D GetDirection() const;												//�����յ��ĽǶ�ת��Ϊ����ʸ��
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
