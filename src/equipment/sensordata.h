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

//����ǶȲв������
inline void CalculateResidual_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi)
{
	r_phi = (d1.m_phi - d2.m_phi) * (d1.m_phi - d2.m_phi);								//����ǶȲв�ƽ��
}

//����ʱ���в������
inline void CalculateResidual_TDOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_timediff)
{
	r_timediff = (d1.m_timeDiff - d2.m_timeDiff) * (d1.m_timeDiff - d2.m_timeDiff);		//����ʱ���в�ƽ��
}

//����ʱ���в�������ͽǶȲв������
inline void CalculateResidual_TDOA_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi, RtLbsType& r_timediff)
{
	r_phi = (d1.m_phi - d2.m_phi) * (d1.m_phi - d2.m_phi);								//����ǶȲв�ƽ��
	r_timediff = (d1.m_timeDiff - d2.m_timeDiff) * (d1.m_timeDiff - d2.m_timeDiff);		//����ʱ���в�ƽ��
}


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
	void SortByPower();																	//���������Ĵ�С��������
	void CalculateTimeDiff();															//����ʱ�Ӳ�ֵ
	std::vector<RtLbsType> GetPowerDiffMatrix() const;									//���㹦�ʲ����
	bool Init(const std::string& filename);
	void Write2Json(const std::string& filename) const;
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) const;
	bool Deserialize(const rapidjson::Value& value);
};

//�����������������ݵĲв������-AOA��λ����-������
inline void CalculateSensorResidual_AOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi)
{
	//����ǶȲв��������
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateResidual_AOA(c1.m_data[i], c2.m_data[i], cur_r_phi);
		r_phi += cur_r_phi;
	}
}

//�������鴫�������ݵĲв������-AOA��λ����-������
inline void CalculateSensorCollectionResidual_AOA_SingleData(std::vector<SensorDataCollection>& c1, std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_powerDiff)
{
	//����ǶȲв��������
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateSensorResidual_AOA_SingleData(c1[i], c2[i], cur_r_phi);
		r_phi += cur_r_phi;
	}

	//���㹦�ʲ�в��������
	for (int i = 1; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType powerDiff1 = c1[i].m_data[0].m_power - c1[0].m_data[0].m_power;
		RtLbsType powerDiff2 = c2[i].m_data[0].m_power - c2[0].m_data[0].m_power;
		r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
	}

}

//�����������������ݵĲв������-AOA��λ����-������
inline void CalculateSensorResidual_AOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_powerdiff)
{
	//1-�淶����������Դ-��������ʱ����һ���ķֱ��ʽ������ɣ����սǶȷֱ��ʣ������޷ֱ���
	//2-�淶������Դ�Ų���ʽ-����������ʱ���չ����Դ��С��˳������Ų�

	//ȡc1��c2���������ٵ���Ϊ�Ƚ�
	int minDataNum = static_cast<int>(std::min(c1.m_data.size(), c1.m_data.size()));
	int zoomFactor = 1;																					/** @brief	�в�����ϵ������c2��ֻ�в�������c1����������ϵ������	*/
	zoomFactor = static_cast<int>(c1.m_data.size()) - minDataNum;										//������ݾͻ�Ŵ󼸱�
	//����ǶȲв��
	for (int i = 0; i < minDataNum; ++i) {
		RtLbsType cur_r_phi;
		CalculateResidual_AOA(c1.m_data[i], c2.m_data[i], cur_r_phi);
		r_phi += cur_r_phi;
	}
	r_phi *= zoomFactor;

	//���㹦�ʲ�в��
	for (int i = 1; i < minDataNum; ++i) {
		RtLbsType powerDiff1 = c1.m_data[i].m_power - c1.m_data[0].m_power;
		RtLbsType powerDiff2 = c2.m_data[i].m_power - c2.m_data[0].m_power;
		r_powerdiff += (powerDiff2 - powerDiff1) * (powerDiff2 - powerDiff1);
	}
	r_powerdiff *= zoomFactor;
}

//�������鴫�������ݵĲв������-AOA��λ�㷨-������
inline void CalculateSensorCollectionResidual_AOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_powerDiff)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0.0;
		RtLbsType cur_r_powerDiff = 0.0;
		CalculateSensorResidual_AOA_MultiData(c1[i], c2[i], cur_r_phi, cur_r_powerDiff);
		r_phi += cur_r_phi;
		r_powerDiff += cur_r_powerDiff;
	}
}


//�����������������ݵĲв������-TDOA��λ����-������
inline void CalculateSensorResidual_TDOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_timeDiff)
{
	//����ʱ�Ӳ�в��������
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_data[i], c2.m_data[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
}


//�������鴫�������ݵĲв������-TDOA��λ����-������
inline void CalculateSensorCollectionResidual_TDOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff)
{
	//����ʱ�Ӳ�в��������
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateSensorResidual_TDOA_SingleData(c1[i], c2[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}

	//���㹦�ʲ�в��������
	for (int i = 1; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType powerDiff1 = c1[i].m_data[0].m_power - c1[0].m_data[0].m_power;
		RtLbsType powerDiff2 = c2[i].m_data[0].m_power - c2[0].m_data[0].m_power;
		r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
	}
}


//�����������������ݵĲв������-TDOA��λ����-������
inline void CalculateSensorResidual_TDOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerdiff)
{
	//ȡc1��c2���������ٵ���Ϊ�Ƚ�
	int minDataNum = static_cast<int>(std::min(c1.m_data.size(), c1.m_data.size()));
	int zoomFactor = 1;																					/** @brief	�в�����ϵ������c2��ֻ�в�������c1����������ϵ������	*/
	zoomFactor = static_cast<int>(c1.m_data.size()) - minDataNum;										//������ݾͻ�Ŵ󼸱�

	//����ʱ�Ӳ�в��������
	for (int i = 0; i < minDataNum; ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_data[i], c2.m_data[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
	r_timeDiff *= zoomFactor;

	//���㹦�ʲ�в��������
	for (int i = 1; i < minDataNum; ++i) {
		RtLbsType powerDiff1 = c1.m_data[i].m_power - c1.m_data[0].m_power;
		RtLbsType powerDiff2 = c2.m_data[i].m_power - c2.m_data[0].m_power;
		r_powerdiff += (powerDiff2 - powerDiff1) * (powerDiff2 - powerDiff1);
	}
	r_powerdiff *= zoomFactor;
}

//�������鴫�������ݵĲв������-TDOA��λ����-������
inline void CalculateSensorCollectionResidual_TDOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0.0;
		RtLbsType cur_r_powerDiff = 0.0;
		CalculateSensorResidual_TDOA_MultiData(c1[i], c2[i], cur_r_timeDiff, cur_r_powerDiff);
		r_timeDiff += cur_r_timeDiff;
		r_powerDiff += cur_r_powerDiff;
	}
}


//�����������������ݵĲв������-TDOA/AOA��λ����-������
inline void CalculateSensorResidual_TDOA_AOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff)
{
	//����ǶȲв��������
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateResidual_AOA(c1.m_data[i], c2.m_data[i], cur_r_phi);
		r_phi += cur_r_phi;
	}

	//����ʱ�Ӳ�в��������
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_data[i], c2.m_data[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
}


//�������鴫�������ݵĲв������-TDOA/AOA��λ����-������
inline void CalculateSensorCollectionResidual_TDOA_AOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff)
{
	//����ʱ�Ӳ�ǶȲв��������
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		RtLbsType cur_r_timeDiff = 0;
		CalculateSensorResidual_TDOA_AOA_SingleData(c1[i], c2[i], cur_r_phi, cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}

	//���㹦�ʲ�в��������
	for (int i = 1; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType powerDiff1 = c1[i].m_data[0].m_power - c1[0].m_data[0].m_power;
		RtLbsType powerDiff2 = c2[i].m_data[0].m_power - c2[0].m_data[0].m_power;
		r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
	}
}

//�����������������ݵĲв������-TDOA/AOA��λ����-������
inline void CalculateSensorResidual_TDOA_AOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerdiff)
{
	//ȡc1��c2���������ٵ���Ϊ�Ƚ�
	int minDataNum = static_cast<int>(std::min(c1.m_data.size(), c1.m_data.size()));
	int zoomFactor = 1;																					/** @brief	�в�����ϵ������c2��ֻ�в�������c1����������ϵ������	*/
	zoomFactor = static_cast<int>(c1.m_data.size()) - minDataNum;										//������ݾͻ�Ŵ󼸱�

	//����ǶȲв��������
	for (int i = 0; i < minDataNum; ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateResidual_AOA(c1.m_data[i], c2.m_data[i], cur_r_phi);
		r_phi += cur_r_phi;
	}
	r_phi *= zoomFactor;

	//����ʱ�Ӳ�в��������
	for (int i = 0; i < minDataNum; ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_data[i], c2.m_data[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
	r_timeDiff *= zoomFactor;

	//���㹦�ʲ�в��������
	for (int i = 1; i < minDataNum; ++i) {
		RtLbsType powerDiff1 = c1.m_data[i].m_power - c1.m_data[0].m_power;
		RtLbsType powerDiff2 = c2.m_data[i].m_power - c2.m_data[0].m_power;
		r_powerdiff += (powerDiff2 - powerDiff1) * (powerDiff2 - powerDiff1);
	}
	r_powerdiff *= zoomFactor;
}

//�������鴫�������ݵĲв������-TDOA/AOA��λ����-������
inline void CalculateSensorCollectionResidual_TDOA_AOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0.0;
		RtLbsType cur_r_timeDiff = 0.0;
		RtLbsType cur_r_powerDiff = 0.0;
		CalculateSensorResidual_TDOA_AOA_MultiData(c1[i], c2[i], cur_r_phi, cur_r_timeDiff, cur_r_powerDiff);
		r_timeDiff += cur_r_timeDiff;
		r_powerDiff += cur_r_powerDiff;
	}
}





#endif
