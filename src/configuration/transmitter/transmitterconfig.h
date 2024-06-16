#ifndef RTLBS_TRANSMITTERCONFIG
#define RTLBS_TRANSMITTERCONFIG

#include "antenna/antenna.h"
#include "utility/enum.h"
#include "geometry/point3d.h"
#include "utility/serializable.h"
#include "utility/define.h"

const std::string KEY_TRANSMITTERCONFIG_ANTID = "AntId";
const std::string KEY_TRANSMITTERCONFIG_ANTNAME = "AntName";
const std::string KEY_TRANSMITTERCONFIG_POWER = "Power";
const std::string KEY_TRANSMITTERCONFIG_INTERLOSS = "InsertLoss";
const std::string KEY_TRANSMITTERCONFIG_ATTACHGAIN = "AttachGain";
const std::string KEY_TRANSMITTERCONFIG_POSITION = "Position";
const std::string KEY_TRANSMITTERCONFIG_POSTURE = "Posture";
const std::string KEY_TRANSMITTERCONFIG_VELOCITY = "Velocity";
const std::string KEY_TRANSMITTERCONFIG = "TransmitterConfig";


class TransmitterConfig :public Serializable {
public:
	unsigned m_antId;					/** @brief	�������߱��	*/
	std::string m_antName;				/** @brief	�����������������	*/
	RtLbsType m_power;					/** @brief	����˹���/W	*/
	RtLbsType m_interLoss;				/** @brief	����˲������	*/
	RtLbsType m_attachGain;				/** @brief	����˸�������	*/
	Point3D m_position;					/** @brief	������������	*/
	Euler m_posture;					/** @brief	����������̬	*/
	RtLbsType m_velocity;				/** @brief	���������ƶ��ٶ�	*/

public:
	TransmitterConfig();
	TransmitterConfig(const TransmitterConfig& config);
	~TransmitterConfig();
	TransmitterConfig& operator = (const TransmitterConfig& config);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

bool Init_TransmitterConfigs(const std::string& filename, std::vector<TransmitterConfig>& configs);						//��ʼ�����������
void Write2Json_TransmitterConfigs(const std::string& filename, const std::vector<TransmitterConfig>& configs);			//���л�������������ļ�

#endif
