#ifndef RTLBS_RECEIVERUNITCONFIG
#define RTLBS_RECEIVERUNITCONFIG

#include "rtlbs.h"
#include "utility/enum.h"
#include "utility/serializable.h"
#include "utility/define.h"
#include "geometry/point3d.h"
#include "antenna/antenna.h"

const std::string KEY_RECEIVERUNITCONFIG_ANTID = "AntID";
const std::string KEY_RECEIVERUNITCONFIG_ANTNAME = "AntName";
const std::string KEY_RECEIVERUNITCONFIG_INSERTLOSS = "InsertLoss";
const std::string KEY_RECEIVERUNITCONFIG_ATTACHGAIN = "AttachGain";
const std::string KEY_RECEIVERUNITCONFIG_POWERSHRESHOLD = "PowerShreshold";
const std::string KEY_RECEIVERUNITCONFIG_POSITION = "Position";
const std::string KEY_RECEIVERUNITCONFIG_POSTURE = "Posture";
const std::string KEY_RECEIVERUNITCONFIG_VELOCITY = "Velocity";


class ReceiverUnitConfig : public Serializable {
public:
	int m_antId;						/** @brief	��������ID	*/
	std::string m_antName;				/** @brief	��������	*/
	RtLbsType m_insertLoss;				/** @brief	�������	*/
	RtLbsType m_attachGain;				/** @brief	�������� dB	*/
	RtLbsType m_powerShreshold;			/** @brief	���ջ���͵�ƽʶ�𣬵�λ��dBm	*/
	Point3D m_position;					/** @brief	λ����Ϣ	*/
	Euler m_posture;					/** @brief	��̬��Ϣ	*/
	RtLbsType m_velocity;				/** @brief	�ٶ���Ϣ	*/

public:
	ReceiverUnitConfig();
	ReceiverUnitConfig(const Point3D& p);											//ͨ���������г�ʼ��
	ReceiverUnitConfig(const ReceiverUnitConfig& config);
	~ReceiverUnitConfig();
	ReceiverUnitConfig& operator = (const ReceiverUnitConfig& config);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
