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
	int m_antId;						/** @brief	天线类型ID	*/
	std::string m_antName;				/** @brief	天线名称	*/
	RtLbsType m_insertLoss;				/** @brief	插入损耗	*/
	RtLbsType m_attachGain;				/** @brief	附加增益 dB	*/
	RtLbsType m_powerShreshold;			/** @brief	接收机最低电平识别，单位：dBm	*/
	Point3D m_position;					/** @brief	位置信息	*/
	Euler m_posture;					/** @brief	姿态信息	*/
	RtLbsType m_velocity;				/** @brief	速度信息	*/

public:
	ReceiverUnitConfig();
	ReceiverUnitConfig(const Point3D& p);											//通过点来进行初始化
	ReceiverUnitConfig(const ReceiverUnitConfig& config);
	~ReceiverUnitConfig();
	ReceiverUnitConfig& operator = (const ReceiverUnitConfig& config);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
