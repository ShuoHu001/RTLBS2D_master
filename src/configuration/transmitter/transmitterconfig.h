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
	unsigned m_antId;					/** @brief	发射天线编号	*/
	std::string m_antName;				/** @brief	发射端所用天线名称	*/
	RtLbsType m_power;					/** @brief	发射端功率/W	*/
	RtLbsType m_interLoss;				/** @brief	发射端插入损耗	*/
	RtLbsType m_attachGain;				/** @brief	发射端附加增益	*/
	Point3D m_position;					/** @brief	发射天线坐标	*/
	Euler m_posture;					/** @brief	发射天线姿态	*/
	RtLbsType m_velocity;				/** @brief	发射天线移动速度	*/

public:
	TransmitterConfig();
	TransmitterConfig(const TransmitterConfig& config);
	~TransmitterConfig();
	TransmitterConfig& operator = (const TransmitterConfig& config);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

bool Init_TransmitterConfigs(const std::string& filename, std::vector<TransmitterConfig>& configs);						//初始化发射机配置
void Write2Json_TransmitterConfigs(const std::string& filename, const std::vector<TransmitterConfig>& configs);			//序列化发射机配置至文件

#endif
