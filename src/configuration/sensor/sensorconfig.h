#ifndef RTLBS_SENSORCONFIG
#define RTLBS_SENSORCONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"
#include "geometry/point2d.h"
#include "geometry/point3d.h"

const std::string KEY_SENSORCONFIG_ID = "Id";
const std::string KEY_SENSORCONFIG_ANTID = "AntId";
const std::string KEY_SENSORCONFIG_ANTNAME = "AntName";
const std::string KEY_SENSORCONFIG_POSITION = "Position";
const std::string KEY_SENSORCONFIG_INSERTLOSS = "InsertLoss";
const std::string KEY_SENSORCONFIG_ATTACHGAIN = "AttachGain";
const std::string KEY_SENSORCONFIG_PHIDEGREEERRORSTD = "PhiDegreeErrorSTD";
const std::string KEY_SENSORCONFIG_TIMEERRORSTD = "TimeErrorSTD";
const std::string KEY_SENSORCONFIG_POWERERRORSTD = "PowerErrorSTD";
const std::string KEY_SENSORCONFIG_SENSORDATAFILENAME = "SensorDataFileName";

class SensorConfig {
public:
	unsigned m_id;									/** @brief	传感器ID	*/
	unsigned m_antId;								/** @brief	传感器天线ID	*/
	std::string m_antName;							/** @brief	传感器天线名称	*/
	Point3D m_position;								/** @brief	传感器位置	*/
	RtLbsType m_insertLoss;							/** @brief	传感器插入损耗	*/
	RtLbsType m_attachGain;							/** @brief	传感器附加增益	*/
	RtLbsType m_phiErrorSTD;						/** @brief	传感器角度测量误差标准差 单位弧度	*/
	RtLbsType m_timeErrorSTD;						/** @brief	传感器角度测量误差标准差 单位ns	*/
	RtLbsType m_powerErrorSTD;						/** @brief	传感器功率测量误差标准差 单位dB	*/
	std::string m_sensorDataFileName;				/** @brief	传感器对应的数据	*/
	RtLbsType m_phiDegreeErrorSTD;					/** @brief	传感器角度测量误差标准差 单位°	*/

public:
	SensorConfig();
	SensorConfig(const SensorConfig& config);
	~SensorConfig();
	SensorConfig& operator = (const SensorConfig& config);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
