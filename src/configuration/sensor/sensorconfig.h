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
	unsigned m_id;									/** @brief	������ID	*/
	unsigned m_antId;								/** @brief	����������ID	*/
	std::string m_antName;							/** @brief	��������������	*/
	Point3D m_position;								/** @brief	������λ��	*/
	RtLbsType m_insertLoss;							/** @brief	�������������	*/
	RtLbsType m_attachGain;							/** @brief	��������������	*/
	RtLbsType m_phiErrorSTD;						/** @brief	�������ǶȲ�������׼�� ��λ����	*/
	RtLbsType m_timeErrorSTD;						/** @brief	�������ǶȲ�������׼�� ��λns	*/
	RtLbsType m_powerErrorSTD;						/** @brief	���������ʲ�������׼�� ��λdB	*/
	std::string m_sensorDataFileName;				/** @brief	��������Ӧ������	*/
	RtLbsType m_phiDegreeErrorSTD;					/** @brief	�������ǶȲ�������׼�� ��λ��	*/

public:
	SensorConfig();
	SensorConfig(const SensorConfig& config);
	~SensorConfig();
	SensorConfig& operator = (const SensorConfig& config);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
