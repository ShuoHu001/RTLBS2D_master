#ifndef RTLBS_SOLIDCONFIG
#define RTLBS_SOLIDCONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "math/point3d.h"
#include "receiverunitconfig.h"

const std::string KEY_SOLIDCONFIG_XMIN = "XMin";
const std::string KEY_SOLIDCONFIG_XMAX = "XMax";
const std::string KEY_SOLIDCONFIG_YMIN = "YMin";
const std::string KEY_SOLIDCONFIG_YMAX = "YMax";
const std::string KEY_SOLIDCONFIG_ZMIN = "ZMin";
const std::string KEY_SOLIDCONFIG_ZMAX = "ZMax";
const std::string KEY_SOLIDCONFIG_XINTERVAL = "XInterval";
const std::string KEY_SOLIDCONFIG_YINTERVAL = "YInterval";
const std::string KEY_SOLIDCONFIG_ZINTERVAL = "ZInterval";
const std::string KEY_SOLIDCONFIG_VELOCITY = "Velocity";

class SolidConfig :public Serializable {
public:
	RtLbsType m_xMin;					/** @brief	X 方向最小值	*/
	RtLbsType m_xMax;					/** @brief	X 方向最大值	*/
	RtLbsType m_yMin;					/** @brief	Y 方向最小值	*/
	RtLbsType m_yMax;					/** @brief	Y 方向最大值	*/
	RtLbsType m_zMin;					/** @brief	Z 方向最小值	*/
	RtLbsType m_zMax;					/** @brief	Z 方向最大值	*/
	RtLbsType m_xInterval;				/** @brief	X 方向间隔	*/
	RtLbsType m_yInterval;				/** @brief	Y 方向间隔	*/
	RtLbsType m_zInterval;				/** @brief	Z 方向间隔	*/
	RtLbsType m_velocity;				/** @brief	速度信息	*/

public:
	SolidConfig();
	SolidConfig(const SolidConfig& config);
	~SolidConfig();
	SolidConfig& operator = (const SolidConfig& config);
	void CalculateRxPosition(std::vector<ReceiverUnitConfig>& configs) const;
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
