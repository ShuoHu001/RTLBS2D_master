#ifndef RTLBS_PLANECONFIG
#define RTLBS_PLANECONFIG

#include "math/point3d.h"
#include "utility/serializable.h"
#include "utility/define.h"
#include "receiverunitconfig.h"

const std::string KEY_PLANECONFIG_XINTERVAL = "XInterval";
const std::string KEY_PLANECONFIG_YINTERVAL = "YInterval";
const std::string KEY_PLANECONFIG_XMIN = "XMin";
const std::string KEY_PLANECONFIG_XMAX = "XMax";
const std::string KEY_PLANECONFIG_YMIN = "YMin";
const std::string KEY_PLANECONFIG_YMAX = "YMax";
const std::string KEY_PLANECONFIG_HEIGHT = "Height";
const std::string KEY_PLANECONFIG_VELOCITY = "Velocity";

class PlaneConfig : public Serializable {
public:
	RtLbsType m_xMin;			/** @brief	X 方向最小	*/
	RtLbsType m_xMax;			/** @brief	X 方向最大	*/
	RtLbsType m_yMin;			/** @brief	Y 方向最小	*/
	RtLbsType m_yMax;			/** @brief	Y 方向最大	*/
	RtLbsType m_xInterval;		/** @brief	面型接收机预测时X方向间隔	*/
	RtLbsType m_yInterval;		/** @brief	面型接收机预测时Y方向间隔	*/
	RtLbsType m_height;			/** @brief	预测平面的高度	*/
	RtLbsType m_velocity;		/** @brief	移动速度	*/

public:
	PlaneConfig();
	PlaneConfig(const PlaneConfig& config);
	~PlaneConfig();
	PlaneConfig& operator = (const PlaneConfig& config);
	void CalculateRxPosition(std::vector<ReceiverUnitConfig>& configs);

public:
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
