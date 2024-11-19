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
	RtLbsType m_xMin;			/** @brief	X ������С	*/
	RtLbsType m_xMax;			/** @brief	X �������	*/
	RtLbsType m_yMin;			/** @brief	Y ������С	*/
	RtLbsType m_yMax;			/** @brief	Y �������	*/
	RtLbsType m_xInterval;		/** @brief	���ͽ��ջ�Ԥ��ʱX������	*/
	RtLbsType m_yInterval;		/** @brief	���ͽ��ջ�Ԥ��ʱY������	*/
	RtLbsType m_height;			/** @brief	Ԥ��ƽ��ĸ߶�	*/
	RtLbsType m_velocity;		/** @brief	�ƶ��ٶ�	*/

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
