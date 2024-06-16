#ifndef RTLBS_SINGLECONFIG
#define RTLBS_SINGLECONFIG

#include "rtlbs.h"
#include "geometry/point3d.h"
#include "utility/serializable.h"
#include "receiverunitconfig.h"

const std::string KEY_SINGLECONFIG_POSITION = "Position";
const std::string KEY_SINGLECONFIG_VELOCITY = "Velocity";

class SingleConfig :public Serializable {
public:
	Point3D m_position;						/** @brief	λ����Ϣ	*/
	RtLbsType m_velocity;					/** @brief	�ٶ���Ϣ	*/
public:
	SingleConfig();
	SingleConfig(const SingleConfig& config);
	~SingleConfig();
	SingleConfig& operator = (const SingleConfig& config);
	void CalculateRxPosition(std::vector<ReceiverUnitConfig>& configs);

public:
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
