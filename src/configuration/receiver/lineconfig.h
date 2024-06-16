#ifndef RTLBS_LINECONFIG
#define RTLBS_LINECONFIG

#include "rtlbs.h"
#include "geometry/point3d.h"
#include "geometry/vector3d.h"
#include "utility/serializable.h"
#include "utility/define.h"
#include "receiverunitconfig.h"


const std::string KEY_SINGLEROUTECONFIG_ROUTEID = "Id";
const std::string KEY_SINGLEROUTECONFIG_STARTPOINT = "StartPoint";
const std::string KEY_SINGLEROUTECONFIG_ENDPOINT = "EndPoint";
const std::string KEY_SINGLEROUTECONFIG_INTERVAL = "Interval";
const std::string KEY_SINGLEROUTECONFIG_VELOCITY = "Velocity";

const std::string KEY_LINECONFIG_ROUTES = "Routes";

class SingleRoute :public Serializable {
public:
	unsigned m_routeId;			/** @brief	路径Id	*/
	Point3D m_startPoint;		/** @brief	路径起始点	*/
	Point3D m_endPoint;			/** @brief	路径终止点	*/
	double m_interval;			/** @brief	路径间隔	*/
	RtLbsType m_velocity;		/** @brief	路径上的平均速度	*/
public:
	SingleRoute();
	SingleRoute(const SingleRoute& config);
	~SingleRoute();
	SingleRoute& operator = (const SingleRoute& config);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
	void CalculateRxPosition(std::vector<ReceiverUnitConfig>& configs) const;
};


class LineConfig :public Serializable {
public:
	std::vector<SingleRoute> m_routes;

public:
	LineConfig();
	LineConfig(const LineConfig& config);
	~LineConfig();
	LineConfig& operator = (const LineConfig& config);
	void CalculateRxPosition(std::vector<ReceiverUnitConfig>& configs) const;
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};




#endif
