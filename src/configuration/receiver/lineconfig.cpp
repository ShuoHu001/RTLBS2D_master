#include "lineconfig.h"




SingleRoute::SingleRoute()
	: m_routeId(0)
	, m_interval(1.0)
	, m_velocity(0.0)
{
}

SingleRoute::SingleRoute(const SingleRoute& config)
	: m_routeId(config.m_routeId)
	, m_startPoint(config.m_startPoint)
	, m_endPoint(config.m_endPoint)
	, m_interval(config.m_interval)
	, m_velocity(config.m_velocity)
{
}

SingleRoute::~SingleRoute()
{
}

SingleRoute& SingleRoute::operator=(const SingleRoute& config)
{
	m_routeId = config.m_routeId;
	m_startPoint = config.m_startPoint;
	m_endPoint = config.m_endPoint;
	m_interval = config.m_interval;
	m_velocity = config.m_velocity;
	return *this;
}

void SingleRoute::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_SINGLEROUTECONFIG_ROUTEID.c_str());									writer.Uint(m_routeId);
	writer.Key(KEY_SINGLEROUTECONFIG_STARTPOINT.c_str());								m_startPoint.Serialize(writer);
	writer.Key(KEY_SINGLEROUTECONFIG_ENDPOINT.c_str());									m_endPoint.Serialize(writer);
	writer.Key(KEY_SINGLEROUTECONFIG_INTERVAL.c_str());									writer.Double(m_interval);
	writer.Key(KEY_SINGLEROUTECONFIG_VELOCITY.c_str());									writer.Double(m_velocity);
	writer.EndObject();
}

bool SingleRoute::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "SingleRoute: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_SINGLEROUTECONFIG_ROUTEID.c_str())) {
		LOG_ERROR << "SingleRoute: missing " << KEY_SINGLEROUTECONFIG_ROUTEID.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SINGLEROUTECONFIG_STARTPOINT.c_str())) {
		LOG_ERROR << "SingleRoute: missing " << KEY_SINGLEROUTECONFIG_STARTPOINT.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SINGLEROUTECONFIG_ENDPOINT.c_str())) {
		LOG_ERROR << "SingleRoute: missing " << KEY_SINGLEROUTECONFIG_ENDPOINT.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SINGLEROUTECONFIG_INTERVAL.c_str())) {
		LOG_ERROR << "SingleRoute: missing " << KEY_SINGLEROUTECONFIG_INTERVAL.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SINGLEROUTECONFIG_VELOCITY.c_str())) {
		LOG_ERROR << "SingleRoute: missing " << KEY_SINGLEROUTECONFIG_VELOCITY.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& routeIdValue = value[KEY_SINGLEROUTECONFIG_ROUTEID.c_str()];
	const rapidjson::Value& startPointValue = value[KEY_SINGLEROUTECONFIG_STARTPOINT.c_str()];
	const rapidjson::Value& endPointValue = value[KEY_SINGLEROUTECONFIG_ENDPOINT.c_str()];
	const rapidjson::Value& intervalValue = value[KEY_SINGLEROUTECONFIG_INTERVAL.c_str()];
	const rapidjson::Value& velocityValue = value[KEY_SINGLEROUTECONFIG_VELOCITY.c_str()];

	if(!routeIdValue.IsUint()){
		LOG_ERROR << "SingleRoute: " << KEY_SINGLEROUTECONFIG_ROUTEID.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!startPointValue.IsObject()) {
		LOG_ERROR << "SingleRoute: " << KEY_SINGLEROUTECONFIG_STARTPOINT.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!endPointValue.IsObject()) {
		LOG_ERROR << "SingleRoute: " << KEY_SINGLEROUTECONFIG_ENDPOINT.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!intervalValue.IsDouble()) {
		LOG_ERROR << "SingleRoute: " << KEY_SINGLEROUTECONFIG_INTERVAL.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!velocityValue.IsDouble()) {
		LOG_ERROR << "SingleRoute: " << KEY_SINGLEROUTECONFIG_VELOCITY.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	m_routeId = routeIdValue.GetUint();
	m_interval = intervalValue.GetDouble();
	m_velocity = velocityValue.GetDouble();

	if (!m_startPoint.Deserialize(startPointValue)) {
		LOG_ERROR << "SingleRoute: " << KEY_SINGLEROUTECONFIG_STARTPOINT.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_endPoint.Deserialize(endPointValue)) {
		LOG_ERROR << "SingleRoute: " << KEY_SINGLEROUTECONFIG_ENDPOINT.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	return true;
}

void SingleRoute::CalculateRxPosition(std::vector<ReceiverUnitConfig>& configs) const
{
	Vector3D increment = m_endPoint - m_startPoint;
	RtLbsType routeLen = increment.Length();
	configs.push_back(m_startPoint);
	if (m_interval <= 0 || m_interval > routeLen) { //非正常情况，只增加收尾两点
		if (routeLen == 0) {
			return;
		}
		configs.push_back(m_endPoint);
		return;
	}
	int stepNum = static_cast<int>(floor(routeLen / m_interval));
	Vector3D increUnit = increment / routeLen;
	for (int i = 1; i <= stepNum; ++i) {
		Point3D p = m_startPoint + increUnit * i;
		configs.push_back(p);
	}
	return;
}

LineConfig::LineConfig()
{
}

LineConfig::LineConfig(const LineConfig& config)
	: m_routes(config.m_routes)
{
}

LineConfig::~LineConfig()
{
}

LineConfig& LineConfig::operator=(const LineConfig& config)
{
	m_routes = config.m_routes;
	return *this;
}

void LineConfig::CalculateRxPosition(std::vector<ReceiverUnitConfig>& configs) const
{
	configs.clear(); // 清空原有的点位置
	for (auto it = m_routes.begin(); it != m_routes.end(); ++it) {
		(*it).CalculateRxPosition(configs);
	}
	return;
}

void LineConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_LINECONFIG_ROUTES.c_str()); SerializeArray(m_routes, writer);
	writer.EndObject();
}

bool LineConfig::Deserialize(const rapidjson::Value& value)
{
	if (value.IsObject()) {
		if (value.HasMember(KEY_LINECONFIG_ROUTES.c_str())) {
			const rapidjson::Value& routesValue = value[KEY_LINECONFIG_ROUTES.c_str()];
			if (routesValue.IsArray()) {
				return DeserializeArray(m_routes, routesValue);
			}
		}
	}
	return false;
}

