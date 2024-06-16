#include "solidconfig.h"


SolidConfig::SolidConfig()
	: m_xMin(0.0)
	, m_xMax(100.0)
	, m_yMin(0.0)
	, m_yMax(100.0)
	, m_zMin(0.0)
	, m_zMax(50.0)
	, m_xInterval(10)
	, m_yInterval(10)
	, m_zInterval(10)
	, m_velocity(0.0)
{
}

SolidConfig::SolidConfig(const SolidConfig& config)
	: m_xMin(config.m_xMin)
	, m_xMax(config.m_xMax)
	, m_yMin(config.m_yMin)
	, m_yMax(config.m_yMax)
	, m_zMin(config.m_zMin)
	, m_zMax(config.m_zMax)
	, m_xInterval(config.m_xInterval)
	, m_yInterval(config.m_yInterval)
	, m_zInterval(config.m_zInterval)
	, m_velocity(config.m_velocity)
{
}

SolidConfig::~SolidConfig()
{
}

SolidConfig& SolidConfig::operator=(const SolidConfig& config)
{
	m_xMin = config.m_xMin;
	m_xMax = config.m_xMax;
	m_yMin = config.m_yMin;
	m_yMax = config.m_yMax;
	m_zMin = config.m_zMin;
	m_zMax = config.m_zMax;
	m_xInterval = config.m_xInterval;
	m_yInterval = config.m_yInterval;
	m_zInterval = config.m_zInterval;
	m_velocity = config.m_velocity;
	return *this;
}

void SolidConfig::CalculateRxPosition(std::vector<ReceiverUnitConfig>& configs) const
{
	if (m_xInterval <= 0 || m_yInterval <= 0 || m_zInterval <= 0)
		return;
	for (RtLbsType x = m_xMin; x <= m_xMax; x += m_xInterval) {
		for (RtLbsType y = m_yMin; y <= m_yMax; y += m_yInterval) {
			for (RtLbsType z = m_zMin; z <= m_zMax; z += m_zInterval) {
				Point3D p(x, y, z);
				configs.push_back(p);
			}
		}
	}

}

void SolidConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_SOLIDCONFIG_XMIN.c_str());											writer.Double(m_xMin);
	writer.Key(KEY_SOLIDCONFIG_XMAX.c_str());											writer.Double(m_xMax);
	writer.Key(KEY_SOLIDCONFIG_YMIN.c_str());											writer.Double(m_yMin);
	writer.Key(KEY_SOLIDCONFIG_YMAX.c_str());											writer.Double(m_yMax);
	writer.Key(KEY_SOLIDCONFIG_ZMIN.c_str());											writer.Double(m_zMin);
	writer.Key(KEY_SOLIDCONFIG_ZMAX.c_str());											writer.Double(m_zMax);
	writer.Key(KEY_SOLIDCONFIG_XINTERVAL.c_str());										writer.Double(m_xInterval);
	writer.Key(KEY_SOLIDCONFIG_YINTERVAL.c_str());										writer.Double(m_yInterval);
	writer.Key(KEY_SOLIDCONFIG_ZINTERVAL.c_str());										writer.Double(m_zInterval);
	writer.Key(KEY_SOLIDCONFIG_VELOCITY.c_str());										writer.Double(m_velocity);
	writer.EndObject();
}

bool SolidConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "SolidConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_SOLIDCONFIG_XMIN.c_str())) {
		LOG_ERROR << "SolidConfig: missing " << KEY_SOLIDCONFIG_XMIN.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SOLIDCONFIG_XMAX.c_str())) {
		LOG_ERROR << "SolidConfig: missing " << KEY_SOLIDCONFIG_XMAX.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SOLIDCONFIG_YMIN.c_str())) {
		LOG_ERROR << "SolidConfig: missing " << KEY_SOLIDCONFIG_YMIN.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SOLIDCONFIG_YMAX.c_str())) {
		LOG_ERROR << "SolidConfig: missing " << KEY_SOLIDCONFIG_YMAX.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SOLIDCONFIG_ZMIN.c_str())) {
		LOG_ERROR << "SolidConfig: missing " << KEY_SOLIDCONFIG_ZMIN.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SOLIDCONFIG_ZMAX.c_str())) {
		LOG_ERROR << "SolidConfig: missing " << KEY_SOLIDCONFIG_ZMAX.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SOLIDCONFIG_XINTERVAL.c_str())) {
		LOG_ERROR << "SolidConfig: missing " << KEY_SOLIDCONFIG_XINTERVAL.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SOLIDCONFIG_YINTERVAL.c_str())) {
		LOG_ERROR << "SolidConfig: missing " << KEY_SOLIDCONFIG_YINTERVAL.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SOLIDCONFIG_ZINTERVAL.c_str())) {
		LOG_ERROR << "SolidConfig: missing " << KEY_SOLIDCONFIG_ZINTERVAL.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SOLIDCONFIG_VELOCITY.c_str())) {
		LOG_ERROR << "SolidConfig: missing " << KEY_SOLIDCONFIG_VELOCITY.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& xMinValue = value[KEY_SOLIDCONFIG_XMIN.c_str()];
	const rapidjson::Value& xMaxValue = value[KEY_SOLIDCONFIG_XMAX.c_str()];
	const rapidjson::Value& yMinValue = value[KEY_SOLIDCONFIG_YMIN.c_str()];
	const rapidjson::Value& yMaxValue = value[KEY_SOLIDCONFIG_YMAX.c_str()];
	const rapidjson::Value& zMinValue = value[KEY_SOLIDCONFIG_ZMIN.c_str()];
	const rapidjson::Value& zMaxValue = value[KEY_SOLIDCONFIG_ZMAX.c_str()];
	const rapidjson::Value& xIntervalValue = value[KEY_SOLIDCONFIG_XINTERVAL.c_str()];
	const rapidjson::Value& yIntervalValue = value[KEY_SOLIDCONFIG_YINTERVAL.c_str()];
	const rapidjson::Value& zIntervalValue = value[KEY_SOLIDCONFIG_ZINTERVAL.c_str()];
	const rapidjson::Value& velocityValue = value[KEY_SOLIDCONFIG_VELOCITY.c_str()];

	if (!xMinValue.IsDouble()) {
		LOG_ERROR << "SolidConfig: " << KEY_SOLIDCONFIG_XMIN.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!xMaxValue.IsDouble()) {
		LOG_ERROR << "SolidConfig: " << KEY_SOLIDCONFIG_XMAX.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!yMinValue.IsDouble()) {
		LOG_ERROR << "SolidConfig: " << KEY_SOLIDCONFIG_YMIN.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!yMaxValue.IsDouble()) {
		LOG_ERROR << "SolidConfig: " << KEY_SOLIDCONFIG_YMAX.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!zMinValue.IsDouble()) {
		LOG_ERROR << "SolidConfig: " << KEY_SOLIDCONFIG_ZMIN.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!zMaxValue.IsDouble()) {
		LOG_ERROR << "SolidConfig: " << KEY_SOLIDCONFIG_ZMAX.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!xIntervalValue.IsDouble()) {
		LOG_ERROR << "SolidConfig: " << KEY_SOLIDCONFIG_XINTERVAL.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!yIntervalValue.IsDouble()) {
		LOG_ERROR << "SolidConfig: " << KEY_SOLIDCONFIG_YINTERVAL.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!zIntervalValue.IsDouble()) {
		LOG_ERROR << "SolidConfig: " << KEY_SOLIDCONFIG_ZINTERVAL.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!velocityValue.IsDouble()) {
		LOG_ERROR << "SolidConfig: " << KEY_SOLIDCONFIG_VELOCITY.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_xMin = xMinValue.GetDouble();
	m_xMax = xMaxValue.GetDouble();
	m_yMin = zMinValue.GetDouble();
	m_yMax = zMaxValue.GetDouble();
	m_zMin = zMinValue.GetDouble();
	m_zMax = zMaxValue.GetDouble();
	m_xInterval = xIntervalValue.GetDouble();
	m_yInterval = yIntervalValue.GetDouble();
	m_zInterval = zIntervalValue.GetDouble();
	m_velocity = velocityValue.GetDouble();

	return true;
}
