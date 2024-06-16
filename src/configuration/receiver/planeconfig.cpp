#include "planeconfig.h"

PlaneConfig::PlaneConfig()
	: m_xInterval(10.0)
	, m_yInterval(10.0)
	, m_xMin(0.0)
	, m_xMax(100.0)
	, m_yMin(0.0)
	, m_yMax(100.0)
	, m_height(2.0)
	, m_velocity(0.0)
{
}

PlaneConfig::PlaneConfig(const PlaneConfig& config)
	: m_xMin(config.m_xMin)
	, m_xMax(config.m_xMax)
	, m_yMin(config.m_yMin)
	, m_yMax(config.m_yMax)
	, m_xInterval(config.m_xInterval)
	, m_yInterval(config.m_yInterval)
	, m_height(config.m_height)
	, m_velocity(config.m_velocity)
{
}

PlaneConfig::~PlaneConfig()
{
}

PlaneConfig& PlaneConfig::operator=(const PlaneConfig& config)
{
	m_xMin = config.m_xMin;
	m_xMax = config.m_xMax;
	m_yMin = config.m_yMin;
	m_yMax = config.m_yMax;
	m_xInterval = config.m_xInterval;
	m_yInterval = config.m_yInterval;
	m_height = config.m_height;
	m_velocity = config.m_velocity;
	return *this;
}

void PlaneConfig::CalculateRxPosition(std::vector<ReceiverUnitConfig>& configs)
{
	// 清空原来的数据
	configs.clear();

	// 计算X和Y方向的点数
	size_t xCount = static_cast<size_t>((m_xMax - m_xMin) / m_xInterval) + 1;
	size_t yCount = static_cast<size_t>((m_yMax - m_yMin) / m_yInterval) + 1;

	// 计算接收机的实际坐标
	for (size_t i = 0; i < yCount; ++i) {
		for (size_t j = 0; j < xCount; ++j) {
			RtLbsType x = m_xMin + j * m_xInterval;
			RtLbsType y = m_yMin + i * m_yInterval;
			configs.push_back(Point3D(x, y, m_height));
		}
	}
}

void PlaneConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_PLANECONFIG_XINTERVAL.c_str());										writer.Double(m_xInterval);
	writer.Key(KEY_PLANECONFIG_YINTERVAL.c_str());										writer.Double(m_yInterval);
	writer.Key(KEY_PLANECONFIG_XMIN.c_str());											writer.Double(m_xMin);
	writer.Key(KEY_PLANECONFIG_XMAX.c_str());											writer.Double(m_xMax);
	writer.Key(KEY_PLANECONFIG_YMIN.c_str());											writer.Double(m_yMin);
	writer.Key(KEY_PLANECONFIG_YMAX.c_str());											writer.Double(m_yMax);
	writer.Key(KEY_PLANECONFIG_HEIGHT.c_str());											writer.Double(m_height);
	writer.Key(KEY_PLANECONFIG_VELOCITY.c_str());										writer.Double(m_velocity);
	writer.EndObject();
}

bool PlaneConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "PlaneConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_PLANECONFIG_XINTERVAL.c_str())) {
		LOG_ERROR << "PlaneConfig: missing " << KEY_PLANECONFIG_XINTERVAL.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_PLANECONFIG_YINTERVAL.c_str())) {
		LOG_ERROR << "PlaneConfig: missing " << KEY_PLANECONFIG_YINTERVAL.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_PLANECONFIG_XMIN.c_str())) {
		LOG_ERROR << "PlaneConfig: missing " << KEY_PLANECONFIG_XMIN.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_PLANECONFIG_XMAX.c_str())) {
		LOG_ERROR << "PlaneConfig: missing " << KEY_PLANECONFIG_XMAX.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_PLANECONFIG_YMIN.c_str())) {
		LOG_ERROR << "PlaneConfig: missing " << KEY_PLANECONFIG_YMIN.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_PLANECONFIG_YMAX.c_str())) {
		LOG_ERROR << "PlaneConfig: missing " << KEY_PLANECONFIG_YMAX.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_PLANECONFIG_HEIGHT.c_str())) {
		LOG_ERROR << "PlaneConfig: missing " << KEY_PLANECONFIG_HEIGHT.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_PLANECONFIG_VELOCITY.c_str())) {
		LOG_ERROR << "PlaneConfig: missing " << KEY_PLANECONFIG_VELOCITY.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& xIntervalValue = value[KEY_PLANECONFIG_XINTERVAL.c_str()];
	const rapidjson::Value& yIntervalValue = value[KEY_PLANECONFIG_YINTERVAL.c_str()];
	const rapidjson::Value& xMinValue = value[KEY_PLANECONFIG_XMIN.c_str()];
	const rapidjson::Value& xMaxValue = value[KEY_PLANECONFIG_XMAX.c_str()];
	const rapidjson::Value& yMinValue = value[KEY_PLANECONFIG_YMIN.c_str()];
	const rapidjson::Value& yMaxValue = value[KEY_PLANECONFIG_YMAX.c_str()];
	const rapidjson::Value& heightValue = value[KEY_PLANECONFIG_HEIGHT.c_str()];
	const rapidjson::Value& velocityValue = value[KEY_PLANECONFIG_VELOCITY.c_str()];

	if (!xIntervalValue.IsDouble()) {
		LOG_ERROR << "PlaneConfig: " << KEY_PLANECONFIG_XINTERVAL.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!yIntervalValue.IsDouble()) {
		LOG_ERROR << "PlaneConfig: " << KEY_PLANECONFIG_YINTERVAL.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!xMinValue.IsDouble()) {
		LOG_ERROR << "PlaneConfig: " << KEY_PLANECONFIG_XMIN.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!xMaxValue.IsDouble()) {
		LOG_ERROR << "PlaneConfig: " << KEY_PLANECONFIG_XMAX.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!yMinValue.IsDouble()) {
		LOG_ERROR << "PlaneConfig: " << KEY_PLANECONFIG_YMIN.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!yMaxValue.IsDouble()) {
		LOG_ERROR << "PlaneConfig: " << KEY_PLANECONFIG_YMAX.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!heightValue.IsDouble()) {
		LOG_ERROR << "PlaneConfig: " << KEY_PLANECONFIG_HEIGHT.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!velocityValue.IsDouble()) {
		LOG_ERROR << "PlaneConfig: " << KEY_PLANECONFIG_VELOCITY.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_xInterval = static_cast<RtLbsType>(xIntervalValue.GetDouble());
	m_yInterval = static_cast<RtLbsType>(yIntervalValue.GetDouble());
	m_xMin = static_cast<RtLbsType>(xMinValue.GetDouble());
	m_xMax = static_cast<RtLbsType>(xMaxValue.GetDouble());
	m_yMin = static_cast<RtLbsType>(yMinValue.GetDouble());
	m_yMax = static_cast<RtLbsType>(yMaxValue.GetDouble());
	m_height = static_cast<RtLbsType>(heightValue.GetDouble());
	m_velocity = static_cast<RtLbsType>(velocityValue.GetDouble());

	return true;
}
