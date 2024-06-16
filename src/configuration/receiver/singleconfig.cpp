#include "singleconfig.h"



SingleConfig::SingleConfig()
	: m_velocity(0.0)
{
}

SingleConfig::SingleConfig(const SingleConfig& config)
	: m_position(config.m_position)
	, m_velocity(config.m_velocity)
{
}

SingleConfig::~SingleConfig()
{
}

SingleConfig& SingleConfig::operator=(const SingleConfig& config)
{
	m_position = config.m_position;
	m_velocity = config.m_velocity;
	return *this;
}

void SingleConfig::CalculateRxPosition(std::vector<ReceiverUnitConfig>& configs)
{
	configs.clear();
	configs.push_back(m_position);
}

void SingleConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_SINGLECONFIG_POSITION.c_str());										m_position.Serialize(writer);
	writer.Key(KEY_SINGLECONFIG_VELOCITY.c_str());										writer.Double(m_velocity);
	writer.EndObject();
}

bool SingleConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "SingleConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_SINGLECONFIG_POSITION.c_str())) {
		LOG_ERROR << "SingleConfig: missing " << KEY_SINGLECONFIG_POSITION.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SINGLECONFIG_VELOCITY.c_str())) {
		LOG_ERROR << "SingleConfig: missing " << KEY_SINGLECONFIG_VELOCITY.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& positionValue = value[KEY_SINGLECONFIG_POSITION.c_str()];
	const rapidjson::Value& velocityValue = value[KEY_SINGLECONFIG_VELOCITY.c_str()];

	if (!positionValue.IsObject()) {
		LOG_ERROR << "SingleConfig: " << KEY_SINGLECONFIG_POSITION.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!velocityValue.IsDouble()) {
		LOG_ERROR << "SingleConfig: " << KEY_SINGLECONFIG_VELOCITY.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_velocity = velocityValue.GetDouble();

	if (!m_position.Deserialize(positionValue)) {
		LOG_ERROR << "SingleConfig: " << KEY_SINGLECONFIG_POSITION.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	
	return true;
}
