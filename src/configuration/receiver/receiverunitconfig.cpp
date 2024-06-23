#include "receiverunitconfig.h"

ReceiverUnitConfig::ReceiverUnitConfig()
	: m_antId(0)
	, m_antName("全向天线")
	, m_insertLoss(0.0)
	, m_attachGain(0.0)
	, m_powerShreshold(-160.0)
	, m_angularThreshold(1.0)
	, m_delayThreshold(5.0e-9)
	, m_velocity(0.0)
{
}

ReceiverUnitConfig::ReceiverUnitConfig(const Point3D& p)
	: m_antId(0)
	, m_antName("omni")
	, m_insertLoss(0.0)
	, m_attachGain(0.0)
	, m_powerShreshold(-160.0)
	, m_angularThreshold(1.0)
	, m_delayThreshold(5.0e-9)
	, m_velocity(0.0)
{
	m_position = p;
}

ReceiverUnitConfig::ReceiverUnitConfig(const ReceiverUnitConfig& config)
	: m_antId(config.m_antId)
	, m_antName(config.m_antName)
	, m_insertLoss(config.m_insertLoss)
	, m_attachGain(config.m_attachGain)
	, m_position(config.m_position)
	, m_posture(config.m_posture)
	, m_powerShreshold(config.m_powerShreshold)
	, m_angularThreshold(config.m_angularThreshold)
	, m_delayThreshold(config.m_delayThreshold)
	, m_velocity(config.m_velocity)
{
}

ReceiverUnitConfig::~ReceiverUnitConfig()
{
}

ReceiverUnitConfig& ReceiverUnitConfig::operator=(const ReceiverUnitConfig& config)
{
	m_antId = config.m_antId;
	m_antName = config.m_antName;
	m_insertLoss = config.m_insertLoss;
	m_attachGain = config.m_attachGain;
	m_powerShreshold = config.m_powerShreshold;
	m_angularThreshold = config.m_angularThreshold;
	m_delayThreshold = config.m_delayThreshold;
	m_position = config.m_position;
	m_posture = config.m_posture;
	m_velocity = config.m_velocity;
	return *this;
}

void ReceiverUnitConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_RECEIVERUNITCONFIG_ANTID.c_str());											writer.Int(m_antId);
	writer.Key(KEY_RECEIVERUNITCONFIG_ANTNAME.c_str());											writer.String(m_antName.c_str());
	writer.Key(KEY_RECEIVERUNITCONFIG_INSERTLOSS.c_str());										writer.Double(m_insertLoss);
	writer.Key(KEY_RECEIVERUNITCONFIG_ATTACHGAIN.c_str());										writer.Double(m_attachGain);
	writer.Key(KEY_RECEIVERUNITCONFIG_POWERTHRESHOLD.c_str());									writer.Double(m_powerShreshold);
	writer.Key(KEY_RECEIVERUNITCONFIG_ANGULARTHRESHOLD.c_str());								writer.Double(m_angularThreshold);
	writer.Key(KEY_RECEIVERUNITCONFIG_DELAYTHRESHOLD.c_str());									writer.Double(m_delayThreshold);
	writer.Key(KEY_RECEIVERUNITCONFIG_POSITION.c_str());										m_position.Serialize(writer);
	writer.Key(KEY_RECEIVERUNITCONFIG_POSTURE.c_str());											m_posture.Serialize(writer);
	writer.Key(KEY_RECEIVERUNITCONFIG_VELOCITY.c_str());										writer.Double(m_velocity);
	writer.EndObject();
}

bool ReceiverUnitConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "ReceiverUnitConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_RECEIVERUNITCONFIG_ANTID.c_str())) {
		LOG_ERROR << "ReceiverUnitConfig: missing " << KEY_RECEIVERUNITCONFIG_ANTID.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERUNITCONFIG_ANTNAME.c_str())) {
		LOG_ERROR << "ReceiverUnitConfig: missing " << KEY_RECEIVERUNITCONFIG_ANTNAME.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERUNITCONFIG_INSERTLOSS.c_str())) {
		LOG_ERROR << "ReceiverUnitConfig: missing " << KEY_RECEIVERUNITCONFIG_INSERTLOSS.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERUNITCONFIG_ATTACHGAIN.c_str())) {
		LOG_ERROR << "ReceiverUnitConfig: missing " << KEY_RECEIVERUNITCONFIG_ATTACHGAIN.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERUNITCONFIG_POWERTHRESHOLD.c_str())) {
		LOG_ERROR << "ReceiverUnitConfig: missing " << KEY_RECEIVERUNITCONFIG_POWERTHRESHOLD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERUNITCONFIG_ANGULARTHRESHOLD.c_str())) {
		LOG_ERROR << "ReceiverUnitConfig: missing " << KEY_RECEIVERUNITCONFIG_ANGULARTHRESHOLD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERUNITCONFIG_DELAYTHRESHOLD.c_str())) {
		LOG_ERROR << "ReceiverUnitConfig: missing " << KEY_RECEIVERUNITCONFIG_DELAYTHRESHOLD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERUNITCONFIG_POSITION.c_str())) {
		LOG_ERROR << "ReceiverUnitConfig: missing " << KEY_RECEIVERUNITCONFIG_POSITION.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERUNITCONFIG_POSTURE.c_str())) {
		LOG_ERROR << "ReceiverUnitConfig: missing " << KEY_RECEIVERUNITCONFIG_POSTURE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERUNITCONFIG_VELOCITY.c_str())) {
		LOG_ERROR << "ReceiverUnitConfig: missing " << KEY_RECEIVERUNITCONFIG_VELOCITY.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& antIdValue = value[KEY_RECEIVERUNITCONFIG_ANTID.c_str()];
	const rapidjson::Value& antNameValue = value[KEY_RECEIVERUNITCONFIG_ANTNAME.c_str()];
	const rapidjson::Value& insertLossValue = value[KEY_RECEIVERUNITCONFIG_INSERTLOSS.c_str()];
	const rapidjson::Value& attachGainValue = value[KEY_RECEIVERUNITCONFIG_ATTACHGAIN.c_str()];
	const rapidjson::Value& powerThresholdValue = value[KEY_RECEIVERUNITCONFIG_POWERTHRESHOLD.c_str()];
	const rapidjson::Value& angularThresholdValue = value[KEY_RECEIVERUNITCONFIG_ANGULARTHRESHOLD.c_str()];
	const rapidjson::Value& delayThresholdValue = value[KEY_RECEIVERUNITCONFIG_DELAYTHRESHOLD.c_str()];
	const rapidjson::Value& positionValue = value[KEY_RECEIVERUNITCONFIG_POSITION.c_str()];
	const rapidjson::Value& postureValue = value[KEY_RECEIVERUNITCONFIG_POSTURE.c_str()];
	const rapidjson::Value& velocityValue = value[KEY_RECEIVERUNITCONFIG_VELOCITY.c_str()];

	if (!antIdValue.IsInt()) {
		LOG_ERROR << "ReceiverUnitConfig: " << KEY_RECEIVERUNITCONFIG_ANTID.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!antNameValue.IsString()) {
		LOG_ERROR << "ReceiverUnitConfig: " << KEY_RECEIVERUNITCONFIG_ANTNAME.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!insertLossValue.IsDouble()) {
		LOG_ERROR << "ReceiverUnitConfig: " << KEY_RECEIVERUNITCONFIG_INSERTLOSS.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!attachGainValue.IsDouble()) {
		LOG_ERROR << "ReceiverUnitConfig: " << KEY_RECEIVERUNITCONFIG_ATTACHGAIN.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!powerThresholdValue.IsDouble()) {
		LOG_ERROR << "ReceiverUnitConfig: " << KEY_RECEIVERUNITCONFIG_POWERTHRESHOLD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!angularThresholdValue.IsDouble()) {
		LOG_ERROR << "ReceiverUnitConfig: " << KEY_RECEIVERUNITCONFIG_ANGULARTHRESHOLD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!delayThresholdValue.IsDouble()) {
		LOG_ERROR << "ReceiverUnitConfig: " << KEY_RECEIVERUNITCONFIG_DELAYTHRESHOLD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!positionValue.IsObject()) {
		LOG_ERROR << "ReceiverUnitConfig: " << KEY_RECEIVERUNITCONFIG_POSITION.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!postureValue.IsObject()) {
		LOG_ERROR << "ReceiverUnitConfig: " << KEY_RECEIVERUNITCONFIG_POSTURE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!velocityValue.IsDouble()) {
		LOG_ERROR << "ReceiverUnitConfig: " << KEY_RECEIVERUNITCONFIG_VELOCITY.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_antId = antIdValue.GetInt();
	m_antName = antNameValue.GetString();
	m_insertLoss = insertLossValue.GetDouble();
	m_attachGain = attachGainValue.GetDouble();
	m_powerShreshold = powerThresholdValue.GetDouble();
	m_angularThreshold = angularThresholdValue.GetDouble();
	m_delayThreshold = delayThresholdValue.GetDouble();
	m_velocity = velocityValue.GetDouble();

	if (!m_position.Deserialize(positionValue)) {
		LOG_ERROR << "ReceiverUnitConfig: " << KEY_RECEIVERUNITCONFIG_POSITION.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_posture.Deserialize(postureValue)) {
		LOG_ERROR << "ReceiverUnitConfig: " << KEY_RECEIVERUNITCONFIG_POSTURE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	return true;
}
