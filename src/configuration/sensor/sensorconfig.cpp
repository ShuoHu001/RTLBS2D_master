#include "sensorconfig.h"

SensorConfig::SensorConfig()
	: m_id(0)
	, m_antId(0)
	, m_antName("0-Omni")
	, m_insertLoss(0.0)
	, m_attachGain(0.0)
	, m_phiDegreeErrorSTD(0.0)
	, m_phiErrorSTD(0.0)
	, m_timeErrorSTD(0.0)
{

}

SensorConfig::SensorConfig(const SensorConfig& config)
	: m_id(config.m_id)
	, m_antId(config.m_antId)
	, m_antName(config.m_antName)
	, m_position(config.m_position)
	, m_insertLoss(config.m_insertLoss)
	, m_attachGain(config.m_attachGain)
	, m_phiDegreeErrorSTD(config.m_phiDegreeErrorSTD)
	, m_phiErrorSTD(config.m_phiErrorSTD)
	, m_timeErrorSTD(config.m_timeErrorSTD)
	, m_sensorDataFileName(config.m_sensorDataFileName)
{
}

SensorConfig::~SensorConfig()
{
}

SensorConfig& SensorConfig::operator=(const SensorConfig& config)
{
	m_id = config.m_id;
	m_antId = config.m_antId;
	m_antName = config.m_antName;
	m_position = config.m_position;
	m_insertLoss = config.m_insertLoss;
	m_attachGain = config.m_attachGain;
	m_phiDegreeErrorSTD = config.m_phiDegreeErrorSTD;
	m_phiErrorSTD = config.m_phiErrorSTD;
	m_timeErrorSTD = config.m_timeErrorSTD;
	m_sensorDataFileName = config.m_sensorDataFileName;
	return *this;
}

void SensorConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_SENSORCONFIG_ID.c_str());											writer.Uint(m_id);
	writer.Key(KEY_SENSORCONFIG_ANTID.c_str());											writer.Uint(m_antId);
	writer.Key(KEY_SENSORCONFIG_ANTNAME.c_str());										writer.String(m_antName.c_str());
	writer.Key(KEY_SENSORCONFIG_POSITION.c_str());										m_position.Serialize(writer);
	writer.Key(KEY_SENSORCONFIG_INSERTLOSS.c_str());									writer.Double(m_insertLoss);
	writer.Key(KEY_SENSORCONFIG_ATTACHGAIN.c_str());									writer.Double(m_attachGain);
	writer.Key(KEY_SENSORCONFIG_THETADEGREEERRORSTD.c_str());							writer.Double(m_phiDegreeErrorSTD);
	writer.Key(KEY_SENSORCONFIG_TIMEERRORSTD.c_str());									writer.Double(m_timeErrorSTD);
	writer.Key(KEY_SENSORCONFIG_SENSORDATAFILENAME.c_str());							writer.String(m_sensorDataFileName.c_str());
	writer.EndObject();
}

bool SensorConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "SensorConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_SENSORCONFIG_ID.c_str())) {
		LOG_ERROR << "SensorConfig: missing " << KEY_SENSORCONFIG_ID.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SENSORCONFIG_ANTID.c_str())) {
		LOG_ERROR << "SensorConfig: missing " << KEY_SENSORCONFIG_ANTID.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SENSORCONFIG_ANTNAME.c_str())) {
		LOG_ERROR << "SensorConfig: missing " << KEY_SENSORCONFIG_ANTNAME.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SENSORCONFIG_POSITION.c_str())) {
		LOG_ERROR << "SensorConfig: missing " << KEY_SENSORCONFIG_POSITION.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SENSORCONFIG_INSERTLOSS.c_str())) {
		LOG_ERROR << "SensorConfig: missing " << KEY_SENSORCONFIG_INSERTLOSS.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SENSORCONFIG_ATTACHGAIN.c_str())) {
		LOG_ERROR << "SensorConfig: missing " << KEY_SENSORCONFIG_ATTACHGAIN.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SENSORCONFIG_THETADEGREEERRORSTD.c_str())) {
		LOG_ERROR << "SensorConfig: missing " << KEY_SENSORCONFIG_THETADEGREEERRORSTD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SENSORCONFIG_TIMEERRORSTD.c_str())) {
		LOG_ERROR << "SensorConfig: missing " << KEY_SENSORCONFIG_TIMEERRORSTD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SENSORCONFIG_SENSORDATAFILENAME.c_str())) {
		LOG_ERROR << "SensorConfig: missing " << KEY_SENSORCONFIG_SENSORDATAFILENAME.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& idValue = value[KEY_SENSORCONFIG_ID.c_str()];
	const rapidjson::Value& antIdValue = value[KEY_SENSORCONFIG_ANTID.c_str()];
	const rapidjson::Value& antNameValue = value[KEY_SENSORCONFIG_ANTNAME.c_str()];
	const rapidjson::Value& positionValue = value[KEY_SENSORCONFIG_POSITION.c_str()];
	const rapidjson::Value& insertLossValue = value[KEY_SENSORCONFIG_INSERTLOSS.c_str()];
	const rapidjson::Value& attachGainValue = value[KEY_SENSORCONFIG_ATTACHGAIN.c_str()];
	const rapidjson::Value& phiDegreeErrorSTDValue = value[KEY_SENSORCONFIG_THETADEGREEERRORSTD.c_str()];
	const rapidjson::Value& timeErrorSTDValue = value[KEY_SENSORCONFIG_TIMEERRORSTD.c_str()];
	const rapidjson::Value& sensorDataFileNameValue = value[KEY_SENSORCONFIG_SENSORDATAFILENAME.c_str()];

	if (!idValue.IsUint()) {
		LOG_ERROR << "SensorConfig: " << KEY_SENSORCONFIG_ID.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!antIdValue.IsUint()) {
		LOG_ERROR << "SensorConfig: " << KEY_SENSORCONFIG_ANTID.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!antNameValue.IsString()) {
		LOG_ERROR << "SensorConfig: " << KEY_SENSORCONFIG_ANTNAME.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!positionValue.IsObject()) {
		LOG_ERROR << "SensorConfig: " << KEY_SENSORCONFIG_POSITION.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!insertLossValue.IsDouble()) {
		LOG_ERROR << "SensorConfig: " << KEY_SENSORCONFIG_INSERTLOSS.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!attachGainValue.IsDouble()) {
		LOG_ERROR << "SensorConfig: " << KEY_SENSORCONFIG_ATTACHGAIN.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!phiDegreeErrorSTDValue.IsDouble()) {
		LOG_ERROR << "SensorConfig: " << KEY_SENSORCONFIG_THETADEGREEERRORSTD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!timeErrorSTDValue.IsDouble()) {
		LOG_ERROR << "SensorConfig: " << KEY_SENSORCONFIG_TIMEERRORSTD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!sensorDataFileNameValue.IsString()) {
		LOG_ERROR << "SensorConfig: " << KEY_SENSORCONFIG_SENSORDATAFILENAME.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_id = idValue.GetUint();
	m_antId = antIdValue.GetUint();
	m_antName = antNameValue.GetString();
	m_insertLoss = insertLossValue.GetDouble();
	m_attachGain = attachGainValue.GetDouble();
	m_phiDegreeErrorSTD = phiDegreeErrorSTDValue.GetDouble();
	m_timeErrorSTD = timeErrorSTDValue.GetDouble();
	m_sensorDataFileName = sensorDataFileNameValue.GetString();

	m_phiErrorSTD = (m_phiDegreeErrorSTD / 180.0) * PI;

	if (!m_position.Deserialize(positionValue)) {
		LOG_ERROR << "SensorConfig: " << KEY_SENSORCONFIG_POSITION.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	return true;
}
