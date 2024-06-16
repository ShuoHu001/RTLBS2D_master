#include "antennaconfig.h"

AntennaConfig::AntennaConfig()
	: m_antId(-1)
	, m_typeId(0)
	, m_antName("Omni-Antenna")
	, m_typicalGain(0)
	, m_freqMin(18e9)
	, m_freqMax(1e7)
	, m_patternFileName("")
{
}

AntennaConfig::AntennaConfig(const AntennaConfig& config)
	: m_antId(config.m_antId)
	, m_typeId(config.m_typeId)
	, m_antName(config.m_antName)
	, m_typicalGain(config.m_typicalGain)
	, m_freqMin(config.m_freqMin)
	, m_freqMax(config.m_freqMax)
	, m_polarization(config.m_polarization)
	, m_posture(config.m_posture)
	, m_patternFileName(config.m_patternFileName)
{
}

AntennaConfig::~AntennaConfig()
{
}

AntennaConfig& AntennaConfig::operator=(const AntennaConfig& config)
{
	m_antId = config.m_antId;
	m_typeId = config.m_typeId;
	m_antName = config.m_antName;
	m_typicalGain = config.m_typicalGain;
	m_freqMin = config.m_freqMin;
	m_freqMax = config.m_freqMax;
	m_polarization = config.m_polarization;
	m_posture = config.m_posture;
	m_patternFileName = config.m_patternFileName;
	return *this;
}

void AntennaConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_ANTENNACONFIG_ANTID.c_str());											writer.Int(m_antId);
	writer.Key(KEY_ANTENNACONFIG_TYPEID.c_str());											writer.Int(m_typeId);
	writer.Key(KEY_ANTENNACONFIG_ANTNAME.c_str());											writer.String(m_antName.c_str());
	writer.Key(KEY_ANTENNACONFIG_TYPECALGAIN.c_str());										writer.Double(m_typicalGain);
	writer.Key(KEY_ANTENNACONFIG_FREQMIN.c_str());											writer.Double(m_freqMin);
	writer.Key(KEY_ANTENNACONFIG_FREQMAX.c_str());											writer.Double(m_freqMax);
	writer.Key(KEY_ANTENNACONFIG_POLARIZATION.c_str());										m_polarization.Serialize(writer);
	writer.Key(KEY_ANTENNACONFIG_POSTURE.c_str());											m_posture.Serialize(writer);
	writer.Key(KEY_ANTENNACONFIG_PATTERNFILENAME.c_str());									writer.String(m_patternFileName.c_str());
	writer.EndObject();
}

bool AntennaConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "AntennaConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_ANTENNACONFIG_ANTID.c_str())) {
		LOG_ERROR << "AntennaConfig: missing " << KEY_ANTENNACONFIG_ANTID.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_ANTENNACONFIG_TYPEID.c_str())) {
		LOG_ERROR << "AntennaConfig: missing " << KEY_ANTENNACONFIG_TYPEID.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_ANTENNACONFIG_ANTNAME.c_str())) {
		LOG_ERROR << "AntennaConfig: missing " << KEY_ANTENNACONFIG_ANTNAME.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_ANTENNACONFIG_TYPECALGAIN.c_str())) {
		LOG_ERROR << "AntennaConfig: missing " << KEY_ANTENNACONFIG_TYPECALGAIN.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_ANTENNACONFIG_FREQMIN.c_str())) {
		LOG_ERROR << "AntennaConfig: missing " << KEY_ANTENNACONFIG_FREQMIN.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_ANTENNACONFIG_FREQMAX.c_str())) {
		LOG_ERROR << "AntennaConfig: missing " << KEY_ANTENNACONFIG_FREQMAX.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_ANTENNACONFIG_POLARIZATION.c_str())) {
		LOG_ERROR << "AntennaConfig: missing " << KEY_ANTENNACONFIG_POLARIZATION.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_ANTENNACONFIG_POSTURE.c_str())) {
		LOG_ERROR << "AntennaConfig: missing " << KEY_ANTENNACONFIG_POSTURE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_ANTENNACONFIG_PATTERNFILENAME.c_str())) {
		LOG_ERROR << "AntennaConfig: missing " << KEY_ANTENNACONFIG_PATTERNFILENAME.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& antIdValue = value[KEY_ANTENNACONFIG_ANTID.c_str()];
	const rapidjson::Value& typeIdValue = value[KEY_ANTENNACONFIG_TYPEID.c_str()];
	const rapidjson::Value& antNameValue = value[KEY_ANTENNACONFIG_ANTNAME.c_str()];
	const rapidjson::Value& typicalGainValue = value[KEY_ANTENNACONFIG_TYPECALGAIN.c_str()];
	const rapidjson::Value& freqMinValue = value[KEY_ANTENNACONFIG_FREQMIN.c_str()];
	const rapidjson::Value& freqMaxValue = value[KEY_ANTENNACONFIG_FREQMAX.c_str()];
	const rapidjson::Value& polarizationValue = value[KEY_ANTENNACONFIG_POLARIZATION.c_str()];
	const rapidjson::Value& postureValue = value[KEY_ANTENNACONFIG_POSTURE.c_str()];
	const rapidjson::Value& patternFileNameValue = value[KEY_ANTENNACONFIG_PATTERNFILENAME.c_str()];

	if (!antIdValue.IsInt()) {
		LOG_ERROR << "AntennaConfig: " << KEY_ANTENNACONFIG_ANTID.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!typeIdValue.IsInt()) {
		LOG_ERROR << "AntennaConfig: " << KEY_ANTENNACONFIG_TYPEID.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!antNameValue.IsString()) {
		LOG_ERROR << "AntennaConfig: " << KEY_ANTENNACONFIG_ANTNAME.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!typicalGainValue.IsDouble()) {
		LOG_ERROR << "AntennaConfig: " << KEY_ANTENNACONFIG_TYPECALGAIN.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!freqMinValue.IsDouble()) {
		LOG_ERROR << "AntennaConfig: " << KEY_ANTENNACONFIG_FREQMIN.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!freqMaxValue.IsDouble()) {
		LOG_ERROR << "AntennaConfig: " << KEY_ANTENNACONFIG_FREQMAX.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!polarizationValue.IsObject()) {
		LOG_ERROR << "AntennaConfig: " << KEY_ANTENNACONFIG_POLARIZATION.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!postureValue.IsObject()) {
		LOG_ERROR << "AntennaConfig: " << KEY_ANTENNACONFIG_POSTURE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!patternFileNameValue.IsString()) {
		LOG_ERROR << "AntennaConfig: " << KEY_ANTENNACONFIG_PATTERNFILENAME.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_antId = antIdValue.GetInt();
	m_typeId = typeIdValue.GetInt();
	m_antName = antNameValue.GetString();
	m_typicalGain = typicalGainValue.GetDouble();
	m_freqMin = freqMinValue.GetDouble();
	m_freqMax = freqMaxValue.GetDouble();
	m_patternFileName = patternFileNameValue.GetString();

	if (!m_polarization.Deserialize(polarizationValue)) {
		LOG_ERROR << "AntennaConfig: " << KEY_ANTENNACONFIG_POLARIZATION.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_posture.Deserialize(postureValue)) {
		LOG_ERROR << "AntennaConfig: " << KEY_ANTENNACONFIG_POSTURE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	
	return true;
}
