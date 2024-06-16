#include "localizeconfig.h"

LocalizeConfig::LocalizeConfig()
	: m_lbsMode(LBS_MODE_SPSTMD)
	, m_lbsMethod(LBS_METHOD_RT_AOA)
{
}

LocalizeConfig::LocalizeConfig(const LocalizeConfig& config)
	: m_lbsMode(config.m_lbsMode)
	, m_lbsMethod(config.m_lbsMethod)
{
}

LocalizeConfig::~LocalizeConfig()
{
}

LocalizeConfig& LocalizeConfig::operator=(const LocalizeConfig& config)
{
	m_lbsMode = config.m_lbsMode;
	m_lbsMethod = config.m_lbsMethod;
	return *this;
}

void LocalizeConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE.c_str());							SerializeEnum(m_lbsMode, writer);
	writer.Key(KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD.c_str());							SerializeEnum(m_lbsMethod, writer);
	writer.EndObject();
}

bool LocalizeConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "LocalizeConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE.c_str())) {
		LOG_ERROR << "LocalizeConfig: missing " << KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD.c_str())) {
		LOG_ERROR << "LocalizeConfig: missing " << KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& m_lbsModeValue = value[KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE.c_str()];
	const rapidjson::Value& m_lbsMethodValue = value[KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD.c_str()];

	if (!m_lbsModeValue.IsInt()) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!m_lbsMethodValue.IsInt()) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	if (!DeserializeEnum(m_lbsMode, m_lbsModeValue)) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!DeserializeEnum(m_lbsMethod, m_lbsMethodValue)) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	if (!IsValid()) {
		return false;
	}

	return true;
}

bool LocalizeConfig::IsValid() const
{
	if (m_lbsMode == LBS_MODE_SPSTMD) {
		if (m_lbsMethod == LBS_METHOD_RT_AOA_TDOA || m_lbsMethod == LBS_METHOD_RT_TDOA) {
			LOG_ERROR << "LocalizeConfig: SPSTMD not support TDOA method." << ENDL;
			return false;
		}
	}
	else if (m_lbsMode == LBS_MODE_SPMTMD) {
		if (m_lbsMethod == LBS_METHOD_RT_AOA_TDOA || m_lbsMethod == LBS_METHOD_RT_TDOA) {
			LOG_ERROR << "LocalizeConfig: SPMTMD not support TDOA method." << ENDL;
			return false;
		}
	}
	else if (m_lbsMode == LBS_MODE_MPMTMD) {
		if (m_lbsMethod == LBS_METHOD_RT_AOA_TDOA || m_lbsMethod == LBS_METHOD_RT_TDOA) {
			LOG_ERROR << "LocalizeConfig: MPMTMD not support TDOA method." << ENDL;
			return false;
		}
	}
	return true;
}
