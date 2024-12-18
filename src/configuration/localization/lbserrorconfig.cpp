#include "lbserrorconfig.h"

LBSErrorConfig::LBSErrorConfig()
	: m_errorType(LBSERRORTYPE_AOA)
{
}

LBSErrorConfig::~LBSErrorConfig()
{
}

void LBSErrorConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_LBSERRORCONFIG_ERRORTYPE.c_str());										SerializeEnum(m_errorType, writer);
	writer.Key(KEY_LBSERRORCONFIG_PHIDEGREEERRORSIGMAS.c_str());							SerializeArray(m_phiDegreeErrorSigmas, writer);
	writer.Key(KEY_LBSERRORCONFIG_TIMENSERRORSIGMAS.c_str());								SerializeArray(m_timeNSErrorSigmas, writer);
	writer.EndObject();
}

bool LBSErrorConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "LBSErrorConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_LBSERRORCONFIG_ERRORTYPE.c_str())) {
		LOG_ERROR << "LBSErrorConfig: missing " << KEY_LBSERRORCONFIG_ERRORTYPE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LBSERRORCONFIG_PHIDEGREEERRORSIGMAS.c_str())) {
		LOG_ERROR << "LBSErrorConfig: missing " << KEY_LBSERRORCONFIG_PHIDEGREEERRORSIGMAS.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LBSERRORCONFIG_TIMENSERRORSIGMAS.c_str())) {
		LOG_ERROR << "LBSErrorConfig: missing " << KEY_LBSERRORCONFIG_TIMENSERRORSIGMAS.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& errorTypeValue = value[KEY_LBSERRORCONFIG_ERRORTYPE.c_str()];
	const rapidjson::Value& phiDegreeErrorSigmasValue = value[KEY_LBSERRORCONFIG_PHIDEGREEERRORSIGMAS.c_str()];
	const rapidjson::Value& timeNSErrorSigmasValue = value[KEY_LBSERRORCONFIG_TIMENSERRORSIGMAS.c_str()];

	if (!errorTypeValue.IsInt()) {
		LOG_ERROR << "LBSErrorConfig: " << KEY_LBSERRORCONFIG_ERRORTYPE.c_str() << " wrong value format." << ENDL;
		return false;
	}
	if (!phiDegreeErrorSigmasValue.IsArray()) {
		LOG_ERROR << "LBSErrorConfig: " << KEY_LBSERRORCONFIG_PHIDEGREEERRORSIGMAS.c_str() << " wrong value format." << ENDL;
		return false;
	}
	if (!timeNSErrorSigmasValue.IsArray()) {
		LOG_ERROR << "LBSErrorConfig: " << KEY_LBSERRORCONFIG_TIMENSERRORSIGMAS.c_str() << " wrong value format." << ENDL;
		return false;
	}

	if (!DeserializeEnum(m_errorType, errorTypeValue)) {
		LOG_ERROR << "SolvingConfig: " << KEY_LBSERRORCONFIG_ERRORTYPE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!DeserializeArray(m_phiDegreeErrorSigmas, phiDegreeErrorSigmasValue)) {
		LOG_ERROR << "SolvingConfig: " << KEY_LBSERRORCONFIG_PHIDEGREEERRORSIGMAS.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!DeserializeArray(m_timeNSErrorSigmas, timeNSErrorSigmasValue)) {
		LOG_ERROR << "SolvingConfig: " << KEY_LBSERRORCONFIG_TIMENSERRORSIGMAS.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	//将degree转换为弧度
	m_phiErrorSigmas.reserve(m_phiDegreeErrorSigmas.size());
	for (auto& phi : m_phiDegreeErrorSigmas) {
		m_phiErrorSigmas.push_back(phi * ONE_DEGEREE);
	}

	//将timeNs转换为距离
	m_timeMErrorSigmas.reserve(m_timeNSErrorSigmas.size());
	for (auto& timeNS : m_timeNSErrorSigmas) {
		m_timeMErrorSigmas.push_back(timeNS * LIGHT_DISTANCE_PER_NS);
	}

	return true;
}
