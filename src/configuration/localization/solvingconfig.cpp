#include "solvingconfig.h"

SolvingConfig::SolvingConfig()
	: m_solvingMode(SOLVING_WIRLS)
	, m_lossType(LOSS_CAUCHY)
	, m_iterNum(20)
	, m_tolerance(1e-6)
{
}

SolvingConfig::SolvingConfig(const SolvingConfig& config)
	: m_solvingMode(config.m_solvingMode)
	, m_lossType(config.m_lossType)
	, m_iterNum(config.m_iterNum)
	, m_tolerance(config.m_tolerance)
{
}

SolvingConfig::~SolvingConfig()
{
}

SolvingConfig& SolvingConfig::operator=(const SolvingConfig& config)
{
	m_solvingMode = config.m_solvingMode;
	m_lossType = config.m_lossType;
	m_iterNum = config.m_iterNum;
	m_tolerance = config.m_tolerance;
	return *this;
}

void SolvingConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_SOLVINGCONFIG_SOLVINGMODE.c_str());							SerializeEnum(m_solvingMode, writer);
	writer.Key(KEY_SOLVINGCONFIG_LOSSTYPE.c_str());								SerializeEnum(m_lossType, writer);
	writer.Key(KEY_SOLVINGCONFIG_ITERNUM.c_str());								writer.Int(m_iterNum);
	writer.Key(KEY_SOLVINGCONFIG_TOLERANCE.c_str());							writer.Double(m_tolerance);
	writer.EndObject();
}

bool SolvingConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "SolvingConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_SOLVINGCONFIG_SOLVINGMODE.c_str())) {
		LOG_ERROR << "SolvingConfig: missing " << KEY_SOLVINGCONFIG_SOLVINGMODE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SOLVINGCONFIG_LOSSTYPE.c_str())) {
		LOG_ERROR << "SolvingConfig: missing " << KEY_SOLVINGCONFIG_LOSSTYPE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SOLVINGCONFIG_ITERNUM.c_str())) {
		LOG_ERROR << "SolvingConfig: missing " << KEY_SOLVINGCONFIG_ITERNUM.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SOLVINGCONFIG_TOLERANCE.c_str())) {
		LOG_ERROR << "SolvingConfig: missing " << KEY_SOLVINGCONFIG_TOLERANCE.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& solvingModeValue = value[KEY_SOLVINGCONFIG_SOLVINGMODE.c_str()];
	const rapidjson::Value& lossTypeValue = value[KEY_SOLVINGCONFIG_LOSSTYPE.c_str()];
	const rapidjson::Value& iterNumValue = value[KEY_SOLVINGCONFIG_ITERNUM.c_str()];
	const rapidjson::Value& toleranceValue = value[KEY_SOLVINGCONFIG_TOLERANCE.c_str()];

	if (!solvingModeValue.IsInt()) {
		LOG_ERROR << "SolvingConfig: " << KEY_SOLVINGCONFIG_SOLVINGMODE.c_str() << " wrong value format." << ENDL;
		return false;
	}
	if (!lossTypeValue.IsInt()) {
		LOG_ERROR << "SolvingConfig: " << KEY_SOLVINGCONFIG_LOSSTYPE.c_str() << " wrong value format." << ENDL;
		return false;
	}
	if (!iterNumValue.IsInt()) {
		LOG_ERROR << "SolvingConfig: " << KEY_SOLVINGCONFIG_ITERNUM.c_str() << " wrong value format." << ENDL;
		return false;
	}
	if (!toleranceValue.IsDouble()) {
		LOG_ERROR << "SolvingConfig: " << KEY_SOLVINGCONFIG_TOLERANCE.c_str() << " wrong value format." << ENDL;
		return false;
	}

	m_iterNum = iterNumValue.GetInt();
	m_tolerance = toleranceValue.GetDouble();

	if (!DeserializeEnum(m_solvingMode, solvingModeValue)) {
		LOG_ERROR << "SolvingConfig: " << KEY_SOLVINGCONFIG_SOLVINGMODE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!DeserializeEnum(m_lossType, lossTypeValue)) {
		LOG_ERROR << "SolvingConfig: " << KEY_SOLVINGCONFIG_LOSSTYPE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	return true;
}
