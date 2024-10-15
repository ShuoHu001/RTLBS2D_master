#include "materialconfig.h"

MaterialConfig::MaterialConfig()
	: m_id(-1)
	, m_name("")
	, m_permittivity(0.0)
	, m_conductivity(0.0)
	, m_penetrationLoss(5.0)
{
}

MaterialConfig::MaterialConfig(int matId, const std::string& name, RtLbsType permittivity, RtLbsType conductivity)
	: m_id(matId)
	, m_name(name)
	, m_permittivity(permittivity)
	, m_conductivity(conductivity)
	, m_penetrationLoss(5.0)
{
}

MaterialConfig::MaterialConfig(int matId, const std::string& name, RtLbsType permittivity, RtLbsType conductivity, RtLbsType penetractionLoss)
	: m_id(matId)
	, m_name(name)
	, m_permittivity(permittivity)
	, m_conductivity(conductivity)
	, m_penetrationLoss(penetractionLoss)
{
}

MaterialConfig::MaterialConfig(const MaterialConfig& mat)
	: m_id(mat.m_id)
	, m_name(mat.m_name)
	, m_permittivity(mat.m_permittivity)
	, m_conductivity(mat.m_conductivity)
	, m_penetrationLoss(mat.m_penetrationLoss)
{
}

MaterialConfig::~MaterialConfig()
{
}

MaterialConfig& MaterialConfig::operator=(const MaterialConfig& mat)
{
	m_id = mat.m_id;
	m_name = mat.m_name;
	m_permittivity = mat.m_permittivity;
	m_conductivity = mat.m_conductivity;
	m_penetrationLoss = mat.m_penetrationLoss;
	return *this;
}

void MaterialConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_MATERIALCONFIG_ID.c_str()); writer.Int(m_id);
	writer.Key(KEY_MATERIALCONFIG_NAME.c_str()); writer.String(m_name.c_str());
	writer.Key(KEY_MATERIALCONFIG_PERMITTIVITY.c_str()); writer.Double(m_permittivity);
	writer.Key(KEY_MATERIALCONFIG_CONDUCTIVITY.c_str()); writer.Double(m_conductivity);
	writer.Key(KEY_MATERIALCONFIG_PENETRATIONLOSS.c_str()); writer.Double(m_penetrationLoss);
	writer.EndObject();
}

bool MaterialConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "MaterialConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_MATERIALCONFIG_ID.c_str())) {
		LOG_ERROR << "MaterialConfig: missing " << KEY_MATERIALCONFIG_ID.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_MATERIALCONFIG_NAME.c_str())) {
		LOG_ERROR << "MaterialConfig: missing " << KEY_MATERIALCONFIG_NAME.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_MATERIALCONFIG_PERMITTIVITY.c_str())) {
		LOG_ERROR << "MaterialConfig: missing " << KEY_MATERIALCONFIG_PERMITTIVITY.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_MATERIALCONFIG_CONDUCTIVITY.c_str())) {
		LOG_ERROR << "MaterialConfig: missing " << KEY_MATERIALCONFIG_CONDUCTIVITY.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_MATERIALCONFIG_PENETRATIONLOSS.c_str())) {
		LOG_ERROR << "MaterialConfig: missing " << KEY_MATERIALCONFIG_PENETRATIONLOSS.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& idValue = value[KEY_MATERIALCONFIG_ID.c_str()];
	const rapidjson::Value& nameValue = value[KEY_MATERIALCONFIG_NAME.c_str()];
	const rapidjson::Value& permittivityValue = value[KEY_MATERIALCONFIG_PERMITTIVITY.c_str()];
	const rapidjson::Value& conductivityValue = value[KEY_MATERIALCONFIG_CONDUCTIVITY.c_str()];
	const rapidjson::Value& penetrationLossValue = value[KEY_MATERIALCONFIG_PENETRATIONLOSS.c_str()];

	if (!idValue.IsInt()) {
		LOG_ERROR << "MaterialConfig: " << KEY_MATERIALCONFIG_ID.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!nameValue.IsString()) {
		LOG_ERROR << "MaterialConfig: " << KEY_MATERIALCONFIG_NAME.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!permittivityValue.IsDouble()) {
		LOG_ERROR << "MaterialConfig: " << KEY_MATERIALCONFIG_PERMITTIVITY.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!conductivityValue.IsDouble()) {
		LOG_ERROR << "MaterialConfig: " << KEY_MATERIALCONFIG_CONDUCTIVITY.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!penetrationLossValue.IsDouble()) {
		LOG_ERROR << "MaterialConfig: " << KEY_MATERIALCONFIG_PENETRATIONLOSS.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_id = idValue.GetInt();
	m_name = nameValue.GetString();
	m_permittivity = permittivityValue.GetDouble();
	m_conductivity = conductivityValue.GetDouble();
	m_penetrationLoss = penetrationLossValue.GetDouble();

	return true;
}
