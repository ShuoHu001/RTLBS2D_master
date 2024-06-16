#include "terrainobjectconfig.h"

TerrainObjectConfig::TerrainObjectConfig()
	: m_fileName("")
	, m_windingOrder(COUNTCLOCKWISE)
	, m_reverseCoord(false)
{
}

TerrainObjectConfig::TerrainObjectConfig(const TerrainObjectConfig& config)
	: m_fileName(config.m_fileName)
	, m_windingOrder(config.m_windingOrder)
	, m_reverseCoord(config.m_reverseCoord)
	, m_matNames(config.m_matNames)
{
}

TerrainObjectConfig::~TerrainObjectConfig()
{
}

TerrainObjectConfig& TerrainObjectConfig::operator=(const TerrainObjectConfig& config)
{
	m_fileName = config.m_fileName;
	m_windingOrder = config.m_windingOrder;
	m_reverseCoord = config.m_reverseCoord;
	m_matNames = config.m_matNames;
	return *this;
}

bool TerrainObjectConfig::operator==(const TerrainObjectConfig& config) const
{
	if (m_fileName != config.m_fileName)
		return false;
	if (m_windingOrder != config.m_windingOrder)
		return false;
	if (m_reverseCoord != config.m_reverseCoord)
		return false;
	return true;
}

bool TerrainObjectConfig::operator!=(const TerrainObjectConfig& config) const
{
	return !(*this == config);
}

void TerrainObjectConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_TERRAINOBJECTCONFIG_FILENAME.c_str());										writer.String(m_fileName.c_str());
	writer.Key(KEY_TERRAINOBJECTCONFIG_WINDINGORDER.c_str());									SerializeEnum(m_windingOrder, writer);
	writer.Key(KEY_TERRAINOBJECTCONFIG_REVERSECOORD.c_str());									writer.Bool(m_reverseCoord);
	writer.Key(KEY_TERRAINOBJECTCONFIG_MATNAMES.c_str());										SerializeArray(m_matNames, writer);
	writer.EndObject();
}

bool TerrainObjectConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "TerrainObjectConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_TERRAINOBJECTCONFIG_FILENAME.c_str())) {
		LOG_ERROR << "TerrainObjectConfig: missing " << KEY_TERRAINOBJECTCONFIG_FILENAME.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TERRAINOBJECTCONFIG_WINDINGORDER.c_str())) {
		LOG_ERROR << "TerrainObjectConfig: missing " << KEY_TERRAINOBJECTCONFIG_WINDINGORDER.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TERRAINOBJECTCONFIG_REVERSECOORD.c_str())) {
		LOG_ERROR << "TerrainObjectConfig: missing " << KEY_TERRAINOBJECTCONFIG_REVERSECOORD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TERRAINOBJECTCONFIG_MATNAMES.c_str())) {
		LOG_ERROR << "TerrainObjectConfig: missing " << KEY_TERRAINOBJECTCONFIG_MATNAMES.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& filenameValue = value[KEY_TERRAINOBJECTCONFIG_FILENAME.c_str()];
	const rapidjson::Value& windingOrderValue = value[KEY_TERRAINOBJECTCONFIG_WINDINGORDER.c_str()];
	const rapidjson::Value& reverseCoordValue = value[KEY_TERRAINOBJECTCONFIG_REVERSECOORD.c_str()];
	const rapidjson::Value& matNamesValue = value[KEY_TERRAINOBJECTCONFIG_MATNAMES.c_str()];

	if (!filenameValue.IsString()) {
		LOG_ERROR << "TerrainObjectConfig: " << KEY_TERRAINOBJECTCONFIG_FILENAME.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!windingOrderValue.IsUint()) {
		LOG_ERROR << "TerrainObjectConfig: " << KEY_TERRAINOBJECTCONFIG_WINDINGORDER.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!reverseCoordValue.IsBool()) {
		LOG_ERROR << "TerrainObjectConfig: " << KEY_TERRAINOBJECTCONFIG_REVERSECOORD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!matNamesValue.IsArray()) {
		LOG_ERROR << "TerrainObjectConfig: " << KEY_TERRAINOBJECTCONFIG_MATNAMES.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_fileName = filenameValue.GetString();
	m_reverseCoord = reverseCoordValue.GetBool();

	if (!DeserializeEnum(m_windingOrder, windingOrderValue)) {
		LOG_ERROR << "TerrainObjectConfig: " << KEY_TERRAINOBJECTCONFIG_WINDINGORDER.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!DeserializeArray(m_matNames, matNamesValue)) {
		LOG_ERROR << "TerrainObjectConfig: " << KEY_TERRAINOBJECTCONFIG_MATNAMES.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	return true;
}
