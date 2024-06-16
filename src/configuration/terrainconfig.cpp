#include "terrainconfig.h"

TerrainGridConfig::TerrainGridConfig()
	: m_heightMatrixFile("")
	, m_materialMatrixFile("")
{
}

TerrainGridConfig::TerrainGridConfig(const TerrainGridConfig& config)
	: m_heightMatrixFile(config.m_heightMatrixFile)
	, m_materialMatrixFile(config.m_materialMatrixFile)
	, m_matNames(config.m_matNames)
{
}

TerrainGridConfig::~TerrainGridConfig()
{
}

TerrainGridConfig& TerrainGridConfig::operator=(const TerrainGridConfig& config)
{
	m_heightMatrixFile = config.m_heightMatrixFile;
	m_materialMatrixFile = config.m_materialMatrixFile;
	m_matNames = config.m_matNames;
	return *this;
}

bool TerrainGridConfig::operator==(const TerrainGridConfig& config) const
{
	if (m_heightMatrixFile != config.m_heightMatrixFile)
		return false;
	if (m_materialMatrixFile != config.m_materialMatrixFile)
		return false;
	if (m_matNames.size() != config.m_matNames.size())
		return false;
	for (int i = 0; i < m_matNames.size(); ++i) {
		if (m_matNames[i] != config.m_matNames[i])
			return false;
	}
	return true;
}

bool TerrainGridConfig::operator!=(const TerrainGridConfig& config) const
{
	return !(*this == config);
}

void TerrainGridConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_TERRAINGRIDCONFIG_GEOMETRYMATRIXFILE.c_str());							writer.String(m_heightMatrixFile.c_str());
	writer.Key(KEY_TERRAINGRIDCONFIG_MATERIALMATRIXFILE.c_str());							writer.String(m_materialMatrixFile.c_str());
	writer.Key(KEY_TERRAINGRIDCONFIG_MATERIALS.c_str());									SerializeArray(m_matNames, writer);
	writer.EndObject();
}

bool TerrainGridConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "TerrainGridConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_TERRAINGRIDCONFIG_GEOMETRYMATRIXFILE.c_str())) {
		LOG_ERROR << "TerrainGridConfig: missing " << KEY_TERRAINGRIDCONFIG_GEOMETRYMATRIXFILE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TERRAINGRIDCONFIG_MATERIALMATRIXFILE.c_str())) {
		LOG_ERROR << "TerrainGridConfig: missing " << KEY_TERRAINGRIDCONFIG_MATERIALMATRIXFILE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TERRAINGRIDCONFIG_MATERIALS.c_str())) {
		LOG_ERROR << "TerrainGridConfig: missing " << KEY_TERRAINGRIDCONFIG_MATERIALS.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& geometryMatrixFileValue = value[KEY_TERRAINGRIDCONFIG_GEOMETRYMATRIXFILE.c_str()];
	const rapidjson::Value& materialMatrixFileValue = value[KEY_TERRAINGRIDCONFIG_MATERIALMATRIXFILE.c_str()];
	const rapidjson::Value& materialsValue = value[KEY_TERRAINGRIDCONFIG_MATERIALS.c_str()];

	if (!geometryMatrixFileValue.IsString()) {
		LOG_ERROR << "TerrainGridConfig: " << KEY_TERRAINGRIDCONFIG_GEOMETRYMATRIXFILE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!materialMatrixFileValue.IsString()) {
		LOG_ERROR << "TerrainGridConfig: " << KEY_TERRAINGRIDCONFIG_MATERIALMATRIXFILE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!materialsValue.IsArray()) {
		LOG_ERROR << "TerrainGridConfig: " << KEY_TERRAINGRIDCONFIG_MATERIALS.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_heightMatrixFile = geometryMatrixFileValue.GetString();
	m_materialMatrixFile = materialMatrixFileValue.GetString();

	if (!DeserializeArray(m_matNames, materialsValue)) {
		LOG_ERROR << "TerrainGridConfig: " << KEY_TERRAINGRIDCONFIG_MATERIALS.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	return true;
}


TerrainConfig::TerrainConfig()
	: m_loadingMode(TERRAIN_OBJECT)
	, m_simplifyFlag(false)
	, m_simplifyRate(0.0)				//默认不简化
	, m_averageRidgeGap(100.0)			//默认地形峰峦间隔为100m
	, m_category(GRIDCELL)
{
}

TerrainConfig::TerrainConfig(const TerrainConfig& config)
	: m_loadingMode(config.m_loadingMode)
	, m_category(config.m_category)
	, m_simplifyFlag(config.m_simplifyFlag)
	, m_simplifyRate(config.m_simplifyRate)
	, m_averageRidgeGap(config.m_averageRidgeGap)
	, m_objectConfig(config.m_objectConfig)
	, m_gridConfig(config.m_gridConfig)
{
}

TerrainConfig::~TerrainConfig()
{
}

TerrainConfig& TerrainConfig::operator=(const TerrainConfig& config)
{
	m_loadingMode = config.m_loadingMode;
	m_category = config.m_category;
	m_simplifyFlag = config.m_simplifyFlag;
	m_simplifyRate = config.m_simplifyRate;
	m_averageRidgeGap = config.m_averageRidgeGap;
	m_objectConfig = config.m_objectConfig;
	m_gridConfig = config.m_gridConfig;
	return *this;
}

bool TerrainConfig::operator==(const TerrainConfig& config) const
{
	if (m_loadingMode != config.m_loadingMode)
		return false;
	if (m_category != config.m_category)
		return false;
	if (m_simplifyFlag != config.m_simplifyFlag)
		return false;
	if (m_simplifyRate != config.m_simplifyRate)
		return false;
	if (m_averageRidgeGap != config.m_averageRidgeGap)
		return false;
	if (m_objectConfig != config.m_objectConfig)
		return false;
	if (m_gridConfig != config.m_gridConfig)
		return false;
	return true;
}

bool TerrainConfig::operator!=(const TerrainConfig& config) const
{
	return !(*this == config);
}

void TerrainConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_TERRAINCONFIG_LOADMODE.c_str());										SerializeEnum(m_loadingMode, writer);
	writer.Key(KEY_TERRAINCONFIG_CATEGORY.c_str());										SerializeEnum(m_category, writer);
	writer.Key(KEY_TERRAINCONFIG_SIMPLIFYFLAG.c_str());									writer.Bool(m_simplifyFlag);
	writer.Key(KEY_TERRAINCONFIG_SIMPLIFYRATE.c_str());									writer.Double(m_simplifyRate);
	writer.Key(KEY_TERRAINCONFIG_AVERAGERIDGEGAP.c_str());								writer.Double(m_averageRidgeGap);
	writer.Key(KEY_TERRAINCONFIG_OBJECTCONFIG.c_str());									m_objectConfig.Serialize(writer);
	writer.Key(KEY_TERRAINCONFIG_GRIDCONFIG.c_str());									m_gridConfig.Serialize(writer);
	writer.EndObject();
}

bool TerrainConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "TerrainConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_TERRAINCONFIG_LOADMODE.c_str())) {
		LOG_ERROR << "TerrainConfig: missing " << KEY_TERRAINCONFIG_LOADMODE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TERRAINCONFIG_CATEGORY.c_str())) {
		LOG_ERROR << "TerrainConfig: missing " << KEY_TERRAINCONFIG_CATEGORY.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TERRAINCONFIG_SIMPLIFYFLAG.c_str())) {
		LOG_ERROR << "TerrainConfig: missing " << KEY_TERRAINCONFIG_SIMPLIFYFLAG.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TERRAINCONFIG_SIMPLIFYRATE.c_str())) {
		LOG_ERROR << "TerrainConfig: missing " << KEY_TERRAINCONFIG_SIMPLIFYRATE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TERRAINCONFIG_AVERAGERIDGEGAP.c_str())) {
		LOG_ERROR << "TerrainConfig: missing " << KEY_TERRAINCONFIG_AVERAGERIDGEGAP.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TERRAINCONFIG_OBJECTCONFIG.c_str())) {
		LOG_ERROR << "TerrainConfig: missing " << KEY_TERRAINCONFIG_OBJECTCONFIG.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TERRAINCONFIG_GRIDCONFIG.c_str())) {
		LOG_ERROR << "TerrainConfig: missing " << KEY_TERRAINCONFIG_GRIDCONFIG.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& loadingModeValue = value[KEY_TERRAINCONFIG_LOADMODE.c_str()];
	const rapidjson::Value& typeValue = value[KEY_TERRAINCONFIG_CATEGORY.c_str()];
	const rapidjson::Value& simplifyFlagValue = value[KEY_TERRAINCONFIG_SIMPLIFYFLAG.c_str()];
	const rapidjson::Value& simplifyRateValue = value[KEY_TERRAINCONFIG_SIMPLIFYRATE.c_str()];
	const rapidjson::Value& averageRidgeGapValue = value[KEY_TERRAINCONFIG_AVERAGERIDGEGAP.c_str()];
	const rapidjson::Value& objectConfigValue = value[KEY_TERRAINCONFIG_OBJECTCONFIG.c_str()];
	const rapidjson::Value& gridConfigValue = value[KEY_TERRAINCONFIG_GRIDCONFIG.c_str()];

	if (!loadingModeValue.IsInt()) {
		LOG_ERROR << "TerrainConfig: " << KEY_TERRAINCONFIG_LOADMODE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!typeValue.IsInt()) {
		LOG_ERROR << "TerrainConfig: " << KEY_TERRAINCONFIG_CATEGORY.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!simplifyFlagValue.IsBool()) {
		LOG_ERROR << "TerrainConfig: " << KEY_TERRAINCONFIG_SIMPLIFYFLAG.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!simplifyRateValue.IsDouble()) {
		LOG_ERROR << "TerrainConfig: " << KEY_TERRAINCONFIG_SIMPLIFYRATE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!averageRidgeGapValue.IsDouble()) {
		LOG_ERROR << "TerrainConfig: " << KEY_TERRAINCONFIG_AVERAGERIDGEGAP.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!objectConfigValue.IsObject()) {
		LOG_ERROR << "TerrainConfig: " << KEY_TERRAINCONFIG_OBJECTCONFIG.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!gridConfigValue.IsObject()) {
		LOG_ERROR << "TerrainConfig: " << KEY_TERRAINCONFIG_GRIDCONFIG.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_simplifyFlag = simplifyFlagValue.GetBool();
	m_simplifyRate = simplifyRateValue.GetDouble();
	m_averageRidgeGap = averageRidgeGapValue.GetDouble();

	if (!DeserializeEnum(m_loadingMode, loadingModeValue)) {
		LOG_ERROR << "TerrainConfig: " << KEY_TERRAINCONFIG_LOADMODE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!DeserializeEnum(m_category, typeValue)) {
		LOG_ERROR << "TerrainConfig: " << KEY_TERRAINCONFIG_CATEGORY.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_objectConfig.Deserialize(objectConfigValue)) {
		LOG_ERROR << "TerrainConfig: " << KEY_TERRAINCONFIG_OBJECTCONFIG.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_gridConfig.Deserialize(gridConfigValue)) {
		LOG_ERROR << "TerrainConfig: " << KEY_TERRAINCONFIG_GRIDCONFIG.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	return true;
}

bool TerrainConfig::Init(std::string filename)
{
	std::ifstream config_ifs(filename);
	if (!config_ifs.is_open()) {
		LOG_ERROR << "TerrainConfig: fail to open " << filename << ENDL;
		this->Write2Json(filename);
		LOG_INFO << "TerrainConfig: have wrote to default configuration to file: " << filename << ENDL;
		return false;
	}
	std::stringstream ss;
	ss << config_ifs.rdbuf();
	config_ifs.close();
	std::string jsonString = ss.str();
	rapidjson::Document doc;
	doc.Parse(jsonString.c_str());
	if (doc.HasMember(KEY_TERRAINCONFIG.c_str())) {
		rapidjson::Value& value = doc[KEY_TERRAINCONFIG.c_str()];
		if (value.IsObject()) {
			if (this->Deserialize(value)) {
				LOG_INFO << "TerrainConfig: load data success!" << ENDL;
				return true;
			}
		}
	}
	return false;
}

void TerrainConfig::Write2Json(std::string filename)
{
	std::ofstream config_ofs(filename);
	if (!config_ofs.is_open()) {
		LOG_ERROR << "TerrainConfig: fail to open " << filename << ENDL;
		return;
	}
	rapidjson::StringBuffer sb;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
	writer.SetFormatOptions(rapidjson::PrettyFormatOptions::kFormatSingleLineArray);
	writer.SetIndent('\t', 1);
	writer.StartObject();
	config_ofs << sb.GetString();
	config_ofs.close();
	return;
}
