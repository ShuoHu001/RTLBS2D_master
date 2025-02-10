#include "geometryconfig.h"

GeometryConfig::GeometryConfig()
	: m_buildingFile("")
	, m_buildingAttributeFile("")
	, m_vegetationFile("")
	, m_vegetationAttributeFile("")
	, m_wallFile("")
	, m_wallAttributeFile("")
	, m_loadingTerrainFlag(false)
	, m_positionError(0.0)
{
}

GeometryConfig::GeometryConfig(const GeometryConfig& config)
	: m_buildingFile(config.m_buildingFile)
	, m_buildingAttributeFile(config.m_buildingAttributeFile)
	, m_vegetationFile(config.m_vegetationFile)
	, m_vegetationAttributeFile(config.m_vegetationAttributeFile)
	, m_wallFile(config.m_wallFile)
	, m_wallAttributeFile(config.m_wallAttributeFile)
	, m_loadingTerrainFlag(config.m_loadingTerrainFlag)
	, m_terrainConfig(config.m_terrainConfig)
	, m_positionError(config.m_positionError)
{
}

GeometryConfig::~GeometryConfig()
{
}

GeometryConfig& GeometryConfig::operator=(const GeometryConfig& config)
{
	m_buildingFile = config.m_buildingFile;
	m_buildingAttributeFile = config.m_buildingAttributeFile;
	m_vegetationFile = config.m_vegetationFile;
	m_vegetationAttributeFile = config.m_vegetationAttributeFile;
	m_wallFile = config.m_wallFile;
	m_wallAttributeFile = config.m_wallAttributeFile;
	m_loadingTerrainFlag = config.m_loadingTerrainFlag;
	m_terrainConfig = config.m_terrainConfig;
	m_positionError = config.m_positionError;
	return* this;
}

void GeometryConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_GEOMETRYCONFIG_BUILDINGFILE.c_str());									writer.String(m_buildingFile.c_str());
	writer.Key(KEY_GEOMETRYCONFIG_BUILDINGATTRIBUTEFILE.c_str());							writer.String(m_buildingAttributeFile.c_str());
	writer.Key(KEY_GEOMETRYCONFIG_VEGETATIONFILE.c_str());									writer.String(m_vegetationFile.c_str());
	writer.Key(KEY_GEOMETRYCONFIG_VEGETATIONATTRIBUTEFILE.c_str());							writer.String(m_vegetationAttributeFile.c_str());
	writer.Key(KEY_GEOMETRYCONFIG_WALLFILE.c_str());										writer.String(m_wallFile.c_str());
	writer.Key(KEY_GEOMETRYCONFIG_WALLATTRIBUTEFILE.c_str());								writer.String(m_wallAttributeFile.c_str());
	writer.Key(KEY_GEOMETRYCONFIG_LOADINGTERRAINFLAG.c_str());								writer.Bool(m_loadingTerrainFlag);
	writer.Key(KEY_GEOMETRYCONFIG_TERRAINCOFNIG.c_str());									m_terrainConfig.Serialize(writer);
	writer.Key(KEY_GEOMETRYCONFIG_POSITIONERROR.c_str());									writer.Double(m_positionError);
	writer.EndObject();
}

bool GeometryConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "GeometryConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_GEOMETRYCONFIG_BUILDINGFILE.c_str())) {
		LOG_ERROR << "GeometryConfig: missing " << KEY_GEOMETRYCONFIG_BUILDINGFILE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_GEOMETRYCONFIG_BUILDINGATTRIBUTEFILE.c_str())) {
		LOG_ERROR << "GeometryConfig: missing " << KEY_GEOMETRYCONFIG_BUILDINGATTRIBUTEFILE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_GEOMETRYCONFIG_VEGETATIONFILE.c_str())) {
		LOG_ERROR << "GeometryConfig: missing " << KEY_GEOMETRYCONFIG_VEGETATIONFILE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_GEOMETRYCONFIG_VEGETATIONATTRIBUTEFILE.c_str())) {
		LOG_ERROR << "GeometryConfig: missing " << KEY_GEOMETRYCONFIG_VEGETATIONATTRIBUTEFILE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_GEOMETRYCONFIG_WALLFILE.c_str())) {
		LOG_ERROR << "GeometryConfig: missing " << KEY_GEOMETRYCONFIG_WALLFILE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_GEOMETRYCONFIG_WALLATTRIBUTEFILE.c_str())) {
		LOG_ERROR << "GeometryConfig: missing " << KEY_GEOMETRYCONFIG_WALLATTRIBUTEFILE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_GEOMETRYCONFIG_LOADINGTERRAINFLAG.c_str())) {
		LOG_ERROR << "GeometryConfig: missing " << KEY_GEOMETRYCONFIG_LOADINGTERRAINFLAG.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_GEOMETRYCONFIG_TERRAINCOFNIG.c_str())) {
		LOG_ERROR << "GeometryConfig: missing " << KEY_GEOMETRYCONFIG_TERRAINCOFNIG.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_GEOMETRYCONFIG_POSITIONERROR.c_str())) {
		LOG_ERROR << "GeometryConfig: missing " << KEY_GEOMETRYCONFIG_POSITIONERROR.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& buildingFileValue = value[KEY_GEOMETRYCONFIG_BUILDINGFILE.c_str()];
	const rapidjson::Value& buildingAttributeFileValue = value[KEY_GEOMETRYCONFIG_BUILDINGATTRIBUTEFILE.c_str()];
	const rapidjson::Value& vegetationFileValue = value[KEY_GEOMETRYCONFIG_VEGETATIONFILE.c_str()];
	const rapidjson::Value& vegetationAttributeFileValue = value[KEY_GEOMETRYCONFIG_VEGETATIONATTRIBUTEFILE.c_str()];
	const rapidjson::Value& wallFileValue = value[KEY_GEOMETRYCONFIG_WALLFILE.c_str()];
	const rapidjson::Value& wallAttributeFileValue = value[KEY_GEOMETRYCONFIG_WALLATTRIBUTEFILE.c_str()];
	const rapidjson::Value& loadingTerrainFlagValue = value[KEY_GEOMETRYCONFIG_LOADINGTERRAINFLAG.c_str()];
	const rapidjson::Value& terrainConfigValue = value[KEY_GEOMETRYCONFIG_TERRAINCOFNIG.c_str()];
	const rapidjson::Value& positionErrorValue = value[KEY_GEOMETRYCONFIG_POSITIONERROR.c_str()];

	if (!buildingFileValue.IsString()) {
		LOG_ERROR << "GeometryConfig: " << KEY_GEOMETRYCONFIG_BUILDINGFILE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!buildingAttributeFileValue.IsString()) {
		LOG_ERROR << "GeometryConfig: " << KEY_GEOMETRYCONFIG_BUILDINGATTRIBUTEFILE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!vegetationFileValue.IsString()) {
		LOG_ERROR << "GeometryConfig: " << KEY_GEOMETRYCONFIG_VEGETATIONFILE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!vegetationAttributeFileValue.IsString()) {
		LOG_ERROR << "GeometryConfig: " << KEY_GEOMETRYCONFIG_VEGETATIONATTRIBUTEFILE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!wallFileValue.IsString()) {
		LOG_ERROR << "GeometryConfig: " << KEY_GEOMETRYCONFIG_WALLFILE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!wallAttributeFileValue.IsString()) {
		LOG_ERROR << "GeometryConfig: " << KEY_GEOMETRYCONFIG_WALLATTRIBUTEFILE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!loadingTerrainFlagValue.IsBool()) {
		LOG_ERROR << "GeometryConfig: " << KEY_GEOMETRYCONFIG_LOADINGTERRAINFLAG.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!terrainConfigValue.IsObject()) {
		LOG_ERROR << "GeometryConfig: " << KEY_GEOMETRYCONFIG_TERRAINCOFNIG.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!positionErrorValue.IsDouble()) {
		LOG_ERROR << "GeometryConfig: " << KEY_GEOMETRYCONFIG_POSITIONERROR.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_buildingFile = buildingFileValue.GetString();
	m_buildingAttributeFile = buildingAttributeFileValue.GetString();
	m_vegetationFile = vegetationFileValue.GetString();
	m_vegetationAttributeFile = vegetationAttributeFileValue.GetString();
	m_wallFile = wallFileValue.GetString();
	m_wallAttributeFile = wallAttributeFileValue.GetString();
	m_loadingTerrainFlag = loadingTerrainFlagValue.GetBool();
	m_positionError = positionErrorValue.GetDouble();

	if (!m_terrainConfig.Deserialize(terrainConfigValue)) {
		LOG_ERROR << "GeometryConfig: " << KEY_GEOMETRYCONFIG_TERRAINCOFNIG.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	return true;
}

bool GeometryConfig::Init(const std::string& filename)
{
	std::ifstream config_ifs(filename);
	if (!config_ifs.is_open()) {
		LOG_ERROR << filename << ": file not exist" << ENDL;
		this->Write2Json(filename);
		LOG_WARNING << "GeomteryConfig: failed to read data from" << filename << " ." << ENDL;
		return false;
	}
	std::stringstream ss;
	ss << config_ifs.rdbuf();
	std::string jsonString = ss.str();
	rapidjson::Document doc;
	doc.Parse(jsonString.c_str());
	rapidjson::Value& value = doc[KEY_GEOMETRYCONFIG.c_str()];
	if (value.IsObject()) {
		if (Deserialize(value)) {
			LOG_INFO << "GeomteryConfig: loading configuration success." << ENDL;
			return true;
		}
	}
	LOG_ERROR << "GeomteryConfig: failed to init from " << filename << " ." << ENDL;
	return false;
}

void GeometryConfig::Write2Json(const std::string& filename)
{
	std::ofstream config_ofs(filename);
	if (!config_ofs.is_open()) {
		LOG_ERROR << "GeomteryConfig: failed to open " << filename << " ." << ENDL;
		return;
	}
	rapidjson::StringBuffer sb;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
	writer.SetFormatOptions(rapidjson::PrettyFormatOptions::kFormatSingleLineArray);
	writer.SetIndent('\t', 1);
	writer.StartObject();
	writer.Key(KEY_GEOMETRYCONFIG.c_str()); this->Serialize(writer);
	writer.EndObject();
	config_ofs << sb.GetString();
	return;
}
