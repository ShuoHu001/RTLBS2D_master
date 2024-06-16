#include "antennalibraryconfig.h"

AntennaLibraryConfig::AntennaLibraryConfig()
{
}

AntennaLibraryConfig::AntennaLibraryConfig(const AntennaLibraryConfig& config)
{
	m_antennas = config.m_antennas;
}

AntennaLibraryConfig::~AntennaLibraryConfig()
{
}

AntennaLibraryConfig& AntennaLibraryConfig::operator=(const AntennaLibraryConfig& config)
{
	// TODO: 在此处插入 return 语句
	m_antennas = config.m_antennas;
	return *this;
}

bool AntennaLibraryConfig::Init(const std::string& filename)
{
	std::ifstream config_ifs(filename);
	if (!config_ifs.is_open()) {
		LOG_ERROR << "AntennaLibraryConfig: fail to open " << filename << ENDL;
		this->Write2Json(filename);
		LOG_INFO << "AntennaLibraryConfig: have wrote to default configuration to file: " << filename << ENDL;
		return false;
	}
	std::stringstream ss;
	ss << config_ifs.rdbuf();
	config_ifs.close();
	std::string jsonString = ss.str();
	rapidjson::Document doc;
	doc.Parse(jsonString.c_str());
	if (doc.HasMember(KEY_ANTENNALIBRARYCONFIG.c_str())) {
		rapidjson::Value& value = doc[KEY_ANTENNALIBRARYCONFIG.c_str()];
		if (value.IsObject()) {
			if (this->Deserialize(value)) {
				LOG_INFO << "AntennaLibraryConfig: load data success!" << ENDL;
				return true;
			}
		}
	}
	return false;
}

void AntennaLibraryConfig::Write2Json(const std::string& filename)
{
	std::ofstream config_ofs(filename);
	if (!config_ofs.is_open()) {
		LOG_ERROR << "AntennaLibraryConfig: fail to open " << filename << ENDL;
		return;
	}
	rapidjson::StringBuffer sb;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
	writer.SetFormatOptions(rapidjson::PrettyFormatOptions::kFormatSingleLineArray);
	writer.SetIndent('\t', 1);
	writer.StartObject();
	writer.Key(KEY_ANTENNALIBRARYCONFIG.c_str()); this->Serialize(writer);
	writer.EndObject();
	config_ofs << sb.GetString();
	config_ofs.close();
	return;
}

void AntennaLibraryConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_ANTENNALIBRARYCONFIG_ANTENNAS.c_str());										SerializeArray(m_antennas, writer);
	writer.EndObject();
	return;
}

bool AntennaLibraryConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "AntennaLibraryConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_ANTENNALIBRARYCONFIG_ANTENNAS.c_str())) {
		LOG_ERROR << "AntennaLibraryConfig: missing   " << KEY_ANTENNALIBRARYCONFIG_ANTENNAS.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& antennasValue = value[KEY_ANTENNALIBRARYCONFIG_ANTENNAS.c_str()];

	if (!antennasValue.IsArray()) {
		LOG_ERROR << "AntennaLibraryConfig: " << KEY_ANTENNALIBRARYCONFIG_ANTENNAS.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	return true;
}
