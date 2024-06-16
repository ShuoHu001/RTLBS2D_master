#include "transmittercollectionconfig.h"

TransmitterCollectionConfig::TransmitterCollectionConfig()
{
}

TransmitterCollectionConfig::TransmitterCollectionConfig(const TransmitterCollectionConfig& config)
{
	m_transmitterConfigs = config.m_transmitterConfigs;
}

TransmitterCollectionConfig::~TransmitterCollectionConfig()
{
}

TransmitterCollectionConfig& TransmitterCollectionConfig::operator=(const TransmitterCollectionConfig& config)
{
	// TODO: 在此处插入 return 语句
	m_transmitterConfigs = config.m_transmitterConfigs;
	return *this;
}

bool TransmitterCollectionConfig::Init(std::string filename)
{
	if (filename.empty()) {
		LOG_INFO << "TransmitterCollectionConfig: Skip, not configure." << ENDL;
		return true;
	}
	std::ifstream config_ifs(filename);
	if (!config_ifs.is_open()) {
		LOG_ERROR << "TransmitterCollectionConfig: fail to open " << filename << ENDL;
		this->Write2Json(filename);
		LOG_INFO << "TransmitterCollectionConfig: have wrote to default configuration to file: " << filename << ENDL;
		return false;
	}
	std::stringstream ss;
	ss << config_ifs.rdbuf();
	config_ifs.close();
	std::string jsonString = ss.str();
	rapidjson::Document doc;
	doc.Parse(jsonString.c_str());
	if (doc.HasMember(KEY_TRANSMITTERCOLLECTIONCONFIG.c_str())) {
		rapidjson::Value& value = doc[KEY_TRANSMITTERCOLLECTIONCONFIG.c_str()];
		if (value.IsObject()) {
			if (this->Deserialize(value)) {
				LOG_INFO << "TransmitterCollectionConfig: load data success!" << ENDL;
				return true;
			}
		}
	}
	return false;
}

void TransmitterCollectionConfig::Write2Json(const std::string& filename)
{
	std::ofstream config_ofs(filename);
	if (!config_ofs.is_open()) {
		LOG_ERROR << "TransmitterCollectionConfig: fail to open " << filename << ENDL;
		return;
	}
	rapidjson::StringBuffer sb;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
	writer.SetFormatOptions(rapidjson::PrettyFormatOptions::kFormatSingleLineArray);
	writer.SetIndent('\t', 1);
	writer.StartObject();
	writer.Key(KEY_TRANSMITTERCOLLECTIONCONFIG.c_str()); this->Serialize(writer);
	writer.EndObject();
	config_ofs << sb.GetString();
	config_ofs.close();
	return;
}

void TransmitterCollectionConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_TRANSMITTERCOLLECTIONCONFIG_TRANSMITTERCONFIGS.c_str());								SerializeArray(m_transmitterConfigs, writer);
	writer.EndObject();
}

bool TransmitterCollectionConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "TransmitterCollectionConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_TRANSMITTERCOLLECTIONCONFIG_TRANSMITTERCONFIGS.c_str())) {
		LOG_ERROR << "TransmitterCollectionConfig: missing " << KEY_TRANSMITTERCOLLECTIONCONFIG_TRANSMITTERCONFIGS.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& transmitterConfigValue = value[KEY_TRANSMITTERCOLLECTIONCONFIG_TRANSMITTERCONFIGS.c_str()];

	if (!transmitterConfigValue.IsArray()) {
		LOG_ERROR << "TransmitterCollectionConfig: " << KEY_TRANSMITTERCOLLECTIONCONFIG_TRANSMITTERCONFIGS.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	if (!DeserializeArray(m_transmitterConfigs, transmitterConfigValue)) {
		LOG_ERROR << "TransmitterCollectionConfig: " << KEY_TRANSMITTERCOLLECTIONCONFIG_TRANSMITTERCONFIGS.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	return true;
}
