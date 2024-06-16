#include "sensorcollectionconfig.h"

SensorCollectionConfig::SensorCollectionConfig()
{
}

SensorCollectionConfig::SensorCollectionConfig(const SensorCollectionConfig& config)
{
	m_sensorConfigs = config.m_sensorConfigs;
}

SensorCollectionConfig::~SensorCollectionConfig()
{
}

SensorCollectionConfig& SensorCollectionConfig::operator=(const SensorCollectionConfig& config)
{
	m_sensorConfigs = config.m_sensorConfigs;
	return *this;
}

bool SensorCollectionConfig::Init(const std::string& filename)
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
	if (doc.HasMember(KEY_SENSORCOLLECTIONCONFIG.c_str())) {
		rapidjson::Value& value = doc[KEY_SENSORCOLLECTIONCONFIG.c_str()];
		if (value.IsObject()) {
			if (this->Deserialize(value)) {
				LOG_INFO << "SensorCollectionConfig: load data success!" << ENDL;
				return true;
			}
		}
	}
	return false;
}

void SensorCollectionConfig::Write2Json(const std::string& filename) const
{
	std::ofstream config_ofs(filename);
	if (!config_ofs.is_open()) {
		LOG_ERROR << "SensorCollectionConfig: fail to open " << filename << ENDL;
		return;
	}
	rapidjson::StringBuffer sb;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
	writer.SetFormatOptions(rapidjson::PrettyFormatOptions::kFormatSingleLineArray);
	writer.SetIndent('\t', 1);
	writer.StartObject();
	writer.Key(KEY_SENSORCOLLECTIONCONFIG.c_str()); this->Serialize(writer);
	writer.EndObject();
	config_ofs << sb.GetString();
	config_ofs.close();
	return;
}

void SensorCollectionConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) const
{
	writer.StartObject();
	writer.Key(KEY_SENSORCOLLECTIONCONFIG_SENSORCONFIGS.c_str());								SerializeArray(m_sensorConfigs, writer);
	writer.EndObject();
}

bool SensorCollectionConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "SensorCollectionConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_SENSORCOLLECTIONCONFIG_SENSORCONFIGS.c_str())) {
		LOG_ERROR << "SensorCollectionConfig: missing " << KEY_SENSORCOLLECTIONCONFIG_SENSORCONFIGS.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& sensorConfigValue = value[KEY_SENSORCOLLECTIONCONFIG_SENSORCONFIGS.c_str()];

	if (!sensorConfigValue.IsArray()) {
		LOG_ERROR << "SensorCollectionConfig: " << KEY_SENSORCOLLECTIONCONFIG_SENSORCONFIGS.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	if (!DeserializeArray(m_sensorConfigs, sensorConfigValue)) {
		LOG_ERROR << "SensorCollectionConfig: " << KEY_SENSORCOLLECTIONCONFIG_SENSORCONFIGS.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	return true;
}
