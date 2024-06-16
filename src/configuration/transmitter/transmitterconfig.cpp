#include "transmitterconfig.h"


TransmitterConfig::TransmitterConfig()
	: m_antId(0)
	, m_power(1.0)
	, m_interLoss(0.0)
	, m_attachGain(0.0)
	, m_velocity(0.0)
{
}

TransmitterConfig::TransmitterConfig(const TransmitterConfig& config)
	: m_antId(config.m_antId)
	, m_antName(config.m_antName)
	, m_power(config.m_power)
	, m_interLoss(config.m_interLoss)
	, m_attachGain(config.m_attachGain)
	, m_position(config.m_position)
	, m_posture(config.m_posture)
	, m_velocity(config.m_velocity)
{
}

TransmitterConfig::~TransmitterConfig()
{
}

TransmitterConfig& TransmitterConfig::operator=(const TransmitterConfig& config)
{
	m_antId = config.m_antId;
	m_antName = config.m_antName;
	m_power = config.m_power;
	m_interLoss = config.m_interLoss;
	m_attachGain = config.m_attachGain;
	m_position = config.m_position;
	m_posture = config.m_posture;
	m_velocity = config.m_velocity;
	return *this;
	// TODO: 在此处插入 return 语句
}

void TransmitterConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_TRANSMITTERCONFIG_ANTID.c_str());										writer.Int(static_cast<int>(m_antId));
	writer.Key(KEY_TRANSMITTERCONFIG_ANTNAME.c_str());										writer.String(m_antName.c_str());
	writer.Key(KEY_TRANSMITTERCONFIG_POWER.c_str());										writer.Double(m_power);
	writer.Key(KEY_TRANSMITTERCONFIG_INTERLOSS.c_str());									writer.Double(m_interLoss);
	writer.Key(KEY_TRANSMITTERCONFIG_ATTACHGAIN.c_str());									writer.Double(m_attachGain);
	writer.Key(KEY_TRANSMITTERCONFIG_POSITION.c_str());										m_position.Serialize(writer);
	writer.Key(KEY_TRANSMITTERCONFIG_POSTURE.c_str());										m_posture.Serialize(writer);
	writer.Key(KEY_TRANSMITTERCONFIG_VELOCITY.c_str());										writer.Double(m_velocity);
	writer.EndObject();
}

bool TransmitterConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "TransmitterConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_TRANSMITTERCONFIG_ANTID.c_str())) {
		LOG_ERROR << "TransmitterConfig: missing " << KEY_TRANSMITTERCONFIG_ANTID.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TRANSMITTERCONFIG_ANTNAME.c_str())) {
		LOG_ERROR << "TransmitterConfig: missing " << KEY_TRANSMITTERCONFIG_ANTNAME.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TRANSMITTERCONFIG_POWER.c_str())) {
		LOG_ERROR << "TransmitterConfig: missing " << KEY_TRANSMITTERCONFIG_POWER.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TRANSMITTERCONFIG_INTERLOSS.c_str())) {
		LOG_ERROR << "TransmitterConfig: missing " << KEY_TRANSMITTERCONFIG_INTERLOSS.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TRANSMITTERCONFIG_ATTACHGAIN.c_str())) {
		LOG_ERROR << "TransmitterConfig: missing " << KEY_TRANSMITTERCONFIG_ATTACHGAIN.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TRANSMITTERCONFIG_POSITION.c_str())) {
		LOG_ERROR << "TransmitterConfig: missing " << KEY_TRANSMITTERCONFIG_POSITION.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TRANSMITTERCONFIG_POSTURE.c_str())) {
		LOG_ERROR << "TransmitterConfig: missing " << KEY_TRANSMITTERCONFIG_POSTURE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_TRANSMITTERCONFIG_VELOCITY.c_str())) {
		LOG_ERROR << "TransmitterConfig: missing " << KEY_TRANSMITTERCONFIG_VELOCITY.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& antIdValue = value[KEY_TRANSMITTERCONFIG_ANTID.c_str()];
	const rapidjson::Value& antNameValue = value[KEY_TRANSMITTERCONFIG_ANTNAME.c_str()];
	const rapidjson::Value& powerValue = value[KEY_TRANSMITTERCONFIG_POWER.c_str()];
	const rapidjson::Value& interLossValue = value[KEY_TRANSMITTERCONFIG_INTERLOSS.c_str()];
	const rapidjson::Value& attachGainValue = value[KEY_TRANSMITTERCONFIG_ATTACHGAIN.c_str()];
	const rapidjson::Value& positionValue = value[KEY_TRANSMITTERCONFIG_POSITION.c_str()];
	const rapidjson::Value& postureValue = value[KEY_TRANSMITTERCONFIG_POSTURE.c_str()];
	const rapidjson::Value& velocityValue = value[KEY_TRANSMITTERCONFIG_VELOCITY.c_str()];

	if (!antIdValue.IsUint()) {
		LOG_ERROR << "TransmitterConfig: " << KEY_TRANSMITTERCONFIG_ANTID.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!antNameValue.IsString()) {
		LOG_ERROR << "TransmitterConfig: " << KEY_TRANSMITTERCONFIG_ANTNAME.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!powerValue.IsDouble()) {
		LOG_ERROR << "TransmitterConfig: " << KEY_TRANSMITTERCONFIG_POWER.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!interLossValue.IsDouble()) {
		LOG_ERROR << "TransmitterConfig: " << KEY_TRANSMITTERCONFIG_INTERLOSS.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!attachGainValue.IsDouble()) {
		LOG_ERROR << "TransmitterConfig: " << KEY_TRANSMITTERCONFIG_ATTACHGAIN.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!positionValue.IsObject()) {
		LOG_ERROR << "TransmitterConfig: " << KEY_TRANSMITTERCONFIG_POSITION.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!postureValue.IsObject()) {
		LOG_ERROR << "TransmitterConfig: " << KEY_TRANSMITTERCONFIG_POSTURE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!velocityValue.IsDouble()) {
		LOG_ERROR << "TransmitterConfig: " << KEY_TRANSMITTERCONFIG_VELOCITY.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_antId = antIdValue.GetUint();
	m_antName = antNameValue.GetString();
	m_power = powerValue.GetDouble();
	m_interLoss = interLossValue.GetDouble();
	m_attachGain = attachGainValue.GetDouble();
	m_velocity = velocityValue.GetDouble();

	if (!m_position.Deserialize(positionValue)) {
		LOG_ERROR << "TransmitterConfig: " << KEY_TRANSMITTERCONFIG_POSITION.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_posture.Deserialize(postureValue)) {
		LOG_ERROR << "TransmitterConfig: " << KEY_TRANSMITTERCONFIG_POSTURE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}


	return true;
}

bool Init_TransmitterConfigs(const std::string& filename, std::vector<TransmitterConfig>& configs)
{
	if (filename.empty()) {
		LOG_ERROR << "TransmitterConfig: not configure. " << ENDL;
		return false;
	}
	std::ifstream config_ifs(filename);
	if (!config_ifs.is_open()) {
		LOG_ERROR << "TransmitterConfig: fail to open " << filename << ENDL;
		Write2Json_TransmitterConfigs(filename, configs);
		LOG_INFO << "TransmitterConfig: have wrote to default configuration to file: " << filename << ENDL;
		return false;
	}
	std::stringstream ss;
	ss << config_ifs.rdbuf();
	config_ifs.close();
	std::string jsonString = ss.str();
	rapidjson::Document doc;
	doc.Parse(jsonString.c_str());
	if (doc.HasMember(KEY_TRANSMITTERCONFIG.c_str())) {
		rapidjson::Value& value = doc[KEY_TRANSMITTERCONFIG.c_str()];
		if (value.IsArray()) {
			if (DeserializeArray(configs, value)) {
				LOG_INFO << "TransmitterConfig: load data success!" << ENDL;
				return true;
			}
		}
	}
	return false;
}

void Write2Json_TransmitterConfigs(const std::string& filename, const std::vector<TransmitterConfig>& configs)
{
	std::ofstream config_ofs(filename);
	if (!config_ofs.is_open()) {
		LOG_ERROR << "TransmitterConfig: fail to open " << filename << ENDL;
		return;
	}
	rapidjson::StringBuffer sb;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
	writer.SetFormatOptions(rapidjson::PrettyFormatOptions::kFormatSingleLineArray);
	writer.SetIndent('\t', 1);
	writer.StartObject();
	writer.Key(KEY_TRANSMITTERCONFIG.c_str()); SerializeArray(configs, writer);
	writer.EndObject();
	config_ofs << sb.GetString();
	config_ofs.close();
	return;
}
