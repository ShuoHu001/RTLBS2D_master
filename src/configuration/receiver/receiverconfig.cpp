#include "receiverconfig.h"

ReceiverConfig::ReceiverConfig()
	: m_predictionMode(PREDICTION_SINGLEPOINT)
	, m_antId(0)
	, m_antName("")
	, m_insertLoss(0.0)
	, m_attachGain(0.0)
	, m_powerThreshold(-160)
	, m_angularThreshold(1.0)
	, m_delayThreshold(1e-9)
{
}

ReceiverConfig::ReceiverConfig(const ReceiverConfig& config)
	: m_predictionMode(config.m_predictionMode)
	, m_singleConfig(config.m_singleConfig)
	, m_lineConfig(config.m_lineConfig)
	, m_scatterConfig(config.m_scatterConfig)
	, m_planeConfig(config.m_planeConfig)
	, m_solidConfig(config.m_solidConfig)
	, m_antId(config.m_antId)
	, m_antName(config.m_antName)
	, m_insertLoss(config.m_insertLoss)
	, m_attachGain(config.m_attachGain)
	, m_powerThreshold(config.m_powerThreshold)
	, m_angularThreshold(config.m_angularThreshold)
	, m_delayThreshold(config.m_delayThreshold)
{
}

ReceiverConfig::~ReceiverConfig()
{
}

ReceiverConfig& ReceiverConfig::operator=(const ReceiverConfig& config)
{
	m_predictionMode = config.m_predictionMode;
	m_singleConfig = config.m_singleConfig;
	m_lineConfig = config.m_lineConfig;
	m_scatterConfig = config.m_scatterConfig;
	m_planeConfig = config.m_planeConfig;
	m_solidConfig = config.m_solidConfig;
	m_antId = config.m_antId;
	m_antName = config.m_antName;
	m_insertLoss = config.m_insertLoss;
	m_attachGain = config.m_attachGain;
	m_powerThreshold = config.m_powerThreshold;
	return *this;
}

void ReceiverConfig::CalculateRxPositions()
{
	if (m_predictionMode != PREDICTION_FILE) {		//常规模式加载
		if (m_predictionMode == PREDICTION_SINGLEPOINT) {
			m_singleConfig.CalculateRxPosition(m_receiverconfigs);
		}
		else if (m_predictionMode == PREDICTION_LINE) {
			m_lineConfig.CalculateRxPosition(m_receiverconfigs);
		}
		else if (m_predictionMode == PREDICTION_SCARRERPOINT) {
			m_scatterConfig.CalculateRxPosition(m_receiverconfigs);
		}
		else if (m_predictionMode == PREDICTION_PLANE) {
			m_planeConfig.CalculateRxPosition(m_receiverconfigs);
		}
		else if (m_predictionMode == PREDICTION_SOLID) {
			m_solidConfig.CalculateRxPosition(m_receiverconfigs);
		}
	}
	else {											//文件模式加载
		if (!this->InitReceiversFromFile(m_attachFileName)) {
			LOG_ERROR << "ReceiverConfig: file error, deserialize failed." << ENDL;
		}
	}
	for (auto& unitConfig : m_receiverconfigs) {
		unitConfig.m_powerShreshold = m_powerThreshold;
		unitConfig.m_angularThreshold = m_angularThreshold;
		unitConfig.m_delayThreshold = m_delayThreshold;
	}
}


void ReceiverConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_RECEIVERCONFIG_PREDICTIONMODE.c_str());									SerializeEnum(m_predictionMode, writer);
	writer.Key(KEY_RECEIVERCONFIG_SINGLECONFIG.c_str());									m_singleConfig.Serialize(writer);
	writer.Key(KEY_RECEIVERCONFIG_LINECONFIG.c_str());										m_lineConfig.Serialize(writer);
	writer.Key(KEY_RECEIVERCONFIG_SCATTERCONFIG.c_str());									m_scatterConfig.Serialize(writer);
	writer.Key(KEY_RECEIVERCONFIG_PLANECONFIG.c_str());										m_planeConfig.Serialize(writer);
	writer.Key(KEY_RECEIVERCONFIG_SOLIDCONFIG.c_str());										m_solidConfig.Serialize(writer);
	writer.Key(KEY_RECEIVERCONFIG_ATTACHFILENAME.c_str());									writer.String(m_attachFileName.c_str());
	writer.Key(KEY_RECEIVERCONFIG_ANTID.c_str());											writer.Double(m_antId);
	writer.Key(KEY_RECEIVERCONFIG_ANTNAME.c_str());											writer.String(m_antName.c_str());
	writer.Key(KEY_RECEIVERCONFIG_INSERTLOSS.c_str());										writer.Double(m_insertLoss);
	writer.Key(KEY_RECEIVERCONFIG_ATTACHGAIN.c_str());										writer.Double(m_attachGain);
	writer.Key(KEY_RECEIVERCONFIG_POWERTHRESHOLD.c_str());									writer.Double(m_powerThreshold);
	writer.Key(KEY_RECEIVERCONFIG_ANGULARTHRESHOLD.c_str());								writer.Double(m_angularThreshold);
	writer.Key(KEY_RECEIVERCONFIG_DELAYTHRESHOLD.c_str());									writer.Double(m_delayThreshold);
	writer.Key(KEY_RECEIVERCONFIG_POSTURE.c_str());											m_posture.Serialize(writer);
	writer.EndObject();
}

bool ReceiverConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "ReceiverConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_RECEIVERCONFIG_PREDICTIONMODE.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_PREDICTIONMODE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERCONFIG_SINGLECONFIG.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_SINGLECONFIG.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERCONFIG_LINECONFIG.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_LINECONFIG.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERCONFIG_SCATTERCONFIG.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_SCATTERCONFIG.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERCONFIG_PLANECONFIG.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_PLANECONFIG.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERCONFIG_SOLIDCONFIG.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_SOLIDCONFIG.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERCONFIG_ATTACHFILENAME.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_ATTACHFILENAME.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERCONFIG_ANTID.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_ANTID.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERCONFIG_ANTNAME.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_ANTNAME.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERCONFIG_INSERTLOSS.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_INSERTLOSS.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERCONFIG_ATTACHGAIN.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_ATTACHGAIN.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERCONFIG_POWERTHRESHOLD.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_POWERTHRESHOLD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERCONFIG_ANGULARTHRESHOLD.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_ANGULARTHRESHOLD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERCONFIG_DELAYTHRESHOLD.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_DELAYTHRESHOLD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RECEIVERCONFIG_POSTURE.c_str())) {
		LOG_ERROR << "ReceiverConfig: missing " << KEY_RECEIVERCONFIG_POSTURE.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& predictionModeValue = value[KEY_RECEIVERCONFIG_PREDICTIONMODE.c_str()];
	const rapidjson::Value& singleConfigValue = value[KEY_RECEIVERCONFIG_SINGLECONFIG.c_str()];
	const rapidjson::Value& lineConfigValue = value[KEY_RECEIVERCONFIG_LINECONFIG.c_str()];
	const rapidjson::Value& scatterConfigValue = value[KEY_RECEIVERCONFIG_SCATTERCONFIG.c_str()];
	const rapidjson::Value& planeConfigValue = value[KEY_RECEIVERCONFIG_PLANECONFIG.c_str()];
	const rapidjson::Value& solidConfigValue = value[KEY_RECEIVERCONFIG_SOLIDCONFIG.c_str()];
	const rapidjson::Value& attachFileNameValue = value[KEY_RECEIVERCONFIG_ATTACHFILENAME.c_str()];
	const rapidjson::Value& antIdValue = value[KEY_RECEIVERCONFIG_ANTID.c_str()];
	const rapidjson::Value& antNameVlaue = value[KEY_RECEIVERCONFIG_ANTNAME.c_str()];
	const rapidjson::Value& insertLossValue = value[KEY_RECEIVERCONFIG_INSERTLOSS.c_str()];
	const rapidjson::Value& attachGainValue = value[KEY_RECEIVERCONFIG_ATTACHGAIN.c_str()];
	const rapidjson::Value& powerThresholdValue = value[KEY_RECEIVERCONFIG_POWERTHRESHOLD.c_str()];
	const rapidjson::Value& angularThresholdValue = value[KEY_RECEIVERCONFIG_ANGULARTHRESHOLD.c_str()];
	const rapidjson::Value& delayThresholdValue = value[KEY_RECEIVERCONFIG_DELAYTHRESHOLD.c_str()];
	const rapidjson::Value& postureValue = value[KEY_RECEIVERCONFIG_POSTURE.c_str()];

	if (!predictionModeValue.IsInt()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_PREDICTIONMODE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!singleConfigValue.IsObject()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_SINGLECONFIG.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!lineConfigValue.IsObject()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_LINECONFIG.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!scatterConfigValue.IsObject()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_SCATTERCONFIG.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!planeConfigValue.IsObject()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_PLANECONFIG.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!solidConfigValue.IsObject()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_SOLIDCONFIG.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!attachFileNameValue.IsString()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_ATTACHFILENAME.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!antIdValue.IsUint()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_ANTID.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!antNameVlaue.IsString()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_ANTNAME.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!insertLossValue.IsDouble()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_INSERTLOSS.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!attachGainValue.IsDouble()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_ATTACHGAIN.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!powerThresholdValue.IsDouble()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_POWERTHRESHOLD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!angularThresholdValue.IsDouble()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_ANGULARTHRESHOLD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!delayThresholdValue.IsDouble()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_DELAYTHRESHOLD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!postureValue.IsObject()) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_POSTURE.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_attachFileName = attachFileNameValue.GetString();
	m_antId = antIdValue.GetUint();
	m_antName = antNameVlaue.GetString();
	m_insertLoss = insertLossValue.GetDouble();
	m_attachGain = attachGainValue.GetDouble();
	m_powerThreshold = powerThresholdValue.GetDouble();
	m_angularThreshold = angularThresholdValue.GetDouble();
	m_delayThreshold = delayThresholdValue.GetDouble();

	if (!DeserializeEnum(m_predictionMode, predictionModeValue)) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_PREDICTIONMODE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_singleConfig.Deserialize(singleConfigValue)) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_SINGLECONFIG.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_lineConfig.Deserialize(lineConfigValue)) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_LINECONFIG.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_scatterConfig.Deserialize(scatterConfigValue)) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_SCATTERCONFIG.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_planeConfig.Deserialize(planeConfigValue)) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_PLANECONFIG.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_solidConfig.Deserialize(solidConfigValue)) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_SOLIDCONFIG.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_posture.Deserialize(postureValue)) {
		LOG_ERROR << "ReceiverConfig: " << KEY_RECEIVERCONFIG_POSTURE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	return true;
}

bool ReceiverConfig::InitReceiversFromFile(std::string filename)
{
	if (filename.empty()) {
		LOG_INFO << "ReceiverConfig: failed to load receivers." << ENDL;
		return false;
	}
	std::ifstream config_ifs(filename);
	if (!config_ifs.is_open()) {
		LOG_ERROR << "ReceiverConfig: fail to open " << filename << ENDL;
		return false;
	}
	std::stringstream ss;
	ss << config_ifs.rdbuf();
	config_ifs.close();
	std::string jsonString = ss.str();
	rapidjson::Document doc;
	doc.Parse(jsonString.c_str());
	if (doc.HasMember(KEY_RECEIVERCONFIG_RECEIVERS.c_str())) {
		rapidjson::Value& value = doc[KEY_RECEIVERCONFIG_RECEIVERS.c_str()];
		if (value.IsArray()) {
			return DeserializeArray(m_receiverconfigs, value);
		}
	}
	return false;
}

bool ReceiverConfig::Init(const std::string& filename)
{
	if (filename.empty()) {
		LOG_ERROR << "ReceiverConfig: not configure. " << ENDL;
		return false;
	}
	std::ifstream config_ifs(filename);
	if (!config_ifs.is_open()) {
		LOG_ERROR << "ReceiverConfig: fail to open " << filename << ENDL;
		this->Write2Json(filename);
		LOG_INFO << "ReceiverConfig: have wrote to default configuration to file: " << filename << ENDL;
		return false;
	}
	std::stringstream ss;
	ss << config_ifs.rdbuf();
	config_ifs.close();
	std::string jsonString = ss.str();
	rapidjson::Document doc;
	doc.Parse(jsonString.c_str());
	if (doc.HasMember(KEY_RECEIVERCONFIG.c_str())) {
		rapidjson::Value& value = doc[KEY_RECEIVERCONFIG.c_str()];
		if (value.IsObject()) {
			if (this->Deserialize(value)) {
				LOG_INFO << "ReceiverConfig: load data success!" << ENDL;
				CalculateRxPositions();						//内部计算接收机位置
				return true;
			}
		}
	}
	return false;
}

void ReceiverConfig::Write2Json(const std::string& filename)
{
	std::ofstream config_ofs(filename);
	if (!config_ofs.is_open()) {
		LOG_ERROR << "ReceiverConfig: fail to open " << filename << ENDL;
		return;
	}
	rapidjson::StringBuffer sb;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
	writer.SetFormatOptions(rapidjson::PrettyFormatOptions::kFormatSingleLineArray);
	writer.SetIndent('\t', 1);
	writer.StartObject();
	writer.Key(KEY_RECEIVERCONFIG.c_str()); this->Serialize(writer);
	writer.EndObject();
	config_ofs << sb.GetString();
	config_ofs.close();
	return;
}
