#include "simconfig.h"


SimConfig::SimConfig()
	: m_systemMode(MODE_RT)
	, m_geometryConfigFile("environment.json")
	, m_materialLibraryConfigFile("materials.json")
	, m_antennaLibraryConfigFile("antennas.json")
	, m_receiverConfigFile("receivers.json")
	, m_transmitterConfigFile("transmitters.json")
{
}

SimConfig::~SimConfig()
{
}

bool SimConfig::Init(const std::string& filename)
{
	std::ifstream config_ifs(filename);
	if(!config_ifs.is_open()){
		LOG_ERROR << "SimConfig: failed to load " << filename << " ." << ENDL;
		this->Writer2Json(filename);
		LOG_WARNING << "SimConfig: system have write to default configuration to file:" << filename << " please check and modify." << ENDL;
		return false;
	}
	std::stringstream ss;
	ss << config_ifs.rdbuf();
	std::string jsonString = ss.str();
	rapidjson::Document doc;
	doc.Parse(jsonString.c_str());
	rapidjson::Value& value = doc[KEY_SIMCONFIG.c_str()];
	if (!value.IsObject()) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG.c_str() << ", wrong value format.";
		return false;
	}
	if (!Deserialize(value)) {
		LOG_ERROR << "SimConfig: failed to deserialize " << filename << " ." << ENDL;
		return false;
	}

	//-----------------------------------------------------------几何文件读取----------------------------------------------------------------------------
	if (!m_geometryConfigFile.empty()) {
		if (!m_geometryConfig.Init(m_geometryConfigFile)) {
			LOG_ERROR << "Simconfig: failed to init geometry configuration." << ENDL;
			return false;
		}
		LOG_INFO << "Simconfig: init geometry configuration success." << ENDL;
	}
	else {
		LOG_ERROR << "Simconfig: geometry configuration file empty." << ENDL;
		return false;
	}

	//-----------------------------------------------------------材质文件读取----------------------------------------------------------------------------
	if (!m_materialLibraryConfigFile.empty()) {
		if (!m_materialLibraryConfig.Init(m_materialLibraryConfigFile)) {
			LOG_ERROR << "Simconfig: failed to init material configuration." << ENDL;
			return false;
		}
		LOG_INFO << "Simconfig: init material configuration success." << ENDL;
	}
	else {
		LOG_ERROR << "Simconfig: material configuration file empty." << ENDL;
		return false;
	}

	//-----------------------------------------------------------天线文件读取----------------------------------------------------------------------------
	if (!m_antennaLibraryConfigFile.empty()) {
		if (!m_antennaLibraryConfig.Init(m_antennaLibraryConfigFile)) {
			LOG_ERROR << "Simconfig: failed to init antenna configuration." << ENDL;
			return false;
		}
		LOG_INFO << "Simconfig: init antenna configuration success." << ENDL;
	}
	else {
		LOG_ERROR << "Simconfig: antenna configuration file empty." << ENDL;
		return false;
	}

	//-----------------------------------------------------------发射天线文件读取-------------------------------------------------------------------------
	if (!m_transmitterConfigFile.empty()) {
		if (!m_transmitterConfig.Init(m_transmitterConfigFile)) {
			LOG_ERROR << "Simconfig: failed to init transmitter configuration." << ENDL;
			return false;
		}
		LOG_INFO << "Simconfig: init transmitter configuration success." << ENDL;
	}
	else {
		LOG_ERROR << "Simconfig: transmitter configuration file empty." << ENDL;
		return false;
	}

	//-----------------------------------------------------------接收天线文件读取-------------------------------------------------------------------------
	if (!m_receiverConfigFile.empty()) {
		if (!m_receiverConfig.Init(m_receiverConfigFile)) {
			LOG_ERROR << "Simconfig: failed to init receiver configuration." << ENDL;
			return false;
		}
		LOG_INFO << "Simconfig: init receiver configuration success." << ENDL;
	}
	else {
		LOG_ERROR << "Simconfig: receiver configuration file empty." << ENDL;
		return false;
	}

	//------------------------------------------------------------传感器文件读取-------------------------------------------------------------------------
	if (!m_sensorConfigFile.empty()) {
		if (!m_sensorConfig.Init(m_sensorConfigFile)) {
			LOG_ERROR << "Simconfig: failed to init sensor configuration." << ENDL;
			return false;
		}
		LOG_INFO << "Simconfig: init sensor configuration success." << ENDL;
	}
	else {
		LOG_ERROR << "Simconfig: sensor configuration file empty." << ENDL;
		return false;
	}

	if (!Valid()) {			//若数据验证不通过，则返回false
		return false;
	}

	return true;
}

void SimConfig::Writer2Json(const std::string& filename)
{
	std::ofstream config_ofs(filename);
	if (!config_ofs.is_open()) {
		LOG_ERROR << "SimConfig" << filename << ": file not exist or can't open." << ENDL;
		return;
	}
	rapidjson::StringBuffer sb;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
	writer.SetFormatOptions(rapidjson::PrettyFormatOptions::kFormatSingleLineArray);
	writer.SetIndent('\t',1);
	writer.StartObject();
	writer.Key(KEY_SIMCONFIG.c_str()); this->Serialize(writer);
	writer.EndObject();
	config_ofs << sb.GetString();
	config_ofs.close();
	m_geometryConfig.Write2Json(m_geometryConfigFile);
	m_materialLibraryConfig.Write2Json(m_materialLibraryConfigFile);
	m_antennaLibraryConfig.Write2Json(m_antennaLibraryConfigFile);
	m_transmitterConfig.Write2Json(m_transmitterConfigFile);
	m_receiverConfig.Write2Json(m_receiverConfigFile);
	return;
}



void SimConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_SIMCONFIG_SYSTEMMODE.c_str());									writer.Uint(m_systemMode);
	writer.Key(KEY_SIMCONFIG_GEOMETRYCONFIGFILE.c_str());							writer.String(m_geometryConfigFile.c_str());
	writer.Key(KEY_SIMCONFIG_MATERIALLIBRARYCONFIGFILE.c_str());					writer.String(m_materialLibraryConfigFile.c_str());
	writer.Key(KEY_SIMCOFNIG_ANTENNALIBRARYCONFIGFILE.c_str());						writer.String(m_antennaLibraryConfigFile.c_str());
	writer.Key(KEY_SIMCONFIG_TRANSMITTERCONFIGFILE.c_str());						writer.String(m_transmitterConfigFile.c_str());
	writer.Key(KEY_SIMCONFIG_RECEIVERCONFIGFILE.c_str());							writer.String(m_receiverConfigFile.c_str());
	writer.Key(KEY_SIMCONFIG_SENSORCONFIGFILE.c_str());								writer.String(m_sensorConfigFile.c_str());
	writer.Key(KEY_SIMCONFIG_RAYTRACINGCONFIG.c_str());								m_raytracingConfig.Serialize(writer);
	writer.Key(KEY_SIMCONFIG_LOCALIZATIONCONFIG.c_str());							m_lbsConfig.Serialize(writer);
	writer.Key(KEY_SIMCONFIG_FREQUENCYCONFIG.c_str());								m_frequencyConfig.Serialize(writer);
	writer.Key(KEY_SIMCONFIG_OUTPUTCONFIG.c_str());									m_outputConfig.Serialize(writer);
	writer.EndObject();
}

bool SimConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "SimConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_SIMCONFIG_SYSTEMMODE.c_str())) {
		LOG_ERROR << "SimConfig: missing  " << KEY_SIMCONFIG_SYSTEMMODE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SIMCONFIG_FREQUENCYCONFIG.c_str())) {
		LOG_ERROR << "SimConfig: missing  " << KEY_SIMCONFIG_FREQUENCYCONFIG.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SIMCONFIG_RAYTRACINGCONFIG.c_str())) {
		LOG_ERROR << "SimConfig: missing  " << KEY_SIMCONFIG_RAYTRACINGCONFIG.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SIMCONFIG_LOCALIZATIONCONFIG.c_str())) {
		LOG_ERROR << "SimConfig: missing  " << KEY_SIMCONFIG_LOCALIZATIONCONFIG.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SIMCONFIG_OUTPUTCONFIG.c_str())) {
		LOG_ERROR << "SimConfig: missing  " << KEY_SIMCONFIG_OUTPUTCONFIG.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SIMCONFIG_GEOMETRYCONFIGFILE.c_str())) {
		LOG_ERROR << "SimConfig: missing  " << KEY_SIMCONFIG_GEOMETRYCONFIGFILE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SIMCONFIG_MATERIALLIBRARYCONFIGFILE.c_str())) {
		LOG_ERROR << "SimConfig: missing  " << KEY_SIMCONFIG_MATERIALLIBRARYCONFIGFILE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SIMCOFNIG_ANTENNALIBRARYCONFIGFILE.c_str())) {
		LOG_ERROR << "SimConfig: missing  " << KEY_SIMCOFNIG_ANTENNALIBRARYCONFIGFILE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SIMCONFIG_TRANSMITTERCONFIGFILE.c_str())) {
		LOG_ERROR << "SimConfig: missing  " << KEY_SIMCONFIG_TRANSMITTERCONFIGFILE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SIMCONFIG_RECEIVERCONFIGFILE.c_str())) {
		LOG_ERROR << "SimConfig: missing  " << KEY_SIMCONFIG_RECEIVERCONFIGFILE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SIMCONFIG_SENSORCONFIGFILE.c_str())) {
		LOG_ERROR << "SimConfig: missing  " << KEY_SIMCONFIG_SENSORCONFIGFILE.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& systemModeValue = value[KEY_SIMCONFIG_SYSTEMMODE.c_str()];
	const rapidjson::Value& frequencyConfigValue = value[KEY_SIMCONFIG_FREQUENCYCONFIG.c_str()];
	const rapidjson::Value& raytracingConfigValue = value[KEY_SIMCONFIG_RAYTRACINGCONFIG.c_str()];
	const rapidjson::Value& localizeConfigValue = value[KEY_SIMCONFIG_LOCALIZATIONCONFIG.c_str()];
	const rapidjson::Value& outputConfigValue = value[KEY_SIMCONFIG_OUTPUTCONFIG.c_str()];
	const rapidjson::Value& geometryConfigFileValue = value[KEY_SIMCONFIG_GEOMETRYCONFIGFILE.c_str()];
	const rapidjson::Value& materialLibraryConfigFileValue = value[KEY_SIMCONFIG_MATERIALLIBRARYCONFIGFILE.c_str()];
	const rapidjson::Value& antennaLibraryConfigFileValue = value[KEY_SIMCOFNIG_ANTENNALIBRARYCONFIGFILE.c_str()];
	const rapidjson::Value& transmitterConfigFileValue = value[KEY_SIMCONFIG_TRANSMITTERCONFIGFILE.c_str()];
	const rapidjson::Value& receiverConfigFileValue = value[KEY_SIMCONFIG_RECEIVERCONFIGFILE.c_str()];
	const rapidjson::Value& sensorConfigFileValue = value[KEY_SIMCONFIG_SENSORCONFIGFILE.c_str()];

	if (!systemModeValue.IsUint()) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_SYSTEMMODE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!frequencyConfigValue.IsObject()) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_FREQUENCYCONFIG.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!raytracingConfigValue.IsObject()) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_RAYTRACINGCONFIG.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!localizeConfigValue.IsObject()) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_LOCALIZATIONCONFIG.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputConfigValue.IsObject()) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_OUTPUTCONFIG.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!geometryConfigFileValue.IsString()) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_GEOMETRYCONFIGFILE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!materialLibraryConfigFileValue.IsString()) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_MATERIALLIBRARYCONFIGFILE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!antennaLibraryConfigFileValue.IsString()) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCOFNIG_ANTENNALIBRARYCONFIGFILE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!transmitterConfigFileValue.IsString()) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_TRANSMITTERCONFIGFILE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!receiverConfigFileValue.IsString()) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_RECEIVERCONFIGFILE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!sensorConfigFileValue.IsString()) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_SENSORCONFIGFILE.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_geometryConfigFile = geometryConfigFileValue.GetString();
	m_materialLibraryConfigFile = materialLibraryConfigFileValue.GetString();
	m_antennaLibraryConfigFile = antennaLibraryConfigFileValue.GetString();
	m_transmitterConfigFile = transmitterConfigFileValue.GetString();
	m_receiverConfigFile = receiverConfigFileValue.GetString();
	m_sensorConfigFile = sensorConfigFileValue.GetString();

	if (!DeserializeEnum(m_systemMode, systemModeValue)) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_SYSTEMMODE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_frequencyConfig.Deserialize(frequencyConfigValue)) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_FREQUENCYCONFIG.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_raytracingConfig.Deserialize(raytracingConfigValue)) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_RAYTRACINGCONFIG.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_lbsConfig.Deserialize(localizeConfigValue)) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_LOCALIZATIONCONFIG.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!m_outputConfig.Deserialize(outputConfigValue)) {
		LOG_ERROR << "SimConfig: " << KEY_SIMCONFIG_OUTPUTCONFIG.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	return true;
}

bool SimConfig::Valid() const
{
	if (m_systemMode == MODE_RT) {
		//1-验证传感器输出数据的有效性
		bool isOutputSensorData = false;					/** @brief	是否输出传感器数据标志	*/
		if (m_outputConfig.m_outputSensorDataSPSTMD) {		//单站单源多数据定位, 接收机为1个，发射机可以为多个（多次定位）
			if (m_receiverConfig.m_receiverconfigs.size()!=1) {
				LOG_ERROR << "SimConfig: receivers not satisfy SPSTMD Localization Condition." << ENDL;
				return false;
			}
			isOutputSensorData = true;
		}
		if (m_outputConfig.m_outputSensorDataMPSTSD) {		//多站单源单数据定位，接收机为多个，发射机可以为多个（多次定位
			if (m_receiverConfig.m_receiverconfigs.size() < 2) {
				LOG_ERROR << "SimConfig: receivers not satisfy MPSTSD Localization Condition." << ENDL;
				return false;
			}
			isOutputSensorData = true;
		}
		if (m_outputConfig.m_outputSensorDataSPMTMD) {		//单站多源多数据定位，接收机为1个，发射机为多个
			if (m_transmitterConfig.m_transmitterConfigs.size() < 2) {
				LOG_ERROR << "SimConfig: transmitters not satisfy SPMTMD Localization Condition." << ENDL;
				return false;
			}
			if (m_receiverConfig.m_receiverconfigs.size() != 1) {
				LOG_ERROR << "SimConfig: receivers not satisfy SPMTMD Localization Condition." << ENDL;
				return false;
			}
			isOutputSensorData = true;
		}
		if (m_outputConfig.m_outputSensorDataMPMTMD) {		//多站多源多数据定位，接收机为多个，发射机也为多个
			if (m_transmitterConfig.m_transmitterConfigs.size() < 2) {
				LOG_ERROR << "SimConfig: transmitters not satisfy SPMTMD Localization Condition." << ENDL;
				return false;
			}
			if (m_receiverConfig.m_receiverconfigs.size() < 2) {
				LOG_ERROR << "SimConfig: receivers not satisfy SPMTMD Localization Condition." << ENDL;
				return false;
			}
			isOutputSensorData = true;
		}
		if (isOutputSensorData) {				//若开启了定位模式，则频率数量需要固定为单频点
			if (m_frequencyConfig.m_subCarrierNum != 1) {
				LOG_ERROR << "SimConfig: frequencyconfig not satisfy SPMTMD Localization Condition." << ENDL;
				return false;
			}
		}
	}
	else if (m_systemMode == MODE_LBS) {
		if (m_lbsConfig.m_lbsMode == LBS_MODE_SPSTMD) {
			if (m_sensorConfig.m_sensorConfigs.size() != 1) {
				LOG_ERROR << "Simconfig: sensor num not satisfied SPSTMD Mode. Sensor num must equal to 1." << ENDL;
				return false;
			}
		}
		else if (m_lbsConfig.m_lbsMode == LBS_MODE_MPSTSD) {
			if (m_lbsConfig.m_lbsMethod == LBS_METHOD_RT_TDOA || m_lbsConfig.m_lbsMethod == LBS_METHOD_RT_AOA_TDOA) {
				if (m_sensorConfig.m_sensorConfigs.size() < 3) {
					LOG_ERROR << "Simconfig: sensor num not satisfied MPSTSD Mode. Sensor num must large than 3." << ENDL;
					return false;
				}
			}
			else {
				if (m_sensorConfig.m_sensorConfigs.size() < 2) {
					LOG_ERROR << "Simconfig: sensor num not satisfied MPSTSD Mode. Sensor num must large than 2." << ENDL;
					return false;
				}
			}
		}
		else if (m_lbsConfig.m_lbsMode == LBS_MODE_SPMTMD) {
			if (m_sensorConfig.m_sensorConfigs.size() != 1) {
				LOG_ERROR << "Simconfig: sensor num not satisfied SPMTMD Mode. Sensor num must equal to 1." << ENDL;
				return false;
			}
		}
		else if (m_lbsConfig.m_lbsMode == LBS_MODE_MPMTMD) {
			if (m_sensorConfig.m_sensorConfigs.size() != 1) {
				LOG_ERROR << "Simconfig: sensor num not satisfied MPMTMD Mode. Sensor num must equal to 1." << ENDL;
				return false;
			}
		}
	}
	return true;
}
