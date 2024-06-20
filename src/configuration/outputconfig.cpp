#include "outputconfig.h"

OutputConfig::OutputConfig()
	: m_outputPower(false)
	, m_outputLoss(false)
	, m_outputMagnitude(false)
	, m_outputMultipath(false)
	, m_outputPDP(false)
	, m_outputCFR(false)
	, m_outputCIR(false)
	, m_outputAOA(false)
	, m_outputAOD(false)
	, m_outputSensorDataSPSTMD(false)
	, m_outputSensorDataMPSTSD(false)
	, m_outputSensorDataSPMTMD(false)
	, m_outputSensorDataMPMTMD(false)
	, m_outputSensorDataSparseFactor(1.0)
{
}

OutputConfig::OutputConfig(const OutputConfig& config)
	: m_rtDirectory(config.m_rtDirectory)
	, m_lbsDirectory(config.m_lbsDirectory)
	, m_outputPower(config.m_outputPower)
	, m_outputLoss(config.m_outputLoss)
	, m_outputMagnitude(config.m_outputMagnitude)
	, m_outputMultipath(config.m_outputMultipath)
	, m_outputPDP(config.m_outputPDP)
	, m_outputCFR(config.m_outputCFR)
	, m_outputCIR(config.m_outputCIR)
	, m_outputAOA(config.m_outputAOA)
	, m_outputAOD(config.m_outputAOD)
	, m_outputSensorDataSPSTMD(config.m_outputSensorDataSPSTMD)
	, m_outputSensorDataMPSTSD(config.m_outputSensorDataMPSTSD)
	, m_outputSensorDataSPMTMD(config.m_outputSensorDataSPMTMD)
	, m_outputSensorDataMPMTMD(config.m_outputSensorDataMPMTMD)
	, m_outputSensorDataSparseFactor(config.m_outputSensorDataSparseFactor)
{
}

OutputConfig::~OutputConfig()
{
}

OutputConfig& OutputConfig::operator=(const OutputConfig& config)
{
	m_rtDirectory = config.m_rtDirectory;
	m_lbsDirectory = config.m_lbsDirectory;
	m_outputPower = config.m_outputPower;
	m_outputLoss = config.m_outputLoss;
	m_outputMagnitude=config.m_outputMagnitude;
	m_outputMultipath=config.m_outputMultipath;
	m_outputPDP=config.m_outputPDP;
	m_outputCFR=config.m_outputCFR;
	m_outputCIR=config.m_outputCIR;
	m_outputAOA=config.m_outputAOA;
	m_outputAOD=config.m_outputAOD;
	m_outputSensorDataSPSTMD=config.m_outputSensorDataSPSTMD;
	m_outputSensorDataMPSTSD=config.m_outputSensorDataMPSTSD;
	m_outputSensorDataSPMTMD=config.m_outputSensorDataSPMTMD;
	m_outputSensorDataMPMTMD=config.m_outputSensorDataMPMTMD;
	m_outputSensorDataSparseFactor = config.m_outputSensorDataSparseFactor;
	return *this;
}

bool OutputConfig::IsValid() const
{
	int sensorDataOutputStateSum =
		static_cast<int>(m_outputSensorDataSPSTMD) +
		static_cast<int>(m_outputSensorDataMPSTSD) +
		static_cast<int>(m_outputSensorDataSPMTMD) +
		static_cast<int>(m_outputSensorDataMPMTMD);
	if (sensorDataOutputStateSum > 1) {					//若数量总数大于1，则表明两种状态同时存在，错误
		LOG_ERROR << "OutputConfig: sensor data output flag can't exist simultaneously." << ENDL;
		return false;
	}
	if (m_outputSensorDataSparseFactor < 0.1) {
		LOG_ERROR << "OuputConfig: sensor data output sparse factor must greater than 0.1." << ENDL;
		return false;
	}
	return true;
}

void OutputConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_OUTPUTCONFIG_RTDIRECTORY.c_str());										writer.String(m_rtDirectory.c_str());
	writer.Key(KEY_OUTPUTCONFIG_LBSDIRECTORY.c_str());										writer.String(m_lbsDirectory.c_str());
	writer.Key(KEY_OUTPUTCONFIG_OUTPUTPOWER.c_str());										writer.Bool(m_outputPower);
	writer.Key(KEY_OUTPUTCONFIG_OUTPUTLOSS.c_str());										writer.Bool(m_outputLoss);
	writer.Key(KEY_OUTPUTCONFIG_OUTPUTMAGNITUDE.c_str());									writer.Bool(m_outputMagnitude);
	writer.Key(KEY_OUTPUTCONFIG_OUTPUTMULTIPATH.c_str());									writer.Bool(m_outputMultipath);
	writer.Key(KEY_OUTPUTCONFIG_OUTPUTPDP.c_str());											writer.Bool(m_outputPDP);
	writer.Key(KEY_OUTPUTCONFIG_OUTPUTCFR.c_str());											writer.Bool(m_outputCFR);
	writer.Key(KEY_OUTPUTCONFIG_OUTPUTCIR.c_str());											writer.Bool(m_outputCIR);
	writer.Key(KEY_OUTPUTCONFIG_OUTPUTAOA.c_str());											writer.Bool(m_outputAOA);
	writer.Key(KEY_OUTPUTCONFIG_OUTPUTAOD.c_str());											writer.Bool(m_outputAOD);
	writer.Key(KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_SPSTMD.c_str());							writer.Bool(m_outputSensorDataSPSTMD);
	writer.Key(KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_MPSTSD.c_str());							writer.Bool(m_outputSensorDataMPSTSD);
	writer.Key(KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_SPMTMD.c_str());							writer.Bool(m_outputSensorDataSPMTMD);
	writer.Key(KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_MPMTMD.c_str());							writer.Bool(m_outputSensorDataMPMTMD);
	writer.Key(KEY_OUTPUTCONFIG_OUTPUTSENSORDATASPARSEFACTOR.c_str());							writer.Double(m_outputSensorDataSparseFactor);
	writer.EndObject();
}

bool OutputConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "OutputConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_OUTPUTCONFIG_RTDIRECTORY.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_RTDIRECTORY.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_LBSDIRECTORY.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_LBSDIRECTORY.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_OUTPUTPOWER.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_OUTPUTPOWER.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_OUTPUTLOSS.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_OUTPUTLOSS.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_OUTPUTMAGNITUDE.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_OUTPUTMAGNITUDE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_OUTPUTMULTIPATH.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_OUTPUTMULTIPATH.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_OUTPUTPDP.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_OUTPUTPDP.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_OUTPUTCFR.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_OUTPUTCFR.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_OUTPUTCIR.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_OUTPUTCIR.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_OUTPUTAOA.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_OUTPUTAOA.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_OUTPUTAOD.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_OUTPUTAOD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_SPSTMD.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_SPSTMD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_MPSTSD.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_MPSTSD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_SPMTMD.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_SPMTMD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_MPMTMD.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_MPMTMD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_OUTPUTCONFIG_OUTPUTSENSORDATASPARSEFACTOR.c_str())) {
		LOG_ERROR << "OutputConfig: missing " << KEY_OUTPUTCONFIG_OUTPUTSENSORDATASPARSEFACTOR.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& rtDirectoryValue = value[KEY_OUTPUTCONFIG_RTDIRECTORY.c_str()];
	const rapidjson::Value& lbsDirectoryValue = value[KEY_OUTPUTCONFIG_LBSDIRECTORY.c_str()];
	const rapidjson::Value& outputPowerValue = value[KEY_OUTPUTCONFIG_OUTPUTPOWER.c_str()];
	const rapidjson::Value& outputLossValue = value[KEY_OUTPUTCONFIG_OUTPUTLOSS.c_str()];
	const rapidjson::Value& outputMagnitudeValue = value[KEY_OUTPUTCONFIG_OUTPUTMAGNITUDE.c_str()];
	const rapidjson::Value& outputMultipathValue = value[KEY_OUTPUTCONFIG_OUTPUTMULTIPATH.c_str()];
	const rapidjson::Value& outputPDPValue = value[KEY_OUTPUTCONFIG_OUTPUTPDP.c_str()];
	const rapidjson::Value& outputCFRValue = value[KEY_OUTPUTCONFIG_OUTPUTCFR.c_str()];
	const rapidjson::Value& outputCIRValue = value[KEY_OUTPUTCONFIG_OUTPUTCIR.c_str()];
	const rapidjson::Value& outputAOAValue = value[KEY_OUTPUTCONFIG_OUTPUTAOA.c_str()];
	const rapidjson::Value& outputAODValue = value[KEY_OUTPUTCONFIG_OUTPUTAOD.c_str()];
	const rapidjson::Value& outputSensorDataSPSTMDValue = value[KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_SPSTMD.c_str()];
	const rapidjson::Value& outputSensorDataMPSTSDValue = value[KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_MPSTSD.c_str()];
	const rapidjson::Value& outputSensorDataSPMTMDValue = value[KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_SPMTMD.c_str()];
	const rapidjson::Value& outputSensorDataMPMTMDValue = value[KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_MPMTMD.c_str()];
	const rapidjson::Value& outputSensorDataSparseFactorValue = value[KEY_OUTPUTCONFIG_OUTPUTSENSORDATASPARSEFACTOR.c_str()];

	if (!rtDirectoryValue.IsString()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_RTDIRECTORY.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!lbsDirectoryValue.IsString()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_LBSDIRECTORY.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputPowerValue.IsBool()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_OUTPUTPOWER.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputLossValue.IsBool()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_OUTPUTLOSS.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputMagnitudeValue.IsBool()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_OUTPUTMAGNITUDE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputMultipathValue.IsBool()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_OUTPUTMULTIPATH.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputPDPValue.IsBool()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_OUTPUTPDP.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputCFRValue.IsBool()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_OUTPUTCFR.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputCIRValue.IsBool()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_OUTPUTCIR.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputAOAValue.IsBool()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_OUTPUTAOA.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputAODValue.IsBool()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_OUTPUTAOD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputSensorDataSPSTMDValue.IsBool()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_SPSTMD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputSensorDataMPSTSDValue.IsBool()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_MPSTSD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputSensorDataSPMTMDValue.IsBool()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_SPMTMD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputSensorDataMPMTMDValue.IsBool()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_MPMTMD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!outputSensorDataSparseFactorValue.IsDouble()) {
		LOG_ERROR << "OutputConfig: " << KEY_OUTPUTCONFIG_OUTPUTSENSORDATASPARSEFACTOR.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_rtDirectory = rtDirectoryValue.GetString();
	m_lbsDirectory = lbsDirectoryValue.GetString();
	m_outputPower = outputPowerValue.GetBool();
	m_outputLoss = outputLossValue.GetBool();
	m_outputMagnitude = outputMagnitudeValue.GetBool();
	m_outputMultipath = outputMultipathValue.GetBool();
	m_outputPDP = outputPDPValue.GetBool();
	m_outputCFR = outputCFRValue.GetBool();
	m_outputCIR = outputCIRValue.GetBool();
	m_outputAOA = outputAOAValue.GetBool();
	m_outputAOD = outputAODValue.GetBool();
	m_outputSensorDataSPSTMD = outputSensorDataSPSTMDValue.GetBool();
	m_outputSensorDataMPSTSD = outputSensorDataMPSTSDValue.GetBool();
	m_outputSensorDataSPMTMD = outputSensorDataSPMTMDValue.GetBool();
	m_outputSensorDataMPMTMD = outputSensorDataMPMTMDValue.GetBool();
	m_outputSensorDataSparseFactor = outputSensorDataSparseFactorValue.GetDouble();

	if (!IsValid()) {													//若验证不通过，返回false
		return false;
	}

	return true;
}
