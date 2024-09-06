#include "localizeconfig.h"

LocalizeConfig::LocalizeConfig()
	: m_lbsMode(LBS_MODE_SPSTMD)
	, m_lbsMethod(LBS_METHOD_RT_AOA)
	, m_hardWareMode(CPU_SINGLETHREAD)
	, m_threadNum(4)
	, m_rayLaunchHalfTheta(0.01)
	, m_gsPairClusterThreshold(1.0)
	, m_extendAroundPointState(true)
	, m_hasSimuError(true)
{
}

LocalizeConfig::LocalizeConfig(const LocalizeConfig& config)
	: m_lbsMode(config.m_lbsMode)
	, m_lbsMethod(config.m_lbsMethod)
	, m_hardWareMode(config.m_hardWareMode)
	, m_threadNum(config.m_threadNum)
	, m_rayLaunchHalfTheta(config.m_rayLaunchHalfTheta)
	, m_gsPairClusterThreshold(config.m_gsPairClusterThreshold)
	, m_weightFactor(config.m_weightFactor)
	, m_extendAroundPointState(config.m_extendAroundPointState)
	, m_shiftErrorMatrixFileName(config.m_shiftErrorMatrixFileName)
	, m_hasSimuError(config.m_hasSimuError)
{
}

LocalizeConfig::~LocalizeConfig()
{
}

LocalizeConfig& LocalizeConfig::operator=(const LocalizeConfig& config)
{
	m_lbsMode = config.m_lbsMode;
	m_lbsMethod = config.m_lbsMethod;
	m_hardWareMode = config.m_hardWareMode;
	m_threadNum = config.m_threadNum;
	m_rayLaunchHalfTheta = config.m_rayLaunchHalfTheta;
	m_gsPairClusterThreshold = config.m_gsPairClusterThreshold;
	m_weightFactor = config.m_weightFactor;
	m_extendAroundPointState = config.m_extendAroundPointState;
	m_shiftErrorMatrixFileName = config.m_shiftErrorMatrixFileName;
	m_hasSimuError = config.m_hasSimuError;
	return *this;
}

void LocalizeConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE.c_str());							SerializeEnum(m_lbsMode, writer);
	writer.Key(KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD.c_str());							SerializeEnum(m_lbsMethod, writer);
	writer.Key(KEY_LOCALIZATIONCONFIG_HARDWAREMODE.c_str());								SerializeEnum(m_hardWareMode, writer);
	writer.Key(KEY_LOCALIZATIONCONFIG_THREADNUM.c_str());									writer.Uint(m_threadNum);
	writer.Key(KEY_LOCALIZATIONCONFIG_RAYLAUNCHHALFTHETA.c_str());							writer.Double(m_rayLaunchHalfTheta);
	writer.Key(KEY_LOCALIZATIONCONFIG_GSPAIRCLUSTERTHRESHOLD.c_str());						writer.Double(m_gsPairClusterThreshold);
	writer.Key(KEY_LOCALIZATIONCONFIG_WEIGHTFACTOR.c_str());								m_weightFactor.Serialize(writer);
	writer.Key(KEY_LOCALIZATIONCONFIG_EXTENDAROUNDPOINTSTATE.c_str());						writer.Bool(m_extendAroundPointState);
	writer.Key(KEY_LOCALIZATIONCONFIG_SHIFTERRORMATRIXFILENAME.c_str());					writer.String(m_shiftErrorMatrixFileName.c_str());
	writer.Key(KEY_LOCALIZATIONCONFIG_HASSIMUERROR.c_str());								writer.Bool(m_hasSimuError);
	writer.EndObject();
}

bool LocalizeConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "LocalizeConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE.c_str())) {
		LOG_ERROR << "LocalizeConfig: missing " << KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD.c_str())) {
		LOG_ERROR << "LocalizeConfig: missing " << KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LOCALIZATIONCONFIG_HARDWAREMODE.c_str())) {
		LOG_ERROR << "LocalizeConfig: missing " << KEY_LOCALIZATIONCONFIG_HARDWAREMODE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LOCALIZATIONCONFIG_THREADNUM.c_str())) {
		LOG_ERROR << "LocalizeConfig: missing " << KEY_LOCALIZATIONCONFIG_THREADNUM.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LOCALIZATIONCONFIG_RAYLAUNCHHALFTHETA.c_str())) {
		LOG_ERROR << "LocalizeConfig: missing " << KEY_LOCALIZATIONCONFIG_RAYLAUNCHHALFTHETA.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LOCALIZATIONCONFIG_GSPAIRCLUSTERTHRESHOLD.c_str())) {
		LOG_ERROR << "LocalizeConfig: missing " << KEY_LOCALIZATIONCONFIG_GSPAIRCLUSTERTHRESHOLD.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LOCALIZATIONCONFIG_WEIGHTFACTOR.c_str())) {
		LOG_ERROR << "LocalizeConfig: missing " << KEY_LOCALIZATIONCONFIG_WEIGHTFACTOR.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LOCALIZATIONCONFIG_EXTENDAROUNDPOINTSTATE.c_str())) {
		LOG_ERROR << "LocalizeConfig: missing " << KEY_LOCALIZATIONCONFIG_EXTENDAROUNDPOINTSTATE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LOCALIZATIONCONFIG_SHIFTERRORMATRIXFILENAME.c_str())) {
		LOG_ERROR << "LocalizeConfig: missing " << KEY_LOCALIZATIONCONFIG_SHIFTERRORMATRIXFILENAME.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LOCALIZATIONCONFIG_HASSIMUERROR.c_str())) {
		LOG_ERROR << "LocalizeConfig: missing " << KEY_LOCALIZATIONCONFIG_HASSIMUERROR.c_str() << ENDL;
		return false;
	}


	const rapidjson::Value& lbsModeValue = value[KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE.c_str()];
	const rapidjson::Value& lbsMethodValue = value[KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD.c_str()];
	const rapidjson::Value& hardwareModeValue = value[KEY_LOCALIZATIONCONFIG_HARDWAREMODE.c_str()];
	const rapidjson::Value& threadNumValue = value[KEY_LOCALIZATIONCONFIG_THREADNUM.c_str()];
	const rapidjson::Value& rayLaunchHalfThetaValue = value[KEY_LOCALIZATIONCONFIG_RAYLAUNCHHALFTHETA.c_str()];
	const rapidjson::Value& gsPairClusterThresholdValue = value[KEY_LOCALIZATIONCONFIG_GSPAIRCLUSTERTHRESHOLD.c_str()];
	const rapidjson::Value& weightFactorValue = value[KEY_LOCALIZATIONCONFIG_WEIGHTFACTOR.c_str()];
	const rapidjson::Value& extendAroundPointState = value[KEY_LOCALIZATIONCONFIG_EXTENDAROUNDPOINTSTATE.c_str()];
	const rapidjson::Value& shiftErrorMatrixFileNameValue = value[KEY_LOCALIZATIONCONFIG_SHIFTERRORMATRIXFILENAME.c_str()];
	const rapidjson::Value& hasSimuErrorValue = value[KEY_LOCALIZATIONCONFIG_HASSIMUERROR.c_str()];

	if (!lbsModeValue.IsInt()) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!lbsMethodValue.IsInt()) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!hardwareModeValue.IsInt()) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_HARDWAREMODE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!threadNumValue.IsUint()) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_THREADNUM.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!rayLaunchHalfThetaValue.IsDouble()) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_RAYLAUNCHHALFTHETA.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!gsPairClusterThresholdValue.IsDouble()) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_GSPAIRCLUSTERTHRESHOLD.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!weightFactorValue.IsObject()) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_WEIGHTFACTOR.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!extendAroundPointState.IsBool()) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_EXTENDAROUNDPOINTSTATE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!shiftErrorMatrixFileNameValue.IsString()) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_SHIFTERRORMATRIXFILENAME.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!hasSimuErrorValue.IsBool()) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_HASSIMUERROR.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_threadNum = static_cast<uint16_t>(threadNumValue.GetUint());
	m_rayLaunchHalfTheta = rayLaunchHalfThetaValue.GetDouble();
	m_gsPairClusterThreshold = gsPairClusterThresholdValue.GetDouble();
	m_extendAroundPointState = extendAroundPointState.GetBool();
	m_shiftErrorMatrixFileName = shiftErrorMatrixFileNameValue.GetString();
	m_hasSimuError = hasSimuErrorValue.GetBool();

	if (!DeserializeEnum(m_lbsMode, lbsModeValue)) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!DeserializeEnum(m_lbsMethod, lbsMethodValue)) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!DeserializeEnum(m_hardWareMode, hardwareModeValue)) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_HARDWAREMODE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	if (!m_weightFactor.Deserialize(weightFactorValue)) {
		LOG_ERROR << "LocalizeConfig: " << KEY_LOCALIZATIONCONFIG_WEIGHTFACTOR.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	if (!IsValid()) {
		return false;
	}

	return true;
}

bool LocalizeConfig::IsValid() const
{
	if (m_lbsMode == LBS_MODE_SPMTMD) {
		if (m_lbsMethod == LBS_METHOD_RT_AOA_TDOA || m_lbsMethod == LBS_METHOD_RT_TDOA) {
			LOG_ERROR << "LocalizeConfig: SPMTMD not support TDOA method." << ENDL;
			return false;
		}
	}
	else if (m_lbsMode == LBS_MODE_MPMTMD) {
		if (m_lbsMethod == LBS_METHOD_RT_AOA_TDOA || m_lbsMethod == LBS_METHOD_RT_TDOA) {
			LOG_ERROR << "LocalizeConfig: MPMTMD not support TDOA method." << ENDL;
			return false;
		}
	}
	return true;
}
