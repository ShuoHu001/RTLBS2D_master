#include "raytracingconfig.h"

RaytrcingConfig::RaytrcingConfig()
	: m_rayNum(1000)
	, m_diffractRayNum(3000)
	, m_raySplitFlag(false)
	, m_raySplitRadius(1.0)
	, m_sceneAccelType(ACCEL_NONE)
	, m_hardwareMode(CPU_SINGLETHREAD)
	, m_rayLaunchMode(UNIFORM_ICOSAHEDRON)
	, m_cpuThreadNum(20)
{
}

void RaytrcingConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_RAYTRACINGCONFIG_RAYNUM.c_str());										writer.Int64(m_rayNum);
	writer.Key(KET_RAYTRACINGCONFIG_DIFFRACTRAYNUM.c_str());								writer.Int64(m_diffractRayNum);
	writer.Key(KEY_RAYTRACINGCONFIG_LIMITINFO.c_str());										m_limitInfo.Serialize(writer);
	writer.Key(KEY_RAYTRACINGCONFIG_RAYSPLITFLAG.c_str());									writer.Bool(m_raySplitFlag);
	writer.Key(KEY_RAYTRACINGCONFIG_RAYSPLITRAIDUS.c_str());								writer.Double(m_raySplitRadius);
	writer.Key(KEY_RAYTRACINGCONFIG_SCENEACCELTYPE.c_str());								SerializeEnum(m_sceneAccelType, writer);
	writer.Key(KEY_RAYTRACINGCONFIG_HARDWAREMODE.c_str());									SerializeEnum(m_hardwareMode, writer);
	writer.Key(KEY_RAYTRACINGCONFIG_RAYLAUNCHMODE.c_str());									SerializeEnum(m_rayLaunchMode, writer);
	writer.Key(KEY_RAYTRACINGCONFIG_CPUTHREADNUM.c_str());									writer.Int(m_cpuThreadNum);
	writer.EndObject();
}

bool RaytrcingConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "RaytrcingConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_RAYTRACINGCONFIG_RAYNUM.c_str())) {
		LOG_ERROR << "RaytrcingConfig: missing " << KEY_RAYTRACINGCONFIG_RAYNUM.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KET_RAYTRACINGCONFIG_DIFFRACTRAYNUM.c_str())) {
		LOG_ERROR << "RaytrcingConfig: missing " << KET_RAYTRACINGCONFIG_DIFFRACTRAYNUM.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RAYTRACINGCONFIG_LIMITINFO.c_str())) {
		LOG_ERROR << "RaytrcingConfig: missing " << KEY_RAYTRACINGCONFIG_LIMITINFO.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RAYTRACINGCONFIG_RAYSPLITFLAG.c_str())) {
		LOG_ERROR << "RaytrcingConfig: missing " << KEY_RAYTRACINGCONFIG_RAYSPLITFLAG.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RAYTRACINGCONFIG_RAYSPLITRAIDUS.c_str())) {
		LOG_ERROR << "RaytrcingConfig: missing " << KEY_RAYTRACINGCONFIG_RAYSPLITRAIDUS.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RAYTRACINGCONFIG_SCENEACCELTYPE.c_str())) {
		LOG_ERROR << "RaytrcingConfig: missing " << KEY_RAYTRACINGCONFIG_SCENEACCELTYPE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RAYTRACINGCONFIG_HARDWAREMODE.c_str())) {
		LOG_ERROR << "RaytrcingConfig: missing " << KEY_RAYTRACINGCONFIG_HARDWAREMODE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RAYTRACINGCONFIG_RAYLAUNCHMODE.c_str())) {
		LOG_ERROR << "RaytrcingConfig: missing " << KEY_RAYTRACINGCONFIG_RAYLAUNCHMODE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_RAYTRACINGCONFIG_CPUTHREADNUM.c_str())) {
		LOG_ERROR << "RaytrcingConfig: missing " << KEY_RAYTRACINGCONFIG_CPUTHREADNUM.c_str() << ENDL;
		return false;
	}
	
	const rapidjson::Value& rayNumValue = value[KEY_RAYTRACINGCONFIG_RAYNUM.c_str()];
	const rapidjson::Value& diffractRayNumValue = value[KET_RAYTRACINGCONFIG_DIFFRACTRAYNUM.c_str()];
	const rapidjson::Value& limitInfoValue = value[KEY_RAYTRACINGCONFIG_LIMITINFO.c_str()];
	const rapidjson::Value& raySplitFlagValue = value[KEY_RAYTRACINGCONFIG_RAYSPLITFLAG.c_str()];
	const rapidjson::Value& raySplitRadiusValue = value[KEY_RAYTRACINGCONFIG_RAYSPLITRAIDUS.c_str()];
	const rapidjson::Value& sceneAccelTypeValue = value[KEY_RAYTRACINGCONFIG_SCENEACCELTYPE.c_str()];
	const rapidjson::Value& hardwareModeValue = value[KEY_RAYTRACINGCONFIG_HARDWAREMODE.c_str()];
	const rapidjson::Value& rayLaunchModeValue = value[KEY_RAYTRACINGCONFIG_RAYLAUNCHMODE.c_str()];
	const rapidjson::Value& cpuThreadNumValue = value[KEY_RAYTRACINGCONFIG_CPUTHREADNUM.c_str()];

	if (!rayNumValue.IsInt64()) {
		LOG_ERROR << "RaytrcingConfig: " << KEY_RAYTRACINGCONFIG_RAYNUM.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!diffractRayNumValue.IsInt()) {
		LOG_ERROR << "RaytrcingConfig: " << KET_RAYTRACINGCONFIG_DIFFRACTRAYNUM.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!limitInfoValue.IsObject()) {
		LOG_ERROR << "RaytrcingConfig: " << KEY_RAYTRACINGCONFIG_LIMITINFO.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!raySplitFlagValue.IsBool()) {
		LOG_ERROR << "RaytrcingConfig: " << KEY_RAYTRACINGCONFIG_RAYSPLITFLAG.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!raySplitRadiusValue.IsDouble()) {
		LOG_ERROR << "RaytrcingConfig: " << KEY_RAYTRACINGCONFIG_RAYSPLITRAIDUS.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!sceneAccelTypeValue.IsInt()) {
		LOG_ERROR << "RaytrcingConfig: " << KEY_RAYTRACINGCONFIG_SCENEACCELTYPE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!hardwareModeValue.IsInt()) {
		LOG_ERROR << "RaytrcingConfig: " << KEY_RAYTRACINGCONFIG_HARDWAREMODE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!rayLaunchModeValue.IsInt()) {
		LOG_ERROR << "RaytrcingConfig: " << KEY_RAYTRACINGCONFIG_RAYLAUNCHMODE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!cpuThreadNumValue.IsInt()) {
		LOG_ERROR << "RaytrcingConfig: " << KEY_RAYTRACINGCONFIG_CPUTHREADNUM.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	
	m_rayNum = rayNumValue.GetUint64();
	m_diffractRayNum = diffractRayNumValue.GetInt();
	m_raySplitFlag = raySplitFlagValue.GetBool();
	m_raySplitRadius = raySplitRadiusValue.GetDouble();
	m_cpuThreadNum = cpuThreadNumValue.GetInt();

	if (!DeserializeEnum(m_sceneAccelType, sceneAccelTypeValue)) {
		
	}
	if (!m_limitInfo.Deserialize(limitInfoValue)) {
		LOG_ERROR << "RaytrcingConfig: " << KEY_RAYTRACINGCONFIG_LIMITINFO.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	if (!DeserializeEnum(m_hardwareMode, hardwareModeValue)) {
		LOG_ERROR << "RaytrcingConfig: " << KEY_RAYTRACINGCONFIG_HARDWAREMODE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!DeserializeEnum(m_rayLaunchMode, rayLaunchModeValue)) {
		LOG_ERROR << "RaytrcingConfig: " << KEY_RAYTRACINGCONFIG_RAYLAUNCHMODE.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	return true;
}
