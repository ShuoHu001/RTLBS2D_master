#include "pathinfo.h"

PathInfo::PathInfo()
	: m_rayPathType(RAYPATH_COMMON)
	, m_rayPath(nullptr)
	, m_terrainDiffractionPath(nullptr)
	, m_freq(1e9)
	, m_power(0.0)
	, m_powerRatio(0.0)
	, m_scalarEField(0.0)
	, m_timeDelay(0.0)
	, m_phaseOffset(0.0)
	, m_aoDPhi(0.0)
	, m_aoDTheta(0.0)
	, m_aoAPhi(0.0)
	, m_aoATheta(0.0)
{
}

PathInfo::PathInfo(const PathInfo& pathInfo)
	: m_rayPathType(pathInfo.m_rayPathType)
	, m_rayPath(pathInfo.m_rayPath)
	, m_terrainDiffractionPath(pathInfo.m_terrainDiffractionPath)
	, m_freq(pathInfo.m_freq)
	, m_power(pathInfo.m_power)
	, m_powerRatio(pathInfo.m_powerRatio)
	, m_scalarEField(pathInfo.m_scalarEField)
	, m_vectorEField(pathInfo.m_vectorEField)
	, m_magnitude(pathInfo.m_magnitude)
	, m_timeDelay(pathInfo.m_timeDelay)
	, m_phaseOffset(pathInfo.m_phaseOffset)
	, m_aoDPhi(pathInfo.m_aoDPhi)
	, m_aoDTheta(pathInfo.m_aoDTheta)
	, m_aoAPhi(pathInfo.m_aoAPhi)
	, m_aoATheta(pathInfo.m_aoATheta)
{
}

PathInfo::~PathInfo()
{
}

PathInfo& PathInfo::operator=(const PathInfo& pathInfo)
{
	m_rayPathType = pathInfo.m_rayPathType;
	m_rayPath = pathInfo.m_rayPath;
	m_terrainDiffractionPath = pathInfo.m_terrainDiffractionPath;
	m_freq = pathInfo.m_freq;
	m_power = pathInfo.m_power;
	m_powerRatio = pathInfo.m_powerRatio;
	m_scalarEField = pathInfo.m_scalarEField;
	m_vectorEField = pathInfo.m_vectorEField;
	m_magnitude = pathInfo.m_magnitude;
	m_timeDelay = pathInfo.m_timeDelay;
	m_phaseOffset = pathInfo.m_phaseOffset;
	m_aoDPhi = pathInfo.m_aoDPhi;
	m_aoDTheta = pathInfo.m_aoDTheta;
	m_aoAPhi = pathInfo.m_aoAPhi;
	m_aoATheta = pathInfo.m_aoATheta;
	return *this;
}

void PathInfo::SetRayPath(RayPath3D* path)
{
	m_rayPath = path;
	m_rayPathType = path->m_type;
}

void PathInfo::SetRayPath(TerrainDiffractionPath* path)
{
	m_terrainDiffractionPath = path;
	m_rayPathType = RAYPATH_TERRAIN_DIFFRACTION;
}

void PathInfo::CalculateBaseInfo(RtLbsType freq, const MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction, Transmitter* transmitter, Receiver* receiver)
{
	//赋值
	m_freq = freq;
	RtLbsType lamda = LIGHT_VELOCITY_AIR / freq;				//波长，单位m
	//1-计算电磁场
	if (m_rayPathType != RAYPATH_TERRAIN_DIFFRACTION) {			//常规路径电磁计算方法
		m_vectorEField = m_rayPath->CalculateStrengthField(transmitter->m_power, freq, tranFunction, matLibrary, transmitter->m_antenna, receiver->m_antenna);
		m_scalarEField = m_vectorEField.MValue();
		m_timeDelay = m_rayPath->GetPropagationTime();
		m_phaseOffset = m_rayPath->GetPhaseOffset(freq);
		m_aoDPhi = m_rayPath->GetAOD_Phi();
		m_aoDTheta = m_rayPath->GetAOD_Theta();
		m_aoAPhi = m_rayPath->GetAOA_Phi();
		m_aoATheta = m_rayPath->GetAOA_Theta();
	}
	else {														//绕射路径电磁计算方法
		m_vectorEField = m_terrainDiffractionPath->CalculateTerrainDiffractionEField(transmitter->m_power, freq, matLibrary, tranFunction, transmitter->m_antenna, receiver->m_antenna);
		m_scalarEField = m_vectorEField.MValue();
		m_timeDelay = m_terrainDiffractionPath->GetPropagationTime();
		m_phaseOffset = m_terrainDiffractionPath->GetPhaseOffset(freq);
		m_aoDPhi = m_terrainDiffractionPath->GetAngleofDeparture_Phi();
		m_aoDTheta = m_terrainDiffractionPath->GetAngleofDeparture_Theta();
		m_aoAPhi = m_terrainDiffractionPath->GetAngleofArrival_Phi();
		m_aoATheta = m_terrainDiffractionPath->GetAngleofArrival_Theta();

	}
	//2-计算功率
	RtLbsType gain = transmitter->GetGain() + receiver->GetGain();									//获取系统总增益
	RtLbsType conterm = 20 * log10(LIGHT_VELOCITY_AIR / (freq / QUARTER_PI)) - 20 * log10(30);		//中间变量
	m_power = 20 * log10(m_scalarEField) + conterm + 30 + gain;										//计算功率
	m_magnitude.Init(pow(10,m_power/10), m_phaseOffset);												//计算复数强度
}

void PathInfo::CalculateBaseInfo(RtLbsType power, RtLbsType freq, const AntennaLibrary* antLibrary, const MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction, const Sensor* sensor)
{
	//赋值
	m_freq = freq;
	RtLbsType lamda = LIGHT_VELOCITY_AIR / freq;				//波长，单位m
	const Antenna* omniAntenna = antLibrary->GetAntenna(0);			//获取全向天线
	//1-计算电磁场
	if (m_rayPathType != RAYPATH_TERRAIN_DIFFRACTION) {			//常规路径电磁计算方法
		m_vectorEField = m_rayPath->CalculateStrengthField(power, freq, tranFunction, matLibrary, omniAntenna, sensor->m_antenna);
		m_scalarEField = m_vectorEField.MValue();
		m_timeDelay = m_rayPath->GetPropagationTime();
		m_phaseOffset = m_rayPath->GetPhaseOffset(freq);
		m_aoDPhi = m_rayPath->GetAOD_Phi();
		m_aoDTheta = m_rayPath->GetAOD_Theta();
		m_aoAPhi = m_rayPath->GetAOA_Phi();
		m_aoATheta = m_rayPath->GetAOA_Theta();
	}
	else {														//绕射路径电磁计算方法
		m_vectorEField = m_terrainDiffractionPath->CalculateTerrainDiffractionEField(power, freq, matLibrary, tranFunction, omniAntenna, sensor->m_antenna);
		m_scalarEField = m_vectorEField.MValue();
		m_timeDelay = m_terrainDiffractionPath->GetPropagationTime();
		m_phaseOffset = m_terrainDiffractionPath->GetPhaseOffset(freq);
		m_aoDPhi = m_terrainDiffractionPath->GetAngleofDeparture_Phi();
		m_aoDTheta = m_terrainDiffractionPath->GetAngleofDeparture_Theta();
		m_aoAPhi = m_terrainDiffractionPath->GetAngleofArrival_Phi();
		m_aoATheta = m_terrainDiffractionPath->GetAngleofArrival_Theta();

	}
	//2-计算功率
	RtLbsType gain = sensor->GetGain();																//获取系统总增益
	RtLbsType conterm = 20 * log10(LIGHT_VELOCITY_AIR / (freq / QUARTER_PI)) - 20 * log10(30);		//中间变量
	m_power = 20 * log10(m_scalarEField) + conterm + 30 + gain;										//计算功率
	m_magnitude.Init(pow(10, m_power / 10), m_phaseOffset);											//计算复数强度
}

void PathInfo::Convert2SensorData(SensorData& data) const
{
	data.m_power = m_power;
	data.m_phi = m_aoAPhi;
	data.m_phiDegree = m_aoAPhi / PI * 180;
	data.m_time = m_timeDelay;
	data.m_timeDiff = 0.0;
}
