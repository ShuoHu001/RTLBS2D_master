#include "antennalibrary.h"

AntennaLibrary::AntennaLibrary()
{
	InitInternalAntennas();
}

AntennaLibrary::~AntennaLibrary()
{
    m_antennas.clear();
    m_antennas.shrink_to_fit();
}

Antenna* AntennaLibrary::GetAntenna(unsigned id) const
{
	for (auto it = m_antennas.begin(); it != m_antennas.end(); ++it) {
		const Antenna* ant = *it;
		if (ant->m_typeId == id)
			return new Antenna(*ant);
	}
	return nullptr;
}

Antenna* AntennaLibrary::GetAntenna(const std::string name) const
{
	for (auto it = m_antennas.begin(); it != m_antennas.end(); ++it) {
		const Antenna* ant = *it;
		if (ant->m_antName == name)
			return new Antenna(*ant);
	}
	return nullptr;
}

Antenna* AntennaLibrary::GetAntenna(unsigned id, const Euler& posture) const
{
	for (auto it = m_antennas.begin(); it != m_antennas.end(); ++it) {
		const Antenna* ant = *it;
		if (ant->m_typeId == id) {
			Antenna* antReVal = new Antenna(*ant);
			antReVal->m_posture = posture;
			return antReVal;
		}
	}
	return nullptr;
}

Antenna* AntennaLibrary::GetAntenna(const std::string name, const Euler& posture) const
{
	for (auto it = m_antennas.begin(); it != m_antennas.end(); ++it) {
		const Antenna* ant = *it;
		if (ant->m_antName == name) {
			Antenna* antReVal = new Antenna(*ant);
			antReVal->m_posture = posture;
			return antReVal;
		}
	}
	return nullptr;
}

bool AntennaLibrary::Init(const AntennaLibraryConfig& config)
{
	const std::vector<AntennaConfig> configs = config.m_antennas;
	if (configs.size() == 0)										//修正，若天线配置数为0,则启用内部定义5类天线方向图
		return true;
	int originSize = static_cast<int>(m_antennas.size());
	m_antennas.resize(originSize + configs.size());
	for (int i = 0; i < configs.size(); ++i) {
		Antenna* antenna = new Antenna(configs[i]);
		m_antennas[originSize + i] = antenna;
	}
	return false;
}

void AntennaLibrary::InitInternalAntennas()
{
	//目前内置的天线方向图主要分为两种类型，一种为全向型天线，另一种问半波偶极子类型的天线
	Antenna* ant_Omni = new Antenna();																	//全向天线
	ant_Omni->m_antId = 0;
	ant_Omni->m_typeId = 0;
	ant_Omni->m_antName = "0-Omni";
	ant_Omni->m_gain = 0;
	ant_Omni->m_freqMin = 100e6;
	ant_Omni->m_freqMax = 100e9;
	ant_Omni->m_polarization = Vector3D(0, 0, 1, true);
	m_antennas.push_back(ant_Omni);

	Antenna* ant_WaveDipole = new Antenna();																//全波偶极子天线
	ant_WaveDipole->m_antId = 1;
	ant_WaveDipole->m_typeId = 1;
	ant_WaveDipole->m_antName = "1-full-dipole";
	ant_WaveDipole->m_gain = 1.2589;
	ant_WaveDipole->m_freqMin = 100e6;
	ant_WaveDipole->m_freqMax = 100e9;
	ant_WaveDipole->m_polarization = Vector3D(0, 0, 1, true);
	m_antennas.push_back(ant_WaveDipole);

	Antenna* ant_HalfWaveDipole = new Antenna();															//半波偶极子天线
	ant_HalfWaveDipole->m_antId = 2;
	ant_HalfWaveDipole->m_typeId = 2;
	ant_HalfWaveDipole->m_antName = "2-half-dipole";
	ant_HalfWaveDipole->m_gain = 1.2589;
	ant_HalfWaveDipole->m_freqMin = 100e6;
	ant_HalfWaveDipole->m_freqMax = 100e9;
	ant_HalfWaveDipole->m_polarization = Vector3D(0, 0, 1, true);
	m_antennas.push_back(ant_HalfWaveDipole);

	Antenna* ant_ThreeHalfWaveDipole = new Antenna();													//三倍半波偶极子天线
	ant_ThreeHalfWaveDipole->m_antId = 3;
	ant_ThreeHalfWaveDipole->m_typeId = 3;
	ant_ThreeHalfWaveDipole->m_antName = "3-three-half-dipole";
	ant_ThreeHalfWaveDipole->m_gain = 1.2589;
	ant_ThreeHalfWaveDipole->m_freqMin = 100e6;
	ant_ThreeHalfWaveDipole->m_freqMax = 100e9;
	ant_ThreeHalfWaveDipole->m_polarization = Vector3D(0, 0, 1, true);
	m_antennas.push_back(ant_ThreeHalfWaveDipole);

	Antenna* ant_SingleWoundSpiral = new Antenna();														//单饶螺旋天线
	ant_SingleWoundSpiral->m_antId = 4;
	ant_SingleWoundSpiral->m_typeId = 4;
	ant_SingleWoundSpiral->m_antName = "4-single-wound-helical";
	ant_SingleWoundSpiral->m_gain = 1.2589;
	ant_SingleWoundSpiral->m_freqMin = 100e6;
	ant_SingleWoundSpiral->m_freqMax = 100e9;
	ant_SingleWoundSpiral->m_polarization = Vector3D(0, 0, 1, true);
	m_antennas.push_back(ant_SingleWoundSpiral);
}
