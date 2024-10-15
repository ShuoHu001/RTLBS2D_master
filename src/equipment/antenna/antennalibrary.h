#ifndef RTLBS_ANTENNALIBRARY
#define RTLBS_ANTENNALIBRARY


#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "configuration/library/antennalibraryconfig.h"
#include "antenna.h"


//0-99为内置天线方向图
//100-为外置天线方向图

class AntennaLibrary {
private:
	std::vector<Antenna*> m_antennas;												/** @brief	天线库（外部天线库）	*/
	const static int m_interalAntNum = 5;											/** @brief	当前内部的天线库数量,每增加内置数量时需要进行更新此项	*/

public:
	AntennaLibrary();
	~AntennaLibrary();
	Antenna* GetAntenna(unsigned id) const;											//通过天线id来获取对应的天线对象
	Antenna* GetAntenna(const std::string name) const;								//通过天线名称来获取对应的天线对象
	Antenna* GetAntenna(unsigned id, const Euler& posture) const;					//通过天线Id和姿态来获取对应的天线对象
	Antenna* GetAntenna(const std::string name, const Euler& posture) const;		//通过天线名称和姿态来获取对应的天线对象
	bool Init(const AntennaLibraryConfig& config);									//初始化天线库

private:
	void InitInternalAntennas();													//加载内置的天线方向图
	

};

inline void InitInternalAntennas(std::vector<Antenna>& innerAntennas) {
	//目前内置的天线方向图主要分为两种类型，一种为全向型天线，另一种问半波偶极子类型的天线
	Antenna ant_Omni;																	//全向天线
	ant_Omni.m_antId = 0;
	ant_Omni.m_typeId = 0;
	ant_Omni.m_antName = "0-Omni";
	ant_Omni.m_gain = 0;
	ant_Omni.m_freqMin = 100e6;
	ant_Omni.m_freqMax = 100e9;
	ant_Omni.m_polarization = Vector3D(0, 0, 1, true);
	innerAntennas.push_back(ant_Omni);

	Antenna ant_WaveDipole;																//全波偶极子天线
	ant_WaveDipole.m_antId = 1;
	ant_WaveDipole.m_typeId = 1;
	ant_WaveDipole.m_antName = "1-full-dipole";
	ant_WaveDipole.m_gain = 1.2589;
	ant_WaveDipole.m_freqMin = 100e6;
	ant_WaveDipole.m_freqMax = 100e9;
	ant_WaveDipole.m_polarization = Vector3D(0, 0, 1, true);
	innerAntennas.push_back(ant_WaveDipole);

	Antenna ant_HalfWaveDipole;															//半波偶极子天线
	ant_HalfWaveDipole.m_antId = 2;
	ant_HalfWaveDipole.m_typeId = 2;
	ant_HalfWaveDipole.m_antName = "2-half-dipole";
	ant_HalfWaveDipole.m_gain = 1.2589;
	ant_HalfWaveDipole.m_freqMin = 100e6;
	ant_HalfWaveDipole.m_freqMax = 100e9;
	ant_HalfWaveDipole.m_polarization = Vector3D(0, 0, 1, true);
	innerAntennas.push_back(ant_HalfWaveDipole);

	Antenna ant_ThreeHalfWaveDipole;													//三倍半波偶极子天线
	ant_ThreeHalfWaveDipole.m_antId = 3;
	ant_ThreeHalfWaveDipole.m_typeId = 3;
	ant_ThreeHalfWaveDipole.m_antName = "3-three-half-dipole";
	ant_ThreeHalfWaveDipole.m_gain = 1.2589;
	ant_ThreeHalfWaveDipole.m_freqMin = 100e6;
	ant_ThreeHalfWaveDipole.m_freqMax = 100e9;
	ant_ThreeHalfWaveDipole.m_polarization = Vector3D(0, 0, 1, true);
	innerAntennas.push_back(ant_ThreeHalfWaveDipole);

	Antenna ant_SingleWoundSpiral;														//单饶螺旋天线
	ant_SingleWoundSpiral.m_antId = 4;
	ant_SingleWoundSpiral.m_typeId = 4;
	ant_SingleWoundSpiral.m_antName = "4-single-wound-helical";
	ant_SingleWoundSpiral.m_gain = 1.2589;
	ant_SingleWoundSpiral.m_freqMin = 100e6;
	ant_SingleWoundSpiral.m_freqMax = 100e9;
	ant_SingleWoundSpiral.m_polarization = Vector3D(0, 0, 1, true);
	innerAntennas.push_back(ant_SingleWoundSpiral);
}

#endif
