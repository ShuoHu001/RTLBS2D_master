#ifndef RTLBS_ANTENNALIBRARY
#define RTLBS_ANTENNALIBRARY


#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "configuration/library/antennalibraryconfig.h"
#include "antenna.h"


//0-99Ϊ�������߷���ͼ
//100-Ϊ�������߷���ͼ

class AntennaLibrary {
private:
	std::vector<Antenna*> m_antennas;												/** @brief	���߿⣨�ⲿ���߿⣩	*/
	const static int m_interalAntNum = 5;											/** @brief	��ǰ�ڲ������߿�����,ÿ������������ʱ��Ҫ���и��´���	*/

public:
	AntennaLibrary();
	~AntennaLibrary();
	Antenna* GetAntenna(unsigned id) const;											//ͨ������id����ȡ��Ӧ�����߶���
	Antenna* GetAntenna(const std::string name) const;								//ͨ��������������ȡ��Ӧ�����߶���
	Antenna* GetAntenna(unsigned id, const Euler& posture) const;					//ͨ������Id����̬����ȡ��Ӧ�����߶���
	Antenna* GetAntenna(const std::string name, const Euler& posture) const;		//ͨ���������ƺ���̬����ȡ��Ӧ�����߶���
	bool Init(const AntennaLibraryConfig& config);									//��ʼ�����߿�

private:
	void InitInternalAntennas();													//�������õ����߷���ͼ
	

};

inline void InitInternalAntennas(std::vector<Antenna>& innerAntennas) {
	//Ŀǰ���õ����߷���ͼ��Ҫ��Ϊ�������ͣ�һ��Ϊȫ�������ߣ���һ���ʰ벨ż�������͵�����
	Antenna ant_Omni;																	//ȫ������
	ant_Omni.m_antId = 0;
	ant_Omni.m_typeId = 0;
	ant_Omni.m_antName = "0-Omni";
	ant_Omni.m_gain = 0;
	ant_Omni.m_freqMin = 100e6;
	ant_Omni.m_freqMax = 100e9;
	ant_Omni.m_polarization = Vector3D(0, 0, 1, true);
	innerAntennas.push_back(ant_Omni);

	Antenna ant_WaveDipole;																//ȫ��ż��������
	ant_WaveDipole.m_antId = 1;
	ant_WaveDipole.m_typeId = 1;
	ant_WaveDipole.m_antName = "1-full-dipole";
	ant_WaveDipole.m_gain = 1.2589;
	ant_WaveDipole.m_freqMin = 100e6;
	ant_WaveDipole.m_freqMax = 100e9;
	ant_WaveDipole.m_polarization = Vector3D(0, 0, 1, true);
	innerAntennas.push_back(ant_WaveDipole);

	Antenna ant_HalfWaveDipole;															//�벨ż��������
	ant_HalfWaveDipole.m_antId = 2;
	ant_HalfWaveDipole.m_typeId = 2;
	ant_HalfWaveDipole.m_antName = "2-half-dipole";
	ant_HalfWaveDipole.m_gain = 1.2589;
	ant_HalfWaveDipole.m_freqMin = 100e6;
	ant_HalfWaveDipole.m_freqMax = 100e9;
	ant_HalfWaveDipole.m_polarization = Vector3D(0, 0, 1, true);
	innerAntennas.push_back(ant_HalfWaveDipole);

	Antenna ant_ThreeHalfWaveDipole;													//�����벨ż��������
	ant_ThreeHalfWaveDipole.m_antId = 3;
	ant_ThreeHalfWaveDipole.m_typeId = 3;
	ant_ThreeHalfWaveDipole.m_antName = "3-three-half-dipole";
	ant_ThreeHalfWaveDipole.m_gain = 1.2589;
	ant_ThreeHalfWaveDipole.m_freqMin = 100e6;
	ant_ThreeHalfWaveDipole.m_freqMax = 100e9;
	ant_ThreeHalfWaveDipole.m_polarization = Vector3D(0, 0, 1, true);
	innerAntennas.push_back(ant_ThreeHalfWaveDipole);

	Antenna ant_SingleWoundSpiral;														//������������
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
