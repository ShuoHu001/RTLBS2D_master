#include "calradiationfield.h"

void CalRadiationField_ReverseRT(const Antenna* antenna, RtLbsType power, RtLbsType freq, RtLbsType phi, RtLbsType theta, RtLbsType distance, Polarization3D& efield)
{
	//计算天线周围的辐射场
	RtLbsType eSize = antenna->GetAntennaNormalizedGain(freq, phi, theta);							//计算天线的归一化增益值
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;										//空气中的波数
	Complex constantTerm;																			/** @brief	计算场值的实数项	*/
	constantTerm.m_real = 60 * std::sqrt(power / 36.55) / distance * eSize;							/** @brief	计算电场实部	*/
	Complex phaseVar;																				/** @brief	计算场值的相位项	*/
	phaseVar.m_real = std::cos(waveNumber * distance);
	phaseVar.m_imag = std::sin(waveNumber * distance);
	Complex amplitude = constantTerm * phaseVar;													/** @brief	复数幅值	*/
	//将幅值分配到三维空间中的场值
	efield = Polarization3D(amplitude, phi, theta);
}

void CalRadiationField_ForwardRT(const Antenna* antenna, RtLbsType power, RtLbsType freq, RtLbsType phi, RtLbsType theta, Polarization3D& efield)
{
	//按照常规1m损耗的电场
	RtLbsType eSize = antenna->GetAntennaNormalizedGain(freq, phi, theta);							//计算天线的归一化增益值
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;										//空气中的波数
	Complex constantTerm;																			/** @brief	计算场值的实数项	*/
	constantTerm.m_real = 60 * std::sqrt(power / 36.55) / 100 * eSize;								/** @brief	计算电场实部,为了能够多频率满足远场条件，这里以100m距离为计算	*/
	Complex phaseVar;																				/** @brief	计算场值的相位项	*/
	phaseVar.m_real = std::cos(waveNumber);
	phaseVar.m_imag = std::sin(waveNumber);
	Complex amplitude = constantTerm * phaseVar;													/** @brief	复数幅值	*/
	efield = Polarization3D(amplitude, phi, theta);													//三维复电场
	//电磁场逆计算得到初始的能量场
	efield.CalculateLOSFieldByDistance(-100, freq);													//逆向计算100m处的电场
}
