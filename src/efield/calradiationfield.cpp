#include "calradiationfield.h"

void CalRadiationField_ReverseRT(const Antenna* antenna, RtLbsType power, RtLbsType freq, RtLbsType phi, RtLbsType theta, RtLbsType distance, Polarization3D& efield)
{
	//����������Χ�ķ��䳡
	RtLbsType eSize = antenna->GetAntennaNormalizedGain(freq, phi, theta);							//�������ߵĹ�һ������ֵ
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;										//�����еĲ���
	Complex constantTerm;																			/** @brief	���㳡ֵ��ʵ����	*/
	constantTerm.m_real = 60 * std::sqrt(power / 36.55) / distance * eSize;							/** @brief	����糡ʵ��	*/
	Complex phaseVar;																				/** @brief	���㳡ֵ����λ��	*/
	phaseVar.m_real = std::cos(waveNumber * distance);
	phaseVar.m_imag = std::sin(waveNumber * distance);
	Complex amplitude = constantTerm * phaseVar;													/** @brief	������ֵ	*/
	//����ֵ���䵽��ά�ռ��еĳ�ֵ
	efield = Polarization3D(amplitude, phi, theta);
}

void CalRadiationField_ForwardRT(const Antenna* antenna, RtLbsType power, RtLbsType freq, RtLbsType phi, RtLbsType theta, Polarization3D& efield)
{
	//���ճ���1m��ĵĵ糡
	RtLbsType eSize = antenna->GetAntennaNormalizedGain(freq, phi, theta);							//�������ߵĹ�һ������ֵ
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;										//�����еĲ���
	Complex constantTerm;																			/** @brief	���㳡ֵ��ʵ����	*/
	constantTerm.m_real = 60 * std::sqrt(power / 36.55) / 100 * eSize;								/** @brief	����糡ʵ��,Ϊ���ܹ���Ƶ������Զ��������������100m����Ϊ����	*/
	Complex phaseVar;																				/** @brief	���㳡ֵ����λ��	*/
	phaseVar.m_real = std::cos(waveNumber);
	phaseVar.m_imag = std::sin(waveNumber);
	Complex amplitude = constantTerm * phaseVar;													/** @brief	������ֵ	*/
	efield = Polarization3D(amplitude, phi, theta);													//��ά���糡
	//��ų������õ���ʼ��������
	efield.CalculateLOSFieldByDistance(-100, freq);													//�������100m���ĵ糡
}
