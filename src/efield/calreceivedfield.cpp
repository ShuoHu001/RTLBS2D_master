#include "calreceivedfield.h"

void CalReceivedField(const Antenna* antenna, Polarization3D& eIn, RtLbsType freq, RtLbsType phi, RtLbsType theta, Complex& eOut)
{
	//������������ǰ�Ŀռ䳡��������߷���ͼ�ͼ�����ʽ��������ճ�
	RtLbsType eSize = antenna->GetAntennaNormalizedGain(freq, phi, theta);
	//Ŀǰֻ���㵥һ�ز�
	eOut = eIn * antenna->m_polarization * eSize;
}
