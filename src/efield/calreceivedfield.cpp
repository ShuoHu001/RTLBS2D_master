#include "calreceivedfield.h"

void CalReceivedField(const Antenna* antenna, Polarization3D& eIn, RtLbsType freq, RtLbsType phi, RtLbsType theta, Complex& eOut)
{
	//基于射入天线前的空间场，结合天线方向图和极化方式，计算接收场
	RtLbsType eSize = antenna->GetAntennaNormalizedGain(freq, phi, theta);
	//目前只计算单一载波
	eOut = eIn * antenna->m_polarization * eSize;
}
