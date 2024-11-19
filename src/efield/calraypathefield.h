#ifndef RTLBS_CALRAYPATHEFIELD
#define RTLBS_CALRAYPATHEFIELD

#include "rtlbs.h"
#include "utility/define.h"
#include "radiowave/raypath/raypath3d.h"
#include "equipment/antenna/antenna.h"
#include "calreflectionfield.h"
#include "caltransmissionfield.h"
#include "caldiffractionfield.h"
#include "calscatteringfield.h"
#include "calradiationfield.h"
#include "calreceivedfield.h"



Polarization3D CalculateStrengthField3D(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna);								//����·����ά���糡ֵ
Polarization3D CalculateStrengthField3DReverse(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna);						//�������·����ά���糡ֵ
Complex CalculateStrengthField(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna, const Antenna* rxAntenna);				//����·�����糡ֵ
Complex CalculateStrengthFieldReverse(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna, const Antenna* rxAntenna);		//�������·�����糡ֵ
RtLbsType CalculatePowerInLBSSystem(const RayPath3D* path, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* trxAntenna);													//����LBS��λϵͳ�еĹ���ֵ

#endif
