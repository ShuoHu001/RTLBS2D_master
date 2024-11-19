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



Polarization3D CalculateStrengthField3D(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna);								//计算路径三维复电场值
Polarization3D CalculateStrengthField3DReverse(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna);						//反向计算路径三维复电场值
Complex CalculateStrengthField(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna, const Antenna* rxAntenna);				//计算路径复电场值
Complex CalculateStrengthFieldReverse(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna, const Antenna* rxAntenna);		//反向计算路径复电场值
RtLbsType CalculatePowerInLBSSystem(const RayPath3D* path, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* trxAntenna);													//计算LBS定位系统中的功率值

#endif
