#ifndef RTLBS_CALTERRAINRAYPATHFIELD
#define RTLBS_CALTERRAINRAYPATHFIELD

#include "rtlbs.h"
#include "utility/define.h"
#include "radiowave/raypath/terraindiffractionpath.h"
#include "caldiffractionfield.h"
#include "calradiationfield.h"
#include "calreceivedfield.h"

Complex CalculateDiffractionEField_PICQUENARD(const TerrainDiffractionPath* path, RtLbsType power, RtLbsType freq, const Antenna* txAntenna, const Antenna* rxAntenna);				//计算地形绕射损耗 Picquenard 方法
Complex CalculateDiffractionEField_EPSTEIN(const TerrainDiffractionPath* path, RtLbsType power, RtLbsType freq, const Antenna* txAntenna, const Antenna* rxAntenna);				//计算地形绕射损耗 EPSTEIN方法
Complex CalculateDiffractionEField_UTD(const TerrainDiffractionPath* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna, const Antenna* rxAntenna);					//计算地形绕射损耗 UTD方法
Complex CalculateTerrainDiffractionEField(const TerrainDiffractionPath* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna, const Antenna* rxAntenna);					//计算绕射损耗

#endif
