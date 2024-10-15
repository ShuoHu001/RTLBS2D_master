#ifndef RTLBS_CALRADIATIONFIELD
#define RTLBS_CALRADIATIONFIELD


#include "rtlbs.h"
#include "utility/define.h"
#include "equipment/antenna/antenna.h"
#include "math/polarization3d.h"

void CalRadiationField_ReverseRT(const Antenna* antenna, RtLbsType power, RtLbsType freq, RtLbsType phi, RtLbsType theta, RtLbsType distance, Polarization3D& efield);		//计算天线周围的辐射场
void CalRadiationField_ForwardRT(const Antenna* antenna, RtLbsType power, RtLbsType freq, RtLbsType phi, RtLbsType theta, Polarization3D& efield);							//计算天线周围的辐射场-正向电磁计算（与距离无关）


#endif
