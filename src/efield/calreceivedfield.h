#ifndef RTLBS_CALRECEIVEDFIELD
#define RTLBS_CALRECEIVEDFIELD

#include "rtlbs.h"
#include "utility/define.h"
#include "equipment/antenna/antenna.h"
#include "math/polarization3d.h"

void CalReceivedField(const Antenna* antenna, Polarization3D& eIn, RtLbsType freq, RtLbsType phi, RtLbsType theta, Complex& eOut);								//计算天线周围接收场

#endif
