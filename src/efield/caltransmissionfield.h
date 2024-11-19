#ifndef RTLBS_CALTRANSMISSIONFIELD
#define RTLBS_CALTRANSMISSIONFIELD

#include "math/polarization3d.h"

void CalculateTransmissionField_ReverseRT(Polarization3D& inField, RtLbsType& st, const Point3D& sP, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq);			//计算透射电磁场-反向电磁计算方法
void CalculateTransmissionField_ForwardRT(Polarization3D& inField, RtLbsType& st, const Point3D& sP, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq);			//计算透射电磁场-正向电磁计算方法
void CalculateStraightTransmissionField_ReverseRT(Polarization3D& inField, RtLbsType& st, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq);						//计算经验透射电磁场-反向电磁计算方法


Polarization2D _calculateTransmissionCoef(RtLbsType freq, RtLbsType theta, Material* mat);																							//计算透射系数
#endif
