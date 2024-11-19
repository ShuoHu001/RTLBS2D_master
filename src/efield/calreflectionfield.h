#ifndef RTLBS_REFLECTIONRADIATIONFIELD
#define RTLBS_REFLECTIONRADIATIONFIELD

#include "math/polarization3d.h"


void CalculateReflectionField_ReverseRT(Polarization3D& inField, RtLbsType& st, const Point3D& sP, const Point3D& rP, const Point3D& eP, Material* mat, RtLbsType freq);				//计算反射电磁场-反向电磁计算方法
void CalculateReflectionField_ForwardRT(Polarization3D& inField, RtLbsType& st, const Point3D& sP, const Point3D rP, const Point3D& eP, Material* mat, RtLbsType freq);								//计算反射电磁场-正向电磁计算方法

Polarization2D _calculateReflectionCoef(RtLbsType freq, RtLbsType theta, Material* mat);															//计算反射系数






#endif