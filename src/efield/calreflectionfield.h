#ifndef RTLBS_REFLECTIONRADIATIONFIELD
#define RTLBS_REFLECTIONRADIATIONFIELD

#include "math/polarization3d.h"


void CalculateReflectionField_ReverseRT(Polarization3D& inField, RtLbsType& st, const Point3D& sP, const Point3D& rP, const Point3D& eP, Material* mat, RtLbsType freq);				//���㷴���ų�-�����ż��㷽��
void CalculateReflectionField_ForwardRT(Polarization3D& inField, RtLbsType& st, const Point3D& sP, const Point3D rP, const Point3D& eP, Material* mat, RtLbsType freq);								//���㷴���ų�-�����ż��㷽��

Polarization2D _calculateReflectionCoef(RtLbsType freq, RtLbsType theta, Material* mat);															//���㷴��ϵ��






#endif