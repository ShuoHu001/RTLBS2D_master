#ifndef RTLBS_CALDIFFRACTIONFIELD
#define RTLBS_CALDIFFRACTIONFIELD

#include "math/polarization3d.h"

void CalculateDiffractionField_ReverseRT(Polarization3D& inField, RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const Wedge2D* wedge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunctionData);			//计算绕射电磁场-反向电磁计算方法
void CalculateDiffractionField_ForwardRT(Polarization3D& inField, RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const Wedge2D* wedge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunctionData);			//计算绕射电磁场-正向电磁计算方法
void CalculateDiffractionField_TerrainUTD(Polarization3D& inField, RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const TerrainRidge* ridge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunctionData);		//计算绕射电磁场-地形绕射UTD方法


Polarization2D _calculateSallabiDiffractionCoef(RtLbsType n, RtLbsType a1, RtLbsType a2, RtLbsType l, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunctionData);			//计算绕射系数
Polarization2D _specularReflectionCoef(RtLbsType theta, const Material* mat, RtLbsType freq);														//反射系数
RtLbsType _redefineAngle(RtLbsType n, RtLbsType phi1, RtLbsType phi2);																				//重新定义N面的反射角
Complex _newTransactionFunction(double x, const std::vector<Complex>& tranFunctionData);																											//新的过渡函数的定义
Polarization2D _rectifiedReflectionCoef(RtLbsType a1, RtLbsType a2, const Material* mat, RtLbsType freq);											//修正的反射系数(绕射用到)
#endif
