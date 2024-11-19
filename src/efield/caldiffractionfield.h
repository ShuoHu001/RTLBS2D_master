#ifndef RTLBS_CALDIFFRACTIONFIELD
#define RTLBS_CALDIFFRACTIONFIELD

#include "math/polarization3d.h"

void CalculateDiffractionField_ReverseRT(Polarization3D& inField, RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const Wedge2D* wedge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunctionData);			//���������ų�-�����ż��㷽��
void CalculateDiffractionField_ForwardRT(Polarization3D& inField, RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const Wedge2D* wedge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunctionData);			//���������ų�-�����ż��㷽��
void CalculateDiffractionField_TerrainUTD(Polarization3D& inField, RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const TerrainRidge* ridge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunctionData);		//���������ų�-��������UTD����


Polarization2D _calculateSallabiDiffractionCoef(RtLbsType n, RtLbsType a1, RtLbsType a2, RtLbsType l, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunctionData);			//��������ϵ��
Polarization2D _specularReflectionCoef(RtLbsType theta, const Material* mat, RtLbsType freq);														//����ϵ��
RtLbsType _redefineAngle(RtLbsType n, RtLbsType phi1, RtLbsType phi2);																				//���¶���N��ķ����
Complex _newTransactionFunction(double x, const std::vector<Complex>& tranFunctionData);																											//�µĹ��ɺ����Ķ���
Polarization2D _rectifiedReflectionCoef(RtLbsType a1, RtLbsType a2, const Material* mat, RtLbsType freq);											//�����ķ���ϵ��(�����õ�)
#endif
