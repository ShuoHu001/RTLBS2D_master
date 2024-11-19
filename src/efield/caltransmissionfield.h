#ifndef RTLBS_CALTRANSMISSIONFIELD
#define RTLBS_CALTRANSMISSIONFIELD

#include "math/polarization3d.h"

void CalculateTransmissionField_ReverseRT(Polarization3D& inField, RtLbsType& st, const Point3D& sP, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq);			//����͸���ų�-�����ż��㷽��
void CalculateTransmissionField_ForwardRT(Polarization3D& inField, RtLbsType& st, const Point3D& sP, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq);			//����͸���ų�-�����ż��㷽��
void CalculateStraightTransmissionField_ReverseRT(Polarization3D& inField, RtLbsType& st, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq);						//���㾭��͸���ų�-�����ż��㷽��


Polarization2D _calculateTransmissionCoef(RtLbsType freq, RtLbsType theta, Material* mat);																							//����͸��ϵ��
#endif
