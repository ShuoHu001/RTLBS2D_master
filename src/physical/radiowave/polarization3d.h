#ifndef RTLBS3D_POLARIZATION3D
#define RTLBS3D_POLARIZATION3D
#include "rtlbs.h"
#include "utility/define.h"
#include "math/complex.h"
#include "geometry/vector3d.h"
#include "geometry/point3d.h"
#include "material/material.h"
#include "geometry/wedge2d.h"
#include "polarization2d.h"
#include "geometry/terrain/terrainridge.h"
#include "global/globalvariables.h"

//��ά�����糡��x��y��z��������ĵ糡����ʸ����
class Polarization3D {
public:
	Complex ex;
	Complex ey;
	Complex ez;
public:
	Polarization3D();
	Polarization3D(Complex x, Complex y, Complex z);
	Polarization3D(const Complex& ampli, RtLbsType phi, RtLbsType theta);
	Polarization3D(Polarization3D& p);
	~Polarization3D();
	Polarization3D& operator = (const Polarization3D& p);			//��ֵ�����
	Polarization3D operator + (const Polarization3D& p) const;		//������ά����ʸ�����
	Polarization3D& operator += (const Polarization3D& p);			//������ά����ʸ�����
	Polarization3D operator - (const Polarization3D& p) const;		//������ά����ʸ�����
	Polarization3D& operator -= (const Polarization3D& p);			//������ά����ʸ�����
	Polarization3D operator * (const RtLbsType num) const;			//������ά����ʸ�����Գ���
	Polarization3D& operator *= (const RtLbsType num);				//������ά����ʸ�����Գ���
	Polarization3D operator * (const Complex& c) const;				//��ά����ʸ�����Ը���
	Polarization3D& operator *= (const Complex& c);					//��ά����ʸ�����Ը���
	Complex operator * (const Vector3D& v) const;					//������ά�����糡��v�����ϵķ���
	Polarization3D operator / (const RtLbsType num) const;			//������ά����ʸ�����Գ���
	Polarization3D& operator /= (const RtLbsType num);				//������ά����ʸ�����Գ���

	Vector3D GetRealComponent() const;								//��ȡʵ��ʸ��
	Vector3D GetImagComponent() const;								//��ȡ�鲿ʸ��

	void CalculateDirectionField(RtLbsType& st, const Point3D& sP, const Point3D& eP, RtLbsType freq);												//�����Ӿ��ų�-�����ż��㷽��
	void CalculateReflectionField_ReverseRT(RtLbsType& st, const Point3D& sP, const Point3D& rP, const Point3D& eP, Material* mat, RtLbsType freq);				//���㷴���ų�-�����ż��㷽��
	void CalculateReflectionField_ForwardRT(RtLbsType& st, const Point3D& sP, const Point3D rP, const Point3D& eP, Material* mat, RtLbsType freq);								//���㷴���ų�-�����ż��㷽��
	void CalculateTransmissionField_ReverseRT(RtLbsType& st, const Point3D& sP, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq);			//����͸���ų�-�����ż��㷽��
	void CalculateTransmissionField_ForwardRT(RtLbsType& st, const Point3D& sP, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq);							//����͸���ų�-�����ż��㷽��
	void CalculateStraightTransmissionField_ReverseRT(RtLbsType& st, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq);						//���㾭��͸���ų�-�����ż��㷽��
	void CalculateDiffractionField_ReverseRT(RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const Wedge2D* wedge,
		const Material* mat, RtLbsType freq);																			//���������ų�-�����ż��㷽��
	void CalculateDiffractionField_ForwardRT(RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const Wedge2D* wedge,
		const Material* mat, RtLbsType freq);																			//���������ų�-�����ż��㷽��
	void CalculateDiffractionField_TerrainUTD(RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const TerrainRidge* ridge,
		const Material* mat, RtLbsType freq);																			//���������ų�-��������UTD����
	void CalculateLOSFieldByDistance(RtLbsType s, RtLbsType freq);			//�����Ӿ��ų�-���վ�����м��㣨����Ϊ��ֵ����ģ���ֵ�����棩
	void CalculateLOSFieldByLoss(RtLbsType loss, RtLbsType freq);			//�����Ӿ��ų�-�������ֵ���м��㣨lossΪ��ֵ����ģ���ֵ�����棩
	bool IsZero() const;													//�Ƿ�����ֵ(���ҽ�������������Ϊ0ֵ�ǳ���)


private:
	Polarization2D _calculateReflectionCoef(RtLbsType freq, RtLbsType theta, Material* mat);			//���㷴��ϵ��
	Polarization2D _calculateTransmissionCoef(RtLbsType freq, RtLbsType theta, Material* mat);			//����͸��ϵ��
	Polarization2D _calculateSallabiDiffractionCoef(RtLbsType n, RtLbsType a1, RtLbsType a2, RtLbsType l, const Material* mat, RtLbsType freq);	//	��������ϵ��
	Polarization2D _specularReflectionCoef(RtLbsType theta, const Material* mat, RtLbsType freq);				//����ϵ��
	RtLbsType _redefineAngle(RtLbsType n, RtLbsType phi1, RtLbsType phi2);								//���¶���N��ķ����
	Complex _newTransactionFunction(double x);						//�µĹ��ɺ����Ķ���
	Polarization2D _rectifiedReflectionCoef(RtLbsType a1, RtLbsType a2, const Material* mat, RtLbsType freq);	//�����ķ���ϵ��(�����õ�)
};

inline Complex GetProjectField(const Polarization3D& p, const Vector3D& v) {
	Complex reVal;
	reVal.m_real = p.GetRealComponent() * v;
	reVal.m_imag = p.GetImagComponent() * v;
	return reVal;
}

#endif
