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

//三维极化电场（x、y、z三个方向的电场极化矢量）
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
	Polarization3D& operator = (const Polarization3D& p);			//赋值运算符
	Polarization3D operator + (const Polarization3D& p) const;		//计算三维极化矢量相加
	Polarization3D& operator += (const Polarization3D& p);			//计算三维极化矢量相加
	Polarization3D operator - (const Polarization3D& p) const;		//计算三维极化矢量相减
	Polarization3D& operator -= (const Polarization3D& p);			//计算三维极化矢量相减
	Polarization3D operator * (const RtLbsType num) const;			//计算三维极化矢量乘以常数
	Polarization3D& operator *= (const RtLbsType num);				//计算三维极化矢量乘以常数
	Polarization3D operator * (const Complex& c) const;				//三维极化矢量乘以复数
	Polarization3D& operator *= (const Complex& c);					//三维极化矢量乘以复数
	Complex operator * (const Vector3D& v) const;					//计算三维极化电场在v方向上的分量
	Polarization3D operator / (const RtLbsType num) const;			//计算三维极化矢量除以常数
	Polarization3D& operator /= (const RtLbsType num);				//计算三维极化矢量除以常数

	Vector3D GetRealComponent() const;								//获取实部矢量
	Vector3D GetImagComponent() const;								//获取虚部矢量

	void CalculateDirectionField(RtLbsType& st, const Point3D& sP, const Point3D& eP, RtLbsType freq);												//计算视距电磁场-反向电磁计算方法
	void CalculateReflectionField_ReverseRT(RtLbsType& st, const Point3D& sP, const Point3D& rP, const Point3D& eP, Material* mat, RtLbsType freq);				//计算反射电磁场-反向电磁计算方法
	void CalculateReflectionField_ForwardRT(RtLbsType& st, const Point3D& sP, const Point3D rP, const Point3D& eP, Material* mat, RtLbsType freq);								//计算反射电磁场-正向电磁计算方法
	void CalculateTransmissionField_ReverseRT(RtLbsType& st, const Point3D& sP, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq);			//计算透射电磁场-反向电磁计算方法
	void CalculateTransmissionField_ForwardRT(RtLbsType& st, const Point3D& sP, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq);							//计算透射电磁场-正向电磁计算方法
	void CalculateStraightTransmissionField_ReverseRT(RtLbsType& st, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq);						//计算经验透射电磁场-反向电磁计算方法
	void CalculateDiffractionField_ReverseRT(RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const Wedge2D* wedge,
		const Material* mat, RtLbsType freq);																			//计算绕射电磁场-反向电磁计算方法
	void CalculateDiffractionField_ForwardRT(RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const Wedge2D* wedge,
		const Material* mat, RtLbsType freq);																			//计算绕射电磁场-正向电磁计算方法
	void CalculateDiffractionField_TerrainUTD(RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const TerrainRidge* ridge,
		const Material* mat, RtLbsType freq);																			//计算绕射电磁场-地形绕射UTD方法
	void CalculateLOSFieldByDistance(RtLbsType s, RtLbsType freq);			//计算视距电磁场-按照距离进行计算（距离为正值表损耗，负值表增益）
	void CalculateLOSFieldByLoss(RtLbsType loss, RtLbsType freq);			//计算视距电磁场-按照损耗值进行计算（loss为正值表损耗，负值表增益）
	bool IsZero() const;													//是否是零值(当且仅当三个分量均为0值是成立)


private:
	Polarization2D _calculateReflectionCoef(RtLbsType freq, RtLbsType theta, Material* mat);			//计算反射系数
	Polarization2D _calculateTransmissionCoef(RtLbsType freq, RtLbsType theta, Material* mat);			//计算透射系数
	Polarization2D _calculateSallabiDiffractionCoef(RtLbsType n, RtLbsType a1, RtLbsType a2, RtLbsType l, const Material* mat, RtLbsType freq);	//	计算绕射系数
	Polarization2D _specularReflectionCoef(RtLbsType theta, const Material* mat, RtLbsType freq);				//反射系数
	RtLbsType _redefineAngle(RtLbsType n, RtLbsType phi1, RtLbsType phi2);								//重新定义N面的反射角
	Complex _newTransactionFunction(double x);						//新的过渡函数的定义
	Polarization2D _rectifiedReflectionCoef(RtLbsType a1, RtLbsType a2, const Material* mat, RtLbsType freq);	//修正的反射系数(绕射用到)
};

inline Complex GetProjectField(const Polarization3D& p, const Vector3D& v) {
	Complex reVal;
	reVal.m_real = p.GetRealComponent() * v;
	reVal.m_imag = p.GetImagComponent() * v;
	return reVal;
}

#endif
