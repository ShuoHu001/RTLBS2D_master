#include "calreflectionfield.h"

void CalculateReflectionField_ReverseRT(Polarization3D& inField, RtLbsType& st, const Point3D& sP, const Point3D& rP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	//首先进行特殊处理，若计算场点位置与反射点的位置较为接近，则不做任何场的叠加计算
	Vector3D sr = rP - sP;
	Vector3D re = eP - rP;
	Vector3D srNormal = Normalize(sr);
	Vector3D reNormal = Normalize(re);
	RtLbsType srLen = sr.Length();                                           /** @brief	场计算起点与反射点坐标间的距离	*/
	RtLbsType reLen = re.Length();                                           /** @brief	场计算反射点与终点坐标间的距离	*/
	if (srLen < EPSILON || reLen < EPSILON) {                                //若累加距离小于系统默认最小值，则认为不进行任何形式的场计算叠加
		return;
	}
	//求解反射系数部分
	//求解入射射线与反射点所在平面的夹角theta
	RtLbsType cosTheta = srNormal * reNormal;
	RtLbsType theta = 0.0;
	if (cosTheta < -1)
		theta = HALF_PI;
	else if (cosTheta == 0)
		theta = 0;
	else
		theta = acos(cosTheta) / 2.0;


	//计算反射后的末场值
	Complex as;									/** @brief	场计算中间变量	*/
	as.m_real = st / (st + reLen);              /** @brief	球面波扩散因子	*/
	st += reLen;

	Vector3D v3 = srNormal.Cross(reNormal);
	Vector3D v4 = srNormal.Cross(v3);
	Vector3D v5 = reNormal.Cross(v3);

	//计算反射系数
	Polarization2D rcoef = _calculateReflectionCoef(freq, theta, mat);

	//将入射电场矢量转换为射线基坐标系下的表示
	Polarization2D inEfield;							 /** @brief	原始入射电场的射线基表示	*/
	inEfield.perp = inField * v3;                         /** @brief	原始电场沿射线基坐标系垂直分量	*/
	inEfield.para = inField * v4;                         /** @brief	原始电场沿射线基坐标值平行分量	*/

	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;
	//反射末场的射线基表示
	Polarization2D rEField;                             /** @brief	反射末场	*/
	rEField.perp = (inEfield.perp * rcoef.perp) * (as * Exp(-waveNumber * reLen));
	rEField.para = (inEfield.para * rcoef.para) * (as * Exp(-waveNumber * reLen));

	//将射线基坐标系反射末场转为笛卡尔坐标系表示
	inField = rEField.perp * v3 + rEField.para * v5;
}

void CalculateReflectionField_ForwardRT(Polarization3D& inField, RtLbsType& st, const Point3D& sP, const Point3D rP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	//1-计算几何常量
	Vector3D sr = rP - sP;
	Vector3D re = eP - rP;
	Vector3D srNormal = Normalize(sr);
	Vector3D reNormal = Normalize(re);
	RtLbsType srLen = sr.Length();                                           /** @brief	场计算起点与反射点坐标间的距离	*/
	RtLbsType reLen = re.Length();                                           /** @brief	场计算反射点与终点坐标间的距离	*/
	st += reLen;															 //传播距离叠加
	Vector3D v3 = srNormal.Cross(reNormal);
	Vector3D v4 = srNormal.Cross(v3);
	Vector3D v5 = reNormal.Cross(v3);
	//2-求解反射系数
	RtLbsType cosTheta = srNormal * reNormal;
	RtLbsType theta = 0.0;
	if (cosTheta < -1)
		theta = HALF_PI;
	else if (cosTheta == 0)
		theta = 0;
	else
		theta = acos(cosTheta) / 2.0;
	Polarization2D rcoef = _calculateReflectionCoef(freq, theta, mat);		/** @brief	反射系数	*/
	//3-求解反射场
	Polarization2D inEfield;												/** @brief	原始入射电场的射线基表示	*/
	inEfield.perp = inField * v3;											/** @brief	原始电场沿射线基坐标系垂直分量	*/
	inEfield.para = inField * v4;											/** @brief	原始电场沿射线基坐标值平行分量	*/
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;		/** @brief	波数	*/
	Polarization2D rEField;                             /** @brief	反射末场-射线基形式表达	*/
	rEField.perp = (inEfield.perp * rcoef.perp) * Exp(-waveNumber * reLen);
	rEField.para = (inEfield.para * rcoef.para) * Exp(-waveNumber * reLen);

	//将射线基坐标系反射末场转为笛卡尔坐标系表示
	inField = rEField.perp * v3 + rEField.para * v5;
}



Polarization2D _calculateReflectionCoef(RtLbsType freq, RtLbsType theta, Material* mat)
{
	Complex matEPara = mat->GetComplexForm(freq);               /** @brief	计算材质的复相对介电常数	*/

	//计算过程中所依赖的中间变量的表达式
	Complex snum = Complex(sin(theta), 0);
	Complex cnum = Complex(cos(theta) * cos(theta), 0);
	Complex ee1 = (matEPara - cnum).Sqrt();

	//开始计算反射系数
	Polarization2D rCoef;												/** @brief	反射系数	*/
	rCoef.para = (matEPara * snum - ee1) / (matEPara * snum + ee1);     /** @brief	平行极化波	*/
	rCoef.perp = (snum - ee1) / (snum + ee1);                           /** @brief	垂直极化波	*/
	return rCoef;
}

