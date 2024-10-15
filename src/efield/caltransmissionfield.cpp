#include "caltransmissionfield.h"

void CalculateTransmissionField_ReverseRT(Polarization3D& inField, RtLbsType& st, const Point3D& sP, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	//首先进行特殊处理，若计算场点位置与反射点的位置较为接近，则不做任何场的叠加计算
	Vector3D sr = tP - sP;
	Vector3D re = eP - tP;
	Vector3D srNormal = Normalize(sr);
	Vector3D reNormal = Normalize(re);
	RtLbsType srLen = sr.Length();                                           /** @brief	场计算起点与反射点坐标间的距离	*/
	RtLbsType reLen = re.Length();                                           /** @brief	场计算反射点与终点坐标间的距离	*/
	if (srLen < EPSILON || reLen < EPSILON) {                                //若累加距离小于系统默认最小值，则认为不进行任何形式的场计算叠加
		return;
	}
	//求解透射系数部分
	//求解入射射线与透射点所在平面的夹角theta
	RtLbsType cosTheta = srNormal * reNormal;
	RtLbsType theta = 0.0;
	if (cosTheta < -1)
		theta = HALF_PI;
	else if (cosTheta == 0)
		theta = 0;
	else
		theta = acos(cosTheta) / 2.0;


	//计算透射后的末场值
	Complex as;									/** @brief	场计算中间变量	*/
	as.m_real = st / (st + reLen);              /** @brief	球面波扩散因子	*/
	st += reLen;

	Vector3D v3 = srNormal.Cross(reNormal);
	Vector3D v4 = srNormal.Cross(v3);
	Vector3D v5 = reNormal.Cross(v3);

	//计算透射系数
	Polarization2D tcoef = _calculateTransmissionCoef(freq, theta, mat);

	//将入射电场矢量转换为射线基坐标系下的表示
	Polarization2D inEfield;                            /** @brief	原始入射电场的射线基表示	*/
	inEfield.perp = inField * v3;                         /** @brief	原始电场沿射线基坐标系垂直分量	*/
	inEfield.para = inField * v4;                         /** @brief	原始电场沿射线基坐标值平行分量	*/

	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;
	//透射末场的射线基表示
	Polarization2D rEField;                             /** @brief	反射末场	*/
	rEField.perp = (inEfield.perp * tcoef.perp) * (as * Exp(-waveNumber * reLen));
	rEField.para = (inEfield.para * tcoef.para) * (as * Exp(-waveNumber * reLen));

	//将射线基坐标系透射末场转为笛卡尔坐标系表示
	inField = rEField.perp * v3 + rEField.para * v5;
}

void CalculateTransmissionField_ForwardRT(Polarization3D& inField, RtLbsType& st, const Point3D& sP, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	//1-计算几何常量
	Vector3D sr = tP - sP;
	Vector3D re = eP - tP;
	Vector3D srNormal = Normalize(sr);
	Vector3D reNormal = Normalize(re);
	RtLbsType srLen = sr.Length();                                           /** @brief	场计算起点与反射点坐标间的距离	*/
	RtLbsType reLen = re.Length();                                           /** @brief	场计算反射点与终点坐标间的距离	*/
	st += reLen;															 //传播距离叠加
	Vector3D v3 = srNormal.Cross(reNormal);
	Vector3D v4 = srNormal.Cross(v3);
	Vector3D v5 = reNormal.Cross(v3);
	//2-求解透射系数部分
	RtLbsType cosTheta = srNormal * reNormal;
	RtLbsType theta = 0.0;
	if (cosTheta < -1)
		theta = HALF_PI;
	else if (cosTheta == 0)
		theta = 0;
	else
		theta = acos(cosTheta) / 2.0;
	Polarization2D tcoef = _calculateTransmissionCoef(freq, theta, mat);
	//3-求解透射场
	Polarization2D inEfield;										/** @brief	原始入射电场的射线基表示	*/
	inEfield.perp = inField * v3;									/** @brief	原始电场沿射线基坐标系垂直分量	*/
	inEfield.para = inField * v4;									/** @brief	原始电场沿射线基坐标值平行分量	*/
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;		/** @brief	波数	*/
	Polarization2D tEField;											/** @brief	透射末场	*/
	tEField.perp = (inEfield.perp * tcoef.perp) * Exp(-waveNumber * reLen);		/** @brief	垂直分量的波	*/
	tEField.para = (inEfield.para * tcoef.para) * Exp(-waveNumber * reLen);		/** @brief	平行分量的波	*/
	//将射线基坐标系透射末场转为笛卡尔坐标系表示
	inField = tEField.perp * v3 + tEField.para * v5;
}

void CalculateStraightTransmissionField_ReverseRT(Polarization3D& inField, RtLbsType& st, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	//计算实际穿透距离
	Vector3D te = eP - tP;
	RtLbsType teLen = te.Length();
	RtLbsType loss_dB = teLen * mat->m_penetrationLoss;
	//将dB转换为线性单位，并乘在值上(实部虚部都乘)
	RtLbsType loss_line = pow(10.0, loss_dB / 10.0);
	inField *= loss_line;
}

Polarization2D _calculateTransmissionCoef(RtLbsType freq, RtLbsType theta, Material* mat)
{
	Complex matEPara = mat->GetComplexForm(freq);               /** @brief	计算材质的复相对介电常数	*/

	//计算过程中所依赖的中间变量的表达式
	Complex snum = Complex(sin(theta), 0);
	Complex cnum = Complex(cos(theta) * cos(theta), 0);
	Complex ee1 = (matEPara - cnum).Sqrt();

	//开始计算透射系数
	Polarization2D tCoef;												/** @brief	透射系数	*/
	tCoef.para = matEPara * snum * 2.0 / (matEPara * snum + ee1);		/** @brief	平行极化波	*/
	tCoef.perp = snum * 2.0 / (snum + ee1);								/** @brief	垂直极化波	*/
	return tCoef;
}