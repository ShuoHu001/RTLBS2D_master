#include "polarization3d.h"

Polarization3D::Polarization3D()
{
}

Polarization3D::Polarization3D(Complex x, Complex y, Complex z)
	: ex(x)
	, ey(y)
	, ez(z)
{
}

Polarization3D::Polarization3D(const Complex& ampli, RtLbsType phi, RtLbsType theta)
{
	//将复数赋值分配至三维空间中
	RtLbsType sinTerm = sin(HALF_PI - theta);
	RtLbsType cosTerm = -cos(HALF_PI - theta);
	RtLbsType cosPhi = cos(phi);
	RtLbsType sinPhi = sin(phi);
	ex.m_real = ampli.m_real * sinTerm * cosPhi;
	ex.m_imag = ampli.m_imag * sinTerm * cosPhi;
	ey.m_real = ampli.m_real * sinTerm * sinPhi;
	ey.m_imag = ampli.m_imag * sinTerm * sinPhi;
	ez.m_real = ampli.m_real * cosTerm;
	ez.m_imag = ampli.m_imag * cosTerm;
}

Polarization3D::Polarization3D(Polarization3D& p)
	: ex(p.ex)
	, ey(p.ey)
	, ez(p.ez)
{
}

Polarization3D::~Polarization3D()
{
}

Polarization3D& Polarization3D::operator=(const Polarization3D& p)
{
	ex = p.ex;
	ey = p.ey;
	ez = p.ez;
	return *this;
}

Polarization3D Polarization3D::operator+(const Polarization3D& p) const
{
	Polarization3D reVal;
	reVal.ex = ex + p.ex;
	reVal.ey = ey + p.ey;
	reVal.ez = ez + p.ez;
	return reVal;
}

Polarization3D& Polarization3D::operator+=(const Polarization3D& p)
{
	ex += p.ex;
	ey += p.ey;
	ez += p.ez;
	return *this;
}

Polarization3D Polarization3D::operator-(const Polarization3D& p) const
{
	Polarization3D reVal;
	reVal.ex = ex - p.ex;
	reVal.ey = ey - p.ey;
	reVal.ez = ez - p.ez;
	return reVal;
}

Polarization3D& Polarization3D::operator-=(const Polarization3D& p)
{
	ex -= p.ex;
	ey -= p.ey;
	ez -= p.ez;
	return *this;
}

Polarization3D Polarization3D::operator*(const RtLbsType num) const
{
	Polarization3D reVal;
	reVal.ex = ex * num;
	reVal.ey = ey * num;
	reVal.ez = ez * num;
	return reVal;
}

Polarization3D& Polarization3D::operator*=(const RtLbsType num)
{
	ex *= num;
	ey *= num;
	ez *= num;
	return *this;
}

Polarization3D Polarization3D::operator*(const Complex& c) const
{
	Polarization3D reVal;
	reVal.ex *= c;
	reVal.ey *= c;
	reVal.ez *= c;
	return reVal;
}

Polarization3D& Polarization3D::operator*=(const Complex& c)
{
	ex *= c;
	ey *= c;
	ez *= c;
	return *this;
}

Complex Polarization3D::operator*(const Vector3D& v) const
{
	Complex reVal;
	reVal.m_real = GetRealComponent() * v;
	reVal.m_imag = GetImagComponent() * v;
	return reVal;
}

Polarization3D Polarization3D::operator/(const RtLbsType num) const
{
	Polarization3D reVal;
	reVal.ex = ex * num;
	reVal.ey = ey * num;
	reVal.ez = ez * num;
	return reVal;
}

Polarization3D& Polarization3D::operator/=(const RtLbsType num)
{
	ex /= num;
	ey /= num;
	ez /= num;
	return *this;
}

Vector3D Polarization3D::GetRealComponent() const
{
	return Vector3D(ex.m_real, ey.m_real, ez.m_real);
}

Vector3D Polarization3D::GetImagComponent() const
{
	return Vector3D(ex.m_imag, ey.m_imag, ez.m_imag);
}

void Polarization3D::CalculateDirectionField(RtLbsType& st, const Point3D& sP, const Point3D& eP, RtLbsType freq)
{
	RtLbsType s = (eP - sP).Length();
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;
	Complex constTerm;
	Complex phaseTerm;
	constTerm.m_real = 1.0 / s;
	phaseTerm.m_real = cos(waveNumber * s);
	phaseTerm.m_imag = sin(waveNumber * s);
	Complex amplitude = constTerm * phaseTerm;
	st += s;
	*this *= amplitude;
}

void Polarization3D::CalculateReflectionField_ReverseRT(RtLbsType& st, const Point3D& sP, const Point3D& rP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	Polarization3D& efield = *this;											/** @brief	入射电场	*/
	//首先进行特殊处理，若计算场点位置与反射点的位置较为接近，则不做任何场的叠加计算
	Vector3D sr = rP - sP;
	Vector3D re = eP - rP;
	RtLbsType srLen = sr.Length();                                           /** @brief	场计算起点与反射点坐标间的距离	*/
	RtLbsType reLen = re.Length();                                           /** @brief	场计算反射点与终点坐标间的距离	*/
	if (srLen < EPSILON || reLen < EPSILON) {                                //若累加距离小于系统默认最小值，则认为不进行任何形式的场计算叠加
		return;
	}
	//求解反射系数部分
	//求解入射射线与反射点所在平面的夹角theta
	RtLbsType cosTheta = sr.Normalize() * re.Normalize();
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

	Vector3D v3 = sr.Cross(re).Normalize();
	Vector3D v4 = sr.Cross(v3).Normalize();
	Vector3D v5 = re.Cross(v3).Normalize();

	//计算反射系数
	Polarization2D rcoef = _calculateReflectionCoef(freq, theta, mat);

	//将入射电场矢量转换为射线基坐标系下的表示
	Polarization2D inEfield;							 /** @brief	原始入射电场的射线基表示	*/
	inEfield.perp = efield * v3;                         /** @brief	原始电场沿射线基坐标系垂直分量	*/
	inEfield.para = efield * v4;                         /** @brief	原始电场沿射线基坐标值平行分量	*/

	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;
	//反射末场的射线基表示
	Polarization2D rEField;                             /** @brief	反射末场	*/
	rEField.perp = (inEfield.perp * rcoef.perp) * (as * Exp(-waveNumber * reLen));
	rEField.para = (inEfield.para * rcoef.para) * (as * Exp(-waveNumber * reLen));

	//将射线基坐标系反射末场转为笛卡尔坐标系表示
	efield = rEField.perp * v3 + rEField.para * v5;
}

void Polarization3D::CalculateReflectionField_ForwardRT(RtLbsType& st, const Point3D& sP, const Point3D rP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	Polarization3D& efield = *this;											/** @brief	入射电场	*/
	//1-计算几何常量
	Vector3D sr = rP - sP;
	Vector3D re = eP - rP;
	RtLbsType srLen = sr.Length();                                           /** @brief	场计算起点与反射点坐标间的距离	*/
	RtLbsType reLen = re.Length();                                           /** @brief	场计算反射点与终点坐标间的距离	*/
	st += reLen;															 //传播距离叠加
	Vector3D v3 = sr.Cross(re).Normalize();
	Vector3D v4 = sr.Cross(v3).Normalize();
	Vector3D v5 = re.Cross(v3).Normalize();
	//2-求解反射系数
	RtLbsType cosTheta = sr.Normalize() * re.Normalize();
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
	inEfield.perp = efield * v3;											/** @brief	原始电场沿射线基坐标系垂直分量	*/
	inEfield.para = efield * v4;											/** @brief	原始电场沿射线基坐标值平行分量	*/
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;		/** @brief	波数	*/
	Polarization2D rEField;                             /** @brief	反射末场-射线基形式表达	*/
	rEField.perp = (inEfield.perp * rcoef.perp) * Exp(-waveNumber * reLen);
	rEField.para = (inEfield.para * rcoef.para) * Exp(-waveNumber * reLen);

	//将射线基坐标系反射末场转为笛卡尔坐标系表示
	efield = rEField.perp * v3 + rEField.para * v5;
}

void Polarization3D::CalculateTransmissionField_ReverseRT(RtLbsType& st, const Point3D& sP, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	Polarization3D& efield = *this;											/** @brief	入射电场	*/
	//首先进行特殊处理，若计算场点位置与反射点的位置较为接近，则不做任何场的叠加计算
	Vector3D sr = tP - sP;
	Vector3D re = eP - tP;
	RtLbsType srLen = sr.Length();                                           /** @brief	场计算起点与反射点坐标间的距离	*/
	RtLbsType reLen = re.Length();                                           /** @brief	场计算反射点与终点坐标间的距离	*/
	if (srLen < EPSILON || reLen < EPSILON) {                                //若累加距离小于系统默认最小值，则认为不进行任何形式的场计算叠加
		return;
	}
	//求解透射系数部分
	//求解入射射线与透射点所在平面的夹角theta
	RtLbsType cosTheta = sr.Normalize() * re.Normalize();
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

	Vector3D v3 = sr.Cross(re).Normalize();
	Vector3D v4 = sr.Cross(v3).Normalize();
	Vector3D v5 = re.Cross(v3).Normalize();

	//计算透射系数
	Polarization2D tcoef = _calculateTransmissionCoef(freq, theta, mat);

	//将入射电场矢量转换为射线基坐标系下的表示
	Polarization2D inEfield;                            /** @brief	原始入射电场的射线基表示	*/
	inEfield.perp = efield * v3;                         /** @brief	原始电场沿射线基坐标系垂直分量	*/
	inEfield.para = efield * v4;                         /** @brief	原始电场沿射线基坐标值平行分量	*/

	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;
	//透射末场的射线基表示
	Polarization2D rEField;                             /** @brief	反射末场	*/
	rEField.perp = (inEfield.perp * tcoef.perp) * (as * Exp(-waveNumber * reLen));
	rEField.para = (inEfield.para * tcoef.para) * (as * Exp(-waveNumber * reLen));

	//将射线基坐标系透射末场转为笛卡尔坐标系表示
	efield = rEField.perp * v3 + rEField.para * v5;
}

void Polarization3D::CalculateTransmissionField_ForwardRT(RtLbsType& st, const Point3D& sP, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	Polarization3D& efield = *this;											/** @brief	入射电场	*/
	//1-计算几何常量
	Vector3D sr = tP - sP;
	Vector3D re = eP - tP;
	RtLbsType srLen = sr.Length();                                           /** @brief	场计算起点与反射点坐标间的距离	*/
	RtLbsType reLen = re.Length();                                           /** @brief	场计算反射点与终点坐标间的距离	*/
	st += reLen;															 //传播距离叠加
	Vector3D v3 = sr.Cross(re).Normalize();
	Vector3D v4 = sr.Cross(v3).Normalize();
	Vector3D v5 = re.Cross(v3).Normalize();
	//2-求解透射系数部分
	RtLbsType cosTheta = sr.Normalize() * re.Normalize();
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
	inEfield.perp = efield * v3;									/** @brief	原始电场沿射线基坐标系垂直分量	*/
	inEfield.para = efield * v4;									/** @brief	原始电场沿射线基坐标值平行分量	*/
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;		/** @brief	波数	*/
	Polarization2D tEField;											/** @brief	透射末场	*/
	tEField.perp = (inEfield.perp * tcoef.perp) * Exp(-waveNumber * reLen);		/** @brief	垂直分量的波	*/
	tEField.para = (inEfield.para * tcoef.para) * Exp(-waveNumber * reLen);		/** @brief	平行分量的波	*/
	//将射线基坐标系透射末场转为笛卡尔坐标系表示
	efield = tEField.perp * v3 + tEField.para * v5;
}

void Polarization3D::CalculateStraightTransmissionField_ReverseRT(RtLbsType& st, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	//计算实际穿透距离
	Vector3D te = eP - tP;
	RtLbsType teLen = te.Length();
	RtLbsType loss_dB = teLen * mat->m_penetrationLoss;
	//将dB转换为线性单位，并乘在值上(实部虚部都乘)
	RtLbsType loss_line = pow(10.0, loss_dB / 10.0);
	Polarization3D& efield = *this;
	efield *= loss_line;
}

void Polarization3D::CalculateDiffractionField_ReverseRT(RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const Wedge2D* wedge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFucntion)
{
	//首先进行特殊处理，若计算场点的位置与绕射点的位置较为接近，则不进行场的任何计算
	Vector3D sd = dP - sP;																										/** @brief	场计算起点指向绕射点坐标的向量	*/
	Vector3D de = eP - dP;																										/** @brief	绕射点指向终点坐标的向量	*/
	RtLbsType sdLen = sd.Length();																								/** @brief	起点到绕射点的距离	*/
	RtLbsType deLen = de.Length();																								/** @brief	绕射点至终点的距离	*/
	if (sdLen < EPSILON || deLen < EPSILON)																						//若累加距离小于系统的默认值，则不进行任何场计算
		return;
	//求解入射角和绕射角(确定0面和N面)
	RtLbsType phiIncident, phiDiffraction;																						/** @brief	入射角与绕射角	*/
	RtLbsType nValue = wedge->GetNValue();																						/** @brief	棱劈内角n值(2-n)π	*/
	wedge->CalDiffractionParameters(sd.Normalize(), de.Normalize(), dP, phiIncident, phiDiffraction);
	Complex as;
	RtLbsType l = st / (st + deLen);																							/** @brief	球面波扩散因子	*/
	as.m_real = l;
	st += deLen;

	Polarization3D& efield = *this;
	//求解绕射系数
	Polarization2D dCoef = _calculateSallabiDiffractionCoef(nValue, phiIncident, phiDiffraction, l, mat, freq, tranFucntion);
	//入射电场的射线基表示
	Vector3D v3 = sd.Cross(de).Normalize();
	Vector3D v4 = sd.Cross(v3).Normalize();
	Vector3D v5 = de.Cross(v3).Normalize();
	Polarization2D inEField;                                                            /** @brief	原始入射电场的射线基表示	*/
	inEField.perp = efield * v3;                                                        /** @brief	原始电场在射线基坐标系中的垂直分量	*/
	inEField.para = efield * v4;                                                        /** @brief	原始电场在射线基坐标系中的平行分量	*/

	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;

	//绕射末场的射线基表示
	Polarization2D dEField;                                                             /** @brief	绕射末场	*/
	dEField.perp = (inEField.perp * dCoef.perp) * (as * Exp(-waveNumber * deLen));
	dEField.para = (inEField.para * dCoef.para) * (as * Exp(-waveNumber * deLen));

	//将绕射场转为笛卡尔坐标系下的表示
	efield = dEField.perp * v3 + dEField.para * v5;
}

void Polarization3D::CalculateDiffractionField_ForwardRT(RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const Wedge2D* wedge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunction)
{
	Polarization3D& efield = *this;														/** @brief	入射电磁场赋值	*/
	//1-几何参量求解
	Vector3D sd = dP - sP;																										/** @brief	场计算起点指向绕射点坐标的向量	*/
	Vector3D de = eP - dP;																										/** @brief	绕射点指向终点坐标的向量	*/
	RtLbsType sdLen = sd.Length();																								/** @brief	起点到绕射点的距离	*/
	RtLbsType deLen = de.Length();																								/** @brief	绕射点至终点的距离	*/
	Vector3D v3 = sd.Cross(de).Normalize();
	Vector3D v4 = sd.Cross(v3).Normalize();
	Vector3D v5 = de.Cross(v3).Normalize();
	//2-求解绕射系数
	RtLbsType phiIncident, phiDiffraction;																						/** @brief	入射角与绕射角	*/
	RtLbsType nValue = wedge->GetNValue();																						/** @brief	棱劈内角n值(2-n)π	*/
	wedge->CalDiffractionParameters(sd.Normalize(), de.Normalize(), dP, phiIncident, phiDiffraction);							/** @brief	求解绕射参数	*/
	RtLbsType l = st / (st + deLen);																							/** @brief	球面波扩散因子	*/
	st += deLen;																												//传播距离叠加
	Polarization2D dCoef = _calculateSallabiDiffractionCoef(nValue, phiIncident, phiDiffraction, l, mat, freq, tranFunction);	/** @brief	绕射系数	*/
	//3-求解绕射场
	Polarization2D inEField;																									/** @brief	原始入射电场的射线基表示	*/
	inEField.perp = efield * v3;																								/** @brief	原始电场在射线基坐标系中的垂直分量	*/
	inEField.para = efield * v4;																								/** @brief	原始电场在射线基坐标系中的平行分量	*/
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;																	/** @brief	波数	*/
	Polarization2D dEField;																										/** @brief	绕射末场的射线基坐标表示	*/
	dEField.perp = (inEField.perp * dCoef.perp) * Exp(-waveNumber * deLen);
	dEField.para = (inEField.para * dCoef.para) * Exp(-waveNumber * deLen);
	efield = dEField.perp * v3 + dEField.para * v5;																				//将绕射场转为笛卡尔坐标系下的表示
}

void Polarization3D::CalculateDiffractionField_TerrainUTD(RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const TerrainRidge* ridge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunction)
{
	Polarization3D& efield = *this;														/** @brief	入射电磁场赋值	*/
	//1-几何参量求解
	Vector3D sd = dP - sP;																										/** @brief	场计算起点指向绕射点坐标的向量	*/
	Vector3D de = eP - dP;																										/** @brief	绕射点指向终点坐标的向量	*/
	RtLbsType sdLen = sd.Length();																								/** @brief	起点到绕射点的距离	*/
	RtLbsType deLen = de.Length();																								/** @brief	绕射点至终点的距离	*/
	Vector3D v3 = sd.Cross(de).Normalize();
	Vector3D v4 = sd.Cross(v3).Normalize();
	Vector3D v5 = de.Cross(v3).Normalize();
	//2-求解绕射系数
	RtLbsType phiIncident, phiDiffraction;																						/** @brief	入射角与绕射角	*/
	RtLbsType nValue = ridge->GetNValue();																						/** @brief	获得棱劈角的N值	*/
	ridge->CalDiffractionParameters(sd.Normalize(), de.Normalize(), phiIncident, phiDiffraction);								//计算绕射角参数
	RtLbsType l = st / (st + deLen);																							/** @brief	球面波扩散因子	*/
	st += deLen;																												//传播距离叠加
	Polarization2D dCoef = _calculateSallabiDiffractionCoef(nValue, phiIncident, phiDiffraction, l, mat, freq, tranFunction);	/** @brief	绕射系数	*/
	//3-求解绕射场
	Polarization2D inEField;																									/** @brief	原始入射电场的射线基表示	*/
	inEField.perp = efield * v3;																								/** @brief	原始电场在射线基坐标系中的垂直分量	*/
	inEField.para = efield * v4;																								/** @brief	原始电场在射线基坐标系中的平行分量	*/
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;																	/** @brief	波数	*/
	Polarization2D dEField;																										/** @brief	绕射末场的射线基坐标表示	*/
	dEField.perp = (inEField.perp * dCoef.perp) * Exp(-waveNumber * deLen);
	dEField.para = (inEField.para * dCoef.para) * Exp(-waveNumber * deLen);
	efield = dEField.perp * v3 + dEField.para * v5;																				//将绕射场转为笛卡尔坐标系下的表示
}

void Polarization3D::CalculateLOSFieldByDistance(RtLbsType s, RtLbsType freq)
{
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;				/** @brief	波数	*/
	Complex constTerm;														/** @brief	幅度项	*/
	Complex phaseTerm;														/** @brief	相位项	*/
	if (s < 0) {			//s<0表明为逆向电磁计算
		constTerm.m_real = s;
		phaseTerm.m_real = cos(waveNumber * s * -1.0);
		phaseTerm.m_imag = sin(waveNumber * s * -1.0);
	}
	else if(s > 0) {
		constTerm.m_real = 1.0 / s;
		phaseTerm.m_real = cos(waveNumber * s);
		phaseTerm.m_imag = sin(waveNumber * s);
	}
	else {
		return;
	}

	Complex amplitude = constTerm * phaseTerm;
	*this *= amplitude;														//计算迭代视距后的能量
}

void Polarization3D::CalculateLOSFieldByLoss(RtLbsType loss, RtLbsType freq)
{
	//将dB值转换为线性倍数关系
	RtLbsType size = pow(10, loss / 10.0);
	*this *= size;
}

bool Polarization3D::IsZero() const
{
	if (ex.IsZero() &&
		ey.IsZero() &&
		ez.IsZero()) {
		return true;
	}
	return false;
}

Polarization2D Polarization3D::_calculateReflectionCoef(RtLbsType freq, RtLbsType theta, Material* mat)
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

Polarization2D Polarization3D::_calculateTransmissionCoef(RtLbsType freq, RtLbsType theta, Material* mat)
{
	Complex matEPara = mat->GetComplexForm(freq);               /** @brief	计算材质的复相对介电常数	*/

	//计算过程中所依赖的中间变量的表达式
	Complex snum = Complex(sin(theta), 0);
	Complex cnum = Complex(cos(theta) * cos(theta), 0);
	Complex ee1 = (matEPara - cnum).Sqrt();

	//开始计算反射系数
	Polarization2D tCoef;												/** @brief	透射系数	*/
	tCoef.para = matEPara * snum * 2.0 / (matEPara * snum + ee1);		/** @brief	平行极化波	*/
	tCoef.perp = snum*2.0 / (snum + ee1);								/** @brief	垂直极化波	*/
	return tCoef;
}

Polarization2D Polarization3D::_calculateSallabiDiffractionCoef(RtLbsType n, RtLbsType a1, RtLbsType a2, RtLbsType l, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFucntion)
{
	//绕射系数计算需要用到的中间变量
	Complex Cot1, Cot2, Cot3, Cot4;
	Complex D1, D2, D3, D4;
	Complex Dcon;

	RtLbsType a01, an1;                                         /** @brief	重新定义后的0面的入射角和n面的反射角	*/
	RtLbsType gamma1, gamma2, gamma3, gamma4;                   /** @brief	中间变量	*/
	Polarization2D R0, Rn;                                      /** @brief	0面和1面的反射系数	*/
	Polarization2D Dcof;                                        /** @brief	绕射系数	*/
	Polarization2D RnR0;                                        /** @brief	0面1面反射系数之积	*/
	Polarization2D ModifiedR;                                   /** @brief	修正后的反射系数	*/

	/** @brief	重新定义的0面的入射角(入射线与0面法线的夹角)	*/
	if (a1 > n * HALF_PI)
		a01 = fabs(HALF_PI - (n * PI - a1));
	else
		a01 = fabs(HALF_PI - a1);
	R0 = _specularReflectionCoef(HALF_PI - a01, mat, freq);          /** @brief	改正后的反射系数	*/


	/** @brief	重新定义n面的反射角	*/
	an1 = _redefineAngle(n, a1, a2);                                 /** @brief	n面的修正反射角	*/
	Rn = _specularReflectionCoef(HALF_PI - a01, mat, freq);          /** @brief	n面的修正反射系数	*/

	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;
	//计算用到的中间变量
	Dcon.m_real = -1.0 / (4 * n * sqrt(PI * waveNumber));           /** @brief	绕射常数项实部	*/
	Dcon.m_imag = 1.0 / (4 * n * sqrt(PI * waveNumber));            /** @brief	绕射常数项虚部	*/
	gamma1 = (PI - (a2 - a1)) / (2 * n);                            /** @brief	中间变量计算	*/
	gamma2 = (PI + (a2 - a1)) / (2 * n);                            /** @brief	中间变量计算	*/
	gamma3 = (PI - (a2 + a1)) / (2 * n);                            /** @brief	中间变量计算	*/
	gamma4 = (PI + (a2 + a1)) / (2 * n);                            /** @brief	中间变量计算	*/

	//绕射系数D1的计算
	RtLbsType tanGamma1 = tan(gamma1);
	if (tanGamma1 != 0) {       //值不为0
		Cot1.m_real = 1.0 / tanGamma1;      //实部计算
		D1 = Cot1 * _newTransactionFunction(2 * waveNumber * l * n * n * sin(gamma1) * sin(gamma1), tranFucntion);
	}
	else {      //值为0
		D1.m_real = 0.5 * sqrt(2 * PI * 2 * waveNumber * l * n * n);            //计算D1的实部
		D1.m_imag = D1.m_real;                                                  //计算D1的虚部
	}

	//绕射系数D2的计算
	RtLbsType tanGamma2 = tan(gamma2);
	if (tanGamma2 != 0) {
		Cot2.m_real = 1.0 / tanGamma2;      //实部计算
		D2 = Cot2 * _newTransactionFunction(2 * waveNumber * l * n * n * sin(gamma2) * sin(gamma2), tranFucntion);
	}
	else {      //值为0
		D2.m_real = 0.5 * sqrt(2 * PI * 2 * waveNumber * l * n * n);        //计算D2的实部
		D2.m_imag = D2.m_real;
	}

	//绕射系数D3的计算
	RtLbsType tanGamma3 = tan(gamma3);
	if (tanGamma3 != 0) {
		Cot3.m_real = 1.0 / tanGamma3;      //实部计算
		D3 = Cot3 * _newTransactionFunction(2 * waveNumber * l * n * n * sin(gamma3) * sin(gamma3), tranFucntion);
	}
	else {      //值为0
		D3.m_real = 0.5 * sqrt(2 * PI * 2 * waveNumber * l * n * n);        //计算D3的实部
		D3.m_imag = D3.m_real;
	}

	//绕射系数D4的计算
	RtLbsType tanGamma4 = tan(gamma4);
	if (tan(gamma4) != 0) {     //值不为0
		Cot4.m_real = 1.0 / tanGamma4;      //实部计算
		D4 = Cot4 * _newTransactionFunction(2 * waveNumber * l * n * n * sin(gamma4) * sin(gamma4), tranFucntion);
	}
	else {      //值为0
		D4.m_real = 0.5 * sqrt(2 * PI * 2 * waveNumber * l * n * n);        //计算D4的实部
		D4.m_imag = D4.m_real;
	}

	RnR0.perp = Rn.perp * R0.perp;      //两系数积的垂直分量
	RnR0.para = Rn.para * R0.para;      //两系数积的平行分量

	//绕射系数的计算
	if (a1 > PI) {      //入射角大于180度
		Dcof.perp = Dcon * ((RnR0.perp * D1) + D2 + Rn.perp * D3 + R0.perp * D4);       //绕射系数的垂直分量
		Dcof.para = Dcon * ((RnR0.para * D1) + D2 + Rn.para * D3 + R0.para * D4);       //绕射系数的平行分量
	}
	else if (a1 <= (n - 1) * PI) {      //a1小于等于(n-1)π
		Dcof.perp = Dcon * (D1 + RnR0.perp * D2 + R0.perp * D3 + Rn.perp * D4);         //绕射系数的垂直分量
		Dcof.para = Dcon * (D1 + RnR0.para * D2 + R0.para * D3 + Rn.para * D4);         //绕射系数的平行分量
	}
	else {      //其他情况
		if (a2 <= fabs((2 * n - 1) * PI - a1)) {        //a2小于等于fabs((2 * n - 1) * PI - a1))
			if (a2 >= n * PI - a1) {        //a2大于等于n*pi-a1
				ModifiedR = _rectifiedReflectionCoef(n * PI - a1, n * PI - a2, mat, freq);       //修正的反射系数
			}
			else {
				ModifiedR = _rectifiedReflectionCoef(a1, a2, mat, freq);     //修正的反射系数
			}
			Dcof.perp = Dcon * (D1 + D2 + ModifiedR.perp * (D3 + D4));      //绕射系数的垂直分量
			Dcof.para = Dcon * (D1 + D2 + ModifiedR.para * (D3 + D4));      //绕射系数的平行分量
		}
		else {      //其他情况
			Dcof.perp = Dcon * (D1 + RnR0.perp * D2 + R0.perp * D3 + Rn.perp * D4);     //绕射系数的垂直分量
			Dcof.para = Dcon * (D1 + RnR0.para * D2 + R0.para * D3 + Rn.para * D4);     //绕射系数的平行分量
		}
	}
	return Dcof;
}

Polarization2D Polarization3D::_specularReflectionCoef(RtLbsType theta, const Material* mat, RtLbsType freq)
{
	Polarization2D rCoef;                                       /** @brief	反射系数	*/
	Complex ee, ee1;                                            /** @brief	中间变量	*/
	Complex snum, cnum;                                         /** @brief	中间变量	*/
	ee = mat->GetComplexForm(freq);                             /** @brief  复相对介电常数	*/

	//计算反射系数所用的中间变量
	snum.m_real = sin(theta);
	cnum.m_real = cos(theta) * cos(theta);
	ee1 = (ee - snum).Sqrt();

	//反射系数的计算
	rCoef.para = (ee * snum - ee1) / (ee * snum + ee1);
	rCoef.perp = (snum - ee1) / (snum + ee1);

	return rCoef;
}

RtLbsType Polarization3D::_redefineAngle(RtLbsType n, RtLbsType phi1, RtLbsType phi2)
{
	//局部变量定义
	RtLbsType theta;                                                /** @brief	重新定义的n面反射角	*/
	if (n > 1.5) {                                                  //n值大于1.5情形
		if (phi1 < (n - 1) * PI) {                                  //phi1≤(n-1)*pi 情况
			if (phi2 <= PI - phi1)
				theta = fabs(HALF_PI - phi2);                       //n面反射角
			else if (phi2 >= PI)
				theta = fabs(n * PI - phi2 - HALF_PI);
			else
				theta = fabs(phi2 + (n - 2.5) * PI);
		}
		else if (phi1 > PI) {                                       //角度值大于180°
			if (phi2 > PI)
				theta = fabs(n * PI - phi2 - HALF_PI);              //n面反射角
			else if (phi2 < fabs((2 * n - 1) * PI - phi1))
				theta = fabs(HALF_PI - phi2);
			else
				theta = fabs(phi2 + (n - 2.5) * PI);
		}
		else {                                                      //其他
			if (phi2 >= PI)
				theta = fabs(n * PI - phi2 - HALF_PI);
			else {                                                  //绕射角小于180度
				if (phi2 < fabs(PI - phi1))
					theta = fabs(HALF_PI - phi2);
				else
					theta = fabs(phi2 + (n - 2.5) * PI);
			}
		}
	}
	else {                                                          //n小于1.5
		if (phi1 < (n - 1) * PI) {                                  //入射角小于(n-1)π
			if (phi2 <= PI - phi1)
				theta = fabs(HALF_PI - phi2);
			else if (phi2 > PI)
				theta = fabs(n * PI - phi2 - HALF_PI);
			else
				theta = fabs(phi2 - (n - 0.5) * PI);
		}
		else if (phi1 > PI) {                                       //入射角大于180°
			if (phi2 < fabs((2 * n - 1) * PI - phi1))
				theta = fabs(HALF_PI - phi2);
			else if (phi2 > PI)
				theta = fabs(n * PI - phi2 - HALF_PI);
			else
				theta = fabs(phi2 - (n - 0.5) * PI);
		}
		else {                                                      //其他情况
			if (phi2 < fabs(PI - phi1))
				theta = fabs(HALF_PI - phi2);
			else
				theta = fabs(n * PI - phi2 - HALF_PI);
		}
	}
	return theta;
}

Complex Polarization3D::_newTransactionFunction(double x, const std::vector<Complex>& tranFucntion)
{
	//局部变量定义
	Complex ffx;            /** @brief	过渡函数值	*/
	Complex tem;            /** @brief	中间变量	*/
	int index;              /** @brief	临时变量	*/
	RtLbsType m;            /** @brief	临时变量	*/
	if (x > 10) {           //自变量很大的情况
		m = 1 / (x * x);    //计算中间值
		ffx.m_real = 1 + m * (4.6875 * m - 0.75);       //过渡函数实部
		ffx.m_imag = (0.5 - 1.875 * m) / x;             //过渡函数虚部
	}
	else if (x < 0.001) {       //自变量很小时
		tem.m_real = sqrt(PI * x) - sqrt(2.0) * x * (1 + x / 3);        //过渡函数实部
		tem.m_imag = sqrt(2.0) * x * (x / 3 - 1);                       //过渡函数虚部
		ffx = tem * Exp(QUARTER_PI + x);                                //福哦度函数数值计算
	}
	else {      //其他情况
		x = x * 10000 - 1 + 0.5;        //计算中间值
		index = static_cast<int>(x);
		ffx = tranFucntion[index];
	}
	return ffx;
}

Polarization2D Polarization3D::_rectifiedReflectionCoef(RtLbsType a1, RtLbsType a2, const Material* mat, RtLbsType freq)
{
	double deltaH = 0.002;                      /** @brief	反射界面粗糙度引起的衰减因子、表面粗糙度正太分布的标准偏差	*/
	Polarization2D rCoefRectify;                /** @brief	修正后的反射系数	*/
	Complex ee, ee1;                            /** @brief	计算依赖的中间变量	*/
	Complex tao, cnum;                          /** @brief	计算依赖的中间变量	*/

	ee = mat->GetComplexForm(freq);             /** @brief	计算材质复相对介电参数	*/

	//计算反射系数中所用的中间变量
	tao.m_real = 2 * sin(0.5 * a2) * sin(0.5 * a1);     /** @brief	复数实部	*/
	ee1 = (ee - cnum + tao * tao).Sqrt();

	//反射系数的计算
	rCoefRectify.para = (ee * tao - ee1) / (ee * tao + ee1);
	rCoefRectify.perp = (tao - ee1) / (tao + ee1);
	return rCoefRectify;
}
