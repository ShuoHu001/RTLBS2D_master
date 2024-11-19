#include "caldiffractionfield.h"

void CalculateDiffractionField_ReverseRT(Polarization3D& inField, RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const Wedge2D* wedge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunctionData)
{
	//首先进行特殊处理，若计算场点的位置与绕射点的位置较为接近，则不进行场的任何计算
	Vector3D sd = dP - sP;																										/** @brief	场计算起点指向绕射点坐标的向量	*/
	Vector3D de = eP - dP;																										/** @brief	绕射点指向终点坐标的向量	*/
	Vector3D sdNormal = Normalize(sd);
	Vector3D deNormal = Normalize(de);
	RtLbsType sdLen = sd.Length();																								/** @brief	起点到绕射点的距离	*/
	RtLbsType deLen = de.Length();																								/** @brief	绕射点至终点的距离	*/
	if (sdLen < EPSILON || deLen < EPSILON)																						//若累加距离小于系统的默认值，则不进行任何场计算
		return;
	//求解入射角和绕射角(确定0面和N面)
	RtLbsType phiIncident, phiDiffraction;																						/** @brief	入射角与绕射角	*/
	RtLbsType nValue = wedge->GetNValue();																						/** @brief	棱劈内角n值(2-n)π	*/
	wedge->CalDiffractionParameters(sdNormal, deNormal, dP, phiIncident, phiDiffraction);
	Complex as;
	RtLbsType l = st / (st + deLen);																							/** @brief	球面波扩散因子	*/
	as.m_real = l;
	st += deLen;

	//求解绕射系数
	Polarization2D dCoef = _calculateSallabiDiffractionCoef(nValue, phiIncident, phiDiffraction, l, mat, freq, tranFunctionData);
	//入射电场的射线基表示
	Vector3D v3 = sdNormal.Cross(deNormal);
	Vector3D v4 = sdNormal.Cross(v3);
	Vector3D v5 = deNormal.Cross(v3);
	Polarization2D inEField;                                                            /** @brief	原始入射电场的射线基表示	*/
	inEField.perp = inField * v3;                                                        /** @brief	原始电场在射线基坐标系中的垂直分量	*/
	inEField.para = inField * v4;                                                        /** @brief	原始电场在射线基坐标系中的平行分量	*/

	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;

	//绕射末场的射线基表示
	Polarization2D dEField;                                                             /** @brief	绕射末场	*/
	dEField.perp = (inEField.perp * dCoef.perp) * (as * Exp(-waveNumber * deLen));
	dEField.para = (inEField.para * dCoef.para) * (as * Exp(-waveNumber * deLen));

	//将绕射场转为笛卡尔坐标系下的表示
	inField = dEField.perp * v3 + dEField.para * v5;
}

void CalculateDiffractionField_ForwardRT(Polarization3D& inField, RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const Wedge2D* wedge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunctionData)
{
	//1-几何参量求解
	Vector3D sd = dP - sP;																										/** @brief	场计算起点指向绕射点坐标的向量	*/
	Vector3D de = eP - dP;																										/** @brief	绕射点指向终点坐标的向量	*/
	Vector3D sdNormal = Normalize(sd);
	Vector3D deNormal = Normalize(sd);
	RtLbsType sdLen = sd.Length();																								/** @brief	起点到绕射点的距离	*/
	RtLbsType deLen = de.Length();																								/** @brief	绕射点至终点的距离	*/
	Vector3D v3 = sdNormal.Cross(deNormal);
	Vector3D v4 = sdNormal.Cross(v3);
	Vector3D v5 = deNormal.Cross(v3);
	//2-求解绕射系数
	RtLbsType phiIncident, phiDiffraction;																						/** @brief	入射角与绕射角	*/
	RtLbsType nValue = wedge->GetNValue();																						/** @brief	棱劈内角n值(2-n)π	*/
	wedge->CalDiffractionParameters(sdNormal, deNormal, dP, phiIncident, phiDiffraction);										/** @brief	求解绕射参数	*/
	RtLbsType l = st / (st + deLen);																							/** @brief	球面波扩散因子	*/
	st += deLen;																												//传播距离叠加
	Polarization2D dCoef = _calculateSallabiDiffractionCoef(nValue, phiIncident, phiDiffraction, l, mat, freq, tranFunctionData);	/** @brief	绕射系数	*/
	//3-求解绕射场
	Polarization2D inEField;																									/** @brief	原始入射电场的射线基表示	*/
	inEField.perp = inField * v3;																								/** @brief	原始电场在射线基坐标系中的垂直分量	*/
	inEField.para = inField * v4;																								/** @brief	原始电场在射线基坐标系中的平行分量	*/
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;																	/** @brief	波数	*/
	Polarization2D dEField;																										/** @brief	绕射末场的射线基坐标表示	*/
	dEField.perp = (inEField.perp * dCoef.perp) * Exp(-waveNumber * deLen);
	dEField.para = (inEField.para * dCoef.para) * Exp(-waveNumber * deLen);
	inField = dEField.perp * v3 + dEField.para * v5;																				//将绕射场转为笛卡尔坐标系下的表示
}

void CalculateDiffractionField_TerrainUTD(Polarization3D& inField, RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const TerrainRidge* ridge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunctionData)
{
	//1-几何参量求解
	Vector3D sd = dP - sP;																										/** @brief	场计算起点指向绕射点坐标的向量	*/
	Vector3D de = eP - dP;																										/** @brief	绕射点指向终点坐标的向量	*/
	Vector3D sdNormal = Normalize(sd);
	Vector3D deNormal = Normalize(de);
	RtLbsType sdLen = sd.Length();																								/** @brief	起点到绕射点的距离	*/
	RtLbsType deLen = de.Length();																								/** @brief	绕射点至终点的距离	*/
	Vector3D v3 = sdNormal.Cross(deNormal);
	Vector3D v4 = sdNormal.Cross(v3);
	Vector3D v5 = deNormal.Cross(v3);
	//2-求解绕射系数
	RtLbsType phiIncident, phiDiffraction;																						/** @brief	入射角与绕射角	*/
	RtLbsType nValue = ridge->GetNValue();																						/** @brief	获得棱劈角的N值	*/
	ridge->CalDiffractionParameters(sdNormal, deNormal, phiIncident, phiDiffraction);											//计算绕射角参数
	RtLbsType l = st / (st + deLen);																							/** @brief	球面波扩散因子	*/
	st += deLen;																												//传播距离叠加
	Polarization2D dCoef = _calculateSallabiDiffractionCoef(nValue, phiIncident, phiDiffraction, l, mat, freq, tranFunctionData);	/** @brief	绕射系数	*/
	//3-求解绕射场
	Polarization2D inEField;																									/** @brief	原始入射电场的射线基表示	*/
	inEField.perp = inField * v3;																								/** @brief	原始电场在射线基坐标系中的垂直分量	*/
	inEField.para = inField * v4;																								/** @brief	原始电场在射线基坐标系中的平行分量	*/
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;																	/** @brief	波数	*/
	Polarization2D dEField;																										/** @brief	绕射末场的射线基坐标表示	*/
	dEField.perp = (inEField.perp * dCoef.perp) * Exp(-waveNumber * deLen);
	dEField.para = (inEField.para * dCoef.para) * Exp(-waveNumber * deLen);
	inField = dEField.perp * v3 + dEField.para * v5;																				//将绕射场转为笛卡尔坐标系下的表示
}

Polarization2D _calculateSallabiDiffractionCoef(RtLbsType n, RtLbsType a1, RtLbsType a2, RtLbsType l, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunctionData)
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
		D1 = Cot1 * _newTransactionFunction(2 * waveNumber * l * n * n * sin(gamma1) * sin(gamma1), tranFunctionData);
	}
	else {      //值为0
		D1.m_real = 0.5 * sqrt(2 * PI * 2 * waveNumber * l * n * n);            //计算D1的实部
		D1.m_imag = D1.m_real;                                                  //计算D1的虚部
	}

	//绕射系数D2的计算
	RtLbsType tanGamma2 = tan(gamma2);
	if (tanGamma2 != 0) {
		Cot2.m_real = 1.0 / tanGamma2;      //实部计算
		D2 = Cot2 * _newTransactionFunction(2 * waveNumber * l * n * n * sin(gamma2) * sin(gamma2), tranFunctionData);
	}
	else {      //值为0
		D2.m_real = 0.5 * sqrt(2 * PI * 2 * waveNumber * l * n * n);        //计算D2的实部
		D2.m_imag = D2.m_real;
	}

	//绕射系数D3的计算
	RtLbsType tanGamma3 = tan(gamma3);
	if (tanGamma3 != 0) {
		Cot3.m_real = 1.0 / tanGamma3;      //实部计算
		D3 = Cot3 * _newTransactionFunction(2 * waveNumber * l * n * n * sin(gamma3) * sin(gamma3), tranFunctionData);
	}
	else {      //值为0
		D3.m_real = 0.5 * sqrt(2 * PI * 2 * waveNumber * l * n * n);        //计算D3的实部
		D3.m_imag = D3.m_real;
	}

	//绕射系数D4的计算
	RtLbsType tanGamma4 = tan(gamma4);
	if (tan(gamma4) != 0) {     //值不为0
		Cot4.m_real = 1.0 / tanGamma4;      //实部计算
		D4 = Cot4 * _newTransactionFunction(2 * waveNumber * l * n * n * sin(gamma4) * sin(gamma4), tranFunctionData);
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

Polarization2D _specularReflectionCoef(RtLbsType theta, const Material* mat, RtLbsType freq)
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

RtLbsType _redefineAngle(RtLbsType n, RtLbsType phi1, RtLbsType phi2)
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

Complex _newTransactionFunction(double x, const std::vector<Complex>& tranFunctionData)
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
		ffx = tranFunctionData[index];
	}
	return ffx;
}


Polarization2D _rectifiedReflectionCoef(RtLbsType a1, RtLbsType a2, const Material* mat, RtLbsType freq)
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