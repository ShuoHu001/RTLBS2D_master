#include "calreflectionfield.h"

void CalculateReflectionField_ReverseRT(Polarization3D& inField, RtLbsType& st, const Point3D& sP, const Point3D& rP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	//���Ƚ������⴦�������㳡��λ���뷴����λ�ý�Ϊ�ӽ��������κγ��ĵ��Ӽ���
	Vector3D sr = rP - sP;
	Vector3D re = eP - rP;
	Vector3D srNormal = Normalize(sr);
	Vector3D reNormal = Normalize(re);
	RtLbsType srLen = sr.Length();                                           /** @brief	����������뷴��������ľ���	*/
	RtLbsType reLen = re.Length();                                           /** @brief	�����㷴������յ������ľ���	*/
	if (srLen < EPSILON || reLen < EPSILON) {                                //���ۼӾ���С��ϵͳĬ����Сֵ������Ϊ�������κ���ʽ�ĳ��������
		return;
	}
	//��ⷴ��ϵ������
	//������������뷴�������ƽ��ļн�theta
	RtLbsType cosTheta = srNormal * reNormal;
	RtLbsType theta = 0.0;
	if (cosTheta < -1)
		theta = HALF_PI;
	else if (cosTheta == 0)
		theta = 0;
	else
		theta = acos(cosTheta) / 2.0;


	//���㷴����ĩ��ֵ
	Complex as;									/** @brief	�������м����	*/
	as.m_real = st / (st + reLen);              /** @brief	���沨��ɢ����	*/
	st += reLen;

	Vector3D v3 = srNormal.Cross(reNormal);
	Vector3D v4 = srNormal.Cross(v3);
	Vector3D v5 = reNormal.Cross(v3);

	//���㷴��ϵ��
	Polarization2D rcoef = _calculateReflectionCoef(freq, theta, mat);

	//������糡ʸ��ת��Ϊ���߻�����ϵ�µı�ʾ
	Polarization2D inEfield;							 /** @brief	ԭʼ����糡�����߻���ʾ	*/
	inEfield.perp = inField * v3;                         /** @brief	ԭʼ�糡�����߻�����ϵ��ֱ����	*/
	inEfield.para = inField * v4;                         /** @brief	ԭʼ�糡�����߻�����ֵƽ�з���	*/

	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;
	//����ĩ�������߻���ʾ
	Polarization2D rEField;                             /** @brief	����ĩ��	*/
	rEField.perp = (inEfield.perp * rcoef.perp) * (as * Exp(-waveNumber * reLen));
	rEField.para = (inEfield.para * rcoef.para) * (as * Exp(-waveNumber * reLen));

	//�����߻�����ϵ����ĩ��תΪ�ѿ�������ϵ��ʾ
	inField = rEField.perp * v3 + rEField.para * v5;
}

void CalculateReflectionField_ForwardRT(Polarization3D& inField, RtLbsType& st, const Point3D& sP, const Point3D rP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	//1-���㼸�γ���
	Vector3D sr = rP - sP;
	Vector3D re = eP - rP;
	Vector3D srNormal = Normalize(sr);
	Vector3D reNormal = Normalize(re);
	RtLbsType srLen = sr.Length();                                           /** @brief	����������뷴��������ľ���	*/
	RtLbsType reLen = re.Length();                                           /** @brief	�����㷴������յ������ľ���	*/
	st += reLen;															 //�����������
	Vector3D v3 = srNormal.Cross(reNormal);
	Vector3D v4 = srNormal.Cross(v3);
	Vector3D v5 = reNormal.Cross(v3);
	//2-��ⷴ��ϵ��
	RtLbsType cosTheta = srNormal * reNormal;
	RtLbsType theta = 0.0;
	if (cosTheta < -1)
		theta = HALF_PI;
	else if (cosTheta == 0)
		theta = 0;
	else
		theta = acos(cosTheta) / 2.0;
	Polarization2D rcoef = _calculateReflectionCoef(freq, theta, mat);		/** @brief	����ϵ��	*/
	//3-��ⷴ�䳡
	Polarization2D inEfield;												/** @brief	ԭʼ����糡�����߻���ʾ	*/
	inEfield.perp = inField * v3;											/** @brief	ԭʼ�糡�����߻�����ϵ��ֱ����	*/
	inEfield.para = inField * v4;											/** @brief	ԭʼ�糡�����߻�����ֵƽ�з���	*/
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;		/** @brief	����	*/
	Polarization2D rEField;                             /** @brief	����ĩ��-���߻���ʽ���	*/
	rEField.perp = (inEfield.perp * rcoef.perp) * Exp(-waveNumber * reLen);
	rEField.para = (inEfield.para * rcoef.para) * Exp(-waveNumber * reLen);

	//�����߻�����ϵ����ĩ��תΪ�ѿ�������ϵ��ʾ
	inField = rEField.perp * v3 + rEField.para * v5;
}



Polarization2D _calculateReflectionCoef(RtLbsType freq, RtLbsType theta, Material* mat)
{
	Complex matEPara = mat->GetComplexForm(freq);               /** @brief	������ʵĸ���Խ�糣��	*/

	//������������������м�����ı��ʽ
	Complex snum = Complex(sin(theta), 0);
	Complex cnum = Complex(cos(theta) * cos(theta), 0);
	Complex ee1 = (matEPara - cnum).Sqrt();

	//��ʼ���㷴��ϵ��
	Polarization2D rCoef;												/** @brief	����ϵ��	*/
	rCoef.para = (matEPara * snum - ee1) / (matEPara * snum + ee1);     /** @brief	ƽ�м�����	*/
	rCoef.perp = (snum - ee1) / (snum + ee1);                           /** @brief	��ֱ������	*/
	return rCoef;
}

