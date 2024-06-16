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
	//��������ֵ��������ά�ռ���
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
	Polarization3D& efield = *this;											/** @brief	����糡	*/
	//���Ƚ������⴦�������㳡��λ���뷴����λ�ý�Ϊ�ӽ��������κγ��ĵ��Ӽ���
	Vector3D sr = rP - sP;
	Vector3D re = eP - rP;
	RtLbsType srLen = sr.Length();                                           /** @brief	����������뷴��������ľ���	*/
	RtLbsType reLen = re.Length();                                           /** @brief	�����㷴������յ������ľ���	*/
	if (srLen < EPSILON || reLen < EPSILON) {                                //���ۼӾ���С��ϵͳĬ����Сֵ������Ϊ�������κ���ʽ�ĳ��������
		return;
	}
	//��ⷴ��ϵ������
	//������������뷴�������ƽ��ļн�theta
	RtLbsType cosTheta = sr.Normalize() * re.Normalize();
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

	Vector3D v3 = sr.Cross(re).Normalize();
	Vector3D v4 = sr.Cross(v3).Normalize();
	Vector3D v5 = re.Cross(v3).Normalize();

	//���㷴��ϵ��
	Polarization2D rcoef = _calculateReflectionCoef(freq, theta, mat);

	//������糡ʸ��ת��Ϊ���߻�����ϵ�µı�ʾ
	Polarization2D inEfield;							 /** @brief	ԭʼ����糡�����߻���ʾ	*/
	inEfield.perp = efield * v3;                         /** @brief	ԭʼ�糡�����߻�����ϵ��ֱ����	*/
	inEfield.para = efield * v4;                         /** @brief	ԭʼ�糡�����߻�����ֵƽ�з���	*/

	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;
	//����ĩ�������߻���ʾ
	Polarization2D rEField;                             /** @brief	����ĩ��	*/
	rEField.perp = (inEfield.perp * rcoef.perp) * (as * Exp(-waveNumber * reLen));
	rEField.para = (inEfield.para * rcoef.para) * (as * Exp(-waveNumber * reLen));

	//�����߻�����ϵ����ĩ��תΪ�ѿ�������ϵ��ʾ
	efield = rEField.perp * v3 + rEField.para * v5;
}

void Polarization3D::CalculateReflectionField_ForwardRT(RtLbsType& st, const Point3D& sP, const Point3D rP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	Polarization3D& efield = *this;											/** @brief	����糡	*/
	//1-���㼸�γ���
	Vector3D sr = rP - sP;
	Vector3D re = eP - rP;
	RtLbsType srLen = sr.Length();                                           /** @brief	����������뷴��������ľ���	*/
	RtLbsType reLen = re.Length();                                           /** @brief	�����㷴������յ������ľ���	*/
	st += reLen;															 //�����������
	Vector3D v3 = sr.Cross(re).Normalize();
	Vector3D v4 = sr.Cross(v3).Normalize();
	Vector3D v5 = re.Cross(v3).Normalize();
	//2-��ⷴ��ϵ��
	RtLbsType cosTheta = sr.Normalize() * re.Normalize();
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
	inEfield.perp = efield * v3;											/** @brief	ԭʼ�糡�����߻�����ϵ��ֱ����	*/
	inEfield.para = efield * v4;											/** @brief	ԭʼ�糡�����߻�����ֵƽ�з���	*/
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;		/** @brief	����	*/
	Polarization2D rEField;                             /** @brief	����ĩ��-���߻���ʽ���	*/
	rEField.perp = (inEfield.perp * rcoef.perp) * Exp(-waveNumber * reLen);
	rEField.para = (inEfield.para * rcoef.para) * Exp(-waveNumber * reLen);

	//�����߻�����ϵ����ĩ��תΪ�ѿ�������ϵ��ʾ
	efield = rEField.perp * v3 + rEField.para * v5;
}

void Polarization3D::CalculateTransmissionField_ReverseRT(RtLbsType& st, const Point3D& sP, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	Polarization3D& efield = *this;											/** @brief	����糡	*/
	//���Ƚ������⴦�������㳡��λ���뷴����λ�ý�Ϊ�ӽ��������κγ��ĵ��Ӽ���
	Vector3D sr = tP - sP;
	Vector3D re = eP - tP;
	RtLbsType srLen = sr.Length();                                           /** @brief	����������뷴��������ľ���	*/
	RtLbsType reLen = re.Length();                                           /** @brief	�����㷴������յ������ľ���	*/
	if (srLen < EPSILON || reLen < EPSILON) {                                //���ۼӾ���С��ϵͳĬ����Сֵ������Ϊ�������κ���ʽ�ĳ��������
		return;
	}
	//���͸��ϵ������
	//�������������͸�������ƽ��ļн�theta
	RtLbsType cosTheta = sr.Normalize() * re.Normalize();
	RtLbsType theta = 0.0;
	if (cosTheta < -1)
		theta = HALF_PI;
	else if (cosTheta == 0)
		theta = 0;
	else
		theta = acos(cosTheta) / 2.0;


	//����͸����ĩ��ֵ
	Complex as;									/** @brief	�������м����	*/
	as.m_real = st / (st + reLen);              /** @brief	���沨��ɢ����	*/
	st += reLen;

	Vector3D v3 = sr.Cross(re).Normalize();
	Vector3D v4 = sr.Cross(v3).Normalize();
	Vector3D v5 = re.Cross(v3).Normalize();

	//����͸��ϵ��
	Polarization2D tcoef = _calculateTransmissionCoef(freq, theta, mat);

	//������糡ʸ��ת��Ϊ���߻�����ϵ�µı�ʾ
	Polarization2D inEfield;                            /** @brief	ԭʼ����糡�����߻���ʾ	*/
	inEfield.perp = efield * v3;                         /** @brief	ԭʼ�糡�����߻�����ϵ��ֱ����	*/
	inEfield.para = efield * v4;                         /** @brief	ԭʼ�糡�����߻�����ֵƽ�з���	*/

	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;
	//͸��ĩ�������߻���ʾ
	Polarization2D rEField;                             /** @brief	����ĩ��	*/
	rEField.perp = (inEfield.perp * tcoef.perp) * (as * Exp(-waveNumber * reLen));
	rEField.para = (inEfield.para * tcoef.para) * (as * Exp(-waveNumber * reLen));

	//�����߻�����ϵ͸��ĩ��תΪ�ѿ�������ϵ��ʾ
	efield = rEField.perp * v3 + rEField.para * v5;
}

void Polarization3D::CalculateTransmissionField_ForwardRT(RtLbsType& st, const Point3D& sP, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	Polarization3D& efield = *this;											/** @brief	����糡	*/
	//1-���㼸�γ���
	Vector3D sr = tP - sP;
	Vector3D re = eP - tP;
	RtLbsType srLen = sr.Length();                                           /** @brief	����������뷴��������ľ���	*/
	RtLbsType reLen = re.Length();                                           /** @brief	�����㷴������յ������ľ���	*/
	st += reLen;															 //�����������
	Vector3D v3 = sr.Cross(re).Normalize();
	Vector3D v4 = sr.Cross(v3).Normalize();
	Vector3D v5 = re.Cross(v3).Normalize();
	//2-���͸��ϵ������
	RtLbsType cosTheta = sr.Normalize() * re.Normalize();
	RtLbsType theta = 0.0;
	if (cosTheta < -1)
		theta = HALF_PI;
	else if (cosTheta == 0)
		theta = 0;
	else
		theta = acos(cosTheta) / 2.0;
	Polarization2D tcoef = _calculateTransmissionCoef(freq, theta, mat);
	//3-���͸�䳡
	Polarization2D inEfield;										/** @brief	ԭʼ����糡�����߻���ʾ	*/
	inEfield.perp = efield * v3;									/** @brief	ԭʼ�糡�����߻�����ϵ��ֱ����	*/
	inEfield.para = efield * v4;									/** @brief	ԭʼ�糡�����߻�����ֵƽ�з���	*/
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;		/** @brief	����	*/
	Polarization2D tEField;											/** @brief	͸��ĩ��	*/
	tEField.perp = (inEfield.perp * tcoef.perp) * Exp(-waveNumber * reLen);		/** @brief	��ֱ�����Ĳ�	*/
	tEField.para = (inEfield.para * tcoef.para) * Exp(-waveNumber * reLen);		/** @brief	ƽ�з����Ĳ�	*/
	//�����߻�����ϵ͸��ĩ��תΪ�ѿ�������ϵ��ʾ
	efield = tEField.perp * v3 + tEField.para * v5;
}

void Polarization3D::CalculateStraightTransmissionField_ReverseRT(RtLbsType& st, const Point3D& tP, const Point3D& eP, Material* mat, RtLbsType freq)
{
	//����ʵ�ʴ�͸����
	Vector3D te = eP - tP;
	RtLbsType teLen = te.Length();
	RtLbsType loss_dB = teLen * mat->m_penetrationLoss;
	//��dBת��Ϊ���Ե�λ��������ֵ��(ʵ���鲿����)
	RtLbsType loss_line = pow(10.0, loss_dB / 10.0);
	Polarization3D& efield = *this;
	efield *= loss_line;
}

void Polarization3D::CalculateDiffractionField_ReverseRT(RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const Wedge2D* wedge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFucntion)
{
	//���Ƚ������⴦�������㳡���λ����������λ�ý�Ϊ�ӽ����򲻽��г����κμ���
	Vector3D sd = dP - sP;																										/** @brief	���������ָ����������������	*/
	Vector3D de = eP - dP;																										/** @brief	�����ָ���յ����������	*/
	RtLbsType sdLen = sd.Length();																								/** @brief	��㵽�����ľ���	*/
	RtLbsType deLen = de.Length();																								/** @brief	��������յ�ľ���	*/
	if (sdLen < EPSILON || deLen < EPSILON)																						//���ۼӾ���С��ϵͳ��Ĭ��ֵ���򲻽����κγ�����
		return;
	//�������Ǻ������(ȷ��0���N��)
	RtLbsType phiIncident, phiDiffraction;																						/** @brief	������������	*/
	RtLbsType nValue = wedge->GetNValue();																						/** @brief	�����ڽ�nֵ(2-n)��	*/
	wedge->CalDiffractionParameters(sd.Normalize(), de.Normalize(), dP, phiIncident, phiDiffraction);
	Complex as;
	RtLbsType l = st / (st + deLen);																							/** @brief	���沨��ɢ����	*/
	as.m_real = l;
	st += deLen;

	Polarization3D& efield = *this;
	//�������ϵ��
	Polarization2D dCoef = _calculateSallabiDiffractionCoef(nValue, phiIncident, phiDiffraction, l, mat, freq, tranFucntion);
	//����糡�����߻���ʾ
	Vector3D v3 = sd.Cross(de).Normalize();
	Vector3D v4 = sd.Cross(v3).Normalize();
	Vector3D v5 = de.Cross(v3).Normalize();
	Polarization2D inEField;                                                            /** @brief	ԭʼ����糡�����߻���ʾ	*/
	inEField.perp = efield * v3;                                                        /** @brief	ԭʼ�糡�����߻�����ϵ�еĴ�ֱ����	*/
	inEField.para = efield * v4;                                                        /** @brief	ԭʼ�糡�����߻�����ϵ�е�ƽ�з���	*/

	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;

	//����ĩ�������߻���ʾ
	Polarization2D dEField;                                                             /** @brief	����ĩ��	*/
	dEField.perp = (inEField.perp * dCoef.perp) * (as * Exp(-waveNumber * deLen));
	dEField.para = (inEField.para * dCoef.para) * (as * Exp(-waveNumber * deLen));

	//�����䳡תΪ�ѿ�������ϵ�µı�ʾ
	efield = dEField.perp * v3 + dEField.para * v5;
}

void Polarization3D::CalculateDiffractionField_ForwardRT(RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const Wedge2D* wedge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunction)
{
	Polarization3D& efield = *this;														/** @brief	�����ų���ֵ	*/
	//1-���β������
	Vector3D sd = dP - sP;																										/** @brief	���������ָ����������������	*/
	Vector3D de = eP - dP;																										/** @brief	�����ָ���յ����������	*/
	RtLbsType sdLen = sd.Length();																								/** @brief	��㵽�����ľ���	*/
	RtLbsType deLen = de.Length();																								/** @brief	��������յ�ľ���	*/
	Vector3D v3 = sd.Cross(de).Normalize();
	Vector3D v4 = sd.Cross(v3).Normalize();
	Vector3D v5 = de.Cross(v3).Normalize();
	//2-�������ϵ��
	RtLbsType phiIncident, phiDiffraction;																						/** @brief	������������	*/
	RtLbsType nValue = wedge->GetNValue();																						/** @brief	�����ڽ�nֵ(2-n)��	*/
	wedge->CalDiffractionParameters(sd.Normalize(), de.Normalize(), dP, phiIncident, phiDiffraction);							/** @brief	����������	*/
	RtLbsType l = st / (st + deLen);																							/** @brief	���沨��ɢ����	*/
	st += deLen;																												//�����������
	Polarization2D dCoef = _calculateSallabiDiffractionCoef(nValue, phiIncident, phiDiffraction, l, mat, freq, tranFunction);	/** @brief	����ϵ��	*/
	//3-������䳡
	Polarization2D inEField;																									/** @brief	ԭʼ����糡�����߻���ʾ	*/
	inEField.perp = efield * v3;																								/** @brief	ԭʼ�糡�����߻�����ϵ�еĴ�ֱ����	*/
	inEField.para = efield * v4;																								/** @brief	ԭʼ�糡�����߻�����ϵ�е�ƽ�з���	*/
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;																	/** @brief	����	*/
	Polarization2D dEField;																										/** @brief	����ĩ�������߻������ʾ	*/
	dEField.perp = (inEField.perp * dCoef.perp) * Exp(-waveNumber * deLen);
	dEField.para = (inEField.para * dCoef.para) * Exp(-waveNumber * deLen);
	efield = dEField.perp * v3 + dEField.para * v5;																				//�����䳡תΪ�ѿ�������ϵ�µı�ʾ
}

void Polarization3D::CalculateDiffractionField_TerrainUTD(RtLbsType& st, const Vector3D& sP, const Vector3D& dP, const Vector3D& eP, const TerrainRidge* ridge, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFunction)
{
	Polarization3D& efield = *this;														/** @brief	�����ų���ֵ	*/
	//1-���β������
	Vector3D sd = dP - sP;																										/** @brief	���������ָ����������������	*/
	Vector3D de = eP - dP;																										/** @brief	�����ָ���յ����������	*/
	RtLbsType sdLen = sd.Length();																								/** @brief	��㵽�����ľ���	*/
	RtLbsType deLen = de.Length();																								/** @brief	��������յ�ľ���	*/
	Vector3D v3 = sd.Cross(de).Normalize();
	Vector3D v4 = sd.Cross(v3).Normalize();
	Vector3D v5 = de.Cross(v3).Normalize();
	//2-�������ϵ��
	RtLbsType phiIncident, phiDiffraction;																						/** @brief	������������	*/
	RtLbsType nValue = ridge->GetNValue();																						/** @brief	��������ǵ�Nֵ	*/
	ridge->CalDiffractionParameters(sd.Normalize(), de.Normalize(), phiIncident, phiDiffraction);								//��������ǲ���
	RtLbsType l = st / (st + deLen);																							/** @brief	���沨��ɢ����	*/
	st += deLen;																												//�����������
	Polarization2D dCoef = _calculateSallabiDiffractionCoef(nValue, phiIncident, phiDiffraction, l, mat, freq, tranFunction);	/** @brief	����ϵ��	*/
	//3-������䳡
	Polarization2D inEField;																									/** @brief	ԭʼ����糡�����߻���ʾ	*/
	inEField.perp = efield * v3;																								/** @brief	ԭʼ�糡�����߻�����ϵ�еĴ�ֱ����	*/
	inEField.para = efield * v4;																								/** @brief	ԭʼ�糡�����߻�����ϵ�е�ƽ�з���	*/
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;																	/** @brief	����	*/
	Polarization2D dEField;																										/** @brief	����ĩ�������߻������ʾ	*/
	dEField.perp = (inEField.perp * dCoef.perp) * Exp(-waveNumber * deLen);
	dEField.para = (inEField.para * dCoef.para) * Exp(-waveNumber * deLen);
	efield = dEField.perp * v3 + dEField.para * v5;																				//�����䳡תΪ�ѿ�������ϵ�µı�ʾ
}

void Polarization3D::CalculateLOSFieldByDistance(RtLbsType s, RtLbsType freq)
{
	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;				/** @brief	����	*/
	Complex constTerm;														/** @brief	������	*/
	Complex phaseTerm;														/** @brief	��λ��	*/
	if (s < 0) {			//s<0����Ϊ�����ż���
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
	*this *= amplitude;														//��������Ӿ�������
}

void Polarization3D::CalculateLOSFieldByLoss(RtLbsType loss, RtLbsType freq)
{
	//��dBֵת��Ϊ���Ա�����ϵ
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

Polarization2D Polarization3D::_calculateTransmissionCoef(RtLbsType freq, RtLbsType theta, Material* mat)
{
	Complex matEPara = mat->GetComplexForm(freq);               /** @brief	������ʵĸ���Խ�糣��	*/

	//������������������м�����ı��ʽ
	Complex snum = Complex(sin(theta), 0);
	Complex cnum = Complex(cos(theta) * cos(theta), 0);
	Complex ee1 = (matEPara - cnum).Sqrt();

	//��ʼ���㷴��ϵ��
	Polarization2D tCoef;												/** @brief	͸��ϵ��	*/
	tCoef.para = matEPara * snum * 2.0 / (matEPara * snum + ee1);		/** @brief	ƽ�м�����	*/
	tCoef.perp = snum*2.0 / (snum + ee1);								/** @brief	��ֱ������	*/
	return tCoef;
}

Polarization2D Polarization3D::_calculateSallabiDiffractionCoef(RtLbsType n, RtLbsType a1, RtLbsType a2, RtLbsType l, const Material* mat, RtLbsType freq, const std::vector<Complex>& tranFucntion)
{
	//����ϵ��������Ҫ�õ����м����
	Complex Cot1, Cot2, Cot3, Cot4;
	Complex D1, D2, D3, D4;
	Complex Dcon;

	RtLbsType a01, an1;                                         /** @brief	���¶�����0�������Ǻ�n��ķ����	*/
	RtLbsType gamma1, gamma2, gamma3, gamma4;                   /** @brief	�м����	*/
	Polarization2D R0, Rn;                                      /** @brief	0���1��ķ���ϵ��	*/
	Polarization2D Dcof;                                        /** @brief	����ϵ��	*/
	Polarization2D RnR0;                                        /** @brief	0��1�淴��ϵ��֮��	*/
	Polarization2D ModifiedR;                                   /** @brief	������ķ���ϵ��	*/

	/** @brief	���¶����0��������(��������0�淨�ߵļн�)	*/
	if (a1 > n * HALF_PI)
		a01 = fabs(HALF_PI - (n * PI - a1));
	else
		a01 = fabs(HALF_PI - a1);
	R0 = _specularReflectionCoef(HALF_PI - a01, mat, freq);          /** @brief	������ķ���ϵ��	*/


	/** @brief	���¶���n��ķ����	*/
	an1 = _redefineAngle(n, a1, a2);                                 /** @brief	n������������	*/
	Rn = _specularReflectionCoef(HALF_PI - a01, mat, freq);          /** @brief	n�����������ϵ��	*/

	RtLbsType waveNumber = TWO_PI * freq / LIGHT_VELOCITY_AIR;
	//�����õ����м����
	Dcon.m_real = -1.0 / (4 * n * sqrt(PI * waveNumber));           /** @brief	���䳣����ʵ��	*/
	Dcon.m_imag = 1.0 / (4 * n * sqrt(PI * waveNumber));            /** @brief	���䳣�����鲿	*/
	gamma1 = (PI - (a2 - a1)) / (2 * n);                            /** @brief	�м��������	*/
	gamma2 = (PI + (a2 - a1)) / (2 * n);                            /** @brief	�м��������	*/
	gamma3 = (PI - (a2 + a1)) / (2 * n);                            /** @brief	�м��������	*/
	gamma4 = (PI + (a2 + a1)) / (2 * n);                            /** @brief	�м��������	*/

	//����ϵ��D1�ļ���
	RtLbsType tanGamma1 = tan(gamma1);
	if (tanGamma1 != 0) {       //ֵ��Ϊ0
		Cot1.m_real = 1.0 / tanGamma1;      //ʵ������
		D1 = Cot1 * _newTransactionFunction(2 * waveNumber * l * n * n * sin(gamma1) * sin(gamma1), tranFucntion);
	}
	else {      //ֵΪ0
		D1.m_real = 0.5 * sqrt(2 * PI * 2 * waveNumber * l * n * n);            //����D1��ʵ��
		D1.m_imag = D1.m_real;                                                  //����D1���鲿
	}

	//����ϵ��D2�ļ���
	RtLbsType tanGamma2 = tan(gamma2);
	if (tanGamma2 != 0) {
		Cot2.m_real = 1.0 / tanGamma2;      //ʵ������
		D2 = Cot2 * _newTransactionFunction(2 * waveNumber * l * n * n * sin(gamma2) * sin(gamma2), tranFucntion);
	}
	else {      //ֵΪ0
		D2.m_real = 0.5 * sqrt(2 * PI * 2 * waveNumber * l * n * n);        //����D2��ʵ��
		D2.m_imag = D2.m_real;
	}

	//����ϵ��D3�ļ���
	RtLbsType tanGamma3 = tan(gamma3);
	if (tanGamma3 != 0) {
		Cot3.m_real = 1.0 / tanGamma3;      //ʵ������
		D3 = Cot3 * _newTransactionFunction(2 * waveNumber * l * n * n * sin(gamma3) * sin(gamma3), tranFucntion);
	}
	else {      //ֵΪ0
		D3.m_real = 0.5 * sqrt(2 * PI * 2 * waveNumber * l * n * n);        //����D3��ʵ��
		D3.m_imag = D3.m_real;
	}

	//����ϵ��D4�ļ���
	RtLbsType tanGamma4 = tan(gamma4);
	if (tan(gamma4) != 0) {     //ֵ��Ϊ0
		Cot4.m_real = 1.0 / tanGamma4;      //ʵ������
		D4 = Cot4 * _newTransactionFunction(2 * waveNumber * l * n * n * sin(gamma4) * sin(gamma4), tranFucntion);
	}
	else {      //ֵΪ0
		D4.m_real = 0.5 * sqrt(2 * PI * 2 * waveNumber * l * n * n);        //����D4��ʵ��
		D4.m_imag = D4.m_real;
	}

	RnR0.perp = Rn.perp * R0.perp;      //��ϵ�����Ĵ�ֱ����
	RnR0.para = Rn.para * R0.para;      //��ϵ������ƽ�з���

	//����ϵ���ļ���
	if (a1 > PI) {      //����Ǵ���180��
		Dcof.perp = Dcon * ((RnR0.perp * D1) + D2 + Rn.perp * D3 + R0.perp * D4);       //����ϵ���Ĵ�ֱ����
		Dcof.para = Dcon * ((RnR0.para * D1) + D2 + Rn.para * D3 + R0.para * D4);       //����ϵ����ƽ�з���
	}
	else if (a1 <= (n - 1) * PI) {      //a1С�ڵ���(n-1)��
		Dcof.perp = Dcon * (D1 + RnR0.perp * D2 + R0.perp * D3 + Rn.perp * D4);         //����ϵ���Ĵ�ֱ����
		Dcof.para = Dcon * (D1 + RnR0.para * D2 + R0.para * D3 + Rn.para * D4);         //����ϵ����ƽ�з���
	}
	else {      //�������
		if (a2 <= fabs((2 * n - 1) * PI - a1)) {        //a2С�ڵ���fabs((2 * n - 1) * PI - a1))
			if (a2 >= n * PI - a1) {        //a2���ڵ���n*pi-a1
				ModifiedR = _rectifiedReflectionCoef(n * PI - a1, n * PI - a2, mat, freq);       //�����ķ���ϵ��
			}
			else {
				ModifiedR = _rectifiedReflectionCoef(a1, a2, mat, freq);     //�����ķ���ϵ��
			}
			Dcof.perp = Dcon * (D1 + D2 + ModifiedR.perp * (D3 + D4));      //����ϵ���Ĵ�ֱ����
			Dcof.para = Dcon * (D1 + D2 + ModifiedR.para * (D3 + D4));      //����ϵ����ƽ�з���
		}
		else {      //�������
			Dcof.perp = Dcon * (D1 + RnR0.perp * D2 + R0.perp * D3 + Rn.perp * D4);     //����ϵ���Ĵ�ֱ����
			Dcof.para = Dcon * (D1 + RnR0.para * D2 + R0.para * D3 + Rn.para * D4);     //����ϵ����ƽ�з���
		}
	}
	return Dcof;
}

Polarization2D Polarization3D::_specularReflectionCoef(RtLbsType theta, const Material* mat, RtLbsType freq)
{
	Polarization2D rCoef;                                       /** @brief	����ϵ��	*/
	Complex ee, ee1;                                            /** @brief	�м����	*/
	Complex snum, cnum;                                         /** @brief	�м����	*/
	ee = mat->GetComplexForm(freq);                             /** @brief  ����Խ�糣��	*/

	//���㷴��ϵ�����õ��м����
	snum.m_real = sin(theta);
	cnum.m_real = cos(theta) * cos(theta);
	ee1 = (ee - snum).Sqrt();

	//����ϵ���ļ���
	rCoef.para = (ee * snum - ee1) / (ee * snum + ee1);
	rCoef.perp = (snum - ee1) / (snum + ee1);

	return rCoef;
}

RtLbsType Polarization3D::_redefineAngle(RtLbsType n, RtLbsType phi1, RtLbsType phi2)
{
	//�ֲ���������
	RtLbsType theta;                                                /** @brief	���¶����n�淴���	*/
	if (n > 1.5) {                                                  //nֵ����1.5����
		if (phi1 < (n - 1) * PI) {                                  //phi1��(n-1)*pi ���
			if (phi2 <= PI - phi1)
				theta = fabs(HALF_PI - phi2);                       //n�淴���
			else if (phi2 >= PI)
				theta = fabs(n * PI - phi2 - HALF_PI);
			else
				theta = fabs(phi2 + (n - 2.5) * PI);
		}
		else if (phi1 > PI) {                                       //�Ƕ�ֵ����180��
			if (phi2 > PI)
				theta = fabs(n * PI - phi2 - HALF_PI);              //n�淴���
			else if (phi2 < fabs((2 * n - 1) * PI - phi1))
				theta = fabs(HALF_PI - phi2);
			else
				theta = fabs(phi2 + (n - 2.5) * PI);
		}
		else {                                                      //����
			if (phi2 >= PI)
				theta = fabs(n * PI - phi2 - HALF_PI);
			else {                                                  //�����С��180��
				if (phi2 < fabs(PI - phi1))
					theta = fabs(HALF_PI - phi2);
				else
					theta = fabs(phi2 + (n - 2.5) * PI);
			}
		}
	}
	else {                                                          //nС��1.5
		if (phi1 < (n - 1) * PI) {                                  //�����С��(n-1)��
			if (phi2 <= PI - phi1)
				theta = fabs(HALF_PI - phi2);
			else if (phi2 > PI)
				theta = fabs(n * PI - phi2 - HALF_PI);
			else
				theta = fabs(phi2 - (n - 0.5) * PI);
		}
		else if (phi1 > PI) {                                       //����Ǵ���180��
			if (phi2 < fabs((2 * n - 1) * PI - phi1))
				theta = fabs(HALF_PI - phi2);
			else if (phi2 > PI)
				theta = fabs(n * PI - phi2 - HALF_PI);
			else
				theta = fabs(phi2 - (n - 0.5) * PI);
		}
		else {                                                      //�������
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
	//�ֲ���������
	Complex ffx;            /** @brief	���ɺ���ֵ	*/
	Complex tem;            /** @brief	�м����	*/
	int index;              /** @brief	��ʱ����	*/
	RtLbsType m;            /** @brief	��ʱ����	*/
	if (x > 10) {           //�Ա����ܴ�����
		m = 1 / (x * x);    //�����м�ֵ
		ffx.m_real = 1 + m * (4.6875 * m - 0.75);       //���ɺ���ʵ��
		ffx.m_imag = (0.5 - 1.875 * m) / x;             //���ɺ����鲿
	}
	else if (x < 0.001) {       //�Ա�����Сʱ
		tem.m_real = sqrt(PI * x) - sqrt(2.0) * x * (1 + x / 3);        //���ɺ���ʵ��
		tem.m_imag = sqrt(2.0) * x * (x / 3 - 1);                       //���ɺ����鲿
		ffx = tem * Exp(QUARTER_PI + x);                                //��Ŷ�Ⱥ�����ֵ����
	}
	else {      //�������
		x = x * 10000 - 1 + 0.5;        //�����м�ֵ
		index = static_cast<int>(x);
		ffx = tranFucntion[index];
	}
	return ffx;
}

Polarization2D Polarization3D::_rectifiedReflectionCoef(RtLbsType a1, RtLbsType a2, const Material* mat, RtLbsType freq)
{
	double deltaH = 0.002;                      /** @brief	�������ֲڶ������˥�����ӡ�����ֲڶ���̫�ֲ��ı�׼ƫ��	*/
	Polarization2D rCoefRectify;                /** @brief	������ķ���ϵ��	*/
	Complex ee, ee1;                            /** @brief	�����������м����	*/
	Complex tao, cnum;                          /** @brief	�����������м����	*/

	ee = mat->GetComplexForm(freq);             /** @brief	������ʸ���Խ�����	*/

	//���㷴��ϵ�������õ��м����
	tao.m_real = 2 * sin(0.5 * a2) * sin(0.5 * a1);     /** @brief	����ʵ��	*/
	ee1 = (ee - cnum + tao * tao).Sqrt();

	//����ϵ���ļ���
	rCoefRectify.para = (ee * tao - ee1) / (ee * tao + ee1);
	rCoefRectify.perp = (tao - ee1) / (tao + ee1);
	return rCoefRectify;
}
