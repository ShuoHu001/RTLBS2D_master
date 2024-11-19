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







