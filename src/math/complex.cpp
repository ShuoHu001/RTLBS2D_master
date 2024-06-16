#include "complex.h"
#include "physical/radiowave/polarization3d.h"

Complex::Complex()
	: m_real(0.0)
	, m_imag(0.0)
{
}

Complex::Complex(RtLbsType real, RtLbsType imag)
	: m_real(real)
	, m_imag(imag)
{
}

Complex::Complex(const Complex& c)
	: m_real(c.m_real)
	, m_imag(c.m_imag)
{
}

Complex::~Complex()
{
}

Complex& Complex::operator=(const Complex& c)
{
	m_real = c.m_real;
	m_imag = c.m_imag;
	return *this;
}

Complex Complex::operator+(const Complex& c) const
{
	return Complex(m_real + c.m_real, m_imag + c.m_imag);
}

Complex& Complex::operator+=(const Complex& c)
{
	this->m_real += c.m_real;
	this->m_imag += c.m_imag;
	return *this;
}

Complex Complex::operator-(const Complex& c) const
{
	return Complex(m_real - c.m_real, m_imag - c.m_imag);
}

Complex& Complex::operator-=(const Complex& c)
{
	this->m_real -= c.m_real;
	this->m_imag -= c.m_imag;
	return *this;
}

Complex Complex::operator*(const Complex& c) const
{
	Complex reVal;
	reVal.m_real = m_real * c.m_real - m_imag * c.m_imag;
	reVal.m_imag = m_real * c.m_imag + c.m_real * m_imag;
	return reVal;
}

Complex& Complex::operator*=(const Complex& c)
{
	this->m_real = m_real * c.m_real - m_imag * c.m_imag;
	this->m_imag = m_real * c.m_imag + c.m_real * m_imag;
	return *this;
}

Complex Complex::operator*(const RtLbsType num) const
{
	Complex reVal;
	reVal.m_real = m_real * num;
	reVal.m_imag = m_imag * num;
	return reVal;
}

Complex& Complex::operator*=(const RtLbsType num)
{
	m_real *= num;
	m_imag *= num;
	return *this;
}

Complex Complex::operator/(const Complex& c) const
{ 
	RtLbsType mValue2 = c.MValue2(); //c 的模值平方
	Complex reVal;
	reVal.m_real = (m_real * c.m_real + m_imag * c.m_imag) / mValue2;
	reVal.m_imag = (c.m_real * m_imag - m_real * c.m_imag) / mValue2;
	return reVal;
}

Complex& Complex::operator/=(const Complex& c)
{
	RtLbsType mValue2 = c.MValue2(); //c 的模值平方
	this->m_real = (m_real * c.m_real + m_imag * c.m_imag) / mValue2;
	this->m_imag = (c.m_real * m_imag - m_real * c.m_imag) / mValue2;
	return *this;
}

Complex Complex::operator/(const RtLbsType num) const
{
	Complex reVal;
	reVal.m_real = m_real / num;
	reVal.m_imag = m_imag / num;
	return reVal;
}

Complex& Complex::operator/=(const RtLbsType num)
{
	m_real /= num;
	m_imag /= num;
	return *this;
}

RtLbsType Complex::operator[](unsigned id) const
{
	switch (id){
	case 0:
		return m_real;
	case 1:
		return m_imag;
	default:
		return m_real;
	}
}

RtLbsType& Complex::operator[](unsigned id)
{
	switch (id) {
	case 0:
		return m_real;
	case 1:
		return m_imag;
	default:
		return m_real;
	}
}

bool Complex::operator==(const Complex& c) const
{
	if (m_real == c.m_real && m_imag == c.m_imag)
		return true;
	return false;
}

bool Complex::operator!=(const Complex& c) const
{
	return !(*this == c);
}

Polarization3D Complex::operator*(const Vector3D& v) const
{
	Polarization3D reVal;
	reVal.ex.m_real = v.x * m_real;
	reVal.ex.m_imag = v.x * m_imag;
	reVal.ey.m_real = v.y * m_real;
	reVal.ey.m_imag = v.y * m_imag;
	reVal.ez.m_real = v.z * m_real;
	reVal.ez.m_imag = v.z * m_imag;
	return reVal;
}

void Complex::Init(RtLbsType amplitude, RtLbsType phase)
{
	m_real = amplitude * cos(phase);
	m_imag = amplitude * sin(phase);
}

RtLbsType Complex::MValue() const
{
	return sqrt(m_real * m_real + m_imag * m_imag);
}

RtLbsType Complex::MValue2() const
{
	return m_real * m_real + m_imag * m_imag;
}

Complex Complex::Sqrt()
{
	Complex reVal;
	RtLbsType mValue = MValue();
	if (mValue != 0) {
		RtLbsType theta = GetRad();
		reVal.m_real = sqrt(mValue) * cos(0.5 * theta);
		reVal.m_imag = sqrt(mValue) * sin(0.5 * theta);
	}
	return reVal;
}

Complex Complex::Reciprocal()
{
	Complex reVal;
	RtLbsType mValue2 = MValue2();
	reVal.m_real = m_real / mValue2;
	reVal.m_imag = -m_imag / mValue2;
	return reVal;
}

RtLbsType Complex::GetRad() const
{
	return static_cast<RtLbsType>(atan2(m_imag, m_real));
}

bool Complex::IsZero() const
{
	if (m_real == 0.0 && m_imag == 0.0)
		return true;
	return false;
}
