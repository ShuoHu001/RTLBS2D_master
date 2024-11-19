#include "polarization2d.h"

Polarization2D::Polarization2D()
{
}

Polarization2D::Polarization2D(Complex perpE, Complex paraE)
	: perp(perpE)
	, para(paraE)
{
}

Polarization2D::Polarization2D(Polarization2D& p)
	: perp(p.perp)
	, para(p.para)
{
}

Polarization2D::~Polarization2D()
{
}

Polarization2D& Polarization2D::operator=(const Polarization2D& p)
{
	perp = p.perp;
	para = p.para;
	return *this;
}

Polarization2D Polarization2D::operator+(const Polarization2D& p) const
{
	Polarization2D reVal;
	reVal.perp = perp + p.perp;
	reVal.para = para + p.para;
	return reVal;
}

Polarization2D& Polarization2D::operator+=(const Polarization2D& p)
{
	perp += p.perp;
	para += p.para;
	return *this;
}

Polarization2D Polarization2D::operator-(const Polarization2D& p) const
{
	Polarization2D reVal;
	reVal.perp = perp - p.perp;
	reVal.para = para - p.para;
	return reVal;
}

Polarization2D& Polarization2D::operator-=(const Polarization2D& p)
{
	perp -= p.perp;
	para -= p.para;
	return *this;
}

Polarization2D Polarization2D::operator*(const RtLbsType num) const
{
	Polarization2D reVal;
	reVal.perp = perp * num;
	reVal.para = para * num;
	return reVal;
}

Polarization2D& Polarization2D::operator*=(const RtLbsType num)
{
	perp *= num;
	para *= num;
	return *this;
}

Polarization2D Polarization2D::operator/(const RtLbsType num) const
{
	Polarization2D reVal;
	reVal.perp = perp / num;
	reVal.para = para / num;
	return reVal;
}

Polarization2D& Polarization2D::operator/=(const RtLbsType num)
{
	perp /= num;
	para /= num;
	return *this;
}

Complex Polarization2D::operator[](const unsigned id) const
{
	switch (id) {
	case 0:
		return perp;
	case 1:
		return para;
	default:
		return perp;
	}
}

Complex& Polarization2D::operator[](const unsigned id)
{
	switch (id)
	{
	case 0:
		return perp;
	case 1:
		return para;
	default:
		return perp;
	}
}
