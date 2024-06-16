#include "angularspectrum.h"

AngularSpectrum::AngularSpectrum()
	: m_size(360)
{
}

AngularSpectrum::AngularSpectrum(const AngularSpectrum& spectrum)
	: m_size(spectrum.m_size)
	, m_units(spectrum.m_units)
{
}

AngularSpectrum::~AngularSpectrum()
{
}

AngularSpectrum& AngularSpectrum::operator=(const AngularSpectrum& spectrum)
{
	m_size = spectrum.m_size;
	m_units = spectrum.m_units;
	return *this;
}

RtLbsType AngularSpectrum::CalUnitAngle() const
{
	return TWO_PI / m_size;
}

bool AngularSpectrum::InitFromFile(const std::string& filename)
{
	return false;
}

RtLbsType AngularSpectrum::GetReceivedPower(RtLbsType theta)
{
	return RtLbsType();
}

