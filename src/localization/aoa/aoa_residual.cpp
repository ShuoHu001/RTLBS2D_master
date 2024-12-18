#include "aoa_residual.h"

AOAResidual::AOAResidual()
	: m_x(0.0)
	, m_y(0.0)
	, m_phi(0.0)
	, m_weight(1.0)
	, m_cosPhi(0.0)
	, m_sinPhi(0.0)
{
}

AOAResidual::AOAResidual(const GeneralSource* source, RtLbsType weight)
	: m_x(source->m_position.x)
	, m_y(source->m_position.y)
	, m_phi(source->m_sensorData.m_phi)
	, m_weight(weight)
{
	m_cosPhi = cos(m_phi);
	m_sinPhi = sin(m_phi);
}

AOAResidual::AOAResidual(const AOAResidual& residual)
	: m_x(residual.m_x)
	, m_y(residual.m_y)
	, m_phi(residual.m_phi)
	, m_weight(residual.m_weight)
	, m_cosPhi(residual.m_cosPhi)
	, m_sinPhi(residual.m_sinPhi)
{
}

AOAResidual::~AOAResidual()
{
}

AOAResidual& AOAResidual::operator=(const AOAResidual& residual)
{
	m_x = residual.m_x;
	m_y = residual.m_y;
	m_phi = residual.m_phi;
	m_weight = residual.m_weight;
	m_cosPhi = residual.m_cosPhi;
	m_sinPhi = residual.m_sinPhi;
	return *this;
}

void AOAResidual::Init(const GeneralSource* source, RtLbsType weight)
{
	m_x = source->m_position.x;
	m_y = source->m_position.y;
	m_phi = source->m_sensorData.m_phi;
	m_weight = weight;
	m_cosPhi = cos(m_phi);
	m_sinPhi = sin(m_phi);
}

double AOAResidual::GetResidual(const double* position) const
{
	double dx = position[0] - m_x;
	double dy = position[1] - m_y;
	return (dx * m_sinPhi - dy * m_cosPhi) * m_weight;
}

RtLbsType AOAResidual::GetWeight() const
{
	return m_weight;
}

void AOAResidual::SetWeight(RtLbsType weight)
{
	m_weight = weight;
}
