#include "lbsresidual.h"

AOAWLSResidual::AOAWLSResidual()
	: m_x(0.0)
	, m_y(0.0)
	, m_phi(0.0)
	, m_weight(0.0)
	, m_cosPhi(0.0)
	, m_sinPhi(0.0)
{
}

AOAWLSResidual::AOAWLSResidual(RtLbsType x, RtLbsType y, RtLbsType phi, RtLbsType weight)
	: m_x(x)
	, m_y(y)
	, m_phi(phi)
	, m_weight(weight)
{
	m_cosPhi = cos(m_phi);
	m_sinPhi = sin(m_phi);
}

AOAWLSResidual::AOAWLSResidual(const GeneralSource* source)
	: m_x(source->m_position.x)
	, m_y(source->m_position.y)
	, m_phi(source->m_sensorData.m_phi)
	, m_weight(source->m_weight)
{
	m_cosPhi = cos(m_phi);
	m_sinPhi = sin(m_phi);
}

AOAWLSResidual::AOAWLSResidual(const AOAWLSResidual& residual)
	: m_x(residual.m_x)
	, m_y(residual.m_y)
	, m_phi(residual.m_phi)
	, m_weight(residual.m_weight)
	, m_cosPhi(residual.m_cosPhi)
	, m_sinPhi(residual.m_sinPhi)
{
}

AOAWLSResidual::~AOAWLSResidual()
{
}

AOAWLSResidual& AOAWLSResidual::operator=(const AOAWLSResidual& residual)
{
	m_x = residual.m_x;
	m_y = residual.m_y;
	m_phi = residual.m_phi;
	m_weight = residual.m_weight;
	m_cosPhi = residual.m_cosPhi;
	m_sinPhi = residual.m_sinPhi;
	return *this;
		
}

void AOAWLSResidual::Init(const GeneralSource* source)
{
	m_x = source->m_position.x;
	m_y = source->m_position.y;
	m_phi = source->m_sensorData.m_phi;
	m_weight = source->m_weight;
	m_cosPhi = cos(m_phi);
	m_sinPhi = sin(m_phi);
}

RtLbsType AOAWLSResidual::GetWeight() const
{
	return m_weight;
}

RtLbsType AOAWLSResidual::GetResidual(RtLbsType* position) const
{
	RtLbsType dx = position[0] - m_x;
	RtLbsType dy = position[1] - m_y;
	return (dx * m_cosPhi - dy * m_sinPhi) * m_weight;
}

void AOAWLSResidual::SetWeight(RtLbsType weight)
{
	m_weight = weight;
}
