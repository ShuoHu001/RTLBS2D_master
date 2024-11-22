#include "toa_residual.h"

TOAResidual::TOAResidual()
	: m_x(0.0)
	, m_y(0.0)
	, m_time(0.0)
	, m_weight(1.0)
{
}

TOAResidual::TOAResidual(const GeneralSource* source, RtLbsType weight)
	: m_x(source->m_position.x)
	, m_y(source->m_position.y)
	, m_time(source->m_sensorData.m_time)
	, m_weight(weight)
{
}

TOAResidual::TOAResidual(const TOAResidual& r)
	: m_x(r.m_x)
	, m_y(r.m_y)
	, m_time(r.m_time)
	, m_weight(r.m_weight)
{
}

TOAResidual::~TOAResidual()
{
}

TOAResidual& TOAResidual::operator=(const TOAResidual& r)
{
	m_x = r.m_x;
	m_y = r.m_y;
	m_time = r.m_time;
	m_weight = r.m_weight;
	return *this;
}

void TOAResidual::Init(const GeneralSource* source, RtLbsType weight)
{
	m_x = source->m_position.x;
	m_y = source->m_position.y;
	m_time = source->m_sensorData.m_time;
	m_weight = weight;
}


double TOAResidual::GetResidual(const RtLbsType* position) const
{
	double dx = position[0] - m_x;
	double dy = position[1] - m_y;
	double distance = sqrt(dx * dx + dy * dy);
	double calDelay = distance / LIGHT_VELOCITY_AIR;
	return (calDelay - m_time) * 1e9 * m_weight;
}

RtLbsType TOAResidual::GetWeight() const
{
	return m_weight;
}

void TOAResidual::SetWeight(RtLbsType weight)
{
	m_weight = weight;
}