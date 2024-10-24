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

AOALSResidual::AOALSResidual()
	: m_x(0.0)
	, m_y(0.0)
	, m_phi(0.0)
	, m_cosPhi(0.0)
	, m_sinPhi(0.0)
{
}

AOALSResidual::AOALSResidual(RtLbsType x, RtLbsType y, RtLbsType phi)
	: m_x(x)
	, m_y(y)
	, m_phi(phi)
{
	m_cosPhi = cos(m_phi);
	m_sinPhi = sin(m_phi);
}

AOALSResidual::AOALSResidual(const GeneralSource* source)
	: m_x(source->m_position.x)
	, m_y(source->m_position.y)
	, m_phi(source->m_sensorData.m_phi)
{
	m_cosPhi = cos(m_phi);
	m_sinPhi = sin(m_phi);
}

AOALSResidual::~AOALSResidual()
{
}

AOALSResidual& AOALSResidual::operator=(const AOALSResidual& residual)
{
	m_x = residual.m_x;
	m_y = residual.m_y;
	m_phi = residual.m_phi;
	m_cosPhi = residual.m_cosPhi;
	m_sinPhi = residual.m_sinPhi;
	return *this;
}

void AOALSResidual::Init(const GeneralSource* source)
{
	m_x = source->m_position.x;
	m_y = source->m_position.y;
	m_phi = source->m_sensorData.m_phi;
	m_cosPhi = cos(m_phi);
	m_sinPhi = sin(m_phi);
}

RtLbsType AOALSResidual::GetResidual(RtLbsType* position) const
{
	RtLbsType dx = position[0] - m_x;
	RtLbsType dy = position[1] - m_y;
	return dx * m_cosPhi - dy * m_sinPhi;
}

TDOALSResidual::TDOALSResidual()
	: m_x1(0.0)
	, m_y1(0.0)
	, m_xi(0.0)
	, m_yi(0.0)
	, m_timeDiff(0.0)
{
}

TDOALSResidual::TDOALSResidual(RtLbsType ref_x, RtLbsType ref_y, RtLbsType x, RtLbsType y, RtLbsType timeDiff)
	: m_x1(ref_x)
	, m_y1(ref_y)
	, m_xi(x)
	, m_yi(y)
	, m_timeDiff(timeDiff)
{
}

TDOALSResidual::TDOALSResidual(const GeneralSource* refSource, const GeneralSource* dataSource)
{
	m_x1 = refSource->m_position.x;
	m_y1 = refSource->m_position.y;
	m_xi = dataSource->m_position.x;
	m_yi = dataSource->m_position.y;
	m_timeDiff = dataSource->m_sensorData.m_timeDiff;
}

TDOALSResidual::TDOALSResidual(const TDOALSResidual& residual)
	: m_x1(residual.m_x1)
	, m_y1(residual.m_y1)
	, m_xi(residual.m_xi)
	, m_yi(residual.m_yi)
	, m_timeDiff(residual.m_timeDiff)
{
}

TDOALSResidual::~TDOALSResidual()
{
}

TDOALSResidual& TDOALSResidual::operator=(const TDOALSResidual& residual)
{
	m_x1 = residual.m_x1;
	m_y1 = residual.m_y1;
	m_xi = residual.m_xi;
	m_yi = residual.m_yi;
	m_timeDiff = residual.m_timeDiff;
	return *this;
}

void TDOALSResidual::Init(const GeneralSource* refSource, const GeneralSource* dataSource)
{
	m_x1 = refSource->m_position.x;
	m_y1 = refSource->m_position.y;
	m_xi = dataSource->m_position.x;
	m_yi = dataSource->m_position.y;
	m_timeDiff = dataSource->m_sensorData.m_timeDiff;
}

RtLbsType TDOALSResidual::GetResidual(RtLbsType* position) const
{
	RtLbsType x = position[0];							/** @brief	Ԥ��� x����	*/
	RtLbsType y = position[1];							/** @brief	Ԥ��� y����	*/
	RtLbsType d1 = std::sqrt((x - m_x1) * (x - m_x1) + (y - m_y1) * (y - m_y1));
	RtLbsType di = std::sqrt((x - m_xi) * (x - m_xi) + (y - m_yi) * (y - m_yi));
	return (di - d1) - m_timeDiff * LIGHT_VELOCITY_AIR;
}

TDOAWLSResidual::TDOAWLSResidual()
	: m_x1(0.0)
	, m_y1(0.0)
	, m_xi(0.0)
	, m_yi(0.0)
	, m_timeDiff(0.0)
	, m_weight(0.0)
{
}

TDOAWLSResidual::TDOAWLSResidual(RtLbsType ref_x, RtLbsType ref_y, RtLbsType x, RtLbsType y, RtLbsType timeDiff, RtLbsType weight)
	: m_x1(ref_x)
	, m_y1(ref_y)
	, m_xi(x)
	, m_yi(y)
	, m_timeDiff(timeDiff)
	, m_weight(weight)
{
}

TDOAWLSResidual::TDOAWLSResidual(const GeneralSource* refSource, const GeneralSource* dataSource)
{
	m_x1 = refSource->m_position.x;
	m_y1 = refSource->m_position.y;
	m_xi = dataSource->m_position.x;
	m_yi = dataSource->m_position.y;
	m_timeDiff = dataSource->m_sensorData.m_timeDiff;
	m_weight = dataSource->m_weight;
}

TDOAWLSResidual::TDOAWLSResidual(const TDOAWLSResidual& residual)
	: m_x1(residual.m_x1)
	, m_y1(residual.m_y1)
	, m_xi(residual.m_xi)
	, m_yi(residual.m_yi)
	, m_timeDiff(residual.m_timeDiff)
	, m_weight(residual.m_weight)
{
}

TDOAWLSResidual::~TDOAWLSResidual()
{
}

TDOAWLSResidual& TDOAWLSResidual::operator=(const TDOAWLSResidual& residual)
{
	m_x1 = residual.m_x1;
	m_y1 = residual.m_y1;
	m_xi = residual.m_xi;
	m_yi = residual.m_yi;
	m_timeDiff = residual.m_timeDiff;
	m_weight = residual.m_weight;
	return *this;
}

void TDOAWLSResidual::Init(const GeneralSource* refSource, const GeneralSource* dataSource)
{
	m_x1 = refSource->m_position.x;
	m_y1 = refSource->m_position.y;
	m_xi = dataSource->m_position.x;
	m_yi = dataSource->m_position.y;
	m_timeDiff = dataSource->m_sensorData.m_timeDiff;
	m_weight = dataSource->m_weight;
}

RtLbsType TDOAWLSResidual::GetResidual(RtLbsType* position) const
{
	RtLbsType x = position[0];							/** @brief	Ԥ��� x����	*/
	RtLbsType y = position[1];							/** @brief	Ԥ��� y����	*/
	RtLbsType d1 = std::sqrt((x - m_x1) * (x - m_x1) + (y - m_y1) * (y - m_y1));
	RtLbsType di = std::sqrt((x - m_xi) * (x - m_xi) + (y - m_yi) * (y - m_yi));
	return ((di - d1) - m_timeDiff * LIGHT_VELOCITY_AIR) * m_weight;
}

void TDOAWLSResidual::SetWeight(RtLbsType weight)
{
	m_weight = weight;
}

RtLbsType TDOAWLSResidual::GetWeight() const
{
	return m_weight;
}

TOALSResidual::TOALSResidual()
	: m_x(0.0)
	, m_y(0.0)
	, m_time(0.0)
{
}

TOALSResidual::TOALSResidual(RtLbsType x, RtLbsType y, RtLbsType delay)
	: m_x(x)
	, m_y(y)
	, m_time(delay)
{
}

TOALSResidual::TOALSResidual(const GeneralSource* source)
	: m_x(source->m_position.x)
	, m_y(source->m_position.y)
	, m_time(source->m_sensorData.m_time)
{
}

TOALSResidual::~TOALSResidual()
{
}

TOALSResidual& TOALSResidual::operator=(const TOALSResidual& r)
{
	m_x = r.m_x;
	m_y = r.m_y;
	m_time = r.m_time;
	return *this;
}

void TOALSResidual::Init(const GeneralSource* source)
{
	m_x = source->m_position.x;
	m_y = source->m_position.y;
	m_time = source->m_sensorData.m_time;
}

RtLbsType TOALSResidual::GetTOAResidual(RtLbsType* position) const
{
	RtLbsType dx = position[0] - m_x;
	RtLbsType dy = position[1] - m_y;
	RtLbsType distance = sqrt(dx * dx + dy * dy);
	RtLbsType calDelay = distance / LIGHT_VELOCITY_AIR;
	return (calDelay - m_time)*1e9;
}

TOAWLSResidual::TOAWLSResidual()
	: m_x(0.0)
	, m_y(0.0)
	, m_time(0.0)
	, m_weight(0.0)
{
}

TOAWLSResidual::TOAWLSResidual(RtLbsType x, RtLbsType y, RtLbsType delay, RtLbsType weight)
	: m_x(x)
	, m_y(y)
	, m_time(delay)
	, m_weight(weight)
{
}

TOAWLSResidual::TOAWLSResidual(const GeneralSource* source)
	: m_x(source->m_position.x)
	, m_y(source->m_position.y)
	, m_time(source->m_sensorData.m_time)
	, m_weight(source->m_weight)
{
}

TOAWLSResidual::TOAWLSResidual(const TOAWLSResidual& r)
	: m_x(r.m_x)
	, m_y(r.m_y)
	, m_time(r.m_time)
	, m_weight(r.m_weight)
{
}

TOAWLSResidual::~TOAWLSResidual()
{
}

TOAWLSResidual& TOAWLSResidual::operator=(const TOAWLSResidual& r)
{
	m_x = r.m_x;
	m_y = r.m_y;
	m_time = r.m_time;
	m_weight = r.m_weight;
	return *this;
}

void TOAWLSResidual::Init(const GeneralSource* source)
{
	m_x = source->m_position.x;
	m_y = source->m_position.y;
	m_time = source->m_sensorData.m_time;
	m_weight = source->m_weight;
}


RtLbsType TOAWLSResidual::GetResidual(RtLbsType* position) const
{
	RtLbsType dx = position[0] - m_x;
	RtLbsType dy = position[1] - m_y;
	RtLbsType distance = sqrt(dx * dx + dy * dy);
	RtLbsType calDelay = distance / LIGHT_VELOCITY_AIR;
	return m_weight * (calDelay - m_time)*1e9;
}

RtLbsType TOAWLSResidual::GetWeight() const
{
	return m_weight;
}

void TOAWLSResidual::SetWeight(RtLbsType weight)
{
	m_weight = weight;
}

