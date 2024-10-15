#include "transmitter.h"

Transmitter::Transmitter()
	: m_isValid(true)
	, m_id(-1)
	, m_antenna(nullptr)
	, m_velocity(0.0)
	, m_power(0.0)
	, m_interLoss(0.0)
	, m_attachGain(0.0)
{
}

Transmitter::Transmitter(const TransmitterConfig& config, AntennaLibrary* antLibrary)
{
	m_isValid = true;
	m_id = -1;
	m_antenna = antLibrary->GetAntenna(config.m_antId);
	m_position = config.m_position;
	m_posture = config.m_posture;
	m_velocity = config.m_velocity;
	m_power = config.m_power;
	m_interLoss = config.m_interLoss;
	m_attachGain = config.m_attachGain;
}

Transmitter::Transmitter(const Transmitter& tr)
	: m_isValid(tr.m_isValid)
	, m_id(tr.m_id)
	, m_antenna(tr.m_antenna)
	, m_position(tr.m_position)
	, m_posture(tr.m_posture)
	, m_velocity(tr.m_velocity)
	, m_power(tr.m_power)
	, m_interLoss(tr.m_interLoss)
	, m_attachGain(tr.m_attachGain)
{
}

Transmitter::~Transmitter()
{
	delete m_antenna;
}

Transmitter& Transmitter::operator=(const Transmitter& tr)
{
	m_isValid = tr.m_isValid;
	m_id = tr.m_id;
	m_antenna = tr.m_antenna;
	m_position = tr.m_position;
	m_posture = tr.m_posture;
	m_power = tr.m_power;
	m_interLoss = tr.m_interLoss;
	m_attachGain = tr.m_attachGain;
	return *this;
}

RtLbsType Transmitter::GetGain() const
{
	return m_attachGain + m_antenna->m_gain - m_interLoss;
}

Point3D Transmitter::GetPosition3D() const
{
	return m_position;
}

Point2D Transmitter::GetPosition2D() const
{
	return Point2D(m_position.x, m_position.y);
}
