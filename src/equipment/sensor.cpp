#include "sensor.h"

Sensor::Sensor()
	: m_isValid(true)
	, m_id(-1)
	, m_antenna(nullptr)
	, m_interLoss(0.0)
	, m_attachGain(0.0)
	, m_phiErrorSTD(0.1)
	, m_timeErrorSTD(5.0e-9)
{
}

Sensor::Sensor(const Sensor& sensor)
	: m_isValid(sensor.m_isValid)
	, m_id(sensor.m_id)
	, m_position(sensor.m_position)
	, m_antenna(sensor.m_antenna)
	, m_interLoss(sensor.m_interLoss)
	, m_attachGain(sensor.m_attachGain)
	, m_phiErrorSTD(sensor.m_phiErrorSTD)
	, m_timeErrorSTD(sensor.m_timeErrorSTD)
	, m_sensorDataCollection(sensor.m_sensorDataCollection)
{
}

Sensor::Sensor(const SensorConfig& config, AntennaLibrary* antLibrary)
{
	m_isValid = true;
	m_id = -1;
	m_antenna = antLibrary->GetAntenna(config.m_antId);
	m_position = config.m_position;
	m_interLoss = config.m_insertLoss;
	m_attachGain = config.m_attachGain;
	m_phiErrorSTD = config.m_phiErrorSTD;
	m_timeErrorSTD = config.m_timeErrorSTD;
	m_sensorDataCollection.Init(config.m_sensorDataFileName);
}

Sensor::~Sensor()
{
}

Sensor& Sensor::operator=(const Sensor& sensor)
{
	m_isValid = sensor.m_isValid;
	m_id = sensor.m_id;
	m_position = sensor.m_position;
	m_antenna = sensor.m_antenna;
	m_interLoss = sensor.m_interLoss;
	m_attachGain = sensor.m_attachGain;
	m_phiErrorSTD = sensor.m_phiErrorSTD;
	m_timeErrorSTD = sensor.m_timeErrorSTD;
	return *this;
}

RtLbsType Sensor::GetGain() const
{
	return m_attachGain + m_antenna->m_gain - m_interLoss;
}

Point2D Sensor::GetPosition2D() const
{
	return Point2D(m_position.x, m_position.y);
}
