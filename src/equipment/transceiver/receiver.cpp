#include "receiver.h"

Receiver::Receiver()
	: m_isValid(true)
	, m_id(-1)
	, m_antenna(nullptr)
	, m_velocity(0.0)
	, m_interLoss(0.0)
	, m_attachGain(0.0)
	, m_powerThreshold(0.0)
	, m_angularThreshold(ONE_DEGEREE)
	, m_delayThreshold(5.0e-9)
{
}

Receiver::Receiver(const Receiver& re)
	: m_isValid(re.m_isValid)
	, m_id(re.m_id)
	, m_antenna(re.m_antenna)
	, m_position(re.m_position)
	, m_posture(re.m_posture)
	, m_velocity(re.m_velocity)
	, m_interLoss(re.m_interLoss)
	, m_attachGain(re.m_attachGain)
	, m_powerThreshold(re.m_powerThreshold)
	, m_angularThreshold(re.m_angularThreshold)
	, m_delayThreshold(re.m_delayThreshold)
{
}

Receiver::Receiver(const ReceiverUnitConfig& config, AntennaLibrary* antLibrary)
{
	m_isValid = true;
	m_id = -1;
	m_antenna = antLibrary->GetAntenna(config.m_antId);
	m_position = config.m_position;
	m_posture = config.m_posture;
	m_velocity = config.m_velocity;
	m_interLoss = config.m_insertLoss;
	m_attachGain = config.m_attachGain;
	m_powerThreshold = config.m_powerShreshold;
	m_angularThreshold = config.m_angularThreshold * ONE_DEGEREE;		//将角度转换为弧度
	m_delayThreshold = config.m_delayThreshold;
}

Receiver::~Receiver()
{
	delete m_antenna;
}


RtLbsType Receiver::GetGain() const
{
	return m_attachGain + m_antenna->m_gain - m_interLoss;
}

Point3D Receiver::GetPosition3D() const
{
	return m_position;
}

Point2D Receiver::GetPosition2D() const
{
	return Point2D(m_position.x, m_position.y);
}

Receiver& Receiver::operator=(const Receiver& re)
{
	m_isValid = re.m_isValid;
	m_id = re.m_id;
	m_antenna = re.m_antenna;
	m_position = re.m_position;
	m_posture = re.m_posture;
	m_velocity = re.m_velocity;
	m_interLoss = re.m_interLoss;
	m_attachGain = re.m_attachGain;
	m_powerThreshold = re.m_powerThreshold;
	m_angularThreshold = re.m_angularThreshold;
	m_delayThreshold = re.m_delayThreshold;
	return *this;
}
