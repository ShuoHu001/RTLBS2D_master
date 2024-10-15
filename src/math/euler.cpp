#include "euler.h"
#include "vector3d.h"


Euler::Euler()
	: m_isRadian(true)
	, m_yaw(0.0)
	, m_pitch(0.0)
	, m_roll(0.0)
{
}

Euler::Euler(RtLbsType yaw, RtLbsType pitch, RtLbsType roll, bool isRadian)
	: m_isRadian(isRadian)
	, m_yaw(yaw)
	, m_pitch(pitch)
	, m_roll(roll)
{
}

Euler::Euler(const Euler& euler)
	: m_isRadian(euler.m_isRadian)
	, m_yaw(euler.m_yaw)
	, m_pitch(euler.m_pitch)
	, m_roll(euler.m_roll)
{
}

Euler::~Euler()
{
}

Euler& Euler::operator=(const Euler& e)
{
	m_isRadian = e.m_isRadian;
	m_pitch = e.m_pitch;
	m_yaw = e.m_yaw;
	m_roll = e.m_roll;
	return *this;
}

Euler Euler::operator+(const Euler& e) const
{
	return Euler(m_yaw + e.m_yaw, m_pitch + e.m_pitch, m_roll + e.m_roll, m_isRadian);
}

Euler& Euler::operator+=(const Euler& e)
{
	m_yaw += e.m_yaw;
	m_pitch += e.m_pitch;
	m_roll += e.m_roll;
	return *this;
}

Euler Euler::operator-(const Euler& e) const
{
	return Euler(m_yaw - e.m_yaw, m_pitch - e.m_pitch, m_roll - e.m_roll, m_isRadian);
}

Euler& Euler::operator-=(const Euler& e)
{
	m_yaw -= e.m_yaw;
	m_pitch -= e.m_pitch;
	m_roll -= e.m_roll;
	return *this;
}

Euler Euler::operator*(const RtLbsType& t) const
{
	return Euler(m_yaw * t, m_pitch * t, m_roll * t, m_isRadian);
}

Euler& Euler::operator*=(const RtLbsType& t)
{
	m_yaw *= t;
	m_pitch *= t;
	m_roll *= t;
	return *this;
}

Euler Euler::operator/(const RtLbsType& t) const
{
	return Euler(m_yaw / t, m_pitch / t, m_roll / t, m_isRadian);
}

Euler& Euler::operator/=(const RtLbsType& t)
{
	m_yaw /= t;
	m_pitch /= t;
	m_roll /= t;
	return *this;
}

bool Euler::operator==(const Euler& e) const
{
	if (m_yaw == e.m_yaw && m_pitch == e.m_pitch && m_roll == e.m_roll)
		return true;
	return false;
}

bool Euler::operator!=(const Euler& e) const
{
	return !(*this == e);
}

RtLbsType Euler::operator[](unsigned id) const
{
	switch (id)
	{
	case 0:
		return m_yaw;
	case 1:
		return m_pitch;
	case 2:
		return m_roll;
	default:
		return 0;
	}
}

RtLbsType& Euler::operator[](unsigned id)
{
	switch (id)
	{
	case 0:
		return m_yaw;
	case 1:
		return m_pitch;
	case 2:
		return m_roll;
	default:
		LOG_ERROR << "out of index" << CRASH;
		return m_yaw;
	}
}

bool Euler::IsZero() const
{
	if (m_yaw == 0.0 && m_pitch == 0.0 && m_roll == 0.0)
		return true;
	return false;
}

Euler& Euler::ConvertToDegree()
{
	if (m_isRadian) {
		m_yaw = m_yaw / PI * 180;
		m_pitch = m_pitch / PI * 180;
		m_roll = m_roll / PI * 180;
	}
	return *this;
}

Euler& Euler::ConvertToRadian()
{
	if (!m_isRadian) {
		m_yaw = m_yaw / 180 * PI;
		m_pitch = m_pitch / 180 * PI;
		m_roll = m_roll / 180 * PI;
	}
	return *this;
}

Vector3D Euler::CalCartesianCoordinate(RtLbsType t) const
{
	RtLbsType phi = m_pitch;					/** @brief	·½Î»½Ç	*/
	RtLbsType theta = m_roll;					/** @brief	¸©Ñö½Ç	*/
	if (!m_isRadian) {
		phi = m_pitch / 180 / PI;
		theta = m_roll / 180 / PI;
	}
	RtLbsType x = t * sin(theta) * cos(phi);
	RtLbsType y = t * sin(theta) * sin(phi);
	RtLbsType z = t * cos(theta);
	return Vector3D(x, y, z);
}

void Euler::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_EULER_ISRADIAN.c_str()); writer.Bool(m_isRadian);
	writer.Key(KEY_EULER_YAW.c_str()); writer.Double(m_yaw);
	writer.Key(KEY_EULER_PITCH.c_str()); writer.Double(m_pitch);
	writer.Key(KEY_EULER_ROLL.c_str()); writer.Double(m_roll);
	writer.EndObject();
}

bool Euler::Deserialize(const rapidjson::Value& value)
{
	if (value.IsObject()) {
		if (value.HasMember(KEY_EULER_ISRADIAN.c_str()) &&
			value.HasMember(KEY_EULER_YAW.c_str()) &&
			value.HasMember(KEY_EULER_PITCH.c_str()) &&
			value.HasMember(KEY_EULER_ROLL.c_str())) {
			const rapidjson::Value& isRadianValue = value[KEY_EULER_ISRADIAN.c_str()];
			const rapidjson::Value& yawValue = value[KEY_EULER_YAW.c_str()];
			const rapidjson::Value& pitchValue = value[KEY_EULER_PITCH.c_str()];
			const rapidjson::Value& rollValue = value[KEY_EULER_ROLL.c_str()];
			if (isRadianValue.IsBool() &&
				yawValue.IsDouble() &&
				pitchValue.IsDouble() &&
				rollValue.IsDouble()) {
				m_isRadian = isRadianValue.GetBool();
				m_yaw = yawValue.GetDouble();
				m_pitch = pitchValue.GetDouble();
				m_roll = rollValue.GetDouble();
				return true;
			}
		}
	}
	return false;
}

inline Vector3D CalCartesianCoordinate(const Euler& posture, RtLbsType t) {
	RtLbsType phi = posture.m_pitch;					/** @brief	·½Î»½Ç	*/
	RtLbsType theta = posture.m_roll;					/** @brief	¸©Ñö½Ç	*/
	if (!posture.m_isRadian) {
		phi = posture.m_pitch / 180 / PI;
		theta = posture.m_roll / 180 / PI;
	}
	RtLbsType x = t * sin(theta) * cos(phi);
	RtLbsType y = t * sin(theta) * sin(phi);
	RtLbsType z = t * cos(theta);
	return Vector3D(x, y, z);
}
