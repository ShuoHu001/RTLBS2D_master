#include "sensordata.h"

SensorData::SensorData()
	: m_id(-1)
	, m_sensorId(-1)
	, m_time(0.0)
	, m_timeDiff(0.0)
	, m_phiDegree(0.0)
	, m_phi(0.0)
	, m_power(0.0)
	, m_powerLin(0.0)
{
}

SensorData::SensorData(const SensorData& data)
	: m_id(data.m_id)
	, m_sensorId(data.m_sensorId)
	, m_time(data.m_time)
	, m_timeDiff(data.m_timeDiff)
	, m_phiDegree(data.m_phiDegree)
	, m_phi(data.m_phi)
	, m_power(data.m_power)
	, m_powerLin(data.m_powerLin)
{
}

SensorData::~SensorData()
{
}

SensorData SensorData::operator=(const SensorData& data)
{
	m_id = data.m_id;
	m_sensorId = data.m_sensorId;
	m_time = data.m_time;
	m_timeDiff = data.m_timeDiff;
	m_phiDegree = data.m_phiDegree;
	m_phi = data.m_phi;
	m_power = data.m_power;
	m_powerLin = data.m_powerLin;
	return *this;
}

bool SensorData::operator<(const SensorData& data)
{
	return m_power > data.m_power;
}

RtLbsType SensorData::DistanceAOA2D(const SensorData& data) const
{
	RtLbsType dPhi = abs(m_phi - data.m_phi);
	if (dPhi > PI) {							//角度差值小于PI
		dPhi = TWO_PI - dPhi;
	}
	return dPhi;
}

RtLbsType SensorData::DistanceDelay(const SensorData& data) const
{
	RtLbsType dDelay = abs(m_time - data.m_time);
	return dDelay;
}

Vector2D SensorData::GetDirection() const
{
	//默认定义传感器接收到的方位角是与水平X轴方向的夹角
	Vector2D dir(cos(m_phi), sin(m_phi));
	dir.Normalize();
	return dir;
}

void SensorData::AddSimulationError(RtLbsType phiErrorSigma, RtLbsType timeErrorSigma, RtLbsType powerErrorSigma)
{
	RtLbsType phiError = NORMDOUBLE(0, phiErrorSigma);
	RtLbsType timeError = NORMDOUBLE(0, timeErrorSigma);
	RtLbsType powerError = NORMDOUBLE(0, powerErrorSigma);
	m_time += timeError;
	m_timeDiff += timeError;
	m_phi += phiError;
	m_power += powerError;
	m_phiDegree += phiError / ONE_DEGEREE;
	//修正负值角度
	if (m_phi < 0) {
		m_phi += TWO_PI;
		m_phiDegree += 360;
	}
	//修正超值角度
	if (m_phi > TWO_PI) {
		m_phi -= TWO_PI;
		m_phiDegree -= 360;
	}
}


void SensorData::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_SENSORDATA_TIME.c_str());											writer.Double(m_time);
	writer.Key(KEY_SENSORDATA_TIMEDIFF.c_str());										writer.Double(m_timeDiff);
	writer.Key(KEY_SENSORDATA_PHI_DEGREE.c_str());										writer.Double(m_phiDegree);
	writer.Key(KEY_SENSORDATA_POWER.c_str());											writer.Double(m_power);
	writer.EndObject();

}

bool SensorData::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "SensorData: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_SENSORDATA_TIME.c_str())) {
		LOG_ERROR << "SensorData: missing " << KEY_SENSORDATA_TIME.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SENSORDATA_TIMEDIFF.c_str())) {
		LOG_ERROR << "SensorData: missing " << KEY_SENSORDATA_TIMEDIFF.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SENSORDATA_PHI_DEGREE.c_str())) {
		LOG_ERROR << "SensorData: missing " << KEY_SENSORDATA_PHI_DEGREE.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SENSORDATA_POWER.c_str())) {
		LOG_ERROR << "SensorData: missing " << KEY_SENSORDATA_POWER.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& timeValue = value[KEY_SENSORDATA_TIME.c_str()];
	const rapidjson::Value& timeDiffValue = value[KEY_SENSORDATA_TIMEDIFF.c_str()];
	const rapidjson::Value& phiDegreeValue = value[KEY_SENSORDATA_PHI_DEGREE.c_str()];
	const rapidjson::Value& powerValue = value[KEY_SENSORDATA_POWER.c_str()];

	if (!timeValue.IsDouble()) {
		LOG_ERROR << "SensorData: " << KEY_SENSORDATA_TIME.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!timeDiffValue.IsDouble()) {
		LOG_ERROR << "SensorData: " << KEY_SENSORDATA_TIMEDIFF.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!phiDegreeValue.IsDouble()) {
		LOG_ERROR << "SensorData: " << KEY_SENSORDATA_PHI_DEGREE.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!powerValue.IsDouble()) {
		LOG_ERROR << "SensorData: " << KEY_SENSORDATA_POWER.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_time = timeValue.GetDouble();
	m_timeDiff = timeDiffValue.GetDouble();
	m_phiDegree = phiDegreeValue.GetDouble();
	m_power = powerValue.GetDouble();

	m_powerLin = std::pow(10.0, m_power / 10.0);
	m_phi = (m_phiDegree / 180.0) * PI;

	return true;
}
