#include "sensordatacollection.h"

SensorDataCollection::SensorDataCollection()
	: m_sensorId(-1)
{
}

SensorDataCollection::SensorDataCollection(const SensorDataCollection& collection)
	: m_sensorId(collection.m_sensorId)
	, m_data(collection.m_data)
{
}

SensorDataCollection::~SensorDataCollection()
{
}

SensorDataCollection& SensorDataCollection::operator=(const SensorDataCollection& collection)
{
	m_sensorId = collection.m_sensorId;
	m_data = collection.m_data;
	return *this;
}

RtLbsType SensorDataCollection::CalculateRMSAngularSpread() const
{
	if (m_data.size() == 1) {
		return m_data[0].m_phi;
	}
	RtLbsType meanAoA = CalculateMeanArrivedAngle();
	
	RtLbsType conTemp1 = 0.0;
	RtLbsType conTemp2 = 0.0;

	for (auto& data : m_data) {
		conTemp1 += data.m_powerLin * (data.m_phi - meanAoA) * (data.m_phi - meanAoA);
		conTemp2 += data.m_powerLin;
	}

	RtLbsType rmsAoA = sqrt(conTemp1 / conTemp2);

	return rmsAoA;
}

RtLbsType SensorDataCollection::CalculateRMSDelaySpread() const
{
	if (m_data.size() == 1) {
		return m_data[0].m_phi;
	}
	RtLbsType meanDelay = CalculateMeanArrivedDelay();

	RtLbsType conTemp1 = 0.0;
	RtLbsType conTemp2 = 0.0;

	for (auto& data : m_data) {
		conTemp1 += data.m_powerLin * (data.m_time - meanDelay) * (data.m_time - meanDelay);
		conTemp2 += data.m_powerLin;
	}

	RtLbsType rmsDelay = sqrt(conTemp1 / conTemp2);

	return rmsDelay;
}

void SensorDataCollection::ReClusterByAOAError(RtLbsType phiError)
{
	std::vector<SensorDataCluster> clusters = ClusterSensorDataByAOA2D(m_data, phiError);
	m_data.clear();
	for (auto& curCluster : clusters) {
		m_data.push_back(curCluster.m_mergedData);
	}
}

void SensorDataCollection::SortByPower()
{
	std::sort(m_data.begin(), m_data.end());
}

void SensorDataCollection::CalculateTimeDiff()
{
	if (m_data.empty()) {
		return;
	}
	RtLbsType firstTimeDelay = m_data.front().m_time;
	for (auto it = std::next(m_data.begin()); it != m_data.end(); ++it) {
		SensorData& curData = *it;
		curData.m_timeDiff = curData.m_time - firstTimeDelay;
	}
}

std::vector<RtLbsType> SensorDataCollection::GetPowerDiffMatrix() const
{
	std::vector<RtLbsType> powerDiff(m_data.size() - 1);
	for (int i = 1; i < static_cast<int>(m_data.size()); ++i) {
		powerDiff[i - 1] = m_data[i].m_power - m_data[0].m_power;
	}
	return powerDiff;
}

void SensorDataCollection::AddSimulationError(RtLbsType phiErrorSigma, RtLbsType timeErrorSigma, RtLbsType powerErrorSigma)
{
	for (auto& data : m_data) {
		data.AddSimulationError(phiErrorSigma, timeErrorSigma, powerErrorSigma);
	}
}

bool SensorDataCollection::Init(const std::string& filename)
{
	if (filename.empty()) {
		LOG_INFO << "SensorDataCollection: Skip, not configure." << ENDL;
		return true;
	}
	std::ifstream config_ifs(filename);
	if (!config_ifs.is_open()) {
		LOG_ERROR << "SensorDataCollection: fail to open " << filename << ENDL;
		this->Write2Json(filename);
		LOG_INFO << "SensorDataCollection: have wrote to default configuration to file: " << filename << ENDL;
		return false;
	}
	std::stringstream ss;
	ss << config_ifs.rdbuf();
	config_ifs.close();
	std::string jsonString = ss.str();
	rapidjson::Document doc;
	doc.Parse(jsonString.c_str());
	if (doc.HasMember(KEY_SENSORDATACOLLECTION.c_str())) {
		rapidjson::Value& value = doc[KEY_SENSORDATACOLLECTION.c_str()];
		if (value.IsObject()) {
			if (this->Deserialize(value)) {
				LOG_INFO << "SensorDataCollection: load data success!" << ENDL;
				return true;
			}
		}
	}
	return false;
}

void SensorDataCollection::Write2Json(const std::string& filename) const
{
	std::ofstream config_ofs(filename);
	if (!config_ofs.is_open()) {
		LOG_ERROR << "SensorDataCollection: fail to open " << filename << ENDL;
		return;
	}
	rapidjson::StringBuffer sb;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
	writer.SetFormatOptions(rapidjson::PrettyFormatOptions::kFormatSingleLineArray);
	writer.SetIndent('\t', 1);
	writer.StartObject();
	writer.Key(KEY_SENSORDATACOLLECTION.c_str()); this->Serialize(writer);
	writer.EndObject();
	config_ofs << sb.GetString();
	config_ofs.close();
	return;
}

void SensorDataCollection::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) const
{
	writer.StartObject();
	writer.Key(KEY_SENSORDATACOLLECTION_SENSORID.c_str());										writer.Int(m_sensorId);
	writer.Key(KEY_SENSORDATACOLLECTION_SENSORDATAS.c_str());									SerializeArray(m_data, writer);
	writer.EndObject();
}

bool SensorDataCollection::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "SensorDataCollection: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_SENSORDATACOLLECTION_SENSORID.c_str())) {
		LOG_ERROR << "SensorDataCollection: missing " << KEY_SENSORDATACOLLECTION_SENSORID.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SENSORDATACOLLECTION_SENSORDATAS.c_str())) {
		LOG_ERROR << "SensorDataCollection: missing " << KEY_SENSORDATACOLLECTION_SENSORDATAS.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& sensorIdValue = value[KEY_SENSORDATACOLLECTION_SENSORID.c_str()];
	const rapidjson::Value& sensorDataValue = value[KEY_SENSORDATACOLLECTION_SENSORDATAS.c_str()];

	if (!sensorIdValue.IsInt()) {
		LOG_ERROR << "SensorDataCollection: " << KEY_SENSORDATACOLLECTION_SENSORID.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!sensorDataValue.IsArray()) {
		LOG_ERROR << "SensorDataCollection: " << KEY_SENSORDATACOLLECTION_SENSORDATAS.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_sensorId = sensorIdValue.GetInt();

	if (!DeserializeArray(m_data, sensorDataValue)) {
		LOG_ERROR << "SensorDataCollection: " << KEY_SENSORDATACOLLECTION_SENSORDATAS.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

	//对sensorDataCollection中的data数据按照能量大小进行排序,从大到小
	SortByPower();
	return true;
}

RtLbsType SensorDataCollection::CalculateMeanArrivedAngle() const
{
	RtLbsType conTemp1 = 0.0;
	RtLbsType conTemp2 = 0.0;
	for (auto& data : m_data) {
		conTemp1 += data.m_powerLin * data.m_phi;
		conTemp2 += data.m_powerLin;
	}
	return conTemp1 / conTemp2;
}

RtLbsType SensorDataCollection::CalculateMeanArrivedDelay() const
{
	RtLbsType conTemp1 = 0.0;
	RtLbsType conTemp2 = 0.0;
	for (auto& data : m_data) {
		conTemp1 += data.m_powerLin * data.m_time;
		conTemp2 += data.m_powerLin;
	}
	return conTemp1 / conTemp2;
}
