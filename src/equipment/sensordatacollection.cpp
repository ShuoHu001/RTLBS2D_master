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

void SensorDataCollection::SortByPower()
{
	std::sort(m_data.begin(), m_data.end());
}

void SensorDataCollection::CalculateTimeDiff()
{
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