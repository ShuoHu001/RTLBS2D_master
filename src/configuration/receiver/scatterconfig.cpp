#include "scatterconfig.h"

ScatterConfig::ScatterConfig()
	: m_fileName("point.txt")
{
}

ScatterConfig::ScatterConfig(const ScatterConfig& config)
	: m_fileName(config.m_fileName)
	, m_positions(config.m_positions)
	, m_velocities(config.m_velocities)
{
}

ScatterConfig::~ScatterConfig()
{
}

ScatterConfig& ScatterConfig::operator=(const ScatterConfig& config)
{
	m_fileName = config.m_fileName;
	m_positions = config.m_positions;
	m_velocities = config.m_velocities;
	return *this;
}

void ScatterConfig::CalculateRxPosition(std::vector<ReceiverUnitConfig>& configs)
{
	if (m_positions.empty()) {//����ɢ������������ļ��ж�ȡ
		// ���ļ��ж�ȡ��ɢ��
		std::ifstream inFile(m_fileName);
		if (!inFile.is_open()) {
			// ���ļ�ʧ�ܣ���¼�����˳�
			LOG_ERROR << "Failed to open file: " << m_fileName << CRASH;
			return;
		}

		// ���ж�ȡ�ļ�����
		std::string line;
		while (getline(inFile, line)) {
			// ����ÿ������
			std::istringstream iss(line);
			float x, y, z, v;
			if (!(iss >> x >> y >> z >> v)) {
				// ����ʧ�ܣ���¼������������
				LOG_ERROR << "Failed to parse line: " << line << ENDL;
				continue;
			}

			// ��ӵ���ɢ��������
			Point3D p(x, y, z);
			configs.push_back(p);
		}
		return;
	}
	//����ɢ���а������ݣ���ֱ�ӽ����ݸ�ֵ
	configs.resize(m_positions.size());
	for (int i = 0; i < m_positions.size(); ++i) {
		configs[i].m_position = m_positions[i];
		configs[i].m_velocity = m_velocities[i];
	}
}

void ScatterConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_SCATTERCONFIG_FILENAME.c_str());									writer.String(m_fileName.c_str());
	writer.Key(KEY_SCATTERCONFIG_POSITIONS.c_str());									SerializeArray(m_positions, writer);
	writer.Key(KEY_SCATTERCONFIG_VELOCITIES.c_str());									SerializeArray(m_velocities, writer);
	writer.EndObject();
}

bool ScatterConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "ScatterConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_SCATTERCONFIG_FILENAME.c_str())) {
		LOG_ERROR << "ScatterConfig: missing " << KEY_SCATTERCONFIG_FILENAME.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SCATTERCONFIG_POSITIONS.c_str())) {
		LOG_ERROR << "ScatterConfig: missing " << KEY_SCATTERCONFIG_POSITIONS.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SCATTERCONFIG_VELOCITIES.c_str())) {
		LOG_ERROR << "ScatterConfig: missing " << KEY_SCATTERCONFIG_VELOCITIES.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& fileNameValue = value[KEY_SCATTERCONFIG_FILENAME.c_str()];
	const rapidjson::Value& positionsValue = value[KEY_SCATTERCONFIG_POSITIONS.c_str()];
	const rapidjson::Value& velocitiesValue = value[KEY_SCATTERCONFIG_VELOCITIES.c_str()];

	if (!fileNameValue.IsString()) {
		LOG_ERROR << "ScatterConfig: " << KEY_SCATTERCONFIG_FILENAME.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!positionsValue.IsArray()) {
		LOG_ERROR << "ScatterConfig: " << KEY_SCATTERCONFIG_POSITIONS.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!velocitiesValue.IsArray()) {
		LOG_ERROR << "ScatterConfig: " << KEY_SCATTERCONFIG_VELOCITIES.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_fileName = fileNameValue.GetString();

	if (!DeserializeArray(m_positions, positionsValue)) {
		LOG_ERROR << "ScatterConfig: " << KEY_SCATTERCONFIG_POSITIONS.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	if (!DeserializeArray(m_velocities, velocitiesValue)) {
		LOG_ERROR << "ScatterConfig: " << KEY_SCATTERCONFIG_VELOCITIES.c_str() << ", deserialize failed." << ENDL;
		return false;
	}
	
	return true;
}
