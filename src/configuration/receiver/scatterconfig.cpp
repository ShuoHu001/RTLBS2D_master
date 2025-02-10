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
	if (m_positions.empty()) {//若离散点无数据则从文件中读取
		// 从文件中读取离散点
		std::ifstream inFile(m_fileName);
		if (!inFile.is_open()) {
			// 打开文件失败，记录错误并退出
			LOG_ERROR << "Failed to open file: " << m_fileName << CRASH;
			return;
		}

		// 逐行读取文件内容
		std::string line;
		while (getline(inFile, line)) {
			// 解析每行数据
			std::vector<std::string> cols;
			boost::split(cols, line, boost::is_any_of(" \t,"));
			if (cols.size() == 3) {
				float x = boost::lexical_cast<float>(cols[0]);
				float y = boost::lexical_cast<float>(cols[1]);
				float z = boost::lexical_cast<float>(cols[2]);
				// 添加到离散点数组中
				Point3D p(x, y, z);
				ReceiverUnitConfig rxUnitConfig;
				rxUnitConfig.m_position = p;
				rxUnitConfig.m_velocity = 0;
				configs.push_back(rxUnitConfig);
			}
		}
	}
	else {		//从加载的点集数据中读取文件
		for (int i = 0; i < m_positions.size(); ++i) {
			ReceiverUnitConfig rxUnitConfig;
			rxUnitConfig.m_position = m_positions[i];
			//rxUnitConfig.m_velocity = m_velocities[i];  暂不添加速度项
			configs.push_back(rxUnitConfig);
		}
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
