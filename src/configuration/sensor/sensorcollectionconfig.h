#ifndef RTLBS_SENSORCOLLECTIONCONFIG
#define RTLBS_SENSORCOLLECTIONCONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"
#include "sensorconfig.h"

const std::string KEY_SENSORCOLLECTIONCONFIG = "SensorCollectionConfig";
const std::string KEY_SENSORCOLLECTIONCONFIG_SENSORCONFIGS = "SensorConfigs";

class SensorCollectionConfig {
public:
	std::vector<SensorConfig> m_sensorConfigs;									/** @brief	传感器配置集合	*/

public:
	SensorCollectionConfig();
	SensorCollectionConfig(const SensorCollectionConfig& config);
	~SensorCollectionConfig();
	SensorCollectionConfig& operator = (const SensorCollectionConfig& config);
	bool Init(const std::string& filename);
	void Write2Json(const std::string& filename) const;
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) const;
	bool Deserialize(const rapidjson::Value& value);
};

#endif
