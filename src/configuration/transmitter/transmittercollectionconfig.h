#ifndef RTLBS_TRANSMITTERCOLLECTIONCONFIG
#define RTLBS_TRANSMITTERCOLLECTIONCONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "utility/enum.h"
#include "transmitterconfig.h"

const std::string KEY_TRANSMITTERCOLLECTIONCONFIG = "TransmitterCollectionConfig";
const std::string KEY_TRANSMITTERCOLLECTIONCONFIG_TRANSMITTERCONFIGS = "TransmitterConfigs";

class TransmitterCollectionConfig :public Serializable {
public:
	std::vector<TransmitterConfig> m_transmitterConfigs;				/** @brief	发射机配置集合	*/

public:
	TransmitterCollectionConfig();
	TransmitterCollectionConfig(const TransmitterCollectionConfig& config);
	~TransmitterCollectionConfig();
	TransmitterCollectionConfig& operator = (const TransmitterCollectionConfig& config);
	bool Init(std::string filename);
	void Write2Json(const std::string& filename);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
