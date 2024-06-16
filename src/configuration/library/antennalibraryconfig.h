#ifndef RTLBS_ANTENNALIBRARYCONFIG
#define RTLBS_ANTENNALIBRARYCONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"
#include "configuration/antennaconfig.h"
#include "antenna/antenna.h"

const std::string KEY_ANTENNALIBRARYCONFIG = "AntennaLibraryConfig";
const std::string KEY_ANTENNALIBRARYCONFIG_ANTENNAS = "Antennas";

class AntennaLibraryConfig :public Serializable {
public:
	std::vector<AntennaConfig> m_antennas;										/** @brief	天线配置数组	*/
public:
	AntennaLibraryConfig();
	AntennaLibraryConfig(const AntennaLibraryConfig& config);
	~AntennaLibraryConfig();
	AntennaLibraryConfig& operator = (const AntennaLibraryConfig& config);
	bool Init(const std::string& filename);
	void Write2Json(const std::string& filename);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};


#endif
