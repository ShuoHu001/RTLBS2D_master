#ifndef RTLBS_SIMCONFIG
#define RTLBS_SIMCONFIG

#include "rtlbs.h"
#include "utility/enum.h"
#include "utility/serializable.h"
#include "rapidjson/prettywriter.h"
#include "transmitter/transmitterconfig.h"
#include "transmitter/transmittercollectionconfig.h"
#include "receiver/receiverconfig.h"
#include "library/materiallibraryconfig.h"
#include "library/antennalibraryconfig.h"
#include "frequencyconfig.h"
#include "outputconfig.h"
#include "localizeconfig.h"
#include "raytracingconfig.h"
#include "geometryconfig.h"
#include "sensor/sensorconfig.h"
#include "sensor/sensorcollectionconfig.h"

const std::string KEY_SIMCONFIG = "SimConfig";
const std::string KEY_SIMCONFIG_SYSTEMMODE = "SystemMode";
const std::string KEY_SIMCONFIG_FREQUENCYCONFIG = "FrequencyConfig";
const std::string KEY_SIMCONFIG_RAYTRACINGCONFIG = "RaytracingConfig";
const std::string KEY_SIMCONFIG_LOCALIZATIONCONFIG = "LocalizationConfig";
const std::string KEY_SIMCONFIG_OUTPUTCONFIG = "OutputConfig";

const std::string KEY_SIMCONFIG_GEOMETRYCONFIGFILE = "GeometryConfigFile";
const std::string KEY_SIMCONFIG_MATERIALLIBRARYCONFIGFILE = "MaterialLibraryConfigFile";
const std::string KEY_SIMCOFNIG_ANTENNALIBRARYCONFIGFILE = "AntennaLibraryConfigFile";
const std::string KEY_SIMCONFIG_TRANSMITTERCONFIGFILE = "TransmitterConfigFile";
const std::string KEY_SIMCONFIG_RECEIVERCONFIGFILE = "ReceiverConfigFile";
const std::string KEY_SIMCONFIG_SENSORCONFIGFILE = "SensorConfigFile";


class SimConfig: public Serializable {
public:
	SYSTEM_MODE m_systemMode;															/** @brief	系统工作模式	*/
	FrequencyConfig m_frequencyConfig;													/** @brief	频率配置	*/
	RaytrcingConfig m_raytracingConfig;													/** @brief	射线追踪配置	*/
	LocalizeConfig m_lbsConfig;												/** @brief	定位配置	*/
	OutputConfig m_outputConfig;														/** @brief	输出配置	*/

	//下列变量不直接参与到配置的序列化中
	GeometryConfig m_geometryConfig;													/** @brief	场景几何配置	*/
	MaterialLibraryConfig m_materialLibraryConfig;										/** @brief	材质库配置	*/
	AntennaLibraryConfig m_antennaLibraryConfig;										/** @brief	天线库配置	*/
	TransmitterCollectionConfig m_transmitterConfig;									/** @brief	发射机配置	*/
	ReceiverConfig m_receiverConfig;													/** @brief	接收机配置	*/
	SensorCollectionConfig m_sensorConfig;												/** @brief	传感器配置	*/

private:
	std::string m_geometryConfigFile;													/** @brief	场景几何配置文件	*/
	std::string m_materialLibraryConfigFile;											/** @brief	材质库配置文件	*/
	std::string m_antennaLibraryConfigFile;												/** @brief	天线库配置文件	*/
	std::string m_transmitterConfigFile;												/** @brief	发射机配置文件	*/
	std::string m_receiverConfigFile;													/** @brief	接收机配置文件	*/
	std::string m_sensorConfigFile;														/** @brief	传感器配置文件	*/

	
public:
	SimConfig();
	~SimConfig();
	bool Init(const std::string& filename);												//初始化simconfig
	void Writer2Json(const std::string& fielname);										//将json写入指定文件中
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);			//序列化仿真配置对象
	bool Deserialize(const rapidjson::Value& value);									//反序列化仿真配置对象
	bool Valid() const;																//验证配置的正确性
};

#endif
