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
	SYSTEM_MODE m_systemMode;															/** @brief	ϵͳ����ģʽ	*/
	FrequencyConfig m_frequencyConfig;													/** @brief	Ƶ������	*/
	RaytrcingConfig m_raytracingConfig;													/** @brief	����׷������	*/
	LocalizeConfig m_lbsConfig;												/** @brief	��λ����	*/
	OutputConfig m_outputConfig;														/** @brief	�������	*/

	//���б�����ֱ�Ӳ��뵽���õ����л���
	GeometryConfig m_geometryConfig;													/** @brief	������������	*/
	MaterialLibraryConfig m_materialLibraryConfig;										/** @brief	���ʿ�����	*/
	AntennaLibraryConfig m_antennaLibraryConfig;										/** @brief	���߿�����	*/
	TransmitterCollectionConfig m_transmitterConfig;									/** @brief	���������	*/
	ReceiverConfig m_receiverConfig;													/** @brief	���ջ�����	*/
	SensorCollectionConfig m_sensorConfig;												/** @brief	����������	*/

private:
	std::string m_geometryConfigFile;													/** @brief	�������������ļ�	*/
	std::string m_materialLibraryConfigFile;											/** @brief	���ʿ������ļ�	*/
	std::string m_antennaLibraryConfigFile;												/** @brief	���߿������ļ�	*/
	std::string m_transmitterConfigFile;												/** @brief	����������ļ�	*/
	std::string m_receiverConfigFile;													/** @brief	���ջ������ļ�	*/
	std::string m_sensorConfigFile;														/** @brief	�����������ļ�	*/

	
public:
	SimConfig();
	~SimConfig();
	bool Init(const std::string& filename);												//��ʼ��simconfig
	void Writer2Json(const std::string& fielname);										//��jsonд��ָ���ļ���
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);			//���л��������ö���
	bool Deserialize(const rapidjson::Value& value);									//�����л��������ö���
	bool Valid() const;																//��֤���õ���ȷ��
};

#endif
