#ifndef RTLBS_GEOMETRYCONFIG
#define RTLBS_GEOMETRYCONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"
#include "terrainconfig.h"


const std::string KEY_GEOMETRYCONFIG = "GeometryConfig";
const std::string KEY_GEOMETRYCONFIG_BUILDINGFILE = "BuildingFile";
const std::string KEY_GEOMETRYCONFIG_BUILDINGATTRIBUTEFILE = "BuildingAttributeFile";
const std::string KEY_GEOMETRYCONFIG_VEGETATIONFILE = "VegetationFile";
const std::string KEY_GEOMETRYCONFIG_VEGETATIONATTRIBUTEFILE = "VegetationAttributeFile";
const std::string KEY_GEOMETRYCONFIG_WALLFILE = "WallFile";
const std::string KEY_GEOMETRYCONFIG_WALLATTRIBUTEFILE = "WallAttributeFile";
const std::string KEY_GEOMETRYCONFIG_LOADINGTERRAINFLAG = "LoadingTerrainFlag";
const std::string KEY_GEOMETRYCONFIG_TERRAINCOFNIG = "TerrainConfig";
const std::string KEY_GEOMETRYCONFIG_POSITIONERROR = "PositionError";

class GeometryConfig :public Serializable {
public:
	std::string m_buildingFile;					/** @brief	�������ļ�	*/
	std::string m_buildingAttributeFile;		/** @brief	�����������ļ�	*/
	std::string m_vegetationFile;				/** @brief	�̻�ֲ���ļ�	*/
	std::string m_vegetationAttributeFile;		/** @brief	�̻�ֲ�������ļ�	*/
	std::string m_wallFile;						/** @brief	ǽ���ļ�	*/
	std::string m_wallAttributeFile;			/** @brief	ǽ�������ļ�	*/
	bool m_loadingTerrainFlag;					/** @brief	���μ���״̬	*/
	TerrainConfig m_terrainConfig;				/** @brief	��������	*/
	RtLbsType m_positionError;					/** @brief	λ��ƫ�����	*/

public:
	GeometryConfig();
	GeometryConfig(const GeometryConfig& config);
	~GeometryConfig();
	GeometryConfig& operator = (const GeometryConfig& config);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
	bool Init(const std::string& filename);
	void Write2Json(const std::string& filename);
};

#endif
