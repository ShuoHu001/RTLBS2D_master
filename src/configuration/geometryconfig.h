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
	std::string m_buildingFile;					/** @brief	建筑物文件	*/
	std::string m_buildingAttributeFile;		/** @brief	建筑物属性文件	*/
	std::string m_vegetationFile;				/** @brief	绿化植被文件	*/
	std::string m_vegetationAttributeFile;		/** @brief	绿化植被属性文件	*/
	std::string m_wallFile;						/** @brief	墙体文件	*/
	std::string m_wallAttributeFile;			/** @brief	墙体属性文件	*/
	bool m_loadingTerrainFlag;					/** @brief	地形加载状态	*/
	TerrainConfig m_terrainConfig;				/** @brief	地形配置	*/
	RtLbsType m_positionError;					/** @brief	位置偏移误差	*/

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
