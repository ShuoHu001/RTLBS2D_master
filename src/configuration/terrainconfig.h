#ifndef RTLBS_TERRAINLIBRARYELEMENTCONFIG
#define RTLBS_TERRAINLIBRARYELEMENTCONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/serializable.h"
#include "terrainobjectconfig.h"
#include "physical/propagationproperty.h"


const std::string KEY_TERRAINGRIDCONFIG_GEOMETRYMATRIXFILE = "GeometryMatrixFile";
const std::string KEY_TERRAINGRIDCONFIG_MATERIALMATRIXFILE = "MaterialMatrixFile";
const std::string KEY_TERRAINGRIDCONFIG_MATERIALS = "MatNames";

class TerrainGridConfig :public Serializable {
public:
	std::string m_heightMatrixFile;								/** @brief	地形高程矩阵文件名	*/
	std::string m_materialMatrixFile;							/** @brief	地形栅格矩阵文件名	*/
	std::vector<std::string> m_matNames;						/** @brief	材质名称集合	*/
public:
	TerrainGridConfig();
	TerrainGridConfig(const TerrainGridConfig& config);
	~TerrainGridConfig();
	TerrainGridConfig& operator = (const TerrainGridConfig& config);
	bool operator == (const TerrainGridConfig& config) const;
	bool operator != (const TerrainGridConfig& config) const;
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);

};

const std::string KEY_TERRAINCONFIG_LOADINGSTATE = "LoadingState";
const std::string KEY_TERRAINCONFIG = "TerrainConfig";
const std::string KEY_TERRAINCONFIG_LOADMODE = "LoadMode";
const std::string KEY_TERRAINCONFIG_CATEGORY = "Category";
const std::string KEY_TERRAINCONFIG_SIMPLIFYFLAG = "SimplifyFlag";
const std::string KEY_TERRAINCONFIG_SIMPLIFYRATE = "SimplifyRate";
const std::string KEY_TERRAINCONFIG_AVERAGERIDGEGAP = "AverageRidgeGap";
const std::string KEY_TERRAINCONFIG_OBJECTCONFIG = "ObjectConfig";
const std::string KEY_TERRAINCONFIG_GRIDCONFIG = "GridConfig";

class TerrainConfig: public Serializable {
public:
	TERRAINLOADMODE m_loadingMode;									/** @brief	地形读取模式 物体模式or栅格模式	*/
	TERRAINCATEGORY m_category;										/** @brief	地形类型 栅格、mesh、含孔洞mesh	*/
	bool m_simplifyFlag;											/** @brief	地形简化状态	*/
	RtLbsType m_simplifyRate;										/** @brief	地形简化率(0-1之间) 0代表不简化*/
	RtLbsType m_averageRidgeGap;									/** @brief	起伏地形多个峰峦间的平均距离	*/
	TerrainObjectConfig m_objectConfig;								/** @brief	地形物体配置 */
	TerrainGridConfig m_gridConfig;									/** @brief	地形栅格配置	*/
	PropagationProperty m_propagationProperty;						/** @brief	地形传播属性	*/
	
public:
	TerrainConfig();
	TerrainConfig(const TerrainConfig& config);
	~TerrainConfig();
	TerrainConfig& operator = (const TerrainConfig& config);
	bool operator == (const TerrainConfig& config) const;
	bool operator != (const TerrainConfig& config) const;
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
	bool Init(std::string filename);
	void Write2Json(std::string filename);
	
};


#endif
