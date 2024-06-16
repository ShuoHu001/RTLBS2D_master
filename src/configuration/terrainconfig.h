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
	std::string m_heightMatrixFile;								/** @brief	���θ߳̾����ļ���	*/
	std::string m_materialMatrixFile;							/** @brief	����դ������ļ���	*/
	std::vector<std::string> m_matNames;						/** @brief	�������Ƽ���	*/
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
	TERRAINLOADMODE m_loadingMode;									/** @brief	���ζ�ȡģʽ ����ģʽorդ��ģʽ	*/
	TERRAINCATEGORY m_category;										/** @brief	�������� դ��mesh�����׶�mesh	*/
	bool m_simplifyFlag;											/** @brief	���μ�״̬	*/
	RtLbsType m_simplifyRate;										/** @brief	���μ���(0-1֮��) 0������*/
	RtLbsType m_averageRidgeGap;									/** @brief	������ζ�����ͼ��ƽ������	*/
	TerrainObjectConfig m_objectConfig;								/** @brief	������������ */
	TerrainGridConfig m_gridConfig;									/** @brief	����դ������	*/
	PropagationProperty m_propagationProperty;						/** @brief	���δ�������	*/
	
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
