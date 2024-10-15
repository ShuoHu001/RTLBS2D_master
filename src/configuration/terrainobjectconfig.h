#ifndef RTLBS_TERRAINOBJECTCONFIG
#define RTLBS_TERRAINOBJECTCONFIG

#include "utility/serializable.h"
#include "utility/enum.h"
#include "math/vector3d.h"
#include "math/point3d.h"
#include "math/euler.h"
#include "radiowave/propagation/propagationproperty.h"

const std::string KEY_TERRAINOBJECTCONFIG_FILENAME = "FileName";
const std::string KEY_TERRAINOBJECTCONFIG_WINDINGORDER = "WindingOrder";
const std::string KEY_TERRAINOBJECTCONFIG_REVERSECOORD = "ReverseCoord";
const std::string KEY_TERRAINOBJECTCONFIG_MATNAMES = "MatNames";

class TerrainObjectConfig :public Serializable {
public:
	std::string m_fileName;										/** @brief	�ļ�����	*/
	WINDING_ORDER m_windingOrder;								/** @brief	˳��ʱ��洢������Ԫ��	*/
	bool m_reverseCoord;										/** @brief	�Ƿ����෴������ϵ��Ĭ��Ϊfalse����XYZ����ϵ��trueΪXZY����ϵ	*/
	std::vector<std::string> m_matNames;						/** @brief	�������Ƽ���	*/

public:
	TerrainObjectConfig();
	TerrainObjectConfig(const TerrainObjectConfig& config);
	~TerrainObjectConfig();
	TerrainObjectConfig& operator = (const TerrainObjectConfig& config);
	bool operator == (const TerrainObjectConfig& config) const;
	bool operator != (const TerrainObjectConfig& config) const;
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
