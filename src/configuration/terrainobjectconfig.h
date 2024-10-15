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
	std::string m_fileName;										/** @brief	文件名称	*/
	WINDING_ORDER m_windingOrder;								/** @brief	顺逆时针存储三角形元素	*/
	bool m_reverseCoord;										/** @brief	是否是相反的坐标系，默认为false，即XYZ坐标系，true为XZY坐标系	*/
	std::vector<std::string> m_matNames;						/** @brief	材质名称集合	*/

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
