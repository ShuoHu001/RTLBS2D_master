#ifndef RTLBS_MATERIALLIBRARYCONFIG
#define RTLBS_MATERIALLIBRARYCONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"
#include "configuration/materialconfig.h"

const std::string KEY_MATERIALLIBRARYCONFIG_MATERIALS = "MaterialConfigs";
const std::string KEY_MATERIALLIBRARYCONFIG = "MaterialLibraryConfig";

class MaterialLibraryConfig:public Serializable {
public:
	std::vector<MaterialConfig> m_materials;																/** @brief	材质数组	*/
public:
	MaterialLibraryConfig();
	~MaterialLibraryConfig();
	bool Init(std::string filename);																//从json文件中初始化材质库配置
	void Write2Json(std::string filename);															//将材质库配置写入至文件中
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);						//序列化材质库配置
	bool Deserialize(const rapidjson::Value& value);												//反序列化材质库配置
};

#endif
