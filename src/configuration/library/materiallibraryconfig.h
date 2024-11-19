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
	std::vector<MaterialConfig> m_materials;																/** @brief	��������	*/
public:
	MaterialLibraryConfig();
	~MaterialLibraryConfig();
	bool Init(std::string filename);																//��json�ļ��г�ʼ�����ʿ�����
	void Write2Json(std::string filename);															//�����ʿ�����д�����ļ���
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);						//���л����ʿ�����
	bool Deserialize(const rapidjson::Value& value);												//�����л����ʿ�����
};

#endif
