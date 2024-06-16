#ifndef RTLBS_MATERIALLIBRARY
#define RTLBS_MATERIALLIBRARY

#include "rtlbs.h"
#include "utility/define.h"
#include "material.h"
#include "configuration/library/materiallibraryconfig.h"

class MaterialLibrary {
public:
	std::vector<Material*> m_materials;			/** @brief	材质信息	*/

public:
	MaterialLibrary();
	MaterialLibrary(const MaterialLibraryConfig& config);
	~MaterialLibrary();
	bool Init(const MaterialLibraryConfig& config);						//初始化材质库
	Material* GetMaterial(const std::string& matName) const;			//基于名称获取材质信息
	int GetMaterialId(const std::string& matName) const;				//基于名称获取材质ID
	Material* GetMaterial(unsigned matId) const;						//基于ID获取材质信息
	Material* GetDefaultMaterial() const;								//获取默认的材料
};

#endif
