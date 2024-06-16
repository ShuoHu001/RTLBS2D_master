#ifndef RTLBS_MATERIALLIBRARY
#define RTLBS_MATERIALLIBRARY

#include "rtlbs.h"
#include "utility/define.h"
#include "material.h"
#include "configuration/library/materiallibraryconfig.h"

class MaterialLibrary {
public:
	std::vector<Material*> m_materials;			/** @brief	������Ϣ	*/

public:
	MaterialLibrary();
	MaterialLibrary(const MaterialLibraryConfig& config);
	~MaterialLibrary();
	bool Init(const MaterialLibraryConfig& config);						//��ʼ�����ʿ�
	Material* GetMaterial(const std::string& matName) const;			//�������ƻ�ȡ������Ϣ
	int GetMaterialId(const std::string& matName) const;				//�������ƻ�ȡ����ID
	Material* GetMaterial(unsigned matId) const;						//����ID��ȡ������Ϣ
	Material* GetDefaultMaterial() const;								//��ȡĬ�ϵĲ���
};

#endif
