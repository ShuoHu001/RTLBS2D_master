#ifndef RTLBS_MATMANAGER
#define RTLBS_MATMANAGER

#include "utility/singleton.h"
#include <string>
#include <map>


class Material;

class MatManager :public Singleton<MatManager> {
private:
	std::map<std::string, Material*> m_matPool;
	Material* m_Default;
	friend class Singleton<MatManager>;

public:
	MatManager();
	~MatManager();

public:
	Material* FindMaterial(const std::string& mat_name) const;//找到对应的材质
	Material* GetDefaultMat() const;//获得默认的材质
	unsigned ParseMatFile(const std::string& str);//从文件中读取材质信息
	unsigned GetMatCount() const;//获得材质的数量

private:
	void _clearMatPool();
};


#endif
