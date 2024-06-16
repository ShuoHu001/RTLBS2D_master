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
	Material* FindMaterial(const std::string& mat_name) const;//�ҵ���Ӧ�Ĳ���
	Material* GetDefaultMat() const;//���Ĭ�ϵĲ���
	unsigned ParseMatFile(const std::string& str);//���ļ��ж�ȡ������Ϣ
	unsigned GetMatCount() const;//��ò��ʵ�����

private:
	void _clearMatPool();
};


#endif
