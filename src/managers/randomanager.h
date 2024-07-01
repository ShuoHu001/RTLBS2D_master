#ifndef RTLBS_RANDOMANAGER
#define RTLBS_RANDOMANAGER

#include<random>
#include "utility/singleton.h"

#define RANDINT(min_val, max_val) RandoManager::getInstance().RandInt(min_val, max_val)
#define RANDFLOAT(min_val, max_val) RandoManager::getInstance().RandFloat(min_val, max_val)
#define RANDDOUBLE(min_val, max_val) RandoManager::getInstance().RandDouble(min_val, max_val)
#define NORMDOUBLE(mean_val, std_val) RandoManager::getInstance().NormDistributionDouble(mean_val, std_val)


class RandoManager :public Singleton<RandoManager> {
public:
	int RandInt(int min_val, int max_val);
	float RandFloat(float min_val, float max_val);
	double RandDouble(double min_val, double max_val);
	double NormDistributionDouble(RtLbsType mean, RtLbsType mu);
private:
	friend class Singleton<RandoManager>;
	std::mt19937 m_gen; //梅林旋转方法得到随机数
	RandoManager();
	~RandoManager();
};


#endif
