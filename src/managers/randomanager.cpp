#include "randomanager.h"

int RandoManager::RandInt(int min_val, int max_val)
{
	std::uniform_int_distribution<int> dist(min_val, max_val);
	return dist(m_gen);
}

float RandoManager::RandFloat(float min_val, float max_val)
{
	std::uniform_real_distribution<float> dist(min_val, max_val);
	return dist(m_gen);
}

double RandoManager::RandDouble(double min_val, double max_val)
{
	std::uniform_real_distribution<double> dis(min_val, max_val);
	return dis(m_gen);
}

double RandoManager::NormDistributionDouble(RtLbsType mu, RtLbsType sigma)
{
	if (sigma == 0) {
		return 0;
	}
	std::normal_distribution<double> dis(mu, sigma);
	RtLbsType reVal = dis(m_gen);
	//while (std::abs(reVal - mu) > sigma) {
	//	reVal = dis(m_gen);
	//}
	return reVal;
}

RandoManager::RandoManager()
	: m_gen(std::random_device()())
{
}

RandoManager::~RandoManager()
{
}
