#include "sensordatacluster.h"

SensorDataCluster::SensorDataCluster()
{
}

SensorDataCluster::SensorDataCluster(const SensorDataCluster& cluster)
	: m_datas(cluster.m_datas)
	, m_mergedData(cluster.m_mergedData)
{
}

SensorDataCluster::~SensorDataCluster()
{
}

SensorDataCluster& SensorDataCluster::operator=(const SensorDataCluster& cluster)
{
	m_datas = cluster.m_datas;
	m_mergedData = cluster.m_mergedData;
	return *this;
}

bool SensorDataCluster::CanAddToClusterByAOA2D(const SensorData& data, RtLbsType threshold) const
{
	bool addFlag = true;
	if (m_mergedData.DistanceAOA2D(data) > threshold) {
		addFlag = false;
	}
	return addFlag;
}

bool SensorDataCluster::CanAddToClusterByTime(const SensorData& data, RtLbsType threshold) const
{
	bool addFlag = false;
	if (m_mergedData.DistanceTime(data) > threshold) {
		addFlag = false;
	}
	return addFlag;
}

bool SensorDataCluster::CanAddToClusterByTimeDiff(const SensorData& data, RtLbsType threshold) const
{
	bool addFlag = false;
	if (m_mergedData.DistanceTimeDiff(data) > threshold) {
		addFlag = false;
	}
	return addFlag;
}

void SensorDataCluster::CalMergedDataByAOA()
{
	//计算功率权重系数(可同时进行功率合成)
	std::sort(m_datas.begin(), m_datas.end());				//按照功率大小进行排序
	std::vector<RtLbsType> powerWeight;													/** @brief	功率权重	*/
	powerWeight.reserve(m_datas.size());												//预留空间
	RtLbsType sumPowerWeight = 0.0;														/** @brief	功率权重和	*/
	for (auto& curInfo : m_datas) {
		RtLbsType curPowerWeight = pow(10, curInfo.m_power / 10.0);
		powerWeight.push_back(curPowerWeight);
		sumPowerWeight += curPowerWeight;
	}
	for (auto& w : powerWeight) {
		w /= sumPowerWeight;
	}

	m_mergedData.m_power = 10 * log10(sumPowerWeight);									//转换合成功率

	m_mergedData.m_powerLin = sumPowerWeight;

	RtLbsType aoaPhi = 0.0;
	RtLbsType aoaTheta = 0.0;
	for (int i = 0; i < static_cast<int>(m_datas.size()); ++i) {
		aoaPhi += powerWeight[i] * m_datas[i].m_phi;				//计算加权角度phi
	}

	m_mergedData.m_phi = aoaPhi;

	//时延合成，按照功率最大的时延作为其时延
	m_mergedData.m_time = m_datas.front().m_time;
	m_mergedData.m_timeDiff = m_datas.front().m_timeDiff;
}

void SensorDataCluster::CalMergedDataByTime()
{
	//计算功率权重-可同时计算合成功率
	std::sort(m_datas.begin(), m_datas.end());				//按照功率大小进行排序
	std::vector<RtLbsType> powerWeight;													/** @brief	功率权重	*/
	powerWeight.reserve(m_datas.size());												//预留空间
	RtLbsType sumPowerWeight = 0.0;														/** @brief	功率权重和	*/
	for (auto& curInfo : m_datas) {
		RtLbsType curPowerWeight = pow(10, curInfo.m_power / 10.0);
		powerWeight.push_back(curPowerWeight);
		sumPowerWeight += curPowerWeight;
	}
	for (auto& w : powerWeight) {
		w /= sumPowerWeight;
	}

	m_mergedData.m_power = 10 * log10(sumPowerWeight);									//转换合成功率

	RtLbsType mergedTimeDelay = 0.0;
	for (int i = 0; i < static_cast<int>(m_datas.size()); ++i) {						//计算加权时延
		mergedTimeDelay += powerWeight[i] * m_datas[i].m_time;
	}
	m_mergedData.m_time = mergedTimeDelay;
}

void SensorDataCluster::CalMergedDataByTimeDiff()
{
	//计算功率权重-可同时计算合成功率
	std::sort(m_datas.begin(), m_datas.end());				//按照功率大小进行排序
	std::vector<RtLbsType> powerWeight;													/** @brief	功率权重	*/
	powerWeight.reserve(m_datas.size());												//预留空间
	RtLbsType sumPowerWeight = 0.0;														/** @brief	功率权重和	*/
	for (auto& curInfo : m_datas) {
		RtLbsType curPowerWeight = pow(10, curInfo.m_power / 10.0);
		powerWeight.push_back(curPowerWeight);
		sumPowerWeight += curPowerWeight;
	}
	for (auto& w : powerWeight) {
		w /= sumPowerWeight;
	}

	m_mergedData.m_power = 10 * log10(sumPowerWeight);									//转换合成功率

	RtLbsType mergedTimeDiff = 0.0;
	for (int i = 0; i < static_cast<int>(m_datas.size()); ++i) {						//计算加权时间差
		mergedTimeDiff += powerWeight[i] * m_datas[i].m_timeDiff;
	}
	m_mergedData.m_timeDiff = mergedTimeDiff;
}
