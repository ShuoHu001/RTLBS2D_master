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
	//���㹦��Ȩ��ϵ��(��ͬʱ���й��ʺϳ�)
	std::sort(m_datas.begin(), m_datas.end());				//���չ��ʴ�С��������
	std::vector<RtLbsType> powerWeight;													/** @brief	����Ȩ��	*/
	powerWeight.reserve(m_datas.size());												//Ԥ���ռ�
	RtLbsType sumPowerWeight = 0.0;														/** @brief	����Ȩ�غ�	*/
	for (auto& curInfo : m_datas) {
		RtLbsType curPowerWeight = pow(10, curInfo.m_power / 10.0);
		powerWeight.push_back(curPowerWeight);
		sumPowerWeight += curPowerWeight;
	}
	for (auto& w : powerWeight) {
		w /= sumPowerWeight;
	}

	m_mergedData.m_power = 10 * log10(sumPowerWeight);									//ת���ϳɹ���

	m_mergedData.m_powerLin = sumPowerWeight;

	RtLbsType aoaPhi = 0.0;
	RtLbsType aoaTheta = 0.0;
	for (int i = 0; i < static_cast<int>(m_datas.size()); ++i) {
		aoaPhi += powerWeight[i] * m_datas[i].m_phi;				//�����Ȩ�Ƕ�phi
	}

	m_mergedData.m_phi = aoaPhi;

	//ʱ�Ӻϳɣ����չ�������ʱ����Ϊ��ʱ��
	m_mergedData.m_time = m_datas.front().m_time;
	m_mergedData.m_timeDiff = m_datas.front().m_timeDiff;
}

void SensorDataCluster::CalMergedDataByTime()
{
	//���㹦��Ȩ��-��ͬʱ����ϳɹ���
	std::sort(m_datas.begin(), m_datas.end());				//���չ��ʴ�С��������
	std::vector<RtLbsType> powerWeight;													/** @brief	����Ȩ��	*/
	powerWeight.reserve(m_datas.size());												//Ԥ���ռ�
	RtLbsType sumPowerWeight = 0.0;														/** @brief	����Ȩ�غ�	*/
	for (auto& curInfo : m_datas) {
		RtLbsType curPowerWeight = pow(10, curInfo.m_power / 10.0);
		powerWeight.push_back(curPowerWeight);
		sumPowerWeight += curPowerWeight;
	}
	for (auto& w : powerWeight) {
		w /= sumPowerWeight;
	}

	m_mergedData.m_power = 10 * log10(sumPowerWeight);									//ת���ϳɹ���

	RtLbsType mergedTimeDelay = 0.0;
	for (int i = 0; i < static_cast<int>(m_datas.size()); ++i) {						//�����Ȩʱ��
		mergedTimeDelay += powerWeight[i] * m_datas[i].m_time;
	}
	m_mergedData.m_time = mergedTimeDelay;
}

void SensorDataCluster::CalMergedDataByTimeDiff()
{
	//���㹦��Ȩ��-��ͬʱ����ϳɹ���
	std::sort(m_datas.begin(), m_datas.end());				//���չ��ʴ�С��������
	std::vector<RtLbsType> powerWeight;													/** @brief	����Ȩ��	*/
	powerWeight.reserve(m_datas.size());												//Ԥ���ռ�
	RtLbsType sumPowerWeight = 0.0;														/** @brief	����Ȩ�غ�	*/
	for (auto& curInfo : m_datas) {
		RtLbsType curPowerWeight = pow(10, curInfo.m_power / 10.0);
		powerWeight.push_back(curPowerWeight);
		sumPowerWeight += curPowerWeight;
	}
	for (auto& w : powerWeight) {
		w /= sumPowerWeight;
	}

	m_mergedData.m_power = 10 * log10(sumPowerWeight);									//ת���ϳɹ���

	RtLbsType mergedTimeDiff = 0.0;
	for (int i = 0; i < static_cast<int>(m_datas.size()); ++i) {						//�����Ȩʱ���
		mergedTimeDiff += powerWeight[i] * m_datas[i].m_timeDiff;
	}
	m_mergedData.m_timeDiff = mergedTimeDiff;
}
