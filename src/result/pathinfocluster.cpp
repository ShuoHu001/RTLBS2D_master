#include "pathinfocluster.h"

PathInfoCluster::PathInfoCluster()
{
}

PathInfoCluster::PathInfoCluster(const PathInfoCluster& cluster)
	: m_infos(cluster.m_infos)
	, m_mergedInfo(cluster.m_mergedInfo)
{
}

PathInfoCluster::~PathInfoCluster()
{
}

PathInfoCluster& PathInfoCluster::operator=(const PathInfoCluster& cluster)
{
	m_infos = cluster.m_infos;
	m_mergedInfo = cluster.m_mergedInfo;
	return *this;
}

bool PathInfoCluster::CanAddToClusterByAOA2D(const PathInfo& info, RtLbsType threshold) const
{
	//����info��cluster�������ݵľ��룬�����ڴ��ھ������޵���ֵ���򷵻�false
	bool addFlag = true;
	if (m_mergedInfo.DistanceAOA2D(info) > threshold) {
		addFlag = false;
	}
	return addFlag;
}

bool PathInfoCluster::CanAddToClusterByAOA3D(const PathInfo& info, RtLbsType threshold) const
{
	//����info��cluster�������ݵľ��룬�����ڴ��ھ������޵���ֵ���򷵻�false
	bool addFlag = true;
	if (m_mergedInfo.DistanceAOA3D(info) > threshold) {
		addFlag = false;
	}
	return addFlag;
}

bool PathInfoCluster::CanAddToClusterByDelay(const PathInfo& info, RtLbsType threshold) const
{
	//����info��cluster�������ݵľ��룬�����ڴ��ھ������޵���ֵ���򷵻�false
	bool addFlag = true;
	if (m_mergedInfo.DistanceDelay(info) > threshold) {
		addFlag = false;
	}
	return addFlag;
}

void PathInfoCluster::CalMergedInfoByAOA()
{
	//���㹦��Ȩ��ϵ��
	std::sort(m_infos.begin(), m_infos.end(), ComparedByPower_PathInfo);				//���չ��ʴ�С��������
	std::vector<RtLbsType> powerWeight;													/** @brief	����Ȩ��	*/
	powerWeight.reserve(m_infos.size());												//Ԥ���ռ�
	RtLbsType sumPowerWeight = 0.0;														/** @brief	����Ȩ�غ�	*/
	for (auto curInfo : m_infos) {
		RtLbsType curPowerWeight = pow(10, (curInfo.m_power - m_infos.front().m_power) / 10.0);
		powerWeight.push_back(curPowerWeight);
		sumPowerWeight += curPowerWeight;
	}	
	for (auto w : powerWeight) {
		w /= sumPowerWeight;
	}
	for (int i = 0; i < static_cast<int>(m_infos.size()); ++i) {						
		m_mergedInfo.m_aoAPhi += powerWeight[i] * m_infos[i].m_aoAPhi;				//�����Ȩ�Ƕ�phi
		m_mergedInfo.m_aoATheta += powerWeight[i] * m_infos[i].m_aoATheta;			//�����Ȩ�Ƕ�theta
	}

	//ʱ�Ӻϳɣ����չ�������ʱ����Ϊ��ʱ��
	m_mergedInfo.m_timeDelay = m_infos.front().m_timeDelay;

	//���ʺϳɣ����ձ������ʽ��кϳ�
	RtLbsType linearPower = 0.0;														/** @brief	���Թ���	,W*/
	for (auto curInfo : m_infos) {
		linearPower += pow(10, curInfo.m_power / 10);
	}
	m_mergedInfo.m_power = 10 * log10(linearPower);										//ת���ϳɹ���
}

void PathInfoCluster::CalMergedInfoByDelay()
{
	//��ʱ����Ϣ���кϲ�
	std::sort(m_infos.begin(), m_infos.end(), ComparedByPower_PathInfo);				//���չ��ʴ�С��������
	std::vector<RtLbsType> powerWeight;													/** @brief	����Ȩ��	*/
	powerWeight.reserve(m_infos.size());												//Ԥ���ռ�
	RtLbsType sumPowerWeight = 0.0;														/** @brief	����Ȩ�غ�	*/
	for (auto curInfo : m_infos) {
		RtLbsType curPowerWeight = pow(10, (curInfo.m_power - m_infos.front().m_power) / 10.0);
		powerWeight.push_back(curPowerWeight);
		sumPowerWeight += curPowerWeight;
	}
	for (auto w : powerWeight) {
		w /= sumPowerWeight;
	}
	for (int i = 0; i < static_cast<int>(m_infos.size()); ++i) {						//�����Ȩʱ��
		m_mergedInfo.m_timeDelay += powerWeight[i] * m_infos[i].m_timeDelay;
	}

	//���ʺϳɣ����ձ������ʽ��кϳ�
	RtLbsType linearPower = 0.0;														/** @brief	���Թ���	,W*/
	for (auto curInfo : m_infos) {
		linearPower += pow(10, curInfo.m_power / 10);
	}
	m_mergedInfo.m_power = 10 * log10(linearPower);										//ת���ϳɹ���
}


