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
	//���㹦��Ȩ��ϵ��(��ͬʱ���й��ʺϳ�)
	std::sort(m_infos.begin(), m_infos.end(), ComparedByPower_PathInfo);				//���չ��ʴ�С��������
	std::vector<RtLbsType> powerWeight;													/** @brief	����Ȩ��	*/
	powerWeight.reserve(m_infos.size());												//Ԥ���ռ�
	RtLbsType sumPowerWeight = 0.0;														/** @brief	����Ȩ�غ�	*/
	for (auto& curInfo : m_infos) {
		RtLbsType curPowerWeight = pow(10, curInfo.m_power / 10.0);
		powerWeight.push_back(curPowerWeight);
		sumPowerWeight += curPowerWeight;
	}	
	for (auto& w : powerWeight) {
		w /= sumPowerWeight;
	}

	m_mergedInfo.m_power = 10 * log10(sumPowerWeight);									//ת���ϳɹ���

	RtLbsType aoaPhi = 0.0;
	RtLbsType aoaTheta = 0.0;
	for (int i = 0; i < static_cast<int>(m_infos.size()); ++i) {						
		aoaPhi += powerWeight[i] * m_infos[i].m_aoAPhi;				//�����Ȩ�Ƕ�phi
		aoaTheta += powerWeight[i] * m_infos[i].m_aoATheta;			//�����Ȩ�Ƕ�theta
	}

	m_mergedInfo.m_aoAPhi = aoaPhi;
	m_mergedInfo.m_aoATheta = aoaTheta;

	//ʱ�Ӻϳɣ����չ�������ʱ����Ϊ��ʱ��
	m_mergedInfo.m_timeDelay = m_infos.front().m_timeDelay;
}

void PathInfoCluster::CalMergedInfoByDelay()
{
	//���㹦��Ȩ��-��ͬʱ����ϳɹ���
	std::sort(m_infos.begin(), m_infos.end(), ComparedByPower_PathInfo);				//���չ��ʴ�С��������
	std::vector<RtLbsType> powerWeight;													/** @brief	����Ȩ��	*/
	powerWeight.reserve(m_infos.size());												//Ԥ���ռ�
	RtLbsType sumPowerWeight = 0.0;														/** @brief	����Ȩ�غ�	*/
	for (auto& curInfo : m_infos) {
		RtLbsType curPowerWeight = pow(10, curInfo.m_power / 10.0);
		powerWeight.push_back(curPowerWeight);
		sumPowerWeight += curPowerWeight;
	}
	for (auto& w : powerWeight) {
		w /= sumPowerWeight;
	}

	m_mergedInfo.m_power = 10 * log10(sumPowerWeight);									//ת���ϳɹ���

	RtLbsType mergedTimeDelay = 0.0;
	for (int i = 0; i < static_cast<int>(m_infos.size()); ++i) {						//�����Ȩʱ��
		 mergedTimeDelay += powerWeight[i] * m_infos[i].m_timeDelay;
	}
	m_mergedInfo.m_timeDelay = mergedTimeDelay;
	m_mergedInfo.m_aoAPhi = m_infos[0].m_aoAPhi;										//��ȡ���ྶ���ĽǶ�ֵ
	
}


