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
	//计算info与cluster中心数据的距离，若存在大于距离门限的数值，则返回false
	bool addFlag = true;
	if (m_mergedInfo.DistanceAOA2D(info) > threshold) {
		addFlag = false;
	}
	return addFlag;
}

bool PathInfoCluster::CanAddToClusterByAOA3D(const PathInfo& info, RtLbsType threshold) const
{
	//计算info与cluster中心数据的距离，若存在大于距离门限的数值，则返回false
	bool addFlag = true;
	if (m_mergedInfo.DistanceAOA3D(info) > threshold) {
		addFlag = false;
	}
	return addFlag;
}

bool PathInfoCluster::CanAddToClusterByDelay(const PathInfo& info, RtLbsType threshold) const
{
	//计算info与cluster中心数据的距离，若存在大于距离门限的数值，则返回false
	bool addFlag = true;
	if (m_mergedInfo.DistanceDelay(info) > threshold) {
		addFlag = false;
	}
	return addFlag;
}

void PathInfoCluster::CalMergedInfoByAOA()
{
	//计算功率权重系数
	std::sort(m_infos.begin(), m_infos.end(), ComparedByPower_PathInfo);				//按照功率大小进行排序
	std::vector<RtLbsType> powerWeight;													/** @brief	功率权重	*/
	powerWeight.reserve(m_infos.size());												//预留空间
	RtLbsType sumPowerWeight = 0.0;														/** @brief	功率权重和	*/
	for (auto curInfo : m_infos) {
		RtLbsType curPowerWeight = pow(10, (curInfo.m_power - m_infos.front().m_power) / 10.0);
		powerWeight.push_back(curPowerWeight);
		sumPowerWeight += curPowerWeight;
	}	
	for (auto w : powerWeight) {
		w /= sumPowerWeight;
	}
	for (int i = 0; i < static_cast<int>(m_infos.size()); ++i) {						
		m_mergedInfo.m_aoAPhi += powerWeight[i] * m_infos[i].m_aoAPhi;				//计算加权角度phi
		m_mergedInfo.m_aoATheta += powerWeight[i] * m_infos[i].m_aoATheta;			//计算加权角度theta
	}

	//时延合成，按照功率最大的时延作为其时延
	m_mergedInfo.m_timeDelay = m_infos.front().m_timeDelay;

	//功率合成，按照标量功率进行合成
	RtLbsType linearPower = 0.0;														/** @brief	线性功率	,W*/
	for (auto curInfo : m_infos) {
		linearPower += pow(10, curInfo.m_power / 10);
	}
	m_mergedInfo.m_power = 10 * log10(linearPower);										//转换合成功率
}

void PathInfoCluster::CalMergedInfoByDelay()
{
	//对时延信息进行合并
	std::sort(m_infos.begin(), m_infos.end(), ComparedByPower_PathInfo);				//按照功率大小进行排序
	std::vector<RtLbsType> powerWeight;													/** @brief	功率权重	*/
	powerWeight.reserve(m_infos.size());												//预留空间
	RtLbsType sumPowerWeight = 0.0;														/** @brief	功率权重和	*/
	for (auto curInfo : m_infos) {
		RtLbsType curPowerWeight = pow(10, (curInfo.m_power - m_infos.front().m_power) / 10.0);
		powerWeight.push_back(curPowerWeight);
		sumPowerWeight += curPowerWeight;
	}
	for (auto w : powerWeight) {
		w /= sumPowerWeight;
	}
	for (int i = 0; i < static_cast<int>(m_infos.size()); ++i) {						//计算加权时延
		m_mergedInfo.m_timeDelay += powerWeight[i] * m_infos[i].m_timeDelay;
	}

	//功率合成，按照标量功率进行合成
	RtLbsType linearPower = 0.0;														/** @brief	线性功率	,W*/
	for (auto curInfo : m_infos) {
		linearPower += pow(10, curInfo.m_power / 10);
	}
	m_mergedInfo.m_power = 10 * log10(linearPower);										//转换合成功率
}


