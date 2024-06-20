#ifndef RTLBS_PATHINFOCLUSTER
#define RTLBS_PATHINFOCLUSTER

#include "rtlbs.h"
#include "utility/define.h"
#include "pathinfo.h"


//路径信息，用于聚类使用
class PathInfoCluster {
public:
	std::vector<PathInfo> m_infos;								/** @brief	需要类中的信息集合	*/
	PathInfo m_mergedInfo;										/** @brief	合并的信息	*/

public:
	PathInfoCluster();
	PathInfoCluster(const PathInfoCluster& cluster);
	~PathInfoCluster();
	PathInfoCluster& operator = (const PathInfoCluster& cluster);
	bool CanAddToClusterByAOA2D(const PathInfo& info, RtLbsType threshold) const;			//按照二维角度是否可纳入类
	bool CanAddToClusterByAOA3D(const PathInfo& info, RtLbsType threshold) const;			//按照三维角度是否可纳入类
	bool CanAddToClusterByDelay(const PathInfo& info, RtLbsType threshold) const;			//按照时延是否可纳入类
	void CalMergedInfoByAOA();																//计算合成信息,优先按照角度信息来
	void CalMergedInfoByDelay();															//计算合成信息,优先按照时延信息来
};

//按照功率大小进行排序，从大到小
inline bool ComparedByPower_PathInfoCluster(const PathInfoCluster& cluster1, const PathInfoCluster& cluster2) {
	return cluster1.m_mergedInfo.m_power > cluster2.m_mergedInfo.m_power;
}

//按照时延的大小进行排序，从小到大
inline bool ComparedByDelay_PathInfoCluster(const PathInfoCluster& cluster1, const PathInfoCluster& cluster2) {
	return cluster1.m_mergedInfo.m_timeDelay < cluster2.m_mergedInfo.m_timeDelay;
}

//按照AOA二维角度进行聚类
inline std::vector<PathInfoCluster> ClusterPathInfoByAOA2D(std::vector<PathInfo>& infos, RtLbsType threshold)
{
	//先进行聚类
	std::vector<PathInfoCluster> clusters;								/** @brief	需要聚成的类集合	*/

	for (auto& curInfo : infos) {
		bool addFlag = false;
		for (auto& curCluster : clusters) {
			if (curCluster.CanAddToClusterByAOA2D(curInfo, threshold)) {
				curCluster.m_infos.push_back(curInfo);					//重新计算簇中心
				curCluster.CalMergedInfoByAOA();
				addFlag = true;
			}
		}
		if (!addFlag) {													//若没在当前存在的cluster中聚类，则应该构建新类
			PathInfoCluster newCluster;
			newCluster.m_infos.push_back(curInfo);
			newCluster.CalMergedInfoByAOA();							//重新计算簇中心
			clusters.push_back(newCluster);
		}
	}

	return clusters;
}

//按照AOA三维角度进行聚类
inline std::vector<PathInfoCluster> ClusterPathInfoByAOA3D(std::vector<PathInfo>& infos, RtLbsType threshold)
{
	std::vector<PathInfoCluster> clusters;								/** @brief	需要聚成的类集合	*/

	for (auto& curInfo : infos) {
		bool addFlag = false;
		for (auto& curCluster : clusters) {
			if (curCluster.CanAddToClusterByAOA3D(curInfo, threshold)) {
				curCluster.m_infos.push_back(curInfo);
				curCluster.CalMergedInfoByAOA();
				addFlag = true;
			}
		}
		if (!addFlag) {													//若没在当前存在的cluster中聚类，则应该构建新类
			PathInfoCluster newCluster;
			newCluster.m_infos.push_back(curInfo);
			newCluster.CalMergedInfoByAOA();
			clusters.push_back(newCluster);
		}
	}

	return clusters;
}

//按照时延进行聚类
inline std::vector<PathInfoCluster> ClusterPathInfoByDelay(std::vector<PathInfo>& infos, RtLbsType threshold)
{
	std::vector<PathInfoCluster> clusters;								/** @brief	需要聚成的类集合	*/

	for (auto& curInfo : infos) {
		bool addFlag = false;
		for (auto& curCluster : clusters) {
			if (curCluster.CanAddToClusterByDelay(curInfo, threshold)) {
				curCluster.m_infos.push_back(curInfo);
				curCluster.CalMergedInfoByDelay();
				addFlag = true;
			}
		}
		if (!addFlag) {													//若没在当前存在的cluster中聚类，则应该构建新类
			PathInfoCluster newCluster;
			newCluster.m_infos.push_back(curInfo);
			newCluster.CalMergedInfoByDelay();
			clusters.push_back(newCluster);
		}
	}

	return clusters;
}
#endif
