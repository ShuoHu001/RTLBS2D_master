#ifndef RTLBS_GSPAIRCLUSTER
#define RTLBS_GSPAIRCLUSTER

#include "rtlbs.h"
#include "utility/define.h"
#include "gspair.h"


class GSPairCluster {
public:
	bool m_isValid;							/** @brief	簇内元素是否有效，对于定位来说	*/
	std::vector<GSPair*> m_pairs;			/** @brief	簇内广义源对	*/
	Point2D m_point;						/** @brief	平均坐标	*/

public:
	GSPairCluster();
	GSPairCluster(const GSPairCluster& cluster);
	~GSPairCluster();
	GSPairCluster operator = (const GSPairCluster& cluster);
	bool CanAddToClusterByDistance(GSPair* pair, RtLbsType threshold);
	void CalClusterPosition();																							//计算簇内平均坐标点
	void SetElementClusterId(int Id);																					//设置簇内元素的ID
	void SetElementAOAResidual(RtLbsType r_phi, RtLbsType r_powerDiff, int nullDataNum);								//设置AOA型残差
	void SetElementTDOAResidual(RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum);							//设置TDOA型残差
	void SetElementAOATDOAResidual(RtLbsType r_phi, RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum);		//设置AOA/TDOA型残差
};


//按照距离进行聚类
inline std::vector<GSPairCluster> ClusterGSPairByDistance(std::vector<GSPair*>& pairs, RtLbsType threshold) {
	std::vector<GSPairCluster> clusters;

	for (auto& curPair : pairs) {
		bool addFlag = false;
		for (auto& curCluster : clusters) {
			if (curCluster.CanAddToClusterByDistance(curPair, threshold)) {
				curCluster.m_pairs.push_back(curPair);
				curCluster.CalClusterPosition();					//簇内新增元素后计算其簇心坐标
				addFlag = true;
			}
		}
		if (!addFlag) {
			GSPairCluster newCluster;
			newCluster.m_pairs.push_back(curPair);
			newCluster.CalClusterPosition();						//簇内新加元素计算其簇心坐标
			clusters.push_back(newCluster);
		}
	}

	//更新簇内所有元素的簇ID
	for (int i = 0; i < static_cast<int>(clusters.size()); ++i) {
		clusters[i].SetElementClusterId(i);
	}

	return clusters;
}

#endif
