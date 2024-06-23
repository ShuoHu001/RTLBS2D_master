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

inline bool ComparedByClusterSize(const GSPairCluster& cluster1, const GSPairCluster& cluster2) {
	return cluster1.m_pairs.size() > cluster2.m_pairs.size();
}

//按照距离进行聚类
inline std::vector<GSPairCluster> ClusterGSPairByDistance(std::vector<GSPair*>& pairs, RtLbsType threshold, int& maxCluterNum) {
	std::vector<GSPairCluster> clusters;

	for (auto& curPair : pairs) {
		bool addFlag = false;
		for (auto& curCluster : clusters) {
			if (curCluster.CanAddToClusterByDistance(curPair, threshold)) {
				curCluster.m_pairs.push_back(curPair);
				curCluster.CalClusterPosition();					//簇内新增元素后计算其簇心坐标
				addFlag = true;
				break;												//加簇后需要及时跳出
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
		if (maxCluterNum < static_cast<int>(clusters[i].m_pairs.size())) {
			maxCluterNum = static_cast<int>(clusters[i].m_pairs.size());
		}
	}

	//按照簇数量大小进行排序
	std::sort(clusters.begin(), clusters.end(), ComparedByClusterSize);
	return clusters;
}

//计算给定参考广义源下的聚类pair值最优的簇
inline GSPairCluster CalMaxTDOASolutionGSPairCluster(GeneralSource* refSource, const std::vector<GeneralSource*>& sources, const std::vector<SensorData>& sensorDatas, const Scene* scene, RtLbsType threshold) {
	std::vector<GeneralSource*> sourceCopy;
	for (auto& source : sources) {
		if (source != refSource) {
			sourceCopy.push_back(source);
		}
	}

	int sensorDataNum = static_cast<int>(sensorDatas.size());
	//扩展广义源
	std::vector<GeneralSource*> newSources(sourceCopy.size() * (sensorDataNum - 1));
	int sId = 0;
	for (int j = 1; j < sensorDataNum; ++j) {
		const SensorData& curSensorData = sensorDatas[j];
		for (int i = 0; i < static_cast<int>(sourceCopy.size()); ++i) {
			if (sourceCopy[i]->m_depth < refSource->m_depth) {									//在设定参考站(能量值最大的站)后，其他数据站的深度应大于等于参考站的深度
				continue;
			}
			if (refSource->m_type == NODE_DIFF) {
				if (sourceCopy[i]->m_type != NODE_DIFF) {
					if (sourceCopy[i]->m_depth < refSource->m_depth + 1) {							//若参考站为绕射节点，那么数据站深度应大于参考站深度+1
						continue;
					}
				}
			}
			newSources[sId] = new GeneralSource(*sourceCopy[i]);
			newSources[sId]->m_sensorData = curSensorData;
			sId++;
		}
	}
	newSources.resize(sId);

	int newSourceNum = static_cast<int>(newSources.size());						/** @brief	新广义源的数量	*/
	//创建广义源对
	int gsPairNum = static_cast<int>(newSourceNum * (newSourceNum - 1) / 2);
	std::vector<GSPair*> pairs(gsPairNum);
	int pairId = 0;
	for (int i = 0; i < newSourceNum; ++i) {
		for (int j = i + 1; j < newSourceNum; ++j) {
			GSPair* pair = new GSPair(refSource, newSources[i], newSources[j]);
			if (!pair->HasValidTDOASolution(scene)) {
				delete pair;
				continue;
			}
			pairs[pairId++] = pair;
		}
	}
	pairs.resize(pairId);
	pairs.shrink_to_fit();

	if (pairId == 0) {
		GSPairCluster zeroCluster;
		return zeroCluster;
	}

	//删除权重为0的广义源
	for (auto& curGS : newSources) {
		if (!curGS->IsValid()) {
			delete curGS;
			curGS = nullptr;
		}
	}

	newSources.erase(std::remove_if(newSources.begin(), newSources.end(), [](const GeneralSource* source) {
		return source == nullptr;
		}), newSources.end());


	//对pair中的解进行刨根问底追根溯源，找到对应的路径，计算TDOA差值和能量差值，搜索在cluster中残差最小的一份，作为目前的最优解

	//对解进行初步聚类，得到聚类解
	int max_cluster_num = 1;
	std::vector<GSPairCluster> gsPairCluster = ClusterGSPairByDistance(pairs, threshold, max_cluster_num);
	return gsPairCluster.front();
}

#endif
