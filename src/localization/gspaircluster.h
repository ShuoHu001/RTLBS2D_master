#ifndef RTLBS_GSPAIRCLUSTER
#define RTLBS_GSPAIRCLUSTER

#include "rtlbs.h"
#include "utility/define.h"
#include "result/raytracingresult.h"
#include "geometry/scene.h"
#include "global/globalvariables.h"
#include "gspair.h"

class GSPairCluster {
public:
	bool m_isValid;													/** @brief	簇内元素是否有效，对于定位来说	*/
	bool m_isDeviateSolution;										/** @brief	是否是偏离解，解与广义源对的关联性不大，是由m_aroundPoints所提供的解	*/	
	int m_nearExtendNum;											/** @brief	近距离扩展点数量	*/
	int m_farExtendNum;												/** @brief	远距离扩展点数量	*/
	RtLbsType m_deviateDistance;									/** @brief	偏离距离	*/
	std::vector<GSPair*> m_pairs;									/** @brief	簇内广义源对	*/
	Point2D m_point;												/** @brief	平均坐标	*/
	std::vector<Point2D> m_aroundPoints;							/** @brief	周围的坐标	*/
	RtLbsType m_residualFactor;										/** @brief	残差因子	*/
	RtLbsType m_residual;											/** @brief	残差	*/
	std::vector<std::vector<RaytracingResult>> m_rtResult;			/** @brief	需要计算的射线追踪结果，第一维度为解的数量，第二维度为传感器的数量	*/
	RtLbsType m_angularSpreadResidual;
	RtLbsType m_delaySpreadRedisual;
	RtLbsType m_phiResidual;
	RtLbsType m_timeResidual;
	RtLbsType m_timeDiffResidual;
	RtLbsType m_powerDiffResidual;
	int m_nullDataNum;
	RtLbsType m_weight;

public:
	GSPairCluster();
	GSPairCluster(const GSPairCluster& cluster);
	~GSPairCluster();
	GSPairCluster operator = (const GSPairCluster& cluster);
	void UpdateGSPairBelongingCluster();																				//更新gspair的cluster
	void ExtendAroundPoint(bool expandFlag, const Scene* scene);													//扩展近邻点，true表示进行扩展，false表示不进行扩展，只保留一个
	RtLbsType CalTDOAResidualFactor();																					//计算残差因子
	void CalTDOAResidual(RtLbsType maxResidualFactor);																	//计算残差
	bool CanAddToClusterByDistance(GSPair* pair, RtLbsType threshold);
	void CalClusterPosition();																							//计算簇内平均坐标点
	void SetElementClusterId(int Id);																					//设置簇内元素的ID
	void SetElementAOAResidual(RtLbsType r_phi, RtLbsType r_powerDiff, RtLbsType r_angularSpreadResidual, int nullDataNum);								//设置AOA型残差
	void SetElementTDOAResidual(RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum);							//设置TDOA型残差
	void SetElementAOATDOAResidual(RtLbsType r_phi, RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum);		//设置AOA/TDOA型残差
	void SetInValidState();																								//设置为无效
	void CalNormalizedAOAWeight(RtLbsType max_r_phi, RtLbsType max_r_powerDiff, const WeightFactor& w);											//计算簇AOA归一权重
	void CalNormalizedTDOAWeight(RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w);									//计算簇TDOA归一权重
	void CalNormalizedAOATDOAWeight(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w);			//计算簇AOA-TDOA归一权重
	void GetNonRepeatGeneralSource(std::vector<GeneralSource*>& sources);																		//删除重复的广义源对及其广义源对中的广义源
};

inline bool ComparedByClusterSize(const GSPairCluster& cluster1, const GSPairCluster& cluster2) {
	return cluster1.m_pairs.size() > cluster2.m_pairs.size();
}

inline bool ComparedByClusterPairWeight(const GSPairCluster& cluster1, const GSPairCluster& cluster2) {
	return cluster1.m_pairs.front()->m_weight > cluster2.m_pairs.front()->m_weight;
}

inline bool ComparedByClusterWeight(const GSPairCluster& cluster1, const GSPairCluster& cluster2) {
	return cluster1.m_weight > cluster2.m_weight;
}

inline bool ComparedByClusterResidual(const GSPairCluster& cluster1, const GSPairCluster& cluster2) {
	return cluster1.m_residual < cluster2.m_residual;
}

//按照距离进行聚类
inline std::vector<GSPairCluster> ClusterGSPairByDistance(std::vector<GSPair*>& pairs, const Scene* scene, RtLbsType threshold, bool extendAroundPointState, int& maxCluterNum) {
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
	int id = 0;
	for (auto& curCluster : clusters) {
		curCluster.SetElementClusterId(id++);
		if (maxCluterNum < static_cast<int>(curCluster.m_pairs.size())) {
			maxCluterNum = static_cast<int>(curCluster.m_pairs.size());
		}
	}

	//按照簇数量大小进行排序
	std::sort(clusters.begin(), clusters.end(), ComparedByClusterSize);

	//扩展周边点数据&更新父cluster
	for (auto& cluster : clusters) {
		cluster.ExtendAroundPoint(extendAroundPointState, scene);
		cluster.UpdateGSPairBelongingCluster();
	}

	return clusters;
}

//计算给定参考广义源下的聚类pair值最优的簇
inline GSPairCluster CalMaxTDOASolutionGSPairCluster_SPSTMD(GeneralSource* refSource, const std::vector<GeneralSource*>& sources, const std::vector<SensorData>& sensorDatas, const Scene* scene, RtLbsType threshold, bool extendAroundPointState, RtLbsType freq) {
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
			if (newSources[i]->m_sensorData.m_id == newSources[j]->m_sensorData.m_id) {				//相同数据的广义源不纳入
				continue;
			}
			GSPair* pair = new GSPair(refSource, newSources[i], newSources[j]);
			if (!pair->HasValidTDOASolution_SPSTMD(scene, freq)) {
				delete pair;
				continue;
			}
			pairs[pairId++] = pair;
		}
	}
	pairs.resize(pairId);
	std::vector<GSPair*>().swap(pairs);

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


	//对解进行初步聚类，得到聚类解
	int max_cluster_num = 1;
	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(pairs, scene, threshold, extendAroundPointState, max_cluster_num);

	RtLbsType max_rFactor = 0.0;											/** @brief	最大残差因子	*/
	for (auto& curCluster : gsPairClusters) {								//求解残差因子，给出最大的残差因子
		RtLbsType cur_rFactor = curCluster.CalTDOAResidualFactor();
		if (max_rFactor < cur_rFactor) {
			max_rFactor = cur_rFactor;
		}
	}

	for (auto& curCluster : gsPairClusters) {
		curCluster.CalTDOAResidual(max_rFactor);
	}

	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterSize);

	return gsPairClusters.front();
}


inline GSPairCluster CalMaxTDOASolutionGSPairCluster_MPSTSD(GeneralSource* refSource, const std::vector<GeneralSource*>& sources, const Scene* scene, RtLbsType threshold, bool extendAroundPointState, RtLbsType freq) {
	std::vector<GeneralSource*> newSources;
	for (auto& source : sources) {
		if (source != refSource) {
			newSources.push_back(source);
		}
	}


	//创建广义源对
	int newSourceNum = static_cast<int>(sources.size());
	int gsPairNum = static_cast<int>(newSourceNum * (newSourceNum - 1) / 2);
	std::vector<GSPair*> pairs(gsPairNum);
	int pairId = 0;
	for (int i = 0; i < newSourceNum; ++i) {
		for (int j = i + 1; j < newSourceNum; ++j) {
			if (newSources[i]->m_sensorData.m_id == newSources[j]->m_sensorData.m_id) {				//相同数据的广义源不纳入
				continue;
			}
			GSPair* pair = new GSPair(refSource, newSources[i], newSources[j]);
			if (!pair->HasValidTDOASolution_MPSTSD(scene)) {
				delete pair;
				continue;
			}
			pairs[pairId++] = pair;
		}
	}
	pairs.resize(pairId);
	std::vector<GSPair*>().swap(pairs);


	

	if (pairId == 0) {
		GSPairCluster zeroCluster;
		return zeroCluster;
	}


	//对解进行初步聚类，得到聚类解
	int max_cluster_num = 1;
	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(pairs, scene, threshold, extendAroundPointState, max_cluster_num);

	//排除大部分数据后，再通过射线追踪求解点到两个传感器之间的功率
	return gsPairClusters.front();
}
#endif
