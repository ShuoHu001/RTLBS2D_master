#ifndef RTLBS_GSPAIRCLUSTER
#define RTLBS_GSPAIRCLUSTER

#include "rtlbs.h"
#include "utility/define.h"
#include "result/raytracingresult.h"
#include "scene/scene.h"
#include "gspair.h"
#include "general/elevationmatrix.h"

class GSPairCluster {
public:
	bool m_isValid;													/** @brief	簇内元素是否有效，对于定位来说	*/
	bool m_isDeviateSolution;										/** @brief	是否是偏离解，解与广义源对的关联性不大，是由m_aroundPoints所提供的解	*/	
	bool m_isRefGSOccupied;											/** @brief	参考广义源坐标是否被占用，若占用则不再进行赋值	*/
	int m_nearExtendNum;											/** @brief	近距离扩展点数量	*/
	int m_farExtendNum;												/** @brief	远距离扩展点数量	*/
	RtLbsType m_deviateDistance;									/** @brief	偏离距离	*/
	std::vector<GSPair*> m_pairs;									/** @brief	簇内广义源对	*/
	Point2D m_point;												/** @brief	平均坐标	*/
	Point2D m_TDOA_REFGS_Point;										/** @brief	含TDOA定位算法中需要结合RTresult结果计算参考广义源坐标	*/
	std::vector<Point2D> m_aroundPoints;							/** @brief	周围的坐标	*/
	RtLbsType m_residualFactor;										/** @brief	残差因子	*/
	RtLbsType m_residual;											/** @brief	残差	*/
	std::vector<std::vector<RaytracingResult>> m_rtResult;			/** @brief	需要计算的射线追踪结果，第一维度为解的数量，第二维度为传感器的数量	*/
	RtLbsType m_angularSpreadResidual;
	RtLbsType m_delaySpreadRedisual;
	RtLbsType m_phiResidual;
	RtLbsType m_timeResidual;
	RtLbsType m_timeDiffResidual;
	RtLbsType m_powerResidual;
	RtLbsType m_powerDiffResidual;
	int m_nullDataNum;
	RtLbsType m_weight;

public:
	GSPairCluster();
	GSPairCluster(const GSPairCluster& cluster);
	~GSPairCluster();
	GSPairCluster operator = (const GSPairCluster& cluster);
	void UpdateGSPairBelongingCluster();																				//更新gspair的cluster
	void ExtendAroundPoint(bool expandFlag, const ElevationMatrix& lbsShiftErrorMatrix, const Scene* scene);													//扩展近邻点，true表示进行扩展，false表示不进行扩展，只保留一个
	RtLbsType CalTDOAResidualFactor();																					//计算残差因子
	void CalTDOAResidual(RtLbsType maxResidualFactor);																	//计算残差
	bool CanAddToClusterByDistance(GSPair* pair, RtLbsType threshold);
	void CalClusterPosition();																							//计算簇内平均坐标点
	void SetElementClusterId(int Id);																					//设置簇内元素的ID
	void SetElementAOAResidual(RtLbsType r_phi, RtLbsType r_powerDiff, RtLbsType r_angularSpreadResidual, int nullDataNum);								//设置AOA型残差
	void SetElementTOAResidual(RtLbsType r_time, RtLbsType r_power, int nullDataNum);
	void SetElementTDOAResidual(RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum);							//设置TDOA型残差
	void SetElementAOATDOAResidual(RtLbsType r_phi, RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum);		//设置AOA/TDOA型残差
	void SetElementAOATOAResidual(RtLbsType r_phi, RtLbsType r_time, RtLbsType r_power, int nullDataNum);			//设置AOA/TOA型残差
	void SetInValidState();																								//设置为无效
	void CalNormalizedAOAWeight(RtLbsType max_r_phi, RtLbsType max_r_powerDiff, const WeightFactor& w);											//计算簇AOA归一权重
	void CalNormalizedTDOAWeight(RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w);									//计算簇TDOA归一权重
	void CalNormalizedAOATDOAWeight(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w);			//计算簇AOA-TDOA归一权重
	void GetNonRepeatGeneralSource(std::vector<GeneralSource*>& sources);																		//删除重复的广义源对及其广义源对中的广义源，主要用于角度平均
	void GetNonRepeatGeneralSource(GeneralSource* refSource, std::vector<GeneralSource*>& sources);												//删除重复的广义源，主要用于求解AOATDOA定位算法中的有效参考广义源对中的广义源
	void CalculateRefSourceCount() const;																										//计算参考广义源
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
inline std::vector<GSPairCluster> ClusterGSPairByDistance(std::vector<GSPair*>& pairs, const Scene* scene, RtLbsType threshold, bool extendAroundPointState, const ElevationMatrix& lbsShiftErrorMatrix) {
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
	}

	//按照簇数量大小进行排序
	std::sort(clusters.begin(), clusters.end(), ComparedByClusterSize);

	//扩展周边点数据&更新父cluster
	for (auto& cluster : clusters) {
		cluster.ExtendAroundPoint(extendAroundPointState, lbsShiftErrorMatrix, scene);
		cluster.UpdateGSPairBelongingCluster();
	}

	return clusters;
}

#endif
