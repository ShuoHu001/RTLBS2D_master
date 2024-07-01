#ifndef RTLBS_GSPAIRCLUSTER
#define RTLBS_GSPAIRCLUSTER

#include "rtlbs.h"
#include "utility/define.h"
#include "gspair.h"
#include "result/raytracingresult.h"
#include "geometry/scene.h"


class GSPairCluster {
public:
	bool m_isValid;													/** @brief	����Ԫ���Ƿ���Ч�����ڶ�λ��˵	*/
	std::vector<GSPair*> m_pairs;									/** @brief	���ڹ���Դ��	*/
	Point2D m_point;												/** @brief	ƽ������	*/
	std::vector<Point2D> m_aroundPoints;							/** @brief	��Χ������	*/
	RtLbsType m_residualFactor;										/** @brief	�в�����	*/
	RtLbsType m_residual;											/** @brief	�в�	*/
	std::vector<std::vector<RaytracingResult>> m_rtResult;			/** @brief	��Ҫ���������׷�ٽ������һά��Ϊ����������ڶ�ά��Ϊ������������	*/
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
	void ExtendNearPoint(bool expandFlag, const Scene* scene);													//��չ���ڵ㣬true��ʾ������չ��false��ʾ��������չ��ֻ����һ��
	RtLbsType CalTDOAResidualFactor();																					//����в�����
	void CalTDOAResidual(RtLbsType maxResidualFactor);																	//����в�
	bool CanAddToClusterByDistance(GSPair* pair, RtLbsType threshold);
	void CalClusterPosition();																							//�������ƽ�������
	void SetElementClusterId(int Id);																					//���ô���Ԫ�ص�ID
	void SetElementAOAResidual(RtLbsType r_phi, RtLbsType r_powerDiff, int nullDataNum);								//����AOA�Ͳв�
	void SetElementTDOAResidual(RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum);							//����TDOA�Ͳв�
	void SetElementAOATDOAResidual(RtLbsType r_phi, RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum);		//����AOA/TDOA�Ͳв�
	void SetInValidState();																								//����Ϊ��Ч
	void CalNormalizedAOAWeight(RtLbsType max_r_phi, RtLbsType max_r_powerDiff, const WeightFactor& w);											//�����AOA��һȨ��
	void CalNormalizedTDOAWeight(RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w);									//�����TDOA��һȨ��
	void CalNormalizedAOATDOAWeight(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w);			//�����AOA-TDOA��һȨ��
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

//���վ�����о���
inline std::vector<GSPairCluster> ClusterGSPairByDistance(std::vector<GSPair*>& pairs, const Scene* scene, RtLbsType threshold, int& maxCluterNum) {
	std::vector<GSPairCluster> clusters;

	for (auto& curPair : pairs) {
		bool addFlag = false;
		for (auto& curCluster : clusters) {
			if (curCluster.CanAddToClusterByDistance(curPair, threshold)) {
				curCluster.m_pairs.push_back(curPair);
				curCluster.CalClusterPosition();					//��������Ԫ�غ�������������
				addFlag = true;
				break;												//�Ӵغ���Ҫ��ʱ����
			}
		}
		if (!addFlag) {
			GSPairCluster newCluster;
			newCluster.m_pairs.push_back(curPair);
			newCluster.CalClusterPosition();						//�����¼�Ԫ�ؼ������������
			clusters.push_back(newCluster);
		}
	}

	//���´�������Ԫ�صĴ�ID
	for (int i = 0; i < static_cast<int>(clusters.size()); ++i) {
		clusters[i].SetElementClusterId(i);
		if (maxCluterNum < static_cast<int>(clusters[i].m_pairs.size())) {
			maxCluterNum = static_cast<int>(clusters[i].m_pairs.size());
		}
	}

	//���մ�������С��������
	std::sort(clusters.begin(), clusters.end(), ComparedByClusterSize);

	//��չ�ܱߵ�����
	for (auto& cluster : clusters) {
		cluster.ExtendNearPoint(true, scene);
	}

	return clusters;
}

//��������ο�����Դ�µľ���pairֵ���ŵĴ�
inline GSPairCluster CalMaxTDOASolutionGSPairCluster_SPSTMD(GeneralSource* refSource, const std::vector<GeneralSource*>& sources, const std::vector<SensorData>& sensorDatas, const Scene* scene, RtLbsType threshold, RtLbsType freq, const std::vector<Complex>& tranFunction) {
	std::vector<GeneralSource*> sourceCopy;
	for (auto& source : sources) {
		if (source != refSource) {
			sourceCopy.push_back(source);
		}
	}

	int sensorDataNum = static_cast<int>(sensorDatas.size());
	//��չ����Դ
	std::vector<GeneralSource*> newSources(sourceCopy.size() * (sensorDataNum - 1));
	int sId = 0;
	for (int j = 1; j < sensorDataNum; ++j) {
		const SensorData& curSensorData = sensorDatas[j];
		for (int i = 0; i < static_cast<int>(sourceCopy.size()); ++i) {
			if (sourceCopy[i]->m_depth < refSource->m_depth) {									//���趨�ο�վ(����ֵ����վ)����������վ�����Ӧ���ڵ��ڲο�վ�����
				continue;
			}
			if (refSource->m_type == NODE_DIFF) {
				if (sourceCopy[i]->m_type != NODE_DIFF) {
					if (sourceCopy[i]->m_depth < refSource->m_depth + 1) {							//���ο�վΪ����ڵ㣬��ô����վ���Ӧ���ڲο�վ���+1
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

	int newSourceNum = static_cast<int>(newSources.size());						/** @brief	�¹���Դ������	*/
	//��������Դ��
	int gsPairNum = static_cast<int>(newSourceNum * (newSourceNum - 1) / 2);
	std::vector<GSPair*> pairs(gsPairNum);
	int pairId = 0;
	for (int i = 0; i < newSourceNum; ++i) {
		for (int j = i + 1; j < newSourceNum; ++j) {
			if (newSources[i]->m_sensorData.m_id == newSources[j]->m_sensorData.m_id) {				//��ͬ���ݵĹ���Դ������
				continue;
			}
			GSPair* pair = new GSPair(refSource, newSources[i], newSources[j]);
			if (!pair->HasValidTDOASolution_SPSTMD(scene, freq, tranFunction)) {
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

	//ɾ��Ȩ��Ϊ0�Ĺ���Դ
	for (auto& curGS : newSources) {
		if (!curGS->IsValid()) {
			delete curGS;
			curGS = nullptr;
		}
	}

	newSources.erase(std::remove_if(newSources.begin(), newSources.end(), [](const GeneralSource* source) {
		return source == nullptr;
		}), newSources.end());


	//�Խ���г������࣬�õ������
	int max_cluster_num = 1;
	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(pairs, scene, threshold, max_cluster_num);

	RtLbsType max_rFactor = 0.0;											/** @brief	���в�����	*/
	for (auto& curCluster : gsPairClusters) {								//���в����ӣ��������Ĳв�����
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


inline GSPairCluster CalMaxTDOASolutionGSPairCluster_MPSTSD(GeneralSource* refSource, const std::vector<GeneralSource*>& sources, const Scene* scene, RtLbsType threshold, RtLbsType freq, const std::vector<Complex>& tranFunction) {
	std::vector<GeneralSource*> newSources;
	for (auto& source : sources) {
		if (source != refSource) {
			newSources.push_back(source);
		}
	}


	//��������Դ��
	int newSourceNum = static_cast<int>(sources.size());
	int gsPairNum = static_cast<int>(newSourceNum * (newSourceNum - 1) / 2);
	std::vector<GSPair*> pairs(gsPairNum);
	int pairId = 0;
	for (int i = 0; i < newSourceNum; ++i) {
		for (int j = i + 1; j < newSourceNum; ++j) {
			if (newSources[i]->m_sensorData.m_id == newSources[j]->m_sensorData.m_id) {				//��ͬ���ݵĹ���Դ������
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


	//�Խ���г������࣬�õ������
	int max_cluster_num = 1;
	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(pairs, scene, threshold, max_cluster_num);

	//�ų��󲿷����ݺ���ͨ������׷�����㵽����������֮��Ĺ���
	return gsPairClusters.front();
}
#endif
