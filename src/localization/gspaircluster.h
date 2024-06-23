#ifndef RTLBS_GSPAIRCLUSTER
#define RTLBS_GSPAIRCLUSTER

#include "rtlbs.h"
#include "utility/define.h"
#include "gspair.h"


class GSPairCluster {
public:
	bool m_isValid;							/** @brief	����Ԫ���Ƿ���Ч�����ڶ�λ��˵	*/
	std::vector<GSPair*> m_pairs;			/** @brief	���ڹ���Դ��	*/
	Point2D m_point;						/** @brief	ƽ������	*/

public:
	GSPairCluster();
	GSPairCluster(const GSPairCluster& cluster);
	~GSPairCluster();
	GSPairCluster operator = (const GSPairCluster& cluster);
	bool CanAddToClusterByDistance(GSPair* pair, RtLbsType threshold);
	void CalClusterPosition();																							//�������ƽ�������
	void SetElementClusterId(int Id);																					//���ô���Ԫ�ص�ID
	void SetElementAOAResidual(RtLbsType r_phi, RtLbsType r_powerDiff, int nullDataNum);								//����AOA�Ͳв�
	void SetElementTDOAResidual(RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum);							//����TDOA�Ͳв�
	void SetElementAOATDOAResidual(RtLbsType r_phi, RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum);		//����AOA/TDOA�Ͳв�
};

inline bool ComparedByClusterSize(const GSPairCluster& cluster1, const GSPairCluster& cluster2) {
	return cluster1.m_pairs.size() > cluster2.m_pairs.size();
}

//���վ�����о���
inline std::vector<GSPairCluster> ClusterGSPairByDistance(std::vector<GSPair*>& pairs, RtLbsType threshold, int& maxCluterNum) {
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
	return clusters;
}

//��������ο�����Դ�µľ���pairֵ���ŵĴ�
inline GSPairCluster CalMaxTDOASolutionGSPairCluster(GeneralSource* refSource, const std::vector<GeneralSource*>& sources, const std::vector<SensorData>& sensorDatas, const Scene* scene, RtLbsType threshold) {
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


	//��pair�еĽ�����ٸ��ʵ�׷����Դ���ҵ���Ӧ��·��������TDOA��ֵ��������ֵ��������cluster�вв���С��һ�ݣ���ΪĿǰ�����Ž�

	//�Խ���г������࣬�õ������
	int max_cluster_num = 1;
	std::vector<GSPairCluster> gsPairCluster = ClusterGSPairByDistance(pairs, threshold, max_cluster_num);
	return gsPairCluster.front();
}

#endif
