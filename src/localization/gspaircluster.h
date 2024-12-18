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
	bool m_isValid;													/** @brief	����Ԫ���Ƿ���Ч�����ڶ�λ��˵	*/
	bool m_isDeviateSolution;										/** @brief	�Ƿ���ƫ��⣬�������Դ�ԵĹ����Բ�������m_aroundPoints���ṩ�Ľ�	*/	
	bool m_isRefGSOccupied;											/** @brief	�ο�����Դ�����Ƿ�ռ�ã���ռ�����ٽ��и�ֵ	*/
	int m_nearExtendNum;											/** @brief	��������չ������	*/
	int m_farExtendNum;												/** @brief	Զ������չ������	*/
	RtLbsType m_deviateDistance;									/** @brief	ƫ�����	*/
	std::vector<GSPair*> m_pairs;									/** @brief	���ڹ���Դ��	*/
	Point2D m_point;												/** @brief	ƽ������	*/
	Point2D m_TDOA_REFGS_Point;										/** @brief	��TDOA��λ�㷨����Ҫ���RTresult�������ο�����Դ����	*/
	std::vector<Point2D> m_aroundPoints;							/** @brief	��Χ������	*/
	RtLbsType m_residualFactor;										/** @brief	�в�����	*/
	RtLbsType m_residual;											/** @brief	�в�	*/
	std::vector<std::vector<RaytracingResult>> m_rtResult;			/** @brief	��Ҫ���������׷�ٽ������һά��Ϊ����������ڶ�ά��Ϊ������������	*/
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
	void UpdateGSPairBelongingCluster();																				//����gspair��cluster
	void ExtendAroundPoint(bool expandFlag, const ElevationMatrix& lbsShiftErrorMatrix, const Scene* scene);													//��չ���ڵ㣬true��ʾ������չ��false��ʾ��������չ��ֻ����һ��
	RtLbsType CalTDOAResidualFactor();																					//����в�����
	void CalTDOAResidual(RtLbsType maxResidualFactor);																	//����в�
	bool CanAddToClusterByDistance(GSPair* pair, RtLbsType threshold);
	void CalClusterPosition();																							//�������ƽ�������
	void SetElementClusterId(int Id);																					//���ô���Ԫ�ص�ID
	void SetElementAOAResidual(RtLbsType r_phi, RtLbsType r_powerDiff, RtLbsType r_angularSpreadResidual, int nullDataNum);								//����AOA�Ͳв�
	void SetElementTOAResidual(RtLbsType r_time, RtLbsType r_power, int nullDataNum);
	void SetElementTDOAResidual(RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum);							//����TDOA�Ͳв�
	void SetElementAOATDOAResidual(RtLbsType r_phi, RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum);		//����AOA/TDOA�Ͳв�
	void SetElementAOATOAResidual(RtLbsType r_phi, RtLbsType r_time, RtLbsType r_power, int nullDataNum);			//����AOA/TOA�Ͳв�
	void SetInValidState();																								//����Ϊ��Ч
	void CalNormalizedAOAWeight(RtLbsType max_r_phi, RtLbsType max_r_powerDiff, const WeightFactor& w);											//�����AOA��һȨ��
	void CalNormalizedTDOAWeight(RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w);									//�����TDOA��һȨ��
	void CalNormalizedAOATDOAWeight(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w);			//�����AOA-TDOA��һȨ��
	void GetNonRepeatGeneralSource(std::vector<GeneralSource*>& sources);																		//ɾ���ظ��Ĺ���Դ�Լ������Դ���еĹ���Դ����Ҫ���ڽǶ�ƽ��
	void GetNonRepeatGeneralSource(GeneralSource* refSource, std::vector<GeneralSource*>& sources);												//ɾ���ظ��Ĺ���Դ����Ҫ�������AOATDOA��λ�㷨�е���Ч�ο�����Դ���еĹ���Դ
	void CalculateRefSourceCount() const;																										//����ο�����Դ
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
inline std::vector<GSPairCluster> ClusterGSPairByDistance(std::vector<GSPair*>& pairs, const Scene* scene, RtLbsType threshold, bool extendAroundPointState, const ElevationMatrix& lbsShiftErrorMatrix) {
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
	int id = 0;
	for (auto& curCluster : clusters) {
		curCluster.SetElementClusterId(id++);
	}

	//���մ�������С��������
	std::sort(clusters.begin(), clusters.end(), ComparedByClusterSize);

	//��չ�ܱߵ�����&���¸�cluster
	for (auto& cluster : clusters) {
		cluster.ExtendAroundPoint(extendAroundPointState, lbsShiftErrorMatrix, scene);
		cluster.UpdateGSPairBelongingCluster();
	}

	return clusters;
}

#endif
