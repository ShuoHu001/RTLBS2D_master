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


//���վ�����о���
inline std::vector<GSPairCluster> ClusterGSPairByDistance(std::vector<GSPair*>& pairs, RtLbsType threshold) {
	std::vector<GSPairCluster> clusters;

	for (auto& curPair : pairs) {
		bool addFlag = false;
		for (auto& curCluster : clusters) {
			if (curCluster.CanAddToClusterByDistance(curPair, threshold)) {
				curCluster.m_pairs.push_back(curPair);
				curCluster.CalClusterPosition();					//��������Ԫ�غ�������������
				addFlag = true;
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
	}

	return clusters;
}

#endif
