#ifndef RTLBS_SENSORDATACLUSTER
#define RTLBS_SENSORDATACLUSTER

#include "rtlbs.h"
#include "utility/define.h"
#include "sensordata.h"

class SensorDataCluster {
public:
	std::vector<SensorData> m_datas;							/** @brief	���еĴ��������ݼ���	*/
	SensorData m_mergedData;									/** @brief	�ϲ��Ĵ�����������Ϣ	*/
public:
	SensorDataCluster();
	SensorDataCluster(const SensorDataCluster& cluster);
	~SensorDataCluster();
	SensorDataCluster& operator = (const SensorDataCluster& cluster);
	bool CanAddToClusterByAOA2D(const SensorData& data, RtLbsType threshold) const;					//���ն�ά�Ƕ��Ƿ��������
	void CalMergedDataByAOA();																		//����ϳ���Ϣ�����սǶ�
	void CalMergedDataByDelay();																	//����ϳ���Ϣ������ʱ��
};

//���չ��ʴ�С�������򣬴Ӵ�С
inline bool ComparedByPower_SensorDataCluster(const SensorDataCluster& cluster1, const SensorDataCluster& cluster2) {
	return cluster1.m_mergedData.m_power > cluster2.m_mergedData.m_power;
}

//����ʱ�Ӵ�С�������򣬴�С����
inline bool ComparedByDelay_SensorDataCluster(const SensorDataCluster& cluster1, const SensorDataCluster& cluster2) {
	return cluster1.m_mergedData.m_time < cluster2.m_mergedData.m_time;
}

//����AOA��ά�ǶȽ��о���
inline std::vector<SensorDataCluster> ClusterSensorDataByAOA2D(std::vector<SensorData>& datas, RtLbsType threshold) {
	std::vector<SensorDataCluster> clusters;
	for (auto& curData : datas) {
		bool addFlag = false;
		for (auto& curCluster : clusters) {
			if (curCluster.CanAddToClusterByAOA2D(curData, threshold)) {
				curCluster.m_datas.push_back(curData);
				curCluster.CalMergedDataByAOA();
				addFlag = true;
				break;
			}
		}
		if (!addFlag) {
			SensorDataCluster newCluster;
			newCluster.m_datas.push_back(curData);
			newCluster.CalMergedDataByAOA();
			clusters.push_back(newCluster);
		}
	}
	return clusters;
}

#endif
