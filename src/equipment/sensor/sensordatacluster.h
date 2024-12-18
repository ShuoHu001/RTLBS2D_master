#ifndef RTLBS_SENSORDATACLUSTER
#define RTLBS_SENSORDATACLUSTER

#include "rtlbs.h"
#include "utility/define.h"
#include "sensordata.h"

class SensorDataCluster {
public:
	std::vector<SensorData> m_datas;							/** @brief	类中的传感器数据集合	*/
	SensorData m_mergedData;									/** @brief	合并的传感器数据信息	*/
public:
	SensorDataCluster();
	SensorDataCluster(const SensorDataCluster& cluster);
	~SensorDataCluster();
	SensorDataCluster& operator = (const SensorDataCluster& cluster);
	bool CanAddToClusterByAOA2D(const SensorData& data, RtLbsType threshold) const;					//按照二维角度是否可纳入类
	bool CanAddToClusterByTime(const SensorData& data, RtLbsType threshold) const;
	bool CanAddToClusterByTimeDiff(const SensorData& data, RtLbsType threshold) const;
	void CalMergedDataByAOA();																		//计算合成信息，按照角度
	void CalMergedDataByTime();																	//计算合成信息，按照时延
	void CalMergedDataByTimeDiff();																//计算合成信息，按照时延差
};

//按照功率大小进行排序，从大到小
inline bool ComparedByPower_SensorDataCluster(const SensorDataCluster& cluster1, const SensorDataCluster& cluster2) {
	return cluster1.m_mergedData.m_power > cluster2.m_mergedData.m_power;
}

//按照时延大小进行排序，从小到大
inline bool ComparedByDelay_SensorDataCluster(const SensorDataCluster& cluster1, const SensorDataCluster& cluster2) {
	return cluster1.m_mergedData.m_time < cluster2.m_mergedData.m_time;
}

//按照AOA二维角度进行聚类
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

inline std::vector<SensorDataCluster> ClusterSensorDataByTime(std::vector<SensorData>& datas, RtLbsType threshold) {
	std::vector<SensorDataCluster> clusters;
	for (auto& curData : datas) {
		bool addFlag = false;
		for (auto& curCluster : clusters) {
			if (curCluster.CanAddToClusterByTime(curData, threshold)) {
				curCluster.m_datas.push_back(curData);
				curCluster.CalMergedDataByTime();
				addFlag = true;
				break;
			}
		}
		if (!addFlag) {
			SensorDataCluster newCluster;
			newCluster.m_datas.push_back(curData);
			newCluster.CalMergedDataByTime();
			clusters.push_back(newCluster);
		}
	}
	return clusters;
}

inline std::vector<SensorDataCluster> ClusterSensorDataByTimeDiff(std::vector<SensorData>& datas, RtLbsType threshold) {
	std::vector<SensorDataCluster> clusters;
	for (auto& curData : datas) {
		bool addFlag = false;
		for (auto& curCluster : clusters) {
			if (curCluster.CanAddToClusterByTimeDiff(curData, threshold)) {
				curCluster.m_datas.push_back(curData);
				curCluster.CalMergedDataByTimeDiff();
				addFlag = true;
				break;
			}
		}
		if (!addFlag) {
			SensorDataCluster newCluster;
			newCluster.m_datas.push_back(curData);
			newCluster.CalMergedDataByTimeDiff();
			clusters.push_back(newCluster);
		}
	}
	return clusters;
}

#endif
