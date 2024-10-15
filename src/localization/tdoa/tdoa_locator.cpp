#include "tdoa_locator.h"

Point2D LBS_TDOA_LOCATOR_MPSTSD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, HARDWAREMODE hardwareMode, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData,  LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, bool extendAroundPointState, const WeightFactor& weightFactor)
{

	std::vector<LBSInfo*>& lbsInfos = lbsInfoCluster.m_infos;

	//0-计算基本信息-计算广义源的位置
	for (auto& curInfo : lbsInfos) {
		curInfo->CalculateBaseInfo(method);
	}

	//组合数据，遍历每个传感器所产生的广义源，设定参考广义源，组合形成非线性方程
	int sensorNum = static_cast<int>(lbsInfos.size());
	//给每个广义源赋值对应的传感器数据
	std::vector<SensorData> sensorDatas;
	scene->m_sensorDataLibrary.GetAllSensorData(sensorDatas);
	for (int i = 0; i < sensorNum; ++i) {
		lbsInfos[i]->SetSensorData(sensorDatas[i]);
	}
	std::vector<GeneralSource*> refSources = lbsInfos[0]->m_sources;												/** @brief	参考广义源, 默认第0个传感器数据为参考数据传感器	*/
	std::vector<GeneralSource*> dataSources;																			/** @brief	数据广义源	*/
	int dataSourceSize = 0;																								/** @brief	数据广义源的数量	*/
	for (int i = 1; i < sensorNum; ++i) {
		size_t oldSize = dataSources.size();
		dataSourceSize = static_cast<int>(oldSize + lbsInfos[i]->m_sources.size());
		dataSources.resize(dataSourceSize);
		std::copy(lbsInfos[i]->m_sources.begin(), lbsInfos[i]->m_sources.end(), dataSources.begin() + oldSize);
	}

	std::vector<GSPairCluster> gsPairClusters;

	for (auto& refSource : refSources) {
		GSPairCluster newCluster = CalMaxTDOASolutionGSPairCluster_MPSTSD(refSource, dataSources, scene, gsPairClusterThreshold, extendAroundPointState, lbsShiftErrorMatrix, freqConfig.m_centerFrequency);
		if (newCluster.m_pairs.size() != 0) {
			gsPairClusters.push_back(newCluster);
		}
	}


	//采用射线追踪算法计算目标解到各个传感器之间的残差
	int clusterNum = static_cast<int>(gsPairClusters.size());
	std::vector<std::vector<RaytracingResult>> tempRTResult;									//第一维度为初步解的数量，第二维度为传感器的数量，调用射线追踪，判定解对传感器的有效性
	std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	tempRTResult.resize(clusterNum);																/** @brief	传感器数量	*/
	std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	实时计算目标对传感器的数据	*/
	std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	原始的传感器数据	*/
	scene->m_sensorDataLibrary.GetAllSensorDataCollection(originalSensorDataCollection);										//获取原始传感器数据
	std::sort(originalSensorDataCollection.begin(), originalSensorDataCollection.end(), ComparedByPower_SensorDataCollection);	//按照功率对传感器数据进行排序

	if (hardwareMode == CPU_SINGLETHREAD) {
		for (auto& curCluster : gsPairClusters) {
			DirectlySetResultPath_CPUSingleThread(vroots, scene, splitRadius, &curCluster);
		}
	}
	else if (hardwareMode == CPU_MULTITHREAD) {
		DirectlySetResultPath_CPUMultiThread(vroots, scene, splitRadius, threadNum, gsPairClusters);
	}
	else if (hardwareMode == GPU_MULTITHREAD) {
		DirectlySetResultPath_GPUMultiThread(vroots, scene, splitRadius, gsPairClusters);
	}

	RtLbsType max_r_timeDiff = 0.0;
	RtLbsType max_r_powerDiff = 0.0;
	RtLbsType mean_r_timeDiff = 0.0;																							/** @brief	时间差平均残差	*/
	RtLbsType mean_r_powerDiff = 0.0;																							/** @brief	功率平均残差	*/

	for (auto& curCluster : gsPairClusters) {
		RtLbsType curCluster_min_r_timeDiff = FLT_MAX;																			/** @brief	最大时延差残差	*/
		RtLbsType curCluster_min_r_powerDiff = FLT_MAX;																			/** @brief	最大功率差残差	*/
		int curCluster_min_nullDataNum = INT_MAX;																				/** @brief	最大空数据数量	*/

		for (auto& curResult : curCluster.m_rtResult) {
			targetSensorDataCollection.clear();
			targetSensorDataCollection.resize(sensorNum);
			for (int i = 0; i < sensorNum; ++i) {
				const Sensor* curSensor = scene->m_sensors[i];
				curResult[i].CalculateBaseInfo(curSensor, freqs, tranFunctionData, antLibrary);
				curResult[i].GetMaxPowerSensorData_Delay(targetSensorDataCollection[i], curSensor->m_timeErrorSTD);
			}
			RtLbsType cur_r_timeDiff = 0.0;
			RtLbsType cur_r_powerDiff = 0.0;
			int cur_nullDataNum = 0;
			CalculateSensorCollectionResidual_TDOA_SingleData(originalSensorDataCollection, targetSensorDataCollection, cur_r_timeDiff, cur_r_powerDiff, cur_nullDataNum);		//计算残差二范数和
			curCluster_min_r_timeDiff = std::min(curCluster_min_r_timeDiff, cur_r_timeDiff);
			curCluster_min_r_powerDiff = std::min(curCluster_min_r_powerDiff, cur_r_powerDiff);
			curCluster_min_nullDataNum = std::min(curCluster_min_nullDataNum, cur_nullDataNum);
		}

		curCluster.SetElementTDOAResidual(curCluster_min_r_timeDiff, curCluster_min_r_powerDiff, curCluster_min_nullDataNum);
		mean_r_timeDiff += curCluster_min_r_timeDiff;
		mean_r_powerDiff += curCluster_min_r_powerDiff;
	}
	mean_r_timeDiff /= gsPairClusters.size();
	mean_r_powerDiff /= gsPairClusters.size();

	for (auto& curCluster : gsPairClusters) {
		curCluster.m_timeDiffResidual += curCluster.m_nullDataNum * mean_r_timeDiff;
		curCluster.m_powerDiffResidual += curCluster.m_nullDataNum * mean_r_powerDiff;
		max_r_timeDiff = std::max(max_r_timeDiff, curCluster.m_timeDiffResidual);
		max_r_powerDiff = std::max(max_r_powerDiff, curCluster.m_powerDiffResidual);
	}

	//计算cluster权重
	RtLbsType maxWeight = 0.0;
	for (auto& curCluster : gsPairClusters) {
		curCluster.CalNormalizedTDOAWeight(max_r_timeDiff, max_r_powerDiff, weightFactor);
		maxWeight = std::max(maxWeight, curCluster.m_weight);
	}

	//计算归一化权重
	for (auto& curCluster : gsPairClusters) {
		curCluster.m_weight /= maxWeight;
	}

	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);



	Point2D targetPoint = gsPairClusters.front().m_point;

	std::cout << targetPoint.x << "," << targetPoint.y << std::endl;
	return targetPoint;
}

Point2D LBS_TDOA_LOCATOR_SPSTMD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, HARDWAREMODE hardwareMode, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, bool extendAroundPointState, const WeightFactor& weightFactor)
{

	std::vector<LBSInfo*>& lbsInfos = lbsInfoCluster.m_infos;

	//0-计算基本信息-计算广义源的位置 
	for (auto& curInfo: lbsInfos) {
		curInfo->CalculateBaseInfo(method);
	}

	std::vector<GeneralSource*> mergedGSource;

	//1-合并定位结果-将多个传感器的广义源进行合并
	size_t sourceSize = 0;
	for (auto& curInfo: lbsInfos) {
		size_t oldSize = mergedGSource.size();
		sourceSize += curInfo->m_sources.size();
		mergedGSource.resize(sourceSize);
		std::copy(curInfo->m_sources.begin(), curInfo->m_sources.end(), mergedGSource.begin() + oldSize);
	}

	//1-求解最优的参考站，按照循环的模式求解
	std::vector<GeneralSource*> refSources;												/** @brief	参考广义源	*/
	for (auto curSource : mergedGSource) {
		refSources.push_back(curSource);
	}

	std::vector<SensorData> sensorDatas;
	scene->m_sensorDataLibrary.GetAllSensorData(sensorDatas);
	std::vector<GSPairCluster> clusters;
	for (auto& refSource : refSources) {
		refSource->m_sensorData = sensorDatas.front();						//参考源接收数据赋值
		GSPairCluster newCluster = CalMaxTDOASolutionGSPairCluster_SPSTMD(refSource, mergedGSource, sensorDatas, scene, gsPairClusterThreshold, extendAroundPointState, lbsShiftErrorMatrix, freqConfig.m_centerFrequency, tranFunctionData);
		if (newCluster.m_pairs.size() != 0) {
			clusters.push_back(newCluster);
		}
	}

	std::sort(clusters.begin(), clusters.end(), ComparedByClusterResidual);

	Point2D targetPoint = clusters.front().m_point;

	std::cout << targetPoint.x << "," << targetPoint.y << std::endl;
	return targetPoint;
}
