#include "tdoa_locator.h"

Point2D LBS_TDOA_LOCATOR_MPSTSD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, const LocalizeConfig& lbsConfig, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData)
{
	//LOSSFUNCTIONTYPE lossType = lbsConfig.m_solvingConfig.m_lossType;
	//RtLbsType gsPairClusterThreshold = lbsConfig.m_gsPairClusterThreshold;
	//bool extendAroundPointState = lbsConfig.m_extendAroundPointState;
	//WeightFactor weightFactor = lbsConfig.m_weightFactor;
	//HARDWAREMODE hardwareMode = lbsConfig.m_hardWareMode;
	//uint16_t threadNum = lbsConfig.m_threadNum;
	//weightFactor.InitTOAWeight();

	//std::vector<LBSInfo*>& lbsInfos = lbsInfoCluster.m_infos;

	////0-计算基本信息-计算广义源的位置
	//for (auto& curInfo : lbsInfos) {
	//	curInfo->CalculateBaseInfo(LBS_METHOD_RT_TDOA);
	//}

	////组合数据，遍历每个传感器所产生的广义源，设定参考广义源，组合形成非线性方程
	//int sensorNum = static_cast<int>(lbsInfos.size());
	////给每个广义源赋值对应的传感器数据
	//std::vector<SensorData> sensorDatas;
	//scene->m_sensorDataLibrary.GetAllSensorData(sensorDatas);
	//for (int i = 0; i < sensorNum; ++i) {
	//	lbsInfos[i]->SetSensorData(sensorDatas[i]);
	//}
	//std::vector<GeneralSource*> refSources = lbsInfos[0]->m_sources;												/** @brief	参考广义源, 默认第0个传感器数据为参考数据传感器	*/
	//std::vector<GeneralSource*> dataSources;																			/** @brief	数据广义源	*/
	//int dataSourceSize = 0;																								/** @brief	数据广义源的数量	*/
	//for (int i = 1; i < sensorNum; ++i) {
	//	size_t oldSize = dataSources.size();
	//	dataSourceSize = static_cast<int>(oldSize + lbsInfos[i]->m_sources.size());
	//	dataSources.resize(dataSourceSize);
	//	std::copy(lbsInfos[i]->m_sources.begin(), lbsInfos[i]->m_sources.end(), dataSources.begin() + oldSize);
	//}

	//std::vector<GSPairCluster> gsPairClusters;

	//for (auto& refSource : refSources) {
	//	GSPairCluster newCluster = CalMaxTDOASolutionGSPairCluster_MPSTSD(refSource, dataSources, scene, gsPairClusterThreshold, extendAroundPointState, lbsShiftErrorMatrix, freqConfig.m_centerFrequency);
	//	if (newCluster.m_pairs.size() != 0) {
	//		gsPairClusters.push_back(newCluster);
	//	}
	//}


	////采用射线追踪算法计算目标解到各个传感器之间的残差
	//int clusterNum = static_cast<int>(gsPairClusters.size());
	//std::vector<std::vector<RaytracingResult>> tempRTResult;									//第一维度为初步解的数量，第二维度为传感器的数量，调用射线追踪，判定解对传感器的有效性
	//std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	//const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	//tempRTResult.resize(clusterNum);																/** @brief	传感器数量	*/
	//std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	实时计算目标对传感器的数据	*/
	//std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	原始的传感器数据	*/
	//scene->m_sensorDataLibrary.GetAllSensorDataCollection(originalSensorDataCollection);										//获取原始传感器数据
	//std::sort(originalSensorDataCollection.begin(), originalSensorDataCollection.end(), ComparedByPower_SensorDataCollection);	//按照功率对传感器数据进行排序

	//if (hardwareMode == CPU_SINGLETHREAD) {
	//	for (auto& curCluster : gsPairClusters) {
	//		DirectlySetResultPath_CPUSingleThread(vroots, scene, splitRadius, &curCluster);
	//	}
	//}
	//else if (hardwareMode == CPU_MULTITHREAD) {
	//	DirectlySetResultPath_CPUMultiThread(vroots, scene, splitRadius, threadNum, gsPairClusters);
	//}
	//else if (hardwareMode == GPU_MULTITHREAD) {
	//	DirectlySetResultPath_GPUMultiThread(vroots, scene, splitRadius, gsPairClusters);
	//}

	//RtLbsType max_r_timeDiff = 0.0;
	//RtLbsType max_r_powerDiff = 0.0;
	//RtLbsType mean_r_timeDiff = 0.0;																							/** @brief	时间差平均残差	*/
	//RtLbsType mean_r_powerDiff = 0.0;																							/** @brief	功率平均残差	*/

	//for (auto& curCluster : gsPairClusters) {
	//	RtLbsType curCluster_min_r_timeDiff = FLT_MAX;																			/** @brief	最大时延差残差	*/
	//	RtLbsType curCluster_min_r_powerDiff = FLT_MAX;																			/** @brief	最大功率差残差	*/
	//	int curCluster_min_nullDataNum = INT_MAX;																				/** @brief	最大空数据数量	*/

	//	for (auto& curResult : curCluster.m_rtResult) {
	//		targetSensorDataCollection.clear();
	//		targetSensorDataCollection.resize(sensorNum);
	//		for (int i = 0; i < sensorNum; ++i) {
	//			const Sensor* curSensor = scene->m_sensors[i];
	//			curResult[i].CalculateBaseInfo(curSensor, freqs, tranFunctionData, antLibrary);
	//			curResult[i].GetMinDelaySensorData_Delay(targetSensorDataCollection[i], curSensor->m_timeErrorSTD);
	//		}
	//		RtLbsType cur_r_timeDiff = 0.0;
	//		RtLbsType cur_r_powerDiff = 0.0;
	//		int cur_nullDataNum = 0;
	//		CalculateSensorCollectionResidual_TDOA_SingleData(originalSensorDataCollection, targetSensorDataCollection, cur_r_timeDiff, cur_r_powerDiff, cur_nullDataNum);		//计算残差二范数和
	//		curCluster_min_r_timeDiff = std::min(curCluster_min_r_timeDiff, cur_r_timeDiff);
	//		curCluster_min_r_powerDiff = std::min(curCluster_min_r_powerDiff, cur_r_powerDiff);
	//		curCluster_min_nullDataNum = std::min(curCluster_min_nullDataNum, cur_nullDataNum);
	//	}

	//	curCluster.SetElementTDOAResidual(curCluster_min_r_timeDiff, curCluster_min_r_powerDiff, curCluster_min_nullDataNum);
	//	mean_r_timeDiff += curCluster_min_r_timeDiff;
	//	mean_r_powerDiff += curCluster_min_r_powerDiff;
	//}
	//mean_r_timeDiff /= gsPairClusters.size();
	//mean_r_powerDiff /= gsPairClusters.size();

	//for (auto& curCluster : gsPairClusters) {
	//	curCluster.m_timeDiffResidual += curCluster.m_nullDataNum * mean_r_timeDiff;
	//	curCluster.m_powerDiffResidual += curCluster.m_nullDataNum * mean_r_powerDiff;
	//	max_r_timeDiff = std::max(max_r_timeDiff, curCluster.m_timeDiffResidual);
	//	max_r_powerDiff = std::max(max_r_powerDiff, curCluster.m_powerDiffResidual);
	//}

	////计算cluster权重
	//RtLbsType maxWeight = 0.0;
	//for (auto& curCluster : gsPairClusters) {
	//	curCluster.CalNormalizedTDOAWeight(max_r_timeDiff, max_r_powerDiff, weightFactor);
	//	maxWeight = std::max(maxWeight, curCluster.m_weight);
	//}

	////计算归一化权重
	//for (auto& curCluster : gsPairClusters) {
	//	curCluster.m_weight /= maxWeight;
	//}

	//std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);



	//Point2D targetPoint = gsPairClusters.front().m_point;

	//std::cout << targetPoint.x << "," << targetPoint.y << std::endl;
	//return targetPoint;
return Point2D();
}

Point2D LBS_TDOA_LOCATOR_SPSTMD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, const LocalizeConfig& lbsConfig, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData)
{
	LOSSFUNCTIONTYPE lossType = lbsConfig.m_solvingConfig.m_lossType;
	RtLbsType gsPairClusterThreshold = lbsConfig.m_gsPairClusterThreshold;
	bool extendAroundPointState = lbsConfig.m_extendAroundPointState;
	WeightFactor weightFactor = lbsConfig.m_weightFactor;
	HARDWAREMODE hardwareMode = lbsConfig.m_hardWareMode;
	uint16_t threadNum = lbsConfig.m_threadNum;
	weightFactor.InitTOAWeight();

	std::vector<LBSInfo*>& lbsInfos = lbsInfoCluster.m_infos;
	std::vector<SensorData> sensorDatas;									/** @brief	获取到的所有传感器数据	*/
	scene->m_sensorDataLibrary.GetAllSensorData(sensorDatas);
	//0-计算广义源的位置
	for (auto& curInfo : lbsInfos) {
		curInfo->CalculateBaseInfo(sensorDatas, LBS_METHOD_RT_TDOA);
	}

	std::vector<GeneralSource*> mergedGSources;								/** @brief	所有广义源	*/

	//1-合并定位结果-将多个传感器的广义源进行合并
	size_t sourceSize = 0;
	for (auto& curInfo : lbsInfos) {
		size_t oldSize = mergedGSources.size();
		sourceSize += curInfo->m_sources.size();
		mergedGSources.resize(sourceSize);
		std::copy(curInfo->m_sources.begin(), curInfo->m_sources.end(), mergedGSources.begin() + oldSize);
	}

	std::vector<GeneralSource*> allGSCopy = mergedGSources;						/** @brief	所有广义源的复制	*/
	int sourceNum = static_cast<int>(allGSCopy.size());							/** @brief	所有广义源的数量	*/

	//2-求解权重矩阵

	std::vector<GSPair*> gsPairs;

	//每一个源都有可能成为参考源
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < sourceNum; ++i) {
		GeneralSource* refSource = allGSCopy[i];
		for (int j = i + 1; j < sourceNum; j++) {
			for (int k = j + 1; k < sourceNum; k++) {
				GeneralSource* data1 = allGSCopy[j];
				GeneralSource* data2 = allGSCopy[k];
				if (!PreCheckGSPairValidation_TDOA(refSource, data1, data2)) {			//若预检查不通过，则无效
					continue;
				}
				GSPair* newPair = new GSPair(refSource, data1, data2);
				if (!newPair->HasValidTDOASolution(scene)) {
					delete newPair;
					continue;
				}
#pragma omp atomic
				allGSCopy[i]->m_wCount += 1;
#pragma omp atomic
				allGSCopy[j]->m_wCount += 1;
#pragma omp atomic
				allGSCopy[k]->m_wCount += 1;
#pragma omp critical
				{
					gsPairs.push_back(newPair);
				}
			}
		}
	}

	//删除计数为0的广义源
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {		//移除无效的广义源
		return !source->IsValid();
		}), allGSCopy.end());

	//将所有广义源权重置0
	for (int i = 0; i < allGSCopy.size(); ++i) {
		allGSCopy[i]->m_wCount = 0;					//权重计数归零
		allGSCopy[i]->m_weight = 0;					//权重归零
	}

	//2-2 解集初步聚类
	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(gsPairs, scene, gsPairClusterThreshold, extendAroundPointState, lbsShiftErrorMatrix);
	int clusterNum = static_cast<int>(gsPairClusters.size());																	//簇数量

	int max_cluster_num = 0;
	for (auto& cluster : gsPairClusters) {
		max_cluster_num = std::max(max_cluster_num, static_cast<int>(cluster.m_pairs.size()));
	}


	//按照物理条件进行约束，求解权重矩阵
	std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	const MaterialLibrary* matLibrary = &scene->m_materialLibrary;
	const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	int sensorNum = static_cast<int>(scene->m_sensors.size());																	/** @brief	传感器数量	*/
	std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	实时计算目标对传感器的数据	*/
	std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	原始的传感器数据	*/
	scene->m_sensorDataLibrary.GetAllSensorDataCollectionWithTDOAError(originalSensorDataCollection);							//获取原始传感器数据
	std::sort(originalSensorDataCollection.begin(), originalSensorDataCollection.end(), ComparedByPower_SensorDataCollection);	//按照功率对传感器数据进行排序

	//为了同级比较残差，这里将原始数据按照功率的大小进行排序
	RtLbsType max_r_timeDiff = 0.0;																									//时间差最大残差
	RtLbsType max_r_powerDiff = 0.0;																								//功率差最大残差
	RtLbsType mean_r_timeDiff = 0.0;																								/** @brief	时间差平均残差	*/
	RtLbsType mean_r_powerDiff = 0.0;																								/** @brief	功率差平均残差	*/

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

	for (auto& curCluster : gsPairClusters) {																					//遍历所有簇计算广义残差
		RtLbsType curCluster_min_r_timeDiff = FLT_MAX;
		RtLbsType curCluster_min_r_powerDiff = FLT_MAX;
		int curCluster_min_nullDataNum = INT_MAX;
		RtLbsType curCluster_min_r_weight = FLT_MAX;

		std::vector<RtLbsType> curCluster_r_timeDiffs;																					/** @brief	当前的时间残差	*/
		std::vector<RtLbsType> curCluster_r_powerDiffs;																				/** @brief	当前的功率差残差	*/
		std::vector<int> curCluster_nullDataNums;																					/** @brief	不满足条件的匹配数	*/
		std::vector<RtLbsType> curCluster_r_weights;																				/** @brief	当前簇的残差权重	*/

		for (auto& curResult : curCluster.m_rtResult) {
			RtLbsType cur_r_timeDiff = 0.0;
			RtLbsType cur_r_powerDiff = 0.0;
			int cur_nullDataNum = 0;
			RtLbsType cur_r_weight = 0.0;

			targetSensorDataCollection.clear();
			targetSensorDataCollection.resize(sensorNum);

			for (int j = 0; j < sensorNum; ++j) {
				if (curResult[j].m_commonPaths.size() == 0) {		//若当前无多径，则跳过
					continue;
				}
				const Sensor* curSensor = scene->m_sensors[j];
				curResult[j].CalculateBaseInfo(curSensor, freqs, tranFunctionData, antLibrary);						//执行电磁计算,LBS电磁计算
				curResult[j].GetAllSensorData_Delay(targetSensorDataCollection[j], 0.0, 1.0);						//计算所有多径
				CalculateSensorCollectionResidual_TDOA_MultiData(originalSensorDataCollection, targetSensorDataCollection, weightFactor, cur_r_timeDiff, cur_r_powerDiff, cur_nullDataNum);

				cur_r_weight = weightFactor.m_timeWeight * cur_r_timeDiff + weightFactor.m_powerWeight * cur_r_powerDiff;

				curCluster_r_timeDiffs.push_back(cur_r_timeDiff);
				curCluster_r_powerDiffs.push_back(cur_r_powerDiff);
				curCluster_nullDataNums.push_back(cur_nullDataNum);
				curCluster_r_weights.push_back(cur_r_weight);

				//TDOA 算法需要增加计算cluster中的参考广义源
				if (!curCluster.m_isRefGSOccupied) {
					curCluster.m_TDOA_REFGS_Point = curResult[j].GetRefGeneralSource();
					curCluster.m_isRefGSOccupied = true;
				}

				//计算完成后需要删除当前cluster中的多径
				curResult[j].ReleaseAllRayPath();
			}
		}

		if (curCluster_r_weights.size() == 0) {
			curCluster.m_isValid = false;
			curCluster.m_weight = 0.0;
			curCluster.SetInValidState();
			continue;
		}

		int minDataId = 0;							/** @brief	扩展点中的最小的ID值	*/
		if (curCluster.m_aroundPoints.size() != 1) {									//当且仅当周围扩展点数量大于1时触发
			for (int i = 0; i < static_cast<int>(curCluster_r_weights.size()); ++i) {
				if ((curCluster_min_r_weight > curCluster_r_weights[i]) && curCluster_nullDataNums[i] <= curCluster_nullDataNums[0]) {		//在寻找最小值的过程中需要保证空数据数量小于第一值
					curCluster_min_r_weight = curCluster_r_weights[i];
					minDataId = i;
				}
			}

			RtLbsType curCluster_min_r_weight_rectify = FLT_MAX;
			if (minDataId < (curCluster.m_nearExtendNum + 1) && curCluster_nullDataNums[0] != 0) {												//修正存在差额数据导致的“最优权重”过低
				for (int i = 0; i < static_cast<int>(curCluster_r_weights.size()); ++i) {
					if (curCluster_min_r_weight_rectify > curCluster_r_weights[i] && curCluster_nullDataNums[i] == 0) {
						curCluster_min_r_weight_rectify = curCluster_r_weights[i];
						curCluster_min_r_weight = curCluster_r_weights[i];
						minDataId = i;
					}
				}
			}

			if (minDataId != 0 && minDataId > (curCluster.m_nearExtendNum + 1) && curCluster.m_deviateDistance > 5.0 && curCluster_min_r_weight / curCluster_r_weights[0] < 0.5) {					//额外追加大于5m条件,由于远距离扩展导致的误差低事件
				curCluster.m_isDeviateSolution = true;
			}
		}


		curCluster_min_r_timeDiff = curCluster_r_timeDiffs[minDataId];
		curCluster_min_r_powerDiff = curCluster_r_powerDiffs[minDataId];
		curCluster_min_nullDataNum = curCluster_nullDataNums[minDataId];
		curCluster.m_point = curCluster.m_aroundPoints[minDataId];										//更新当前簇的中心点

		curCluster.SetElementTDOAResidual(curCluster_min_r_timeDiff, curCluster_min_r_powerDiff, curCluster_min_nullDataNum);

		mean_r_timeDiff += curCluster_min_r_timeDiff;
		mean_r_powerDiff += curCluster_min_r_powerDiff;
	}

	mean_r_timeDiff /= clusterNum;
	mean_r_powerDiff /= clusterNum;

	for (auto curPair : gsPairs) {														//更新pair内权重并寻找最大权重
		curPair->UpdateResidual_TDOA(mean_r_timeDiff, mean_r_powerDiff);
		max_r_timeDiff = std::max(max_r_timeDiff, curPair->m_timeDiffResidual);
		max_r_powerDiff = std::max(max_r_powerDiff, curPair->m_powerDiffResidual);
	}

	//循环pair计算归一化残差系数,并将系数加入对应的广义源权重矩阵中
	for (auto it = gsPairs.begin(); it != gsPairs.end(); ++it) {
		GSPair* curPair = *it;
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_TDOA(max_r_timeDiff, max_r_powerDiff, weightFactor, max_cluster_num);
		}
	}



	//删除重复的广义源
	EraseRepeatGeneralSources(allGSCopy);			//删除重复的广义源


	//将cluster按照权重进行排序,用于寻找最大权重
	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);

	//计算所有广义源中权重最大的数值，进行权重归一化
	RtLbsType max_weight = gsPairClusters.front().m_weight;
	for (auto it = allGSCopy.begin(); it != allGSCopy.end(); ++it) {
		GeneralSource*& curSource = *it;
		curSource->NormalizedWeight(max_weight);
	}


	//更新完cluster的权重后，需要按照cluster中的倍率进行权重扩大，不影响广义源的权重
	for (auto& curCluster : gsPairClusters) {
		curCluster.m_weight *= curCluster.m_pairs.size();
	}

	//将cluster按照权重进行排序，更新簇权重
	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);

	//选取权重最大的cluster中的参考源作为参考广义源，原因，最小残差下的cluster中的参考源才是最准确的源。
	Point2D refClusterGSPoint = gsPairClusters.front().m_TDOA_REFGS_Point;
	GeneralSource* bestRefSource = nullptr;
	for (auto& curGS : allGSCopy) {
		if ((curGS->m_position - refClusterGSPoint).Length() < 1e-2) {
			bestRefSource = curGS;
			break;
		}
	}
	if (bestRefSource == nullptr) {//若cluster对应的参考源找不到广义源，则需要更新计数权重，得到计数权重最大的参考广义源
		for (auto& curCluster : gsPairClusters) {
			if (curCluster.m_weight > 0.9) {
				curCluster.CalculateRefSourceCount();
			}
		}
		std::sort(allGSCopy.begin(), allGSCopy.end(), ComparedByWCount_GeneralSource);
		bestRefSource = allGSCopy.front();
	}


	//搜索参考源为最佳参考源的所有广义源
	std::vector<GeneralSource*> validGSs;
	for (auto& curCluster : gsPairClusters) {
		curCluster.GetNonRepeatGeneralSource(bestRefSource, validGSs);
	}

	Point2D initPoint = gsPairClusters.front().m_point;				/** @brief	初始解	*/

	//配置求解器
	TDOASolver solver;
	solver.SetGeneralSource(bestRefSource, validGSs);

	Point2D targetPoint = initPoint;
	targetPoint = solver.Solving(lbsConfig.m_solvingConfig, scene->m_bbox, weightFactor, initPoint);

	if ((initPoint - targetPoint).Length() > 20) {							//若偏移程度过大，则恢复原始解（算法解无效）
		targetPoint = initPoint;
	}
	gsPairClusters.clear();
	std::vector<GSPairCluster>().swap(gsPairClusters);

	for (auto& pair : gsPairs) {
		delete pair;
		pair = nullptr;
	}
	gsPairs.clear();
	std::vector<GSPair*>().swap(gsPairs);

	//计算完成后删除所有广义源
	for (auto& source : mergedGSources) {
		delete source;
		source = nullptr;
	}
	mergedGSources.clear();
	std::vector<GeneralSource*>().swap(mergedGSources);

	return targetPoint;
}
