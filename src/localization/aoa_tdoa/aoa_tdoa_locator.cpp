#include "aoa_tdoa_locator.h"

Point2D LBS_AOA_TDOA_Locator_MPSTSD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, const LocalizeConfig& lbsConfig, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData)
{
	LOSSFUNCTIONTYPE lossType = lbsConfig.m_solvingConfig.m_lossType;
	RtLbsType gsPairClusterThreshold = lbsConfig.m_gsPairClusterThreshold;
	bool extendAroundPointState = lbsConfig.m_extendAroundPointState;
	WeightFactor weightFactor = lbsConfig.m_weightFactor;
	HARDWAREMODE hardwareMode = lbsConfig.m_hardWareMode;
	uint16_t threadNum = lbsConfig.m_threadNum;
	weightFactor.InitAOATOAweight();

	std::vector<LBSInfo*>& lbsInfos = lbsInfoCluster.m_infos;
	//0-计算广义源的位置
	for (auto& curInfo : lbsInfos) {
		curInfo->CalculateBaseInfo(LBS_METHOD_RT_AOA_TDOA);
	}

	std::vector<GeneralSource*> mergedGSources;								/** @brief	合并的广义源	*/

	//1-合并定位结果-将多个传感器的广义源进行合并
	size_t sourceSize = 0;
	for (auto& curInfo : lbsInfos) {
		size_t oldSize = mergedGSources.size();
		sourceSize += curInfo->m_sources.size();
		mergedGSources.resize(sourceSize);
		std::copy(curInfo->m_sources.begin(), curInfo->m_sources.end(), mergedGSources.begin() + oldSize);
	}

	std::vector<GeneralSource*> allGSCopy = mergedGSources;						/** @brief	所有广义源的复制	*/

	//2-按照几何约束条件删除无效广义源
	//创建广义源对,数量为 n*(n-1)/2

	size_t pairNum = static_cast<int>(sourceSize * (sourceSize - 1) / 2);
	std::vector<GSPair*> gsPairs;													/** @brief	广义源对	*/
	gsPairs.reserve(pairNum);

	size_t pairId = 0;																/** @brief	广义源对ID	*/
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < sourceSize; ++i) {
		for (int j = i + 1; j < sourceSize; ++j) {
			GSPair* newPair = new GSPair(allGSCopy[i], allGSCopy[j]);
			if (!newPair->HasValidAOASolution(scene)) {					//若广义源对无效，则删除该广义源对，计数停止增加
				delete newPair;
				continue;
			}
#pragma omp atomic
			allGSCopy[i]->m_wCount += 1;
#pragma omp atomic
			allGSCopy[j]->m_wCount += 1;
#pragma omp critical
			{
				gsPairs.push_back(newPair);
			}
		}
	}

	//删除权重为0值的广义源-先释放内存，后从数组中删除
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {		//移除无效的广义源
		return !source->IsValid();
		}), allGSCopy.end());

	//删除pair
	for (auto& curPair : gsPairs) {
		delete curPair;
		curPair = nullptr;
	}
	gsPairs.clear();

	//AOA解筛除广义源,删除无效广义源
	EraseRepeatGeneralSources(allGSCopy);			//删除重复的广义源

	//筛选出参考广义源并且置零广义源计数
	std::vector<GeneralSource*> refSources;
	for (auto& curSource : allGSCopy) {
		curSource->m_wCount = 0;
		if (curSource->m_sensorData.m_timeDiff == 0) {
			refSources.push_back(curSource);
		}
	}

	//组建AOA-TDOA方程组
	sourceSize = allGSCopy.size();
#pragma omp parallel for schedule(dynamic)
	for (int refId = 0; refId < refSources.size(); ++refId) {
		auto& curRefSource = refSources[refId];
		for (int i = 0; i < sourceSize; ++i) {
			for (int j = i + 1; j < sourceSize; ++j) {
				GeneralSource* gs1 = allGSCopy[i];
				GeneralSource* gs2 = allGSCopy[j];
				if (gs1 == curRefSource || gs2 == curRefSource) {			//跳过相同参考广义源
					continue;
				}
				GSPair* newPair = new GSPair(curRefSource, gs1, gs2);
				if (!newPair->HasValidAOATDOASolution(scene)) {					//若广义源对无效，则删除该广义源对，计数停止增加
					delete newPair;
					continue;
				}
#pragma omp atomic
				allGSCopy[i]->m_wCount += 1;
#pragma omp atomic
				allGSCopy[j]->m_wCount += 1;
#pragma omp critical
				{
					gsPairs.push_back(newPair);
				}
			}
		}
	}

	//删除权重为0的广义源
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {		//移除无效的广义源
		return !source->IsValid();
		}), allGSCopy.end());

	if (allGSCopy.size() == 0) {
		return Point2D(0, 0);
	}

	//将所有广义源权重置0
	for (int i = 0; i < allGSCopy.size(); ++i) {
		allGSCopy[i]->m_wCount = 0;					//权重计数归零
		allGSCopy[i]->m_weight = 0;					//权重归零
	}

	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(gsPairs, scene, gsPairClusterThreshold, extendAroundPointState, lbsShiftErrorMatrix);

	int clusterNum = static_cast<int>(gsPairClusters.size());																	//簇数量

	int max_cluster_num = 0;
	//提取广义源非重复元素,用于角度平均
	allGSCopy.clear();
	for (auto& cluster : gsPairClusters) {
		cluster.GetNonRepeatGeneralSource(allGSCopy);
		max_cluster_num = std::max(max_cluster_num, static_cast<int>(cluster.m_pairs.size()));
	}
	sourceSize = allGSCopy.size();

	//删除无效的pair
	for (auto& curPair : gsPairs) {
		if (curPair->m_isValid == false) {
			delete curPair;
			curPair = nullptr;
		}
	}
	gsPairs.erase(std::remove_if(gsPairs.begin(), gsPairs.end(), [](const GSPair* pair) {			//清除pair中无效的数据
		return pair == nullptr;
		}), gsPairs.end());

	//2-2  按照物理约束条件计算权重并删除不满足权重阈值的广义源
	// 采用射线追踪算法计算重新计算广义源与广义源之间可能的结果值，若该值不满足相似度阈值，则舍弃

	std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	const MaterialLibrary* matLibrary = &scene->m_materialLibrary;
	const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	int sensorNum = static_cast<int>(scene->m_sensors.size());																	/** @brief	传感器数量	*/
	std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	实时计算目标对传感器的数据	*/
	std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	原始的传感器数据	*/
	scene->m_sensorDataLibrary.GetAllSensorDataCollection(originalSensorDataCollection);										//获取原始传感器数据

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

	//为了同级比较残差，这里将原始数据按照功率的大小进行排序
	RtLbsType max_r_phi = 0.0;																									/** @brief	角度最大残差	*/
	RtLbsType max_r_timeDiff = 0.0;																								/** @brief	时间差最大残差	*/
	RtLbsType max_r_powerDiff = 0.0;																							/** @brief	功率差最大残差	*/
	RtLbsType mean_r_phi = 0.0;																									/** @brief	角度平均残差	*/
	RtLbsType mean_r_timeDiff = 0.0;																							/** @brief	时间差平均残差	*/
	RtLbsType mean_r_powerDiff = 0.0;																							/** @brief	功率差平均残差	*/

	for (auto& curCluster : gsPairClusters) {																					//遍历所有簇计算广义残差
		RtLbsType curCluster_min_r_phi = FLT_MAX;
		RtLbsType curCluster_min_r_timeDiff = FLT_MAX;
		RtLbsType curCluster_min_r_powerDiff = FLT_MAX;
		int curCluster_min_nullDataNum = INT_MAX;
		RtLbsType curCluster_min_r_weight = FLT_MAX;

		std::vector<RtLbsType> curCluster_r_phis;																					/** @brief	当前的角度残差	*/
		std::vector<RtLbsType> curCluster_r_timeDiffs;																				/** @brief	当前的时间差残差	*/
		std::vector<RtLbsType> curCluster_r_powerDiffs;																				/** @brief	当前的功率差残差	*/
		std::vector<int> curCluster_nullDataNums;																					/** @brief	不满足条件的匹配数	*/
		std::vector<RtLbsType> curCluster_r_weights;																				/** @brief	当前簇的残差权重	*/


		for (auto& curResult : curCluster.m_rtResult) {
			RtLbsType cur_r_phi = 0.0;
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
				curResult[j].GetMinDelaySensorData_Delay(targetSensorDataCollection[j], 0.0);						//计算时延最短多径
				//计算cluster中的参考广义源
				if (!curCluster.m_isRefGSOccupied) {
					curCluster.m_TDOA_REFGS_Point = curResult[0].GetRefGeneralSource();
					curCluster.m_isRefGSOccupied = true;
				}
				//计算完成后需要删除当前cluster中的多径
				curResult[j].ReleaseAllRayPath();
			}

			//计算时间差
			if (targetSensorDataCollection[0].m_datas.size() != 0) {
				RtLbsType refTime = targetSensorDataCollection[0].m_datas[0].m_time;
				for (int j = 1; j < targetSensorDataCollection.size(); j++) {
					if (targetSensorDataCollection[j].m_datas.size() != 0) {
						targetSensorDataCollection[j].m_datas[0].m_timeDiff = targetSensorDataCollection[j].m_datas[0].m_time - refTime;
					}
				}
			}
			
			CalculateSensorCollectionResidual_AOATDOA_SingleData(originalSensorDataCollection, targetSensorDataCollection, cur_r_phi, cur_r_timeDiff, cur_r_powerDiff, cur_nullDataNum);

			cur_r_weight = weightFactor.m_phiWeight * cur_r_phi + weightFactor.m_timeWeight * cur_r_timeDiff + weightFactor.m_powerWeight * cur_r_powerDiff;

			curCluster_r_phis.push_back(cur_r_phi);
			curCluster_r_timeDiffs.push_back(cur_r_timeDiff);
			curCluster_r_powerDiffs.push_back(cur_r_powerDiff);
			curCluster_nullDataNums.push_back(cur_nullDataNum);
			curCluster_r_weights.push_back(cur_r_weight);
		}

		if (curCluster_r_weights.size() == 0) {
			curCluster.m_isValid = false;
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


		curCluster_min_r_phi = curCluster_r_phis[minDataId];
		curCluster_min_r_timeDiff = curCluster_r_timeDiffs[minDataId];
		curCluster_min_r_powerDiff = curCluster_r_powerDiffs[minDataId];
		curCluster_min_nullDataNum = curCluster_nullDataNums[minDataId];

		curCluster.SetElementAOATDOAResidual(curCluster_min_r_phi, curCluster_min_r_timeDiff, curCluster_min_r_powerDiff, curCluster_min_nullDataNum);

		mean_r_phi += curCluster_min_r_phi;
		mean_r_timeDiff += curCluster_min_r_timeDiff;
		mean_r_powerDiff += curCluster_min_r_powerDiff;
	}

	mean_r_phi /= clusterNum;
	mean_r_timeDiff /= clusterNum;
	mean_r_powerDiff /= clusterNum;

	for (auto curPair : gsPairs) {														//更新pair内权重并寻找最大权重
		curPair->UpdateResidual_AOATDOA(mean_r_phi, mean_r_timeDiff, mean_r_powerDiff);
		max_r_phi = std::max(max_r_phi, curPair->m_phiResidual);
		max_r_timeDiff = std::max(max_r_timeDiff, curPair->m_timeDiffResidual);
		max_r_powerDiff = std::max(max_r_powerDiff, curPair->m_powerDiffResidual);
	}

	//循环pair计算归一化残差系数,并将系数加入对应的广义源权重矩阵中
	RtLbsType maxPairWeight = 0.0;
	for (auto it = gsPairs.begin(); it != gsPairs.end(); ++it) {
		GSPair* curPair = *it;
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_AOATDOA(max_r_phi, max_r_timeDiff, max_r_powerDiff, weightFactor, max_cluster_num);
		}
		maxPairWeight = std::max(maxPairWeight, curPair->m_weight);
	}

	//归一化权重
	for (auto& curSource : allGSCopy) {
		curSource->m_weight /= maxPairWeight;
	}
	for (auto& curCluster : gsPairClusters) {
		curCluster.m_weight /= maxPairWeight;
	}

	//将cluster按照权重进行排序
	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);
	Point2D refClusterGSPoint = gsPairClusters.front().m_TDOA_REFGS_Point;
	//寻找最佳参考广义源,两种方案，第一种方案为cluster中寻找，第二种方案为计数权重寻找
	GeneralSource* bestRefSource = nullptr;
	for (auto& curSource : refSources) {
		if ((curSource->m_position - refClusterGSPoint).Length() < 1e-4) {
			bestRefSource = curSource;
		}
	}
	if (bestRefSource == nullptr) {
		for (auto& curCluster : gsPairClusters) {
			if (curCluster.m_weight > 0.9) {
				curCluster.CalculateRefSourceCount();
			}
		}
		std::sort(refSources.begin(), refSources.end(), ComparedByWCount_GeneralSource);
		bestRefSource = refSources.front();
	}


	//广义源增加计数，用于排除无效参考广义源
	for (auto& curPair : gsPairs) {
		if (curPair->m_gsRef == bestRefSource) {
			curPair->m_gs1->m_wCount += 1;
			curPair->m_gs2->m_wCount += 1;
		}
		else {
			curPair->m_gsRef->m_wCount = 0;			//将无效参考广义源的权重置零
		}
	}

	//搜索0.9权重以上的cluster中的广义源
	std::vector<GeneralSource*> validGSs;
	for (auto& curCluster : gsPairClusters) {
		curCluster.GetNonRepeatGeneralSource(bestRefSource, validGSs);
	}

	Point2D initPoint = gsPairClusters.front().m_point;				/** @brief	初始解	*/

	//配置求解器
	AOATDOASolver solver;
	solver.SetGeneralSource(bestRefSource, validGSs);

	Point2D targetPoint = initPoint;
	targetPoint = solver.Solving(lbsConfig.m_solvingConfig, scene->m_bbox, weightFactor, initPoint);

	if ((initPoint - targetPoint).Length() > 100) {							//若偏移程度过大，则恢复原始解（算法解无效）
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

	////计算完成后删除所有广义源
	//for (auto& source : mergedGSources) {
	//	delete source;
	//	source = nullptr;
	//}
	mergedGSources.clear();
	std::vector<GeneralSource*>().swap(mergedGSources);

	return targetPoint;
}

Point2D LBS_AOA_TDOA_Locator_SPSTMD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, const LocalizeConfig& lbsConfig, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData)
{
	LOSSFUNCTIONTYPE lossType = lbsConfig.m_solvingConfig.m_lossType;
	RtLbsType gsPairClusterThreshold = lbsConfig.m_gsPairClusterThreshold;
	bool extendAroundPointState = lbsConfig.m_extendAroundPointState;
	WeightFactor weightFactor = lbsConfig.m_weightFactor;
	HARDWAREMODE hardwareMode = lbsConfig.m_hardWareMode;
	uint16_t threadNum = lbsConfig.m_threadNum;
	weightFactor.InitAOATOAweight();

	std::vector<LBSInfo*>& lbsInfos = lbsInfoCluster.m_infos;
	//0-计算广义源的位置
	for (auto& curInfo : lbsInfos) {
		curInfo->CalculateBaseInfo(LBS_METHOD_RT_AOA_TDOA);
	}

	std::vector<GeneralSource*> mergedGSources;								/** @brief	合并的广义源	*/

	//1-合并定位结果-将多个传感器的广义源进行合并
	size_t sourceSize = 0;
	for (auto& curInfo : lbsInfos) {
		size_t oldSize = mergedGSources.size();
		sourceSize += curInfo->m_sources.size();
		mergedGSources.resize(sourceSize);
		std::copy(curInfo->m_sources.begin(), curInfo->m_sources.end(), mergedGSources.begin() + oldSize);
	}

	std::vector<GeneralSource*> allGSCopy = mergedGSources;						/** @brief	所有广义源的复制	*/


	//2-按照几何约束条件删除无效广义源
	//创建广义源对,数量为 n*(n-1)/2

	bool hasAccurateSolution = false;

	size_t pairNum = static_cast<int>(sourceSize * (sourceSize - 1) / 2);
	std::vector<GSPair*> gsPairs;													/** @brief	广义源对	*/
	gsPairs.reserve(pairNum);

	size_t pairId = 0;																/** @brief	广义源对ID	*/
#pragma omp parallel for schedule(static)
	for (int i = 0; i < sourceSize; ++i) {
		for (int j = i + 1; j < sourceSize; ++j) {
			GSPair* newPair = new GSPair(allGSCopy[i], allGSCopy[j]);
			if (!newPair->HasValidAOASolution(scene)) {					//若广义源对无效，则删除该广义源对，计数停止增加
				delete newPair;
				continue;
			}
			if ((newPair->m_targetSolution - Point2D(70, 90)).Length() < 5.0) {
				hasAccurateSolution = true;
			}
#pragma omp atomic
			allGSCopy[i]->m_wCount += 1;
#pragma omp atomic
			allGSCopy[j]->m_wCount += 1;
#pragma omp critical
			{
				gsPairs.push_back(newPair);
			}
		}
	}



	//删除权重为0值的广义源-先释放内存，后从数组中删除
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {		//移除无效的广义源
		return !source->IsValid();
		}), allGSCopy.end());

	//采用射线追踪回馈计算方法修正角度误差，


	//删除pair
	for (auto& curPair : gsPairs) {
		delete curPair;
		curPair = nullptr;
	}
	gsPairs.clear();



    //AOA解筛除广义源,删除无效广义源
	EraseRepeatGeneralSources(allGSCopy);			//删除重复的广义源


	//筛选出参考广义源并且置零广义源计数
	std::vector<GeneralSource*> refSources;
	for (auto& curSource : allGSCopy) {
		curSource->m_wCount = 0;
		if (curSource->m_sensorData.m_timeDiff == 0) {
			refSources.push_back(curSource);
		}
	}
	//删除所有广义源中的参考广义源
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {		//移除无效的广义源
		return source->m_sensorData.m_id == 0;
		}), allGSCopy.end());


    //组建AOA-TDOA方程组
	sourceSize = allGSCopy.size();
#pragma omp parallel for schedule(static)
	for (int refId = 0; refId < refSources.size(); ++refId) {
		auto& curRefSource = refSources[refId];
		for (int i = 0; i < sourceSize; ++i) {
			for (int j = i + 1; j < sourceSize; ++j) {
				GeneralSource* gs1 = allGSCopy[i];
				GeneralSource* gs2 = allGSCopy[j];
				if (gs1 == curRefSource || gs2 == curRefSource) {			//跳过相同参考广义源
					continue;
				}
				GSPair* newPair = new GSPair(curRefSource, gs1, gs2);
				if (!newPair->HasValidAOATDOASolution(scene)) {					//若广义源对无效，则删除该广义源对，计数停止增加
					delete newPair;
					continue;
				}
#pragma omp atomic
				allGSCopy[i]->m_wCount += 1;
#pragma omp atomic
				allGSCopy[j]->m_wCount += 1;
#pragma omp critical
				{
					gsPairs.push_back(newPair);
				}
			}
		}
	}



	//删除权重为0的广义源
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {		//移除无效的广义源
		return !source->IsValid();
		}), allGSCopy.end());

	//将所有广义源权重置0
	for (int i = 0; i < allGSCopy.size(); ++i) {
		allGSCopy[i]->m_wCount = 0;					//权重计数归零
		allGSCopy[i]->m_weight = 0;					//权重归零
	}

	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(gsPairs, scene, gsPairClusterThreshold, extendAroundPointState, lbsShiftErrorMatrix);

	int clusterNum = static_cast<int>(gsPairClusters.size());																	//簇数量

	int max_cluster_num = 0;
	//提取广义源非重复元素,用于角度平均
	allGSCopy.clear();
	for (auto& cluster : gsPairClusters) {
		cluster.GetNonRepeatGeneralSource(allGSCopy);
		max_cluster_num = std::max(max_cluster_num, static_cast<int>(cluster.m_pairs.size()));
	}
	sourceSize = allGSCopy.size();

	//删除无效的pair
	for (auto& curPair : gsPairs) {
		if (curPair->m_isValid == false) {
			delete curPair;
			curPair = nullptr;
		}
	}
	gsPairs.erase(std::remove_if(gsPairs.begin(), gsPairs.end(), [](const GSPair* pair) {			//清除pair中无效的数据
		return pair == nullptr;
		}), gsPairs.end());

	//2-2  按照物理约束条件计算权重并删除不满足权重阈值的广义源
	// 采用射线追踪算法计算重新计算广义源与广义源之间可能的结果值，若该值不满足相似度阈值，则舍弃

	std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	const MaterialLibrary* matLibrary = &scene->m_materialLibrary;
	const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	int sensorNum = static_cast<int>(scene->m_sensors.size());																	/** @brief	传感器数量	*/
	std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	实时计算目标对传感器的数据	*/
	std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	原始的传感器数据	*/
	scene->m_sensorDataLibrary.GetAllSensorDataCollectionWithError_AOATDOA(originalSensorDataCollection);										//获取原始传感器数据
	//为了同级比较残差，这里将原始数据按照功率的大小进行排序
	RtLbsType max_r_phi = 0.0;																									/** @brief	角度最大残差	*/
	RtLbsType max_r_timeDiff = 0.0;																								/** @brief	时间差最大残差	*/
	RtLbsType max_r_powerDiff = 0.0;																							/** @brief	功率差最大残差	*/
	RtLbsType mean_r_phi = 0.0;																									/** @brief	角度平均残差	*/
	RtLbsType mean_r_timeDiff = 0.0;																							/** @brief	时间差平均残差	*/
	RtLbsType mean_r_powerDiff = 0.0;																							/** @brief	功率差平均残差	*/

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
		RtLbsType curCluster_min_r_phi = FLT_MAX;
		RtLbsType curCluster_min_r_timeDiff = FLT_MAX;
		RtLbsType curCluster_min_r_powerDiff = FLT_MAX;
		int curCluster_min_nullDataNum = INT_MAX;
		RtLbsType curCluster_min_r_weight = FLT_MAX;

		std::vector<RtLbsType> curCluster_r_phis;																					/** @brief	当前的角度残差	*/
		std::vector<RtLbsType> curCluster_r_timeDiffs;																				/** @brief	当前的时间差残差	*/
		std::vector<RtLbsType> curCluster_r_powerDiffs;																				/** @brief	当前的功率差残差	*/
		std::vector<int> curCluster_nullDataNums;																					/** @brief	不满足条件的匹配数	*/
		std::vector<RtLbsType> curCluster_r_weights;																				/** @brief	当前簇的残差权重	*/


		for (auto& curResult : curCluster.m_rtResult) {
			RtLbsType cur_r_phi = 0.0;
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
				CalculateSensorCollectionResidual_AOATDOA_MultiData(originalSensorDataCollection, targetSensorDataCollection, weightFactor, cur_r_phi, cur_r_timeDiff, cur_r_powerDiff, cur_nullDataNum);

				cur_r_weight = weightFactor.m_phiWeight * cur_r_phi + weightFactor.m_timeWeight * cur_r_timeDiff + weightFactor.m_powerWeight * cur_r_powerDiff;

				curCluster_r_phis.push_back(cur_r_phi);
				curCluster_r_timeDiffs.push_back(cur_r_timeDiff);
				curCluster_r_powerDiffs.push_back(cur_r_powerDiff);
				curCluster_nullDataNums.push_back(cur_nullDataNum);
				curCluster_r_weights.push_back(cur_r_weight);

				//计算cluster中的参考广义源
				if (!curCluster.m_isRefGSOccupied) {
					curCluster.m_TDOA_REFGS_Point = curResult[j].GetRefGeneralSource();
					curCluster.m_isRefGSOccupied = true;
				}
				//计算完成后需要删除当前cluster中的多径
				curResult[j].ReleaseAllRayPath();
			}
		}

		if (curCluster_r_weights.size() == 0) {			//若无路径数据，则为无效簇
			curCluster.SetInValidState();
			continue;
		}

		int minDataId = 0;							/** @brief	扩展点中的最小的ID值	*/
		int datasize = static_cast<int>(curCluster_r_weights.size());					/** @brief	簇内所有有效的数据数量	*/
		if (datasize > 1) {									//当且仅当周围扩展点数量大于1时触发
			//重大版本更新，增加对远近扩展点的合理处理
			
			//计算簇内平均权重
			RtLbsType max_cluster_r_weight = vectoroperator::GetMax(curCluster_r_weights);
			for (int i = 0; i < datasize; ++i) {
				curCluster_r_weights[i] += max_cluster_r_weight * curCluster_nullDataNums[i];
			}

			//选取近端最小残差
			minDataId = vectoroperator::GetMinIndex(curCluster_r_weights);


			if (minDataId != 0 && minDataId > (curCluster.m_nearExtendNum + 1) && curCluster.m_deviateDistance > 5.0 && curCluster_min_r_weight / curCluster_r_weights[0] < 0.5) {					//额外追加大于5m条件,由于远距离扩展导致的误差低事件
				curCluster.m_isDeviateSolution = true;
			}
		}


		curCluster_min_r_phi = curCluster_r_phis[minDataId];
		curCluster_min_r_timeDiff = curCluster_r_timeDiffs[minDataId];
		curCluster_min_r_powerDiff = curCluster_r_powerDiffs[minDataId];
		curCluster_min_nullDataNum = curCluster_nullDataNums[minDataId];

		curCluster.SetElementAOATDOAResidual(curCluster_min_r_phi, curCluster_min_r_timeDiff, curCluster_min_r_powerDiff, curCluster_min_nullDataNum);

		mean_r_phi += curCluster_min_r_phi;
		mean_r_timeDiff += curCluster_min_r_timeDiff;
		mean_r_powerDiff += curCluster_min_r_powerDiff;
	}

	mean_r_phi /= clusterNum;
	mean_r_timeDiff /= clusterNum;
	mean_r_powerDiff /= clusterNum;

	for (auto curPair : gsPairs) {														//更新pair内权重并寻找最大权重
		curPair->UpdateResidual_AOATDOA(mean_r_phi, mean_r_timeDiff, mean_r_powerDiff);
		max_r_phi = std::max(max_r_phi, curPair->m_phiResidual);
		max_r_timeDiff = std::max(max_r_timeDiff, curPair->m_timeDiffResidual);
		max_r_powerDiff = std::max(max_r_powerDiff, curPair->m_powerDiffResidual);
	}

	//循环pair计算归一化残差系数,并将系数加入对应的广义源权重矩阵中
	RtLbsType maxPairWeight = 0.0;
	for (auto it = gsPairs.begin(); it != gsPairs.end(); ++it) {
		GSPair* curPair = *it;
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_AOATDOA(max_r_phi, max_r_timeDiff, max_r_powerDiff, weightFactor, max_cluster_num);
		}
		maxPairWeight = std::max(maxPairWeight, curPair->m_weight);
	}

	//归一化权重
	for (auto& curSource : allGSCopy) {
		curSource->m_weight /= maxPairWeight;
	}
	for (auto& curCluster : gsPairClusters) {
		curCluster.m_weight /= maxPairWeight;
	}

	if (gsPairClusters.size() == 0) {
		for (auto& source : mergedGSources) {
			delete source;
			source = nullptr;
		}
		return Point2D(FLT_MAX, FLT_MAX);
	}

	//将cluster按照权重进行排序
	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);
	Point2D refClusterGSPoint = gsPairClusters.front().m_TDOA_REFGS_Point;
	//寻找最佳参考广义源,两种方案，第一种方案为cluster中寻找，第二种方案为计数权重寻找
	GeneralSource* bestRefSource = nullptr;
	for (auto& curSource : refSources) {
		if ((curSource->m_position - refClusterGSPoint).Length() < 1e-4) {
			bestRefSource = curSource;
		}
	}
	if (bestRefSource == nullptr) {
		for (auto& curCluster : gsPairClusters) {
			if (curCluster.m_weight > 0.9) {
				curCluster.CalculateRefSourceCount();
			}
		}
		std::sort(refSources.begin(), refSources.end(), ComparedByWCount_GeneralSource);
		bestRefSource = refSources.front();
	}


	//广义源增加计数，用于排除无效参考广义源
	for (auto& curPair : gsPairs) {
		if (curPair->m_gsRef == bestRefSource) {
			curPair->m_gs1->m_wCount += 1;
			curPair->m_gs2->m_wCount += 1;
		}
		else {
			curPair->m_gsRef->m_wCount = 0;			//将无效参考广义源的权重置零
		}
	}
	
	//搜索0.9权重以上的cluster中的广义源
	std::vector<GeneralSource*> validGSs;
	for (auto& curCluster : gsPairClusters) {
		curCluster.GetNonRepeatGeneralSource(bestRefSource, validGSs);
	}

	Point2D initPoint = gsPairClusters.front().m_point;				/** @brief	初始解	*/

	//配置求解器
	AOATDOASolver solver;
	solver.SetGeneralSource(bestRefSource, allGSCopy);

	Point2D targetPoint = initPoint;
	targetPoint = solver.Solving(lbsConfig.m_solvingConfig, scene->m_bbox, weightFactor, initPoint);

	if ((initPoint - targetPoint).Length() > 200) {							//若偏移程度过大，则恢复原始解（算法解无效）
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

	////计算完成后删除所有广义源
	//for (auto& source : mergedGSources) {
	//	delete source;
	//	source = nullptr;
	//}
	mergedGSources.clear();
	std::vector<GeneralSource*>().swap(mergedGSources);

	return targetPoint;
}
