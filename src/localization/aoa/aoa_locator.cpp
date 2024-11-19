#include "aoa_locator.h"

Point2D LBS_AoA_Locator_MPSTSD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, HARDWAREMODE hardwareMode, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, bool extendAroundPointState, const WeightFactor& weightFactor)
{
	std::vector<LBSInfo*>& lbsInfos = lbsInfoCluster.m_infos;
	//0-计算基本信息-计算广义源的位置 
	for (auto& curInfo : lbsInfos) {
		curInfo->CalculateBaseInfo(method);
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
	for (size_t i = 0; i < sourceSize; ++i) {
		for (size_t j = i + 1; j < sourceSize; ++j) {
			GSPair* newPair = new GSPair(allGSCopy[i], allGSCopy[j]);
			if (!newPair->HasValidAOASolution(scene)) {					//若广义源对无效，则删除该广义源对，计数停止增加
				delete newPair;
				continue;
			}
			allGSCopy[i]->m_wCount += 1;
			allGSCopy[j]->m_wCount += 1;
			gsPairs.push_back(newPair);
		}
	}


	//删除权重为0值的广义源-先释放内存，后从数组中删除
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {		//移除无效的广义源
		return !source->IsValid();
		}), allGSCopy.end());


	//将所有广义源权重置0
	for (int i = 0; i < mergedGSources.size(); ++i) {
		mergedGSources[i]->m_wCount = 0;					//权重计数归零
		mergedGSources[i]->m_weight = 0;					//权重归零
	}

	//2-2 解集初步聚类
	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(gsPairs, scene, gsPairClusterThreshold, extendAroundPointState, lbsShiftErrorMatrix);

	int clusterNum = static_cast<int>(gsPairClusters.size());																	//簇数量

	int max_cluster_num = gsPairClusters.front().m_pairs.size();																//簇中最大元素数量
	//提取广义源非重复元素,用于角度平均
	allGSCopy.clear();
	for (auto& cluster : gsPairClusters) {
		cluster.GetNonRepeatGeneralSource(allGSCopy);
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
	//为了同级比较残差，这里将原始数据按照功率的大小进行排序


	if (hardwareMode == CPU_SINGLETHREAD) {
		for (auto& curClutser : gsPairClusters) {
			DirectlySetResultPath_CPUSingleThread(vroots, scene, splitRadius, &curClutser);
		}
	}
	else if (hardwareMode == CPU_MULTITHREAD) {
		DirectlySetResultPath_CPUMultiThread(vroots, scene, splitRadius, threadNum, gsPairClusters);
	}
	else if (hardwareMode == GPU_MULTITHREAD) {
		DirectlySetResultPath_GPUMultiThread(vroots, scene, splitRadius, gsPairClusters);
	}

	RtLbsType mean_r_phi = 0.0;																										/** @brief	角度平均残差	*/
	RtLbsType mean_r_powerDiff = 0.0;																								/** @brief	功率差平均残差	*/
	RtLbsType max_r_phi = 0.0;																										/** @brief	角度最大残差	*/
	RtLbsType max_r_powerDiff = 0.0;																								/** @brief	功率最大残差	*/

	for (auto& curCluster : gsPairClusters) {
		RtLbsType curCluster_min_r_phi = FLT_MAX;																					/** @brief	当前簇最大角度残差	*/
		RtLbsType curCluster_min_r_powerDiff = FLT_MAX;																				/** @brief	当前簇最大功率差残差	*/
		int curCluster_min_nullDataNum = INT_MAX;																					/** @brief	当前簇最大空数据数量	*/
		RtLbsType curCluster_min_r_weight = FLT_MAX;

		std::vector<RtLbsType> curCluster_r_phis;																					/** @brief	当前簇角度残差	*/
		std::vector<RtLbsType> curCluster_r_powerDiffs;																				/** @brief	当前簇功率差残差	*/
		std::vector<int> curCluster_nullDataNums;																					/** @brief	当前簇空数据数量	*/
		std::vector<RtLbsType> curCluster_r_weights;																				/** @brief	当前簇的残差权重	*/


		for (auto& curResult : curCluster.m_rtResult) {																				//寻找cluster中多个解result中的残差最小值
			RtLbsType cur_r_phi = 0.0;
			RtLbsType cur_r_powerDiff = 0.0;
			int cur_nullDataNum = 0;
			RtLbsType cur_r_weight = 0.0;


			targetSensorDataCollection.clear();
			targetSensorDataCollection.resize(sensorNum);

			for (int i = 0; i < sensorNum; ++i) {
				if (curResult[i].m_commonPaths.size() == 0) {		//若当前无多径，则跳过
					continue;
				}
				const Sensor* curSensor = scene->m_sensors[i];
				curResult[i].CalculateBaseInfo(curSensor, freqs, tranFunctionData, antLibrary);
				curResult[i].GetMaxPowerSensorData_AOA2D(targetSensorDataCollection[i], curSensor->m_phiErrorSTD);
				curResult[i].ReleaseAllRayPath();																		//计算完成后释放多径信息，用于后续排序
			}
			CalculateSensorCollectionResidual_AOA_SingleData(originalSensorDataCollection, targetSensorDataCollection, cur_r_phi, cur_r_powerDiff, cur_nullDataNum);		//计算残差二范数和

			cur_r_weight = weightFactor.m_phiWeight*cur_r_phi+weightFactor.m_powerWeight*cur_r_powerDiff;
			curCluster_r_phis.push_back(cur_r_phi);
			curCluster_r_powerDiffs.push_back(cur_r_powerDiff);
			curCluster_nullDataNums.push_back(cur_nullDataNum);
			curCluster_r_weights.push_back(cur_r_weight);
			//curCluster_min_r_phi = std::min(curCluster_min_r_phi, cur_r_phi);
			//curCluster_min_r_powerDiff = std::min(curCluster_min_r_powerDiff, cur_r_powerDiff);
			//curCluster_min_nullDataNum = std::min(curCluster_min_nullDataNum, cur_nullDataNum);
		}

		//寻找簇中残差最小值对应的解
		if (curCluster_r_weights.size() == 0) {			//若簇中无权重数据，则簇为无效簇
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
		curCluster_min_r_powerDiff = curCluster_r_powerDiffs[minDataId];
		curCluster_min_nullDataNum = curCluster_nullDataNums[minDataId];

		curCluster.SetElementAOAResidual(curCluster_min_r_phi, curCluster_min_r_powerDiff, 1.0, curCluster_min_nullDataNum);

		mean_r_phi += curCluster_min_r_phi;
		mean_r_powerDiff += curCluster_min_r_powerDiff;
	}
	mean_r_phi /= clusterNum;
	mean_r_powerDiff /= clusterNum;

	for (auto& curPair : gsPairs) {
		curPair->UpdateResidual_AOA(mean_r_phi, mean_r_powerDiff);											//更新pair内权重并寻找最大权重
		max_r_phi = std::max(max_r_phi, curPair->m_phiResidual);
		max_r_powerDiff = std::max(max_r_powerDiff, curPair->m_powerDiffResidual);
	}

	//循环pair计算归一化残差系数,并将系数加入对应的广义源权重矩阵中
	for (auto& curPair : gsPairs) {
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_AOA(max_r_phi, max_r_powerDiff, weightFactor, max_cluster_num);
		}
	}

	EraseRepeatGeneralSources(allGSCopy);			//删除重复的广义源

	//将cluster按照权重进行排序
	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);

	//计算所有广义源中权重最大的数值，进行权重归一化
	RtLbsType max_weight = gsPairClusters.front().m_weight;
	for (auto it = allGSCopy.begin(); it != allGSCopy.end(); ++it) {
		GeneralSource*& curSource = *it;
		curSource->NormalizedWeight(max_weight);
	}

	//删除低于权重阈值的广义源
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* s) {
		return s->m_weight < 0.7;
		}), allGSCopy.end());

	//删除无效的广义源
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {
		return  !source->m_isValid;
		}), allGSCopy.end());


	//获取最大权重的Cluster,并将解作为初始解
	RtLbsType maxClusterWeight = 0;
	Point2D initPoint;
	for (auto& curCluster : gsPairClusters) {
		if (maxClusterWeight < curCluster.m_pairs[0]->m_weight) {
			maxClusterWeight = curCluster.m_pairs[0]->m_weight;
			initPoint = curCluster.m_point;
		}
	}

	//配置AOA求解器
	AOASolver* aoaSolver = new AOASolver();
	aoaSolver->SetGeneralSource(mergedGSources);
	Point2D targetPoint = aoaSolver->Solving_WIRLS(20, 1e-6, initPoint);

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

	//计算完成后删除所有广义源
	for (auto& source : mergedGSources) {
		delete source;
		source = nullptr;
	}
	mergedGSources.clear();
	std::vector<GeneralSource*>().swap(mergedGSources);

	return targetPoint;
}

Point2D LBS_AoA_Locator_SPSTMD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, HARDWAREMODE hardwareMode, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, bool extendAroundPointState, const WeightFactor& weightFactor)
{
	std::vector<LBSInfo*>& lbsInfos = lbsInfoCluster.m_infos;
	//0-计算基本信息-计算广义源的位置 
	for (auto& curInfo: lbsInfos) {
		curInfo->CalculateBaseInfo(method);
	}

	std::vector<GeneralSource*> mergedGSources;

	//1-合并定位结果-将多个传感器的广义源进行合并
	size_t sourceSize = 0;
	for (auto& curInfo : lbsInfos) {
		size_t oldSize = mergedGSources.size();
		sourceSize += curInfo->m_sources.size();
		mergedGSources.resize(sourceSize);
		std::copy(curInfo->m_sources.begin(), curInfo->m_sources.end(), mergedGSources.begin() + oldSize);
	}

	std::vector<GeneralSource*> allGSCopy = mergedGSources;						/** @brief	所有广义源的复制	*/

	//2-求解权重矩阵

	//2-按照几何约束条件删除无效广义源
	//创建广义源对,数量为 n*(n-1)/2
	size_t pairNum = static_cast<int>(sourceSize * (sourceSize - 1) / 2);
	std::vector<GSPair*> gsPairs;													/** @brief	广义源对	*/
	gsPairs.reserve(pairNum);


	size_t pairId = 0;																/** @brief	广义源对ID	*/
	for (int i = 0; i < sourceSize; ++i) {
		for (int j = i + 1; j < sourceSize; ++j) {
			GSPair* newPair = new GSPair(allGSCopy[i], allGSCopy[j]);
			if (!newPair->HasValidAOASolution(scene)) {					//若广义源对无效，则删除该广义源对，计数停止增加
				delete newPair;
				continue;
			}
			gsPairs.push_back(newPair);
		}
	}


	//删除权重为0值的广义源-先释放内存，后从数组中删除
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

	int max_cluster_num = gsPairClusters.front().m_pairs.size();																//最大簇中数据大小
	//提取广义源非重复元素,用于角度平均
	allGSCopy.clear();
	for (auto& cluster : gsPairClusters) {
		cluster.GetNonRepeatGeneralSource(allGSCopy);
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
	scene->m_sensorDataLibrary.GetAllSensorDataCollectionWithAOAError(originalSensorDataCollection);							//获取原始传感器数据
	std::sort(originalSensorDataCollection.begin(), originalSensorDataCollection.end(), ComparedByPower_SensorDataCollection);	//按照功率对传感器数据进行排序
	//为了同级比较残差，这里将原始数据按照功率的大小进行排序
	RtLbsType max_r_phi = 0.0;																									//角度最大残差
	RtLbsType max_r_powerDiff = 0.0;																							//功率最大残差
	RtLbsType mean_r_phi = 0.0;																									/** @brief	角度平均残差	*/
	RtLbsType mean_r_powerDiff = 0.0;																							/** @brief	功率平均残差	*/



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




	for (auto& curCluster : gsPairClusters) {																				//遍历所有簇
		RtLbsType curCluster_min_r_angularSpread = FLT_MAX;
		RtLbsType curCluster_min_r_phi = FLT_MAX;
		RtLbsType curCluster_min_r_powerDiff = FLT_MAX;
		int curCluster_min_nullDataNum = INT_MAX;
		RtLbsType curCluster_min_r_weight = FLT_MAX;

		std::vector<RtLbsType> curCluster_r_angularSpreads;																			/** @brief	当前的角度残差	*/
		std::vector<RtLbsType> curCluster_r_phis;																					/** @brief	当前的功率差残差	*/
		std::vector<RtLbsType> curCluster_r_powerDiffs;																				/** @brief	当前的角度扩展	*/
		std::vector<int> curCluster_nullDataNums;																					/** @brief	不满足条件的匹配数	*/
		std::vector<RtLbsType> curCluster_r_weights;																					/** @brief	当前簇的残差权重	*/

		for (auto& curResult : curCluster.m_rtResult) {																				//遍历簇中所有的解（位移导致的多解）
			RtLbsType cur_r_phi = 0.0;
			RtLbsType cur_r_powerDiff = 0.0;
			RtLbsType cur_r_angularSpread = 0.0;
			int cur_nullDataNum = 0;
			RtLbsType cur_r_weight = 0.0;

			targetSensorDataCollection.clear();
			targetSensorDataCollection.resize(sensorNum);
			for (int j = 0; j < sensorNum; ++j) {
				if (curResult[j].m_commonPaths.size() == 0) {
					continue;
				}
				const Sensor* curSensor = scene->m_sensors[j];
				curResult[j].CalculateBaseInfo(curSensor, freqs, tranFunctionData, antLibrary);						//执行电磁计算,LBS电磁计算
				curResult[j].GetAllSensorData_AOA2D(targetSensorDataCollection[j], 0.0, 1.0);			//收集实时计算的传感器结果,由于是仿真,因此不进行稀疏，2.0倍数是±
				CalculateSensorCollectionResidual_AOA_MultiData(originalSensorDataCollection, targetSensorDataCollection, weightFactor, cur_r_phi, cur_r_powerDiff, cur_r_angularSpread, cur_nullDataNum);		//计算残差二范数和

				cur_r_weight = (weightFactor.m_powerWeight * cur_r_phi + weightFactor.m_phiWeight * cur_r_powerDiff) / (weightFactor.m_powerWeight + weightFactor.m_phiWeight);

				curCluster_r_angularSpreads.push_back(cur_r_angularSpread);
				curCluster_r_phis.push_back(cur_r_phi);
				curCluster_r_powerDiffs.push_back(cur_r_powerDiff);
				curCluster_r_weights.push_back(cur_r_weight);
				curCluster_nullDataNums.push_back(cur_nullDataNum);

				//计算完成后需要删除当前cluster中的多径
				curResult[j].ReleaseAllRayPath();
			}
		}

		if (curCluster_r_weights.size() == 0) {
			curCluster.m_isValid = false;
			continue;
		}

		int minDataId = 0;
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



		curCluster_min_r_angularSpread = curCluster_r_angularSpreads[minDataId];
		curCluster_min_r_phi = curCluster_r_phis[minDataId];
		curCluster_min_r_powerDiff = curCluster_r_powerDiffs[minDataId];
		curCluster_min_nullDataNum = curCluster_nullDataNums[minDataId];

		//修正由于角度偏移引起过0和过2pi引起的误差
		for (int i = 1; i < curCluster.m_nearExtendNum; ++i) {
			if (curCluster_min_r_angularSpread > curCluster_r_angularSpreads[i]) {
				curCluster_min_r_angularSpread = curCluster_r_angularSpreads[i];
			}
		}

		curCluster.SetElementAOAResidual(curCluster_min_r_phi, curCluster_min_r_powerDiff, curCluster_min_r_angularSpread, curCluster_min_nullDataNum);
		curCluster.m_angularSpreadResidual = curCluster_min_r_angularSpread;
		mean_r_phi += curCluster_min_r_phi;
		mean_r_powerDiff += curCluster_min_r_powerDiff;
	}



	mean_r_phi /= clusterNum;
	mean_r_powerDiff /= clusterNum;



	for (auto curPair : gsPairs) {
		curPair->m_phiResidual += curPair->m_nullDataNum * mean_r_phi;
		curPair->m_powerDiffResidual += curPair->m_nullDataNum * mean_r_powerDiff;
		max_r_phi = std::max(max_r_phi, curPair->m_phiResidual);
		max_r_powerDiff = std::max(max_r_powerDiff, curPair->m_powerDiffResidual);
	}


	//循环pair计算归一化残差系数,并将系数加入对应的广义源权重矩阵中
	for (auto it = gsPairs.begin(); it != gsPairs.end(); ++it) {
		GSPair* curPair = *it;
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_AOA(max_r_phi, max_r_powerDiff, weightFactor, max_cluster_num);
		}
	}


	//删除重复的广义源
	EraseRepeatGeneralSources(allGSCopy);			//删除重复的广义源


	//将cluster按照权重进行排序
	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);


	//计算所有广义源中权重最大的数值，进行权重归一化
	RtLbsType max_weight = gsPairClusters.front().m_weight;
	for (auto it = allGSCopy.begin(); it != allGSCopy.end(); ++it) {
		GeneralSource*& curSource = *it;
		curSource->NormalizedWeight(max_weight);
	}

	//删除低于权重阈值的广义源
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* s) {
		return s->m_weight < 0.9;
		}), allGSCopy.end());

	//单源定位中，root源的数据最大只保留一个
	int hasRootNum = 0;
	for (auto& source : allGSCopy) {
		if (source->m_type == NODE_ROOT) {
			hasRootNum++;
			if (hasRootNum > 1) {
				source->m_isValid = false;
			}
		}
	}

	//删除无效的广义源
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {
		return  !source->m_isValid;
		}), allGSCopy.end());


	//获取最大权重的Cluster
	RtLbsType maxClusterWeight = 0;
	Point2D initPoint;
	for (auto& curCluster : gsPairClusters) {
		if (maxClusterWeight < curCluster.m_pairs[0]->m_weight) {
			maxClusterWeight = curCluster.m_pairs[0]->m_weight;
			initPoint = curCluster.m_point;
		}
	}


	//配置求解器
	AOASolver* aoaSolver = new AOASolver();
	aoaSolver->SetGeneralSource(allGSCopy);

	Point2D targetPoint = aoaSolver->Solving_WIRLS(20, 1e-6, initPoint);

	if ((initPoint - targetPoint).Length() > 100) {				//若偏移程度过大，则恢复原始点（解无效）
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
