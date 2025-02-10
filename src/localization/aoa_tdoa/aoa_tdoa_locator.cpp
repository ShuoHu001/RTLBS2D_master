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
	//0-�������Դ��λ��
	for (auto& curInfo : lbsInfos) {
		curInfo->CalculateBaseInfo(LBS_METHOD_RT_AOA_TDOA);
	}

	std::vector<GeneralSource*> mergedGSources;								/** @brief	�ϲ��Ĺ���Դ	*/

	//1-�ϲ���λ���-������������Ĺ���Դ���кϲ�
	size_t sourceSize = 0;
	for (auto& curInfo : lbsInfos) {
		size_t oldSize = mergedGSources.size();
		sourceSize += curInfo->m_sources.size();
		mergedGSources.resize(sourceSize);
		std::copy(curInfo->m_sources.begin(), curInfo->m_sources.end(), mergedGSources.begin() + oldSize);
	}

	std::vector<GeneralSource*> allGSCopy = mergedGSources;						/** @brief	���й���Դ�ĸ���	*/

	//2-���ռ���Լ������ɾ����Ч����Դ
	//��������Դ��,����Ϊ n*(n-1)/2

	size_t pairNum = static_cast<int>(sourceSize * (sourceSize - 1) / 2);
	std::vector<GSPair*> gsPairs;													/** @brief	����Դ��	*/
	gsPairs.reserve(pairNum);

	size_t pairId = 0;																/** @brief	����Դ��ID	*/
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < sourceSize; ++i) {
		for (int j = i + 1; j < sourceSize; ++j) {
			GSPair* newPair = new GSPair(allGSCopy[i], allGSCopy[j]);
			if (!newPair->HasValidAOASolution(scene)) {					//������Դ����Ч����ɾ���ù���Դ�ԣ�����ֹͣ����
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

	//ɾ��Ȩ��Ϊ0ֵ�Ĺ���Դ-���ͷ��ڴ棬���������ɾ��
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {		//�Ƴ���Ч�Ĺ���Դ
		return !source->IsValid();
		}), allGSCopy.end());

	//ɾ��pair
	for (auto& curPair : gsPairs) {
		delete curPair;
		curPair = nullptr;
	}
	gsPairs.clear();

	//AOA��ɸ������Դ,ɾ����Ч����Դ
	EraseRepeatGeneralSources(allGSCopy);			//ɾ���ظ��Ĺ���Դ

	//ɸѡ���ο�����Դ�����������Դ����
	std::vector<GeneralSource*> refSources;
	for (auto& curSource : allGSCopy) {
		curSource->m_wCount = 0;
		if (curSource->m_sensorData.m_timeDiff == 0) {
			refSources.push_back(curSource);
		}
	}

	//�齨AOA-TDOA������
	sourceSize = allGSCopy.size();
#pragma omp parallel for schedule(dynamic)
	for (int refId = 0; refId < refSources.size(); ++refId) {
		auto& curRefSource = refSources[refId];
		for (int i = 0; i < sourceSize; ++i) {
			for (int j = i + 1; j < sourceSize; ++j) {
				GeneralSource* gs1 = allGSCopy[i];
				GeneralSource* gs2 = allGSCopy[j];
				if (gs1 == curRefSource || gs2 == curRefSource) {			//������ͬ�ο�����Դ
					continue;
				}
				GSPair* newPair = new GSPair(curRefSource, gs1, gs2);
				if (!newPair->HasValidAOATDOASolution(scene)) {					//������Դ����Ч����ɾ���ù���Դ�ԣ�����ֹͣ����
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

	//ɾ��Ȩ��Ϊ0�Ĺ���Դ
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {		//�Ƴ���Ч�Ĺ���Դ
		return !source->IsValid();
		}), allGSCopy.end());

	if (allGSCopy.size() == 0) {
		return Point2D(0, 0);
	}

	//�����й���ԴȨ����0
	for (int i = 0; i < allGSCopy.size(); ++i) {
		allGSCopy[i]->m_wCount = 0;					//Ȩ�ؼ�������
		allGSCopy[i]->m_weight = 0;					//Ȩ�ع���
	}

	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(gsPairs, scene, gsPairClusterThreshold, extendAroundPointState, lbsShiftErrorMatrix);

	int clusterNum = static_cast<int>(gsPairClusters.size());																	//������

	int max_cluster_num = 0;
	//��ȡ����Դ���ظ�Ԫ��,���ڽǶ�ƽ��
	allGSCopy.clear();
	for (auto& cluster : gsPairClusters) {
		cluster.GetNonRepeatGeneralSource(allGSCopy);
		max_cluster_num = std::max(max_cluster_num, static_cast<int>(cluster.m_pairs.size()));
	}
	sourceSize = allGSCopy.size();

	//ɾ����Ч��pair
	for (auto& curPair : gsPairs) {
		if (curPair->m_isValid == false) {
			delete curPair;
			curPair = nullptr;
		}
	}
	gsPairs.erase(std::remove_if(gsPairs.begin(), gsPairs.end(), [](const GSPair* pair) {			//���pair����Ч������
		return pair == nullptr;
		}), gsPairs.end());

	//2-2  ��������Լ����������Ȩ�ز�ɾ��������Ȩ����ֵ�Ĺ���Դ
	// ��������׷���㷨�������¼������Դ�����Դ֮����ܵĽ��ֵ������ֵ���������ƶ���ֵ��������

	std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	const MaterialLibrary* matLibrary = &scene->m_materialLibrary;
	const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	int sensorNum = static_cast<int>(scene->m_sensors.size());																	/** @brief	����������	*/
	std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	ʵʱ����Ŀ��Դ�����������	*/
	std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	ԭʼ�Ĵ���������	*/
	scene->m_sensorDataLibrary.GetAllSensorDataCollection(originalSensorDataCollection);										//��ȡԭʼ����������

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

	//Ϊ��ͬ���Ƚϲв���ｫԭʼ���ݰ��չ��ʵĴ�С��������
	RtLbsType max_r_phi = 0.0;																									/** @brief	�Ƕ����в�	*/
	RtLbsType max_r_timeDiff = 0.0;																								/** @brief	ʱ������в�	*/
	RtLbsType max_r_powerDiff = 0.0;																							/** @brief	���ʲ����в�	*/
	RtLbsType mean_r_phi = 0.0;																									/** @brief	�Ƕ�ƽ���в�	*/
	RtLbsType mean_r_timeDiff = 0.0;																							/** @brief	ʱ���ƽ���в�	*/
	RtLbsType mean_r_powerDiff = 0.0;																							/** @brief	���ʲ�ƽ���в�	*/

	for (auto& curCluster : gsPairClusters) {																					//�������дؼ������в�
		RtLbsType curCluster_min_r_phi = FLT_MAX;
		RtLbsType curCluster_min_r_timeDiff = FLT_MAX;
		RtLbsType curCluster_min_r_powerDiff = FLT_MAX;
		int curCluster_min_nullDataNum = INT_MAX;
		RtLbsType curCluster_min_r_weight = FLT_MAX;

		std::vector<RtLbsType> curCluster_r_phis;																					/** @brief	��ǰ�ĽǶȲв�	*/
		std::vector<RtLbsType> curCluster_r_timeDiffs;																				/** @brief	��ǰ��ʱ���в�	*/
		std::vector<RtLbsType> curCluster_r_powerDiffs;																				/** @brief	��ǰ�Ĺ��ʲ�в�	*/
		std::vector<int> curCluster_nullDataNums;																					/** @brief	������������ƥ����	*/
		std::vector<RtLbsType> curCluster_r_weights;																				/** @brief	��ǰ�صĲв�Ȩ��	*/


		for (auto& curResult : curCluster.m_rtResult) {
			RtLbsType cur_r_phi = 0.0;
			RtLbsType cur_r_timeDiff = 0.0;
			RtLbsType cur_r_powerDiff = 0.0;
			int cur_nullDataNum = 0;
			RtLbsType cur_r_weight = 0.0;

			targetSensorDataCollection.clear();
			targetSensorDataCollection.resize(sensorNum);

			for (int j = 0; j < sensorNum; ++j) {
				if (curResult[j].m_commonPaths.size() == 0) {		//����ǰ�޶ྶ��������
					continue;
				}
				const Sensor* curSensor = scene->m_sensors[j];
				curResult[j].CalculateBaseInfo(curSensor, freqs, tranFunctionData, antLibrary);						//ִ�е�ż���,LBS��ż���
				curResult[j].GetMinDelaySensorData_Delay(targetSensorDataCollection[j], 0.0);						//����ʱ����̶ྶ
				//����cluster�еĲο�����Դ
				if (!curCluster.m_isRefGSOccupied) {
					curCluster.m_TDOA_REFGS_Point = curResult[0].GetRefGeneralSource();
					curCluster.m_isRefGSOccupied = true;
				}
				//������ɺ���Ҫɾ����ǰcluster�еĶྶ
				curResult[j].ReleaseAllRayPath();
			}

			//����ʱ���
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

		int minDataId = 0;							/** @brief	��չ���е���С��IDֵ	*/
		if (curCluster.m_aroundPoints.size() != 1) {									//���ҽ�����Χ��չ����������1ʱ����
			for (int i = 0; i < static_cast<int>(curCluster_r_weights.size()); ++i) {
				if ((curCluster_min_r_weight > curCluster_r_weights[i]) && curCluster_nullDataNums[i] <= curCluster_nullDataNums[0]) {		//��Ѱ����Сֵ�Ĺ�������Ҫ��֤����������С�ڵ�һֵ
					curCluster_min_r_weight = curCluster_r_weights[i];
					minDataId = i;
				}
			}

			RtLbsType curCluster_min_r_weight_rectify = FLT_MAX;
			if (minDataId < (curCluster.m_nearExtendNum + 1) && curCluster_nullDataNums[0] != 0) {												//�������ڲ�����ݵ��µġ�����Ȩ�ء�����
				for (int i = 0; i < static_cast<int>(curCluster_r_weights.size()); ++i) {
					if (curCluster_min_r_weight_rectify > curCluster_r_weights[i] && curCluster_nullDataNums[i] == 0) {
						curCluster_min_r_weight_rectify = curCluster_r_weights[i];
						curCluster_min_r_weight = curCluster_r_weights[i];
						minDataId = i;
					}
				}
			}

			if (minDataId != 0 && minDataId > (curCluster.m_nearExtendNum + 1) && curCluster.m_deviateDistance > 5.0 && curCluster_min_r_weight / curCluster_r_weights[0] < 0.5) {					//����׷�Ӵ���5m����,����Զ������չ���µ������¼�
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

	for (auto curPair : gsPairs) {														//����pair��Ȩ�ز�Ѱ�����Ȩ��
		curPair->UpdateResidual_AOATDOA(mean_r_phi, mean_r_timeDiff, mean_r_powerDiff);
		max_r_phi = std::max(max_r_phi, curPair->m_phiResidual);
		max_r_timeDiff = std::max(max_r_timeDiff, curPair->m_timeDiffResidual);
		max_r_powerDiff = std::max(max_r_powerDiff, curPair->m_powerDiffResidual);
	}

	//ѭ��pair�����һ���в�ϵ��,����ϵ�������Ӧ�Ĺ���ԴȨ�ؾ�����
	RtLbsType maxPairWeight = 0.0;
	for (auto it = gsPairs.begin(); it != gsPairs.end(); ++it) {
		GSPair* curPair = *it;
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_AOATDOA(max_r_phi, max_r_timeDiff, max_r_powerDiff, weightFactor, max_cluster_num);
		}
		maxPairWeight = std::max(maxPairWeight, curPair->m_weight);
	}

	//��һ��Ȩ��
	for (auto& curSource : allGSCopy) {
		curSource->m_weight /= maxPairWeight;
	}
	for (auto& curCluster : gsPairClusters) {
		curCluster.m_weight /= maxPairWeight;
	}

	//��cluster����Ȩ�ؽ�������
	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);
	Point2D refClusterGSPoint = gsPairClusters.front().m_TDOA_REFGS_Point;
	//Ѱ����Ѳο�����Դ,���ַ�������һ�ַ���Ϊcluster��Ѱ�ң��ڶ��ַ���Ϊ����Ȩ��Ѱ��
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


	//����Դ���Ӽ����������ų���Ч�ο�����Դ
	for (auto& curPair : gsPairs) {
		if (curPair->m_gsRef == bestRefSource) {
			curPair->m_gs1->m_wCount += 1;
			curPair->m_gs2->m_wCount += 1;
		}
		else {
			curPair->m_gsRef->m_wCount = 0;			//����Ч�ο�����Դ��Ȩ������
		}
	}

	//����0.9Ȩ�����ϵ�cluster�еĹ���Դ
	std::vector<GeneralSource*> validGSs;
	for (auto& curCluster : gsPairClusters) {
		curCluster.GetNonRepeatGeneralSource(bestRefSource, validGSs);
	}

	Point2D initPoint = gsPairClusters.front().m_point;				/** @brief	��ʼ��	*/

	//���������
	AOATDOASolver solver;
	solver.SetGeneralSource(bestRefSource, validGSs);

	Point2D targetPoint = initPoint;
	targetPoint = solver.Solving(lbsConfig.m_solvingConfig, scene->m_bbox, weightFactor, initPoint);

	if ((initPoint - targetPoint).Length() > 100) {							//��ƫ�Ƴ̶ȹ�����ָ�ԭʼ�⣨�㷨����Ч��
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

	////������ɺ�ɾ�����й���Դ
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
	//0-�������Դ��λ��
	for (auto& curInfo : lbsInfos) {
		curInfo->CalculateBaseInfo(LBS_METHOD_RT_AOA_TDOA);
	}

	std::vector<GeneralSource*> mergedGSources;								/** @brief	�ϲ��Ĺ���Դ	*/

	//1-�ϲ���λ���-������������Ĺ���Դ���кϲ�
	size_t sourceSize = 0;
	for (auto& curInfo : lbsInfos) {
		size_t oldSize = mergedGSources.size();
		sourceSize += curInfo->m_sources.size();
		mergedGSources.resize(sourceSize);
		std::copy(curInfo->m_sources.begin(), curInfo->m_sources.end(), mergedGSources.begin() + oldSize);
	}

	std::vector<GeneralSource*> allGSCopy = mergedGSources;						/** @brief	���й���Դ�ĸ���	*/


	//2-���ռ���Լ������ɾ����Ч����Դ
	//��������Դ��,����Ϊ n*(n-1)/2

	bool hasAccurateSolution = false;

	size_t pairNum = static_cast<int>(sourceSize * (sourceSize - 1) / 2);
	std::vector<GSPair*> gsPairs;													/** @brief	����Դ��	*/
	gsPairs.reserve(pairNum);

	size_t pairId = 0;																/** @brief	����Դ��ID	*/
#pragma omp parallel for schedule(static)
	for (int i = 0; i < sourceSize; ++i) {
		for (int j = i + 1; j < sourceSize; ++j) {
			GSPair* newPair = new GSPair(allGSCopy[i], allGSCopy[j]);
			if (!newPair->HasValidAOASolution(scene)) {					//������Դ����Ч����ɾ���ù���Դ�ԣ�����ֹͣ����
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



	//ɾ��Ȩ��Ϊ0ֵ�Ĺ���Դ-���ͷ��ڴ棬���������ɾ��
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {		//�Ƴ���Ч�Ĺ���Դ
		return !source->IsValid();
		}), allGSCopy.end());

	//��������׷�ٻ������㷽�������Ƕ���


	//ɾ��pair
	for (auto& curPair : gsPairs) {
		delete curPair;
		curPair = nullptr;
	}
	gsPairs.clear();



    //AOA��ɸ������Դ,ɾ����Ч����Դ
	EraseRepeatGeneralSources(allGSCopy);			//ɾ���ظ��Ĺ���Դ


	//ɸѡ���ο�����Դ�����������Դ����
	std::vector<GeneralSource*> refSources;
	for (auto& curSource : allGSCopy) {
		curSource->m_wCount = 0;
		if (curSource->m_sensorData.m_timeDiff == 0) {
			refSources.push_back(curSource);
		}
	}
	//ɾ�����й���Դ�еĲο�����Դ
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {		//�Ƴ���Ч�Ĺ���Դ
		return source->m_sensorData.m_id == 0;
		}), allGSCopy.end());


    //�齨AOA-TDOA������
	sourceSize = allGSCopy.size();
#pragma omp parallel for schedule(static)
	for (int refId = 0; refId < refSources.size(); ++refId) {
		auto& curRefSource = refSources[refId];
		for (int i = 0; i < sourceSize; ++i) {
			for (int j = i + 1; j < sourceSize; ++j) {
				GeneralSource* gs1 = allGSCopy[i];
				GeneralSource* gs2 = allGSCopy[j];
				if (gs1 == curRefSource || gs2 == curRefSource) {			//������ͬ�ο�����Դ
					continue;
				}
				GSPair* newPair = new GSPair(curRefSource, gs1, gs2);
				if (!newPair->HasValidAOATDOASolution(scene)) {					//������Դ����Ч����ɾ���ù���Դ�ԣ�����ֹͣ����
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



	//ɾ��Ȩ��Ϊ0�Ĺ���Դ
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {		//�Ƴ���Ч�Ĺ���Դ
		return !source->IsValid();
		}), allGSCopy.end());

	//�����й���ԴȨ����0
	for (int i = 0; i < allGSCopy.size(); ++i) {
		allGSCopy[i]->m_wCount = 0;					//Ȩ�ؼ�������
		allGSCopy[i]->m_weight = 0;					//Ȩ�ع���
	}

	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(gsPairs, scene, gsPairClusterThreshold, extendAroundPointState, lbsShiftErrorMatrix);

	int clusterNum = static_cast<int>(gsPairClusters.size());																	//������

	int max_cluster_num = 0;
	//��ȡ����Դ���ظ�Ԫ��,���ڽǶ�ƽ��
	allGSCopy.clear();
	for (auto& cluster : gsPairClusters) {
		cluster.GetNonRepeatGeneralSource(allGSCopy);
		max_cluster_num = std::max(max_cluster_num, static_cast<int>(cluster.m_pairs.size()));
	}
	sourceSize = allGSCopy.size();

	//ɾ����Ч��pair
	for (auto& curPair : gsPairs) {
		if (curPair->m_isValid == false) {
			delete curPair;
			curPair = nullptr;
		}
	}
	gsPairs.erase(std::remove_if(gsPairs.begin(), gsPairs.end(), [](const GSPair* pair) {			//���pair����Ч������
		return pair == nullptr;
		}), gsPairs.end());

	//2-2  ��������Լ����������Ȩ�ز�ɾ��������Ȩ����ֵ�Ĺ���Դ
	// ��������׷���㷨�������¼������Դ�����Դ֮����ܵĽ��ֵ������ֵ���������ƶ���ֵ��������

	std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	const MaterialLibrary* matLibrary = &scene->m_materialLibrary;
	const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	int sensorNum = static_cast<int>(scene->m_sensors.size());																	/** @brief	����������	*/
	std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	ʵʱ����Ŀ��Դ�����������	*/
	std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	ԭʼ�Ĵ���������	*/
	scene->m_sensorDataLibrary.GetAllSensorDataCollectionWithError_AOATDOA(originalSensorDataCollection);										//��ȡԭʼ����������
	//Ϊ��ͬ���Ƚϲв���ｫԭʼ���ݰ��չ��ʵĴ�С��������
	RtLbsType max_r_phi = 0.0;																									/** @brief	�Ƕ����в�	*/
	RtLbsType max_r_timeDiff = 0.0;																								/** @brief	ʱ������в�	*/
	RtLbsType max_r_powerDiff = 0.0;																							/** @brief	���ʲ����в�	*/
	RtLbsType mean_r_phi = 0.0;																									/** @brief	�Ƕ�ƽ���в�	*/
	RtLbsType mean_r_timeDiff = 0.0;																							/** @brief	ʱ���ƽ���в�	*/
	RtLbsType mean_r_powerDiff = 0.0;																							/** @brief	���ʲ�ƽ���в�	*/

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

	for (auto& curCluster : gsPairClusters) {																					//�������дؼ������в�
		RtLbsType curCluster_min_r_phi = FLT_MAX;
		RtLbsType curCluster_min_r_timeDiff = FLT_MAX;
		RtLbsType curCluster_min_r_powerDiff = FLT_MAX;
		int curCluster_min_nullDataNum = INT_MAX;
		RtLbsType curCluster_min_r_weight = FLT_MAX;

		std::vector<RtLbsType> curCluster_r_phis;																					/** @brief	��ǰ�ĽǶȲв�	*/
		std::vector<RtLbsType> curCluster_r_timeDiffs;																				/** @brief	��ǰ��ʱ���в�	*/
		std::vector<RtLbsType> curCluster_r_powerDiffs;																				/** @brief	��ǰ�Ĺ��ʲ�в�	*/
		std::vector<int> curCluster_nullDataNums;																					/** @brief	������������ƥ����	*/
		std::vector<RtLbsType> curCluster_r_weights;																				/** @brief	��ǰ�صĲв�Ȩ��	*/


		for (auto& curResult : curCluster.m_rtResult) {
			RtLbsType cur_r_phi = 0.0;
			RtLbsType cur_r_timeDiff = 0.0;
			RtLbsType cur_r_powerDiff = 0.0;
			int cur_nullDataNum = 0;
			RtLbsType cur_r_weight = 0.0;

			targetSensorDataCollection.clear();
			targetSensorDataCollection.resize(sensorNum);

			for (int j = 0; j < sensorNum; ++j) {
				if (curResult[j].m_commonPaths.size() == 0) {		//����ǰ�޶ྶ��������
					continue;
				}
				const Sensor* curSensor = scene->m_sensors[j];
				curResult[j].CalculateBaseInfo(curSensor, freqs, tranFunctionData, antLibrary);						//ִ�е�ż���,LBS��ż���
				curResult[j].GetAllSensorData_Delay(targetSensorDataCollection[j], 0.0, 1.0);						//�������жྶ
				CalculateSensorCollectionResidual_AOATDOA_MultiData(originalSensorDataCollection, targetSensorDataCollection, weightFactor, cur_r_phi, cur_r_timeDiff, cur_r_powerDiff, cur_nullDataNum);

				cur_r_weight = weightFactor.m_phiWeight * cur_r_phi + weightFactor.m_timeWeight * cur_r_timeDiff + weightFactor.m_powerWeight * cur_r_powerDiff;

				curCluster_r_phis.push_back(cur_r_phi);
				curCluster_r_timeDiffs.push_back(cur_r_timeDiff);
				curCluster_r_powerDiffs.push_back(cur_r_powerDiff);
				curCluster_nullDataNums.push_back(cur_nullDataNum);
				curCluster_r_weights.push_back(cur_r_weight);

				//����cluster�еĲο�����Դ
				if (!curCluster.m_isRefGSOccupied) {
					curCluster.m_TDOA_REFGS_Point = curResult[j].GetRefGeneralSource();
					curCluster.m_isRefGSOccupied = true;
				}
				//������ɺ���Ҫɾ����ǰcluster�еĶྶ
				curResult[j].ReleaseAllRayPath();
			}
		}

		if (curCluster_r_weights.size() == 0) {			//����·�����ݣ���Ϊ��Ч��
			curCluster.SetInValidState();
			continue;
		}

		int minDataId = 0;							/** @brief	��չ���е���С��IDֵ	*/
		int datasize = static_cast<int>(curCluster_r_weights.size());					/** @brief	����������Ч����������	*/
		if (datasize > 1) {									//���ҽ�����Χ��չ����������1ʱ����
			//�ش�汾���£����Ӷ�Զ����չ��ĺ�����
			
			//�������ƽ��Ȩ��
			RtLbsType max_cluster_r_weight = vectoroperator::GetMax(curCluster_r_weights);
			for (int i = 0; i < datasize; ++i) {
				curCluster_r_weights[i] += max_cluster_r_weight * curCluster_nullDataNums[i];
			}

			//ѡȡ������С�в�
			minDataId = vectoroperator::GetMinIndex(curCluster_r_weights);


			if (minDataId != 0 && minDataId > (curCluster.m_nearExtendNum + 1) && curCluster.m_deviateDistance > 5.0 && curCluster_min_r_weight / curCluster_r_weights[0] < 0.5) {					//����׷�Ӵ���5m����,����Զ������չ���µ������¼�
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

	for (auto curPair : gsPairs) {														//����pair��Ȩ�ز�Ѱ�����Ȩ��
		curPair->UpdateResidual_AOATDOA(mean_r_phi, mean_r_timeDiff, mean_r_powerDiff);
		max_r_phi = std::max(max_r_phi, curPair->m_phiResidual);
		max_r_timeDiff = std::max(max_r_timeDiff, curPair->m_timeDiffResidual);
		max_r_powerDiff = std::max(max_r_powerDiff, curPair->m_powerDiffResidual);
	}

	//ѭ��pair�����һ���в�ϵ��,����ϵ�������Ӧ�Ĺ���ԴȨ�ؾ�����
	RtLbsType maxPairWeight = 0.0;
	for (auto it = gsPairs.begin(); it != gsPairs.end(); ++it) {
		GSPair* curPair = *it;
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_AOATDOA(max_r_phi, max_r_timeDiff, max_r_powerDiff, weightFactor, max_cluster_num);
		}
		maxPairWeight = std::max(maxPairWeight, curPair->m_weight);
	}

	//��һ��Ȩ��
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

	//��cluster����Ȩ�ؽ�������
	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);
	Point2D refClusterGSPoint = gsPairClusters.front().m_TDOA_REFGS_Point;
	//Ѱ����Ѳο�����Դ,���ַ�������һ�ַ���Ϊcluster��Ѱ�ң��ڶ��ַ���Ϊ����Ȩ��Ѱ��
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


	//����Դ���Ӽ����������ų���Ч�ο�����Դ
	for (auto& curPair : gsPairs) {
		if (curPair->m_gsRef == bestRefSource) {
			curPair->m_gs1->m_wCount += 1;
			curPair->m_gs2->m_wCount += 1;
		}
		else {
			curPair->m_gsRef->m_wCount = 0;			//����Ч�ο�����Դ��Ȩ������
		}
	}
	
	//����0.9Ȩ�����ϵ�cluster�еĹ���Դ
	std::vector<GeneralSource*> validGSs;
	for (auto& curCluster : gsPairClusters) {
		curCluster.GetNonRepeatGeneralSource(bestRefSource, validGSs);
	}

	Point2D initPoint = gsPairClusters.front().m_point;				/** @brief	��ʼ��	*/

	//���������
	AOATDOASolver solver;
	solver.SetGeneralSource(bestRefSource, allGSCopy);

	Point2D targetPoint = initPoint;
	targetPoint = solver.Solving(lbsConfig.m_solvingConfig, scene->m_bbox, weightFactor, initPoint);

	if ((initPoint - targetPoint).Length() > 200) {							//��ƫ�Ƴ̶ȹ�����ָ�ԭʼ�⣨�㷨����Ч��
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

	////������ɺ�ɾ�����й���Դ
	//for (auto& source : mergedGSources) {
	//	delete source;
	//	source = nullptr;
	//}
	mergedGSources.clear();
	std::vector<GeneralSource*>().swap(mergedGSources);

	return targetPoint;
}
