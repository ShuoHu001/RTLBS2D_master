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

	////0-���������Ϣ-�������Դ��λ��
	//for (auto& curInfo : lbsInfos) {
	//	curInfo->CalculateBaseInfo(LBS_METHOD_RT_TDOA);
	//}

	////������ݣ�����ÿ���������������Ĺ���Դ���趨�ο�����Դ������γɷ����Է���
	//int sensorNum = static_cast<int>(lbsInfos.size());
	////��ÿ������Դ��ֵ��Ӧ�Ĵ���������
	//std::vector<SensorData> sensorDatas;
	//scene->m_sensorDataLibrary.GetAllSensorData(sensorDatas);
	//for (int i = 0; i < sensorNum; ++i) {
	//	lbsInfos[i]->SetSensorData(sensorDatas[i]);
	//}
	//std::vector<GeneralSource*> refSources = lbsInfos[0]->m_sources;												/** @brief	�ο�����Դ, Ĭ�ϵ�0������������Ϊ�ο����ݴ�����	*/
	//std::vector<GeneralSource*> dataSources;																			/** @brief	���ݹ���Դ	*/
	//int dataSourceSize = 0;																								/** @brief	���ݹ���Դ������	*/
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


	////��������׷���㷨����Ŀ��⵽����������֮��Ĳв�
	//int clusterNum = static_cast<int>(gsPairClusters.size());
	//std::vector<std::vector<RaytracingResult>> tempRTResult;									//��һά��Ϊ��������������ڶ�ά��Ϊ����������������������׷�٣��ж���Դ���������Ч��
	//std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	//const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	//tempRTResult.resize(clusterNum);																/** @brief	����������	*/
	//std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	ʵʱ����Ŀ��Դ�����������	*/
	//std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	ԭʼ�Ĵ���������	*/
	//scene->m_sensorDataLibrary.GetAllSensorDataCollection(originalSensorDataCollection);										//��ȡԭʼ����������
	//std::sort(originalSensorDataCollection.begin(), originalSensorDataCollection.end(), ComparedByPower_SensorDataCollection);	//���չ��ʶԴ��������ݽ�������

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
	//RtLbsType mean_r_timeDiff = 0.0;																							/** @brief	ʱ���ƽ���в�	*/
	//RtLbsType mean_r_powerDiff = 0.0;																							/** @brief	����ƽ���в�	*/

	//for (auto& curCluster : gsPairClusters) {
	//	RtLbsType curCluster_min_r_timeDiff = FLT_MAX;																			/** @brief	���ʱ�Ӳ�в�	*/
	//	RtLbsType curCluster_min_r_powerDiff = FLT_MAX;																			/** @brief	����ʲ�в�	*/
	//	int curCluster_min_nullDataNum = INT_MAX;																				/** @brief	������������	*/

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
	//		CalculateSensorCollectionResidual_TDOA_SingleData(originalSensorDataCollection, targetSensorDataCollection, cur_r_timeDiff, cur_r_powerDiff, cur_nullDataNum);		//����в��������
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

	////����clusterȨ��
	//RtLbsType maxWeight = 0.0;
	//for (auto& curCluster : gsPairClusters) {
	//	curCluster.CalNormalizedTDOAWeight(max_r_timeDiff, max_r_powerDiff, weightFactor);
	//	maxWeight = std::max(maxWeight, curCluster.m_weight);
	//}

	////�����һ��Ȩ��
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
	std::vector<SensorData> sensorDatas;									/** @brief	��ȡ�������д���������	*/
	scene->m_sensorDataLibrary.GetAllSensorData(sensorDatas);
	//0-�������Դ��λ��
	for (auto& curInfo : lbsInfos) {
		curInfo->CalculateBaseInfo(sensorDatas, LBS_METHOD_RT_TDOA);
	}

	std::vector<GeneralSource*> mergedGSources;								/** @brief	���й���Դ	*/

	//1-�ϲ���λ���-������������Ĺ���Դ���кϲ�
	size_t sourceSize = 0;
	for (auto& curInfo : lbsInfos) {
		size_t oldSize = mergedGSources.size();
		sourceSize += curInfo->m_sources.size();
		mergedGSources.resize(sourceSize);
		std::copy(curInfo->m_sources.begin(), curInfo->m_sources.end(), mergedGSources.begin() + oldSize);
	}

	std::vector<GeneralSource*> allGSCopy = mergedGSources;						/** @brief	���й���Դ�ĸ���	*/
	int sourceNum = static_cast<int>(allGSCopy.size());							/** @brief	���й���Դ������	*/

	//2-���Ȩ�ؾ���

	std::vector<GSPair*> gsPairs;

	//ÿһ��Դ���п��ܳ�Ϊ�ο�Դ
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < sourceNum; ++i) {
		GeneralSource* refSource = allGSCopy[i];
		for (int j = i + 1; j < sourceNum; j++) {
			for (int k = j + 1; k < sourceNum; k++) {
				GeneralSource* data1 = allGSCopy[j];
				GeneralSource* data2 = allGSCopy[k];
				if (!PreCheckGSPairValidation_TDOA(refSource, data1, data2)) {			//��Ԥ��鲻ͨ��������Ч
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

	//ɾ������Ϊ0�Ĺ���Դ
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {		//�Ƴ���Ч�Ĺ���Դ
		return !source->IsValid();
		}), allGSCopy.end());

	//�����й���ԴȨ����0
	for (int i = 0; i < allGSCopy.size(); ++i) {
		allGSCopy[i]->m_wCount = 0;					//Ȩ�ؼ�������
		allGSCopy[i]->m_weight = 0;					//Ȩ�ع���
	}

	//2-2 �⼯��������
	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(gsPairs, scene, gsPairClusterThreshold, extendAroundPointState, lbsShiftErrorMatrix);
	int clusterNum = static_cast<int>(gsPairClusters.size());																	//������

	int max_cluster_num = 0;
	for (auto& cluster : gsPairClusters) {
		max_cluster_num = std::max(max_cluster_num, static_cast<int>(cluster.m_pairs.size()));
	}


	//����������������Լ�������Ȩ�ؾ���
	std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	const MaterialLibrary* matLibrary = &scene->m_materialLibrary;
	const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	int sensorNum = static_cast<int>(scene->m_sensors.size());																	/** @brief	����������	*/
	std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	ʵʱ����Ŀ��Դ�����������	*/
	std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	ԭʼ�Ĵ���������	*/
	scene->m_sensorDataLibrary.GetAllSensorDataCollectionWithTDOAError(originalSensorDataCollection);							//��ȡԭʼ����������
	std::sort(originalSensorDataCollection.begin(), originalSensorDataCollection.end(), ComparedByPower_SensorDataCollection);	//���չ��ʶԴ��������ݽ�������

	//Ϊ��ͬ���Ƚϲв���ｫԭʼ���ݰ��չ��ʵĴ�С��������
	RtLbsType max_r_timeDiff = 0.0;																									//ʱ������в�
	RtLbsType max_r_powerDiff = 0.0;																								//���ʲ����в�
	RtLbsType mean_r_timeDiff = 0.0;																								/** @brief	ʱ���ƽ���в�	*/
	RtLbsType mean_r_powerDiff = 0.0;																								/** @brief	���ʲ�ƽ���в�	*/

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
		RtLbsType curCluster_min_r_timeDiff = FLT_MAX;
		RtLbsType curCluster_min_r_powerDiff = FLT_MAX;
		int curCluster_min_nullDataNum = INT_MAX;
		RtLbsType curCluster_min_r_weight = FLT_MAX;

		std::vector<RtLbsType> curCluster_r_timeDiffs;																					/** @brief	��ǰ��ʱ��в�	*/
		std::vector<RtLbsType> curCluster_r_powerDiffs;																				/** @brief	��ǰ�Ĺ��ʲ�в�	*/
		std::vector<int> curCluster_nullDataNums;																					/** @brief	������������ƥ����	*/
		std::vector<RtLbsType> curCluster_r_weights;																				/** @brief	��ǰ�صĲв�Ȩ��	*/

		for (auto& curResult : curCluster.m_rtResult) {
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
				CalculateSensorCollectionResidual_TDOA_MultiData(originalSensorDataCollection, targetSensorDataCollection, weightFactor, cur_r_timeDiff, cur_r_powerDiff, cur_nullDataNum);

				cur_r_weight = weightFactor.m_timeWeight * cur_r_timeDiff + weightFactor.m_powerWeight * cur_r_powerDiff;

				curCluster_r_timeDiffs.push_back(cur_r_timeDiff);
				curCluster_r_powerDiffs.push_back(cur_r_powerDiff);
				curCluster_nullDataNums.push_back(cur_nullDataNum);
				curCluster_r_weights.push_back(cur_r_weight);

				//TDOA �㷨��Ҫ���Ӽ���cluster�еĲο�����Դ
				if (!curCluster.m_isRefGSOccupied) {
					curCluster.m_TDOA_REFGS_Point = curResult[j].GetRefGeneralSource();
					curCluster.m_isRefGSOccupied = true;
				}

				//������ɺ���Ҫɾ����ǰcluster�еĶྶ
				curResult[j].ReleaseAllRayPath();
			}
		}

		if (curCluster_r_weights.size() == 0) {
			curCluster.m_isValid = false;
			curCluster.m_weight = 0.0;
			curCluster.SetInValidState();
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


		curCluster_min_r_timeDiff = curCluster_r_timeDiffs[minDataId];
		curCluster_min_r_powerDiff = curCluster_r_powerDiffs[minDataId];
		curCluster_min_nullDataNum = curCluster_nullDataNums[minDataId];
		curCluster.m_point = curCluster.m_aroundPoints[minDataId];										//���µ�ǰ�ص����ĵ�

		curCluster.SetElementTDOAResidual(curCluster_min_r_timeDiff, curCluster_min_r_powerDiff, curCluster_min_nullDataNum);

		mean_r_timeDiff += curCluster_min_r_timeDiff;
		mean_r_powerDiff += curCluster_min_r_powerDiff;
	}

	mean_r_timeDiff /= clusterNum;
	mean_r_powerDiff /= clusterNum;

	for (auto curPair : gsPairs) {														//����pair��Ȩ�ز�Ѱ�����Ȩ��
		curPair->UpdateResidual_TDOA(mean_r_timeDiff, mean_r_powerDiff);
		max_r_timeDiff = std::max(max_r_timeDiff, curPair->m_timeDiffResidual);
		max_r_powerDiff = std::max(max_r_powerDiff, curPair->m_powerDiffResidual);
	}

	//ѭ��pair�����һ���в�ϵ��,����ϵ�������Ӧ�Ĺ���ԴȨ�ؾ�����
	for (auto it = gsPairs.begin(); it != gsPairs.end(); ++it) {
		GSPair* curPair = *it;
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_TDOA(max_r_timeDiff, max_r_powerDiff, weightFactor, max_cluster_num);
		}
	}



	//ɾ���ظ��Ĺ���Դ
	EraseRepeatGeneralSources(allGSCopy);			//ɾ���ظ��Ĺ���Դ


	//��cluster����Ȩ�ؽ�������,����Ѱ�����Ȩ��
	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);

	//�������й���Դ��Ȩ��������ֵ������Ȩ�ع�һ��
	RtLbsType max_weight = gsPairClusters.front().m_weight;
	for (auto it = allGSCopy.begin(); it != allGSCopy.end(); ++it) {
		GeneralSource*& curSource = *it;
		curSource->NormalizedWeight(max_weight);
	}


	//������cluster��Ȩ�غ���Ҫ����cluster�еı��ʽ���Ȩ�����󣬲�Ӱ�����Դ��Ȩ��
	for (auto& curCluster : gsPairClusters) {
		curCluster.m_weight *= curCluster.m_pairs.size();
	}

	//��cluster����Ȩ�ؽ������򣬸��´�Ȩ��
	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);

	//ѡȡȨ������cluster�еĲο�Դ��Ϊ�ο�����Դ��ԭ����С�в��µ�cluster�еĲο�Դ������׼ȷ��Դ��
	Point2D refClusterGSPoint = gsPairClusters.front().m_TDOA_REFGS_Point;
	GeneralSource* bestRefSource = nullptr;
	for (auto& curGS : allGSCopy) {
		if ((curGS->m_position - refClusterGSPoint).Length() < 1e-2) {
			bestRefSource = curGS;
			break;
		}
	}
	if (bestRefSource == nullptr) {//��cluster��Ӧ�Ĳο�Դ�Ҳ�������Դ������Ҫ���¼���Ȩ�أ��õ�����Ȩ�����Ĳο�����Դ
		for (auto& curCluster : gsPairClusters) {
			if (curCluster.m_weight > 0.9) {
				curCluster.CalculateRefSourceCount();
			}
		}
		std::sort(allGSCopy.begin(), allGSCopy.end(), ComparedByWCount_GeneralSource);
		bestRefSource = allGSCopy.front();
	}


	//�����ο�ԴΪ��Ѳο�Դ�����й���Դ
	std::vector<GeneralSource*> validGSs;
	for (auto& curCluster : gsPairClusters) {
		curCluster.GetNonRepeatGeneralSource(bestRefSource, validGSs);
	}

	Point2D initPoint = gsPairClusters.front().m_point;				/** @brief	��ʼ��	*/

	//���������
	TDOASolver solver;
	solver.SetGeneralSource(bestRefSource, validGSs);

	Point2D targetPoint = initPoint;
	targetPoint = solver.Solving(lbsConfig.m_solvingConfig, scene->m_bbox, weightFactor, initPoint);

	if ((initPoint - targetPoint).Length() > 20) {							//��ƫ�Ƴ̶ȹ�����ָ�ԭʼ�⣨�㷨����Ч��
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

	//������ɺ�ɾ�����й���Դ
	for (auto& source : mergedGSources) {
		delete source;
		source = nullptr;
	}
	mergedGSources.clear();
	std::vector<GeneralSource*>().swap(mergedGSources);

	return targetPoint;
}
