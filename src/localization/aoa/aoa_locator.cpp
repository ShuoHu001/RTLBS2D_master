#include "aoa_locator.h"

Point2D LBS_AoA_Locator_MPSTSD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, const LocalizeConfig& lbsConfig, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData)
{
	LOSSFUNCTIONTYPE lossType = lbsConfig.m_solvingConfig.m_lossType;
	RtLbsType gsPairClusterThreshold = lbsConfig.m_gsPairClusterThreshold;
	bool extendAroundPointState = lbsConfig.m_extendAroundPointState;
	WeightFactor weightFactor = lbsConfig.m_weightFactor;
	HARDWAREMODE hardwareMode = lbsConfig.m_hardWareMode;
	uint16_t threadNum = lbsConfig.m_threadNum;
	weightFactor.InitAOAWeight();

	std::vector<LBSInfo*>& lbsInfos = lbsInfoCluster.m_infos;
	//0-���������Ϣ-�������Դ��λ�� 
	for (auto& curInfo : lbsInfos) {
		curInfo->CalculateBaseInfo(LBS_METHOD_RT_AOA);
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
	for (size_t i = 0; i < sourceSize; ++i) {
		for (size_t j = i + 1; j < sourceSize; ++j) {
			GSPair* newPair = new GSPair(allGSCopy[i], allGSCopy[j]);
			if (!newPair->HasValidAOASolution(scene)) {					//������Դ����Ч����ɾ���ù���Դ�ԣ�����ֹͣ����
				delete newPair;
				continue;
			}
			allGSCopy[i]->m_wCount += 1;
			allGSCopy[j]->m_wCount += 1;
			gsPairs.push_back(newPair);
		}
	}


	//ɾ��Ȩ��Ϊ0ֵ�Ĺ���Դ-���ͷ��ڴ棬���������ɾ��
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {		//�Ƴ���Ч�Ĺ���Դ
		return !source->IsValid();
		}), allGSCopy.end());


	//�����й���ԴȨ����0
	for (int i = 0; i < mergedGSources.size(); ++i) {
		mergedGSources[i]->m_wCount = 0;					//Ȩ�ؼ�������
		mergedGSources[i]->m_weight = 0;					//Ȩ�ع���
	}

	//2-2 �⼯��������
	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(gsPairs, scene, gsPairClusterThreshold, extendAroundPointState, lbsShiftErrorMatrix);

	int clusterNum = static_cast<int>(gsPairClusters.size());																	//������

	int max_cluster_num = gsPairClusters.front().m_pairs.size();																//�������Ԫ������
	//��ȡ����Դ���ظ�Ԫ��,���ڽǶ�ƽ��
	allGSCopy.clear();
	for (auto& cluster : gsPairClusters) {
		cluster.GetNonRepeatGeneralSource(allGSCopy);
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
	//Ϊ��ͬ���Ƚϲв���ｫԭʼ���ݰ��չ��ʵĴ�С��������


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

	RtLbsType mean_r_phi = 0.0;																										/** @brief	�Ƕ�ƽ���в�	*/
	RtLbsType mean_r_powerDiff = 0.0;																								/** @brief	���ʲ�ƽ���в�	*/
	RtLbsType max_r_phi = 0.0;																										/** @brief	�Ƕ����в�	*/
	RtLbsType max_r_powerDiff = 0.0;																								/** @brief	�������в�	*/

	for (auto& curCluster : gsPairClusters) {
		RtLbsType curCluster_min_r_phi = FLT_MAX;																					/** @brief	��ǰ�����ǶȲв�	*/
		RtLbsType curCluster_min_r_powerDiff = FLT_MAX;																				/** @brief	��ǰ������ʲ�в�	*/
		int curCluster_min_nullDataNum = INT_MAX;																					/** @brief	��ǰ��������������	*/
		RtLbsType curCluster_min_r_weight = FLT_MAX;

		std::vector<RtLbsType> curCluster_r_phis;																					/** @brief	��ǰ�ؽǶȲв�	*/
		std::vector<RtLbsType> curCluster_r_powerDiffs;																				/** @brief	��ǰ�ع��ʲ�в�	*/
		std::vector<int> curCluster_nullDataNums;																					/** @brief	��ǰ�ؿ���������	*/
		std::vector<RtLbsType> curCluster_r_weights;																				/** @brief	��ǰ�صĲв�Ȩ��	*/


		for (auto& curResult : curCluster.m_rtResult) {																				//Ѱ��cluster�ж����result�еĲв���Сֵ
			RtLbsType cur_r_phi = 0.0;
			RtLbsType cur_r_powerDiff = 0.0;
			int cur_nullDataNum = 0;
			RtLbsType cur_r_weight = 0.0;


			targetSensorDataCollection.clear();
			targetSensorDataCollection.resize(sensorNum);

			for (int i = 0; i < sensorNum; ++i) {
				if (curResult[i].m_commonPaths.size() == 0) {		//����ǰ�޶ྶ��������
					continue;
				}
				const Sensor* curSensor = scene->m_sensors[i];
				curResult[i].CalculateBaseInfo(curSensor, freqs, tranFunctionData, antLibrary);
				curResult[i].GetMaxPowerSensorData_AOA2D(targetSensorDataCollection[i], curSensor->m_phiErrorSTD);
				curResult[i].ReleaseAllRayPath();																		//������ɺ��ͷŶྶ��Ϣ�����ں�������
			}
			CalculateSensorCollectionResidual_AOA_SingleData(originalSensorDataCollection, targetSensorDataCollection, cur_r_phi, cur_r_powerDiff, cur_nullDataNum);		//����в��������

			cur_r_weight = weightFactor.m_phiWeight*cur_r_phi+weightFactor.m_powerWeight*cur_r_powerDiff;
			curCluster_r_phis.push_back(cur_r_phi);
			curCluster_r_powerDiffs.push_back(cur_r_powerDiff);
			curCluster_nullDataNums.push_back(cur_nullDataNum);
			curCluster_r_weights.push_back(cur_r_weight);
			//curCluster_min_r_phi = std::min(curCluster_min_r_phi, cur_r_phi);
			//curCluster_min_r_powerDiff = std::min(curCluster_min_r_powerDiff, cur_r_powerDiff);
			//curCluster_min_nullDataNum = std::min(curCluster_min_nullDataNum, cur_nullDataNum);
		}

		//Ѱ�Ҵ��вв���Сֵ��Ӧ�Ľ�
		if (curCluster_r_weights.size() == 0) {			//��������Ȩ�����ݣ����Ϊ��Ч��
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
		curCluster_min_r_powerDiff = curCluster_r_powerDiffs[minDataId];
		curCluster_min_nullDataNum = curCluster_nullDataNums[minDataId];

		curCluster.SetElementAOAResidual(curCluster_min_r_phi, curCluster_min_r_powerDiff, 1.0, curCluster_min_nullDataNum);

		mean_r_phi += curCluster_min_r_phi;
		mean_r_powerDiff += curCluster_min_r_powerDiff;
	}
	mean_r_phi /= clusterNum;
	mean_r_powerDiff /= clusterNum;

	for (auto& curPair : gsPairs) {
		curPair->UpdateResidual_AOA(mean_r_phi, mean_r_powerDiff);											//����pair��Ȩ�ز�Ѱ�����Ȩ��
		max_r_phi = std::max(max_r_phi, curPair->m_phiResidual);
		max_r_powerDiff = std::max(max_r_powerDiff, curPair->m_powerDiffResidual);
	}

	//ѭ��pair�����һ���в�ϵ��,����ϵ�������Ӧ�Ĺ���ԴȨ�ؾ�����
	for (auto& curPair : gsPairs) {
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_AOA(max_r_phi, max_r_powerDiff, weightFactor, max_cluster_num);
		}
	}

	EraseRepeatGeneralSources(allGSCopy);			//ɾ���ظ��Ĺ���Դ

	//��cluster����Ȩ�ؽ�������
	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);

	//�������й���Դ��Ȩ��������ֵ������Ȩ�ع�һ��
	RtLbsType max_weight = gsPairClusters.front().m_weight;
	for (auto it = allGSCopy.begin(); it != allGSCopy.end(); ++it) {
		GeneralSource*& curSource = *it;
		curSource->NormalizedWeight(max_weight);
	}

	////ɾ������Ȩ����ֵ�Ĺ���Դ
	//allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* s) {
	//	return s->m_weight < 0.7;
	//	}), allGSCopy.end());

	//ɾ����Ч�Ĺ���Դ
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {
		return  !source->m_isValid;
		}), allGSCopy.end());


	//��ȡ���Ȩ�ص�Cluster,��������Ϊ��ʼ��
	RtLbsType maxClusterWeight = 0;
	Point2D initPoint = gsPairClusters.front().m_point;

	//����AOA�����
	AOASolver aoaSolver;
	aoaSolver.SetGeneralSource(mergedGSources);

	Point2D targetPoint = initPoint;
	targetPoint = aoaSolver.Solving(lbsConfig.m_solvingConfig, scene->m_bbox, initPoint);

	if ((initPoint - targetPoint).Length() > 50) {							//��ƫ�Ƴ̶ȹ�����ָ�ԭʼ�⣨�㷨����Ч��
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

Point2D LBS_AoA_Locator_SPSTMD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, const LocalizeConfig& lbsConfig, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData)
{
	LOSSFUNCTIONTYPE lossType = lbsConfig.m_solvingConfig.m_lossType;
	RtLbsType gsPairClusterThreshold = lbsConfig.m_gsPairClusterThreshold;
	bool extendAroundPointState = lbsConfig.m_extendAroundPointState;
	WeightFactor weightFactor = lbsConfig.m_weightFactor;
	HARDWAREMODE hardwareMode = lbsConfig.m_hardWareMode;
	uint16_t threadNum = lbsConfig.m_threadNum;
	weightFactor.InitAOAWeight();

	std::vector<LBSInfo*>& lbsInfos = lbsInfoCluster.m_infos;
	//0-���������Ϣ-�������Դ��λ�� 
	for (auto& curInfo: lbsInfos) {
		curInfo->CalculateBaseInfo(LBS_METHOD_RT_AOA);
	}

	std::vector<GeneralSource*> mergedGSources;

	//1-�ϲ���λ���-������������Ĺ���Դ���кϲ�
	size_t sourceSize = 0;
	for (auto& curInfo : lbsInfos) {
		size_t oldSize = mergedGSources.size();
		sourceSize += curInfo->m_sources.size();
		mergedGSources.resize(sourceSize);
		std::copy(curInfo->m_sources.begin(), curInfo->m_sources.end(), mergedGSources.begin() + oldSize);
	}

	std::vector<GeneralSource*> allGSCopy = mergedGSources;						/** @brief	���й���Դ�ĸ���	*/

	//2-���Ȩ�ؾ���

	//2-���ռ���Լ������ɾ����Ч����Դ
	//��������Դ��,����Ϊ n*(n-1)/2
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

	//�����й���ԴȨ����0
	for (int i = 0; i < allGSCopy.size(); ++i) {
		allGSCopy[i]->m_wCount = 0;					//Ȩ�ؼ�������
		allGSCopy[i]->m_weight = 0;					//Ȩ�ع���
	}

	//2-2 �⼯��������
	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(gsPairs, scene, gsPairClusterThreshold, extendAroundPointState, lbsShiftErrorMatrix);


	int clusterNum = static_cast<int>(gsPairClusters.size());																	//������

	int max_cluster_num = gsPairClusters.front().m_pairs.size();																//���������ݴ�С
	//��ȡ����Դ���ظ�Ԫ��,���ڽǶ�ƽ��
	allGSCopy.clear();
	for (auto& cluster : gsPairClusters) {
		cluster.GetNonRepeatGeneralSource(allGSCopy);
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
	scene->m_sensorDataLibrary.GetAllSensorDataCollectionWithAOAError(originalSensorDataCollection);							//��ȡԭʼ����������
	std::sort(originalSensorDataCollection.begin(), originalSensorDataCollection.end(), ComparedByPower_SensorDataCollection);	//���չ��ʶԴ��������ݽ�������
	//Ϊ��ͬ���Ƚϲв���ｫԭʼ���ݰ��չ��ʵĴ�С��������
	RtLbsType max_r_phi = 0.0;																									//�Ƕ����в�
	RtLbsType max_r_powerDiff = 0.0;																							//�������в�
	RtLbsType mean_r_phi = 0.0;																									/** @brief	�Ƕ�ƽ���в�	*/
	RtLbsType mean_r_powerDiff = 0.0;																							/** @brief	����ƽ���в�	*/



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




	for (auto& curCluster : gsPairClusters) {																				//�������д�
		RtLbsType curCluster_min_r_angularSpread = FLT_MAX;
		RtLbsType curCluster_min_r_phi = FLT_MAX;
		RtLbsType curCluster_min_r_powerDiff = FLT_MAX;
		int curCluster_min_nullDataNum = INT_MAX;
		RtLbsType curCluster_min_r_weight = FLT_MAX;

		std::vector<RtLbsType> curCluster_r_angularSpreads;																			/** @brief	��ǰ�ĽǶȲв�	*/
		std::vector<RtLbsType> curCluster_r_phis;																					/** @brief	��ǰ�Ĺ��ʲ�в�	*/
		std::vector<RtLbsType> curCluster_r_powerDiffs;																				/** @brief	��ǰ�ĽǶ���չ	*/
		std::vector<int> curCluster_nullDataNums;																					/** @brief	������������ƥ����	*/
		std::vector<RtLbsType> curCluster_r_weights;																					/** @brief	��ǰ�صĲв�Ȩ��	*/

		for (auto& curResult : curCluster.m_rtResult) {																				//�����������еĽ⣨λ�Ƶ��µĶ�⣩
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
				curResult[j].CalculateBaseInfo(curSensor, freqs, tranFunctionData, antLibrary);						//ִ�е�ż���,LBS��ż���
				curResult[j].GetAllSensorData_AOA2D(targetSensorDataCollection[j], 0.0, 1.0);			//�ռ�ʵʱ����Ĵ��������,�����Ƿ���,��˲�����ϡ�裬2.0�����ǡ�
				CalculateSensorCollectionResidual_AOA_MultiData(originalSensorDataCollection, targetSensorDataCollection, weightFactor, cur_r_phi, cur_r_powerDiff, cur_r_angularSpread, cur_nullDataNum);		//����в��������

				cur_r_weight = (weightFactor.m_powerWeight * cur_r_phi + weightFactor.m_phiWeight * cur_r_powerDiff) / (weightFactor.m_powerWeight + weightFactor.m_phiWeight);

				curCluster_r_angularSpreads.push_back(cur_r_angularSpread);
				curCluster_r_phis.push_back(cur_r_phi);
				curCluster_r_powerDiffs.push_back(cur_r_powerDiff);
				curCluster_r_weights.push_back(cur_r_weight);
				curCluster_nullDataNums.push_back(cur_nullDataNum);

				//������ɺ���Ҫɾ����ǰcluster�еĶྶ
				curResult[j].ReleaseAllRayPath();
			}
		}

		if (curCluster_r_weights.size() == 0) {
			curCluster.m_isValid = false;
			continue;
		}

		int minDataId = 0;
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



		curCluster_min_r_angularSpread = curCluster_r_angularSpreads[minDataId];
		curCluster_min_r_phi = curCluster_r_phis[minDataId];
		curCluster_min_r_powerDiff = curCluster_r_powerDiffs[minDataId];
		curCluster_min_nullDataNum = curCluster_nullDataNums[minDataId];

		//�������ڽǶ�ƫ�������0�͹�2pi��������
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
		curPair->UpdateResidual_AOA(mean_r_phi, mean_r_powerDiff);
		max_r_phi = std::max(max_r_phi, curPair->m_phiResidual);
		max_r_powerDiff = std::max(max_r_powerDiff, curPair->m_powerDiffResidual);
	}


	//ѭ��pair�����һ���в�ϵ��,����ϵ�������Ӧ�Ĺ���ԴȨ�ؾ�����
	for (auto it = gsPairs.begin(); it != gsPairs.end(); ++it) {
		GSPair* curPair = *it;
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_AOA(max_r_phi, max_r_powerDiff, weightFactor, max_cluster_num);
		}
	}


	//ɾ���ظ��Ĺ���Դ
	EraseRepeatGeneralSources(allGSCopy);			//ɾ���ظ��Ĺ���Դ

	//��cluster����Ȩ�ؽ�������
	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);


	//�������й���Դ��Ȩ��������ֵ������Ȩ�ع�һ��
	RtLbsType max_weight = gsPairClusters.front().m_weight;
	for (auto it = allGSCopy.begin(); it != allGSCopy.end(); ++it) {
		GeneralSource*& curSource = *it;
		curSource->NormalizedWeight(max_weight);
	}

	////ɾ������Ȩ����ֵ�Ĺ���Դ
	//allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* s) {
	//	return s->m_weight < 0.9;
	//	}), allGSCopy.end());

	//��Դ��λ�У�rootԴ���������ֻ����һ��
	int hasRootNum = 0;
	for (auto& source : allGSCopy) {
		if (source->m_type == NODE_ROOT) {
			hasRootNum++;
			if (hasRootNum > 1) {
				source->m_isValid = false;
			}
		}
	}

	//ɾ����Ч�Ĺ���Դ
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* source) {
		return  !source->m_isValid;
		}), allGSCopy.end());


	//��ȡ���Ȩ�ص�Cluster
	RtLbsType maxClusterWeight = 0;
	Point2D initPoint;
	for (auto& curCluster : gsPairClusters) {
		if (maxClusterWeight < curCluster.m_pairs[0]->m_weight) {
			maxClusterWeight = curCluster.m_pairs[0]->m_weight;
			initPoint = curCluster.m_point;
		}
	}


	//���������
	AOASolver aoaSolver;
	aoaSolver.SetGeneralSource(allGSCopy);

	Point2D targetPoint = initPoint;
	targetPoint = aoaSolver.Solving(lbsConfig.m_solvingConfig, scene->m_bbox, initPoint);

	if ((initPoint - targetPoint).Length() > 100) {				//��ƫ�Ƴ̶ȹ�����ָ�ԭʼ�㣨����Ч��
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
