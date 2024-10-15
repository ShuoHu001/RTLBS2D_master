#include "tdoa_locator.h"

Point2D LBS_TDOA_LOCATOR_MPSTSD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, HARDWAREMODE hardwareMode, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData,  LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, bool extendAroundPointState, const WeightFactor& weightFactor)
{

	std::vector<LBSInfo*>& lbsInfos = lbsInfoCluster.m_infos;

	//0-���������Ϣ-�������Դ��λ��
	for (auto& curInfo : lbsInfos) {
		curInfo->CalculateBaseInfo(method);
	}

	//������ݣ�����ÿ���������������Ĺ���Դ���趨�ο�����Դ������γɷ����Է���
	int sensorNum = static_cast<int>(lbsInfos.size());
	//��ÿ������Դ��ֵ��Ӧ�Ĵ���������
	std::vector<SensorData> sensorDatas;
	scene->m_sensorDataLibrary.GetAllSensorData(sensorDatas);
	for (int i = 0; i < sensorNum; ++i) {
		lbsInfos[i]->SetSensorData(sensorDatas[i]);
	}
	std::vector<GeneralSource*> refSources = lbsInfos[0]->m_sources;												/** @brief	�ο�����Դ, Ĭ�ϵ�0������������Ϊ�ο����ݴ�����	*/
	std::vector<GeneralSource*> dataSources;																			/** @brief	���ݹ���Դ	*/
	int dataSourceSize = 0;																								/** @brief	���ݹ���Դ������	*/
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


	//��������׷���㷨����Ŀ��⵽����������֮��Ĳв�
	int clusterNum = static_cast<int>(gsPairClusters.size());
	std::vector<std::vector<RaytracingResult>> tempRTResult;									//��һά��Ϊ��������������ڶ�ά��Ϊ����������������������׷�٣��ж���Դ���������Ч��
	std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	tempRTResult.resize(clusterNum);																/** @brief	����������	*/
	std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	ʵʱ����Ŀ��Դ�����������	*/
	std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	ԭʼ�Ĵ���������	*/
	scene->m_sensorDataLibrary.GetAllSensorDataCollection(originalSensorDataCollection);										//��ȡԭʼ����������
	std::sort(originalSensorDataCollection.begin(), originalSensorDataCollection.end(), ComparedByPower_SensorDataCollection);	//���չ��ʶԴ��������ݽ�������

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
	RtLbsType mean_r_timeDiff = 0.0;																							/** @brief	ʱ���ƽ���в�	*/
	RtLbsType mean_r_powerDiff = 0.0;																							/** @brief	����ƽ���в�	*/

	for (auto& curCluster : gsPairClusters) {
		RtLbsType curCluster_min_r_timeDiff = FLT_MAX;																			/** @brief	���ʱ�Ӳ�в�	*/
		RtLbsType curCluster_min_r_powerDiff = FLT_MAX;																			/** @brief	����ʲ�в�	*/
		int curCluster_min_nullDataNum = INT_MAX;																				/** @brief	������������	*/

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
			CalculateSensorCollectionResidual_TDOA_SingleData(originalSensorDataCollection, targetSensorDataCollection, cur_r_timeDiff, cur_r_powerDiff, cur_nullDataNum);		//����в��������
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

	//����clusterȨ��
	RtLbsType maxWeight = 0.0;
	for (auto& curCluster : gsPairClusters) {
		curCluster.CalNormalizedTDOAWeight(max_r_timeDiff, max_r_powerDiff, weightFactor);
		maxWeight = std::max(maxWeight, curCluster.m_weight);
	}

	//�����һ��Ȩ��
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

	//0-���������Ϣ-�������Դ��λ�� 
	for (auto& curInfo: lbsInfos) {
		curInfo->CalculateBaseInfo(method);
	}

	std::vector<GeneralSource*> mergedGSource;

	//1-�ϲ���λ���-������������Ĺ���Դ���кϲ�
	size_t sourceSize = 0;
	for (auto& curInfo: lbsInfos) {
		size_t oldSize = mergedGSource.size();
		sourceSize += curInfo->m_sources.size();
		mergedGSource.resize(sourceSize);
		std::copy(curInfo->m_sources.begin(), curInfo->m_sources.end(), mergedGSource.begin() + oldSize);
	}

	//1-������ŵĲο�վ������ѭ����ģʽ���
	std::vector<GeneralSource*> refSources;												/** @brief	�ο�����Դ	*/
	for (auto curSource : mergedGSource) {
		refSources.push_back(curSource);
	}

	std::vector<SensorData> sensorDatas;
	scene->m_sensorDataLibrary.GetAllSensorData(sensorDatas);
	std::vector<GSPairCluster> clusters;
	for (auto& refSource : refSources) {
		refSource->m_sensorData = sensorDatas.front();						//�ο�Դ�������ݸ�ֵ
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
