#include "result.h"

Result::Result()
	: m_txNum(0)
	, m_rxNum(0)
	, m_sensorNum(0)
{

}

Result::~Result()
{
	m_raytracingResult.clear();
	std::vector<RaytracingResult>().swap(m_raytracingResult);

	m_lbsGSResult.clear();
	std::vector<LBSResultGS>().swap(m_lbsGSResult);

	m_sensorDataSPSTMD.clear();
	std::vector<SensorDataCollection>().swap(m_sensorDataSPSTMD);

	m_sensorDataMPSTSD.clear();
	std::vector<SensorDataCollection>().swap(m_sensorDataMPSTSD);

	m_sensorDataSPMTMD.clear();
	std::vector<SensorDataCollection>().swap(m_sensorDataSPMTMD);

	m_sensorDataMPMTMD.clear();
	std::vector<SensorDataCollection>().swap(m_sensorDataMPMTMD);

	for (auto& source : m_allGeneralSource) {
		delete source;
		source = nullptr;
	}
	m_allGeneralSource.clear();
	std::vector<GeneralSource*>().swap(m_allGeneralSource);
}

void Result::Init(const std::vector<Transmitter*>& transmitters, const std::vector<Receiver*> receivers)
{
	unsigned txNum = static_cast<unsigned>(transmitters.size());
	unsigned rxNum = static_cast<unsigned>(receivers.size());
	unsigned totalNum = static_cast<unsigned>(txNum * rxNum);
	m_raytracingResult.resize(totalNum);
	for (unsigned i = 0; i < txNum; ++i) {
		for (unsigned j = 0; j < rxNum; ++j) {
			m_raytracingResult[i * rxNum + j].m_transmitter = transmitters[i];
			m_raytracingResult[i * rxNum + j].m_receiver = receivers[j];
		}
	}
	m_txNum = txNum;
	m_rxNum = rxNum;
	//���ö�Ӧ�Ĵ�����������������������������������ݵ�ͬʱ�����Ӧ�Ĵ���������
	m_sensorCollectionConfig.m_sensorConfigs.resize(m_rxNum);
	for (unsigned i = 0; i < m_rxNum; ++i) {
		m_sensorCollectionConfig.m_sensorConfigs[i].m_id = i;
		m_sensorCollectionConfig.m_sensorConfigs[i].m_position = receivers[i]->m_position;
	}

	return;
}

void Result::Init(const std::vector<Sensor*>& sensors, const std::vector<Receiver*>& receivers)
{
	unsigned sensorNum = static_cast<unsigned>(sensors.size());
	unsigned receiverNum = static_cast<unsigned>(receivers.size());
	unsigned totalNum = sensorNum * receiverNum;
	m_rxNum = receiverNum;
	m_sensorNum = sensorNum;
}

void Result::Init(const std::vector<Sensor*>& sensors)
{
	m_lbsGSResult.resize(sensors.size());
	for (int i = 0; i < sensors.size(); ++i) {
		m_lbsGSResult[i].m_sensor = sensors[i];
	}
	m_sensorNum = static_cast<unsigned>(sensors.size());
}

void Result::OutputResult(SYSTEM_MODE systemMode, const OutputConfig& config) const
{
	if (systemMode == MODE_RT) {
		m_directory = config.m_rtDirectory;
		if (m_raytracingResult.empty())
			return;
		if (config.m_outputMagnitude) {
			OutputVectorEField();
			OutputScalarEField();
		}
		if (config.m_outputPower) {
			OutputVectorPower();
			OutputScalarPower();
		}
		if (config.m_outputLoss) {
			OutputLoss();
		}
		if (config.m_outputMultipath) {
			OutputRayPath();
		}
		if (config.m_outputPDP) {
			OutputPDP();
		}
		if (config.m_outputCFR) {
			OutputCFR();
		}
		if (config.m_outputCIR) {
			OutputCIR();
		}
		if (config.m_outputAOA) {
			OutputAOA();
		}
		if (config.m_outputAOD) {
			OutputAOD();
		}
		if (config.m_outputSensorDataSPSTMD) {
			OutputSensorDataSPSTMD();
		}
		if (config.m_outputSensorDataMPSTSD) {
			OutputSensorDataMPSTSD();
		}
		if (config.m_outputSensorDataSPMTMD) {
			OutputSensorDataSPMTMD();
		}
		if (config.m_outputSensorDataMPMTMD) {
			OutputSensorDataMPMTMD();
		}
		if (config.m_outputGSForCRLB) {
			OutputGeneralSourceForCRLB();
		}
	}
	else if (systemMode == MODE_LBS) {
		m_directory = config.m_lbsDirectory;
		OutputGeneralSource();
	}
	

	return;
}

void Result::CalculateResult_RT_SensorData(const OutputConfig& outputConfig)
{
	//��������׷�ٽ��
	std::vector<RtLbsType> freqs = _global_freqConfig.GetFrequencyInformation();
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		RaytracingResult& result = *it;
		result.CalculateBaseInfo(freqs);
	}

	RtLbsType sensorDataSparseFactor = outputConfig.m_outputSensorDataSparseFactor;					/** @brief	������������ݵ�ϡ���	*/
	//���㴫�������ݽ��,Ĭ�϶�λΪ��Ƶ���µĶ�λģʽ
	if (outputConfig.m_outputSensorDataSPSTMD) {				//��Ϊ��վ��Դ�����ݶ�λ�����������Ϊ��������ջ�Ϊ1
		m_sensorDataSPSTMD.resize(m_txNum);
		if (outputConfig.m_outputLBSMethod == LBS_METHOD_RT_AOA || outputConfig.m_outputLBSMethod == LBS_METHOD_RT_AOA_TDOA) {
			RtLbsType threshold = m_raytracingResult[0].m_receiver->m_angularThreshold;
			for (unsigned i = 0; i < m_txNum; ++i) {
				m_raytracingResult[i].GetAllSensorData_AOA2D(m_sensorDataSPSTMD[i], threshold, sensorDataSparseFactor);
				m_sensorDataSPSTMD[i].m_sensorId = 0;				//���㴫����ID
				m_sensorDataSPSTMD[i].CalculateTimeDiff();			//����ʱ�Ӳ�ֵ
			}
		}
		else if (outputConfig.m_outputLBSMethod == LBS_METHOD_RT_TDOA) {
			RtLbsType threshold = m_raytracingResult[0].m_receiver->m_delayThreshold;
			for (unsigned i = 0; i < m_txNum; ++i) {
				m_raytracingResult[i].GetAllSensorData_Delay(m_sensorDataSPSTMD[i], threshold, sensorDataSparseFactor);
				m_sensorDataSPSTMD[i].m_sensorId = 0;				//���㴫����ID
				m_sensorDataSPSTMD[i].CalculateTimeDiff();			//����ʱ�Ӳ�ֵ
			}
		}
	}
	if (outputConfig.m_outputSensorDataMPSTSD) {				//��Ϊ��վ��Դ�����ݶ�λ�����������Ϊ��������ջ�Ϊ���
		//����ÿ������������ÿ���������߼�Ĵ��������ݣ����ֵ��
		m_sensorDataMPSTSD.resize(m_txNum * m_rxNum);
		for (unsigned i = 0; i < m_txNum; ++i) {
			for (unsigned j = 0; j < m_rxNum; ++j) {
				RtLbsType threshold = m_raytracingResult[i * m_rxNum + j].m_receiver->m_angularThreshold;
				m_raytracingResult[i * m_rxNum + j].GetMaxPowerSensorData_AOA2D(m_sensorDataMPSTSD[i * m_rxNum + j], threshold);
				m_sensorDataMPSTSD[i * m_rxNum + j].m_sensorId = j;		//���㴫����ID����ID��Ϊ���ջ���ID
			}
		}

		//����ʱ�Ӳ�ֵ
		for (unsigned i = 0; i < m_txNum; ++i) {
			RtLbsType firstTimeDelay = m_sensorDataMPSTSD[i * m_rxNum].m_data[0].m_time;
			for (unsigned j = 1; j < m_rxNum; ++j) {
				m_sensorDataMPSTSD[i * m_rxNum + j].m_data[0].m_timeDiff = m_sensorDataMPSTSD[i * m_rxNum + j].m_data[0].m_time - firstTimeDelay;
			}
		}
	}
	if (outputConfig.m_outputSensorDataSPMTMD) {				//��Ϊ��վ��Դ�����ݶ�λ�����������Ϊ��������ջ�Ϊ1��
		m_sensorDataSPMTMD.resize(1);
		//��ȡ������������뵥����������֮������ݣ������кϲ�����
		RtLbsType threshold = m_raytracingResult[0].m_receiver->m_angularThreshold;
		for (unsigned i = 0; i < m_txNum; ++i) {
			SensorDataCollection curCollection;
			m_raytracingResult[i].GetAllSensorData_AOA2D(curCollection, threshold, sensorDataSparseFactor);
			for (auto it = curCollection.m_data.begin(); it != curCollection.m_data.end(); ++it) {
				m_sensorDataSPMTMD[0].m_data.push_back(*it);
			}
		}
		m_sensorDataSPMTMD[0].m_sensorId = 0;
	}
	if (outputConfig.m_outputSensorDataMPMTMD) {				//��Ϊ��վ��Դ�����ݶ�λ�������Ϊ��������ջ�ҲΪ���
		m_sensorDataMPMTMD.resize(m_rxNum);
		for (unsigned j = 0; j < m_rxNum; ++j) {
			for (unsigned i = 0; i < m_txNum; ++i) {
				unsigned dataId = i * m_rxNum + j;
				RtLbsType threshold = m_raytracingResult[dataId].m_receiver->m_angularThreshold;
				SensorDataCollection curCollection;
				m_raytracingResult[dataId].GetAllSensorData_AOA2D(curCollection, threshold, sensorDataSparseFactor);
				for (auto it = curCollection.m_data.begin(); it != curCollection.m_data.end(); ++it) {
					m_sensorDataMPMTMD[j].m_data.push_back(*it);
				}
			}
			m_sensorDataMPMTMD[j].m_sensorId = j;				//���㴫����ID
		}
	}
	
}

Point2D Result::CalculateResult_LBS_AOA_MPSTSD(HARDWAREMODE hardwareMode, const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, const WeightFactor& weightFactor)
{
	//0-���������Ϣ-�������Դ��λ�� 
	for (auto it = m_lbsGSResult.begin(); it != m_lbsGSResult.end(); ++it) {
		LBSResultGS& curResult = *it;
		curResult.CalculateBaseInfo(method);
	}

	//1-�ϲ���λ���-������������Ĺ���Դ���кϲ�
	size_t sourceSize = 0;
	for (int i = 0; i < m_lbsGSResult.size(); ++i) {
		size_t oldSize = m_allGeneralSource.size();
		sourceSize += m_lbsGSResult[i].m_sources.size();
		m_allGeneralSource.resize(sourceSize);
		std::copy(m_lbsGSResult[i].m_sources.begin(), m_lbsGSResult[i].m_sources.end(), m_allGeneralSource.begin() + oldSize);
	}

	//2-���Ȩ�ؾ���

	//2-���ռ���Լ������ɾ����Ч����Դ
	//��������Դ��,����Ϊ n*(n-1)/2
	size_t pairNum = static_cast<int>(sourceSize * (sourceSize - 1) / 2);
	std::vector<GSPair*> gsPairs(pairNum);										/** @brief	����Դ��	*/
	size_t pairId = 0;																/** @brief	����Դ��ID	*/
	for (size_t i = 0; i < sourceSize; ++i) {
		for (size_t j = i + 1; j < sourceSize; ++j) {
			gsPairs[pairId] = new GSPair(m_allGeneralSource[i], m_allGeneralSource[j]);
			if (!gsPairs[pairId]->HasValidAOASolution(scene)) {					//������Դ����Ч����ɾ���ù���Դ�ԣ�����ֹͣ����
				delete gsPairs[pairId];
				continue;
			}
			pairId++;															//����Դ����Ч����������
		}
	}
	pairNum = pairId;
	gsPairs.resize(pairNum);
	gsPairs.shrink_to_fit();

	
	//ɾ��Ȩ��Ϊ0ֵ�Ĺ���Դ-���ͷ��ڴ棬���������ɾ��
	for (auto it = m_allGeneralSource.begin(); it != m_allGeneralSource.end(); ++it) {
		GeneralSource*& curGS = *it;
		if (!curGS->IsValid()) {
			delete curGS;
			curGS = nullptr;
		}
	}
	m_allGeneralSource.erase(std::remove_if(m_allGeneralSource.begin(), m_allGeneralSource.end(), [](const GeneralSource* source) {		//�Ƴ���Ч�Ĺ���Դ
		return source == nullptr;
		}), m_allGeneralSource.end());

	//�����й���ԴȨ����0
	for (int i = 0; i < m_allGeneralSource.size(); ++i) {
		m_allGeneralSource[i]->m_wCount = 0;					//Ȩ�ؼ�������
		m_allGeneralSource[i]->m_weight = 0;					//Ȩ�ع���
	}

	//2-2 ȥ���ظ��Ĺ���Դ
	EraseRepeatGeneralSources(m_allGeneralSource);
	sourceSize = m_allGeneralSource.size();
	//���¹�������Դ��
	for (auto it = gsPairs.begin(); it != gsPairs.end(); ++it) {
		GSPair* curPair = *it;
		delete curPair;
	}
	gsPairs.clear();
	pairNum = static_cast<int>(sourceSize * (sourceSize - 1) / 2);
	gsPairs.resize(pairNum);
	pairId = 0;
	for (size_t i = 0; i < sourceSize; ++i) {
		for (size_t j = i + 1; j < sourceSize; ++j) {
			gsPairs[pairId] = new GSPair(m_allGeneralSource[i], m_allGeneralSource[j]);
			if (!gsPairs[pairId]->HasValidAOASolution(scene)) {					//������Դ����Ч����ɾ���ù���Դ�ԣ�����ֹͣ����
				delete gsPairs[pairId];
				continue;
			}
			pairId++;															//����Դ����Ч����������
		}
	}
	pairNum = pairId;
	gsPairs.resize(pairNum);
	gsPairs.shrink_to_fit();

	//�Գ������˺��gspair���о���
	int max_cluster_num = 0;
	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(gsPairs, scene, gsPairClusterThreshold, max_cluster_num);


	//2-2  ��������Լ����������Ȩ�ز�ɾ��������Ȩ����ֵ�Ĺ���Դ
	// ��������׷���㷨�������¼������Դ�����Դ֮����ܵĽ��ֵ������ֵ���������ƶ���ֵ��������
	
	std::vector<RtLbsType> freqs = _global_freqConfig.GetFrequencyInformation();
	const MaterialLibrary* matLibrary = &scene->m_materialLibrary;
	const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	int sensorNum = static_cast<int>(scene->m_sensors.size());																	/** @brief	����������	*/
	std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	ʵʱ����Ŀ��Դ�����������	*/
	std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	ԭʼ�Ĵ���������	*/
	scene->m_sensorDataLibrary.GetAllSensorDataCollection(originalSensorDataCollection);										//��ȡԭʼ����������
	std::sort(originalSensorDataCollection.begin(), originalSensorDataCollection.end(), ComparedByPower_SensorDataCollection);	//���չ��ʶԴ��������ݽ�������
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

	RtLbsType mean_r_phi = 0.0;																									/** @brief	�Ƕ�ƽ���в�	*/
	RtLbsType mean_r_powerDiff = 0.0;																							/** @brief	���ʲ�ƽ���в�	*/
	RtLbsType max_r_phi = 0.0;																									/** @brief	�Ƕ����в�	*/
	RtLbsType max_r_powerDiff = 0.0;																							/** @brief	�������в�	*/

	for (auto& curCluster : gsPairClusters) {
		RtLbsType curCluster_min_r_phi = FLT_MAX;																				/** @brief	���ǶȲв�	*/
		RtLbsType curCluster_min_r_powerDiff = FLT_MAX;																			/** @brief	����ʲ�в�	*/
		int curCluster_min_nullDataNum = INT_MAX;																				/** @brief	������������	*/

		for (auto& curResult : curCluster.m_rtResult) {																			//Ѱ��cluster�ж����result�еĲв���Сֵ
			targetSensorDataCollection.clear();
			targetSensorDataCollection.resize(sensorNum);
			for (int i = 0; i < sensorNum; ++i) {
				const Sensor* curSensor = scene->m_sensors[i];
				curResult[i].CalculateBaseInfo(curSensor, freqs, antLibrary);
				curResult[i].GetMaxPowerSensorData_AOA2D(targetSensorDataCollection[i], curSensor->m_phiErrorSTD);
			}
			RtLbsType cur_r_phi = 0.0;
			RtLbsType cur_r_powerDiff = 0.0;
			int cur_nullDataNum = 0;
			CalculateSensorCollectionResidual_AOA_SingleData(originalSensorDataCollection, targetSensorDataCollection, cur_r_phi, cur_r_powerDiff, cur_nullDataNum);		//����в��������
			curCluster_min_r_phi = std::min(curCluster_min_r_phi, cur_r_phi);
			curCluster_min_r_powerDiff = std::min(curCluster_min_r_powerDiff, cur_r_powerDiff);
			curCluster_min_nullDataNum = std::min(curCluster_min_nullDataNum, cur_nullDataNum);
		}

		curCluster.SetElementAOAResidual(curCluster_min_r_phi, curCluster_min_r_powerDiff, curCluster_min_nullDataNum);

		mean_r_phi += curCluster_min_r_phi;
		mean_r_powerDiff += curCluster_min_r_powerDiff;
	}
	mean_r_phi /= gsPairClusters.size();
	mean_r_powerDiff /= gsPairClusters.size();

	for (auto& curPair : gsPairs) {
		curPair->m_phiResidual += mean_r_phi * curPair->m_nullDataNum;
		curPair->m_powerDiffResidual += mean_r_powerDiff * curPair->m_nullDataNum;
		max_r_phi = std::max(max_r_phi, curPair->m_phiResidual);
		max_r_powerDiff = std::max(max_r_powerDiff, curPair->m_powerDiffResidual);
	}

	//ѭ��pair�����һ���в�ϵ��,����ϵ�������Ӧ�Ĺ���ԴȨ�ؾ�����
	for (auto& curPair : gsPairs) {
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_AOA(max_r_phi, max_r_powerDiff, weightFactor, max_cluster_num);
		}
	}

	//�������й���Դ��Ȩ��������ֵ������Ȩ�ع�һ��
	RtLbsType max_weight = 0.0;
	for (auto it = m_allGeneralSource.begin(); it != m_allGeneralSource.end(); ++it) {
		GeneralSource* curSource = *it;
		if (max_weight < curSource->m_weight) {
			max_weight = curSource->m_weight;
		}
	}

	//����ԴȨ�ع�һ��, ��ɾ��δ�ﵽ��ֵȨ�صĹ���Դ
	for (auto it = m_allGeneralSource.begin(); it != m_allGeneralSource.end(); ++it) {
		GeneralSource*& curSource = *it;
		curSource->NormalizedWeight(max_weight);
		if (curSource->m_weight < 0.5) {
			delete curSource;
			curSource = nullptr;
		}
	}

	//ɾ����Ч�Ĺ���Դ
	m_allGeneralSource.erase(std::remove_if(m_allGeneralSource.begin(), m_allGeneralSource.end(), [](const GeneralSource* source) {
		return source == nullptr;
		}), m_allGeneralSource.end());

	//Pair �е�Ȩ�ع�һ������ɾ��δ�ﵽ��ֵȨ�ص�pair
	for (auto it = gsPairs.begin(); it != gsPairs.end(); ++it) {
		GSPair* curPair = *it;
		curPair->NormalizedWeight(max_weight);
		if (curPair->m_weight < 0.5) {
			delete curPair;
			curPair = nullptr;
		}
	}

	//ɾ����Ч��pair
	gsPairs.erase(std::remove_if(gsPairs.begin(), gsPairs.end(), [](const GSPair* pair) {
		return pair == nullptr;
		}), gsPairs.end());

	Point2D initPoint = { 0,0 };

	return LocalizationSolver(initPoint, m_allGeneralSource);
}

Point2D Result::CalculateResult_LBS_AOA_SPSTMD(HARDWAREMODE hardwareMode, const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, const WeightFactor& weightFactor)
{
	//0-���������Ϣ-�������Դ��λ�� 
	for (auto it = m_lbsGSResult.begin(); it != m_lbsGSResult.end(); ++it) {
		LBSResultGS& curResult = *it;
		curResult.CalculateBaseInfo(method);
	}

	//1-�ϲ���λ���-������������Ĺ���Դ���кϲ�
	size_t sourceSize = 0;
	for (int i = 0; i < m_lbsGSResult.size(); ++i) {
		size_t oldSize = m_allGeneralSource.size();
		sourceSize += m_lbsGSResult[i].m_sources.size();
		m_allGeneralSource.resize(sourceSize);
		std::copy(m_lbsGSResult[i].m_sources.begin(), m_lbsGSResult[i].m_sources.end(), m_allGeneralSource.begin() + oldSize);
	}

	std::vector<GeneralSource*> allGSCopy = m_allGeneralSource;						/** @brief	���й���Դ�ĸ���	*/

	//2-���Ȩ�ؾ���

	//2-���ռ���Լ������ɾ����Ч����Դ
	//��������Դ��,����Ϊ n*(n-1)/2
	size_t pairNum = static_cast<int>(sourceSize * (sourceSize - 1) / 2);
	std::vector<GSPair*> gsPairs;													/** @brief	����Դ��	*/
	gsPairs.reserve(pairNum);
	

	size_t pairId = 0;																/** @brief	����Դ��ID	*/
	#pragma omp parallel for num_threads(10)
	for (size_t i = 0; i < sourceSize; ++i) {
		for (size_t j = i + 1; j < sourceSize; ++j) {
			GSPair* newPair = new GSPair(allGSCopy[i], allGSCopy[j]);
			if (!newPair->HasValidAOASolution(scene)) {					//������Դ����Ч����ɾ���ù���Դ�ԣ�����ֹͣ����
				delete newPair;
				continue;
			}
			gsPairs.push_back(newPair);
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
	int max_cluster_num0 = 0;
	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(gsPairs, scene, gsPairClusterThreshold, max_cluster_num0);

	int clusterNum = static_cast<int>(gsPairClusters.size());																	//������

	int max_cluster_num = 0;
	//��ȡ����Դ���ظ�Ԫ��
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


	std::vector<RtLbsType> freqs = _global_freqConfig.GetFrequencyInformation();
	const MaterialLibrary* matLibrary = &scene->m_materialLibrary;
	const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	int sensorNum = static_cast<int>(scene->m_sensors.size());																	/** @brief	����������	*/
	std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	ʵʱ����Ŀ��Դ�����������	*/
	std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	ԭʼ�Ĵ���������	*/
	scene->m_sensorDataLibrary.GetAllSeneorDataCollectionWithAOAError(originalSensorDataCollection);										//��ȡԭʼ����������
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

		RtLbsType curCluster_min_r_phi = FLT_MAX;
		RtLbsType curCluster_min_r_powerDiff = FLT_MAX;
		int curCluster_min_nullDataNum = INT_MAX;

		for (auto& curResult : curCluster.m_rtResult) {																		//�����������еĽ⣨λ�Ƶ��µĶ�⣩

			RtLbsType cur_r_phi = 0.0;																								/** @brief	��ǰ�ĽǶȲв�	*/
			RtLbsType cur_r_powerDiff = 0.0;																						/** @brief	��ǰ�Ĺ��ʲ�в�	*/
			int cur_nullDataNum = 0;																								/** @brief	������������ƥ����	*/
			targetSensorDataCollection.clear();
			targetSensorDataCollection.resize(sensorNum);
			for (int j = 0; j < sensorNum; ++j) {
				const Sensor* curSensor = scene->m_sensors[j];
				curResult[j].CalculateBaseInfo(curSensor, freqs, antLibrary);						//ִ�е�ż���,LBS��ż���
				curResult[j].GetAllSensorData_AOA2D(targetSensorDataCollection[j], curSensor->m_phiErrorSTD * 2.0, 1.0);			//�ռ�ʵʱ����Ĵ��������,�����Ƿ���,��˲�����ϡ�裬2.0�����ǡ�
				CalculateSensorCollectionResidual_AOA_MultiData(originalSensorDataCollection, targetSensorDataCollection, weightFactor, cur_r_phi, cur_r_powerDiff, cur_nullDataNum);		//����в��������

				curCluster_min_r_phi = std::min(curCluster_min_r_phi, cur_r_phi);
				curCluster_min_r_powerDiff = std::min(curCluster_min_r_powerDiff, cur_r_powerDiff);
				curCluster_min_nullDataNum = std::min(curCluster_min_nullDataNum, cur_nullDataNum);
			}
			if ((curCluster.m_point - Point2D(74, 96)).Length() < 10.0) {
				if (cur_r_phi > 3.0) {
					std::cout << "find it" << std::endl;
				}
			}
		}

		curCluster.SetElementAOAResidual(curCluster_min_r_phi, curCluster_min_r_powerDiff, curCluster_min_nullDataNum);
		mean_r_phi += curCluster_min_r_phi;
		mean_r_powerDiff += curCluster_min_r_powerDiff;
	}


	mean_r_phi /= clusterNum;
	mean_r_powerDiff /= clusterNum;



	#pragma omp parallel for num_threads(10)
	for (auto curPair : gsPairs) {
		curPair->m_phiResidual += curPair->m_nullDataNum * mean_r_phi;
		curPair->m_powerDiffResidual += curPair->m_nullDataNum * mean_r_powerDiff;
		max_r_phi = std::max(max_r_phi, curPair->m_phiResidual);
		max_r_powerDiff = std::max(max_r_powerDiff, curPair->m_powerDiffResidual);
	}


	//ѭ��pair�����һ���в�ϵ��,����ϵ�������Ӧ�Ĺ���ԴȨ�ؾ�����
	for (auto it = gsPairs.begin(); it != gsPairs.end(); ++it) {
		GSPair* curPair = *it;
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_AOA(max_r_phi, max_r_powerDiff, weightFactor,  max_cluster_num);
		}
	}

	//ɾ���ظ��Ĺ���Դ
	EraseRepeatGeneralSources(allGSCopy);			//ɾ���ظ��Ĺ���Դ

	//�������й���Դ��Ȩ��������ֵ������Ȩ�ع�һ��
	RtLbsType max_weight = 0.0;
	for (auto it = allGSCopy.begin(); it != allGSCopy.end(); ++it) {
		GeneralSource* curSource = *it;
		if (max_weight < curSource->m_weight) {
			max_weight = curSource->m_weight;
		}
	}

	//����ԴȨ�ع�һ��, ��ɾ��δ�ﵽ��ֵȨ�صĹ���Դ
	for (auto it = allGSCopy.begin(); it != allGSCopy.end(); ++it) {
		GeneralSource*& curSource = *it;
		curSource->NormalizedWeight(max_weight);
	}

	//ɾ������Ȩ����ֵ�Ĺ���Դ
	allGSCopy.erase(std::remove_if(allGSCopy.begin(), allGSCopy.end(), [](const GeneralSource* s) {
		return s->m_weight < 0.6;
		}), allGSCopy.end());

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

	

	//������Դ���ս��չ��ʵĴ�С��������
	std::sort(allGSCopy.begin(), allGSCopy.end(), ComparedByPower_GeneralSource);

	Point2D targetPoint;
	targetPoint = LocalizationSolver(initPoint, allGSCopy);

	gsPairClusters.clear();
	std::vector<GSPairCluster>().swap(gsPairClusters);

	for (auto& pair : gsPairs) {
		delete pair;
		pair = nullptr;
	}
	gsPairs.clear();
	std::vector<GSPair*>().swap(gsPairs);

	//������ɺ�ɾ�����й���Դ
	for (auto& source : m_allGeneralSource) {
		delete source;
		source = nullptr;
	}
	m_allGeneralSource.clear();
	std::vector<GeneralSource*>().swap(m_allGeneralSource);

	return targetPoint;
}

void Result::CalculateResult_LBS_TDOA_MPSTSD(HARDWAREMODE hardwareMode, const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, const WeightFactor& w)
{
	//0-���������Ϣ-�������Դ��λ��
	for (auto it = m_lbsGSResult.begin(); it != m_lbsGSResult.end(); ++it) {
		LBSResultGS& curResult = *it;
		curResult.CalculateBaseInfo(method);
	}

	

	//������ݣ�����ÿ���������������Ĺ���Դ���趨�ο�����Դ������γɷ����Է���
	int sensorNum = static_cast<int>(m_lbsGSResult.size());
	//��ÿ������Դ��ֵ��Ӧ�Ĵ���������
	std::vector<SensorData> sensorDatas;
	scene->m_sensorDataLibrary.GetAllSensorData(sensorDatas);
	for (int i = 0; i < sensorNum; ++i) {
		m_lbsGSResult[i].SetSensorData(sensorDatas[i]);
	}
	std::vector<GeneralSource*> refSources = m_lbsGSResult[0].m_sources;												/** @brief	�ο�����Դ, Ĭ�ϵ�0������������Ϊ�ο����ݴ�����	*/
	std::vector<GeneralSource*> dataSources;																			/** @brief	���ݹ���Դ	*/
	int dataSourceSize = 0;																								/** @brief	���ݹ���Դ������	*/
	for (int i = 1; i < sensorNum; ++i) {
		size_t oldSize = dataSources.size();
		dataSourceSize = static_cast<int>(oldSize + m_lbsGSResult[i].m_sources.size());
		dataSources.resize(dataSourceSize);
		std::copy(m_lbsGSResult[i].m_sources.begin(), m_lbsGSResult[i].m_sources.end(), dataSources.begin() + oldSize);
	}

	std::vector<GSPairCluster> gsPairClusters;

	for (auto& refSource : refSources) {
		GSPairCluster newCluster = CalMaxTDOASolutionGSPairCluster_MPSTSD(refSource, dataSources, scene, gsPairClusterThreshold, _global_freqConfig.m_centerFrequency);
		if (newCluster.m_pairs.size() != 0) {
			gsPairClusters.push_back(newCluster);
		}
	}


	//��������׷���㷨����Ŀ��⵽����������֮��Ĳв�
	int clusterNum = static_cast<int>(gsPairClusters.size());
	std::vector<std::vector<RaytracingResult>> tempRTResult;									//��һά��Ϊ��������������ڶ�ά��Ϊ����������������������׷�٣��ж���Դ���������Ч��
	std::vector<RtLbsType> freqs = _global_freqConfig.GetFrequencyInformation();
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
				curResult[i].CalculateBaseInfo(curSensor, freqs, antLibrary);
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
		curCluster.CalNormalizedTDOAWeight(max_r_timeDiff, max_r_powerDiff, w);
		maxWeight = std::max(maxWeight, curCluster.m_weight);
	}

	//�����һ��Ȩ��
	for (auto& curCluster : gsPairClusters) {
		curCluster.m_weight /= maxWeight;
	}

	std::sort(gsPairClusters.begin(), gsPairClusters.end(), ComparedByClusterWeight);



	Point2D targetPoint = gsPairClusters.front().m_point;

	std::cout << targetPoint.x << "," << targetPoint.y << std::endl;

}

void Result::CalculateResult_LBS_TDOA_SPSTMD(HARDWAREMODE hardwareMode, const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, const WeightFactor& weightFactor)
{
	//0-���������Ϣ-�������Դ��λ�� 
	for (auto it = m_lbsGSResult.begin(); it != m_lbsGSResult.end(); ++it) {
		LBSResultGS& curResult = *it;
		curResult.CalculateBaseInfo(method);
	}

	//1-�ϲ���λ���-������������Ĺ���Դ���кϲ�
	size_t sourceSize = 0;
	for (int i = 0; i < m_lbsGSResult.size(); ++i) {
		size_t oldSize = m_allGeneralSource.size();
		sourceSize += m_lbsGSResult[i].m_sources.size();
		m_allGeneralSource.resize(sourceSize);
		std::copy(m_lbsGSResult[i].m_sources.begin(), m_lbsGSResult[i].m_sources.end(), m_allGeneralSource.begin() + oldSize);
	}

	//1-������ŵĲο�վ������ѭ����ģʽ���
	std::vector<GeneralSource*> refSources;												/** @brief	�ο�����Դ	*/
	for (auto curSource : m_allGeneralSource) {
		refSources.push_back(curSource);
	}

	std::vector<SensorData> sensorDatas;
	scene->m_sensorDataLibrary.GetAllSensorData(sensorDatas);
	std::vector<GSPairCluster> clusters;
	for (auto& refSource : refSources) {
		refSource->m_sensorData = sensorDatas.front();						//�ο�Դ�������ݸ�ֵ
		GSPairCluster newCluster = CalMaxTDOASolutionGSPairCluster_SPSTMD(refSource, m_allGeneralSource, sensorDatas, scene, gsPairClusterThreshold, _global_freqConfig.m_centerFrequency);
		if (newCluster.m_pairs.size() != 0) {
			clusters.push_back(newCluster);
		}
	}

	std::sort(clusters.begin(), clusters.end(), ComparedByClusterResidual);

	Point2D targetPoint = clusters.front().m_point;

	std::cout << targetPoint.x<<","<<targetPoint.y << std::endl;

}


std::vector<GeneralSource*> Result::GetGeneralSource() const
{
	return m_allGeneralSource;
}

Point2D Result::LocalizationSolver(const Point2D& initPoint, const std::vector<GeneralSource*>& gss)
{
	m_aoaSolver.SetGeneralSource(gss);
	//m_aoaSolver.Solving_LS();										//ʹ�ü�Ȩ��С���˷������
	return m_aoaSolver.Solving_WIRLS(20, 1e-6, initPoint);								//ʹ�õ�����Ȩ��С���˷������
}

void Result::OutputVectorEField() const
{
	std::ofstream stream(m_directory + "vectorfield.txt");
	if (stream.is_open()) {
		for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
			const RaytracingResult& result = *it;
			result.OutputVectorEField(stream);
		}
	}
	else {
		LOG_ERROR << "Result:open vector field file failed" << ENDL;
	}
	stream.close();
}

void Result::OutputScalarEField() const
{
	std::ofstream stream(m_directory + "scalarfield.txt");
	if (stream.is_open()) {
		for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
			const RaytracingResult& result = *it;
			result.OutputScalarEField(stream);
		}
	}
	else {
		LOG_ERROR << "Result:open scalar field file failed" << ENDL;
	}
	stream.close();
}

void Result::OutputVectorPower() const
{
	std::ofstream stream(m_directory + "vectorpower.txt");
	if (stream.is_open()) {
		for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
			const RaytracingResult& result = *it;
			result.OutputVectorPower(stream);
		}
	}
	else {
		LOG_ERROR << "Result:open vector power file failed" << ENDL;
	}
	stream.close();
}

void Result::OutputScalarPower() const
{
	std::ofstream stream(m_directory + "scalarpower.txt");
	if (stream.is_open()) {
		for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
			const RaytracingResult& result = *it;
			result.OutputScalarPower(stream);
		}
	}
	else {
		LOG_ERROR << "Result:open scalar power file failed" << ENDL;
	}
	stream.close();
}

void Result::OutputLoss() const
{
	std::ofstream stream(m_directory + "loss.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open loss file failed." << ENDL;
		return;
	}
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		const RaytracingResult& result = *it;
		result.OutputLoss(stream);
	}
	stream.close();
}

void Result::OutputRayPath() const
{
	std::ofstream stream(m_directory + "multipath.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open multipath file failed." << ENDL;
		return;
	}
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		const RaytracingResult& result = *it;
		result.OutputRayPath(stream);
	}
	stream.close();
}

void Result::OutputPDP() const
{
	std::ofstream stream(m_directory + "pdp.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open pdp file failed." << ENDL;
		return;
	}
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		const RaytracingResult& result = *it;
		result.OutputPDP(stream);
	}
	stream.close();
}

void Result::OutputCFR() const
{
	std::ofstream stream(m_directory + "cfr.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open cfr file failed." << ENDL;
		return;
	}
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		const RaytracingResult& result = *it;
		result.OutputCFR(stream);
	}
	stream.close();
}

void Result::OutputCIR() const
{
	std::ofstream stream(m_directory + "cir.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open cir file failed." << ENDL;
		return;
	}
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		const RaytracingResult& result = *it;
		result.OutputCIR(stream);
	}
	stream.close();
}

void Result::OutputAOA() const
{
	std::ofstream stream(m_directory + "aoa.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open aoa file failed." << ENDL;
		return;
	}
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		const RaytracingResult& result = *it;
		result.OutputAOA(stream);
	}
	stream.close();
}

void Result::OutputAOD() const
{
	std::ofstream stream(m_directory + "aod.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open aod file failed." << ENDL;
		return;
	}
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		const RaytracingResult& result = *it;
		result.OutputAOD(stream);
	}
	stream.close();
}

void Result::OutputSensorDataSPSTMD() const
{
	//�����վ��Դ�����ݶ�λ�����Ϊ�����ļ�
	std::string sensorDataDirectory = m_directory + "sensor data/SPSTMD/";
	for (unsigned i = 0; i < m_txNum; ++i) {
		std::stringstream ss;
		ss << sensorDataDirectory << "tx" << i << "_SPSTMD.json";
		m_sensorDataSPSTMD[i].Write2Json(ss.str());
	}
	//д���Ӧ�Ĵ����������ļ�
	std::stringstream ss1;
	ss1 << sensorDataDirectory << "SPSTMD_sensorconfig.json";
	m_sensorCollectionConfig.Write2Json(ss1.str());

}

void Result::OutputSensorDataMPSTSD() const
{
	//�����վ��Դ�����ݶ�λ�����Ϊ����ļ�
	std::string sensorDataDirectory = m_directory + "sensor data/MPSTSD/";
	for (unsigned j = 0; j < m_rxNum; ++j) {
		for (unsigned i = 0; i < m_txNum; ++i) {
			std::stringstream ss;
			ss << sensorDataDirectory << "tx" << i << "_" << "sensor_" << j << "_MPSTSD.json";
			m_sensorDataMPSTSD[i * m_rxNum + j].Write2Json(ss.str());
		}
	}
	//д���Ӧ�Ĵ����������ļ�
	std::stringstream ss1;
	ss1 << sensorDataDirectory << "MPSTSD_sensorconfig.json";
	m_sensorCollectionConfig.Write2Json(ss1.str());
}

void Result::OutputSensorDataSPMTMD() const
{
	//���Ϊ��վ��Դ�����ݶ�λ�����Ϊ�����ļ�
	std::string sensorDataDirectory = m_directory + "sensor data/SPMTMD/";
	std::stringstream ss;
	ss << sensorDataDirectory << "SPMTMD.json";
	m_sensorDataSPMTMD[0].Write2Json(ss.str());
	//д���Ӧ�Ĵ����������ļ�
	std::stringstream ss1;
	ss1 << sensorDataDirectory << "SPMTMD_sensorconfig.json";
	m_sensorCollectionConfig.Write2Json(ss1.str());
}

void Result::OutputSensorDataMPMTMD() const
{
	//���Ϊ��վ��Դ�����ݶ�λ�����Ϊ�����ļ�
	std::string sensorDataDirectory = m_directory + "sensor data/MPMTMD/";
	for (unsigned i = 0; i < m_rxNum; ++i) {
		std::stringstream ss;
		ss << sensorDataDirectory << "sensor" << i << "_MPMTMD.json";
		m_sensorDataMPMTMD[i].Write2Json(ss.str());
	}
	//д���Ӧ�Ĵ����������ļ�
	std::stringstream ss1;
	ss1 << sensorDataDirectory << "MPMTMD_sensorconfig.json";
	m_sensorCollectionConfig.Write2Json(ss1.str());
}

void Result::OutputGeneralSource() const
{
	std::ofstream stream(m_directory + "generalsource.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open general-source file failed." << ENDL;
		return;
	}
	for (auto it = m_allGeneralSource.begin(); it != m_allGeneralSource.end(); ++it) {
		const GeneralSource* source = *it;
		source->Output2File(stream);
	}
	stream.close();
}

void Result::OutputGeneralSourceForCRLB() const
{
	std::ofstream stream(m_directory + "gscrlb.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open gscrlb file failed." << ENDL;
		return;
	}
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		const RaytracingResult& result = *it;
		result.OutputGeneralSourceForCRLB(stream);
	}
	stream.close();
}
