#include "result.h"

Result::Result()
	: m_txNum(0)
	, m_rxNum(0)
	, m_sensorNum(0)
{

}

Result::~Result()
{
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
	for (int i = 0; i < m_rxNum; ++i) {
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
	}
	else if (systemMode == MODE_LBS) {
		m_directory = config.m_lbsDirectory;
		OutputGeneralSource();
	}
	

	return;
}

void Result::CalculateResult_RT_SensorData(const FrequencyConfig& freqConfig, MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction, const OutputConfig& outputConfig)
{
	//��������׷�ٽ��
	std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		RaytracingResult& result = *it;
		result.CalculateBaseInfo(freqs, matLibrary, tranFunction);
	}

	RtLbsType sensorDataSparseFactor = outputConfig.m_outputSensorDataSparseFactor;					/** @brief	������������ݵ�ϡ���	*/
	//���㴫�������ݽ��,Ĭ�϶�λΪ��Ƶ���µĶ�λģʽ
	if (outputConfig.m_outputSensorDataSPSTMD) {				//��Ϊ��վ��Դ�����ݶ�λ�����������Ϊ��������ջ�Ϊ1
		m_sensorDataSPSTMD.resize(m_txNum);
		RtLbsType threshold = m_raytracingResult[0].m_receiver->m_angularThreshold;
		for (int i = 0; i < m_txNum; ++i) {
			m_raytracingResult[i].GetAllSensorData_AOA2D(m_sensorDataSPSTMD[i], threshold, sensorDataSparseFactor);
			m_sensorDataSPSTMD[i].m_sensorId = 0;				//���㴫����ID
			m_sensorDataSPSTMD[i].CalculateTimeDiff();			//����ʱ�Ӳ�ֵ
		}
	}
	if (outputConfig.m_outputSensorDataMPSTSD) {				//��Ϊ��վ��Դ�����ݶ�λ�����������Ϊ��������ջ�Ϊ���
		//����ÿ������������ÿ���������߼�Ĵ��������ݣ����ֵ��
		m_sensorDataMPSTSD.resize(m_txNum * m_rxNum);
		for (int i = 0; i < m_txNum; ++i) {
			for (int j = 0; j < m_rxNum; ++j) {
				RtLbsType threshold = m_raytracingResult[i * m_rxNum + j].m_receiver->m_angularThreshold;
				m_raytracingResult[i * m_rxNum + j].GetMaxPowerSensorData_AOA2D(m_sensorDataMPSTSD[i * m_rxNum + j], threshold);
				m_sensorDataMPSTSD[i * m_rxNum + j].m_sensorId = j;		//���㴫����ID����ID��Ϊ���ջ���ID
			}
		}

		//����ʱ�Ӳ�ֵ
		for (int i = 0; i < m_txNum; ++i) {
			RtLbsType firstTimeDelay = m_sensorDataMPSTSD[i * m_rxNum].m_data[0].m_time;
			for (int j = 1; j < m_rxNum; ++j) {
				m_sensorDataMPSTSD[i * m_rxNum + j].m_data[0].m_timeDiff = m_sensorDataMPSTSD[i * m_rxNum + j].m_data[0].m_time - firstTimeDelay;
			}
		}
	}
	if (outputConfig.m_outputSensorDataSPMTMD) {				//��Ϊ��վ��Դ�����ݶ�λ�����������Ϊ��������ջ�Ϊ1��
		m_sensorDataSPMTMD.resize(1);
		//��ȡ������������뵥����������֮������ݣ������кϲ�����
		RtLbsType threshold = m_raytracingResult[0].m_receiver->m_angularThreshold;
		for (int i = 0; i < m_txNum; ++i) {
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
		for (int j = 0; j < m_rxNum; ++j) {
			for (int i = 0; i < m_txNum; ++i) {
				int dataId = i * m_rxNum + j;
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

void Result::CalculateResult_LBS_AOA_MPSTSD(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method,const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunction)
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
	std::vector<GSPair*> gsPair(pairNum);										/** @brief	����Դ��	*/
	size_t pairId = 0;																/** @brief	����Դ��ID	*/
	for (size_t i = 0; i < sourceSize; ++i) {
		for (size_t j = i + 1; j < sourceSize; ++j) {
			gsPair[pairId] = new GSPair(m_allGeneralSource[i], m_allGeneralSource[j]);
			if (!gsPair[pairId]->HasValidAOASolution(scene)) {					//������Դ����Ч����ɾ���ù���Դ�ԣ�����ֹͣ����
				delete gsPair[pairId];
				continue;
			}
			pairId++;															//����Դ����Ч����������
		}
	}
	pairNum = pairId;
	gsPair.resize(pairNum);
	gsPair.shrink_to_fit();

	
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
	for (auto it = gsPair.begin(); it != gsPair.end(); ++it) {
		GSPair* curPair = *it;
		delete curPair;
	}
	gsPair.clear();
	pairNum = static_cast<int>(sourceSize * (sourceSize - 1) / 2);
	gsPair.resize(pairNum);
	pairId = 0;
	for (size_t i = 0; i < sourceSize; ++i) {
		for (size_t j = i + 1; j < sourceSize; ++j) {
			gsPair[pairId] = new GSPair(m_allGeneralSource[i], m_allGeneralSource[j]);
			if (!gsPair[pairId]->HasValidAOASolution(scene)) {					//������Դ����Ч����ɾ���ù���Դ�ԣ�����ֹͣ����
				delete gsPair[pairId];
				continue;
			}
			pairId++;															//����Դ����Ч����������
		}
	}
	pairNum = pairId;
	gsPair.resize(pairNum);
	gsPair.shrink_to_fit();

	//2-2  ��������Լ����������Ȩ�ز�ɾ��������Ȩ����ֵ�Ĺ���Դ
	// ��������׷���㷨�������¼������Դ�����Դ֮����ܵĽ��ֵ������ֵ���������ƶ���ֵ��������
	
	std::vector<std::vector<RaytracingResult>> tempRTResult;									//��һά��Ϊ��������������ڶ�ά��Ϊ����������������������׷�٣��ж���Դ���������Ч��
	std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	const MaterialLibrary* matLibrary = &scene->m_materialLibrary;
	const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	tempRTResult.resize(pairNum);
	int sensorNum = static_cast<int>(scene->m_sensors.size());																	/** @brief	����������	*/
	std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	ʵʱ����Ŀ��Դ�����������	*/
	std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	ԭʼ�Ĵ���������	*/
	scene->m_sensorDataLibrary.GetAllSensorDataCollection(originalSensorDataCollection);										//��ȡԭʼ����������
	std::sort(originalSensorDataCollection.begin(), originalSensorDataCollection.end(), ComparedByPower_SensorDataCollection);	//���չ��ʶԴ��������ݽ�������
	//Ϊ��ͬ���Ƚϲв���ｫԭʼ���ݰ��չ��ʵĴ�С��������
	RtLbsType max_r_phi = 0.0, max_r_powerDiff = 0.0;																			//�Ƕ����в�������в�
	for (size_t i = 0; i < pairNum; ++i) {
		targetSensorDataCollection.clear();
		targetSensorDataCollection.resize(sensorNum);
		GSPair* curPair = gsPair[i];
		DirectlySetResultPath_CPUSingleThread(vroots, scene, splitRadius, curPair->m_targetSolution, &tempRTResult[i]);
		for (int j = 0; j< tempRTResult[i].size(); ++j) {
			const Sensor* curSensor = scene->m_sensors[j];
			tempRTResult[i][j].CalculateBaseInfo(curSensor, freqs, antLibrary, matLibrary, tranFunction);						//ִ�е�ż���,LBS��ż���
			tempRTResult[i][j].GetMaxPowerSensorData_AOA2D(targetSensorDataCollection[j], curSensor->m_phiErrorSTD);			//�ռ�ʵʱ����Ĵ��������
			if (targetSensorDataCollection[j].m_data.size() == 0) {
				curPair->m_isValid = false;
			}
		}
		RtLbsType cur_r_phi = 0.0;																								/** @brief	��ǰ�ĽǶȲв�	*/
		RtLbsType cur_r_powerDiff = 0.0;																						/** @brief	��ǰ�Ĺ��ʲ�в�	*/

		if (!curPair->m_isValid) {																								//����ǰ����Դ����Ч����������ǰpair�����Ȩ�ؼ���
			continue;
		}
		std::sort(targetSensorDataCollection.begin(), targetSensorDataCollection.end(), ComparedByPower_SensorDataCollection);	//���չ��ʶԴ��������ݽ�������
		CalculateSensorCollectionResidual_AOA_SingleData(originalSensorDataCollection, targetSensorDataCollection, cur_r_phi, cur_r_powerDiff);		//����в��������
		curPair->m_phiResidual = cur_r_phi;
		curPair->m_powerDiffResidual = cur_r_powerDiff;
		if (max_r_phi < cur_r_phi) {
			max_r_phi = cur_r_phi;
		}
		if (max_r_powerDiff < cur_r_powerDiff) {
			max_r_powerDiff = cur_r_powerDiff;
		}
	}

	
	//ѭ��pair�����һ���в�ϵ��,����ϵ�������Ӧ�Ĺ���ԴȨ�ؾ�����
	for (auto it = gsPair.begin(); it != gsPair.end(); ++it) {
		GSPair* curPair = *it;
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_AOA(max_r_phi, max_r_powerDiff);
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
	for (auto it = gsPair.begin(); it != gsPair.end(); ++it) {
		GSPair* curPair = *it;
		curPair->NormalizedWeight(max_weight);
		if (curPair->m_weight < 0.5) {
			delete curPair;
			curPair = nullptr;
		}
	}

	//ɾ����Ч��pair
	gsPair.erase(std::remove_if(gsPair.begin(), gsPair.end(), [](const GSPair* pair) {
		return pair == nullptr;
		}), gsPair.end());


	LocalizationSolver();
}

void Result::CalculateResult_LBS_AOA_SPSTMD(HARDWAREMODE hardwareMode, const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunction)
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

	//��GSPair�е����ݽ��о��࣬���վ���õ�cluster�����մ����ĵ�������ΪԤ�����꣬���ɴأ���ÿ��gsPair����clusterId
	std::vector<GSPairCluster> gsPairClusters = ClusterGSPairByDistance(gsPairs, gsPairClusterThreshold);						//���о�����࣬���ټ�����
	int clusterNum = static_cast<int>(gsPairClusters.size());																	//������

	//2-2  ��������Լ����������Ȩ�ز�ɾ��������Ȩ����ֵ�Ĺ���Դ
	// ��������׷���㷨�������¼������Դ�����Դ֮����ܵĽ��ֵ������ֵ���������ƶ���ֵ��������

	std::vector<std::vector<RaytracingResult>> tempRTResult;									//��һά��Ϊ��������������ڶ�ά��Ϊ����������������������׷�٣��ж���Դ���������Ч��
	std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	const MaterialLibrary* matLibrary = &scene->m_materialLibrary;
	const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	tempRTResult.resize(clusterNum);
	int sensorNum = static_cast<int>(scene->m_sensors.size());																	/** @brief	����������	*/
	std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	ʵʱ����Ŀ��Դ�����������	*/
	std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	ԭʼ�Ĵ���������	*/
	scene->m_sensorDataLibrary.GetAllSensorDataCollection(originalSensorDataCollection);										//��ȡԭʼ����������
	std::sort(originalSensorDataCollection.begin(), originalSensorDataCollection.end(), ComparedByPower_SensorDataCollection);	//���չ��ʶԴ��������ݽ�������
	//Ϊ��ͬ���Ƚϲв���ｫԭʼ���ݰ��չ��ʵĴ�С��������
	RtLbsType max_r_phi = 0.0, max_r_powerDiff = 0.0;																			//�Ƕ����в�������в�
	RtLbsType mean_r_phi = 0.0;																									/** @brief	�Ƕ�ƽ���в�	*/
	RtLbsType mean_r_powerDiff = 0.0;																							/** @brief	����ƽ���в�	*/

	if (hardwareMode == CPU_SINGLETHREAD) {
		for (size_t i = 0; i < clusterNum; ++i) {
			targetSensorDataCollection.clear();
			targetSensorDataCollection.resize(sensorNum);
			GSPairCluster& curCluster = gsPairClusters[i];																			/** @brief	��ǰ��������cluster	*/
			DirectlySetResultPath_CPUSingleThread(vroots, scene, splitRadius, curCluster.m_point, &tempRTResult[i]);
			for (int j = 0; j < tempRTResult[i].size(); ++j) {
				const Sensor* curSensor = scene->m_sensors[j];
				tempRTResult[i][j].CalculateBaseInfo(curSensor, freqs, antLibrary, matLibrary, tranFunction);						//ִ�е�ż���,LBS��ż���
				tempRTResult[i][j].GetAllSensorData_AOA2D(targetSensorDataCollection[j], curSensor->m_phiErrorSTD, 1.0);			//�ռ�ʵʱ����Ĵ��������,�����Ƿ��棬��˲�����ϡ��
				if (targetSensorDataCollection[j].m_data.size() == 0) {
					curCluster.m_isValid = false;
				}
			}
			RtLbsType cur_r_phi = 0.0;																								/** @brief	��ǰ�ĽǶȲв�	*/
			RtLbsType cur_r_powerDiff = 0.0;																						/** @brief	��ǰ�Ĺ��ʲ�в�	*/
			int cur_nullDataNum = 0;																								/** @brief	������������ƥ����	*/

			if (!curCluster.m_isValid) {																								//����ǰ����Դ����Ч����������ǰpair�����Ȩ�ؼ���
				continue;
			}
			CalculateSensorCollectionResidual_AOA_MultiData(originalSensorDataCollection, targetSensorDataCollection, cur_r_phi, cur_r_powerDiff, cur_nullDataNum);		//����в��������
			curCluster.SetElementAOAResidual(cur_r_phi, cur_r_powerDiff, cur_nullDataNum);
			mean_r_phi += cur_r_phi;
			mean_r_powerDiff += cur_r_powerDiff;

		}
	}
	else if (hardwareMode == CPU_MULTITHREAD) {
		//�����п��ܵĽ⹹������
		std::vector<Point2D> targetPoints;
		targetPoints.reserve(clusterNum);
		for (auto& curCluster : gsPairClusters) {
			targetPoints.push_back(curCluster.m_point);
		}
		DirectlySetResultPath_CPUMultiThread(vroots, scene, splitRadius, targetPoints, threadNum, tempRTResult);

		for (size_t i = 0; i < clusterNum; ++i) {
			targetSensorDataCollection.clear();
			targetSensorDataCollection.resize(sensorNum);
			GSPairCluster& curCluster = gsPairClusters[i];																			/** @brief	��ǰ��������cluster	*/
			for (int j = 0; j < clusterNum; ++j) {
				const Sensor* curSensor = scene->m_sensors[j];
				tempRTResult[i][j].CalculateBaseInfo(curSensor, freqs, antLibrary, matLibrary, tranFunction);						//ִ�е�ż���,LBS��ż���
				tempRTResult[i][j].GetAllSensorData_AOA2D(targetSensorDataCollection[j], curSensor->m_phiErrorSTD, 1.0);			//�ռ�ʵʱ����Ĵ��������,�����Ƿ���,��˲�����ϡ��
				if (targetSensorDataCollection[j].m_data.size() == 0) {
					curCluster.m_isValid = false;
				}
			}
			RtLbsType cur_r_phi = 0.0;																								/** @brief	��ǰ�ĽǶȲв�	*/
			RtLbsType cur_r_powerDiff = 0.0;																						/** @brief	��ǰ�Ĺ��ʲ�в�	*/
			int cur_nullDataNum = 0;																								/** @brief	������������ƥ����	*/

			if (!curCluster.m_isValid) {																								//����ǰ����Դ����Ч����������ǰpair�����Ȩ�ؼ���
				continue;
			}
			CalculateSensorCollectionResidual_AOA_MultiData(originalSensorDataCollection, targetSensorDataCollection, cur_r_phi, cur_r_powerDiff, cur_nullDataNum);		//����в��������
			curCluster.SetElementAOAResidual(cur_r_phi, cur_r_powerDiff, cur_nullDataNum);
			mean_r_phi += cur_r_phi;
			mean_r_powerDiff += cur_r_powerDiff;
		}
	}
	else if (hardwareMode == GPU_MULTITHREAD) {
		//�����п��ܵĽ⹹������
		std::vector<Point2D> targetPoints;
		targetPoints.reserve(clusterNum);
		for (auto& curCluster : gsPairClusters) {
			targetPoints.push_back(curCluster.m_point);
		}

		//����GPU���������м�������׷�ٽ��
		DirectlySetResultPath_GPUMultiThread(vroots, scene, splitRadius, targetPoints, tempRTResult);

		for (size_t i = 0; i < clusterNum; ++i) {
			targetSensorDataCollection.clear();
			targetSensorDataCollection.resize(sensorNum);
			GSPairCluster& curCluster = gsPairClusters[i];																			/** @brief	��ǰ��������cluster	*/
			for (int j = 0; j < tempRTResult[i].size(); ++j) {
				const Sensor* curSensor = scene->m_sensors[j];
				tempRTResult[i][j].CalculateBaseInfo(curSensor, freqs, antLibrary, matLibrary, tranFunction);						//ִ�е�ż���,LBS��ż���
				tempRTResult[i][j].GetAllSensorData_AOA2D(targetSensorDataCollection[j], curSensor->m_phiErrorSTD, 1.0);			//�ռ�ʵʱ����Ĵ��������,�����Ƿ���,��˲�����ϡ��
				if (targetSensorDataCollection[j].m_data.size() == 0) {
					curCluster.m_isValid = false;
				}
			}
			RtLbsType cur_r_phi = 0.0;																								/** @brief	��ǰ�ĽǶȲв�	*/
			RtLbsType cur_r_powerDiff = 0.0;																						/** @brief	��ǰ�Ĺ��ʲ�в�	*/
			int cur_nullDataNum = 0;																								/** @brief	������������ƥ����	*/

			if (!curCluster.m_isValid) {																								//����ǰ����Դ����Ч����������ǰpair�����Ȩ�ؼ���
				continue;
			}
			CalculateSensorCollectionResidual_AOA_MultiData(originalSensorDataCollection, targetSensorDataCollection, cur_r_phi, cur_r_powerDiff, cur_nullDataNum);		//����в��������
			curCluster.SetElementAOAResidual(cur_r_phi, cur_r_powerDiff, cur_nullDataNum);
			mean_r_phi += cur_r_phi;
			mean_r_powerDiff += cur_r_powerDiff;
		}

	}
	mean_r_phi /= clusterNum;
	mean_r_powerDiff /= clusterNum;

	for (auto curPair : gsPairs) {
		curPair->m_phiResidual += curPair->m_nullDataNum * mean_r_phi;
		curPair->m_powerDiffResidual += curPair->m_nullDataNum * mean_r_powerDiff;
		if (max_r_phi < curPair->m_phiResidual) {
			max_r_phi = curPair->m_phiResidual;
		}
		if (max_r_powerDiff < curPair->m_powerDiffResidual) {
			max_r_powerDiff = curPair->m_powerDiffResidual;
		}
	}


	//ѭ��pair�����һ���в�ϵ��,����ϵ�������Ӧ�Ĺ���ԴȨ�ؾ�����
	for (auto it = gsPairs.begin(); it != gsPairs.end(); ++it) {
		GSPair* curPair = *it;
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_AOA(max_r_phi, max_r_powerDiff);
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


	LocalizationSolver();
}

std::vector<GeneralSource*> Result::GetGeneralSource() const
{
	return m_allGeneralSource;
}

void Result::LocalizationSolver()
{
	google::InitGoogleLogging("rtlbs_ceres.log");
	m_aoaSolver.SetGeneralSource(m_allGeneralSource);
	//m_aoaSolver.Solving_LS();										//ʹ�ü�Ȩ��С���˷������
	m_aoaSolver.Solving_WIRLS(20, 1e-6);								//ʹ�õ�����Ȩ��С���˷������
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
	for (int i = 0; i < m_txNum; ++i) {
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
	for (int j = 0; j < m_rxNum; ++j) {
		for (int i = 0; i < m_txNum; ++i) {
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
	for (int i = 0; i < m_rxNum; ++i) {
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
