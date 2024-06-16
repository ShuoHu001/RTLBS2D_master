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
	//配置对应的传感器配置数量，用于输出传感器仿真数据的同时输出对应的传感器配置
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

void Result::CalculateResult(const FrequencyConfig& freqConfig, MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction, const OutputConfig& outputConfig)
{
	//计算射线追踪结果
	std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		RaytracingResult& result = *it;
		result.CalculateBaseInfo(freqs, matLibrary, tranFunction);
	}

	//计算传感器数据结果,默认定位为单频点下的定位模式
	if (outputConfig.m_outputSensorDataSPSTMD) {				//若为单站单源多数据定位，发射机可以为多个，接收机为1
		m_sensorDataSPSTMD.resize(m_txNum);
		for (int i = 0; i < m_txNum; ++i) {
			m_raytracingResult[i].GetAllSensorData(m_sensorDataSPSTMD[i]);
			m_sensorDataSPSTMD[i].m_sensorId = 0;				//计算传感器ID
			m_sensorDataSPSTMD[i].CalculateTimeDiff();			//计算时延差值
		}
	}
	if (outputConfig.m_outputSensorDataMPSTSD) {				//若为多站单源单数据定位，发射机可以为多个，接收机为多个
		//计算每个发射天线与每个接收天线间的传感器数据（最大值）
		m_sensorDataMPSTSD.resize(m_txNum * m_rxNum);
		for (int i = 0; i < m_txNum; ++i) {
			for (int j = 0; j < m_rxNum; ++j) {
				m_raytracingResult[i * m_rxNum + j].GetMaxPowerSensorData(m_sensorDataMPSTSD[i * m_rxNum + j]);
				m_sensorDataMPSTSD[i * m_rxNum + j].m_sensorId = j;		//计算传感器ID，该ID即为接收机的ID
			}
		}

		//计算时延差值
		for (int i = 0; i < m_txNum; ++i) {
			RtLbsType firstTimeDelay = m_sensorDataMPSTSD[i * m_rxNum].m_data[0].m_time;
			for (int j = 1; j < m_rxNum; ++j) {
				m_sensorDataMPSTSD[i * m_rxNum + j].m_data[0].m_timeDiff = m_sensorDataMPSTSD[i * m_rxNum + j].m_data[0].m_time - firstTimeDelay;
			}
		}
	}
	if (outputConfig.m_outputSensorDataSPMTMD) {				//若为单站多源多数据定位，发射机必须为多个，接收机为1个
		m_sensorDataSPMTMD.resize(1);
		//获取多个发射天线与单个接收天线之间的数据，并进行合并处理
		for (int i = 0; i < m_txNum; ++i) {
			SensorDataCollection curCollection;
			m_raytracingResult[i].GetAllSensorData(curCollection);
			for (auto it = curCollection.m_data.begin(); it != curCollection.m_data.end(); ++it) {
				m_sensorDataSPMTMD[0].m_data.push_back(*it);
			}
		}
		m_sensorDataSPMTMD[0].m_sensorId = 0;
	}
	if (outputConfig.m_outputSensorDataMPMTMD) {				//若为多站多源多数据定位，发射机为多个，接收机也为多个
		m_sensorDataMPMTMD.resize(m_rxNum);
		for (int j = 0; j < m_rxNum; ++j) {
			for (int i = 0; i < m_txNum; ++i) {
				int dataId = i * m_rxNum + j;
				SensorDataCollection curCollection;
				m_raytracingResult[dataId].GetAllSensorData(curCollection);
				for (auto it = curCollection.m_data.begin(); it != curCollection.m_data.end(); ++it) {
					m_sensorDataMPMTMD[j].m_data.push_back(*it);
				}
			}
			m_sensorDataMPMTMD[j].m_sensorId = j;				//计算传感器ID
		}
	}
	
}

void Result::CalculateResult(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method,const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunction)
{
	//0-计算基本信息-计算广义源的位置 
	for (auto it = m_lbsGSResult.begin(); it != m_lbsGSResult.end(); ++it) {
		LBSResultGS& curResult = *it;
		curResult.CalculateBaseInfo(method);
	}

	//1-合并定位结果-将多个传感器的广义源进行合并
	size_t sourceSize = 0;
	for (int i = 0; i < m_lbsGSResult.size(); ++i) {
		size_t oldSize = m_allGeneralSource.size();
		sourceSize += m_lbsGSResult[i].m_sources.size();
		m_allGeneralSource.resize(sourceSize);
		std::copy(m_lbsGSResult[i].m_sources.begin(), m_lbsGSResult[i].m_sources.end(), m_allGeneralSource.begin() + oldSize);
	}

	//2-求解权重矩阵

	//2-按照几何约束条件删除无效广义源
	//创建广义源对,数量为 n*(n-1)/2
	size_t pairNum = static_cast<int>(sourceSize * (sourceSize - 1) / 2);
	std::vector<GSPair*> gsPair(pairNum);										/** @brief	广义源对	*/
	size_t pairId = 0;																/** @brief	广义源对ID	*/
	for (size_t i = 0; i < sourceSize; ++i) {
		for (size_t j = i + 1; j < sourceSize; ++j) {
			gsPair[pairId] = new GSPair(m_allGeneralSource[i], m_allGeneralSource[j]);
			if (!gsPair[pairId]->HasValidAOASolution(scene)) {					//若广义源对无效，则删除该广义源对，计数停止增加
				delete gsPair[pairId];
				continue;
			}
			pairId++;															//广义源对有效，计数增加
		}
	}
	pairNum = pairId;
	gsPair.resize(pairNum);
	gsPair.shrink_to_fit();

	
	//删除权重为0值的广义源-先释放内存，后从数组中删除
	for (auto it = m_allGeneralSource.begin(); it != m_allGeneralSource.end(); ++it) {
		GeneralSource*& curGS = *it;
		if (!curGS->IsValid()) {
			delete curGS;
			curGS = nullptr;
		}
	}
	m_allGeneralSource.erase(std::remove_if(m_allGeneralSource.begin(), m_allGeneralSource.end(), [](const GeneralSource* source) {		//移除无效的广义源
		return source == nullptr;
		}), m_allGeneralSource.end());

	//将所有广义源权重置0
	for (int i = 0; i < m_allGeneralSource.size(); ++i) {
		m_allGeneralSource[i]->m_wCount = 0;					//权重计数归零
		m_allGeneralSource[i]->m_weight = 0;					//权重归零
	}

	//2-2 去除重复的广义源
	EraseRepeatGeneralSources(m_allGeneralSource);
	sourceSize = m_allGeneralSource.size();
	//重新构建广义源对
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
			if (!gsPair[pairId]->HasValidAOASolution(scene)) {					//若广义源对无效，则删除该广义源对，计数停止增加
				delete gsPair[pairId];
				continue;
			}
			pairId++;															//广义源对有效，计数增加
		}
	}
	pairNum = pairId;
	gsPair.resize(pairNum);
	gsPair.shrink_to_fit();

	//2-2  按照物理约束条件计算权重并删除不满足权重阈值的广义源
	// 采用射线追踪算法计算重新计算广义源与广义源之间可能的结果值，若该值不满足相似度阈值，则舍弃
	
	std::vector<std::vector<RaytracingResult>> tempRTResult; //第一维度为解的数量，第二维度为传感器的数量，调用射线追踪，填充该数组
	std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	const MaterialLibrary* matLibrary = &scene->m_materialLibrary;
	const AntennaLibrary* antLibrary = &scene->m_antennaLibrary;
	tempRTResult.resize(pairNum);
	int sensorNum = static_cast<int>(scene->m_sensors.size());																	/** @brief	传感器数量	*/
	std::vector<SensorDataCollection> targetSensorDataCollection(sensorNum);													/** @brief	实时计算目标对传感器的数据	*/
	std::vector<SensorDataCollection> originalSensorDataCollection(sensorNum);													/** @brief	原始的传感器数据	*/
	scene->m_sensorDataLibrary.GetAllSensorDataCollection(originalSensorDataCollection);										//获取原始传感器数据
	RtLbsType max_r_phi = 0.0, max_r_powerDiff = 0.0;																			//角度最大残差、功率最大残差
	for (size_t i = 0; i < pairNum; ++i) {
		GSPair* curPair = gsPair[i];
		DirectlySetResultPath_CPUSingleThread(vroots, scene, splitRadius, curPair->m_targetSolution, tempRTResult[i]);
		for (int j = 0; j< tempRTResult[i].size(); ++j) {
			const Sensor* curSensor = scene->m_sensors[j];
			tempRTResult[i][j].CalculateBaseInfo(curSensor, freqs, antLibrary, matLibrary, tranFunction);						//执行电磁计算,LBS电磁计算
			tempRTResult[i][j].GetMaxPowerSensorData(targetSensorDataCollection[j]);											//收集实时计算的传感器结果
			if (targetSensorDataCollection[j].m_data.size() == 0) {
				curPair->m_isValid = false;
			}
		}
		RtLbsType cur_r_phi = 0.0;																								/** @brief	当前的角度残差	*/
		RtLbsType cur_r_powerDiff = 0.0;																						/** @brief	当前的功率差残差	*/

		if (!curPair->m_isValid) {																								//若当前广义源对无效，则跳过当前pair的相关权重计算
			continue;
		}
		CalculateSensorCollectionResidual_AOA_SingleData(originalSensorDataCollection, targetSensorDataCollection, cur_r_phi, cur_r_powerDiff);		//计算残差二范数和
		curPair->m_phiResidual = cur_r_phi;
		curPair->m_powerDiffResidual = cur_r_powerDiff;
		if (max_r_phi < cur_r_phi) {
			max_r_phi = cur_r_phi;
		}
		if (max_r_powerDiff < cur_r_powerDiff) {
			max_r_powerDiff = cur_r_powerDiff;
		}
	}

	
	//循环pair计算归一化残差系数,并将系数加入对应的广义源权重矩阵中
	for (auto it = gsPair.begin(); it != gsPair.end(); ++it) {
		GSPair* curPair = *it;
		if (curPair->m_isValid) {
			curPair->CalNormalizedWeightAndUpdate_AOA(max_r_phi, max_r_powerDiff);
		}
	}


	//计算所有广义源中权重最大的数值，进行权重归一化
	RtLbsType max_weight = 0.0;
	for (auto it = m_allGeneralSource.begin(); it != m_allGeneralSource.end(); ++it) {
		GeneralSource* curSource = *it;
		if (max_weight < curSource->m_weight) {
			max_weight = curSource->m_weight;
		}
	}

	//广义源权重归一化, 并删除未达到阈值权重的广义源
	for (auto it = m_allGeneralSource.begin(); it != m_allGeneralSource.end(); ++it) {
		GeneralSource*& curSource = *it;
		curSource->NormalizedWeight(max_weight);
		if (curSource->m_weight < 0.5) {
			delete curSource;
			curSource = nullptr;
		}
	}

	//删除无效的广义源
	m_allGeneralSource.erase(std::remove_if(m_allGeneralSource.begin(), m_allGeneralSource.end(), [](const GeneralSource* source) {
		return source == nullptr;
		}), m_allGeneralSource.end());

	//Pair 中的权重归一化，并删除未达到阈值权重的pair
	for (auto it = gsPair.begin(); it != gsPair.end(); ++it) {
		GSPair* curPair = *it;
		curPair->NormalizedWeight(max_weight);
		if (curPair->m_weight < 0.5) {
			delete curPair;
			curPair = nullptr;
		}
	}

	//删除无效的pair
	gsPair.erase(std::remove_if(gsPair.begin(), gsPair.end(), [](const GSPair* pair) {
		return pair == nullptr;
		}), gsPair.end());


	LocalizationSolver();
}

std::vector<GeneralSource*> Result::GetGeneralSource() const
{
	return m_allGeneralSource;
}

void Result::LocalizationSolver()
{
	m_aoaSolver.SetGeneralSource(m_allGeneralSource);
	//Solving_WLS();										//使用加权最小二乘方法求解
	m_aoaSolver.Solving_IRLS(20, 1e-6);								//使用迭代加权最小二乘方法秋季
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
	//输出单站单源多数据定位，输出为单个文件
	std::string sensorDataDirectory = m_directory + "sensor data/SPSTMD/";
	for (int i = 0; i < m_txNum; ++i) {
		std::stringstream ss;
		ss << sensorDataDirectory << "tx" << i << "_SPSTMD.json";
		m_sensorDataSPSTMD[i].Write2Json(ss.str());
	}
	//写入对应的传感器配置文件
	std::stringstream ss1;
	ss1 << sensorDataDirectory << "SPSTMD_sensorconfig.json";
	m_sensorCollectionConfig.Write2Json(ss1.str());

}

void Result::OutputSensorDataMPSTSD() const
{
	//输出多站单源单数据定位，输出为多个文件
	std::string sensorDataDirectory = m_directory + "sensor data/MPSTSD/";
	for (int j = 0; j < m_rxNum; ++j) {
		for (int i = 0; i < m_txNum; ++i) {
			std::stringstream ss;
			ss << sensorDataDirectory << "tx" << i << "_" << "sensor_" << j << "_MPSTSD.json";
			m_sensorDataMPSTSD[i * m_rxNum + j].Write2Json(ss.str());
		}
	}
	//写入对应的传感器配置文件
	std::stringstream ss1;
	ss1 << sensorDataDirectory << "MPSTSD_sensorconfig.json";
	m_sensorCollectionConfig.Write2Json(ss1.str());
}

void Result::OutputSensorDataSPMTMD() const
{
	//输出为单站多源多数据定位，输出为单个文件
	std::string sensorDataDirectory = m_directory + "sensor data/SPMTMD/";
	std::stringstream ss;
	ss << sensorDataDirectory << "SPMTMD.json";
	m_sensorDataSPMTMD[0].Write2Json(ss.str());
	//写入对应的传感器配置文件
	std::stringstream ss1;
	ss1 << sensorDataDirectory << "SPMTMD_sensorconfig.json";
	m_sensorCollectionConfig.Write2Json(ss1.str());
}

void Result::OutputSensorDataMPMTMD() const
{
	//输出为多站多源多数据定位，输出为单个文件
	std::string sensorDataDirectory = m_directory + "sensor data/MPMTMD/";
	for (int i = 0; i < m_rxNum; ++i) {
		std::stringstream ss;
		ss << sensorDataDirectory << "sensor" << i << "_MPMTMD.json";
		m_sensorDataMPMTMD[i].Write2Json(ss.str());
	}
	//写入对应的传感器配置文件
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
