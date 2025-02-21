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

	m_sensorDataSPSTMD.clear();
	std::vector<SensorDataCollection>().swap(m_sensorDataSPSTMD);

	m_sensorDataMPSTSD.clear();
	std::vector<SensorDataCollection>().swap(m_sensorDataMPSTSD);

	m_sensorDataSPMTMD.clear();
	std::vector<SensorDataCollection>().swap(m_sensorDataSPMTMD);

	m_sensorDataMPMTMD.clear();
	std::vector<SensorDataCollection>().swap(m_sensorDataMPMTMD);
}

void Result::Init(const std::vector<Transmitter*>& transmitters, const std::vector<Receiver*> receivers)
{
	unsigned txNum = static_cast<unsigned>(transmitters.size());
	unsigned rxNum = static_cast<unsigned>(receivers.size());
	unsigned totalNum = static_cast<unsigned>(txNum * rxNum);
	m_raytracingResult.resize(totalNum);
	m_transmitters = transmitters;
	m_receivers = receivers;
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
	for (unsigned i = 0; i < m_rxNum; ++i) {
		m_sensorCollectionConfig.m_sensorConfigs[i].m_id = i;
		m_sensorCollectionConfig.m_sensorConfigs[i].m_position = receivers[i]->m_position;
	}

	return;
}

void Result::OutputResult(const OutputConfig& config)
{
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
	if (config.m_outputSpreadInfo) {
		OutputSpreadProfile();
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
	if (config.m_outputMultiStationCRLB) {
		OutputMultiStationCRLB(config.m_outputLBSErrorConfig);
	}
	if (config.m_outputMultiStationGDOP) {
		OutputMultiStationGDOP(config.m_outputLBSErrorConfig);
	}
	if (config.m_outputSingleStationCRLB) {
		OutputSingleStationCRLB(config.m_outputLBSErrorConfig);
	}
	if (config.m_outputSingleStationGDOP) {
		OutputSingleStationGDOP(config.m_outputLBSErrorConfig);
	}
	OutputLocationRange();			//输出论文中需要的定位范围
}

void Result::CalculateResult_RT_SensorData(const OutputConfig& outputConfig, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData)
{
	//计算射线追踪结果
	std::vector<RtLbsType> freqs = freqConfig.GetFrequencyInformation();
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		RaytracingResult& result = *it;
		result.CalculateBaseInfo(freqs, tranFunctionData);
	}

	RtLbsType sensorDataSparseFactor = outputConfig.m_outputSensorDataSparseFactor;					/** @brief	输出传感器数据的稀疏度	*/
	//计算传感器数据结果,默认定位为单频点下的定位模式
	if (outputConfig.m_outputSensorDataSPSTMD) {				//若为单站单源多数据定位，发射机可以为多个，接收机为1
		m_sensorDataSPSTMD.resize(m_txNum);
		if (outputConfig.m_outputLBSMethod == LBS_METHOD_RT_AOA) {
			RtLbsType threshold = m_raytracingResult[0].m_receiver->m_angularThreshold;
			for (unsigned i = 0; i < m_txNum; ++i) {
				m_raytracingResult[i].GetAllSensorData_AOA2D(m_sensorDataSPSTMD[i], threshold, sensorDataSparseFactor);
				m_sensorDataSPSTMD[i].m_sensorId = 0;				//计算传感器ID
				m_sensorDataSPSTMD[i].CalculateTimeDiff();			//计算时延差值
			}
		}
		else if (outputConfig.m_outputLBSMethod == LBS_METHOD_RT_TOA ||
			outputConfig.m_outputLBSMethod == LBS_METHOD_RT_AOA_TOA ||
			outputConfig.m_outputLBSMethod == LBS_METHOD_RT_AOA_TDOA) {
			RtLbsType threshold = m_raytracingResult[0].m_receiver->m_delayThreshold;
			for (unsigned i = 0; i < m_txNum; ++i) {
				m_raytracingResult[i].GetAllSensorData_Delay(m_sensorDataSPSTMD[i], threshold, sensorDataSparseFactor);
				m_sensorDataSPSTMD[i].m_sensorId = 0;				//计算传感器ID
				m_sensorDataSPSTMD[i].CalculateTimeDiff();			//计算时延差值
			}
		}
	}
	if (outputConfig.m_outputSensorDataMPSTSD) {				//若为多站单源单数据定位，发射机可以为多个，接收机为多个
		//计算每个发射天线与每个接收天线间的传感器数据（最大值）
		m_sensorDataMPSTSD.resize(m_txNum * m_rxNum);
		if (outputConfig.m_outputLBSMethod == LBS_METHOD_RT_AOA) {
			for (unsigned i = 0; i < m_txNum; ++i) {
				for (unsigned j = 0; j < m_rxNum; ++j) {
					RtLbsType threshold = m_raytracingResult[i * m_rxNum + j].m_receiver->m_angularThreshold;
					m_raytracingResult[i * m_rxNum + j].GetMaxPowerSensorData_AOA2D(m_sensorDataMPSTSD[i * m_rxNum + j], threshold);
					m_sensorDataMPSTSD[i * m_rxNum + j].m_sensorId = j;		//计算传感器ID，该ID即为接收机的ID
				}
			}
		}
		else if (outputConfig.m_outputLBSMethod == LBS_METHOD_RT_TOA ||
			outputConfig.m_outputLBSMethod == LBS_METHOD_RT_AOA_TOA	||
			outputConfig.m_outputLBSMethod == LBS_METHOD_RT_AOA_TDOA) {
			for (unsigned i = 0; i < m_txNum; ++i) {
				for (unsigned j = 0; j < m_rxNum; ++j) {
					RtLbsType threshold = m_raytracingResult[i * m_rxNum + j].m_receiver->m_angularThreshold;
					m_raytracingResult[i * m_rxNum + j].GetMinDelaySensorData_Delay(m_sensorDataMPSTSD[i * m_rxNum + j], threshold);
					m_sensorDataMPSTSD[i * m_rxNum + j].m_sensorId = j;		//计算传感器ID，该ID即为接收机的ID
				}
			}
		}
		//计算时延差值
		for (unsigned i = 0; i < m_txNum; ++i) {
			RtLbsType firstTimeDelay = m_sensorDataMPSTSD[i * m_rxNum].m_datas[0].m_time;
			for (unsigned j = 1; j < m_rxNum; ++j) {
				m_sensorDataMPSTSD[i * m_rxNum + j].m_datas[0].m_timeDiff = m_sensorDataMPSTSD[i * m_rxNum + j].m_datas[0].m_time - firstTimeDelay;
			}
		}
	}
	if (outputConfig.m_outputSensorDataSPMTMD) {				//若为单站多源多数据定位，发射机必须为多个，接收机为1个
		m_sensorDataSPMTMD.resize(1);
		//获取多个发射天线与单个接收天线之间的数据，并进行合并处理
		RtLbsType threshold = m_raytracingResult[0].m_receiver->m_angularThreshold;
		for (unsigned i = 0; i < m_txNum; ++i) {
			SensorDataCollection curCollection;
			m_raytracingResult[i].GetAllSensorData_AOA2D(curCollection, threshold, sensorDataSparseFactor);
			for (auto it = curCollection.m_datas.begin(); it != curCollection.m_datas.end(); ++it) {
				m_sensorDataSPMTMD[0].m_datas.push_back(*it);
			}
		}
		m_sensorDataSPMTMD[0].m_sensorId = 0;
	}
	if (outputConfig.m_outputSensorDataMPMTMD) {				//若为多站多源多数据定位，发射机为多个，接收机也为多个
		m_sensorDataMPMTMD.resize(m_rxNum);
		for (unsigned j = 0; j < m_rxNum; ++j) {
			for (unsigned i = 0; i < m_txNum; ++i) {
				unsigned dataId = i * m_rxNum + j;
				RtLbsType threshold = m_raytracingResult[dataId].m_receiver->m_angularThreshold;
				SensorDataCollection curCollection;
				m_raytracingResult[dataId].GetAllSensorData_AOA2D(curCollection, threshold, sensorDataSparseFactor);
				for (auto it = curCollection.m_datas.begin(); it != curCollection.m_datas.end(); ++it) {
					m_sensorDataMPMTMD[j].m_datas.push_back(*it);
				}
			}
			m_sensorDataMPMTMD[j].m_sensorId = j;				//计算传感器ID
		}
	}

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

void Result::OutputRayPath()
{
	//输出全部路径
	std::ofstream stream(m_directory + "multipath.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open multipath file failed." << ENDL;
		return;
	}
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		RaytracingResult& result = *it;
		result.OutputRayPath(stream);
	}
	stream.close();

	//输出最小时延多径
	std::ofstream stream1(m_directory + "min_delay_multipath.txt");
	if (!stream1.is_open()) {
		LOG_ERROR << "Result:open multipath file failed." << ENDL;
		return;
	}
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		const RaytracingResult& result = *it;
		result.OutputMinDelayRayPath(stream1);
	}
	stream1.close();

	//输出最大功率多径
	std::ofstream stream2(m_directory + "max_power_multipath.txt");
	if (!stream2.is_open()) {
		LOG_ERROR << "Result:open multipath file failed." << ENDL;
		return;
	}
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		const RaytracingResult& result = *it;
		result.OutputMaxPowerRayPath(stream2);
	}
	stream2.close();
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

void Result::OutputSpreadProfile() const
{
	std::ofstream stream(m_directory + "spreadinfo.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open aod file failed." << ENDL;
		return;
	}
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		const RaytracingResult& result = *it;
		result.OutputSpreadProfile(stream);
	}
	stream.close();
}

void Result::OutputSensorDataSPSTMD() const
{
	//输出单站单源多数据定位，输出为单个文件
	std::string sensorDataDirectory = m_directory + "sensor data/SPSTMD/";
	for (unsigned i = 0; i < m_txNum; ++i) {
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
	for (unsigned j = 0; j < m_rxNum; ++j) {
		for (unsigned i = 0; i < m_txNum; ++i) {
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
	for (unsigned i = 0; i < m_rxNum; ++i) {
		std::stringstream ss;
		ss << sensorDataDirectory << "sensor" << i << "_MPMTMD.json";
		m_sensorDataMPMTMD[i].Write2Json(ss.str());
	}
	//写入对应的传感器配置文件
	std::stringstream ss1;
	ss1 << sensorDataDirectory << "MPMTMD_sensorconfig.json";
	m_sensorCollectionConfig.Write2Json(ss1.str());
}

//void Result::OutputGeneralSource() const
//{
//	std::ofstream stream(m_directory + "generalsource.txt");
//	if (!stream.is_open()) {
//		LOG_ERROR << "Result:open general-source file failed." << ENDL;
//		return;
//	}
//	for (auto it = m_allGeneralSource.begin(); it != m_allGeneralSource.end(); ++it) {
//		const GeneralSource* source = *it;
//		source->Output2File(stream);
//	}
//	stream.close();
//}

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

void Result::OutputMultiStationCRLB(const LBSErrorConfig& config) const
{
	std::ofstream stream(m_directory + "multistation_crlb.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open multistation_crlb file failed." << ENDL;
		return;
	}
	std::vector<std::vector<Point2D>> gss;								/** @brief	所有广义源坐标，第一维度为rx数量，第二维度为tx数量，相当于一个rx对应于多个发射机的情况	*/
	gss.resize(m_rxNum);
	//多个发射机为基站，接收机为移动站
	if (config.m_errorType == LBSERRORTYPE_AOA) {
		for (int i = 0; i < m_txNum; ++i) {
			for (int j = 0; j < m_rxNum; ++j) {
				int offset = i * m_rxNum + j;
				Point2D gs;
				if (m_raytracingResult[offset].GetMaxPowerGeneralSource(gs)) {
					gss[j].push_back(gs);
				}
			}
		}
		//在获取广义源之后需要计算CRLB并输出至文件中
		for (int i = 0; i < m_rxNum; ++i) {
			for (auto phiError : config.m_phiErrorSigmas) {
				RtLbsType curCRLB = ComputeCRLBForAOA(gss[i], m_receivers[i]->GetPosition2D(), phiError);
				stream << i << "\t" << m_receivers[i]->m_position.x << "\t" << m_receivers[i]->m_position.y << "\t" << m_receivers[i]->m_position.z << "\t"
					<< phiError << "\t" << curCRLB << std::endl;
			}
		}
	}
	else if (config.m_errorType == LBSERRORTYPE_TOA) {
		for (int i = 0; i < m_txNum; ++i) {
			for (int j = 0; j < m_rxNum; ++j) {
				int offset = i * m_rxNum + j;
				Point2D gs;
				if (m_raytracingResult[offset].GetMinDelayGeneralSource(gs)) {
					gss[j].push_back(gs);
				}
			}
		}
		//在获取广义源之后需要计算CRLB并输出至文件中
		for (int i = 0; i < m_rxNum; ++i) {
			for (auto timeError : config.m_timeMErrorSigmas) {
				RtLbsType curCRLB = ComputeCRLBForTOA(gss[i], m_receivers[i]->GetPosition2D(), timeError);
				stream << i << "\t" << m_receivers[i]->m_position.x << "\t" << m_receivers[i]->m_position.y << "\t" << m_receivers[i]->m_position.z << "\t"
					<< timeError << "\t" << curCRLB << std::endl;
			}
		}
	}
	else if (config.m_errorType == LBSERRORTYPE_AOATOA) {
		for (int i = 0; i < m_txNum; ++i) {
			for (int j = 0; j < m_rxNum; ++j) {
				int offset = i * m_rxNum + j;
				Point2D gs;
				if (m_raytracingResult[offset].GetMinDelayGeneralSource(gs)) {
					gss[j].push_back(gs);
				}
			}
		}
		//在获取广义源之后需要计算CRLB并输出至文件中
		for (int i = 0; i < m_rxNum; ++i) {
			for (auto phiError : config.m_phiErrorSigmas) {
				for (auto timeError : config.m_timeMErrorSigmas) {
					RtLbsType curCRLB = ComputeCRLBForAOATOA(gss[i], m_receivers[i]->GetPosition2D(), phiError, timeError);
					stream << i << "\t" << m_receivers[i]->m_position.x << "\t" << m_receivers[i]->m_position.y << "\t" << m_receivers[i]->m_position.z << "\t"
						<< phiError << "\t" << timeError << "\t" << curCRLB << std::endl;
				}
			}
		}
	}
	else if (config.m_errorType == LBSERRORTYPEAOATDOA) {
		for (int i = 0; i < m_txNum; ++i) {
			for (int j = 0; j < m_rxNum; ++j) {
				int offset = i * m_rxNum + j;
				Point2D gs;
				if (m_raytracingResult[offset].GetMinDelayGeneralSource(gs)) {
					gss[j].push_back(gs);
				}
			}
		}
		//在获取广义源之后需要计算CRLB并输出至文件中
		for (int i = 0; i < m_rxNum; ++i) {
			for (auto phiError : config.m_phiErrorSigmas) {
				for (auto timeError : config.m_timeMErrorSigmas) {
					RtLbsType curCRLB = ComputeCRLBForAOATDOA(gss[i], m_receivers[i]->GetPosition2D(), phiError, timeError);
					stream << i << "\t" << m_receivers[i]->m_position.x << "\t" << m_receivers[i]->m_position.y << "\t" << m_receivers[i]->m_position.z << "\t"
						<< phiError << "\t" << timeError << "\t" << curCRLB << std::endl;
				}
			}
		}
	}
	
	stream.close();
}

void Result::OutputMultiStationGDOP(const LBSErrorConfig& config) const
{
	std::ofstream stream(m_directory + "multistation_gdop.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open multistation_gdop file failed." << ENDL;
		return;
	}
	std::vector<std::vector<Point2D>> gss;								/** @brief	所有广义源坐标，第一维度为rx数量，第二维度为tx数量，相当于一个rx对应于多个发射机的情况	*/
	gss.resize(m_rxNum);
	//多个发射机为基站，接收机为移动站
	if (config.m_errorType == LBSERRORTYPE_AOA) {
		for (int i = 0; i < m_txNum; ++i) {
			for (int j = 0; j < m_rxNum; ++j) {
				int offset = i * m_rxNum + j;
				Point2D gs;
				if (m_raytracingResult[offset].GetMaxPowerGeneralSource(gs)) {
					gss[j].push_back(gs);
				}
			}
		}
		//在获取广义源之后需要计算CRLB并输出至文件中
		for (int i = 0; i < m_rxNum; ++i) {
			RtLbsType curGDOP = ComputeGDOPForAOA(gss[i], m_receivers[i]->GetPosition2D());
			stream << i << "\t" << m_receivers[i]->m_position.x << "\t" << m_receivers[i]->m_position.y << "\t" << m_receivers[i]->m_position.z << "\t"
				<< curGDOP << std::endl;
		}
	}
	else if (config.m_errorType == LBSERRORTYPE_TOA) {
		for (int i = 0; i < m_txNum; ++i) {
			for (int j = 0; j < m_rxNum; ++j) {
				int offset = i * m_rxNum + j;
				Point2D gs;
				if (m_raytracingResult[offset].GetMinDelayGeneralSource(gs)) {
					gss[j].push_back(gs);
				}
			}
		}
		//在获取广义源之后需要计算CRLB并输出至文件中
		for (int i = 0; i < m_rxNum; ++i) {
			RtLbsType curGDOP = ComputeGDOPForTOA(gss[i], m_receivers[i]->GetPosition2D());
			stream << i << "\t" << m_receivers[i]->m_position.x << "\t" << m_receivers[i]->m_position.y << "\t" << m_receivers[i]->m_position.z << "\t"
				 << curGDOP << std::endl;
		}
	}
	else if (config.m_errorType == LBSERRORTYPE_AOATOA) {
		for (int i = 0; i < m_txNum; ++i) {
			for (int j = 0; j < m_rxNum; ++j) {
				int offset = i * m_rxNum + j;
				Point2D gs;
				if (m_raytracingResult[offset].GetMinDelayGeneralSource(gs)) {
					gss[j].push_back(gs);
				}
			}
		}
		//在获取广义源之后需要计算CRLB并输出至文件中
		for (int i = 0; i < m_rxNum; ++i) {
			RtLbsType curGDOP = ComputeGDOPForAOATOA(gss[i], m_receivers[i]->GetPosition2D());
			stream << i << "\t" << m_receivers[i]->m_position.x << "\t" << m_receivers[i]->m_position.y << "\t" << m_receivers[i]->m_position.z << "\t"
				<< curGDOP << std::endl;
		}
	}
	else if (config.m_errorType == LBSERRORTYPEAOATDOA) {
		for (int i = 0; i < m_txNum; ++i) {
			for (int j = 0; j < m_rxNum; ++j) {
				int offset = i * m_rxNum + j;
				Point2D gs;
				if (m_raytracingResult[offset].GetMinDelayGeneralSource(gs)) {
					gss[j].push_back(gs);
				}
			}
		}
		//在获取广义源之后需要计算CRLB并输出至文件中
		for (int i = 0; i < m_rxNum; ++i) {
			RtLbsType curGDOP = ComputeGDOPForAOATDOA(gss[i], m_receivers[i]->GetPosition2D());
			stream << i << "\t" << m_receivers[i]->m_position.x << "\t" << m_receivers[i]->m_position.y << "\t" << m_receivers[i]->m_position.z << "\t"
				<< curGDOP << std::endl;
		}
	}
	stream.close();
}

void Result::OutputSingleStationCRLB(const LBSErrorConfig& config) const
{
	std::ofstream stream(m_directory + "gscrlb.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open gscrlb file failed." << ENDL;
		return;
	}
	for (auto& curResult: m_raytracingResult) {
		curResult.OutputCRLB(stream, config);
	}
	stream.close();
}

void Result::OutputSingleStationGDOP(const LBSErrorConfig& config) const
{
	std::ofstream stream(m_directory + "gdop.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open gdop file failed." << ENDL;
		return;
	}
	for (auto& curResult : m_raytracingResult) {
		curResult.OutputGDOP(stream, config);
	}
	stream.close();
}

void Result::OutputLocationRange() const
{
	std::ofstream stream(m_directory + "lbsrange.txt");
	if (!stream.is_open()) {
		LOG_ERROR << "Result:open loss file failed." << ENDL;
		return;
	}
	for (auto it = m_raytracingResult.begin(); it != m_raytracingResult.end(); ++it) {
		const RaytracingResult& result = *it;
		result.OutputLocationRange(stream);
	}
	stream.close();
}
