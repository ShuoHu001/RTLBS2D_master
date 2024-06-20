#include "raytracingresult.h"

RaytracingResult::RaytracingResult()
	: m_pathNum(0)
	, m_freqNum(0)
	, m_transmitter(nullptr)
	, m_receiver(nullptr)
	, m_terrainDiffPath(nullptr)
{
}

RaytracingResult::RaytracingResult(const RaytracingResult& result)
	: m_pathNum(result.m_pathNum)
	, m_freqNum(result.m_freqNum)
	, m_transmitter(result.m_transmitter)
	, m_receiver(result.m_receiver)
	, m_freqs(result.m_freqs)
	, m_commonPaths(result.m_commonPaths)
	, m_terrainDiffPath(result.m_terrainDiffPath)
	, m_multipathInfo(result.m_multipathInfo)
	, m_totalVectorEField(result.m_totalVectorEField)
	, m_totalScalarEField(result.m_totalScalarEField)
	, m_vectorPower(result.m_vectorPower)
	, m_scalarPower(result.m_scalarPower)
	, m_loss(result.m_loss)
	, m_magnitudesCFR(result.m_magnitudesCFR)
	, m_magnitudesCIR(result.m_magnitudesCIR)
{
}

RaytracingResult::~RaytracingResult()
{
	//删除路径
}

RaytracingResult& RaytracingResult::operator=(const RaytracingResult& result)
{
	m_pathNum = result.m_pathNum;
	m_freqNum = result.m_freqNum;
	m_transmitter = result.m_transmitter;
	m_receiver = result.m_receiver;
	m_freqs = result.m_freqs;
	m_commonPaths = result.m_commonPaths;
	m_terrainDiffPath = result.m_terrainDiffPath;
	m_multipathInfo = result.m_multipathInfo;
	m_totalVectorEField = result.m_totalVectorEField;
	m_totalScalarEField = result.m_totalScalarEField;
	m_vectorPower = result.m_vectorPower;
	m_scalarPower = result.m_scalarPower;
	m_loss = result.m_loss;
	m_magnitudesCFR = result.m_magnitudesCFR;
	m_magnitudesCIR = result.m_magnitudesCIR;
	return *this;
}

void RaytracingResult::SetRayPath(std::vector<RayPath3D*>& paths)
{
	m_commonPaths = paths;
}

void RaytracingResult::SetRayPath(TerrainDiffractionPath* path)
{
	m_terrainDiffPath = path;
}

void RaytracingResult::CalculateBaseInfo(std::vector<RtLbsType>& freqs, const MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction)
{
	//内存分配
	int pathNum = 0;															/** @brief	总路径数量	*/
	pathNum = static_cast<int>(m_commonPaths.size());
	if (m_terrainDiffPath != nullptr)
		pathNum += 1;
	m_freqs = freqs;

	m_pathNum = pathNum;
	m_freqNum = static_cast<int>(m_freqs.size());

	m_multipathInfo.resize(pathNum * freqs.size());								//多径信息内存分配

	m_totalVectorEField.resize(freqs.size());									//总矢量电场内存分配
	m_totalScalarEField.resize(freqs.size());									//总标量电场内存分配
	m_vectorPower.resize(freqs.size());											//总矢量合成功率内存分配
	m_scalarPower.resize(freqs.size());											//总标量合成功率内存分配
	m_magnitudesCFR.resize(freqs.size());										//CFR复数强度内存分配
	m_magnitudesCIR.resize(freqs.size());										//CIR复数强度内存分配
	m_loss.resize(m_freqNum);

	//计算路径信息并计算小尺度基本信息
	int infoId = 0;																/** @brief	多径信息ID	*/
	for (int i = 0; i < m_freqs.size(); ++i) {
		for (int j = 0; j < m_commonPaths.size(); ++j) {						//设置常规多径
			m_multipathInfo[infoId].SetRayPath(m_commonPaths[j]);
			m_multipathInfo[infoId++].CalculateBaseInfo(m_freqs[i], matLibrary, tranFunction, m_transmitter, m_receiver);
		}
		if (m_terrainDiffPath != nullptr) {
			m_multipathInfo[infoId].SetRayPath(m_terrainDiffPath);
			m_multipathInfo[infoId++].CalculateBaseInfo(m_freqs[i], matLibrary, tranFunction, m_transmitter, m_receiver);
		}
	}
	//计算合成场强与功率
	RtLbsType powerdBm = 10 * log10(m_transmitter->m_power * 1000);
	infoId = 0;
	for (int i = 0; i < m_freqs.size(); ++i) {
		for (int j = 0; j < m_commonPaths.size(); ++j) {
			m_totalVectorEField[i] += m_multipathInfo[infoId].m_vectorEField;
			m_totalScalarEField[i] += m_multipathInfo[infoId++].m_scalarEField;
		}
		if (m_terrainDiffPath != nullptr) {
			m_totalVectorEField[i] += m_multipathInfo[infoId].m_vectorEField;
			m_totalScalarEField[i] += m_multipathInfo[infoId++].m_scalarEField;
		}
		//计算功率
		RtLbsType gain = m_transmitter->GetGain() + m_receiver->GetGain();										//获取系统总增益
		RtLbsType conterm = 20 * log10(LIGHT_VELOCITY_AIR / (m_freqs[i] / QUARTER_PI)) - 20 * log10(30);		//中间变量
		m_vectorPower[i] = 20 * log10(m_totalVectorEField[i].MValue()) + conterm + 30 + gain;					//计算矢量合成功率
		m_scalarPower[i] = 20 * log10(m_totalScalarEField[i]) + conterm + 30 + gain;							//计算标量合成功率
		m_loss[i] = powerdBm + gain - m_scalarPower[i];															//计算损耗
		if (std::isinf(m_vectorPower[i])) {
			m_vectorPower[i] = m_receiver->m_powerThreshold;
		}
		if (std::isinf(m_scalarPower[i])) {
			m_scalarPower[i] = m_receiver->m_powerThreshold;
		}
		if (std::isinf(m_loss[i])) {
			m_loss[i] = m_receiver->m_powerThreshold;
		}
	}

	//计算其他参数
	for (int i = 0; i < m_freqs.size(); ++i) {
		for (int j = 0; j < m_pathNum; ++j) {
			int offset = i * pathNum + j;
			m_multipathInfo[offset].m_powerRatio = m_multipathInfo[offset].m_scalarEField / m_totalScalarEField[i];			//计算能量比值
			m_magnitudesCFR[i] += m_multipathInfo[offset].m_magnitude;															//计算合成复数强度
		}
	}

	//将CFR进行逆傅里叶逆变换得到CIR
	fftw_complex* in = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * m_freqNum);
	fftw_complex* ifftOut = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * m_freqNum);
	for (int i = 0; i < m_freqNum; ++i) {
		in[i][0] = m_magnitudesCFR[i].m_real;
		in[i][1] = m_magnitudesCFR[i].m_imag;
	}
	fftw_plan backwardPlan = fftw_plan_dft_1d(m_freqNum, in, ifftOut, FFTW_BACKWARD, FFTW_ESTIMATE);				//给定逆变换参数				
	fftw_execute(backwardPlan);																						//执行逆傅里叶变换
	for (int i = 0; i < m_freqNum; ++i) {
		m_magnitudesCIR[m_freqNum - i - 1].m_real = ifftOut[i][0];
		m_magnitudesCIR[m_freqNum - i - 1].m_imag = ifftOut[i][1];
	}
	fftw_destroy_plan(backwardPlan);
	fftw_free(in);
	fftw_free(ifftOut);

}

void RaytracingResult::CalculateBaseInfo(const Sensor* sensor, std::vector<RtLbsType>& freqs, const AntennaLibrary* antLibrary, const MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction)
{
	//内存分配
	int pathNum = 0;															/** @brief	总路径数量	*/
	pathNum = static_cast<int>(m_commonPaths.size());
	if (m_terrainDiffPath != nullptr)
		pathNum += 1;
	m_freqs = freqs;

	m_pathNum = pathNum;
	m_freqNum = static_cast<int>(m_freqs.size());

	m_multipathInfo.resize(pathNum * freqs.size());								//多径信息内存分配

	m_totalVectorEField.resize(freqs.size());									//总矢量电场内存分配
	m_totalScalarEField.resize(freqs.size());									//总标量电场内存分配
	m_vectorPower.resize(freqs.size());											//总矢量合成功率内存分配
	m_scalarPower.resize(freqs.size());											//总标量合成功率内存分配


	RtLbsType power = 1.0;
	//计算路径信息并计算小尺度基本信息
	int infoId = 0;																/** @brief	多径信息ID	*/
	for (int i = 0; i < m_freqs.size(); ++i) {
		for (int j = 0; j < m_commonPaths.size(); ++j) {						//设置常规多径
			m_multipathInfo[infoId].SetRayPath(m_commonPaths[j]);
			m_multipathInfo[infoId++].CalculateBaseInfo(power, m_freqs[i], antLibrary, matLibrary, tranFunction, sensor);
		}
		if (m_terrainDiffPath != nullptr) {
			m_multipathInfo[infoId].SetRayPath(m_terrainDiffPath);
			m_multipathInfo[infoId++].CalculateBaseInfo(power, m_freqs[i], antLibrary, matLibrary, tranFunction, sensor);
		}
	}

	//计算合成场强与功率
	RtLbsType powerdBm = 10 * log10(power * 1000);
	infoId = 0;
	for (int i = 0; i < m_freqs.size(); ++i) {
		for (int j = 0; j < m_commonPaths.size(); ++j) {
			m_totalVectorEField[i] += m_multipathInfo[infoId].m_vectorEField;
			m_totalScalarEField[i] += m_multipathInfo[infoId++].m_scalarEField;
		}
		if (m_terrainDiffPath != nullptr) {
			m_totalVectorEField[i] += m_multipathInfo[infoId].m_vectorEField;
			m_totalScalarEField[i] += m_multipathInfo[infoId++].m_scalarEField;
		}
		//计算功率
		RtLbsType gain = sensor->GetGain();																		//获取系统总增益
		RtLbsType conterm = 20 * log10(LIGHT_VELOCITY_AIR / (m_freqs[i] / QUARTER_PI)) - 20 * log10(30);		//中间变量
		m_vectorPower[i] = 20 * log10(m_totalVectorEField[i].MValue()) + conterm + 30 + gain;					//计算矢量合成功率
		m_scalarPower[i] = 20 * log10(m_totalScalarEField[i]) + conterm + 30 + gain;							//计算标量合成功率
	}
}

void RaytracingResult::GetAllSensorData_AOA2D(SensorDataCollection& collection, RtLbsType threshold, RtLbsType sparseFactor) const
{
	if (m_pathNum == 0) {
		return;
	}
	std::vector<PathInfo> pathInfoCopy = m_multipathInfo;
	//将pathCopy按照角度进行聚类
	std::vector<PathInfoCluster> clusters = ClusterPathInfoByAOA2D(pathInfoCopy, threshold);
	//按照功率大小对类进行从大到小排序
	std::sort(clusters.begin(), clusters.end(), ComparedByPower_PathInfoCluster);

	int clusterNum = static_cast<int>(clusters.size());															//簇数量
	int sparsedClusterNum = static_cast<int>(std::round(sparseFactor * clusters.size()));									//按照百分比保留稀疏的数据簇
	if (sparsedClusterNum < 3) {																				//至少保留三个有效数据
		if (clusterNum >= 3) {
			sparsedClusterNum = 3;
		}
		else {
			sparsedClusterNum = clusterNum;
			LOG_WARNING << "RaytracingResult: generate sensor data size is " << sparsedClusterNum << " , may not satisfy localization condition." << ENDL;
		}
		
	}

	for (int i = 0; i < sparsedClusterNum; ++i) {
		SensorData curSensorData;
		clusters[i].m_mergedInfo.Convert2SensorData(curSensorData);
		collection.m_data.push_back(curSensorData);
	}
}


void RaytracingResult::GetMaxPowerSensorData_AOA2D(SensorDataCollection& collection, RtLbsType threshold) const
{
	if (m_pathNum == 0) {
		return;
	}
	std::vector<PathInfo> pathInfoCopy = m_multipathInfo;
	//将pathCopy按照角度进行聚类
	std::vector<PathInfoCluster> clusters = ClusterPathInfoByAOA2D(pathInfoCopy, threshold);
	//按照功率大小对类进行从大到小排序
	std::sort(clusters.begin(), clusters.end(), ComparedByPower_PathInfoCluster);
	SensorData maxPowerSensorData;
	clusters.front().m_mergedInfo.Convert2SensorData(maxPowerSensorData);
	
	maxPowerSensorData.m_power = m_scalarPower[0];					//最大功率传感器数据应对应的是合成功率
	collection.m_data.push_back(maxPowerSensorData);
}

void RaytracingResult::GetAllSensorData_AOA3D(SensorDataCollection& collection, RtLbsType threshold, RtLbsType sparseFactor) const
{
	if (m_pathNum == 0) {
		return;
	}
	std::vector<PathInfo> pathInfoCopy = m_multipathInfo;
	//将pathCopy按照角度进行聚类
	std::vector<PathInfoCluster> clusters = ClusterPathInfoByAOA3D(pathInfoCopy, threshold);
	//按照功率大小对类进行从大到小排序
	std::sort(clusters.begin(), clusters.end(), ComparedByPower_PathInfoCluster);

	int clusterNum = static_cast<int>(clusters.size());															//簇数量
	int sparsedClusterNum = static_cast<int>(std::round(sparseFactor * clusters.size()));									//按照百分比保留稀疏的数据簇
	if (sparsedClusterNum < 3) {																				//至少保留三个有效数据
		if (clusterNum >= 3) {
			sparsedClusterNum = 3;
		}
		else {
			sparsedClusterNum = clusterNum;
			LOG_WARNING << "RaytracingResult: generate sensor data size is " << sparsedClusterNum << " , may not satisfy localization condition." << ENDL;
		}

	}

	for (int i = 0; i < sparsedClusterNum; ++i) {
		SensorData curSensorData;
		clusters[i].m_mergedInfo.Convert2SensorData(curSensorData);
		collection.m_data.push_back(curSensorData);
	}
}

void RaytracingResult::GetMaxPowerSensorData_AOA3D(SensorDataCollection& collection, RtLbsType threshold) const
{
	if (m_pathNum == 0) {
		return;
	}
	std::vector<PathInfo> pathInfoCopy = m_multipathInfo;
	//将pathCopy按照角度进行聚类
	std::vector<PathInfoCluster> clusters = ClusterPathInfoByAOA3D(pathInfoCopy, threshold);
	//按照功率大小对类进行从大到小排序
	std::sort(clusters.begin(), clusters.end(), ComparedByPower_PathInfoCluster);
	SensorData maxPowerSensorData;
	clusters.front().m_mergedInfo.Convert2SensorData(maxPowerSensorData);

	maxPowerSensorData.m_power = m_scalarPower[0];					//最大功率传感器数据应对应的是合成功率
	collection.m_data.push_back(maxPowerSensorData);
}

void RaytracingResult::GetAllSensorData_Delay(SensorDataCollection& collection, RtLbsType threshold, RtLbsType sparseFactor) const
{
	if (m_pathNum == 0) {
		return;
	}
	std::vector<PathInfo> pathInfoCopy = m_multipathInfo;
	//将pathCopy按照角度进行聚类
	std::vector<PathInfoCluster> clusters = ClusterPathInfoByDelay(pathInfoCopy, threshold);
	//按照功率大小对类进行从大到小排序
	std::sort(clusters.begin(), clusters.end(), ComparedByDelay_PathInfoCluster);

	int clusterNum = static_cast<int>(clusters.size());															//簇数量
	int sparsedClusterNum = static_cast<int>(std::round(sparseFactor * clusters.size()));									//按照百分比保留稀疏的数据簇
	if (sparsedClusterNum < 3) {																				//至少保留三个有效数据
		if (clusterNum >= 3) {
			sparsedClusterNum = 3;
		}
		else {
			sparsedClusterNum = clusterNum;
			LOG_WARNING << "RaytracingResult: generate sensor data size is " << sparsedClusterNum << " , may not satisfy localization condition." << ENDL;
		}

	}

	for (int i = 0; i < sparsedClusterNum; ++i) {
		SensorData curSensorData;
		clusters[i].m_mergedInfo.Convert2SensorData(curSensorData);
		collection.m_data.push_back(curSensorData);
	}
}

void RaytracingResult::GetMaxPowerSensorData_Delay(SensorDataCollection& collection, RtLbsType threshold) const
{
	if (m_pathNum == 0) {
		return;
	}
	std::vector<PathInfo> pathInfoCopy = m_multipathInfo;
	//将pathCopy按照角度进行聚类
	std::vector<PathInfoCluster> clusters = ClusterPathInfoByDelay(pathInfoCopy, threshold);
	//按照功率大小对类进行从大到小排序
	std::sort(clusters.begin(), clusters.end(), ComparedByDelay_PathInfoCluster);
	SensorData maxPowerSensorData;
	clusters.front().m_mergedInfo.Convert2SensorData(maxPowerSensorData);

	maxPowerSensorData.m_power = m_scalarPower[0];					//最大功率传感器数据应对应的是合成功率
	collection.m_data.push_back(maxPowerSensorData);
}

void RaytracingResult::OutputVectorEField(std::ofstream& stream) const
{
	for (int i = 0; i < m_freqNum; ++i) {
		stream << m_transmitter->m_id << '\t' << m_receiver->m_id<< '\t' << static_cast<int>(m_freqs[i]) << '\t';
		stream << m_transmitter->m_position.x << '\t' << m_transmitter->m_position.y << '\t' << m_transmitter->m_position.z << '\t';
		stream << m_receiver->m_position.x << '\t' << m_receiver->m_position.y << '\t' << m_receiver->m_position.z << '\t';
		stream << m_totalVectorEField[i].m_real << '\t' << m_totalVectorEField[i].m_imag << std::endl;
	}
}

void RaytracingResult::OutputScalarEField(std::ofstream& stream) const
{
	for (int i = 0; i < m_freqNum; ++i) {
		stream << m_transmitter->m_id << '\t' << m_receiver->m_id << '\t' << static_cast<int>(m_freqs[i]) << '\t';
		stream << m_transmitter->m_position.x << '\t' << m_transmitter->m_position.y << '\t' << m_transmitter->m_position.z << '\t';
		stream << m_receiver->m_position.x << '\t' << m_receiver->m_position.y << '\t' << m_receiver->m_position.z << '\t';
		stream << m_totalScalarEField[i] << '\t' << std::endl;
	}
}

void RaytracingResult::OutputVectorPower(std::ofstream& stream) const
{
	for (int i = 0; i < m_freqNum; ++i) {
		stream << m_transmitter->m_id << '\t' << m_receiver->m_id << '\t' << static_cast<int>(m_freqs[i]) << '\t';
		stream << m_transmitter->m_position.x << '\t' << m_transmitter->m_position.y << '\t' << m_transmitter->m_position.z << '\t';
		stream << m_receiver->m_position.x << '\t' << m_receiver->m_position.y << '\t' << m_receiver->m_position.z << '\t';
		stream << m_vectorPower[i] << std::endl;
	}
}

void RaytracingResult::OutputScalarPower(std::ofstream& stream) const
{
	for (int i = 0; i < m_freqNum; ++i) {
		stream << m_transmitter->m_id << '\t' << m_receiver->m_id << '\t' << static_cast<int>(m_freqs[i]) << '\t';
		stream << m_transmitter->m_position.x << '\t' << m_transmitter->m_position.y << '\t' << m_transmitter->m_position.z << '\t';
		stream << m_receiver->m_position.x << '\t' << m_receiver->m_position.y << '\t' << m_receiver->m_position.z << '\t';
		stream << m_scalarPower[i] << std::endl;
	}
}

void RaytracingResult::OutputLoss(std::ofstream& stream) const
{
	for (int i = 0; i < m_freqNum; ++i) {
		stream << m_transmitter->m_id << '\t' << m_receiver->m_id << '\t' << m_freqs[i] << '\t';
		stream << m_transmitter->m_position.x << '\t' << m_transmitter->m_position.y << '\t' << m_transmitter->m_position.z << '\t';
		stream << m_receiver->m_position.x << '\t' << m_receiver->m_position.y << '\t' << m_receiver->m_position.z << '\t';
		stream << m_loss[i] << std::endl;
	}
}

void RaytracingResult::OutputRayPath(std::ofstream& stream) const
{
	stream << "multipath" << "\t" << m_transmitter->m_id << "\t" << m_receiver->m_id << "\t" <<m_pathNum << std::endl;				//写入节点编号信息
	for (auto it = m_commonPaths.begin(); it != m_commonPaths.end(); ++it) {							//输出常规路径
		RayPath3D* path = *it;
		path->OutputRaypath(stream);
	}
	if (m_terrainDiffPath != nullptr) {
		m_terrainDiffPath->OuputRaypath(stream);
	}
}

void RaytracingResult::OutputPDP(std::ofstream& stream) const
{
	stream << "pdpinfo" << "\t" << m_transmitter->m_id << "\t" << m_receiver->m_id << "\t" << m_freqNum << "\t" << m_pathNum << std::endl;				//写入节点编号信息
	for (int i = 0; i < m_freqNum; ++i) {
		for (int j = 0; j < m_pathNum; ++j) {
			int offset = i * m_pathNum + j;
			stream << m_transmitter->m_id << "\t" << m_receiver->m_id << "\t" << i << "\t" << j << "\t" << m_freqs[i] << "\t" << m_multipathInfo[offset].m_timeDelay << "\t" << m_multipathInfo[offset].m_power << std::endl;
		}
	}
}

void RaytracingResult::OutputCFR(std::ofstream& stream) const
{
	stream << "cfrinfo" << "\t" << m_transmitter->m_id << "\t" << m_receiver->m_id << "\t" << m_freqNum << std::endl;				//写入节点编号信息
	for (int i = 0; i < m_freqNum; ++i) {
		stream << m_transmitter->m_id << "\t" << m_receiver->m_id << "\t" << i << "\t" << m_magnitudesCFR[i].m_real << "\t" << m_magnitudesCFR[i].m_imag << std::endl;
	}
}

void RaytracingResult::OutputCIR(std::ofstream& stream) const
{
	stream << "cfrinfo" << "\t" << m_transmitter->m_id << "\t" << m_receiver->m_id << "\t" << m_freqNum << std::endl;				//写入节点编号信息
	for (int i = 0; i < m_freqNum; ++i) {
		stream << m_transmitter->m_id << "\t" << m_receiver->m_id << "\t" << i << "\t" << m_magnitudesCIR[i].m_real << "\t" << m_magnitudesCIR[i].m_imag << std::endl;
	}
}

void RaytracingResult::OutputAOA(std::ofstream& stream) const
{
	stream << "aoainfo" << "\t" << m_transmitter->m_id << "\t" << m_receiver->m_id << "\t" << m_freqNum << "\t" << m_pathNum << std::endl;				//写入节点编号信息
	for (int i = 0; i < m_freqNum; ++i) {
		for (int j = 0; j < m_pathNum; ++j) {
			int offset = i * m_pathNum + j;
			stream << m_transmitter->m_id << "\t" << m_receiver->m_id << "\t" << i << "\t" << j << "\t" << m_freqs[i] << "\t" << m_multipathInfo[offset].m_aoAPhi << "\t" << m_multipathInfo[offset].m_aoATheta << "\t" << m_multipathInfo[offset].m_power << std::endl;
		}
	}
}

void RaytracingResult::OutputAOD(std::ofstream& stream) const
{
	stream << "aoainfo" << "\t" << m_transmitter->m_id << "\t" << m_receiver->m_id << "\t" << m_freqNum << "\t" << m_pathNum << std::endl;				//写入节点编号信息
	for (int i = 0; i < m_freqNum; ++i) {
		for (int j = 0; j < m_pathNum; ++j) {
			int offset = i * m_pathNum + j;
			stream << m_transmitter->m_id << "\t" << m_receiver->m_id << "\t" << i << "\t" << j << "\t" << m_freqs[i] << "\t" << m_multipathInfo[offset].m_aoDPhi << "\t" << m_multipathInfo[offset].m_aoDTheta << "\t" << m_multipathInfo[offset].m_power << std::endl;
		}
	}
}
