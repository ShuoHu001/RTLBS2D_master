#include "system.h"
#include "tree/gpu/raypathgpu.h"
#include "accel/gpu/sdfgpu.h"

System::System()
	: m_sysMode(MODE_RT)
{
	m_configFileName = std::filesystem::current_path().generic_string() + "/config/sysconfig.json";;
	m_scene = new Scene();
}

System::~System()
{
	delete m_scene;
}

bool System::Setup()
{
	//0-数据初始化
	m_tranFunctionData = GetTranFunctionData();

	//1-初始化整体配置
	if (!m_simConfig.Init(m_configFileName))
		return false;
	m_sysMode = m_simConfig.m_systemMode;
	//2-初始化场景
	if (!m_scene->LoadScene(m_simConfig))
		return false;
	//3-配置计算模式
	if (m_sysMode == MODE_RT) {				//射线追踪模式
		_initRTSystemMemory();				//初始化射线追踪系统内存
	}
	else if(m_sysMode == MODE_LBS) {		//定位模式
		_initLBSSystemMemory();				//初始化定位系统内存
	}
	PreProcess();				//初始化程序参数
	return true;
}

void System::Render()
{
	SYSTEM_MODE systemMode = m_simConfig.m_systemMode;
	HARDWAREMODE hardwareMode = m_simConfig.m_raytracingConfig.m_hardwareMode;
	RAYLAUNCHMODE rayLaunchMode = m_simConfig.m_raytracingConfig.m_rayLaunchMode;
	uint64_t& rayNum = m_simConfig.m_raytracingConfig.m_rayNum;
	RayLaunch(rayLaunchMode, rayNum);
	if (systemMode == MODE_RT) {
		RayTracing(hardwareMode);
		PathBuilder(hardwareMode);
	}
	else if (systemMode == MODE_LBS) {
		RayTracingLBS(hardwareMode);
		TreeNodeGenerator(hardwareMode);
	}
	
	LOG_INFO << "完成RT" << ENDL;
}

void System::RayLaunch(const RAYLAUNCHMODE& mode, uint64_t rayNum)
{
	if (m_sysMode == MODE_RT) {
		size_t txNum = m_scene->m_transmitters.size();						/** @brief	tx数量	*/
		if (mode == SINGLE_DIRECTION) {
			double rayHalfTheta = QUARTER_PI / 2.0;								//默认定义单根射线半张角为 PI/8=22.5°
			for (int i = 0; i < txNum; ++i) {
				Point2D curTxPosition = m_scene->m_transmitters[i]->GetPosition2D();
				Point2D curRxPosition = m_scene->m_receivers[0]->GetPosition2D();
				Vector2D dir = (curRxPosition - curTxPosition).Normalize();
				RayLaunch_SingleDirection(rayNum, curTxPosition, dir, rayHalfTheta, m_rtInitRays[i]);
			}
		}
		else if (mode == UNIFORM_ICOSAHEDRON) {
			for (int i = 0; i < txNum; ++i) {
				Point2D curTxPosition = m_scene->m_transmitters[i]->GetPosition2D();
				RayLaunch_Uniform(rayNum, curTxPosition, m_rtInitRays[i]);
			}
		}
	}
	else if (m_sysMode == MODE_LBS) {
		LOCALIZATION_METHOD localizeMethod = m_simConfig.m_lbsConfig.m_lbsMethod;
		int sensorNum = static_cast<int>(m_scene->m_sensors.size());
		for (int i = 0; i < sensorNum; ++i) {
			const Sensor* curSensor = m_scene->m_sensors[i];
			const Point2D& sPosition = curSensor->GetPosition2D();
			RayLaunch_Uniform(rayNum, sPosition, m_rtInitRays[i]);							//增加项-在定位模式中增加从传感器中的射线追踪模块
			RayLaunch_BySensor(localizeMethod, rayNum, curSensor, m_lbsInitRays[i]);
		}
	}
	return;
}

void System::RayTracing(const HARDWAREMODE hardwareMode)
{
	const LimitInfo& limitinfo = m_simConfig.m_raytracingConfig.m_limitInfo;
	bool raySplitFlag = m_simConfig.m_raytracingConfig.m_raySplitFlag;
	RtLbsType raySplitRadius = m_simConfig.m_raytracingConfig.m_raySplitRadius;
	

	if (hardwareMode == CPU_SINGLETHREAD) {
		RayTracing_CPUSingleThread(m_sysMode, m_rtInitRays, limitinfo, raySplitFlag, raySplitRadius, m_scene, m_rtTreeRoot);
	}
	else if (hardwareMode == CPU_MULTITHREAD) {
		uint16_t threadNum = m_simConfig.m_raytracingConfig.m_cpuThreadNum;
		RayTracing_CPUMultiThread(m_sysMode, m_rtInitRays, limitinfo, raySplitFlag, raySplitRadius, m_scene, threadNum, m_rtTreeRoot);
	}
	else if (hardwareMode == GPU_MULTITHREAD) {
		m_scene->ConvertToGPUHostScene();																							//在使用GPU进行计算时需要将场景转换为GPU场景
		RayTracing_GPUMultiThread(m_rtInitRays, limitinfo, raySplitFlag, raySplitRadius, m_scene, m_rtGPUPathNodes);
		
	}
	LOG_INFO << "raytracing: has already traced all the rays." << ENDL;
}

void System::RayTracingLBS(const HARDWAREMODE hardwareMode)
{
	HARDWAREMODE curHardWareMode = hardwareMode;
	if (hardwareMode == GPU_MULTITHREAD) {								//定位算法中初始的射线追踪禁止采用GPU加速
		curHardWareMode = CPU_MULTITHREAD;
	}

	//定位射线追踪模块
	const LimitInfo& limitinfo = m_simConfig.m_raytracingConfig.m_limitInfo;
	bool raySplitFlag = m_simConfig.m_raytracingConfig.m_raySplitFlag;
	RtLbsType raySplitRadius = m_simConfig.m_raytracingConfig.m_raySplitRadius;
	if (curHardWareMode == CPU_SINGLETHREAD) {
		RayTracingLBS_CPUSingleThread(m_sysMode, m_lbsInitRays, limitinfo, raySplitFlag, raySplitRadius, m_scene, m_lbsTreeRoot);
	}
	else if (curHardWareMode == CPU_MULTITHREAD) {				
		uint16_t threadNum = m_simConfig.m_raytracingConfig.m_cpuThreadNum;
		RayTracingLBS_CPUMultiThread(m_sysMode, m_lbsInitRays, limitinfo, raySplitFlag, raySplitRadius, m_scene, threadNum, m_lbsTreeRoot);
	}
	//else if (hardwareMode == GPU_MULTITHREAD) {
	//	m_scene->ConvertToGPUHostScene();																							//在使用GPU进行计算时需要将场景转换为GPU场景
	//	RayTracing_GPUMultiThreadWithNode(m_lbsInitRays, limitinfo, raySplitFlag, raySplitRadius, m_scene, m_gpuTreeNodes);
	//}

	//增加-在LBS的同时进行射线追踪，用于计算路径做校正处理
	RayTracing(curHardWareMode);

	
	LOG_INFO << "raytracing: has already traced all the rays." << ENDL;
}

void System::PathBuilder(const HARDWAREMODE mode)
{
	////debug
	//PathBuilder_DEBUG(m_aroot, m_scene);

	HARDWAREMODE hardwareMode = m_simConfig.m_raytracingConfig.m_hardwareMode;														/** @brief	硬件加速模式	*/
	if (m_scene->m_receivers.size() < m_simConfig.m_raytracingConfig.m_cpuThreadNum * 2) {											//不满足多核心共同加速的条件，设置为单核心结果输出
		if (hardwareMode == CPU_MULTITHREAD)
			hardwareMode = CPU_SINGLETHREAD;
	}
	size_t rayNum = m_simConfig.m_raytracingConfig.m_rayNum;
	RtLbsType splitRadius = m_simConfig.m_raytracingConfig.m_raySplitRadius;
	if (hardwareMode == CPU_SINGLETHREAD) {
		PathBuilder_CPUSingleThread(m_rtTreeRoot, m_scene, splitRadius, m_result);
	}
	else if (hardwareMode == CPU_MULTITHREAD) {
		uint16_t threadNum = m_simConfig.m_raytracingConfig.m_cpuThreadNum;
		PathBuilder_CPUMultiThread(m_rtTreeRoot, m_scene, splitRadius, threadNum, m_result);
	}
	else if (hardwareMode == GPU_MULTITHREAD) {
		PathBuilder_GPUMultiThread(m_rtGPUPathNodes, m_scene, m_result);
	}
	return;
}

void System::TreeNodeGenerator(const HARDWAREMODE hardwareNode)
{
	if (hardwareNode == CPU_SINGLETHREAD) {
		TreeNodeGenerator_CPUSingleThread(m_lbsTreeRoot, m_scene, m_result);
	}
	else if (hardwareNode == CPU_MULTITHREAD) {
		int16_t threadNum = m_simConfig.m_raytracingConfig.m_cpuThreadNum;
		TreeNodeGenerator_CPUMultiThread(m_lbsTreeRoot, m_scene, threadNum, m_result);
	}
	else if (hardwareNode == GPU_MULTITHREAD) {
		TreeNodeGenerator_GPUMultiThread(m_gpuTreeNodes, m_scene, m_result);
	}
}

void System::PostProcessing()
{
	//计算基本信息
	const FrequencyConfig& freqConfig = m_simConfig.m_frequencyConfig;
	if (m_sysMode == MODE_RT) {
		
		const OutputConfig& outputConfig = m_simConfig.m_outputConfig;
		m_result.CalculateResult_RT_SensorData(freqConfig, &m_scene->m_materialLibrary, m_tranFunctionData, outputConfig);
		
	}
	else if(m_sysMode == MODE_LBS) {
		HARDWAREMODE hardwareMode = m_simConfig.m_lbsConfig.m_hardWareMode;								/** @brief	定位算法工作模式	*/
		LOCALIZATION_MODE lbsMode = m_simConfig.m_lbsConfig.m_lbsMode;
		LOCALIZATION_METHOD lbsMethod = m_simConfig.m_lbsConfig.m_lbsMethod;
		uint16_t threadNum = m_simConfig.m_lbsConfig.m_threadNum;
		RtLbsType gsPairClusterThreshold = m_simConfig.m_lbsConfig.m_gsPairClusterThreshold;
		RtLbsType splitRadius = m_simConfig.m_raytracingConfig.m_raySplitRadius;
		
		
		//增加输入射线追踪树结构
		if (lbsMode == LBS_MODE_MPSTSD) {
			m_result.CalculateResult_LBS_AOA_MPSTSD(m_rtTreeRoot, m_scene, splitRadius, lbsMethod, freqConfig, m_tranFunctionData);
		}
		else if (lbsMode == LBS_MODE_SPSTMD) {
			m_result.CalculateResult_LBS_AOA_SPSTMD(hardwareMode, m_rtTreeRoot, m_scene, splitRadius, lbsMethod, threadNum, gsPairClusterThreshold, freqConfig, m_tranFunctionData);
		}
	}
	
}

void System::OutputResults()
{
	//输出结果至文件
	m_result.OutputResult(m_sysMode, m_simConfig.m_outputConfig);
}

void System::PreProcess()
{
	if (m_simConfig.m_raytracingConfig.m_hardwareMode == GPU_MULTITHREAD) {			//若硬件配置为GPU配置，则采用距离场作为加速方法
		m_scene->PreProcess(ACCEL_SDF);
	}
	else {
		m_scene->PreProcess(m_simConfig.m_raytracingConfig.m_sceneAccelType);		//预处理环境面元
	}
}


bool System::_initRTSystemMemory()
{
	try {
		unsigned m_txNum = static_cast<unsigned>(m_scene->m_transmitters.size());
		unsigned m_rxNum = static_cast<unsigned>(m_scene->m_receivers.size());
		m_rtTreeRoot.resize(m_txNum);							//射线追踪结构树根
		m_rtInitRays.resize(m_txNum);							//射线集合
		for (unsigned i = 0; i < m_txNum; ++i) {
			const Point3D& txPosition = m_scene->m_transmitters[i]->m_position;
			Point2D txPosition2D(txPosition.x, txPosition.y);
			PathNode rootNode(m_simConfig.m_raytracingConfig.m_limitInfo, NODE_ROOT, txPosition2D);		//根节点赋值
			m_rtTreeRoot[i] = new RayTreeNode(rootNode);
			m_rtInitRays[i].resize(m_simConfig.m_raytracingConfig.m_rayNum);
		}
		m_result.Init(m_scene->m_transmitters, m_scene->m_receivers);										//初始化结果
		LOG_INFO << "system: allocate memory success" << ENDL;
		return true;
	}
	catch (std::exception& e) {
		LOG_ERROR << e.what() << ENDL;
		return false;
	}
}

bool System::_initLBSSystemMemory()
{
	try {
		unsigned rxNum = static_cast<unsigned>(m_scene->m_receivers.size());
		unsigned sensorNum = static_cast<unsigned>(m_scene->m_sensors.size());
		m_rtTreeRoot.resize(sensorNum);							//射线追踪结构树根
		m_lbsTreeRoot.resize(sensorNum);						//定位结构射线追踪树根
		m_rtInitRays.resize(sensorNum);							//定位模式伴随的射线追踪初始射线内存分配
		m_lbsInitRays.resize(sensorNum);						//定位模式射线内存分配
		for (unsigned i = 0; i < sensorNum; ++i) {
			const Point2D& sensorPosition = m_scene->m_sensors[i]->GetPosition2D();
			PathNode rootNode(m_simConfig.m_raytracingConfig.m_limitInfo, NODE_ROOT, sensorPosition);		//根节点赋值
			m_rtTreeRoot[i] = new RayTreeNode(rootNode);
			m_lbsTreeRoot[i] = new RayTreeNode(rootNode);
		}
		m_result.Init(m_scene->m_sensors);													//初始化结果
	}
	catch (std::exception& e) {
		LOG_ERROR << e.what() << ENDL;
		return false;
	}
	return true;
}