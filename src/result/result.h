#ifndef RTLBS_RESULT
#define RTLBS_RESULT
#include "geometry/point2d.h"
#include "tree/raypath.h"
#include "utility/define.h"
#include "raytracingresult.h"
#include "equipment/transmitter.h"
#include "equipment/receiver.h"
#include "equipment/sensor.h"
#include "configuration/outputconfig.h"
#include "configuration/frequencyconfig.h"
#include "material/materiallibrary.h"
#include "math/complex.h"
#include "result/lbsresult.h"
#include "equipment/sensordata.h"
#include "equipment/sensordatacollection.h"
#include "equipment/sensordataprocessing.h"
#include "configuration/sensor/sensorcollectionconfig.h"
#include "localization/generalsource.h"
#include "localization/gspair.h"
#include "physical/pathbuilder.h"
#include "localization/aoa/aoasolver.h"
#include "physical/gpu/pathbuildergpu.h"
#include "localization/gspaircluster.h"
#include "localization/weightfactor.h"



class Result {

public:
	unsigned m_txNum;														/** @brief	发射机数量	*/
	unsigned m_rxNum;														/** @brief	接收机数量	*/
	unsigned m_sensorNum;													/** @brief	传感器数量	*/
	std::vector<RaytracingResult> m_raytracingResult;						/** @brief	射线追踪仿真结果	*/
	std::vector<LBSResultGS> m_lbsGSResult;									/** @brief	定位结果 广义源	*/

	std::vector<SensorDataCollection> m_sensorDataSPSTMD;					/** @brief	单站单源多数据定位, 接收数量为1	  支持定位类型：TOA、TDOA、AOA*/
	std::vector<SensorDataCollection> m_sensorDataMPSTSD;					/** @brief	多站单源单数据定位, 数量为发射机数量*接收机的数量 支持定位类型：TOA、TDOA、AOA*/
	std::vector<SensorDataCollection> m_sensorDataSPMTMD;					/** @brief	单站多源多数据定位, 数量为接收机的数量	支持定位类型 AOA*/
	std::vector<SensorDataCollection> m_sensorDataMPMTMD;					/** @brief	多站多源多数据定位, 数量为接收机的数量	支持定位类型 AOA*/
private:
	mutable std::string m_directory;										/** @brief	文件存储路径	*/
	std::vector<GeneralSource*> m_allGeneralSource;							/** @brief	所有的广义源	*/
	SensorCollectionConfig m_sensorCollectionConfig;						/** @brief	RT模式下输出传感器数据的同时输出的传感器配置文件	*/
	AOASolver m_aoaSolver;													/** @brief	AOA定位求解器	*/


public:
	Result();
	~Result();
	void Init(const std::vector<Transmitter*>& transmitters, const std::vector<Receiver*> receivers);										//初始化结果-射线追踪
	void Init(const std::vector<Sensor*>& sensors, const std::vector<Receiver*>& receivers);												//初始化结果-无源定位
	void Init(const std::vector<Sensor*>& sensors);																							//初始化结果-无源定位GS
	void OutputResult(SYSTEM_MODE systemMode, const OutputConfig& config) const;															//输出计算结果
	void CalculateResult_RT_SensorData(const FrequencyConfig& freqConfig, MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction, const OutputConfig& outputConfig);														//计算射线追踪结果
	Point2D CalculateResult_LBS_AOA_MPSTSD(HARDWAREMODE hardwareMode, const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, const WeightFactor& weightFactor, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunction);
	Point2D CalculateResult_LBS_AOA_SPSTMD(HARDWAREMODE hardwareMode, const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, const WeightFactor& weightFactor, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunction);		//计算结果，GS定位方法AOASPSTMD定位模式
	void CalculateResult_LBS_TDOA_MPSTSD(HARDWAREMODE hardwareMode, const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, const WeightFactor& weightFactor, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunction);		//计算结果，GS定位方法TDOAMPSTSD定位模式
	void CalculateResult_LBS_TDOA_SPSTMD(HARDWAREMODE hardwareMode, const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, const WeightFactor& weightFactor, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunction);		//计算结果，GS定位方法TDOASPSTMD定位模式
	std::vector<GeneralSource*> GetGeneralSource() const;																					//获得广义源
	Point2D LocalizationSolver(const Point2D& initPoint);																												//定位求解器

private:
	void OutputVectorEField() const;																										//输出矢量电磁场
	void OutputScalarEField() const;																										//输出标量电磁场
	void OutputVectorPower() const;																											//输出矢量功率
	void OutputScalarPower() const;																											//输出标量功率
	void OutputLoss() const;																												//输出损耗
	void OutputRayPath() const;																												//输出射线路径
	void OutputPDP() const;																													//输出功率时延信息
	void OutputCFR() const;																													//输出信道频率响应信息
	void OutputCIR() const;																													//输出信道冲激相应信息
	void OutputAOA() const;																													//输出到达角信息
	void OutputAOD() const;																													//输出离开角信息
	void OutputSensorDataSPSTMD() const;																									//输出单站单源多数据定位传感器仿真数据
	void OutputSensorDataMPSTSD() const;																									//输出多站单源单数据定位传感器仿真数据
	void OutputSensorDataSPMTMD() const;																									//输出单站多源多数据定位传感器仿真数据
	void OutputSensorDataMPMTMD() const;																									//输出多站多源多数据定位传感器仿真数据
	void OutputGeneralSource() const;																										//输出广义源信息
	
};

#endif
