#ifndef RTLBS_RESULT
#define RTLBS_RESULT
#include "math/point2d.h"
#include "radiowave/raypath/raypath.h"
#include "utility/define.h"
#include "raytracingresult.h"
#include "equipment/transceiver/transmitter.h"
#include "equipment/transceiver/receiver.h"
#include "equipment/sensor/sensor.h"
#include "configuration/outputconfig.h"
#include "configuration/frequencyconfig.h"
#include "material/materiallibrary.h"
#include "math/complex.h"
#include "equipment/sensor/sensordata.h"
#include "equipment/sensor/sensordatacollection.h"
#include "equipment/sensor/sensordataprocessing.h"
#include "configuration/sensor/sensorcollectionconfig.h"
#include "configuration/localization/lbserrorconfig.h"
#include "localization/localizationfunction/crlb.h"
#include "localization/localizationfunction/gdop.h"



class Result {

public:
	unsigned m_txNum;														/** @brief	发射机数量	*/
	unsigned m_rxNum;														/** @brief	接收机数量	*/
	unsigned m_sensorNum;													/** @brief	传感器数量	*/
	std::vector<Transmitter*> m_transmitters;								/** @brief	发射机集合	*/
	std::vector<Receiver*> m_receivers;										/** @brief	接收机集合	*/
	std::vector<RaytracingResult> m_raytracingResult;						/** @brief	射线追踪仿真结果	*/

	std::vector<SensorDataCollection> m_sensorDataSPSTMD;					/** @brief	单站单源多数据定位, 接收数量为1	  支持定位类型：TOA、TDOA、AOA*/
	std::vector<SensorDataCollection> m_sensorDataMPSTSD;					/** @brief	多站单源单数据定位, 数量为发射机数量*接收机的数量 支持定位类型：TOA、TDOA、AOA*/
	std::vector<SensorDataCollection> m_sensorDataSPMTMD;					/** @brief	单站多源多数据定位, 数量为接收机的数量	支持定位类型 AOA*/
	std::vector<SensorDataCollection> m_sensorDataMPMTMD;					/** @brief	多站多源多数据定位, 数量为接收机的数量	支持定位类型 AOA*/
private:
	mutable std::string m_directory;										/** @brief	文件存储路径	*/
	SensorCollectionConfig m_sensorCollectionConfig;						/** @brief	RT模式下输出传感器数据的同时输出的传感器配置文件	*/


public:
	Result();
	~Result();
	void Init(const std::vector<Transmitter*>& transmitters, const std::vector<Receiver*> receivers);										//初始化结果-射线追踪
	void OutputResult(const OutputConfig& config);																					//输出计算结果
	void CalculateResult_RT_SensorData(const OutputConfig& outputConfig, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData);																	//计算射线追踪结果，用于模拟传感器数据

private:
	void OutputVectorEField() const;																										//输出矢量电磁场
	void OutputScalarEField() const;																										//输出标量电磁场
	void OutputVectorPower() const;																											//输出矢量功率
	void OutputScalarPower() const;																											//输出标量功率
	void OutputLoss() const;																												//输出损耗
	void OutputRayPath();																												//输出射线路径
	void OutputPDP() const;																													//输出功率时延信息
	void OutputCFR() const;																													//输出信道频率响应信息
	void OutputCIR() const;																													//输出信道冲激相应信息
	void OutputAOA() const;																													//输出到达角信息
	void OutputAOD() const;																													//输出离开角信息
	void OutputSpreadProfile() const;																										//输出扩展信息
	void OutputSensorDataSPSTMD() const;																									//输出单站单源多数据定位传感器仿真数据
	void OutputSensorDataMPSTSD() const;																									//输出多站单源单数据定位传感器仿真数据
	void OutputSensorDataSPMTMD() const;																									//输出单站多源多数据定位传感器仿真数据
	void OutputSensorDataMPMTMD() const;																									//输出多站多源多数据定位传感器仿真数据
	void OutputGeneralSourceForCRLB() const;																								//输出广义源信息，用于计算CRLB
	void OutputMultiStationCRLB(const LBSErrorConfig& config) const;																		//计算并输出多站CRLB
	void OutputMultiStationGDOP(const LBSErrorConfig& config) const;																		//计算并输出多站GDOP
	void OutputSingleStationCRLB(const LBSErrorConfig& config) const;																		//计算并输出单站CRLB
	void OutputSingleStationGDOP(const LBSErrorConfig& config) const;																		//计算并输出单站GDOP

	//论文中需要输出的内容
	void OutputLocationRange() const;																										//输出定位范围
	
};

#endif
