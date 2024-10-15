#ifndef RTLBS_RAYTRACINGRESULT
#define RTLBS_RAYTRACINGRESULT


#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "equipment/transceiver/transmitter.h"
#include "equipment/transceiver/receiver.h"
#include "equipment/sensor/sensor.h"
#include "radiowave/raypath/raypath3d.h"
#include "radiowave/raypath/terraindiffractionpath.h"
#include "pathinfo.h"
#include "pathinfocluster.h"
#include "math/complex.h"
#include "material/material.h"
#include "material/materiallibrary.h"
#include "equipment/sensor/sensordata.h"



class RaytracingResult {
public:
	bool m_isValid;											/** @brief	结果是否有效	*/
	int m_pathNum;											/** @brief	路径数量	*/
	int m_freqNum;											/** @brief	频率数量	*/
	Transmitter* m_transmitter;								/** @brief	发射机	*/
	Receiver* m_receiver;									/** @brief	接收机	*/
	std::vector<RtLbsType> m_freqs;							/** @brief	计算的频点	*/
	std::vector<RayPath3D*> m_commonPaths;					/** @brief	常规路径	*/
	TerrainDiffractionPath* m_terrainDiffPath;				/** @brief	地形绕射路径	*/

	std::vector<PathInfo> m_multipathInfo;					/** @brief	多径信息	*/
	std::vector<Complex> m_totalVectorEField;				/** @brief	合成矢量电场	*/
	std::vector<RtLbsType> m_totalScalarEField;				/** @brief	合成标量电场	*/
	std::vector<RtLbsType> m_vectorPower;					/** @brief	矢量接收功率	*/
	std::vector<RtLbsType> m_scalarPower;					/** @brief	标量接收功率	*/
	std::vector<RtLbsType> m_loss;							/** @brief	损耗	*/
	std::vector<Complex> m_magnitudesCFR;					/** @brief	信道频率响应CFR	*/
	std::vector<Complex> m_magnitudesCIR;					/** @brief	信道冲激响应CIR	*/
	RtLbsType m_rmsDelaySpread;								/** @brief	均方根时延扩展	*/
	RtLbsType m_rmsAngularSpread;							/** @brief	均方根角度扩展	*/

public:
	RaytracingResult();
	RaytracingResult(const RaytracingResult& result);
	~RaytracingResult();
	RaytracingResult& operator = (const RaytracingResult& result);


	void SetRayPath(std::vector<RayPath3D*>& paths);		//设置路径信息 常规路径
	void SetRayPath(TerrainDiffractionPath* path);			//设置路径信息 地形绕射
	void ReleaseAllRayPath();								//释放所包含的路径信息
	void CalculateBaseInfo(std::vector<RtLbsType>& freqs, const std::vector<Complex>& tranFunctionData);								//计算基本信息-射线追踪模式
	
	void CalculateBaseInfo(const Sensor* sensor, std::vector<RtLbsType>& freqs, const std::vector<Complex>& tranFunctionData, const AntennaLibrary* antLibrary);		//计算基本信息-定位模式中的伴随射线追踪
	void GetAllSensorData_AOA2D(SensorDataCollection& collection, RtLbsType threshold, RtLbsType sparseFactor) const;						//获取所有的传感器数据,适用于AOA2D定位
	void GetMaxPowerSensorData_AOA2D(SensorDataCollection& collection, RtLbsType threshold) const;											//获取最大功率的传感器数据,适用于AOA2D定位
	void GetAllSensorData_AOA3D(SensorDataCollection& collection, RtLbsType threshold, RtLbsType sparseFactor) const;						//获取所有的传感器数据,适用于AOA3D定位
	void GetMaxPowerSensorData_AOA3D(SensorDataCollection& collection, RtLbsType threshold) const;											//获取最大功率的传感器数据,适用于AOA3D定位
	void GetAllSensorData_Delay(SensorDataCollection& collection, RtLbsType threshold, RtLbsType sparseFactor) const;						//获取所有的传感器数据,适用于时延型定位
	void GetMaxPowerSensorData_Delay(SensorDataCollection& collection, RtLbsType threshold) const;											//获取最大功率的传感器数据,适用于时延型定位
	void OutputVectorEField(std::ofstream& stream) const;																					//输出矢量场
	void OutputScalarEField(std::ofstream& stream) const;																					//输出标量场
	void OutputVectorPower(std::ofstream& stream) const;																					//输出矢量功率信息
	void OutputScalarPower(std::ofstream& stream) const;																					//输出标量功率信息
	void OutputLoss(std::ofstream& stream) const;																							//输出损耗信息
	void OutputRayPath(std::ofstream& stream) const;																						//输出多径信息
	void OutputPDP(std::ofstream& stream) const;																							//输出PDP信息
	void OutputCFR(std::ofstream& stream) const;																							//输出CFR信息
	void OutputCIR(std::ofstream& stream) const;																							//输出CIR信息
	void OutputAOA(std::ofstream& stream) const;																							//输出AOA信息
	void OutputAOD(std::ofstream& stream) const;																							//输出AOD信息
	void OutputSpreadProfile(std::ofstream& stream) const;																					//输出扩展相关信息，包括时延扩展和角度扩展
	void OutputGeneralSourceForCRLB(std::ofstream& stream) const;																			//输出广义源信息，为了计算CRLB

private:
	RtLbsType CalculateMeanArrivedDelay() const;																							//计算平均到达时延
	RtLbsType CalculateMeanArrivedAngle() const;																							//计算平均到达角度
	RtLbsType CalculateRMSDelaySpread() const;																								//计算均方根时延扩展
	RtLbsType CalculateRMSAngularSpread() const;																							//计算均方根角度扩展

};

#endif
