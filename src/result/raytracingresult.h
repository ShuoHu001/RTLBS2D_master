#ifndef RTLBS_RAYTRACINGRESULT
#define RTLBS_RAYTRACINGRESULT


#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "equipment/transmitter.h"
#include "equipment/receiver.h"
#include "equipment/sensor.h"
#include "tree/raypath3d.h"
#include "tree/terraindiffractionpath.h"
#include "pathinfo.h"
#include "math/complex.h"
#include "material/material.h"
#include "material/materiallibrary.h"
#include "equipment/sensordata.h"

class RaytracingResult {
public:
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


public:
	RaytracingResult();
	RaytracingResult(const RaytracingResult& result);
	~RaytracingResult();
	RaytracingResult& operator = (const RaytracingResult& result);


	void SetRayPath(std::vector<RayPath3D*>& paths);		//设置路径信息 常规路径
	void SetRayPath(TerrainDiffractionPath* path);			//设置路径信息 地形绕射
	void CalculateBaseInfo(std::vector<RtLbsType>& freqs, const MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction);								//计算基本信息-射线追踪模式
	void CalculateBaseInfo(const Sensor* sensor, std::vector<RtLbsType>& freqs, const AntennaLibrary* antLibrary, const MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction);		//计算基本信息-定位模式中的伴随射线追踪
	void GetAllSensorData(SensorDataCollection& collection) const;			//获取所有的传感器数据（转化）
	void GetAllSensorData(SensorDataCollection* collection) const;			//获取所有的传感器数据		指针型
	void GetMaxPowerSensorData(SensorDataCollection& collection) const;		//获取最大功率的传感器数据 （转化）
	void GetMaxPowerSensorData(SensorDataCollection* collection) const;		//获取最大功率的传感器数据 指针型
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
};

#endif
