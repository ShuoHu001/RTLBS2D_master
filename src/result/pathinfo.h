#ifndef RTLBS_PATHINFO
#define RTLBS_PATHINFO

#include "rtlbs.h"
#include "utility/define.h"
#include "math/complex.h"
#include "utility/enum.h"
#include "tree/raypath3d.h"
#include "tree/terraindiffractionpath.h"
#include "equipment/transmitter.h"
#include "equipment/receiver.h"
#include "equipment/sensordata.h"
#include "equipment/sensor.h"


class PathInfo {
public:
	RAYPATHTYPE m_rayPathType;								/** @brief	路径类型	*/
	RtLbsType m_freq;										/** @brief	频率	  单位：Hz*/
	RtLbsType m_power;										/** @brief	功率	 单位：dBm*/
	RtLbsType m_powerRatio;									/** @brief	能量占比	*/
	RtLbsType m_scalarEField;								/** @brief	标量场强	 单位：V/m*/
	Complex m_vectorEField;									/** @brief	矢量场强	 单位：V/m*/
	Complex m_magnitude;									/** @brief	复数场强，用于推算CFR、CIR等	*/
	RtLbsType m_timeDelay;									/** @brief	传播时延	单位：ns*/
	RtLbsType m_phaseOffset;								/** @brief	相位	偏移 单位：弧度*/
	RtLbsType m_aoDPhi;										/** @brief	离开角(水平)	 单位：弧度*/
	RtLbsType m_aoDTheta;									/** @brief	离开角(垂直)	 单位：弧度*/
	RtLbsType m_aoAPhi;										/** @brief	到达角(水平)	 单位：弧度*/
	RtLbsType m_aoATheta;									/** @brief	到达角(垂直)	 单位：弧度*/

private:
	RayPath3D* m_rayPath;									/** @brief	常规路径	*/
	TerrainDiffractionPath* m_terrainDiffractionPath;		/** @brief	地形绕射路径	*/

public:
	PathInfo();
	PathInfo(const PathInfo& pathInfo);
	~PathInfo();
	PathInfo& operator = (const PathInfo& pathInfo);
	void SetRayPath(RayPath3D* path);
	void SetRayPath(TerrainDiffractionPath* path);
	void CalculateBaseInfo(RtLbsType freq, const MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction, Transmitter* transmitter, Receiver* receiver);					//计算基本信息-射线追踪模式
	void CalculateBaseInfo(RtLbsType power, RtLbsType freq, const AntennaLibrary* antLibrary, const MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction, const Sensor* sensor);		//计算基本信息-定位模式中的盘岁射线追踪
	void Convert2SensorData(SensorData& data) const;				//转换为传感器数据
};

//按照PathInfo中的能量的大小进行排序-逆方向排序
inline bool ComparedByPower_PathInfo(const PathInfo& info1, const PathInfo& info2) {
	return info1.m_power > info2.m_power;
}

//按照PathInfo中的时间大小进行排序-逆方向排序
inline bool ComparedByDelay_PathInfo(const PathInfo& info1, const PathInfo& info2) {
	return info1.m_timeDelay > info2.m_timeDelay;
}

//按照PathInfo中的到达角的phi值进行排序-逆方向排序
inline bool ComparedByAOAPhi_PathInfo(const PathInfo& info1, const PathInfo& info2) {
	return info1.m_aoAPhi > info2.m_aoAPhi;
}

//按照PathInfo中的离开角的phi值进行排序-逆方向排序
inline bool ComparedByAODPhi_PathInfo(const PathInfo& info1, const PathInfo& info2) {
	return info1.m_aoDPhi > info2.m_aoDPhi;
}

//按照PathInfo中的到达角的theta值进行排序-逆方向排序
inline bool ComparedByAOATheta_PathInfo(const PathInfo& info1, const PathInfo& info2) {
	return info1.m_aoATheta > info2.m_aoATheta;
}

//按照PathInfo中的离开角的theta值进行排序-逆方向排序
inline bool ComparedByAODTheta_PathInfo(const PathInfo& info1, const PathInfo& info2) {
	return info1.m_aoDTheta > info2.m_aoDTheta;
}


#endif
