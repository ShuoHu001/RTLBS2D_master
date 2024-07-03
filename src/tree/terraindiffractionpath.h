#ifndef RTLBS_TERRAINDIFFRACTIONPATH
#define RTLBS_TERRAINDIFFRACTIONPATH

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/point3d.h"
#include "geometry/terrain/terrainridge.h"
#include "raypath.h"
#include "antenna/antenna.h"
#include "material/materiallibrary.h"
#include "physical/radiowave/polarization3d.h"



//此头文件为地形绕射路径对象描述


//地形路径节点坐标
class TerrainPathNode {
public:
	Point3D m_point;					/** @brief	点坐标	*/
	RtLbsType m_clearanceHeight;		/** @brief	净空高度	*/
	RtLbsType m_s1;						/** @brief	距离起点的高度	*/
	RtLbsType m_s2;						/** @brief	距离终点的高度	*/
	const TerrainRidge* m_ridge;		/** @brief	每个节点对应的峰峦	*/

public:
	TerrainPathNode();
	TerrainPathNode(const Point3D& edgePoint);																		//初始化-收发节点（非峰值节点）
	TerrainPathNode(const TerrainRidge* ridge);
	TerrainPathNode(Point3D& p, RtLbsType& clearanceHeight, RtLbsType& sPs, RtLbsType& sPe);
	TerrainPathNode(const TerrainRidge* prevRidge, const TerrainRidge* curRidge, const TerrainRidge* nextRidge);
	TerrainPathNode(const TerrainPathNode& node);
	~TerrainPathNode();
	TerrainPathNode& operator = (const TerrainPathNode& node);
	bool operator == (const TerrainPathNode& node) const;
	bool operator != (const TerrainPathNode& node) const;
};


//地形绕射路径
class TerrainDiffractionPath {
public:
	std::vector<TerrainPathNode*> m_nodes;				/** @brief	地形绕射点坐标	*/
	TERRAINDIFFRACTIONMODE m_terrainDiffractionMode;	/** @brief	地形绕射计算模式	*/

public:
	TerrainDiffractionPath();
	~TerrainDiffractionPath();
	Complex CalculateDiffractionEField_PICQUENARD(RtLbsType power, RtLbsType freq, const Antenna* txAntenna, const Antenna* rxAntenna) const;			//计算地形绕射损耗 Picquenard 方法
	Complex CalculateDiffractionEField_EPSTEIN(RtLbsType power, RtLbsType freq, const Antenna* txAntenna, const Antenna* rxAntenna) const;				//计算地形绕射损耗 EPSTEIN方法
	Complex CalculateDiffractionEField_UTD(RtLbsType power, RtLbsType freq, const Antenna* txAntenna, const Antenna* rxAntenna) const;					//计算地形绕射损耗 UTD方法
	Complex CalculateTerrainDiffractionEField(RtLbsType power, RtLbsType freq, const Antenna* txAntenna, const Antenna* rxAntenna) const;										//计算绕射损耗
	RtLbsType GetPropagationTime() const;							//计算路径传播时间
	RtLbsType GetPropagationLength() const;							//计算路径传播距离
	RtLbsType GetPhaseOffset(RtLbsType freq) const;					//计算相位偏移
	RtLbsType GetAngleofDeparture_Phi() const;						//计算路径离开方位角
	RtLbsType GetAngleofDeparture_Theta() const;					//计算路径离开俯仰角
	RtLbsType GetAngleofArrival_Phi() const;						//计算路径到达付俯仰角
	RtLbsType GetAngleofArrival_Theta() const;						//计算路径到达俯仰角
	RtLbsType CalDopplerShift(RtLbsType freq, const Vector3D& txVelocity, const Vector3D& rxVelocity);		//计算地形路径的多普勒频移
	void OuputRaypath(std::ofstream& stream) const;					//输出多径信息
	void RectifySParameters();										//更新S参数信息
};


#endif
