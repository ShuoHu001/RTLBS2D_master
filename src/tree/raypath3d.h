#ifndef RTLBS_RAYPATH3D
#define RTLBS_RAYPATH3D

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/point3d.h"
#include "geometry/vector3d.h"
#include "physical/radiowave/polarization3d.h"
#include "material/materiallibrary.h"
#include "material/material.h"
#include "antenna/antenna.h"
#include "raypath.h"
#include "pathnode3d.h"

class RayPathGPU;

class RayPath3D {
public:
	std::vector<PathNode3D*> m_nodes;						/** @brief	路径节点集合	*/
	RtLbsType m_propagationLength;							/** @brief	传播长度	*/
	RtLbsType m_energyRatio;								/** @brief	能量比例	*/
	bool m_containRefract;									/** @brief	是否包含透射	*/
	bool m_containSplitNode;								/** @brief	是否包含分裂节点	*/
	bool m_isValid;											/** @brief	是否是有效路径	*/
	RAYPATHTYPE m_type;										/** @brief	路径类型	*/
	int m_angularSpectrumCategoryId;						/** @brief	路径所在的传感器角度谱种类ID	*/

public:
	RayPath3D();																									//默认构造函数
	RayPath3D(const RayPath& path, const Point3D& tx, const Point3D& rx);											//二维射线路径构造函数
	RayPath3D(const RayPath& path, RtLbsType height);																//二维射线路径构造函数-用于定位中的电磁计算
	RayPath3D(const RayPathGPU& path, const Point3D& tx, const Point3D& rx, const std::vector<Segment2D*>& segments, const std::vector<Wedge2D*>& wedges);			//GPU二维射线路径构造函数
	RayPath3D(const RayPath3D& path);																				//赋值构造函数
	~RayPath3D();																									//析构函数
	RayPath3D& operator = (const RayPath3D& path);																	//赋值操作符
	void ReverseRayPath();																							//将路径的头和尾互换，逆转路径

	void ConvertByRayPath(const RayPath& path, const Point3D& tx, const Point3D& rx);								//将二维路径转换为三维路径
	void Union(PathNode3D* node);																					//扩展路径节点
	Polarization3D CalculateStrengthField3D(RtLbsType power, RtLbsType freq,
		const std::vector<Complex>& tranFucntion, const MaterialLibrary* matLibrary,
		const Antenna* txAntenna);																					//计算路径三维复电场值（涉及传输函数的计算）
	Polarization3D CalculateStrengthField3D(RtLbsType power, RtLbsType freq,
		const MaterialLibrary* matLibrary, const Antenna* txAntenna);												//计算路径三维复电场值（不涉及传输函数的计算）
	Polarization3D CalculateStrengthField3DReverse(RtLbsType power, RtLbsType freq,
		const std::vector<Complex>& tranFucntion, const MaterialLibrary* matLibrary,
		const Antenna* txAntenna);																					//反向计算路径三维复电场值（涉及传输函数的计算）
	Polarization3D CalculateStrengthField3DReverse(RtLbsType power, RtLbsType freq,
		const MaterialLibrary* matLibrary, const Antenna* txAntenna);												//反向计算路径三维复电场值（不涉及传输函数的计算）
	Complex CalculateStrengthField(RtLbsType power, RtLbsType freq,
		const std::vector<Complex>& tranFucntion, const MaterialLibrary* matLibrary,
		const Antenna* txAntenna, const Antenna* rxAntenna);														//计算路径复电场值（涉及传输函数的计算）
	Complex CalculateStrengthField(RtLbsType power, RtLbsType freq, const MaterialLibrary* matLibrary,
		const Antenna* txAntenna, const Antenna* rxAntenna);														//计算路径复电场值（不涉及传输函数的计算）
	Complex CalculateStrengthFieldReverse(RtLbsType power, RtLbsType freq,
		const std::vector<Complex>& tranFucntion, const MaterialLibrary* matLibrary,
		const Antenna* txAntenna, const Antenna* rxAntenna);														//反向计算路径复电场值（涉及传输函数的计算）
	Complex CalculateStrengthFieldReverse(RtLbsType power, RtLbsType freq, const MaterialLibrary* matLibrary,
		const Antenna* txAntenna, const Antenna* rxAntenna);														//反向计算路径复电场值（不涉及传输函数的计算）
	RtLbsType CalculatePowerInLBSSystem(RtLbsType freq, const std::vector<Complex>& tranFucntion, const MaterialLibrary* matLibrary, const Antenna* trxAntenna);			//计算LBS定位系统中的功率值
	RtLbsType GetPropagationTime() const;																			//计算多径传播时间
	RtLbsType GetPropagationLength() const;																			//计算传播距离
	RtLbsType GetPhaseOffset(RtLbsType freq) const;																	//计算相位偏移量	
	RtLbsType GetAOD_Phi() const;																					//计算路径离开方位角
	RtLbsType GetAOD_Theta() const;																					//计算路径离开俯仰角
	RtLbsType GetAOA_Phi() const;																					//计算路径到达付俯仰角
	RtLbsType GetAOA_Theta() const;																					//计算路径到达俯仰角
	void DeepCopy(const RayPath3D* path);																				//深度复制
	void DeepDestroy();																								//深度销毁
	void Clear();
	void OutputRaypath(std::ofstream& stream) const;																//输出多径至文件中
};

#endif
