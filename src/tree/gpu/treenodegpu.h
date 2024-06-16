#ifndef RTLBS_TREENODEGPU
#define RTLBS_TREENODEGPU

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/point2d.h"
#include "geometry/gpu/ray2dgpu.h"

class TreeNodeGPU {
public:
	bool m_isValid;											/** @brief	是否有效标志	*/
	PATHNODETYPE m_type;									/** @brief	节点类型	*/
	uint16_t m_depth;										/** @brief	节点深度	*/
	RtLbsType m_t;											/** @brief	传播距离	*/
	Point2D m_point;										/** @brief	节点坐标	*/
	int m_segmentId;										/** @brief	反射/透射 判定的面元Id	*/
	int m_wedgeId;											/** @brief	绕射判定的楞劈Id	*/
	Ray2DGPU m_nextRay;										/** @brief	由节点引出的射线	*/

public:
	HOST_DEVICE_FUNC TreeNodeGPU();												//常规构造函数
	HOST_DEVICE_FUNC TreeNodeGPU(const TreeNodeGPU& node);						//赋值构造函数
	HOST_DEVICE_FUNC ~TreeNodeGPU();											//析构函数
	HOST_DEVICE_FUNC TreeNodeGPU operator = (const TreeNodeGPU& node);			//=计算符号
	HOST_DEVICE_FUNC Point2D GetGeneralSource();								//计算广义源
};


#endif
