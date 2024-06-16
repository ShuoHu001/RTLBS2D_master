#ifndef RTLBS_PATHNODEGPU
#define RTLBS_PATHNODEGPU

#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/gpu/ray2dgpu.h"
#include "geometry/point2d.h"
#include "geometry/gpu/Intersection2Dgpu.h"

class PathNodeGPU {

public:
	bool m_isValid;										/** @brief	是否有效	*/
	int m_layer;										/** @brief	节点所在的迭代层数	*/
	int m_rxId;											/** @brief	节点归属的rx编号	*/
	RtLbsType m_ft;										/** @brief	传播长度	*/
	Intersection2DGPU m_inter;							/** @brief	节点所在的交点信息	*/
	

public:
	HOST_DEVICE_FUNC PathNodeGPU();
	HOST_DEVICE_FUNC PathNodeGPU(const PathNodeGPU& node);										//赋值构造函数
	HOST_DEVICE_FUNC ~PathNodeGPU() {};
	HOST_DEVICE_FUNC PathNodeGPU(bool isValid, int layer, int rxId, Intersection2DGPU inter);
	HOST_DEVICE_FUNC PathNodeGPU& operator = (const PathNodeGPU& node);//赋值运算符重载
};

#endif
