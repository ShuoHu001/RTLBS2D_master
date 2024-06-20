#ifndef RTLBS_CPUCONVERTERPATHNODE
#define RTLBS_CPUCONVERTERPATHNODE

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "tree/pathnode.h"
#include "geometry/gpu/ray2dgpu.h"
#include "geometry/segment2d.h"


class CPUConverterPathNode {
public:
	int m_fatherNodeId;										/** @brief	父节点ID	*/
	int m_layerId;											/** @brief	当前节点所在的层ID	*/
	int m_sensorId;											/** @brief	当前节点归属的传感器 ID	*/
	PATHNODETYPE m_type;									/** @brief	节点类型	*/
	Point2D m_point;										/** @brief	节点坐标	*/
	int m_segmentId;										/** @brief	节点所在线段ID	*/
	int m_wedgeId;											/** @brief	节点所在	棱劈ID*/
	Ray2DGPU m_prevRay;										/** @brief	节点之前的射线	*/
	Point2D m_generalSource;								/** @brief	广义源节点坐标	*/
	RtLbsType m_ft;											/** @brief	当前节点距离root源的距离	*/

public:
	HOST_DEVICE_FUNC CPUConverterPathNode();
	CPUConverterPathNode(const PathNode& node, int fatherNodeId, int sensorId, int layerId);
	HOST_DEVICE_FUNC CPUConverterPathNode(const CPUConverterPathNode& node);
	HOST_DEVICE_FUNC ~CPUConverterPathNode();
	HOST_DEVICE_FUNC CPUConverterPathNode& operator = (const CPUConverterPathNode& node);
	HOST_DEVICE_FUNC bool IsCapturedByPoint(const Point2D& p, RtLbsType splitRadius, CPUConverterPathNode* fatherNode) const;			//当前节点是否捕获到坐标P
};

#endif
