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
	int m_fatherNodeId;										/** @brief	���ڵ�ID	*/
	int m_layerId;											/** @brief	��ǰ�ڵ����ڵĲ�ID	*/
	int m_sensorId;											/** @brief	��ǰ�ڵ�����Ĵ����� ID	*/
	PATHNODETYPE m_type;									/** @brief	�ڵ�����	*/
	Point2D m_point;										/** @brief	�ڵ�����	*/
	int m_segmentId;										/** @brief	�ڵ������߶�ID	*/
	int m_wedgeId;											/** @brief	�ڵ�����	����ID*/
	Ray2DGPU m_prevRay;										/** @brief	�ڵ�֮ǰ������	*/
	Point2D m_generalSource;								/** @brief	����Դ�ڵ�����	*/
	RtLbsType m_ft;											/** @brief	��ǰ�ڵ����rootԴ�ľ���	*/

public:
	HOST_DEVICE_FUNC CPUConverterPathNode();
	CPUConverterPathNode(const PathNode& node, int fatherNodeId, int sensorId, int layerId);
	HOST_DEVICE_FUNC CPUConverterPathNode(const CPUConverterPathNode& node);
	HOST_DEVICE_FUNC ~CPUConverterPathNode();
	HOST_DEVICE_FUNC CPUConverterPathNode& operator = (const CPUConverterPathNode& node);
	HOST_DEVICE_FUNC bool IsCapturedByPoint(const Point2D& p, RtLbsType splitRadius, CPUConverterPathNode* fatherNode) const;			//��ǰ�ڵ��Ƿ񲶻�����P
};

#endif
