#ifndef RTLBS_LBSTREENODE
#define RTLBS_LBSTREENODE

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/point2d.h"
#include "geometry/ray2d.h"
#include "geometry/segment2d.h"
#include "geometry/wedge2d.h"
#include "localization/generalsource.h"
#include "tree/raytreenode.h"
#include "tree/gpu/treenodegpu.h"
#include "equipment/sensordata.h"


class LBSTreeNode {
public:
	PATHNODETYPE m_type;											/** @brief	节点类型	*/
	uint16_t m_depth;												/** @brief	节点所在的深度	*/
	Point2D m_position;												/** @brief	节点所在坐标	*/
	Point2D m_sourcePosition;										/** @brief	广义源点所在的坐标	*/
	Vector2D m_rayDir;												/** @brief	节点引出射线的方向向量	*/
	SensorData* m_sensorData;										/** @brief	传感器数据	*/
	Segment2D* m_segment;											/** @brief	节点所在面元	*/
	Wedge2D* m_wedge;												/** @brief	节点所在棱劈	*/

public:
	LBSTreeNode();
	LBSTreeNode(const LBSTreeNode& node);
	LBSTreeNode(const PathNode& node, SensorData* sensorData);													//由路径节点进行初始化
	LBSTreeNode(const TreeNodeGPU& node, Segment2D* segment, Wedge2D* wedge, SensorData* sensorData);			//由GPU树节点进行初始化
	~LBSTreeNode();
	LBSTreeNode& operator = (const LBSTreeNode& node);

	void GetGeneralSource(GeneralSource* source) const;
};

#endif
