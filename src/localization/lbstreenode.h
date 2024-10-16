#ifndef RTLBS_LBSTREENODE
#define RTLBS_LBSTREENODE

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "math/point2d.h"
#include "geometry/ray2d.h"
#include "geometry/segment2d.h"
#include "geometry/wedge2d.h"
#include "localization/generalsource.h"
#include "radiowave/raypath/raytreenode.h"
#include "radiowave/raypath/gpu/treenodegpu.h"
#include "equipment/sensor/sensordata.h"


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
	PathNode m_originPathNode;										/** @brief	从pathnode来源的本体	*/

public:
	LBSTreeNode();
	LBSTreeNode(const LBSTreeNode& node);
	LBSTreeNode(const PathNode& node, SensorData* sensorData);													//由路径节点进行初始化，含AOA型的定位算法
	LBSTreeNode(const PathNode& node);																			//由路径节点进行初始化，只适合TDOA的定位算法
	LBSTreeNode(const TreeNodeGPU& node, Segment2D* segment, Wedge2D* wedge, SensorData* sensorData);			//由GPU树节点进行初始化-弃用
	~LBSTreeNode();
	LBSTreeNode& operator = (const LBSTreeNode& node);

	void GetGeneralSource_AOA(GeneralSource* source) const;														//获取广义源-AOA型方法，包含： AOA、AOA-TDOA
	void GetGeneralSource_TOA(GeneralSource* source) const;														//获取广义源-TOA方法, 包含TOA
	void GetGeneralSource_TDOA(GeneralSource* source) const;													//获取广义源-TDOA型方法，包含：TDOA
};

#endif
