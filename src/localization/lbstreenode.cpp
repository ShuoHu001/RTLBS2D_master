#include "lbstreenode.h"

LBSTreeNode::LBSTreeNode()
	: m_type(NODE_INIT)
	, m_depth(0)
	, m_tMin(0.0)
	, m_tMax(0.0)
	, m_sensorData(nullptr)
	, m_segment(nullptr)
	, m_wedge(nullptr)
{
}

LBSTreeNode::LBSTreeNode(const LBSTreeNode& node)
	: m_type(node.m_type)
	, m_depth(node.m_depth)
	, m_tMin(node.m_tMin)
	, m_tMax(node.m_tMax)
	, m_position(node.m_position)
	, m_sourcePosition(node.m_sourcePosition)
	, m_rayDir(node.m_rayDir)
	, m_sensorData(node.m_sensorData)
	, m_segment(node.m_segment)
	, m_wedge(node.m_wedge)
	, m_originPathNode(node.m_originPathNode)
{
}

LBSTreeNode::LBSTreeNode(const PathNode& node, SensorData* sensorData, RtLbsType ftmin, RtLbsType ftmax)
	: m_tMin(ftmin)
	, m_tMax(ftmax)
{
	m_type = node.m_type;
	m_depth = node.m_limitInfo.m_depth;
	m_position = node.m_point;
	m_sourcePosition = node.m_source;
	if (m_type != NODE_ROOT) {
		m_rayDir = node.m_nextRay.m_Dir;
	}
	m_sensorData = sensorData;
	m_segment = node.m_segment;
	m_wedge = node.m_wedge;
	m_originPathNode = node;
}

LBSTreeNode::LBSTreeNode(const PathNode& node)
	: m_tMin(node.m_nextRay.m_tMin)
	, m_tMax(node.m_nextRay.m_tMax)
{
	m_type = node.m_type;
	m_depth = node.m_limitInfo.m_depth;
	m_position = node.m_point;
	m_sourcePosition = node.m_source;
	m_sensorData = nullptr;
	m_segment = node.m_segment;
	m_wedge = node.m_wedge;
	m_originPathNode = node;
}

LBSTreeNode::LBSTreeNode(const PathNode& curNode, const PathNode& nextNode)
{
	m_type = curNode.m_type;
	m_depth = curNode.m_limitInfo.m_depth;
	m_tMin = curNode.m_ft;
	m_tMax = nextNode.m_ft;
	m_position = curNode.m_point;
	m_sourcePosition = curNode.m_source;
	m_sensorData = nullptr;
	m_segment = curNode.m_segment;
	m_wedge = curNode.m_wedge;
	m_originPathNode = curNode;
}

LBSTreeNode::LBSTreeNode(const TreeNodeGPU& node, Segment2D* segment, Wedge2D* wedge, SensorData* sensorData)
	: m_tMin(node.m_nextRay.m_tMin)
	, m_tMax(node.m_nextRay.m_tMax)
{
	m_type = node.m_type;
	m_depth = node.m_depth;
	m_position = node.m_point;
	m_sourcePosition = node.m_nextRay.GetVisualSource();
	m_rayDir = node.m_nextRay.m_Dir;
	m_sensorData = sensorData;
	m_segment = segment;
	m_wedge = wedge;
}

LBSTreeNode::~LBSTreeNode()
{
}

LBSTreeNode& LBSTreeNode::operator=(const LBSTreeNode& node)
{
	m_type = node.m_type;
	m_depth = node.m_depth;
	m_tMin = node.m_tMin;
	m_tMax = node.m_tMax;
	m_position = node.m_position;
	m_sourcePosition = node.m_sourcePosition;
	m_rayDir = node.m_rayDir;
	m_sensorData = node.m_sensorData;
	m_segment = node.m_segment;
	m_wedge = node.m_wedge;
	m_originPathNode = node.m_originPathNode;
	return *this;
}

void LBSTreeNode::GetGeneralSource_AOA(GeneralSource* source) const
{
	if (source == nullptr) {
		source = new GeneralSource();
	}
	source->m_type = m_type;
	source->m_depth = m_depth;
	source->m_sensorData = *m_sensorData;
	if (m_type != NODE_ROOT) {
		source->m_sensorData.m_phi = m_rayDir.GetTheta();		//更新传感器数据中的角度值
	}
	source->m_position = m_sourcePosition;					
	source->m_nodePosition = m_position;					//赋值节点所在的坐标
	source->m_segment = m_segment;
	source->m_wedge = m_wedge;
	source->m_originPathNode = m_originPathNode;
}

void LBSTreeNode::GetGeneralSource_TOA(GeneralSource* source) const
{
	if (source == nullptr) {
		source = new GeneralSource();
	}
	source->m_type = m_type;
	source->m_depth = m_depth;
	source->m_sensorData = *m_sensorData;
	source->m_position = m_sourcePosition;
	source->m_nodePosition = m_position;					//赋值节点所在的坐标
	source->m_segment = m_segment;
	source->m_wedge = m_wedge;
	source->m_originPathNode = m_originPathNode;
}

void LBSTreeNode::GetGeneralSource_TOA(GeneralSource* source, SensorData& sensorData)
{
	if (source == nullptr) {
		source = new GeneralSource();
	}
	source->m_type = m_type;
	source->m_depth = m_depth;
	source->m_sensorData = sensorData;
	source->m_position = m_sourcePosition;
	source->m_nodePosition = m_position;					//赋值节点所在的坐标
	source->m_segment = m_segment;
	source->m_wedge = m_wedge;
	source->m_originPathNode = m_originPathNode;
}

void LBSTreeNode::GetGeneralSource_TDOA(GeneralSource* source) const
{
	if (source == nullptr) {
		source = new GeneralSource();
	}
	source->m_type = m_type;
	source->m_depth = m_depth;
	source->m_position = m_sourcePosition;
	source->m_nodePosition = m_position;					//赋值节点所在的坐标
	source->m_segment = m_segment;
	source->m_wedge = m_wedge;
	source->m_originPathNode = m_originPathNode;
}
