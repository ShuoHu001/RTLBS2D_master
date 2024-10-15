#include "treenodegpu.h"

TreeNodeGPU::TreeNodeGPU()
	: m_isValid(false)
	, m_type(NODE_INIT)
	, m_depth(0)
	, m_t(0.0)
	, m_segmentId(-1)
	, m_wedgeId(-1)
{
}

TreeNodeGPU::TreeNodeGPU(const TreeNodeGPU& node)
	: m_isValid(node.m_isValid)
	, m_type(node.m_type)
	, m_depth(node.m_depth)
	, m_t(node.m_t)
	, m_point(node.m_point)
	, m_segmentId(node.m_segmentId)
	, m_wedgeId(node.m_wedgeId)
	, m_nextRay(node.m_nextRay)
{
}

TreeNodeGPU::~TreeNodeGPU()
{
}

TreeNodeGPU TreeNodeGPU::operator=(const TreeNodeGPU& node)
{
	m_isValid = node.m_isValid;
	m_type = node.m_type;
	m_depth = node.m_depth;
	m_t = node.m_t;
	m_point = node.m_point;
	m_segmentId = node.m_segmentId;
	m_wedgeId = node.m_wedgeId;
	m_nextRay = node.m_nextRay;
	return *this;
}

Point2D TreeNodeGPU::GetGeneralSource()
{
	return m_nextRay.GetVisualSource();
}
