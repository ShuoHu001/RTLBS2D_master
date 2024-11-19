#include "raypathgpu.h"

RayPathGPU::RayPathGPU()
	:m_bContainRefract(false)
{
}

RayPathGPU::RayPathGPU(std::vector<PathNodeGPU*> nodes, bool containRefract)
	: m_nodes(nodes)
	, m_bContainRefract(containRefract)
{
	//计算节点传播累积距离
	RtLbsType st = 0;
	for (int i = static_cast<int>(m_nodes.size()) - 1; i >= 0; --i) {
		if (i == 0) {									//第一个节点为真实路径的末尾节点，在GPU程序中未考虑到上一个交点到接收点的距离，因此需要重新计算距离
			st += (m_nodes[1]->m_inter.m_intersect - m_nodes[0]->m_inter.m_intersect).Length();
		}
		else {
			st += m_nodes[i]->m_inter.m_ft;
		}
		m_nodes[i]->m_ft = st;
	}
}

RayPathGPU::~RayPathGPU()
{
}

bool RayPathGPU::operator==(const RayPathGPU& path) const
{
	const std::vector<PathNodeGPU*>& nodesA = m_nodes;
	const std::vector<PathNodeGPU*>& nodesB = path.m_nodes;
	if (nodesA.size() != nodesB.size())
		return false;
	for (int i = 0; i < nodesA.size(); ++i) {
		if (nodesA[i]->m_inter.m_type != nodesB[i]->m_inter.m_type)
			return false;
		if (nodesA[i]->m_inter.m_type == NODE_REFL ||
			nodesA[i]->m_inter.m_type == NODE_TRANIN ||
			nodesA[i]->m_inter.m_type == NODE_TRANOUT || 
			nodesA[i]->m_inter.m_type == NODE_ETRANIN ||
			nodesA[i]->m_inter.m_type == NODE_ETRANOUT) {
			if (nodesA[i]->m_inter.m_segmentId != nodesB[i]->m_inter.m_segmentId)
				return false;
		}
		if (nodesA[i]->m_inter.m_type == NODE_DIFF) {
			if (nodesA[i]->m_inter.m_wedgeId != nodesB[i]->m_inter.m_wedgeId)
				return false;
		}
	}
	return true;
}

bool RayPathGPU::operator!=(const RayPathGPU& path) const
{
	return !(*this == path);
}



