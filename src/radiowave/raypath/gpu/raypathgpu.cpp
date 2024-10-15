#include "raypathgpu.h"

RayPathGPU::RayPathGPU()
	:m_bContainRefract(false)
{
}

RayPathGPU::RayPathGPU(std::vector<PathNodeGPU*> nodes, bool containRefract)
	: m_nodes(nodes)
	, m_bContainRefract(containRefract)
{
	//����ڵ㴫���ۻ�����
	RtLbsType st = 0;
	for (int i = static_cast<int>(m_nodes.size()) - 1; i >= 0; --i) {
		if (i == 0) {									//��һ���ڵ�Ϊ��ʵ·����ĩβ�ڵ㣬��GPU������δ���ǵ���һ�����㵽���յ�ľ��룬�����Ҫ���¼������
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



