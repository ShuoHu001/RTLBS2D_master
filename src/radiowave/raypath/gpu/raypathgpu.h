#ifndef RTLBS_RAYPATHGPU
#define RTLBS_RAYPATHGPU
#include "rtlbs.h"
#include "utility/define.h"
#include "pathnodegpu.h"

class RayPathGPU {
public:
	std::vector<PathNodeGPU*> m_nodes;										/** @brief	路径节点	*/
	bool m_bContainRefract;													/** @brief	是否是可回溯路径（透射路径不可回溯，需要进行二次处理）	*/

public:
	RayPathGPU();
	RayPathGPU(std::vector<PathNodeGPU*> nodes, bool containRefract);
	~RayPathGPU();
	bool operator == (const RayPathGPU& path) const;
	bool operator != (const RayPathGPU& path) const;

};

inline bool RepeatePathComparison(const RayPathGPU& pathA, const RayPathGPU& pathB) {
	if (!pathA.m_bContainRefract || !pathB.m_bContainRefract)//目前只针对透射路径进行判别相同性
		return false;
	const std::vector<PathNodeGPU*>& nodesA = pathA.m_nodes;
	const std::vector<PathNodeGPU*>& nodesB = pathB.m_nodes;
	if (nodesA.size() != nodesB.size())
		return false;
	for (int i = 0; i < nodesA.size(); ++i) {
		if (nodesA[i]->m_inter.m_type != nodesB[i]->m_inter.m_type)
			return false;
		if (nodesA[i]->m_inter.m_type == NODE_REFL ||
			nodesA[i]->m_inter.m_type == NODE_TRANIN ||
			nodesA[i]->m_inter.m_type == NODE_TRANOUT ||
			nodesA[i]->m_inter.m_type == NODE_ETRANIN ||
			nodesA[i]->m_inter.m_type == NODE_ETRANOUT ) {
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





#endif
