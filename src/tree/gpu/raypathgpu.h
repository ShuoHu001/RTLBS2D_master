#ifndef RTLBS_RAYPATHGPU
#define RTLBS_RAYPATHGPU
#include "rtlbs.h"
#include "utility/define.h"
#include "pathnodegpu.h"
#include "physical/gpu/pathtracinggpu.h"

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
	bool IsValidAndRectifyCommon(const Point2D& p, const Scene* scene, const Segment2DGPU* segments);											//基于p点对路径的正确性进行检测和修正，非透射路径
	bool IsValidAndRectifyRefractMixed(const Point2D& p, const Scene* scene, const Segment2DGPU* segments);									//基于p点对路径的正确定进行检测和修正，包含透射路径

private:
	bool IsValidAndRectifyRefract(const Segment2DGPU* segments);//修正透射路径

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


//输出单个tx对所有rx坐标点的多径
void GenerateMultiPathofRxSingleTxGPU(const std::vector<PathNodeGPU*>& nodes, const Point2D& txPosition, const std::vector<Point2D>& rxPositions, const Segment2DGPU* segments, const Scene* scene, std::vector<std::vector<RayPathGPU*>>& outRayPath);
//基于GPU运算的一维nodes节点解算出每个rx上的坐标值
void GenerateMultiPathofRxGPU(std::vector<std::vector<PathNodeGPU*>> nodes, std::vector<Point2D> txPositions, std::vector<Point2D> rxPositions, Segment2DGPU* segments, const Scene* scene, std::vector<std::vector<std::vector<RayPathGPU*>>>& outRayPaths);


#endif
