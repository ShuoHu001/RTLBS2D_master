#ifndef RTLBS_RAYPATH
#define RTLBS_RAYPATH

#include "rtlbs.h"
#include "utility/define.h"
#include "pathnode.h"
#include "geometry/ray2d.h"
#include "geometry/Intersection2D.h"
#include "geometry/segment2d.h"


class CPUConverterPathNode;
class Scene;


class RayPath {
public:
	std::vector<PathNode*> m_nodes;							/** @brief	路径节点	*/
	bool m_bContainRefract;									/** @brief	是否包含透射	*/
	int m_angularSpectrumCategoryId;						/** @brief	路径所在的传感器角度谱种类ID	*/

public:
	RayPath();
	RayPath(const std::vector<PathNode*>& nodes, bool containRefract);
	RayPath(const RayPath& path);
	~RayPath();
	bool operator == (const RayPath& path) const;
	bool operator != (const RayPath& path) const;

	void DeepCopy(const RayPath* path);															//深度复制路径
	void DeepDestroy();																			//深度销毁路径
	void Clear();
	void Write2File(std::ofstream& stream) const;														//写入至文件中
	void ConvertFrom(const std::vector<CPUConverterPathNode*>& nodes, const std::vector<Segment2D*>& segments, const std::vector<Wedge2D*>& wedges);
};

//若路径上的面元全相同，则为相同路径
inline bool RepatePathComparison(const RayPath& pathA, const RayPath& pathB) {
	if (!pathA.m_bContainRefract || !pathB.m_bContainRefract)//目前只针对透射路径进行修正
		return false;
	const std::vector<PathNode*>& nodesA = pathA.m_nodes;
	const std::vector<PathNode*>& nodesB = pathB.m_nodes;
	if (nodesA.size() != nodesB.size())
		return false;
	for (size_t i = 0; i < nodesA.size(); ++i) {
		if (nodesA[i]->m_type != nodesB[i]->m_type)
			return false;
		if (nodesA[i]->m_type == NODE_REFL ||
			nodesA[i]->m_type == NODE_TRANIN ||
			nodesA[i]->m_type == NODE_TRANOUT ||
			nodesA[i]->m_type == NODE_ETRANIN ||
			nodesA[i]->m_type == NODE_ETRANOUT) {
			if (*nodesA[i]->m_segment != *nodesB[i]->m_segment)
				return false;
		}
		if (nodesA[i]->m_type == NODE_DIFF) {
			if (*nodesA[i]->m_wedge != *nodesB[i]->m_wedge)
				return false;
		}
	}
	return true;
}

#endif
