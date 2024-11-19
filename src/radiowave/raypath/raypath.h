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
	std::vector<PathNode*> m_nodes;							/** @brief	·���ڵ�	*/
	bool m_bContainRefract;									/** @brief	�Ƿ����͸��	*/
	int m_angularSpectrumCategoryId;						/** @brief	·�����ڵĴ������Ƕ�������ID	*/

public:
	RayPath();
	RayPath(const std::vector<PathNode*>& nodes, bool containRefract);
	RayPath(const RayPath& path);
	~RayPath();
	bool operator == (const RayPath& path) const;
	bool operator != (const RayPath& path) const;

	void DeepCopy(const RayPath* path);															//��ȸ���·��
	void DeepDestroy();																			//�������·��
	void Clear();
	void Write2File(std::ofstream& stream) const;														//д�����ļ���
	void ConvertFrom(const std::vector<CPUConverterPathNode*>& nodes, const std::vector<Segment2D*>& segments, const std::vector<Wedge2D*>& wedges);
};

//��·���ϵ���Ԫȫ��ͬ����Ϊ��ͬ·��
inline bool RepatePathComparison(const RayPath& pathA, const RayPath& pathB) {
	if (!pathA.m_bContainRefract || !pathB.m_bContainRefract)//Ŀǰֻ���͸��·����������
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
