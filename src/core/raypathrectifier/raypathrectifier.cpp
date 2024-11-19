#include "raypathrectifier.h"

bool RectifyRayPath(const Scene* scene, RayPath*& path, const Point2D& p)
{
	if (!path->m_bContainRefract) {
		return _isValidAndRectifyCommonRayPath(scene, path, p);
	}
	return _isValidAndRectifyRefractedRayPath(scene, path, p);
}

bool RectifyGPURayPath(const Scene* scene, RayPathGPU& path, const Point2D& p)
{
	if (!path.m_bContainRefract) {
		return _isValidAndRectifyCommonGPURayPath(scene, path, p);
	}
	return _isValidAndRectifyRefractedGPURayPath(scene, path, p);
}

bool _isValidAndRectifyCommonRayPath(const Scene* scene, RayPath*& path, const Point2D& p)
{
	//����˼·�Ǵ�ĩβ�ڵ������������ǰ���������Դ��·��
	if (path->m_nodes.empty())
		return false;
	PathNode*& endNode = path->m_nodes.back();
	endNode->m_point = p;//����ĩβ�ڵ�Ľڵ�����
	endNode->m_type = NODE_STOP;
	Point2D endPoint = p;//ĩβ�ڵ������
	Ray2D t_ray;//�����ཻ���Ե�ray
	Intersection2D t_inter;//�����ཻ���Ե�intersect

	for (auto it = std::next(path->m_nodes.rbegin()); it != path->m_nodes.rend(); ++it) { //����͸����з�ɢ�ԣ��������ⷽ������У������Ҫ�޸��㷨
		PathNode*& curNode = *it;
		if (curNode->m_type == NODE_ROOT)
			continue;
		if (curNode->m_type == NODE_DIFF) {//����Դ�ڵ�
			endPoint = curNode->m_point;//����ĩβ�ڵ����꣬������һ�ֵĵ�������
			continue;
		}
		if (curNode->m_type == NODE_REFL) {//����ڵ�
			//1-������Դ
			Point2D& vSource = curNode->m_source;
			t_ray.m_Ori = vSource;
			t_ray.m_Dir = (endPoint - vSource).Normalize();
			Segment2D* segment = curNode->m_segment;
			if (!segment->GetIntersect(t_ray, &t_inter))//�����ڽ��㣬�������߲�����
				return false;
			curNode->m_point = t_inter.m_intersect;//���µ�ǰ�ڵ�Ľ�����Ϣ
			endPoint = curNode->m_point;//��һ�ε������õ�ǰ���¹��Ľڵ�����
			continue;
		}
	}

	//20240521�����ж�ĩβ��ǰ�ڵ㹹�ɵ��߶��뻷���ཻ�����ཻ�Ҳ��ǽڵ����ڵ���Ԫ����·����Ч
	Segment2D rayTestSegment(path->m_nodes[path->m_nodes.size() - 1]->m_point, path->m_nodes[path->m_nodes.size() - 2]->m_point);               /** @brief	���ڼ��ڵ��Ƿ��뻷���ཻ	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //���߶��뻷���ཻ����·����Ч
		return false;
	}

	return true;
}

bool _isValidAndRectifyRefractedRayPath(const Scene* scene, RayPath*& path, const Point2D& p)
{
	//����˼·��
	// 1.����·����Ϊ����·�����͸��·���飬��ˮ��Ϊ����Դ��
	// 2.����·������һ�㷽������
	// 3.͸��·�����շ������߷�������

	if (path->m_nodes.empty())
		return false;
	path->m_nodes.back()->m_point = p;//����ĩβ�ڵ�Ľڵ�����

	std::vector<RayPath*> splitpaths; //�����·��
	std::vector<PathNode*> nodes;
	bool containRefract = false;
	//1-��path���չ���Դ���зֶ�
	for (auto it = path->m_nodes.begin(); it != path->m_nodes.end(); ++it) {
		PathNode*& node = *it;
		nodes.push_back(node);
		if (node->m_type == NODE_DIFF) { //��ǰ�ڵ�Ϊ����Դ�ڵ㣬
			RayPath* splitPath = new RayPath(nodes, containRefract);
			splitpaths.push_back(splitPath);
			nodes.clear();
			nodes.push_back(node);//����nodes
			containRefract = false;//����͸��·������״̬
			continue;
		}
		if (std::next(it) == path->m_nodes.end()) {//β���ڵ�
			RayPath* splitPath = new RayPath(nodes, containRefract);
			splitpaths.push_back(splitPath);
			nodes.clear();
		}
		if (node->m_type == NODE_TRANOUT) {			//���ҽ�����͸���ʱ������·���ű���Ϊ��͸��·��
			containRefract = true;
			continue;
		}

	}

	//2-����ͬ��·�����в�ͬ�Ĵ�����,��԰���͸���·������Ϊĩβ�ڵ�(STOP�ڵ������DIFF�ڵ�)Ϊ����Ҫ�Ż��Ľڵ�

	for (auto it = splitpaths.begin(); it != splitpaths.end(); ++it) {
		RayPath*& curSplitPath = *it;
		PathNode*& endNode = path->m_nodes.back();
		if (!path->m_bContainRefract) {//��͸��·��
			if (!_isValidAndRectifyCommonRayPath(scene, curSplitPath, p))//����·������������ֱ���˳��������
				return false;
			continue;
		}
		//͸��·������ģ��
		if (!PathTraceLite(path)) {
			return false;
		}
		continue;
	}

	//3-��������ɵ�·������ƴ�Ӻϳ�
	std::vector<PathNode*> newNodes;
	for (RayPath*& newPath : splitpaths) {
		if (&newPath != &splitpaths.back()) {//�������һ��·��
			for (auto it = newPath->m_nodes.begin(); it != prev(newPath->m_nodes.end()); ++it) {//�������һ��Ԫ��
				PathNode* node = *it;
				newNodes.push_back(node);
			}
			continue;
		}
		//�����һ��·��
		for (auto it = newPath->m_nodes.begin(); it != newPath->m_nodes.end(); ++it) {//�������һ��Ԫ��
			PathNode* node = *it;
			newNodes.push_back(node);
		}
	}

	path->m_nodes = newNodes;//����·�������͸��·��������

	//20240521�����ж�ĩβ��ǰ�ڵ㹹�ɵ��߶��뻷���ཻ�����ཻ�Ҳ��ǽڵ����ڵ���Ԫ����·����Ч
	Segment2D rayTestSegment(path->m_nodes[path->m_nodes.size() - 1]->m_point, path->m_nodes[path->m_nodes.size() - 2]->m_point);               /** @brief	���ڼ��ڵ��Ƿ��뻷���ཻ	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //���߶��뻷���ཻ����·����Ч
		return false;
	}

	return true;
}

bool _isValidAndRectifyCommonGPURayPath(const Scene* scene, RayPathGPU& path, const Point2D& p)
{
	//��ĩβ�ڵ������������ǰ���������Դ��·��
	if (path.m_nodes.empty())
		return false;
	PathNodeGPU* endNode = path.m_nodes.front(); endNode->m_inter.m_intersect = p;//����·��ĩβ�ڵ�����꣬����ġ�ĩβ��ָ��������·����ĩβ

	Point2D endPoint = p;//ĩβ�ڵ������
	Ray2DGPU tRay;//�����ཻ���Ե�ray
	Intersection2DGPU testInter;//�����ཻ���Ե�intersect
	for (auto it = std::next(path.m_nodes.begin()); it != path.m_nodes.end(); ++it) {
		PathNodeGPU* curNode = *it;			/** @brief	��ǰ�ڵ�	*/
		PathNodeGPU* prevNode = *std::prev(it);	/** @brief	ǰһ���ڵ�	*/
		if (curNode->m_inter.m_type == NODE_ROOT)//������ʼ�ڵ�
			continue;
		if (curNode->m_inter.m_type == NODE_DIFF) {//����Դ�ڵ�
			endPoint = curNode->m_inter.m_intersect;
			continue;
		}
		if (curNode->m_inter.m_type == NODE_REFL) {//����ڵ�
			//������Դ����
			Point2D vSource = prevNode->m_inter.GetVisualSource();
			tRay.m_Ori = vSource;
			tRay.m_Dir = (endPoint - vSource).Normalize();
			const Segment2DGPU* segment = &scene->m_gpuSegmentBuf[curNode->m_inter.m_segmentId];
			if (!segment->GetIntersect(tRay, &testInter))//�����ڽ��㣬�������߲�����
				return false;
			curNode->m_inter.m_intersect = testInter.m_intersect;//���µ�ǰ�ڵ�Ľ�����Ϣ
			endPoint = curNode->m_inter.m_intersect; //��һ�ε������õ�ǰ���¹��Ľڵ�����
			continue;
		}
	}

	//20240521�����ж�ĩβ��ǰ�ڵ㹹�ɵ��߶��뻷���ཻ�����ཻ�Ҳ��ǽڵ����ڵ���Ԫ����·����Ч
	Segment2D rayTestSegment(path.m_nodes[0]->m_inter.m_intersect, path.m_nodes[1]->m_inter.m_intersect);               /** @brief	���ڼ��ڵ��Ƿ��뻷���ཻ	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //���߶��뻷���ཻ����·����Ч
		return false;
	}

	return true;
}

bool _isValidAndRectifyRefractedGPURayPath(const Scene* scene, RayPathGPU& path, const Point2D& p)
{
	//����˼·��
	// 1.����·����Ϊ����·�����͸��·���飬��ˮ��Ϊ����Դ��
	// 2.����·������һ�㷽������
	// 3.͸��·�����շ������߷�������

	if (path.m_nodes.empty())
		return false;
	path.m_nodes.front()->m_inter.m_intersect = p; //����·��ĩβ�ڵ������

	std::vector<RayPathGPU> splitPaths; //���ݹ���Դ�����·��
	std::vector<PathNodeGPU*> nodes;
	bool containRefract = false;
	//1-��path���չ���Դ���зֶδ���
	for (auto it = path.m_nodes.begin(); it != path.m_nodes.end(); ++it) {
		PathNodeGPU* node = *it;
		nodes.push_back(node);
		if (node->m_inter.m_type == NODE_DIFF) {//��ǰ�ڵ�Ϊ����Դ�ڵ�
			RayPathGPU newPath(nodes, containRefract);
			splitPaths.push_back(newPath);
			nodes.clear();
			nodes.push_back(node);//����nodes������nodes�еĳ�ʼֵ
			containRefract = false;
			continue;
		}
		if (next(it) == path.m_nodes.end()) {//�Ƿ񵽴����е�β��
			RayPathGPU newPath(nodes, containRefract);
			splitPaths.push_back(newPath);
			nodes.clear();
			std::vector<PathNodeGPU*>().swap(nodes);
		}
		if (node->m_inter.m_type == NODE_TRANIN ||
			node->m_inter.m_type == NODE_TRANOUT ||
			node->m_inter.m_type == NODE_ETRANIN ||
			node->m_inter.m_type == NODE_ETRANOUT) {//��ǰ�ڵ�Ϊ͸��ڵ�
			containRefract = true;
			continue;
		}
	}

	//2-��Բ�ͬ��·�����в�ͬ�Ĵ���������԰���͸���·������Ϊĩβ�ڵ�Ϊ����Ҫ�Ż��Ľڵ�

	for (auto it = splitPaths.begin(); it != splitPaths.end(); ++it) {
		RayPathGPU& splitPath = *it;
		PathNodeGPU* endNode = splitPath.m_nodes.front();
		if (!splitPath.m_bContainRefract) {//��͸��·��
			if (!_isValidAndRectifyCommonGPURayPath(scene, splitPath, p))
				return false;
			continue;
		}
		//͸��·������ģ��
		if (!PathTraceGPULite(splitPath, nullptr))
			return false;
		continue;
	}

	//3- ��������ɵ�·������ƴ�Ӻϳ�
	std::vector<PathNodeGPU*> newNodes;
	for (auto it = splitPaths.begin(); it != splitPaths.end(); ++it) {
		if (it != prev(splitPaths.end())) { //�������һ��·��
			const RayPathGPU& newPath = *it;
			for (auto it1 = newPath.m_nodes.begin(); it1 != prev(newPath.m_nodes.end()); ++it1) {
				PathNodeGPU* node = *it1;
				newNodes.push_back(node);
			}
			continue;
		}
		//���һ��·��
		const RayPathGPU& newPath = *it;
		for (auto it1 = newPath.m_nodes.begin(); it1 != newPath.m_nodes.end(); ++it) {//���һ��·���������һ��Ԫ��
			PathNodeGPU* node = *it1;
			newNodes.push_back(node);
		}
	}
	path.m_nodes = newNodes;//����·�������͸��·��������

	//20240521�����ж�ĩβ��ǰ�ڵ㹹�ɵ��߶��뻷���ཻ�����ཻ�Ҳ��ǽڵ����ڵ���Ԫ����·����Ч
	Segment2D rayTestSegment(path.m_nodes[0]->m_inter.m_intersect, path.m_nodes[1]->m_inter.m_intersect);               /** @brief	���ڼ��ڵ��Ƿ��뻷���ཻ	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //���߶��뻷���ཻ����·����Ч
		return false;
	}

	return true;
}
