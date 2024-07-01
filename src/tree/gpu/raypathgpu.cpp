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

bool RayPathGPU::IsValidAndRectifyCommon(const Point2D& p, const Scene* scene, const Segment2DGPU* segments)//���ڷ�������׷���㷨��������
{
	//��ĩβ�ڵ������������ǰ���������Դ��·��
	if (m_nodes.empty())
		return false;
	PathNodeGPU* endNode = m_nodes.front(); endNode->m_inter.m_intersect = p;//����·��ĩβ�ڵ�����꣬����ġ�ĩβ��ָ��������·����ĩβ
	
	Point2D endPoint = p;//ĩβ�ڵ������
	Ray2DGPU tRay;//�����ཻ���Ե�ray
	Intersection2DGPU testInter;//�����ཻ���Ե�intersect
	for (auto it = next(m_nodes.begin()); it != m_nodes.end(); ++it) {
		PathNodeGPU* curNode = *it;			/** @brief	��ǰ�ڵ�	*/
		PathNodeGPU* prevNode = *prev(it);	/** @brief	ǰһ���ڵ�	*/
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
			const Segment2DGPU* segment = &segments[curNode->m_inter.m_segmentId];
			if (!segment->GetIntersect(tRay, &testInter))//�����ڽ��㣬�������߲�����
				return false;
			curNode->m_inter.m_intersect = testInter.m_intersect;//���µ�ǰ�ڵ�Ľ�����Ϣ
			endPoint = curNode->m_inter.m_intersect; //��һ�ε������õ�ǰ���¹��Ľڵ�����
			continue;
		}
	}

	//20240521�����ж�ĩβ��ǰ�ڵ㹹�ɵ��߶��뻷���ཻ�����ཻ�Ҳ��ǽڵ����ڵ���Ԫ����·����Ч
	Segment2D rayTestSegment(m_nodes[0]->m_inter.m_intersect, m_nodes[1]->m_inter.m_intersect);               /** @brief	���ڼ��ڵ��Ƿ��뻷���ཻ	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //���߶��뻷���ཻ����·����Ч
		return false;
	}

	return true;
}

bool RayPathGPU::IsValidAndRectifyRefractMixed(const Point2D& p, const Scene* scene, const Segment2DGPU* segments)
{
	//����˼·��
	// 1.����·����Ϊ����·�����͸��·���飬��ˮ��Ϊ����Դ��
	// 2.����·������һ�㷽������
	// 3.͸��·�����շ������߷�������

	if (m_nodes.empty())
		return false;
	m_nodes.front()->m_inter.m_intersect = p; //����·��ĩβ�ڵ������

	std::vector<RayPathGPU> splitPaths; //���ݹ���Դ�����·��
	std::vector<PathNodeGPU*> nodes;
	bool containRefract = false;
	//1-��path���չ���Դ���зֶδ���
	for (auto it = m_nodes.begin(); it != m_nodes.end(); ++it) {
		PathNodeGPU* node = *it;
		nodes.push_back(node);
		if (node->m_inter.m_type == NODE_DIFF) {//��ǰ�ڵ�Ϊ����Դ�ڵ�
			RayPathGPU path(nodes, containRefract);
			nodes.clear();
			nodes.push_back(node);//����nodes������nodes�еĳ�ʼֵ
			containRefract = false;
			continue;
		}
		if (next(it) == m_nodes.end()) {//�Ƿ񵽴����е�β��
			RayPathGPU path(nodes, containRefract);
			splitPaths.push_back(path);
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
		RayPathGPU& path = *it;
		PathNodeGPU* endNode = path.m_nodes.front();
		if (!path.m_bContainRefract) {//��͸��·��
			if (!path.IsValidAndRectifyCommon(p, scene, segments))
				return false;
			continue;
		}
		//͸��·������ģ��
		if (!path.IsValidAndRectifyRefract(segments))
			return false;
		continue;
	}

	//3- ��������ɵ�·������ƴ�Ӻϳ�
	std::vector<PathNodeGPU*> newNodes;
	for (auto it = splitPaths.begin(); it != splitPaths.end();++it) {
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
	m_nodes = newNodes;//����·�������͸��·��������

	//20240521�����ж�ĩβ��ǰ�ڵ㹹�ɵ��߶��뻷���ཻ�����ཻ�Ҳ��ǽڵ����ڵ���Ԫ����·����Ч
	Segment2D rayTestSegment(m_nodes[0]->m_inter.m_intersect, m_nodes[1]->m_inter.m_intersect);               /** @brief	���ڼ��ڵ��Ƿ��뻷���ཻ	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //���߶��뻷���ཻ����·����Ч
		return false;
	}

	return true;
}


bool RayPathGPU::IsValidAndRectifyRefract(const Segment2DGPU* segments) {
	if (!PathTraceGPULite((*this), segments))
		return false;
	return true;
}

void GenerateMultiPathofRxSingleTxGPU(const std::vector<PathNodeGPU*>& nodes, const Point2D& txPosition, const std::vector<Point2D>& rxPositions, const Segment2DGPU* segments, const Scene* scene, std::vector<std::vector<RayPathGPU*>>& outRayPath) {
	PathNodeGPU* rootNode = new PathNodeGPU();//�������ڵ�
	rootNode->m_inter.m_intersect = txPosition;
	rootNode->m_inter.m_type = NODE_ROOT;
	std::vector<PathNodeGPU*> newNodes;
	int curRxId = -1;//��ʼ��rx���
	bool containReftact = false;
	for (int i = 0; i < nodes.size(); ++i) {
		newNodes.push_back(nodes[i]);
		if (nodes[i]->m_inter.m_type == NODE_TRANIN ||
			nodes[i]->m_inter.m_type == NODE_TRANOUT ||
			nodes[i]->m_inter.m_type == NODE_ETRANIN ||
			nodes[i]->m_inter.m_type == NODE_ETRANOUT) {
			containReftact = true;
		}
			
		if (nodes[i]->m_layer == 0) {//Ѱ�ҵ�����·��
			curRxId = nodes[i]->m_rxId;
			newNodes.push_back(rootNode);
			RayPathGPU* newPath = new RayPathGPU(newNodes, containReftact);
			if (!containReftact) {
				if (newPath->IsValidAndRectifyCommon(rxPositions[curRxId], scene, segments))
					outRayPath[curRxId].push_back(newPath);
			}
			else {
				if (newPath->IsValidAndRectifyRefractMixed(rxPositions[curRxId], scene, segments))
					outRayPath[curRxId].push_back(newPath);
			}
			newNodes.clear();//��������
			containReftact = false;
		}
	}
}

void GenerateMultiPathofRxGPU(std::vector<std::vector<PathNodeGPU*>> nodes, std::vector<Point2D> txPositions, std::vector<Point2D> rxPositions, Segment2DGPU* segments, const Scene* scene, std::vector<std::vector<std::vector<RayPathGPU*>>>& outRayPaths) {
	//���������nodes������Ч��raypath��ȡ����
	//����ȡ��������д����outRayPath�в�����У������
	for (int i = 0; i < nodes.size(); ++i) {//��һ��ѭ��Ϊÿ��tx, outRayPaths�е�һ��Ϊtx������outRayPaths�ڶ���Ϊrx������outRayPaths������Ϊ�ྶ����
		GenerateMultiPathofRxSingleTxGPU(nodes[i], txPositions[i], rxPositions, segments, scene, outRayPaths[i]);
	}
	////���ྶ�ļ�д�뵽�ļ�
	//for (int i = 0; i < txPositions.size(); ++i) {
	//	for (int j = 0; j < rxPositions.size(); ++j) {
	//		std::stringstream filename;
	//		filename << "path-tx" << i + 1 << "-rx" << j + 1 << ".txt";
	//		std::ofstream outFile(filename.str());
	//		if (outFile.is_open()) {
	//			for (int k = 0; k < outRayPaths[i][j].size(); ++k) {
	//				RayPathGPU& newPath = outRayPaths[i][j][k];
	//				outFile << newPath.m_nodes.size() << ",";
	//				for (int m = static_cast<int>(newPath.m_nodes.size()) - 1; m >= 0; --m) {
	//					Intersection2DGPU& inter = newPath.m_nodes[m].m_inter;
	//					outFile << inter.m_intersect.x << "," << inter.m_intersect.y << ",";
	//				}
	//				outFile << std::endl;
	//			}
	//			outFile.close();
	//		}
	//	}
	//}
}
