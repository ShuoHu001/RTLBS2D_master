#include "raypath.h"
#include "physical/pathtracing.h"
#include "geometry/scene.h"
#include "tree/gpu/cpuconverterpathnode.h"

RayPath::RayPath()
    : m_bContainRefract(false)
    , m_angularSpectrumCategoryId(-1)
{
}

RayPath::RayPath(const std::vector<PathNode*>& nodes, bool containRefract)
    : m_nodes(nodes)
    , m_bContainRefract(containRefract)
    , m_angularSpectrumCategoryId(-1)
{
}


RayPath::~RayPath()
{
    for (int i = 0; i < m_nodes.size(); ++i) {
        delete m_nodes[i];
    }
}

bool RayPath::operator==(const RayPath& path) const
{
	const std::vector<PathNode*>& nodesA = m_nodes;
	const std::vector<PathNode*>& nodesB = path.m_nodes;
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
			if (nodesA[i]->m_segment != nodesB[i]->m_segment)
				return false;
		}
		if (nodesA[i]->m_type == NODE_DIFF) {
			if (nodesA[i]->m_wedge != nodesB[i]->m_wedge)
				return false;
		}
	}
	return true;
}

bool RayPath::operator!=(const RayPath& path) const
{
    return !(*this == path);
}

bool RayPath::IsValidAndRectify(const Point2D& p, const Scene* scene)
{
    if (!m_bContainRefract) {
        return IsValidAndRectifyCommon(p, scene);
    }
    return IsValidAndRectifyRefractMixed(p, scene);
}

void RayPath::DeepCopy(const RayPath* path)
{
    if (path == nullptr)
        return;
    m_bContainRefract = path->m_bContainRefract;
    m_angularSpectrumCategoryId = path->m_angularSpectrumCategoryId;
    m_nodes.resize(path->m_nodes.size());
    for (int i = 0; i < path->m_nodes.size(); ++i) {
        m_nodes[i] = new PathNode(*path->m_nodes[i]);
    }
}

void RayPath::DeepDestroy()
{
    for (int i = 0; i < m_nodes.size(); ++i) {
        delete m_nodes[i];
    }
}

void RayPath::Clear()
{
	m_nodes.clear();
	m_bContainRefract = false;
}

void RayPath::Write2File(std::ofstream& stream) const
{
    stream << m_nodes.size() << ",";
    for (size_t i = 0; i < m_nodes.size(); ++i) {
        stream << m_nodes[i]->m_point.x << "," << m_nodes[i]->m_point.y << ",";
    }
    stream << "\n";
}

void RayPath::ConvertFrom(const std::vector<CPUConverterPathNode*>& nodes, const std::vector<Segment2D*>& segments, const std::vector<Wedge2D*>& wedges)
{
	m_nodes.resize(nodes.size());
	for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        PathNode* newNode = new PathNode();
        newNode->ConvertFrom(*nodes[i], segments, wedges);
        m_nodes[i] = newNode;
        if (newNode->m_type == NODE_TRANIN || newNode->m_type == NODE_TRANOUT) {
            m_bContainRefract = true;
        }
	}
}

bool RayPath::IsValidAndRectifyCommon(const Point2D& p, const Scene* scene)//�����㷨����·������
{
	//����˼·�Ǵ�ĩβ�ڵ������������ǰ���������Դ��·��
	if (m_nodes.empty())
		return false;
	PathNode*& endNode = m_nodes.back(); endNode->m_point = p;//����ĩβ�ڵ�Ľڵ�����
	Point2D endPoint = p;//ĩβ�ڵ������
	Ray2D t_ray;//�����ཻ���Ե�ray
	Intersection2D t_inter;//�����ཻ���Ե�intersect

	for (auto it = std::next(m_nodes.rbegin()); it != m_nodes.rend(); ++it) { //����͸����з�ɢ�ԣ��������ⷽ������У������Ҫ�޸��㷨
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
	Segment2D rayTestSegment(m_nodes[m_nodes.size() - 1]->m_point, m_nodes[m_nodes.size() - 2]->m_point);               /** @brief	���ڼ��ڵ��Ƿ��뻷���ཻ	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //���߶��뻷���ཻ����·����Ч
		return false;
	}

	return true;
}

bool RayPath::IsValidAndRectifyRefractMixed(const Point2D& p, const Scene* scene)
{
	//����˼·��
	// 1.����·����Ϊ����·�����͸��·���飬��ˮ��Ϊ����Դ��
	// 2.����·������һ�㷽������
	// 3.͸��·�����շ������߷�������

	if (m_nodes.empty())
		return false;
	m_nodes.back()->m_point = p;//����ĩβ�ڵ�Ľڵ�����

	std::vector<RayPath> splitpaths; //�����·��
	std::vector<PathNode*> nodes;
	bool containRefract = false;
	//1-��path���չ���Դ���зֶ�
	for (auto it = m_nodes.begin(); it != m_nodes.end(); ++it) {
		PathNode*& node = *it;
		nodes.push_back(node);
		if (node->m_type == NODE_DIFF) { //��ǰ�ڵ�Ϊ����Դ�ڵ㣬
			RayPath path(nodes, containRefract);
			splitpaths.push_back(path);
			nodes.clear();
			nodes.push_back(node);//����nodes
			containRefract = false;//����͸��·������״̬
			continue;
		}
		if (std::next(it) == m_nodes.end()) {//β���ڵ�
			RayPath path(nodes, containRefract);
			splitpaths.push_back(path);
			nodes.clear(); nodes.shrink_to_fit();//�ͷ��ڴ�
		}
		if (node->m_type == NODE_REFL ||
			node->m_type == NODE_TRANIN ||
			node->m_type == NODE_TRANOUT ||
			node->m_type == NODE_ETRANIN ||
			node->m_type == NODE_ETRANOUT) {
			containRefract = true;
			continue;
		}

	}

	//2-����ͬ��·�����в�ͬ�Ĵ�����,��԰���͸���·������Ϊĩβ�ڵ�(STOP�ڵ������DIFF�ڵ�)Ϊ����Ҫ�Ż��Ľڵ�

	for (auto it = splitpaths.begin(); it != splitpaths.end(); ++it) {
		RayPath& path = *it;
		PathNode*& endNode = path.m_nodes.back();
		if (!path.m_bContainRefract) {//��͸��·��
			if (!path.IsValidAndRectifyCommon(endNode->m_point, scene))//����·������������ֱ���˳��������
				return false;
			continue;
		}
		//͸��·������ģ��
		if (!path.IsValidAndRectifyRefract())
			return false;
		continue;
	}

	//3-��������ɵ�·������ƴ�Ӻϳ�
	std::vector<PathNode*> newNodes;
	for (const RayPath& newPath : splitpaths) {
		if (&newPath != &splitpaths.back()) {//�������һ��·��
			for (auto it = newPath.m_nodes.begin(); it != prev(newPath.m_nodes.end()); ++it) {//�������һ��Ԫ��
				PathNode* node = *it;
				newNodes.push_back(node);
			}
			continue;
		}
		//�����һ��·��
		for (auto it = newPath.m_nodes.begin(); it != newPath.m_nodes.end(); ++it) {//�������һ��Ԫ��
			PathNode* node = *it;
			newNodes.push_back(node);
		}
	}
	m_nodes = newNodes;//����·�������͸��·��������

	//20240521�����ж�ĩβ��ǰ�ڵ㹹�ɵ��߶��뻷���ཻ�����ཻ�Ҳ��ǽڵ����ڵ���Ԫ����·����Ч
	Segment2D rayTestSegment(m_nodes[m_nodes.size() - 1]->m_point, m_nodes[m_nodes.size() - 2]->m_point);               /** @brief	���ڼ��ڵ��Ƿ��뻷���ཻ	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //���߶��뻷���ཻ����·����Ч
		return false;
	}

	return true;
}

bool RayPath::IsValidAndRectifyRefract()
{
    if (!PathTraceLite(*this))
        return false;
    return true;
}
