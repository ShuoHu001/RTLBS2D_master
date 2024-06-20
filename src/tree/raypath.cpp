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

bool RayPath::IsValidAndRectifyCommon(const Point2D& p, const Scene* scene)//反向算法进行路径修正
{
	//整体思路是从末尾节点出发，修正到前面各个广义源的路径
	if (m_nodes.empty())
		return false;
	PathNode*& endNode = m_nodes.back(); endNode->m_point = p;//更新末尾节点的节点坐标
	Point2D endPoint = p;//末尾节点的坐标
	Ray2D t_ray;//用于相交测试的ray
	Intersection2D t_inter;//用于相交测试的intersect

	for (auto it = std::next(m_nodes.rbegin()); it != m_nodes.rend(); ++it) { //由于透射具有发散性，采用特殊方法进行校正，需要修改算法
		PathNode*& curNode = *it;
		if (curNode->m_type == NODE_ROOT)
			continue;
		if (curNode->m_type == NODE_DIFF) {//广义源节点
			endPoint = curNode->m_point;//更新末尾节点坐标，进行新一轮的迭代计算
			continue;
		}
		if (curNode->m_type == NODE_REFL) {//反射节点
			//1-求解广义源
			Point2D& vSource = curNode->m_source;
			t_ray.m_Ori = vSource;
			t_ray.m_Dir = (endPoint - vSource).Normalize();
			Segment2D* segment = curNode->m_segment;
			if (!segment->GetIntersect(t_ray, &t_inter))//不存在交点，该条射线不存在
				return false;
			curNode->m_point = t_inter.m_intersect;//更新当前节点的交点信息
			endPoint = curNode->m_point;//下一次迭代采用当前更新过的节点坐标
			continue;
		}
	}

	//20240521增加判定末尾和前节点构成的线段与环境相交，若相交且不是节点所在的面元，则路径无效
	Segment2D rayTestSegment(m_nodes[m_nodes.size() - 1]->m_point, m_nodes[m_nodes.size() - 2]->m_point);               /** @brief	用于检测节点是否与环境相交	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //若线段与环境相交，则路径无效
		return false;
	}

	return true;
}

bool RayPath::IsValidAndRectifyRefractMixed(const Point2D& p, const Scene* scene)
{
	//总体思路：
	// 1.将总路径分为常规路径组合透射路径组，分水岭为广义源；
	// 2.常规路径按照一般方法进行
	// 3.透射路径按照发射射线方法进行

	if (m_nodes.empty())
		return false;
	m_nodes.back()->m_point = p;//更新末尾节点的节点坐标

	std::vector<RayPath> splitpaths; //分离的路径
	std::vector<PathNode*> nodes;
	bool containRefract = false;
	//1-将path按照广义源进行分段
	for (auto it = m_nodes.begin(); it != m_nodes.end(); ++it) {
		PathNode*& node = *it;
		nodes.push_back(node);
		if (node->m_type == NODE_DIFF) { //当前节点为广义源节点，
			RayPath path(nodes, containRefract);
			splitpaths.push_back(path);
			nodes.clear();
			nodes.push_back(node);//重置nodes
			containRefract = false;//重置透射路径包含状态
			continue;
		}
		if (std::next(it) == m_nodes.end()) {//尾部节点
			RayPath path(nodes, containRefract);
			splitpaths.push_back(path);
			nodes.clear(); nodes.shrink_to_fit();//释放内存
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

	//2-将不同的路径进行不同的处理方法,针对包含透射的路径，认为末尾节点(STOP节点或者是DIFF节点)为其需要优化的节点

	for (auto it = splitpaths.begin(); it != splitpaths.end(); ++it) {
		RayPath& path = *it;
		PathNode*& endNode = path.m_nodes.back();
		if (!path.m_bContainRefract) {//非透射路径
			if (!path.IsValidAndRectifyCommon(endNode->m_point, scene))//常规路径不满足条件直接退出处理程序
				return false;
			continue;
		}
		//透射路径修正模块
		if (!path.IsValidAndRectifyRefract())
			return false;
		continue;
	}

	//3-将修正完成的路径进行拼接合成
	std::vector<PathNode*> newNodes;
	for (const RayPath& newPath : splitpaths) {
		if (&newPath != &splitpaths.back()) {//不是最后一段路径
			for (auto it = newPath.m_nodes.begin(); it != prev(newPath.m_nodes.end()); ++it) {//舍弃最后一个元素
				PathNode* node = *it;
				newNodes.push_back(node);
			}
			continue;
		}
		//是最后一段路径
		for (auto it = newPath.m_nodes.begin(); it != newPath.m_nodes.end(); ++it) {//保留最后一个元素
			PathNode* node = *it;
			newNodes.push_back(node);
		}
	}
	m_nodes = newNodes;//更新路径，完成透射路径的修正

	//20240521增加判定末尾和前节点构成的线段与环境相交，若相交且不是节点所在的面元，则路径无效
	Segment2D rayTestSegment(m_nodes[m_nodes.size() - 1]->m_point, m_nodes[m_nodes.size() - 2]->m_point);               /** @brief	用于检测节点是否与环境相交	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //若线段与环境相交，则路径无效
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
