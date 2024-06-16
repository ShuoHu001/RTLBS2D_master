#include "kdtree.h"

KDTreeNode::KDTreeNode()
	: m_axis(0)
	, left(nullptr)
	, right(nullptr)
{
}

KDTreeNode::~KDTreeNode()
{
	delete left;
	delete right;
}

bool KDTreeNode::IsLeaf() const
{
	return left == nullptr && right == nullptr;
}

KDTree::KDTree()
	: m_root(nullptr)
{
}

KDTree::~KDTree()
{
	deleteNode();
}

KDTreeNode* KDTree::Construct(std::vector<Segment2D*>& segments)
{
	//创建新节点
	KDTreeNode* node = new KDTreeNode();

	//计算节点包围盒
	node->m_bbox = computeBBox(segments);

	if (segments.size() <= leafSize) {//若当前节点面元数量小于阈值，不再进行分裂
		node->m_segments = segments;
		return node;
	}

	auto splitSegments = splitBySAH(segments, node->m_axis);

	//特殊情况-检查是否所有图元都集中再一个位置，若是，则将当前节点设置为叶子节点,不再进行分裂
	if (splitSegments.first.empty() || splitSegments.second.empty()) {
		node->m_segments = segments;
		return node;
	}
	
	node->left = Construct(splitSegments.first);
	node->right = Construct(splitSegments.second);

	return node;
}

bool KDTree::GetIntersect(const Ray2D& ray, Intersection2D* intersect) const 
{
	if (!m_root)
		return false;

	std::stack<KDTreeNode*> nodeStack;
	nodeStack.push(m_root);

	while (!nodeStack.empty()) {
		KDTreeNode* node = nodeStack.top();
		nodeStack.pop();

		// 如果射线与当前节点的包围盒不相交，则跳过该节点
		if (!node->m_bbox.Intersect(ray))
			continue;
		if (node->IsLeaf()) {//当前节点为叶子节点，检测射线与其中元素是否都相交
			const std::vector<Segment2D*>& segments = node->m_segments;
			for (auto it = segments.begin(); it != segments.end(); ++it) {
				Segment2D* segment = *it;
				Intersection2D tempIntersect;
				if (segment->GetIntersect(ray, &tempIntersect)) {//若当前射线与面元有交点
					if (!intersect)//若只判断相交，直接返回true
						return true;
					if (tempIntersect.m_ft < intersect->m_ft) {
						*intersect = tempIntersect;
					}
				}

			}
		}
		else {//若不是叶子节点，则将其子节点纳入堆栈中
			KDTreeNode* first = ray.m_Dir[node->m_axis] < 0 ? node->right : node->left;
			KDTreeNode* second = ray.m_Dir[node->m_axis] < 0 ? node->left : node->right;

			if (second && second->m_bbox.Intersect(ray))
				nodeStack.push(second);
			if (first)
				nodeStack.push(first);
		}
	}

	//如果交点被更新过，返回true，否则返回false
	return intersect && intersect->m_ft < FLT_MAX;
}

void KDTree::Build()
{
	m_root = Construct(*m_segments);
}

ACCEL_TYPE KDTree::GetAccelType() const
{
	return ACCEL_KD;
}

std::pair<std::vector<Segment2D*>, std::vector<Segment2D*>> KDTree::splitByMiddlePoint(std::vector<Segment2D*>& segments)
{
	//创建返回的图元集合
	std::vector<Segment2D*> leftSegments;
	std::vector<Segment2D*> rightSegments;

	if (segments.empty())//若分割的图元为空，直接返回
		return { leftSegments, rightSegments };

	auto [variances, means] = computeVariances(segments);

	//选择具有最大方差的轴作为分割轴
	int axis = variances[0] > variances[1] ? 0 : 1;

	//基于选择的轴进行划分
	RtLbsType midPoint = means[axis];
	for (auto it = segments.begin(); it != segments.end(); ++it) {
		Segment2D* segment = *it;
		if ((segment->m_ps[axis] + segment->m_pe[axis]) * 0.5 < midPoint) {
			leftSegments.push_back(segment);
		}
		else {
			rightSegments.push_back(segment);
		}
	}

	return {leftSegments, rightSegments};
}

std::pair<std::vector<Segment2D*>, std::vector<Segment2D*>> KDTree::splitBySAH(std::vector<Segment2D*>& segments, int& splitaxis)
{
	if (segments.empty()) {//若分割的图元为空，直接返回
		return std::pair<std::vector<Segment2D*>, std::vector<Segment2D*>>();
	}

	RtLbsType minSAH = std::numeric_limits<RtLbsType>::max();
	int minAxis = -1;
	int minPos = -1;
	// 首先，我们需要对每个轴进行排序，以便能够快速计算出分割平面的位置
	for (int axis = 0; axis < 2; ++axis) {
		std::sort(segments.begin(), segments.end(), [axis](const Segment2D* a, const Segment2D* b) {
			return a->GetCenter(axis) < b->GetCenter(axis);
			});

		// 接着，我们对每个可能的分割位置计算其 SAH 值
		for (int i = 0; i < segments.size() - 1; ++i) {
			// 计算左侧和右侧的包围盒
			BBox2D leftBBox = ComputeBBoxByIndex(segments.begin(), segments.begin() + i + 1);
			BBox2D rightBBox = ComputeBBoxByIndex(segments.begin() + i + 1, segments.end());

			// 计算左侧和右侧的表面积
			RtLbsType leftArea = leftBBox.HalfSurfaceArea();
			RtLbsType rightArea = rightBBox.HalfSurfaceArea();

			// 计算 SAH 值
			RtLbsType sah = i * leftArea + (segments.size() - i) * rightArea;
			if (sah < minSAH) {
				minSAH = sah;
				minAxis = axis;
				minPos = i;
			}
		}
	}
	splitaxis = minAxis;
	// 根据找到的最佳分割轴和位置，将图元分割成两个子集
	std::sort(segments.begin(), segments.end(), [minAxis](const Segment2D* a, const Segment2D* b) {
		return a->GetCenter(minAxis) < b->GetCenter(minAxis);
		});

	std::vector<Segment2D*> leftSegments(segments.begin(), segments.begin() + minPos + 1);
	std::vector<Segment2D*> rightSegments(segments.begin() + minPos + 1, segments.end());

	return { leftSegments, rightSegments };
}

std::pair<RtLbsType*, RtLbsType*> KDTree::computeVariances(const std::vector<Segment2D*>& segments)
{
	RtLbsType variances[2] = { 0, 0 };
	RtLbsType means[2] = { 0, 0 };

	for (auto it = segments.begin(); it != segments.end(); ++it) {//计算均值
		Segment2D* segment = *it;
		Point2D midPoint = (segment->m_ps + segment->m_pe) * 0.5;
		means[0] += midPoint.x;
		means[1] += midPoint.y;
	}
	means[0] /= segments.size();
	means[1] /= segments.size();

	for (auto it = segments.begin(); it != segments.end(); ++it) {
		Segment2D* segment = *it;
		Point2D midPoint = (segment->m_ps + segment->m_pe) * 0.5;
		RtLbsType dx = midPoint.x - means[0];
		RtLbsType dy = midPoint.y - means[1];
		variances[0] += dx * dx;
		variances[1] += dy * dy;
	}
	variances[0] /= segments.size();
	variances[1] /= segments.size();
	return std::make_pair(variances, means);
}

void KDTree::deleteNode()
{
	std::stack<KDTreeNode*> nodes;
	nodes.push(m_root);
	while (!nodes.empty()) {
		KDTreeNode* node = nodes.top();
		nodes.pop();
		if (node != nullptr) {
			nodes.push(node->left);
			nodes.push(node->right);
			delete node;
		}
	}
}
