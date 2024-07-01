#include "bvh.h"
/*---------------------------------------BVHNode结构--------------------------------------------------*/
BVHNode::BVHNode(std::vector<Segment2D*> segments)
	: m_segments(segments)
	, left(nullptr)
	, right(nullptr)
{
}

BVHNode::~BVHNode()
{
}

bool BVHNode::IsLeaf() const
{
	return left == nullptr && right == nullptr;
}


/*---------------------------------------BVH结构--------------------------------------------------*/
BVH::BVH()
	: m_root(nullptr)
{
}

BVH::~BVH()
{
	deleteNode();
}

BVHNode* BVH::Construct(std::vector<Segment2D*>& segments)
{
	//创建新节点
	BVHNode* node = new BVHNode(segments);
	//计算包围盒
	node->m_bbox = computeBBox(segments);

	if (segments.size() <= leafSize) {//若当前节点面元数量小于阈值，不再进行分裂
		node->m_segments = segments;
		return node;
	}

	auto splitSegments = splitBySAH(segments);//SAH 启发式分割

	//特殊情况-检查是否所有图元都集中再一个位置，若是，则将当前节点设置为叶子节点,不再进行分裂
	if (splitSegments.first.empty() || splitSegments.second.empty()) {
		node->m_segments = segments;
		return node;
	}

	node->left = Construct(splitSegments.first);
	node->right = Construct(splitSegments.second);

	return node;
}

bool BVH::GetIntersect(const Ray2D& ray, Intersection2D* intersect) const
{
	if (!m_root)
		return false;

	std::stack<BVHNode*> nodeStack;
	nodeStack.push(m_root);

	while (!nodeStack.empty()) {
		BVHNode* node = nodeStack.top();
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
			if (node->left)
				nodeStack.push(node->left);
			if (node->right)
				nodeStack.push(node->right);
		}
	}

	//如果交点被更新过，返回true，否则返回false
	return intersect && intersect->m_ft < FLT_MAX;
}

void BVH::Build()
{
	m_root = Construct(*m_segments);
}

ACCEL_TYPE BVH::GetAccelType() const
{
	return ACCEL_BVH;
}



std::pair<std::vector<Segment2D*>, std::vector<Segment2D*>> BVH::splitByAxisAlignment(std::vector<Segment2D*>& segments)
{
	BBox2D bbox = computeBBox(segments);
	Vector2D size = bbox.m_max - bbox.m_min;
	int maxAxis = (size.x > size.y) ? 0 : 1;
	RtLbsType mid = (bbox.m_max[maxAxis] + bbox.m_min[maxAxis]) * 0.5;//获得中间轴

	std::vector<Segment2D*> leftSegments;
	std::vector<Segment2D*> rightSegments;

	for (auto it = segments.begin(); it != segments.end(); ++it) {
		Segment2D* segment = *it;
		Point2D center = (segment->m_ps + segment->m_pe) * 0.5;
		if (center[maxAxis] < mid) {
			leftSegments.push_back(segment);
		}
		else {
			rightSegments.push_back(segment);
		}
	}

	//防止所有图元都在同一侧
	if (leftSegments.empty()) {
		leftSegments = rightSegments;
		rightSegments.clear();
	}
	else if (rightSegments.empty()) {
		rightSegments = leftSegments;
		leftSegments.clear();
	}
	return { leftSegments,rightSegments };
}

std::pair<std::vector<Segment2D*>, std::vector<Segment2D*>> BVH::splitBySAH(std::vector<Segment2D*>& segments)
{
	std::vector<Segment2D*> bestLeftSegments;
	std::vector<Segment2D*> bestRightSegments;

	//对每个轴进行考察，确定最优轴
	RtLbsType bestCost = FLT_MAX;//先给最大值，后续迭代会更新
	for (int axis = 0; axis < 2; ++axis) {
		sort(segments.begin(), segments.end(), [axis](const Segment2D* a, const Segment2D* b) {
			return a->GetCenter(axis) < b->GetCenter(axis);//选取中心坐标作为比较对象
		});
		//针对每个可能的分割点，计算分割成本
		for (size_t i = 1; i < segments.size(); ++i) {
			std::vector<Segment2D*> leftSegments(segments.begin(), segments.begin() + i);
			std::vector<Segment2D*> rightSegments(segments.begin() + i, segments.end());
			BBox2D leftBBox = computeBBox(leftSegments);
			BBox2D rightBBox = computeBBox(rightSegments);
			RtLbsType cost = leftSegments.size() * leftBBox.HalfSurfaceArea() + rightSegments.size() * rightBBox.HalfSurfaceArea();//启发式面积越小，表明射线经过的面元数量越小
			//如果分割成本比之前的成本还低，则选择更好的分割点
			if (cost < bestCost) {
				bestCost = cost;
				bestLeftSegments = leftSegments;
				bestRightSegments = rightSegments;
			}
		}
	}

	return { bestLeftSegments, bestRightSegments };
}

void BVH::deleteNode()
{
	std::stack<BVHNode*> nodes;
	nodes.push(m_root);
	while (!nodes.empty()) {
		BVHNode* node = nodes.top();
		nodes.pop();
		if (node != nullptr) {
			nodes.push(node->left);
			nodes.push(node->right);
			delete node;
		}
	}
}
