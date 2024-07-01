#include "bvh.h"
/*---------------------------------------BVHNode�ṹ--------------------------------------------------*/
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


/*---------------------------------------BVH�ṹ--------------------------------------------------*/
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
	//�����½ڵ�
	BVHNode* node = new BVHNode(segments);
	//�����Χ��
	node->m_bbox = computeBBox(segments);

	if (segments.size() <= leafSize) {//����ǰ�ڵ���Ԫ����С����ֵ�����ٽ��з���
		node->m_segments = segments;
		return node;
	}

	auto splitSegments = splitBySAH(segments);//SAH ����ʽ�ָ�

	//�������-����Ƿ�����ͼԪ��������һ��λ�ã����ǣ��򽫵�ǰ�ڵ�����ΪҶ�ӽڵ�,���ٽ��з���
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

		// ��������뵱ǰ�ڵ�İ�Χ�в��ཻ���������ýڵ�
		if (!node->m_bbox.Intersect(ray))
			continue;
		if (node->IsLeaf()) {//��ǰ�ڵ�ΪҶ�ӽڵ㣬�������������Ԫ���Ƿ��ཻ
			const std::vector<Segment2D*>& segments = node->m_segments;
			for (auto it = segments.begin(); it != segments.end(); ++it) {
				Segment2D* segment = *it;
				Intersection2D tempIntersect;
				if (segment->GetIntersect(ray, &tempIntersect)) {//����ǰ��������Ԫ�н���
					if (!intersect)//��ֻ�ж��ཻ��ֱ�ӷ���true
						return true;
					if (tempIntersect.m_ft < intersect->m_ft) {
						*intersect = tempIntersect;
					}
				}

			}
		}
		else {//������Ҷ�ӽڵ㣬�����ӽڵ������ջ��
			if (node->left)
				nodeStack.push(node->left);
			if (node->right)
				nodeStack.push(node->right);
		}
	}

	//������㱻���¹�������true�����򷵻�false
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
	RtLbsType mid = (bbox.m_max[maxAxis] + bbox.m_min[maxAxis]) * 0.5;//����м���

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

	//��ֹ����ͼԪ����ͬһ��
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

	//��ÿ������п��죬ȷ��������
	RtLbsType bestCost = FLT_MAX;//�ȸ����ֵ���������������
	for (int axis = 0; axis < 2; ++axis) {
		sort(segments.begin(), segments.end(), [axis](const Segment2D* a, const Segment2D* b) {
			return a->GetCenter(axis) < b->GetCenter(axis);//ѡȡ����������Ϊ�Ƚ϶���
		});
		//���ÿ�����ܵķָ�㣬����ָ�ɱ�
		for (size_t i = 1; i < segments.size(); ++i) {
			std::vector<Segment2D*> leftSegments(segments.begin(), segments.begin() + i);
			std::vector<Segment2D*> rightSegments(segments.begin() + i, segments.end());
			BBox2D leftBBox = computeBBox(leftSegments);
			BBox2D rightBBox = computeBBox(rightSegments);
			RtLbsType cost = leftSegments.size() * leftBBox.HalfSurfaceArea() + rightSegments.size() * rightBBox.HalfSurfaceArea();//����ʽ���ԽС���������߾�������Ԫ����ԽС
			//����ָ�ɱ���֮ǰ�ĳɱ����ͣ���ѡ����õķָ��
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
