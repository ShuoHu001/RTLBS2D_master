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
	//�����½ڵ�
	KDTreeNode* node = new KDTreeNode();

	//����ڵ��Χ��
	node->m_bbox = computeBBox(segments);

	if (segments.size() <= leafSize) {//����ǰ�ڵ���Ԫ����С����ֵ�����ٽ��з���
		node->m_segments = segments;
		return node;
	}

	auto splitSegments = splitBySAH(segments, node->m_axis);

	//�������-����Ƿ�����ͼԪ��������һ��λ�ã����ǣ��򽫵�ǰ�ڵ�����ΪҶ�ӽڵ�,���ٽ��з���
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
			KDTreeNode* first = ray.m_Dir[node->m_axis] < 0 ? node->right : node->left;
			KDTreeNode* second = ray.m_Dir[node->m_axis] < 0 ? node->left : node->right;

			if (second && second->m_bbox.Intersect(ray))
				nodeStack.push(second);
			if (first)
				nodeStack.push(first);
		}
	}

	//������㱻���¹�������true�����򷵻�false
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
	//�������ص�ͼԪ����
	std::vector<Segment2D*> leftSegments;
	std::vector<Segment2D*> rightSegments;

	if (segments.empty())//���ָ��ͼԪΪ�գ�ֱ�ӷ���
		return { leftSegments, rightSegments };

	auto [variances, means] = computeVariances(segments);

	//ѡ�������󷽲������Ϊ�ָ���
	int axis = variances[0] > variances[1] ? 0 : 1;

	//����ѡ�������л���
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
	if (segments.empty()) {//���ָ��ͼԪΪ�գ�ֱ�ӷ���
		return std::pair<std::vector<Segment2D*>, std::vector<Segment2D*>>();
	}

	RtLbsType minSAH = std::numeric_limits<RtLbsType>::max();
	int minAxis = -1;
	int minPos = -1;
	// ���ȣ�������Ҫ��ÿ������������Ա��ܹ����ټ�����ָ�ƽ���λ��
	for (int axis = 0; axis < 2; ++axis) {
		std::sort(segments.begin(), segments.end(), [axis](const Segment2D* a, const Segment2D* b) {
			return a->GetCenter(axis) < b->GetCenter(axis);
			});

		// ���ţ����Ƕ�ÿ�����ܵķָ�λ�ü����� SAH ֵ
		for (int i = 0; i < segments.size() - 1; ++i) {
			// ���������Ҳ�İ�Χ��
			BBox2D leftBBox = ComputeBBoxByIndex(segments.begin(), segments.begin() + i + 1);
			BBox2D rightBBox = ComputeBBoxByIndex(segments.begin() + i + 1, segments.end());

			// ���������Ҳ�ı����
			RtLbsType leftArea = leftBBox.HalfSurfaceArea();
			RtLbsType rightArea = rightBBox.HalfSurfaceArea();

			// ���� SAH ֵ
			RtLbsType sah = i * leftArea + (segments.size() - i) * rightArea;
			if (sah < minSAH) {
				minSAH = sah;
				minAxis = axis;
				minPos = i;
			}
		}
	}
	splitaxis = minAxis;
	// �����ҵ�����ѷָ����λ�ã���ͼԪ�ָ�������Ӽ�
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

	for (auto it = segments.begin(); it != segments.end(); ++it) {//�����ֵ
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
