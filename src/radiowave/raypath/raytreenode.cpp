#include "raytreenode.h"
#include "scene/scene.h"

RayTreeNode::RayTreeNode()
	: m_isValid(true)
	, m_data(nullptr)
	, m_pLeft(nullptr)
	, m_pRight(nullptr)
	, m_pGeneralFather(nullptr)
{
}

RayTreeNode::RayTreeNode(PathNode*& data) 
	: m_isValid(true)
	, m_data(data)
	, m_pLeft(nullptr)
	, m_pRight(nullptr)
	, m_pGeneralFather(nullptr)
{
}

RayTreeNode::~RayTreeNode()
{
	if (m_data != nullptr) {
		delete m_data;
		m_data = nullptr;
	}
}

bool RayTreeNode::IsValidLeafNode() const
{
	if (m_pLeft == nullptr) {		//���ýڵ����ӽڵ㣬��Ϊ����Ҷ�ӽڵ㣬�ýڵ���Ч
		return false;
	}
	if (m_pLeft->m_pLeft != nullptr) { //���ýڵ�
		return false;
	}
	return true;
}

void RayTreeNode::SetGeneralFatherNode(RayTreeNode* prevNode)
{
	if (m_data->m_type == NODE_ROOT || m_data->m_type == NODE_DIFF) {		//�����ѽڵ㱾��Ϊ���ڵ㣬���϶��䱾��Ϊ���常�ڵ�
		m_pGeneralFather = this;
	}
	else {																//�������������ԭ�ڵ�Ĺ��常�ڵ�
		m_pGeneralFather = prevNode->m_pGeneralFather;
	}
}

bool RayTreeNode::IsCaptureByPoint(const Point2D& p, RtLbsType splitRadius, RayTreeNode* prev_treenode) 
{
	if (!m_isValid)						//����ǰ�ڵ���Ч����ֱ�ӷ���false
		return false;
	if (prev_treenode == nullptr)//����ǰ�ڵ�Ϊ���ڵ㣬�����м�֦
		return false;
	if (m_data->m_type == NODE_DIFF)			//����ǰ�ڵ�Ϊ����ڵ㣬�򲻽��н����ж���������·��Ϊ�������·�����Ѵ������߹ܣ�
		return false;
	PathNode* cur_node = m_data;
	PathNode* prev_node = prev_treenode->m_data;

	Vector2D op = p - prev_node->m_source ;   //����Դ����ǰ�ڵ�����������������ǰһ���ڵ������ֵ 
	RtLbsType t_op = op.Length();				//��ǰ�ڵ�������Դ�ľ���
	RtLbsType t = t_op + cur_node->m_prevRay.m_tMin;//����ΪrootԴ�ľ���  p������Դ����+����Դ���ڽڵ�������ߵ���С����ֵ
	RtLbsType tmin = prev_node->m_ft;//������Сֵ
	RtLbsType tmax = cur_node->m_ft + splitRadius;//�������ֵ ��������splitRadius(����)

	//20240521 �����������tֵ���ж�



	if (t > tmax || t < tmin) {//���ڷ�Χ��
		return false;
	}
		
	//����-1 ���нǶ��ж�
	
	//const Ray2D& ray = cur_node.m_prevRay;
	//double costheta_op = ray.m_Dir * op / t;
	//if (acos(costheta_op) > ray.m_theta)
	//	return false;
	//����-2 ���������ж�
	const Ray2D& ray = cur_node->m_prevRay;
	double costheta_op = ray.m_Dir * op / t_op;//��������ֵ
	if (costheta_op < ray.m_costheta)//�������߹���
		return false;
	return true;
}
