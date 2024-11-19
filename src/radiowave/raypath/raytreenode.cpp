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
	if (m_pLeft == nullptr) {		//若该节点无子节点，则为真正叶子节点，该节点无效
		return false;
	}
	if (m_pLeft->m_pLeft != nullptr) { //若该节点
		return false;
	}
	return true;
}

void RayTreeNode::SetGeneralFatherNode(RayTreeNode* prevNode)
{
	if (m_data->m_type == NODE_ROOT || m_data->m_type == NODE_DIFF) {		//若分裂节点本身为根节点，则认定其本身为广义父节点
		m_pGeneralFather = this;
	}
	else {																//其余情况均复制原节点的广义父节点
		m_pGeneralFather = prevNode->m_pGeneralFather;
	}
}

bool RayTreeNode::IsCaptureByPoint(const Point2D& p, RtLbsType splitRadius, RayTreeNode* prev_treenode) 
{
	if (!m_isValid)						//若当前节点无效，则直接返回false
		return false;
	if (prev_treenode == nullptr)//若当前节点为根节点，不进行剪枝
		return false;
	if (m_data->m_type == NODE_DIFF)			//若当前节点为绕射节点，则不进行接收判定（因绕射路径为多余出的路径，已存在射线管）
		return false;
	PathNode* cur_node = m_data;
	PathNode* prev_node = prev_treenode->m_data;

	Vector2D op = p - prev_node->m_source ;   //广义源到当前节点的向量，这里必须用前一个节点的坐标值 
	RtLbsType t_op = op.Length();				//当前节点距离广义源的距离
	RtLbsType t = t_op + cur_node->m_prevRay.m_tMin;//修正为root源的距离  p到广义源距离+广义源所在节点出射射线的最小距离值
	RtLbsType tmin = prev_node->m_ft;//距离最小值
	RtLbsType tmax = cur_node->m_ft + splitRadius;//距离最大值 增加修正splitRadius(近似)

	//20240521 增加修正最大t值的判定



	if (t > tmax || t < tmin) {//不在范围内
		return false;
	}
		
	//方案-1 进行角度判定
	
	//const Ray2D& ray = cur_node.m_prevRay;
	//double costheta_op = ray.m_Dir * op / t;
	//if (acos(costheta_op) > ray.m_theta)
	//	return false;
	//方案-2 进行余弦判定
	const Ray2D& ray = cur_node->m_prevRay;
	double costheta_op = ray.m_Dir * op / t_op;//计算余弦值
	if (costheta_op < ray.m_costheta)//不在射线管内
		return false;
	return true;
}
