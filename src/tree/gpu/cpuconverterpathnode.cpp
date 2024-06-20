#include "cpuconverterpathnode.h"

CPUConverterPathNode::CPUConverterPathNode()
	: m_fatherNodeId(-1)
	, m_layerId(-1)
	, m_sensorId(-1)
	, m_type(NODE_INIT)
	, m_segmentId(-1)
	, m_wedgeId(-1)
	, m_ft(0.0)
{
}

CPUConverterPathNode::CPUConverterPathNode(const PathNode& node, int fatherNodeId, int sensorId, int layerId)
	: m_fatherNodeId(fatherNodeId)
	, m_layerId(layerId)
	, m_sensorId(sensorId)
	, m_type(node.m_type)
	, m_point(node.m_point)
	, m_segmentId(-1)
	, m_wedgeId(-1)
	, m_generalSource(node.m_source)
	, m_ft(node.m_ft)
{
	if (node.m_segment != nullptr) {
		m_segmentId = node.m_segment->m_id;
	}
	if (node.m_wedge != nullptr) {
		m_wedgeId = node.m_wedge->m_globalId;
	}
	m_prevRay = node.m_prevRay.Convert2GPU();
}

CPUConverterPathNode::CPUConverterPathNode(const CPUConverterPathNode& node)
	: m_fatherNodeId(node.m_fatherNodeId)
	, m_layerId(node.m_layerId)
	, m_sensorId(node.m_sensorId)
	, m_type(node.m_type)
	, m_point(node.m_point)
	, m_segmentId(node.m_segmentId)
	, m_wedgeId(node.m_wedgeId)
	, m_prevRay(node.m_prevRay)
	, m_generalSource(node.m_generalSource)
	, m_ft(node.m_ft)
{
}

CPUConverterPathNode::~CPUConverterPathNode()
{
}

CPUConverterPathNode& CPUConverterPathNode::operator=(const CPUConverterPathNode& node)
{
	m_fatherNodeId = node.m_fatherNodeId;
	m_layerId = node.m_layerId;
	m_sensorId = node.m_sensorId;
	m_type = node.m_type;
	m_point = node.m_point;
	m_segmentId = node.m_segmentId;
	m_wedgeId = node.m_wedgeId;
	m_prevRay = node.m_prevRay;
	m_generalSource = node.m_generalSource;
	m_ft = node.m_ft;
	return *this;
}

bool CPUConverterPathNode::IsCapturedByPoint(const Point2D& p, RtLbsType splitRadius, CPUConverterPathNode* fatherNode) const
{
	if (m_type == NODE_ROOT || m_type == NODE_DIFF) {							//���ڵ������ڵ㲻�������ж�
		return false;
	}
	Vector2D op = p - fatherNode->m_generalSource;								/** @brief	����Դ����ǰ�ڵ������	*/
	RtLbsType t_op = op.Length();												/** @brief	��ǰ�ڵ�������Դ�ڵ�ľ���	*/
	RtLbsType t = t_op + m_prevRay.m_fMin;										/** @brief	��ǰ�ڵ㵽���ڵ�ľ���	*/

	RtLbsType tmin = fatherNode->m_ft;											/** @brief	�ڵ�����Сֵ	*/
	RtLbsType tmax = m_ft + splitRadius;										/** @brief	�ڵ������ֵ	*/

	if (t > tmax || t < tmin) {//���ڷ�Χ��
		return false;
	}

	
	double costheta_op = m_prevRay.m_Dir * op / t_op;//��������ֵ
	if (costheta_op < m_prevRay.m_costheta)//�������߹���
		return false;
	
	return true;
}
