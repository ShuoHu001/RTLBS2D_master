#include "pathnode.h"
#include "pathnode3d.h"

PathNode::PathNode()
    : m_type(NODE_INIT)
    , m_segment(nullptr)
    , m_wedge(nullptr)
    , m_ft(static_cast<RtLbsType>(0.0))
    , m_bTraceBack(true)
{
}


PathNode::PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point)
	: m_limitInfo(limitInfo)
	, m_type(type)
	, m_point(point)
	, m_segment(nullptr)
	, m_wedge(nullptr)
    , m_ft(static_cast<RtLbsType>(0.0))
    , m_source(point)
    , m_bTraceBack(true)
{
}

PathNode::PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, const Ray2D& prevRay) //�Ӿ�ڵ�-��ֹ�ڵ�, ray Ϊָ���node�����ߣ���nodeΪ�յ㣩
    : m_limitInfo(limitInfo)
    , m_type(type)
    , m_point(point)
    , m_segment(nullptr)
    , m_wedge(nullptr)
    , m_prevRay(prevRay)
    , m_bTraceBack(true)
{
	
	m_ft = prevRay.m_fMax + (point - prevRay.m_Ori).Length();//��rootԴ�ľ���
    RtLbsType t_relative = prevRay.m_fMax - prevRay.m_fMin;
	m_source = GetRayCoordinate(prevRay, -t_relative);
}

PathNode::PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Segment2D* primitive, const Ray2D& prevRay) //������ֹ�ڵ�, ray Ϊָ���node�����ߣ���nodeΪ�յ㣩
    : m_limitInfo(limitInfo)
    , m_type(type)
    , m_point(point)
    , m_segment(primitive)
    , m_wedge(nullptr)
    , m_prevRay(prevRay)
    , m_bTraceBack(true)
{
	m_ft = prevRay.m_fMax + (point - prevRay.m_Ori).Length();//��rootԴ�ľ���
	RtLbsType t_relative = prevRay.m_fMax - prevRay.m_fMin;
	m_source = GetRayCoordinate(prevRay, -t_relative);
}

PathNode::PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Segment2D* primitive, const Ray2D& prevRay, const Ray2D& nextRay) //����ڵ�
	: m_limitInfo(limitInfo)
	, m_type(type)
	, m_point(point)
	, m_segment(primitive)
	, m_wedge(nullptr)
	, m_prevRay(prevRay)
    , m_nextRay(nextRay)
	, m_bTraceBack(true)
{
	m_ft = nextRay.m_fMax;//��rootԴ�ľ���
	RtLbsType t_relative = nextRay.m_fMax - nextRay.m_fMin;//�������һ������Դ�ľ���
	m_source = GetRayCoordinate(nextRay, -t_relative);//���Ƴ�����Դ��λ��
}

PathNode::PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Wedge2D* wedge, const Ray2D& prevRay, const Ray2D& nextRay)
	: m_limitInfo(limitInfo)
	, m_type(type)
	, m_point(point)
	, m_wedge(wedge)
	, m_segment(nullptr)
	, m_prevRay(prevRay)
    , m_nextRay(nextRay)
	, m_bTraceBack(true)
{
	m_ft = nextRay.m_fMin;//��rootԴ�ľ���
	m_source = wedge->m_point;
}



PathNode::PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Wedge2D* wedge, const Ray2D& prevRay) //������ֹ�ڵ�, ray Ϊָ���node�����ߣ���nodeΪ�յ㣩
    : m_limitInfo(limitInfo)
    , m_type(type)
    , m_point(point)
    , m_wedge(wedge)
    , m_segment(nullptr)
    , m_prevRay(prevRay)
    , m_bTraceBack(true)
{
	m_ft = prevRay.m_fMax + (point - prevRay.m_Ori).Length();//��rootԴ�ľ���
	RtLbsType t_relative = prevRay.m_fMax - prevRay.m_fMin;
	m_source = GetRayCoordinate(prevRay, -t_relative);
}

PathNode::PathNode(const PathNode& pathnode)
    : m_limitInfo(pathnode.m_limitInfo)
    , m_type(pathnode.m_type)
    , m_point(pathnode.m_point)
    , m_segment(pathnode.m_segment)
    , m_wedge(pathnode.m_wedge)
    , m_prevRay(pathnode.m_prevRay)
    , m_nextRay(pathnode.m_nextRay)
    , m_ft(pathnode.m_ft)
    , m_source(pathnode.m_source)
    , m_bTraceBack(pathnode.m_bTraceBack)
{
}

PathNode& PathNode::operator=(PathNode& node)
{
    m_limitInfo = node.m_limitInfo;
    m_type = node.m_type;
    m_point = node.m_point;
    m_segment = node.m_segment;
    m_wedge = node.m_wedge;
    m_prevRay = node.m_prevRay;
    m_nextRay = node.m_nextRay;
    m_source = node.m_source;
    m_ft = node.m_ft;
    m_bTraceBack = node.m_bTraceBack;
    return *this;//���ص�ǰ���������
}

bool PathNode::IsContainPointByAngle(Point2D p)
{
    Vector2D op = (p - m_source).Normalize();
    double costheta_op = op * m_nextRay.m_Dir;//op �����߼�ļн�����
    if (costheta_op < m_nextRay.m_costheta)//�нǴ������߰��Ž�
        return false;
    return true;
}


