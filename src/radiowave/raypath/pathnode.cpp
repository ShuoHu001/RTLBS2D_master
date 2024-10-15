#include "pathnode.h"
#include "pathnode3d.h"
#include "gpu/cpuconverterpathnode.h"

PathNode::PathNode()
    : m_type(NODE_INIT)
    , m_mat(nullptr)
    , m_segment(nullptr)
    , m_wedge(nullptr)
    , m_ft(0.0)
    , m_fatherNodeId(-1)
{
}

PathNode::PathNode(const Point2D& point, PATHNODETYPE type)
	: m_type(type)
	, m_point(point)
    , m_mat(nullptr)
	, m_segment(nullptr)
	, m_wedge(nullptr)
	, m_ft(static_cast<RtLbsType>(0.0))
	, m_source(point)
	, m_fatherNodeId(-1)
{
}


PathNode::PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point)
	: m_limitInfo(limitInfo)
	, m_type(type)
	, m_point(point)
    , m_mat(nullptr)
	, m_segment(nullptr)
	, m_wedge(nullptr)
    , m_ft(static_cast<RtLbsType>(0.0))
    , m_source(point)
    , m_fatherNodeId(-1)
{
}

PathNode::PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, const Ray2D& prevRay) //视距节点-终止节点, ray 为指向该node的射线（该node为终点）
    : m_limitInfo(limitInfo)
    , m_type(type)
    , m_point(point)
    , m_mat(nullptr)
    , m_segment(nullptr)
    , m_wedge(nullptr)
    , m_prevRay(prevRay)
    , m_fatherNodeId(-1)
{
	
	m_ft = prevRay.m_fMax + (point - prevRay.m_Ori).Length();//到root源的距离
    RtLbsType t_relative = prevRay.m_fMax - prevRay.m_fMin;
	m_source = GetRayCoordinate(prevRay, -t_relative);
}

PathNode::PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Segment2D* primitive, const Ray2D& prevRay) //反射终止节点, ray 为指向该node的射线（该node为终点）
    : m_limitInfo(limitInfo)
    , m_type(type)
    , m_mat(primitive->m_mat)
    , m_point(point)
    , m_segment(primitive)
    , m_wedge(nullptr)
    , m_prevRay(prevRay)
    , m_fatherNodeId(-1)
{
	m_ft = prevRay.m_fMax + (point - prevRay.m_Ori).Length();//到root源的距离
	RtLbsType t_relative = prevRay.m_fMax - prevRay.m_fMin;
	m_source = GetRayCoordinate(prevRay, -t_relative);
}

PathNode::PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Segment2D* primitive, const Ray2D& prevRay, const Ray2D& nextRay) //反射节点
	: m_limitInfo(limitInfo)
	, m_type(type)
    , m_mat(primitive->m_mat)
	, m_point(point)
	, m_segment(primitive)
	, m_wedge(nullptr)
	, m_prevRay(prevRay)
    , m_nextRay(nextRay)
    , m_fatherNodeId(-1)
{
	m_ft = nextRay.m_fMax;//到root源的距离
	RtLbsType t_relative = nextRay.m_fMax - nextRay.m_fMin;//相对于上一个广义源的距离
	m_source = GetRayCoordinate(nextRay, -t_relative);//倒推出广义源的位置
}

PathNode::PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Wedge2D* wedge, const Ray2D& prevRay, const Ray2D& nextRay)
	: m_limitInfo(limitInfo)
	, m_type(type)
    , m_mat(wedge->m_face1->m_mat)
	, m_point(point)
	, m_wedge(wedge)
	, m_segment(nullptr)
	, m_prevRay(prevRay)
    , m_nextRay(nextRay)
    , m_fatherNodeId(-1)
{
	m_ft = nextRay.m_fMin;//到root源的距离
	m_source = wedge->m_point;
}



PathNode::PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Wedge2D* wedge, const Ray2D& prevRay) //绕射终止节点, ray 为指向该node的射线（该node为终点）
    : m_limitInfo(limitInfo)
    , m_type(type)
    , m_mat(wedge->m_face1->m_mat)
    , m_point(point)
    , m_wedge(wedge)
    , m_segment(nullptr)
    , m_prevRay(prevRay)
    , m_fatherNodeId(-1)
{
	m_ft = prevRay.m_fMax + (point - prevRay.m_Ori).Length();//到root源的距离
	RtLbsType t_relative = prevRay.m_fMax - prevRay.m_fMin;
	m_source = GetRayCoordinate(prevRay, -t_relative);
}

PathNode::PathNode(const PathNode& pathnode)
    : m_limitInfo(pathnode.m_limitInfo)
    , m_type(pathnode.m_type)
    , m_mat(pathnode.m_mat)
    , m_point(pathnode.m_point)
    , m_segment(pathnode.m_segment)
    , m_wedge(pathnode.m_wedge)
    , m_prevRay(pathnode.m_prevRay)
    , m_nextRay(pathnode.m_nextRay)
    , m_ft(pathnode.m_ft)
    , m_source(pathnode.m_source)
    , m_fatherNodeId(pathnode.m_fatherNodeId)
{
}

PathNode& PathNode::operator=(const PathNode& node)
{
    m_limitInfo = node.m_limitInfo;
    m_type = node.m_type;
    m_mat = node.m_mat;
    m_point = node.m_point;
    m_segment = node.m_segment;
    m_wedge = node.m_wedge;
    m_prevRay = node.m_prevRay;
    m_nextRay = node.m_nextRay;
    m_source = node.m_source;
    m_ft = node.m_ft;
    m_fatherNodeId = node.m_fatherNodeId;
    return *this;//返回当前对象的引用
}

bool PathNode::IsContainPointByAngle(Point2D p)
{
    Vector2D op = (p - m_source).Normalize();
    double costheta_op = op * m_nextRay.m_Dir;//op 与射线间的夹角余弦
    if (costheta_op < m_nextRay.m_costheta)//夹角大于射线半张角
        return false;
    return true;
}

void PathNode::ConvertFrom(const CPUConverterPathNode& node, const std::vector<Segment2D*>& segments, const std::vector<Wedge2D*>& wedges)
{
    m_type = node.m_type;
    m_point = node.m_point;
    m_source = node.m_generalSource;
    m_ft = node.m_ft;
	if (node.m_segmentId != -1) {
		m_segment = segments[node.m_segmentId];
        m_mat = m_segment->m_mat;
	}
	if (node.m_wedgeId != -1) {
		m_wedge = wedges[node.m_wedgeId];
        m_mat = m_wedge->m_face1->m_mat;
	}
	m_prevRay = node.m_prevRay;
    m_fatherNodeId = -1;
}


