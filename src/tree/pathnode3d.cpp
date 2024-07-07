#include "pathnode3d.h"

PathNode3D::PathNode3D()
	: m_depth(0)
	, m_type(NODE_INIT)
	, m_mat(nullptr)
	, m_primitive(nullptr)
	, m_wedge(nullptr)
	, m_terrainFacet(nullptr)
{
}

PathNode3D::PathNode3D(const PathNode& node, RtLbsType h)
	: m_depth(node.m_limitInfo.m_limitTotal)
	, m_type(node.m_type)
	, m_mat(node.m_mat)
	, m_primitive(node.m_segment)
	, m_wedge(node.m_wedge)
	, m_terrainFacet(nullptr)
{
	//计算并修正节点坐标
	m_point.x = node.m_point.x;
	m_point.y = node.m_point.y;
	m_point.z = h;
	m_gs2D = node.m_source;
}

PathNode3D::PathNode3D(const Point3D& point, PATHNODETYPE type, int depth)
	: m_depth(depth)
	, m_type(type)
	, m_mat(nullptr)
	, m_point(point)
	, m_primitive(nullptr)
	, m_wedge(nullptr)
	, m_terrainFacet(nullptr)
{
}

PathNode3D::PathNode3D(const Point3D& point, PATHNODETYPE type, const Segment2D* segment, int depth)
	: m_depth(depth)
	, m_type(type)
	, m_mat(segment->m_mat)
	, m_point(point)
	, m_primitive(segment)
	, m_wedge(nullptr)
	, m_terrainFacet(nullptr)
{
}

PathNode3D::PathNode3D(const Point3D& point, PATHNODETYPE type, const TerrainFacet* facet, int depth)
	: m_depth(depth)
	, m_type(type)
	, m_mat(facet->m_mat)
	, m_point(point)
	, m_primitive(nullptr)
	, m_wedge(nullptr)
	, m_terrainFacet(facet)
{
}

PathNode3D::PathNode3D(const Point3D& point, PATHNODETYPE type, const Wedge2D* wedge, int depth)
	: m_depth(depth)
	, m_type(type)
	, m_mat(wedge->m_face1->m_mat)
	, m_point(point)
	, m_primitive(nullptr)
	, m_wedge(wedge)
	, m_terrainFacet(nullptr)
{
}

PathNode3D::~PathNode3D()
{
}

void PathNode3D::ConvertBy(const PathNode& node, RtLbsType h)
{
	m_depth = node.m_limitInfo.m_limitTotal;
	m_type = node.m_type;
	m_mat = node.m_mat;
	m_primitive = node.m_segment;
	m_wedge = node.m_wedge;
	m_terrainFacet = nullptr;
	m_point = Point3D(node.m_point.x, node.m_point.y, h);
	m_gs2D = node.m_source;
	return;
}

void PathNode3D::ConvertBy(const PathNodeGPU& node, RtLbsType h, const std::vector<Segment2D*>& segments, const std::vector<Wedge2D*>& wedges)
{
	m_depth = 0;
	m_type = node.m_inter.m_type;
	m_terrainFacet = nullptr;
	int segId = node.m_inter.m_segmentId;
	int wegId = node.m_inter.m_wedgeId;
	if (segId != -1) {
		m_primitive = segments[segId];
		m_mat = m_primitive->m_mat;
	}
	if (wegId != -1) {
		m_wedge = wedges[wegId];
		m_mat = m_wedge->m_face1->m_mat;
	}
	m_point = Point3D(node.m_inter.m_intersect.x, node.m_inter.m_intersect.y, h);
	return;
}

std::string PathNode3D::ToString() const
{
	std::stringstream ss;
	if (m_type == NODE_ROOT) {
		ss << m_type << ",";
	}
	else if (m_type == NODE_REFL) {
		ss << m_type << "," << m_primitive->m_id << ",";
	}
	else if (m_type == NODE_DIFF) {
		ss << m_type << "," << m_wedge->m_globalId << ",";
	}
	return ss.str();
}
