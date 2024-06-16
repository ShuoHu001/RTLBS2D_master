#include "pathnode3d.h"

PathNode3D::PathNode3D()
	: m_depth(0)
	, m_type(NODE_INIT)
	, m_primitive(nullptr)
	, m_wedge(nullptr)
{
}

PathNode3D::PathNode3D(const PathNode& node, RtLbsType h)
	: m_depth(node.m_limitInfo.m_limitTotal)
	, m_type(node.m_type)
	, m_primitive(node.m_segment)
	, m_wedge(node.m_wedge)
	, m_terrainFacet(nullptr)
{
	//计算并修正节点坐标
	m_point.x = node.m_point.x;
	m_point.y = node.m_point.y;
	m_point.z = h;
}

PathNode3D::PathNode3D(const Point3D& point, PATHNODETYPE type, int depth)
	: m_depth(depth)
	, m_type(type)
	, m_point(point)
	, m_primitive(nullptr)
	, m_wedge(nullptr)
	, m_terrainFacet(nullptr)
{
}

PathNode3D::PathNode3D(const Point3D& point, PATHNODETYPE type, const Segment2D* segment, int depth)
	: m_depth(depth)
	, m_type(type)
	, m_point(point)
	, m_primitive(segment)
	, m_wedge(nullptr)
	, m_terrainFacet(nullptr)
{
	m_matId = segment->m_matId;
}

PathNode3D::PathNode3D(const Point3D& point, PATHNODETYPE type, const TerrainFacet* facet, int depth)
	: m_depth(depth)
	, m_type(type)
	, m_point(point)
	, m_primitive(nullptr)
	, m_wedge(nullptr)
	, m_terrainFacet(facet)
{
}

PathNode3D::PathNode3D(const Point3D& point, PATHNODETYPE type, const Wedge2D* wedge, int depth)
	: m_depth(depth)
	, m_type(type)
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
	m_primitive = node.m_segment;
	m_wedge = node.m_wedge;
	m_terrainFacet = nullptr;
	m_point = Point3D(node.m_point.x, node.m_point.y, h);
	return;
}

void PathNode3D::ConvertBy(const PathNodeGPU& node, RtLbsType h, const std::vector<Segment2D*>& segments, const std::vector<Wedge2D*>& wedges)
{
	m_depth = 0;
	m_type = node.m_inter.m_type;
	m_terrainFacet = nullptr;
	int segId = node.m_inter.m_segmentId;
	int wegId = node.m_inter.m_wedgeId;
	if (segId != -1)
		m_primitive = segments[segId];
	if (wegId != -1)
		m_wedge = wedges[wegId];
	m_point = Point3D(node.m_inter.m_intersect.x, node.m_inter.m_intersect.y, h);
	return;
}
