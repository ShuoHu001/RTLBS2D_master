#include "Intersection2D.h"
#include "segment2d.h"

Intersection2D::Intersection2D()
	: m_type(NODE_INIT)
	, m_ft(FLT_MAX)
	, m_mat(nullptr)
	, m_segment(nullptr)
	, m_u(-FLT_MAX)
{
}

Intersection2D::Intersection2D(Point2D intersect, PATHNODETYPE type, RtLbsType t, Segment2D* segment)
	: m_intersect(intersect)
	, m_type(type)
	, m_ft(t)
	, m_mat(segment->m_mat)
	, m_segment(segment)
	, m_u(-FLT_MAX) 
{
}

Intersection2D::Intersection2D(Point2D intersect, PATHNODETYPE type, RtLbsType t)
	: m_intersect(intersect)
	, m_type(type)
	, m_ft(t)
	, m_mat(nullptr)
	, m_segment(nullptr)
	, m_u(-FLT_MAX)
{
}

Intersection2D::~Intersection2D()
{
}

bool Intersection2D::Update(const Ray2D& ray)
{
	return ValidWedges(ray);//更新*intersect,找到相交面元后再进行楔形角判定
}

Intersection2DGPU Intersection2D::Convert2GPU()
{
	Intersection2DGPU intersectGPU;
	intersectGPU.m_intersect = m_intersect;
	intersectGPU.m_type = m_type;
	intersectGPU.m_ft = m_ft;
	if (m_mat != nullptr) {
		intersectGPU.m_matId = m_mat->m_id;
	}
	if (m_segment != nullptr)
		intersectGPU.m_segmentId = m_segment->m_id;
	if (!m_wedges.empty()) { //若wedge有值，则用wedge初始值
		intersectGPU.m_wedgeId = m_wedges[0]->m_globalId;
	}
	return intersectGPU;
}

bool Intersection2D::ValidWedges(const Ray2D& ray)
{
	//针对交点为重复绕射点的特殊情况修正
	if (m_type == NODE_DIFF && ray.m_vWedge.size() != 0) {
		auto it = std::find(ray.m_vWedge.begin(), ray.m_vWedge.end(), m_wedges[0]);
		if (it != ray.m_vWedge.end()) {//在ray wedges 中找到对应的元素,与环境不相交
			return false;
		}
	}
	return true;
}
