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
	return ValidWedges(ray);//����*intersect,�ҵ��ཻ��Ԫ���ٽ���Ш�ν��ж�
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
	if (!m_wedges.empty()) { //��wedge��ֵ������wedge��ʼֵ
		intersectGPU.m_wedgeId = m_wedges[0]->m_globalId;
	}
	return intersectGPU;
}

bool Intersection2D::ValidWedges(const Ray2D& ray)
{
	//��Խ���Ϊ�ظ������������������
	if (m_type == NODE_DIFF && ray.m_vWedge.size() != 0) {
		auto it = std::find(ray.m_vWedge.begin(), ray.m_vWedge.end(), m_wedges[0]);
		if (it != ray.m_vWedge.end()) {//��ray wedges ���ҵ���Ӧ��Ԫ��,�뻷�����ཻ
			return false;
		}
	}
	return true;
}
