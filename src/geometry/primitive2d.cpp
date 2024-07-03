#include "primitive2d.h"

Primitive2D::Primitive2D()
	: m_id(-1)
	, m_objectId(-1)
	, m_objectCategory(BUILDING2D)
	, m_mat(nullptr)
	, m_refractN(static_cast<RtLbsType>(1.0))
	, m_refractNOut(static_cast<RtLbsType>(1.0))
{
}

Primitive2D::Primitive2D(int id, int matId, RtLbsType refractN)
	: m_id(id)
	, m_objectId(-1)
	, m_objectCategory(BUILDING2D)
	, m_mat(nullptr)
	, m_refractN(refractN)
	, m_refractNOut(static_cast<RtLbsType>(1.0))
{
}

Primitive2D::Primitive2D(int id, int matId, RtLbsType refractN, RtLbsType refractNOut)
	: m_id(id)
	, m_objectId(-1)
	, m_objectCategory(BUILDING2D)
	, m_mat(nullptr)
	, m_refractN(refractN)
	, m_refractNOut(refractNOut)
{
}

void Primitive2D::CLearBBoxCache()
{
	
}
