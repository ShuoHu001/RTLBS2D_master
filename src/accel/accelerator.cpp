#include "accelerator.h"

Accelerator::Accelerator()
	: m_segments(nullptr)
{
}

void Accelerator::SetPrimitives(std::vector<Segment2D*>* segments)
{
	m_segments = segments;
	m_bbox = computeBBox(*segments);
	m_bbox.m_min.x -= 10;
	m_bbox.m_min.y -= 10;
	m_bbox.m_max.x += 10;
	m_bbox.m_max.y += 10;
}


std::vector<Segment2D*> Accelerator::GetSegments()
{
	return *m_segments;
}

BBox2D Accelerator::GetBBox()
{
	return m_bbox;
}
