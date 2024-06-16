#include "wedge2dgpu.h"




Wedge2DGPU::Wedge2DGPU(const Wedge2DGPU& wedge)
	: m_wedge_id(wedge.m_wedge_id)
	, m_face1(wedge.m_face1)
	, m_face2(wedge.m_face2)
	, m_edge(wedge.m_edge)
	, m_fExternalAngle(wedge.m_fExternalAngle)
	, m_vector1(wedge.m_vector1)
	, m_vector2(wedge.m_vector2)
{
}

bool Wedge2DGPU::operator==(const Wedge2DGPU& other) const
{
	if (m_wedge_id != other.m_wedge_id)
		return false;
	return true;
}

bool Wedge2DGPU::operator!=(const Wedge2DGPU& other) const
{
	return !(*this == other);
}

Wedge2DGPU& Wedge2DGPU::operator=(const Wedge2DGPU& wedge)
{
	if (this != &wedge) {
		m_wedge_id = wedge.m_wedge_id;
		m_face1 = wedge.m_face1;
		m_face2 = wedge.m_face2;
		m_edge = wedge.m_edge;
		m_fExternalAngle = wedge.m_fExternalAngle;
		m_vector1 = wedge.m_vector1;
		m_vector2 = wedge.m_vector2;
	}
	return *this;
}

