#include "terrainprofilesegment.h"

TerrainProfileSegment::TerrainProfileSegment()
	: m_ps(nullptr)
	, m_pe(nullptr)
{
}

TerrainProfileSegment::TerrainProfileSegment(TerrainProfilePoint* ps, TerrainProfilePoint* pe)
	: m_ps(ps)
	, m_pe(pe)
{
	m_dir = pe->m_point2d - ps->m_point2d;
	m_dir.Normalize();
}

TerrainProfileSegment::TerrainProfileSegment(const TerrainProfileSegment& segment)
	: m_ps(segment.m_ps)
	, m_pe(segment.m_pe)
	, m_dir(segment.m_dir)
{
}

TerrainProfileSegment::~TerrainProfileSegment()
{
}

TerrainProfileSegment& TerrainProfileSegment::operator=(const TerrainProfileSegment& segment)
{
	m_ps = segment.m_ps;
	m_pe = segment.m_pe;
	m_dir = segment.m_dir;
	return *this;
}

bool TerrainProfileSegment::operator==(const TerrainProfileSegment& other) const
{
	if ((m_ps == other.m_ps && m_pe == other.m_pe) || (m_ps == other.m_pe && m_pe == other.m_ps)) {
		return true;
	}
	return false;
}

bool TerrainProfileSegment::operator!=(const TerrainProfileSegment& other) const
{
	return !(*this == other);
}

RtLbsType TerrainProfileSegment::GetHeight(TerrainProfilePoint* p) const
{
	RtLbsType l = p->m_point2d[0] - m_ps->m_point2d[0];
	RtLbsType L = m_pe->m_point2d[0] - m_ps->m_point2d[0];
	RtLbsType H = m_pe->m_point2d[1] - m_ps->m_point2d[1];
	RtLbsType deltaH = l / L * H; //p点增量高度
	return m_ps->m_point2d[1] + deltaH;
}

void TerrainProfileSegment::Init(TerrainProfilePoint* ps, TerrainProfilePoint* pe)
{
	m_ps = ps;
	m_pe = pe;
	m_dir = pe->m_point2d - ps->m_point2d;
	m_dir.Normalize();
}
