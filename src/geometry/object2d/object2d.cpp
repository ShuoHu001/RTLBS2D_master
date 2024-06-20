#include "object2d.h"

Object2D::Object2D()
	: m_objectId(-1)
	, m_category(BUILDING2D)
	, m_matId(-1)
	, m_height(0.0)
	, m_foundationHeight(0.0)
{
}

Object2D::~Object2D()
{
	for (auto it = m_segments.begin(); it != m_segments.end(); ++it) {
		delete* it;
	}
	m_segments.clear();

	for (auto it = m_wedges.begin(); it != m_wedges.end(); ++it) {
		delete* it;
	}
	m_wedges.clear();
}

void Object2D::SetSegments(const std::vector<Segment2D*>& segments)
{
	m_segments.resize(segments.size());
	for (int i = 0; i < segments.size(); ++i) {
		m_segments[i] = segments[i];
	}
}

void Object2D::InitWedges()
{
	//遍历所有线段，计算相同的点，这里采用一种基于哈希映射的方法确定公共棱边
	int wedgeNum = 0;
	for (int i = 0; i < m_segments.size(); ++i) {
		for (int j = i + 1; j < m_segments.size(); ++j) {
			if (m_segments[i]->m_ps == m_segments[j]->m_ps || m_segments[i]->m_ps == m_segments[j]->m_pe) {
				Wedge2D* wedge = new Wedge2D(wedgeNum++, m_segments[i], m_segments[j], m_segments[i]->m_ps);
				m_wedges.push_back(wedge);
			}
			if (m_segments[i]->m_pe == m_segments[j]->m_ps || m_segments[i]->m_pe == m_segments[j]->m_pe) {
				Wedge2D* wedge = new Wedge2D(wedgeNum++, m_segments[i], m_segments[j], m_segments[i]->m_pe);
				m_wedges.push_back(wedge);
			}
		}
	}
}

bool Object2D::IsContain(const Point2D& p) const
{
	//基于pnpoly方法判定是否在多边形内部
	int pSize = static_cast<int>(m_segments.size());			/** @brief	点数量	*/
	int i, j;
	bool c = false;
	for (i = 0, j = pSize - 1; i < pSize; j = i++) {
		const Point2D& ipoint = m_segments[i]->m_ps;			/** @brief	第i个坐标点	*/
		const Point2D& jpoint = m_segments[j]->m_ps;			/** @brief	第j个坐标点	*/
		if ((ipoint.y > p.y) != (jpoint.y > p.y) &&
			(p.x < (jpoint.x - ipoint.x) * (p.y - ipoint.y) / (jpoint.y - ipoint.y) + ipoint.x)) {
			c = !c;
		}
	}

	//增加判定点在object的边上（与内部同理）
	for (auto segPtr : m_segments) {
		if (segPtr->IsInSegment(p)) {				//若在边上，则直接返回true
			return true;
		}
	}


	return c;
}

bool Object2D::IsContain(const Point3D& p) const
{
	//基于pnpoly方法判定是否在多边形内部
	int pSize = static_cast<int>(m_segments.size());			/** @brief	点数量	*/
	int i, j;
	bool c = false;
	for (i = 0, j = pSize - 1; i < pSize; j = i++) {
		const Point2D& ipoint = m_segments[i]->m_ps;			/** @brief	第i个坐标点	*/
		const Point2D& jpoint = m_segments[j]->m_ps;			/** @brief	第j个坐标点	*/
		if ((ipoint.y > p.y) != (jpoint.y > p.y) &&
			(p.x < (jpoint.x - ipoint.x) * (p.y - ipoint.y) / (jpoint.y - ipoint.y) + ipoint.x)) {
			c = !c;
		}
	}
	if (c == true) {											//在物体（多边形）内部
		if (p.z > m_height)
			return false;										//若物体高度高于建筑物，则不在建筑物内部
	}
	return c;
}

RtLbsType Object2D::GetObjectHeight() const
{
	return m_foundationHeight + m_height;
}
