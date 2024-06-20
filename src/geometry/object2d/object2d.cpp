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
	//���������߶Σ�������ͬ�ĵ㣬�������һ�ֻ��ڹ�ϣӳ��ķ���ȷ���������
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
	//����pnpoly�����ж��Ƿ��ڶ�����ڲ�
	int pSize = static_cast<int>(m_segments.size());			/** @brief	������	*/
	int i, j;
	bool c = false;
	for (i = 0, j = pSize - 1; i < pSize; j = i++) {
		const Point2D& ipoint = m_segments[i]->m_ps;			/** @brief	��i�������	*/
		const Point2D& jpoint = m_segments[j]->m_ps;			/** @brief	��j�������	*/
		if ((ipoint.y > p.y) != (jpoint.y > p.y) &&
			(p.x < (jpoint.x - ipoint.x) * (p.y - ipoint.y) / (jpoint.y - ipoint.y) + ipoint.x)) {
			c = !c;
		}
	}

	//�����ж�����object�ı��ϣ����ڲ�ͬ��
	for (auto segPtr : m_segments) {
		if (segPtr->IsInSegment(p)) {				//���ڱ��ϣ���ֱ�ӷ���true
			return true;
		}
	}


	return c;
}

bool Object2D::IsContain(const Point3D& p) const
{
	//����pnpoly�����ж��Ƿ��ڶ�����ڲ�
	int pSize = static_cast<int>(m_segments.size());			/** @brief	������	*/
	int i, j;
	bool c = false;
	for (i = 0, j = pSize - 1; i < pSize; j = i++) {
		const Point2D& ipoint = m_segments[i]->m_ps;			/** @brief	��i�������	*/
		const Point2D& jpoint = m_segments[j]->m_ps;			/** @brief	��j�������	*/
		if ((ipoint.y > p.y) != (jpoint.y > p.y) &&
			(p.x < (jpoint.x - ipoint.x) * (p.y - ipoint.y) / (jpoint.y - ipoint.y) + ipoint.x)) {
			c = !c;
		}
	}
	if (c == true) {											//�����壨����Σ��ڲ�
		if (p.z > m_height)
			return false;										//������߶ȸ��ڽ�������ڽ������ڲ�
	}
	return c;
}

RtLbsType Object2D::GetObjectHeight() const
{
	return m_foundationHeight + m_height;
}
