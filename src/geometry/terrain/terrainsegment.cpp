#include "terrainsegment.h"
#include "terrainfacet.h"

TerrainSegment::TerrainSegment()
    : m_segmentId(-1)
	, m_isShared(false)
	, m_ps(nullptr)
	, m_pe(nullptr)
    , m_facet1(nullptr)
    , m_facet2(nullptr)
{
}

TerrainSegment::TerrainSegment(int id, Point3D* ps, Point3D* pe, TerrainFacet* facet1)
	: m_segmentId(id)
	, m_isShared(false)
	, m_ps(ps)
	, m_pe(pe)
	, m_facet1(facet1)
	, m_facet2(nullptr)
{
	m_dir = (*pe - *ps).Normalize();
	m_bbox.Union(*ps);
	m_bbox.Union(*pe);
}

TerrainSegment::TerrainSegment(int id, Point3D* ps, Point3D* pe)
	: m_segmentId(id)
	, m_isShared(false)
    , m_ps(ps)
    , m_pe(pe)
	, m_facet1(nullptr)
	, m_facet2(nullptr)
{
	m_dir = (*pe - *ps).Normalize();
	m_bbox.Union(*ps);
	m_bbox.Union(*pe);
}

TerrainSegment::TerrainSegment(int id, Point3D* ps, Point3D* pe, TerrainFacet* facet1, TerrainFacet* facet2)
	: m_segmentId(id)
	, m_isShared(false)
	, m_ps(ps)
	, m_pe(pe)
	, m_facet1(facet1)
	, m_facet2(facet2)
{
	m_dir = (*pe - *ps).Normalize();
	m_bbox.Union(*ps);
	m_bbox.Union(*pe);
}

TerrainSegment::TerrainSegment(const TerrainSegment& ts)
	: m_segmentId(ts.m_segmentId)
	, m_isShared(ts.m_isShared)
	, m_ps(ts.m_ps)
	, m_pe(ts.m_pe)
	, m_dir(ts.m_dir)
	, m_facet1(ts.m_facet1)
	, m_facet2(ts.m_facet2)
	, m_bbox(ts.m_bbox)
{
}

TerrainSegment::~TerrainSegment()
{
}

TerrainSegment& TerrainSegment::operator=(const TerrainSegment& ts)
{
	m_segmentId = ts.m_segmentId;
	m_isShared = ts.m_isShared;
	m_ps = ts.m_ps;
	m_pe = ts.m_pe;
	m_dir = ts.m_dir;
	m_facet1 = ts.m_facet1;
	m_facet2 = ts.m_facet2;
	m_bbox = ts.m_bbox;
    return *this;
}

bool TerrainSegment::operator==(const TerrainSegment& ts) const
{
    return m_segmentId == ts.m_segmentId;
}

bool TerrainSegment::operator!=(const TerrainSegment& ts) const
{
    return !(*this==ts);
}

Point2D TerrainSegment::GetStartPoint2D() const
{
	return Point2D(m_ps->x,m_ps->y);
}

Point2D TerrainSegment::GetEndPoint2D() const
{
	return Point2D(m_pe->x,m_pe->y);
}

TerrainFacet* TerrainSegment::GetAdjacentFacet(TerrainFacet* facet)
{
	if (facet == m_facet1)
		return m_facet2;
	else if (facet == m_facet2)
		return m_facet1;
	else
		return nullptr;
}

bool TerrainSegment::Intersect(Ray2D* ray, RtLbsType* st, RtLbsType* height) const
{
	//����u�Ĳ�����������ߵĸ߶�
	Vector2D dir = NormalizeXY(m_dir);										//�����߶εĶ�ά����
	Vector2D ps(m_ps->x, m_ps->y);											//�����߶���ʼ��
	Vector2D pe(m_pe->x, m_pe->y);											//�����߶���ֹ��
	RtLbsType hs = m_ps->z;
	RtLbsType he = m_pe->z;
	bool hasIntersect = false; //�Ƿ��ཻ�ı�־λ
	RtLbsType denominator = Cross(ray->m_Dir, dir);
	if (abs(denominator) < EPSILON) { //�������߶ι������
		if (abs(Cross(ray->m_Dir, ps - ray->m_Ori)) < EPSILON) {//��ǰ��������������ཻ
			RtLbsType tmax;
			RtLbsType tmin = m_bbox.Intersect(*ray, &tmax);
			if (tmin < 0.0) {//�����Χ���ཻ
				return false;
			}
			*st = tmin;//���½������Դ��ľ���
			*height = hs;//����߶�-����Ϊ�߶ε���㴦
			return true;
		}
		if (abs(Cross(ray->m_Dir, pe - ray->m_Ori)) < EPSILON) { //��ǰ�������߶��յ��ཻ
			RtLbsType tmax;
			RtLbsType tmin = m_bbox.Intersect(*ray, &tmax);
			if (tmin < 0.0) {//�����Χ���ཻ
				return false;
			}
			*st = tmin;//���½������Դ��ľ���
			*height = he;//����߶�-����Ϊ�߶ε���㴦
			return true;
		}
		hasIntersect = false; //���߲����߶��ཻ
		return false;
	}
	Vector2D p = ps - ray->m_Ori;
	RtLbsType t = static_cast<RtLbsType>(Cross(p, dir) / denominator /*- EPSILON*/); //t:���������Ԫ���ľ���,��Сt��ֵ�����ڹ�ܼ�������
	RtLbsType u = static_cast<RtLbsType>(Cross(p, ray->m_Dir) / denominator / (pe - ps).Length()); //�������߶��ϵİٷֱ�
	if (u < 0 && abs(u) < 1e-9) u = 0.0;
	if (u > 1.0 && abs(u - 1.0) < 1e-9) u = 1.0;
	if (t >= -EPSILON && u >= 0.0 && u <= 1.0) {//���￼��t>EPSILONָ�����ж��������Ԫ�ϵ������ཻ,�����ӿ������
		*st = t;
		*height = hs + u * (he - hs);//����߶�
		return true;
	}
	return false;
}

bool TerrainSegment::HasIntersect(Ray2D* ray, RtLbsType& out_t) const
{
	//����u�Ĳ�����������ߵĸ߶�
	Vector2D dir = NormalizeXY(m_dir);										//�����߶εĶ�ά����
	Vector2D ps(m_ps->x, m_ps->y);											//�����߶���ʼ��
	Vector2D pe(m_pe->x, m_pe->y);											//�����߶���ֹ��
	bool hasIntersect = false; //�Ƿ��ཻ�ı�־λ
	RtLbsType denominator = Cross(ray->m_Dir, dir);
	if (abs(denominator) < EPSILON) { //�������߶ι������
		if (abs(Cross(ray->m_Dir, ps - ray->m_Ori)) < EPSILON) {//��ǰ��������������ཻ
			RtLbsType tmax;
			RtLbsType tmin = m_bbox.Intersect(*ray, &tmax);
			if (tmin < 0.0) {//�����Χ���ཻ
				return false;
			}
			out_t = tmin;
			return true;
		}
		if (abs(Cross(ray->m_Dir, pe - ray->m_Ori)) < EPSILON) { //��ǰ�������߶��յ��ཻ
			RtLbsType tmax;
			RtLbsType tmin = m_bbox.Intersect(*ray, &tmax);
			if (tmin < 0.0) {//�����Χ���ཻ
				return false;
			}
			out_t = tmin;
			return true;
		}
		hasIntersect = false; //���߲����߶��ཻ
		return false;
	}
	Vector2D p = ps - ray->m_Ori;
	RtLbsType t = static_cast<RtLbsType>(Cross(p, dir) / denominator /*- EPSILON*/); //t:���������Ԫ���ľ���,��Сt��ֵ�����ڹ�ܼ�������
	RtLbsType u = static_cast<RtLbsType>(Cross(p, ray->m_Dir) / denominator / (pe - ps).Length()); //�������߶��ϵİٷֱ�
	if (abs(u) < EPSILON) u = 0.0;
	if (abs(u - 1.0) < EPSILON) u = 1.0;
	if (t >= -EPSILON && u >= 0.0 && u <= 1.0) {//���￼��t>EPSILONָ�����ж��������Ԫ�ϵ������ཻ,�����ӿ������
		out_t = t;
		return true;
	}
	return false;
}

RtLbsType TerrainSegment::GetLengthXY() const
{
	return (*m_pe - *m_ps).LengthXY();
}

RtLbsType TerrainSegment::GetLength() const
{
	return (*m_pe - *m_ps).Length();
}

Vector2D TerrainSegment::GetDirXY() const
{
	return NormalizeXY(*m_pe - *m_ps);
}

bool TerrainSegment::ValidIntersect(const Ray3DLite& ray)
{
	return false;
}

std::string TerrainSegment::ToString() const
{
	std::stringstream ss;
	if (*m_ps > *m_pe) //�͵���ǰģʽ
		ss << m_pe->ToString() << "," << m_ps->ToString();
	else
		ss << m_ps->ToString() << "," << m_pe->ToString();
	return ss.str();
}

size_t TerrainSegment::GetHash() const
{
	return util::Hash64(ToString());
}

void TerrainSegment::Update()
{
	m_dir = (*m_pe - *m_ps).Normalize();			//�����߶η�������
	m_bbox.InvalidBBox();							//��Χ��ʹ��ʧЧ
	m_bbox.Union(*m_ps);
	m_bbox.Union(*m_pe);							//���°�Χ��
}
