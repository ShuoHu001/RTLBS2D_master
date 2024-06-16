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
	//根据u的参数计算出射线的高度
	Vector2D dir = NormalizeXY(m_dir);										//地形线段的二维方向
	Vector2D ps(m_ps->x, m_ps->y);											//地形线段起始点
	Vector2D pe(m_pe->x, m_pe->y);											//地形线段终止点
	RtLbsType hs = m_ps->z;
	RtLbsType he = m_pe->z;
	bool hasIntersect = false; //是否相交的标志位
	RtLbsType denominator = Cross(ray->m_Dir, dir);
	if (abs(denominator) < EPSILON) { //射线与线段共线情况
		if (abs(Cross(ray->m_Dir, ps - ray->m_Ori)) < EPSILON) {//当前射线与射线起点相交
			RtLbsType tmax;
			RtLbsType tmin = m_bbox.Intersect(*ray, &tmax);
			if (tmin < 0.0) {//不与包围盒相交
				return false;
			}
			*st = tmin;//更新交点距离源点的距离
			*height = hs;//计算高度-交点为线段的起点处
			return true;
		}
		if (abs(Cross(ray->m_Dir, pe - ray->m_Ori)) < EPSILON) { //当前射线与线段终点相交
			RtLbsType tmax;
			RtLbsType tmin = m_bbox.Intersect(*ray, &tmax);
			if (tmin < 0.0) {//不与包围盒相交
				return false;
			}
			*st = tmin;//更新交点距离源点的距离
			*height = he;//计算高度-交点为线段的起点处
			return true;
		}
		hasIntersect = false; //射线不与线段相交
		return false;
	}
	Vector2D p = ps - ray->m_Ori;
	RtLbsType t = static_cast<RtLbsType>(Cross(p, dir) / denominator /*- EPSILON*/); //t:交点距离面元起点的距离,减小t的值有利于规避计算误差极限
	RtLbsType u = static_cast<RtLbsType>(Cross(p, ray->m_Dir) / denominator / (pe - ps).Length()); //交点在线段上的百分比
	if (u < 0 && abs(u) < 1e-9) u = 0.0;
	if (u > 1.0 && abs(u - 1.0) < 1e-9) u = 1.0;
	if (t >= -EPSILON && u >= 0.0 && u <= 1.0) {//这里考虑t>EPSILON指明不判定起点在面元上的射线相交,并附加考虑误差
		*st = t;
		*height = hs + u * (he - hs);//计算高度
		return true;
	}
	return false;
}

bool TerrainSegment::HasIntersect(Ray2D* ray, RtLbsType& out_t) const
{
	//根据u的参数计算出射线的高度
	Vector2D dir = NormalizeXY(m_dir);										//地形线段的二维方向
	Vector2D ps(m_ps->x, m_ps->y);											//地形线段起始点
	Vector2D pe(m_pe->x, m_pe->y);											//地形线段终止点
	bool hasIntersect = false; //是否相交的标志位
	RtLbsType denominator = Cross(ray->m_Dir, dir);
	if (abs(denominator) < EPSILON) { //射线与线段共线情况
		if (abs(Cross(ray->m_Dir, ps - ray->m_Ori)) < EPSILON) {//当前射线与射线起点相交
			RtLbsType tmax;
			RtLbsType tmin = m_bbox.Intersect(*ray, &tmax);
			if (tmin < 0.0) {//不与包围盒相交
				return false;
			}
			out_t = tmin;
			return true;
		}
		if (abs(Cross(ray->m_Dir, pe - ray->m_Ori)) < EPSILON) { //当前射线与线段终点相交
			RtLbsType tmax;
			RtLbsType tmin = m_bbox.Intersect(*ray, &tmax);
			if (tmin < 0.0) {//不与包围盒相交
				return false;
			}
			out_t = tmin;
			return true;
		}
		hasIntersect = false; //射线不与线段相交
		return false;
	}
	Vector2D p = ps - ray->m_Ori;
	RtLbsType t = static_cast<RtLbsType>(Cross(p, dir) / denominator /*- EPSILON*/); //t:交点距离面元起点的距离,减小t的值有利于规避计算误差极限
	RtLbsType u = static_cast<RtLbsType>(Cross(p, ray->m_Dir) / denominator / (pe - ps).Length()); //交点在线段上的百分比
	if (abs(u) < EPSILON) u = 0.0;
	if (abs(u - 1.0) < EPSILON) u = 1.0;
	if (t >= -EPSILON && u >= 0.0 && u <= 1.0) {//这里考虑t>EPSILON指明不判定起点在面元上的射线相交,并附加考虑误差
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
	if (*m_ps > *m_pe) //低点在前模式
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
	m_dir = (*m_pe - *m_ps).Normalize();			//更新线段方向向量
	m_bbox.InvalidBBox();							//包围盒使能失效
	m_bbox.Union(*m_ps);
	m_bbox.Union(*m_pe);							//更新包围盒
}
