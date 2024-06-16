#include "bbox3d.h"


BBox3D::BBox3D()
{
	m_min = Point3D(FLT_MAX, FLT_MAX, FLT_MAX);
	m_max = Point3D(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	m_isValid = false;
}

BBox3D::BBox3D(const Point3D& p0, const Point3D& p1, bool isValid)
	: m_min(Point3D(std::min(p0.x, p1.x), std::min(p0.y, p1.y), std::min(p0.z, p1.z)))
	, m_max(Point3D(std::max(p0.x, p1.x), std::max(p0.y, p1.y), std::max(p0.z, p1.z)))
	, m_isValid(isValid)
{
}

BBox3D::BBox3D(const BBox3D& bbox)
	: m_min(bbox.m_min)
	, m_max(bbox.m_max)
	, m_isValid(bbox.m_isValid)
{
}

BBox3D::~BBox3D()
{
}

BBox3D& BBox3D::operator=(const BBox3D& bbox)
{
	m_min = bbox.m_min;
	m_max = bbox.m_max;
	m_isValid = bbox.m_isValid;
	return *this;
}

BBox3D BBox3D::operator+(const BBox3D& bbox) const
{
	BBox3D bbox_re;
	bbox_re.m_min.x = std::min(m_min.x, bbox.m_min.x);
	bbox_re.m_min.y = std::min(m_min.y, bbox.m_min.y);
	bbox_re.m_min.z = std::min(m_min.z, bbox.m_min.z);
	bbox_re.m_max.x = std::max(m_max.x, bbox.m_max.x);
	bbox_re.m_max.y = std::max(m_max.y, bbox.m_max.y);
	bbox_re.m_max.z = std::max(m_max.z, bbox.m_max.z);
	return bbox_re;
}

BBox3D& BBox3D::operator+=(const BBox3D& bbox)
{
	m_min.x = std::min(m_min.x, bbox.m_min.x);
	m_min.y = std::min(m_min.y, bbox.m_min.y);
	m_min.z = std::min(m_min.z, bbox.m_min.z);
	m_max.x = std::max(m_max.x, bbox.m_max.x);
	m_max.y = std::max(m_max.y, bbox.m_max.y);
	m_max.z = std::max(m_max.z, bbox.m_max.z);
	return *this;
}

BBox3D BBox3D::operator+(const Vector3D& offset) const
{
	BBox3D bboxRe;
	bboxRe.m_min = m_min + offset;
	bboxRe.m_max = m_max + offset;
	return bboxRe;
}

BBox3D& BBox3D::operator+=(const Vector3D& offset)
{
	m_min += offset;
	m_max += offset;
	return *this;
}

BBox3D BBox3D::operator-(const Vector3D& offset) const
{
	BBox3D bboxRe;
	bboxRe.m_min = m_min - offset;
	bboxRe.m_max = m_max - offset;
	return bboxRe;
}

BBox3D& BBox3D::operator-=(const Vector3D& offset)
{
	m_min -= offset;
	m_max -= offset;
	return *this;
}

BBox3D BBox3D::operator*(const Euler& angle) const
{
	//注意：调用该函数时，需要保障包围盒的中心为旋转中心（确定包围盒的中心即可）
	BBox3D bboxRe;
	bboxRe.m_min = m_min * angle;
	bboxRe.m_max = m_max * angle;
	return bboxRe;
}

BBox3D& BBox3D::operator*=(const Euler& angle)
{
	m_min *= angle;
	m_max *= angle;
	return *this;
}

bool BBox3D::IsInBBox(const Point3D& p, RtLbsType delta) const
{
	if (p.x<m_min.x - delta || p.x>m_max.x + delta ||
		p.y<m_min.y - delta || p.y>m_max.y + delta ||
		p.z<m_min.z - delta || p.z>m_max.z + delta)
		return false;
	return true;
}

RtLbsType BBox3D::SurfaceArea() const
{
	return 2.0 * HalfSurafaceArea();
}

RtLbsType BBox3D::HalfSurafaceArea() const
{
	Vector3D offset = m_max - m_min;
	return offset.x * offset.y + offset.y * offset.z + offset.z * offset.x;
}

Vector3D BBox3D::GetCenter() const
{
	return (m_min + m_max) / 2;
}


RtLbsType BBox3D::Volume() const
{
	Vector3D offset = m_max - m_min;
	return offset.x * offset.y * offset.z;
}

unsigned BBox3D::MaxAxisId() const
{
	Vector3D offset = m_max - m_min;
	if (offset.x >= offset.y && offset.x >= offset.z)
		return 0;
	else if (offset.y >= offset.x && offset.y >= offset.z)
		return 1;
	else
		return 2;
}

void BBox3D::Union(const Point3D& p)
{
	m_min.x = std::min(m_min.x, p.x);
	m_min.y = std::min(m_min.y, p.y);
	m_min.z = std::min(m_min.z, p.z);
	m_max.x = std::max(m_max.x, p.x);
	m_max.y = std::max(m_max.y, p.y);
	m_max.z = std::max(m_max.z, p.z);
}

void BBox3D::Union(const Point3D* p)
{
	m_min.x = std::min(m_min.x, p->x);
	m_min.y = std::min(m_min.y, p->y);
	m_min.z = std::min(m_min.z, p->z);
	m_max.x = std::max(m_max.x, p->x);
	m_max.y = std::max(m_max.y, p->y);
	m_max.z = std::max(m_max.z, p->z);
}

void BBox3D::Union(const BBox3D& bbox)
{
	m_min.x = std::min(m_min.x, bbox.m_min.x);
	m_min.y = std::min(m_min.y, bbox.m_min.y);
	m_min.z = std::min(m_min.z, bbox.m_min.z);
	m_max.x = std::max(m_max.x, bbox.m_max.x);
	m_max.y = std::max(m_max.y, bbox.m_max.y);
	m_max.z = std::max(m_max.z, bbox.m_max.z);
}

RtLbsType BBox3D::Delta(unsigned k) const
{
	if (k > 2) //超出维度范围
		return 0.0;
	return m_max[k] - m_min[k];
}

void BBox3D::InvalidBBox()
{
	m_min = Point3D(FLT_MAX, FLT_MAX, FLT_MAX);
	m_max = Point3D(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	m_isValid = false;
}

RtLbsType BBox3D::Intersect(Ray3DLite* ray, RtLbsType* fmax) const
{
	RtLbsType tmax = FLT_MAX;
	RtLbsType tmin = -FLT_MAX;
	for (unsigned axis = 0; axis < 3; ++axis) {
		if (abs(ray->m_dir[axis]) < EPSILON) {
			if (ray->m_ori[axis] > m_max[axis] || ray->m_ori[axis] < m_min[axis])//若射线在特定轴上无方向增量，满足不在边界框内时即不相交
				return -1.0;
		}
		else {
			RtLbsType ood = 1.0 / ray->m_dir[axis];
			RtLbsType t1 = (m_min[axis] - ray->m_ori[axis]) * ood; //距离最小值
			RtLbsType t2 = (m_max[axis] - ray->m_ori[axis]) * ood; //距离最大值
			if (t1 > t2)
				std::swap(t1, t2);
			tmin = std::max(0.0, std::max(t1, tmin));//射线起点在框内不为负值
			tmax = std::min(t2, tmax);
			if (tmin > tmax)
				return -1.0;
		}
	}
	if (fmax)
		*fmax = tmax;
	if (tmin <= 0)//若射线起点在边界框内部，则tmin为负值，为了计算，返回最大值
		return tmax;
	return tmin;
}

bool BBox3D::IsContainPoint(const Point3D& p) const
{
	if (p.x<m_min.x || p.x>m_max.x)
		return false;
	if (p.y<m_min.y || p.y>m_max.y)
		return false;
	if (p.z<m_min.z || p.z>m_max.z)
		return false;
	return true;
}

void BBox3D::Update(const Vector3D& offset)
{
	*this += offset;
}

RtLbsType Intersect_BBox3D(Ray3DLite* ray, const BBox3D& bbox, RtLbsType* fmax) {
	RtLbsType tmax = FLT_MAX;
	RtLbsType tmin = -FLT_MAX;
	for (unsigned axis = 0; axis < 3; ++axis) {
		if (ray->m_dir[axis]<EPSILON && ray->m_dir[axis]>-EPSILON) {
			if (ray->m_ori[axis] > bbox.m_max[axis] || ray->m_ori[axis] < bbox.m_min[axis])//若射线在特定轴上无方向增量，满足不在边界框内时即不相交
				return -1.0;
		}
		else {
			RtLbsType ood = 1.0 / ray->m_dir[axis];
			RtLbsType t1 = (bbox.m_min[axis] - ray->m_ori[axis]) * ood;
			RtLbsType t2 = (bbox.m_max[axis] - ray->m_ori[axis]) * ood;
			if (t1 > t2)
				std::swap(t1, t2);
			tmin = std::max(0.0, std::max(t1, tmin));//射线起点在框内不为负值
			tmax = std::min(t2, tmax);
			if (tmin > tmax)
				return -1.0;
		}
	}
	if (fmax)
		*fmax = tmax;
	if (tmin <= 0)//若射线起点在边界框内部，则tmin为负值，为了计算，返回最大值
		return tmax;
	return tmin;
}