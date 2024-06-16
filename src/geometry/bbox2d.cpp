#include "bbox2d.h"

BBox2D::BBox2D()
{
	m_min = Point2D(FLT_MAX, FLT_MAX);
	m_max = Point2D(-FLT_MAX, -FLT_MAX);
	m_isValid = false;
}

BBox2D::BBox2D(const Point2D& p0, const Point2D& p1, bool bValid)
	: m_isValid(bValid)
{
	m_min = Point2D(std::min(p0.x, p1.x), std::min(p0.y, p1.y));
	m_max = Point2D(std::max(p0.x, p1.x), std::max(p0.y, p1.y));
}

BBox2D::BBox2D(const BBox2D& bbox)
	: m_min(bbox.m_min), m_max(bbox.m_max), m_isValid(bbox.m_isValid)
{
}

bool BBox2D::IsInBBox(const Point2D& p, RtLbsType delta) const
{
	if (p.x < m_min.x - delta || p.x > m_max.x + delta ||
		p.y < m_min.y - delta || p.y > m_max.y + delta) {
		return false;
	}
	return true;
}

RtLbsType BBox2D::SurfaceArea() const
{
	RtLbsType w = m_max.x - m_min.x;
	RtLbsType h = m_max.y - m_min.y;
	return 2.0f * (w + h);
}

RtLbsType BBox2D::HalfSurfaceArea() const
{
	RtLbsType w = m_max.x - m_min.x;
	RtLbsType h = m_max.y - m_min.y;
	return w + h;
}

unsigned BBox2D::MaxAxisId() const
{
	RtLbsType w = m_max.x - m_min.x;
	RtLbsType h = m_max.y - m_min.y;
	if (w > h) {
		return 0; // 最长边为x轴
	}
	return 1; // 最长边为y轴
}

void BBox2D::Union(const Point2D& p)
{
	m_min.x = std::min(m_min.x, p.x);
	m_min.y = std::min(m_min.y, p.y);
	m_max.x = std::max(m_max.x, p.x);
	m_max.y = std::max(m_max.y, p.y);
}

void BBox2D::Union(const Point3D& p)
{
	m_min.x = std::min(m_min.x, p.x);
	m_min.y = std::min(m_min.y, p.y);
	m_max.x = std::max(m_max.x, p.x);
	m_max.y = std::max(m_max.y, p.y);
}

void BBox2D::Union(const BBox2D& box)
{
	m_min.x = std::min(m_min.x, box.m_min.x);
	m_min.y = std::min(m_min.y, box.m_min.y);
	m_max.x = std::max(m_max.x, box.m_max.x);
	m_max.y = std::max(m_max.y, box.m_max.y);
}

RtLbsType BBox2D::Delta(unsigned k) const
{
	if (k > 1) {// 超出维度范围
		return 0.0f;
	}
	return m_max[k] - m_min[k];
}

void BBox2D::InvalidBBox()
{
	m_min = Point2D(FLT_MAX, FLT_MAX);
	m_max = Point2D(-FLT_MAX, -FLT_MAX);
	m_isValid = false;
}

RtLbsType BBox2D::Intersect(const Ray2D& ray, RtLbsType* fmax) const
{
	RtLbsType tmin, tmax, tymin, tymax;
	if (ray.m_Dir.x == 0) {
		tmin = -FLT_MAX;
		tmax = FLT_MAX;
	}
	else {
		tmin = (m_min.x - ray.m_Ori.x) / ray.m_Dir.x;
		tmax = (m_max.x - ray.m_Ori.x) / ray.m_Dir.x;
		if (tmin > tmax) std::swap(tmin, tmax);
	}

	if (ray.m_Dir.y == 0) {
		tymin = -FLT_MAX;
		tymax = FLT_MAX;
	}
	else {
		tymin = (m_min.y - ray.m_Ori.y) / ray.m_Dir.y;
		tymax = (m_max.y - ray.m_Ori.y) / ray.m_Dir.y;
		if (tymin > tymax) std::swap(tymin, tymax);
	}

	if ((tmin > tymax && abs(tmin - tymax) >= EPSILON) || (tymin > tmax && abs(tymin - tmax) > EPSILON)) return -1.0; // ray misses bbox,修正，防止数相同造成的误差

	// update tmin and tmax to account for bbox intersection
	tmin = std::max(0.0, std::max(tymin, tmin));//限制射线在边界框内的情况
	tmax = std::min(tymax, tmax);

	if (fmax != nullptr)
	{
		*fmax = tmax;
	}
	if (tmin == 0) {//在边界框内
		return tmax;
	}
	return tmin;
}

BBox2D& BBox2D::operator=(const BBox2D& bbox)
{
	m_min = bbox.m_min;
	m_max = bbox.m_max;
	m_isValid = bbox.m_isValid;
	return (*this);
}

bool BBox2D::IsContainPoint(const Point2D& p) const
{
	if (p.x<m_min.x || p.x>m_max.x)
		return false;
	if (p.y<m_min.y || p.y>m_max.y)
		return false;
	return true;
}

bool BBox2D::IsContainPoint(const Point3D& p) const
{
	if (p.x<m_min.x || p.x>m_max.x)
		return false;
	if (p.y<m_min.y || p.y>m_max.y)
		return false;
	return true;
}

BBox2DGPU BBox2D::Convert2GPU()
{
	BBox2DGPU bboxGPU;
	bboxGPU.m_Min = m_min;
	bboxGPU.m_Max = m_max;
	bboxGPU.m_bValid = m_isValid;
	return bboxGPU;
}

void BBox2D::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.String("m_Min"); m_min.Serialize(writer);
	writer.String("m_Max"); m_max.Serialize(writer);
	writer.String("m_bValid"); writer.Bool(m_isValid);
	writer.EndObject();
}

bool BBox2D::Deserialize(const rapidjson::Value& value)
{
	if (value.IsObject()) {
		const rapidjson::Value& m_MinValue = value["m_Min"];
		const rapidjson::Value& m_MaxValue = value["m_Max"];
		const rapidjson::Value& m_bValidValue = value["m_bValid"];
		if (m_MinValue.IsObject() && m_MaxValue.IsObject() && m_bValidValue.IsBool()) {
			m_isValid = m_bValidValue.GetBool();
			return m_min.Deserialize(m_MinValue) && m_min.Deserialize(m_MaxValue);
		}
	}
	return false;
}
