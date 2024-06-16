#include "bbox2dgpu.h"

HOST_DEVICE_FUNC bool BBox2DGPU::IsInBBox(const Point2D& p, RtLbsType delta) const
{
	if (p.x < m_Min.x - delta || p.x > m_Max.x + delta ||
		p.y < m_Min.y - delta || p.y > m_Max.y + delta) {
		return false;
	}
	return true;
}

HOST_DEVICE_FUNC RtLbsType BBox2DGPU::SurfaceArea() const
{
	RtLbsType w = m_Max.x - m_Min.x;
	RtLbsType h = m_Max.y - m_Min.y;
	return 2.0f * (w + h);
}

HOST_DEVICE_FUNC RtLbsType BBox2DGPU::HalfSurfaceArea() const
{
	RtLbsType w = m_Max.x - m_Min.x;
	RtLbsType h = m_Max.y - m_Min.y;
	return w + h;
}

HOST_DEVICE_FUNC unsigned BBox2DGPU::MaxAxisId() const
{
	RtLbsType w = m_Max.x - m_Min.x;
	RtLbsType h = m_Max.y - m_Min.y;
	if (w > h) {
		return 0; // 最长边为x轴
	}
	return 1; // 最长边为y轴
}

HOST_DEVICE_FUNC RtLbsType BBox2DGPU::Delta(unsigned k) const
{
	if (k > 1) {// 超出维度范围
		return 0.0f;
	}
	return m_Max[k] - m_Min[k];
}

HOST_DEVICE_FUNC void BBox2DGPU::InvalidBBox()
{
	m_Min = Point2D(FLT_MAX, FLT_MAX);
	m_Max = Point2D(-FLT_MAX, -FLT_MAX);
	m_bValid = false;
}

HOST_DEVICE_FUNC RtLbsType BBox2DGPU::Intersect(const Ray2DGPU& ray, RtLbsType* fmax)
{
	RtLbsType tmin, tmax, tymin, tymax;
	if (ray.m_Dir.x == 0) {
		tmin = -FLT_MAX;
		tmax = FLT_MAX;
	}
	else {
		tmin = (m_Min.x - ray.m_Ori.x) / ray.m_Dir.x;
		tmax = (m_Max.x - ray.m_Ori.x) / ray.m_Dir.x;
		if (tmin > tmax) thrust::swap(tmin, tmax);
	}

	if (ray.m_Dir.y == 0) {
		tymin = -FLT_MAX;
		tymax = FLT_MAX;
	}
	else {
		tymin = (m_Min.y - ray.m_Ori.y) / ray.m_Dir.y;
		tymax = (m_Max.y - ray.m_Ori.y) / ray.m_Dir.y;
		if (tymin > tymax) thrust::swap(tymin, tymax);
	}

	if ((tmin > tymax && abs(tmin - tymax) >= EPSILON) || (tymin > tmax && abs(tymin - tmax) > EPSILON)) return -1.0; // ray misses bbox,修正，防止数相同造成的误差

	// update tmin and tmax to account for bbox intersection
	tmin = thrust::max(0.0, thrust::max(tymin, tmin));//限制射线在边界框内的情况
	tmax = thrust::min(tymax, tmax);

	if (fmax != nullptr)
	{
		*fmax = tmax;
	}
	if (tmin == 0) {//在边界框内
		return tmax;
	}
	return tmin;
}

HOST_DEVICE_FUNC BBox2DGPU& BBox2DGPU::operator=(const BBox2DGPU& bbox)
{
	m_Min = bbox.m_Min;
	m_Max = bbox.m_Max;
	m_bValid = bbox.m_bValid;
	return (*this);
}