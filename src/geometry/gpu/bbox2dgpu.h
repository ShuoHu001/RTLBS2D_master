#ifndef RTLBS_BBOX2DGPU
#define RTLBS_BBOX2DGPU

#include "utility/serializable.h"
#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/point2d.h"
#include <float.h>

#include "ray2dgpu.h"


class BBox2DGPU {

public:
	Point2D m_Min; /** @brief	最小值点	*/
	Point2D m_Max; /** @brief	最大值点	*/
	bool m_bValid; /** @brief	包围盒是否有效	*/

public:
	HOST_DEVICE_FUNC BBox2DGPU()
		: m_bValid(false){}
	HOST_DEVICE_FUNC BBox2DGPU(Point2D minp, Point2D maxp, bool valid)
		: m_Min(minp)
		, m_Max(maxp)
		, m_bValid(valid) {}

	HOST_DEVICE_FUNC ~BBox2DGPU() {};

public:
	HOST_DEVICE_FUNC bool IsInBBox(const Point2D& p, RtLbsType delta) const;
	HOST_DEVICE_FUNC RtLbsType SurfaceArea() const;
	HOST_DEVICE_FUNC RtLbsType HalfSurfaceArea() const;
	HOST_DEVICE_FUNC unsigned MaxAxisId() const;
	HOST_DEVICE_FUNC RtLbsType Delta(unsigned k) const;
	HOST_DEVICE_FUNC void InvalidBBox();
	HOST_DEVICE_FUNC RtLbsType Intersect(const Ray2DGPU& ray, RtLbsType* fmax = 0);
	HOST_DEVICE_FUNC BBox2DGPU& operator = (const BBox2DGPU& bbox);

};

HOST_DEVICE_FUNC inline RtLbsType Intersect_BBox2D(const Ray2DGPU& ray, const BBox2DGPU& bbox, RtLbsType* fmax = 0) {//slabs 方法求解交点
	RtLbsType tmin, tmax, tymin, tymax;
	if (ray.m_Dir.x == 0) {
		tmin = -FLT_MAX;
		tmax = FLT_MAX;
	}
	else {
		tmin = (bbox.m_Min.x - ray.m_Ori.x) / ray.m_Dir.x;
		tmax = (bbox.m_Max.x - ray.m_Ori.x) / ray.m_Dir.x;
		if (tmin > tmax) thrust::swap(tmin, tmax);
	}

	if (ray.m_Dir.y == 0) {
		tymin = -FLT_MAX;
		tymax = FLT_MAX;
	}
	else {
		tymin = (bbox.m_Min.y - ray.m_Ori.y) / ray.m_Dir.y;
		tymax = (bbox.m_Max.y - ray.m_Ori.y) / ray.m_Dir.y;
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

#endif
