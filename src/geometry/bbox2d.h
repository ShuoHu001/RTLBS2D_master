#ifndef RTLBS_BBOX2D
#define RTLBS_BBOX2D

#include "utility/serializable.h"
#include "rtlbs.h"
#include "ray2d.h"
#include "utility/define.h"
#include "geometry/gpu/bbox2dgpu.h"
#include "geometry/point3d.h"


class BBox2D {

public:
	Point2D m_min; /** @brief	��Сֵ��	*/
	Point2D m_max; /** @brief	���ֵ��	*/
	bool m_isValid; /** @brief	��Χ���Ƿ���Ч	*/

public:
	BBox2D();
	BBox2D(const Point2D& p0, const Point2D& p1, bool bValid = true);
	BBox2D(const BBox2D& bbox);
	~BBox2D() {};

public:
	bool IsInBBox(const Point2D& p, RtLbsType delta) const;
	RtLbsType SurfaceArea() const;
	RtLbsType HalfSurfaceArea() const;
	unsigned MaxAxisId() const;
	void Union(const Point2D& p);
	void Union(const Point3D& p);
	void Union(const BBox2D& box);
	RtLbsType Delta(unsigned k) const;
	void InvalidBBox();
	RtLbsType Intersect(const Ray2D& ray, RtLbsType* fmax = 0) const;
	BBox2D& operator = (const BBox2D& bbox);
	bool IsContainPoint(const Point2D& p) const;											//����Χ���Ƿ������ά�����
	bool IsContainPoint(const Point3D& p) const;											//����Χ���Ƿ������ά�����
	BBox2DGPU Convert2GPU();

public:
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);

};


inline BBox2D Union(const BBox2D& bbox, const Point2D& p) {
	BBox2D bbox_re = bbox;
	for (unsigned i = 0; i < 2; i++) {
		if (p[i] < bbox.m_min[i])
			bbox_re.m_min[i] = p[i];
		if (p[i] > bbox.m_max[i])
			bbox_re.m_max[i] = p[i];
	}
	return bbox_re;
}

inline BBox2D Union(const BBox2D& bbox0, const BBox2D& bbox1) {
	BBox2D bbox_re;
	for (unsigned i = 0; i < 2; i++) {
		bbox_re.m_min[i] = std::min(bbox0.m_min[i], bbox1.m_min[i]);
		bbox_re.m_max[i] = std::max(bbox0.m_max[i], bbox1.m_max[i]);
	}
	return bbox_re;
}


inline RtLbsType Intersect_BBox2D(const Ray2D& ray, const BBox2D& bbox, RtLbsType* fmax = 0) {//slabs ������⽻��
	RtLbsType tmin, tmax, tymin, tymax;
	if (ray.m_Dir.x == 0) {
		tmin = -FLT_MAX;
		tmax = FLT_MAX;
	}
	else {
		tmin = (bbox.m_min.x - ray.m_Ori.x) / ray.m_Dir.x;
		tmax = (bbox.m_max.x - ray.m_Ori.x) / ray.m_Dir.x;
		if (tmin > tmax) std::swap(tmin, tmax);
	}

	if (ray.m_Dir.y == 0) {
		tymin = -FLT_MAX;
		tymax = FLT_MAX;
	}
	else {
		tymin = (bbox.m_min.y - ray.m_Ori.y) / ray.m_Dir.y;
		tymax = (bbox.m_max.y - ray.m_Ori.y) / ray.m_Dir.y;
		if (tymin > tymax) std::swap(tymin, tymax);
	}

	if ((tmin > tymax && abs(tmin - tymax) >= EPSILON) || (tymin > tmax && abs(tymin - tmax) > EPSILON)) return static_cast<RtLbsType>(-1.0); // ray misses bbox,��������ֹ����ͬ��ɵ����

	// update tmin and tmax to account for bbox intersection

	tmin = std::max(0.0, std::max(tymin, tmin));//���������ڱ߽���ڵ����
	tmax = std::min(tymax, tmax);

	if (fmax != nullptr)
	{
		*fmax = tmax;
	}
	if (tmin == 0) {//�ڱ߽����
		return tmax;
	}
	return tmin;
}

#endif
