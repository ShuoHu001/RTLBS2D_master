#ifndef RTLBS_BBOX3D
#define RTLBS_BBOX3D
#include "rtlbs.h"
#include "utility/define.h"
#include "math/point3d.h"
#include "math/vector3d.h"
#include "math/euler.h"
#include "ray3dlite.h"

class Ray3DLite;

class BBox3D {
public:
	Point3D m_min;
	Point3D m_max;
	bool m_isValid;

public:
	BBox3D();
	BBox3D(const Point3D& p0, const Point3D& p1, bool isValid = true);
	BBox3D(const BBox3D& bbox);
	~BBox3D();

public:
	BBox3D& operator = (const BBox3D& bbox);
	BBox3D operator + (const BBox3D& bbox) const;
	BBox3D& operator += (const BBox3D& bbox);
	BBox3D operator + (const Vector3D& offset) const;
	BBox3D& operator += (const Vector3D& offset);
	BBox3D operator - (const Vector3D& offset) const;
	BBox3D& operator -= (const Vector3D& offset);
	BBox3D operator * (const Euler& angle) const;
	BBox3D& operator *= (const Euler& angle);
	bool IsInBBox(const Point3D& p, RtLbsType delta) const;
	RtLbsType SurfaceArea() const;
	RtLbsType HalfSurafaceArea() const;
	Vector3D GetCenter() const;
	RtLbsType Volume() const;
	unsigned MaxAxisId() const;
	void Union(const Point3D& p);
	void Union(const Point3D* p);
	void Union(const BBox3D& bbox);
	RtLbsType Delta(unsigned k) const;
	void InvalidBBox();
	RtLbsType Intersect(Ray3DLite* ray, RtLbsType* fmax = 0) const;
	bool IsContainPoint(const Point3D& p) const; //检测包围盒是否包含某个点
	void Update(const Vector3D& offset);									//按照位移更新包围盒
};

inline BBox3D Union(const BBox3D& bbox, const Point3D& p) {
	BBox3D bbox_re = bbox;
	for (unsigned i = 0; i < 3; ++i) {
		if (p[i] < bbox.m_min[i])
			bbox_re.m_min[i] = p[i];
		if (p[i] > bbox.m_max[i])
			bbox_re.m_max[i] = p[i];
	}
	return bbox_re;
}

inline BBox3D Union(const BBox3D& bbox, const Point3D* p) {
	BBox3D bboxRe = bbox;
	for (unsigned i = 0; i < 3; ++i) {
		if ((*p)[i] < bbox.m_min[i])
			bboxRe.m_min[i] = (*p)[i];
		if ((*p)[i] > bbox.m_max[i])
			bboxRe.m_max[i] = (*p)[i];
	}
	return bboxRe;
}

inline BBox3D Union(const BBox3D& bbox0, const BBox3D& bbox1) {
	BBox3D bbox_re;
	for (unsigned i = 0; i < 3; ++i) {
		bbox_re.m_min[i] = std::min(bbox0.m_min[i], bbox1.m_min[i]);
		bbox_re.m_max[i] = std::max(bbox0.m_max[i], bbox1.m_max[i]);
	}
	return bbox_re;
}

RtLbsType Intersect_BBox3D(Ray3DLite* ray, const BBox3D& bbox, RtLbsType* fmax = 0);

#endif
