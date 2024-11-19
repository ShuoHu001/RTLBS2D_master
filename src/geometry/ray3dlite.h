#ifndef RTLBS_RAY3DLITE
#define RTLBS_RAY3DLITE

#include "rtlbs.h"
#include "utility/define.h"
#include "math/point3d.h"
#include "math/vector3d.h"

//轻量化三维射线
class Ray3DLite {

public:
	Point3D m_ori;						/** @brief	射线起始坐标	*/
	Vector3D m_dir;						/** @brief	中心射线方向	*/
	int m_facetId;						/** @brief	射线基点所在的面元Id, 地形面元	*/

public:
	explicit Ray3DLite();
	Ray3DLite(const Point3D& ori, const Vector3D& dir); 
	Ray3DLite(const Point3D& ori, const Point3D& tar);
	Ray3DLite(const Ray3DLite& r);
	~Ray3DLite();
	Point3D operator () (RtLbsType t) const;
	Ray3DLite& operator = (const Ray3DLite& ray);
	Point3D GetRayCoordinate(RtLbsType t) const;
	RtLbsType GetRayHeight(RtLbsType t2d) const; //基于XOY二维线段长度计算出目标点处对应的高度
};

//全局函数
inline Point3D GetRayCoordinate(const Ray3DLite& ray, RtLbsType t) {
	return ray.m_ori + ray.m_dir * t;
}

inline RtLbsType GetSquaredDistanceToPoint(const Ray3DLite& ray, Point3D& p) {
	Vector3D op = p - ray.m_ori;
	Vector3D oq = ray.m_dir * op * ray.m_dir;
	Vector3D qp = op - oq;
	return qp.SquaredLength();
}

#endif
