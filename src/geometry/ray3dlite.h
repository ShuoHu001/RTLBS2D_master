#ifndef RTLBS_RAY3DLITE
#define RTLBS_RAY3DLITE

#include "rtlbs.h"
#include "utility/define.h"
#include "math/point3d.h"
#include "math/vector3d.h"

//��������ά����
class Ray3DLite {

public:
	Point3D m_ori;						/** @brief	������ʼ����	*/
	Vector3D m_dir;						/** @brief	�������߷���	*/
	int m_facetId;						/** @brief	���߻������ڵ���ԪId, ������Ԫ	*/

public:
	explicit Ray3DLite();
	Ray3DLite(const Point3D& ori, const Vector3D& dir); 
	Ray3DLite(const Point3D& ori, const Point3D& tar);
	Ray3DLite(const Ray3DLite& r);
	~Ray3DLite();
	Point3D operator () (RtLbsType t) const;
	Ray3DLite& operator = (const Ray3DLite& ray);
	Point3D GetRayCoordinate(RtLbsType t) const;
	RtLbsType GetRayHeight(RtLbsType t2d) const; //����XOY��ά�߶γ��ȼ����Ŀ��㴦��Ӧ�ĸ߶�
};

//ȫ�ֺ���
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
