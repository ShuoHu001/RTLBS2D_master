#include "ray3dlite.h"

Ray3DLite::Ray3DLite()
	: m_facetId(-1)
{
}

Ray3DLite::Ray3DLite(const Point3D& ori, const Vector3D& dir)
	: m_ori(ori)
	, m_dir(dir)
	, m_facetId(-1)
{
}

Ray3DLite::Ray3DLite(const Point3D& ori, const Point3D& tar)
	: m_ori(ori)
	, m_facetId(-1)
{
	m_dir = (tar - ori).Normalize();
}


Ray3DLite::Ray3DLite(const Ray3DLite& r)
	: m_ori(r.m_ori)
	, m_dir(r.m_dir)
	, m_facetId(r.m_facetId)
{
}

Ray3DLite::~Ray3DLite()
{
}

Point3D Ray3DLite::operator()(RtLbsType t) const
{
	return m_ori + m_dir * t;
}

Ray3DLite& Ray3DLite::operator=(const Ray3DLite& ray)
{
	m_ori = ray.m_ori;
	m_dir = ray.m_dir;
	m_facetId = ray.m_facetId;
	return *this;
}

Point3D Ray3DLite::GetRayCoordinate(RtLbsType t) const
{
	return m_ori + m_dir * t;
}

RtLbsType Ray3DLite::GetRayHeight(RtLbsType t2d) const
{
	//1-计算三维的射线长度
	if (abs(m_dir.z - 1.0) < EPSILON) //若z值为1高度为无穷，朝上方向,返回高度最大值
		return FLT_MAX;
	RtLbsType costheta = sqrt(1 - m_dir.z * m_dir.z); //计算射线方向与XOY平面上的夹角余弦
	RtLbsType t3d = t2d / costheta;
	RtLbsType height = t3d * m_dir.z;//2-计算三维射线所对应的高度并返回
	return m_ori.z + height;
}

