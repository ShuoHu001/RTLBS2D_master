#include "ray2dgpu.h"
#include "Intersection2Dgpu.h"
#include "accel/gpu/sdfgpu.h"
#include "segment2dgpu.h"
#include "Intersection2Dgpu.h"

HOST_DEVICE_FUNC Ray2DGPU::Ray2DGPU()
	: m_isValid(false)
	, m_tMax(-FLT_MAX)
	, m_tMin(FLT_MAX)
	, m_tLimit(FLT_MAX)
	, m_theta(0.0)
	, m_costheta(0.0)
	, m_nodeType(NODE_INIT)
	, m_primitiveId(-1)
	, m_fRefractiveIndex(1.0)
	, m_bsplit(false)
	, m_limTotl(0)
	, m_limRefl(0)
	, m_limTran(0)
	, m_limDiff(0)
	, m_limScat(0)
	, m_prevInterId(-1)
	, m_sensorDataId(-1)
{
}

HOST_DEVICE_FUNC Ray2DGPU::Ray2DGPU(const Ray2DGPU& other)
{
	m_isValid = other.m_isValid;
	m_Ori = other.m_Ori;
	m_Dir = other.m_Dir;
	m_tMax = other.m_tMax;
	m_tMin = other.m_tMin;
	m_tLimit = other.m_tLimit;
	m_theta = other.m_theta;
	m_costheta = other.m_costheta;
	m_nodeType = other.m_nodeType;
	m_primitiveId = other.m_primitiveId;
	m_fRefractiveIndex = other.m_fRefractiveIndex;
	m_bsplit = other.m_bsplit;
	m_limTotl = other.m_limTotl;
	m_limRefl = other.m_limRefl;
	m_limTran = other.m_limTran;
	m_limDiff = other.m_limDiff;
	m_limScat = other.m_limScat;
	m_prevInterId = other.m_prevInterId;
	m_sensorDataId = other.m_sensorDataId;
}

HOST_DEVICE_FUNC Ray2DGPU& Ray2DGPU::operator=(const Ray2DGPU& ray)
{
	m_isValid = ray.m_isValid;
	m_Ori = ray.m_Ori;
	m_Dir = ray.m_Dir;
	m_tMax = ray.m_tMax;
	m_tMin = ray.m_tMin;
	m_tLimit = ray.m_tLimit;
	m_theta = ray.m_theta;
	m_costheta = ray.m_costheta;
	m_nodeType = ray.m_nodeType;
	m_primitiveId = ray.m_primitiveId;
	m_fRefractiveIndex = ray.m_fRefractiveIndex;
	m_bsplit = ray.m_bsplit;
	m_limTotl = ray.m_limTotl;
	m_limRefl = ray.m_limRefl;
	m_limTran = ray.m_limTran;
	m_limDiff = ray.m_limDiff;
	m_limScat = ray.m_limScat;
	m_prevInterId = ray.m_prevInterId;
	m_sensorDataId = ray.m_sensorDataId;
	return *this;
}

HOST_DEVICE_FUNC Point2D Ray2DGPU::operator()(RtLbsType t) const
{
	return m_Ori + m_Dir * t;
}

HOST_DEVICE_FUNC RtLbsType Ray2DGPU::GetRayRadis(RtLbsType t) const
{
	//根据theta角和t值确定在t处的半径
	RtLbsType t_total = t + m_tMax - m_tMin;//修正-计算从广义源发出的射线半径角度
	RtLbsType r = t_total * static_cast<RtLbsType>(sin(m_theta) / m_costheta);
	return r;
}


HOST_DEVICE_FUNC Point2D Ray2DGPU::GetVisualSource() const
{
	RtLbsType tRef = m_tMax - m_tMin;//当前节点到广义源的距离
	return (*this)(-tRef);
}

HOST_DEVICE_FUNC RtLbsType Ray2DGPU::GetSquaredDistanceToPoint(const Point2D& p) const
{
	Vector2D op = p - m_Ori;
	Vector2D oq = m_Dir * op * m_Dir;
	Vector2D qp = op - oq;
	return qp.x * qp.x + qp.y * qp.y;
}
