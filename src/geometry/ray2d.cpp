#include "ray2d.h"

Ray2D::Ray2D()
	: m_fRefractiveIndex(1.0)
	, m_fMax(0.0)
	, m_fMin(0.0)
	, m_theta(0.0)
	, m_costheta(0.0)
	, m_bsplit(true)
	, m_sensorDataId(-1)
{
}

Ray2D::Ray2D(const Point2D& ori, const Vector2D& dir)
	: m_Ori(ori)
	, m_Dir(dir)
	, m_fRefractiveIndex(1.0)
	, m_fMax(0)
	, m_fMin(0)
	, m_theta(0.0)
	, m_costheta(0.0)
	, m_bsplit(true)
	, m_sensorDataId(-1)
{
}

Ray2D::Ray2D(const Point2D& ori, const Point2D& tar)
	: m_Ori(ori)
	, m_fRefractiveIndex(1.0)
	, m_fMax(0)
	, m_fMin(0)
	, m_theta(0.0)
	, m_costheta(0.0)
	, m_bsplit(true)
	, m_sensorDataId(-1)
{
	if (ori == tar) {
		LOG_ERROR << "ray set error: zero direction." << ENDL;
	}
	m_Dir = (tar - ori).Normalize();
}

Ray2D::Ray2D(const Point2D& ori, const Vector2D& dir, double theta, double costheta)
	: m_Ori(ori)
	, m_Dir(dir)
	, m_fRefractiveIndex(1.0)
	, m_fMax(0)
	, m_fMin(0)
	, m_theta(theta)
	, m_costheta(costheta)
	, m_bsplit(true)
	, m_sensorDataId(-1)
{
}

Ray2D::Ray2D(const Ray2DGPU& rayGPU)
	: m_Ori(rayGPU.m_Ori)
	, m_Dir(rayGPU.m_Dir)
	, m_fRefractiveIndex(rayGPU.m_fRefractiveIndex)
	, m_fMax(rayGPU.m_fMax)
	, m_fMin(rayGPU.m_fMin)
	, m_theta(rayGPU.m_theta)
	, m_costheta(rayGPU.m_costheta)
	, m_bsplit(rayGPU.m_bsplit)
	, m_sensorDataId(rayGPU.m_sensorDataId)
{
}

Ray2D::Ray2D(const Ray2D& r)
	: m_Ori(r.m_Ori)
	, m_Dir(r.m_Dir)
	, m_fRefractiveIndex(r.m_fRefractiveIndex)
	, m_fMax(r.m_fMax)
	, m_fMin(r.m_fMin)
	, m_theta(r.m_theta)
	, m_costheta(r.m_costheta)
	, m_bsplit(r.m_bsplit)
	, m_sensorDataId(r.m_sensorDataId)
	, m_vWedge(r.m_vWedge)
{
}

Ray2D Ray2D::operator=(const Ray2D& ray)
{
	m_Ori = ray.m_Ori;
	m_Dir = ray.m_Dir;
	m_fRefractiveIndex = ray.m_fRefractiveIndex;
	m_fMax = ray.m_fMax;
	m_fMin = ray.m_fMin;
	m_theta = ray.m_theta;
	m_costheta = ray.m_costheta;
	m_bsplit = ray.m_bsplit;
	m_sensorDataId = ray.m_sensorDataId;
	m_vWedge = ray.m_vWedge;
	return *this;
}

Ray2D Ray2D::operator=(const Ray2DGPU& rayGPU)
{
	m_Ori = rayGPU.m_Ori;
	m_Dir = rayGPU.m_Dir;
	m_fRefractiveIndex = rayGPU.m_fRefractiveIndex;
	m_fMax = rayGPU.m_fMax;
	m_fMin = rayGPU.m_fMin;
	m_theta = rayGPU.m_theta;
	m_costheta = rayGPU.m_costheta;
	m_bsplit = rayGPU.m_bsplit;
	m_sensorDataId = rayGPU.m_sensorDataId;
	return *this;
}

Point2D Ray2D::operator()(RtLbsType t) const
{
	return m_Ori + m_Dir * t;
}

Point2D Ray2D::GetRayCoordinate(RtLbsType t)
{
    return m_Ori + m_Dir * t;
}

RtLbsType Ray2D::GetRayRadius(RtLbsType t) const
{
	//根据theta角和t值确定在t处的半径
	RtLbsType t_total = t + m_fMax - m_fMin;//修正-计算从广义源发出的射线半径角度
	RtLbsType r = t_total * static_cast<RtLbsType>(sin(m_theta) / m_costheta);
	return r;
}

RtLbsType Ray2D::GetSquaredDistanceToPoint(const Point2D& p)
{
	Vector2D op = p - m_Ori;
	Vector2D oq = m_Dir * op * m_Dir;
	Vector2D qp = op - oq;
	return qp.x * qp.x + qp.y * qp.y;
}

Ray2DGPU Ray2D::Convert2GPU() const
{
	Ray2DGPU rayGPU;
	rayGPU.m_Ori = m_Ori;
	rayGPU.m_Dir = m_Dir;
	rayGPU.m_fMax = m_fMax;
	rayGPU.m_fMin = m_fMin;
	rayGPU.m_costheta = m_costheta;
	rayGPU.m_theta = m_theta;
	rayGPU.m_sensorDataId = m_sensorDataId;
	return rayGPU;
}


