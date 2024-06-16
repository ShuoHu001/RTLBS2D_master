#include "ray2dgpu.h"
#include "Intersection2Dgpu.h"
#include "accel/gpu/sdfgpu.h"
#include "segment2dgpu.h"
#include "Intersection2Dgpu.h"

HOST_DEVICE_FUNC Ray2DGPU::Ray2DGPU()
	: m_isValid(false)
	, m_fMax(-FLT_MAX)
	, m_fMin(FLT_MAX)
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
	m_fMax = other.m_fMax;
	m_fMin = other.m_fMin;
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
	m_fMax = ray.m_fMax;
	m_fMin = ray.m_fMin;
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
	RtLbsType t_total = t + m_fMax - m_fMin;//修正-计算从广义源发出的射线半径角度
	RtLbsType r = t_total * static_cast<RtLbsType>(sin(m_theta) / m_costheta);
	return r;
}

HOST_DEVICE_FUNC bool Ray2DGPU::IsCaptureByWedgePoint(Point2D p, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments) const
{
	//计算广义源位置
	RtLbsType t = m_fMax - m_fMin;//与广义源的距离
	Point2D vSource = m_Ori + m_Dir * -t;
	Vector2D op = (p - vSource).Normalize();
	double op_costheta = op * m_Dir;
	if (op_costheta < m_costheta)//op 张角大于射线张角，未被射线管捕捉
		return false;
	Ray2DGPU newRay(*this);
	newRay.m_Ori = vSource;
	newRay.m_Dir = op;
	//下面分为广义源和非广义源进行讨论：广义源无需修正路径节点，非广义源需要修正路径节点

	if (m_nodeType == NODE_ROOT || m_nodeType == NODE_DIFF) {//广义源情形判定
		Intersection2DGPU inter1;
		sdf->GetIntersect(newRay, &inter1, segments);
		if (!inter1.m_isValid)//不相交
			return false;
		if (inter1.m_intersect != p)
			return false;
		//*intersect = inter1;
		return true;
	}

	//非广义源情况
	Intersection2DGPU inter1;
	const Segment2DGPU& segment = segments[m_primitiveId];
	if (!segment.GetIntersect(newRay, &inter1))
		return false;
	Vector2D ip = (p - inter1.m_intersect).Normalize();
	if (op * ip < 0)//p 点在面元后方
		return false;
	newRay.m_Ori = inter1.m_intersect;
	Intersection2DGPU inter2;
	sdf->GetIntersect(newRay, &inter2, segments);
	if (!inter2.m_isValid)
		return false;
	if (inter2.m_intersect != p)
		return false;
	//*intersect = inter2;
	return true;
	//这里由于GPU的性能限制，不进行重复绕射判定
}

HOST_DEVICE_FUNC Point2D Ray2DGPU::GetVisualSource() const
{
	RtLbsType tRef = m_fMax - m_fMin;//当前节点到广义源的距离
	return (*this)(-tRef);
}

HOST_DEVICE_FUNC RtLbsType Ray2DGPU::GetSquaredDistanceToPoint(const Point2D& p) const
{
	Vector2D op = p - m_Ori;
	Vector2D oq = m_Dir * op * m_Dir;
	Vector2D qp = op - oq;
	return qp.x * qp.x + qp.y * qp.y;
}
