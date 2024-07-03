#include "Intersection2Dgpu.h"
#include "ray2dgpu.h"

HOST_DEVICE_FUNC Intersection2DGPU::Intersection2DGPU()
	: m_isValid(false)
	, m_type(NODE_INIT)
	, m_ft(0.0)
	, m_matId(-1)
	, m_segmentId(-1)
	, m_wedgeId(-1)
	, m_prevId(-1)
{
}
HOST_DEVICE_FUNC Intersection2DGPU::Intersection2DGPU(const Intersection2DGPU& intersect)
	: m_isValid(intersect.m_isValid)
	, m_intersect(intersect.m_intersect)
	, m_type(intersect.m_type)
	, m_ft(intersect.m_ft)
	, m_matId(intersect.m_matId)
	, m_segmentId(intersect.m_segmentId)
	, m_wedgeId(intersect.m_wedgeId)
	, m_ray(intersect.m_ray)
	, m_prevId(intersect.m_prevId)
	, m_propagationProperty(intersect.m_propagationProperty)
{
}
HOST_DEVICE_FUNC Intersection2DGPU::~Intersection2DGPU()
{
}

HOST_DEVICE_FUNC Intersection2DGPU& Intersection2DGPU::operator=(const Intersection2DGPU& intersect)
{
	m_isValid = intersect.m_isValid;
	m_intersect = intersect.m_intersect;
	m_type = intersect.m_type;
	m_ft = intersect.m_ft;
	m_matId = intersect.m_matId;
	m_segmentId = intersect.m_segmentId;
	m_wedgeId = intersect.m_wedgeId;
	m_ray = intersect.m_ray;
	m_prevId = intersect.m_prevId;
	m_propagationProperty = intersect.m_propagationProperty;
	return *this;
}

HOST_DEVICE_FUNC bool Intersection2DGPU::IsCaptureRx(Point2D rx)
{
	//һ��-����Դ
	RtLbsType t_refVsource = m_ray.m_fMax - m_ray.m_fMin; //t����ڹ���Դ��ֵ
	RtLbsType tmax = t_refVsource + m_ft;
	Point2D vSource = m_ray(-t_refVsource);
	Vector2D op = rx - vSource;
	RtLbsType t_op = op.Length();
	//����-�����ж�
	if (t_op > tmax || t_op < t_refVsource)//���ڷ�Χ��
		return false;
	double costheta_op = m_ray.m_Dir * op / t_op;
	//����-�Ƕ��ж�
	if (costheta_op < m_ray.m_costheta) //�������߹ܽǶ���
		return false;
	return true;
}

HOST_DEVICE_FUNC Point2D Intersection2DGPU::GetVisualSource()
{
	return m_ray.GetVisualSource();
}

