#include "segment2dgpu.h"
#include "Intersection2Dgpu.h"
#include "ray2dgpu.h"




HOST_DEVICE_FUNC Segment2DGPU::Segment2DGPU()
	: m_length(0.0)
	, m_primitive_id(-1)
	, m_matId(-1)
	, m_refractN(1.0)
	, m_refractNOut(1.0)
{
}

HOST_DEVICE_FUNC Segment2DGPU::Segment2DGPU(Point2D ps, Point2D pe, Vector2D normal, Vector2D dir, RtLbsType length, BBox2DGPU bbox, int primitive_id, int matId, RtLbsType refractN, RtLbsType refractNOut)
	: m_ps(ps)
	, m_pe(pe)
	, m_normal(normal)
	, m_dir(dir)
	, m_length(length)
	, m_bbox(bbox)
	, m_primitive_id(primitive_id)
	, m_matId(matId)
	, m_refractN(refractN)
	, m_refractNOut(refractNOut)
{
}

Segment2DGPU::Segment2DGPU(const Segment2DGPU& segment)
	: m_ps(segment.m_ps)
	, m_pe(segment.m_pe)
	, m_normal(segment.m_normal)
	, m_dir(segment.m_dir)
	, m_length(segment.m_length)
	, m_bbox(segment.m_bbox)
	, m_primitive_id(segment.m_primitive_id)
	, m_matId(segment.m_matId)
	, m_refractN(segment.m_refractN)
	, m_refractNOut(segment.m_refractNOut)
	, m_propagationProperty(segment.m_propagationProperty)
{
}

HOST_DEVICE_FUNC Segment2DGPU::~Segment2DGPU()
{
}

HOST_DEVICE_FUNC bool Segment2DGPU::operator==(const Segment2DGPU& other) const
{
	if (m_primitive_id != other.m_primitive_id)
		return false;
	return true;
}

HOST_DEVICE_FUNC bool Segment2DGPU::operator!=(const Segment2DGPU& other) const
{
	return !(*this == other);
}

HOST_DEVICE_FUNC Segment2DGPU& Segment2DGPU::operator=(const Segment2DGPU& segment)
{
	if (*this != segment) {
		m_ps = segment.m_ps;
		m_pe = segment.m_pe;
		m_normal = segment.m_normal;
		m_dir = segment.m_dir;
		m_length = segment.m_length;
		m_bbox = segment.m_bbox;
		m_primitive_id = segment.m_primitive_id;
		m_matId = segment.m_matId;
		m_refractN = segment.m_refractN;
		m_refractNOut = segment.m_refractNOut;
		m_propagationProperty = segment.m_propagationProperty;
	}
	return (*this);
}

HOST_DEVICE_FUNC bool Segment2DGPU::GetIntersect(const Ray2DGPU& ray, Intersection2DGPU* intersect) const
{
	RtLbsType denominator = Cross(ray.m_Dir, m_dir);
	if (abs(denominator) < EPSILON) { //��������Ԫ�������
		if (abs(Cross(ray.m_Dir, m_ps - ray.m_Ori)) < EPSILON) {//��ǰ��������Ԫ����,���ֻ��һ�������

			if ((ray.m_Ori - m_ps) * (ray.m_Ori - m_pe) <= 0)//�ų������������Ԫ��
				return false;
			if (intersect) {
				RtLbsType tmax;
				RtLbsType tmin = Intersect_BBox2D(ray, m_bbox, &tmax);
				if (tmin < 0.0) {//���ཻ
					return false;
				}
				intersect->m_ft = tmin;//���½������Դ��ľ���
				intersect->m_segmentId = m_primitive_id;//���������Ԫ��ֵ�������ж�
				intersect->m_matId = m_matId;
				//ȡ�������ж��������������
				//����ڵ�
				intersect->m_type = NODE_REFL;
				intersect->m_intersect = ray.m_Ori + ray.m_Dir * tmin;
				intersect->m_isValid = true;
				intersect->m_ray = ray;
				intersect->m_propagationProperty = m_propagationProperty;
				return true;
			}
		}
		return false;
	}
	Vector2D p = m_ps - ray.m_Ori;
	RtLbsType t = static_cast<RtLbsType>(Cross(p, m_dir) / denominator /*- EPSILON*/); //t:���������Ԫ���ľ���,��Сt��ֵ�����ڹ�ܼ�������
	RtLbsType u = static_cast<RtLbsType>(Cross(p, ray.m_Dir) / denominator / (m_pe - m_ps).Length()); //�������߶��ϵİٷֱ�
	if (abs(u) < EPSILON) u = 0.0;
	if (abs(u - 1.0) < EPSILON) u = 1.0;
	if (t >= EPSILON && u >= 0.0 && u <= 1.0) {//���￼��t>EPSILONָ�����ж��������Ԫ�ϵ������ཻ,�����ӿ������
		if (intersect) {
			intersect->m_intersect = ray.m_Ori + ray.m_Dir * t;
			intersect->m_ft = t;
			intersect->m_segmentId = m_primitive_id; //����Id��ֵ
			intersect->m_matId = m_matId;
			intersect->m_type = NODE_REFL;//����ڵ�
			intersect->m_isValid = true;
			intersect->m_ray = ray;
			intersect->m_propagationProperty = m_propagationProperty;
			//ȡ�������ж��������������
		}
		return true;
	}
	return false;
}

HOST_DEVICE_FUNC const BBox2DGPU& Segment2DGPU::GetBBox() const
{
	return m_bbox;
}
