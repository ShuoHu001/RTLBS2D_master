#include "reflection.h"


bool GenerateReflectRay(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray) {
	if (inter.m_segment->m_refractN == inter.m_segment->m_refractNOut)//��Ԫ�������������������ͬ���޷���
		return false;
	if (!inter.m_propagationProperty.m_hasRelfection)			//����Ԫ�����з������ԣ��򷵻�false
		return false;
	if (incident_ray.m_fRefractiveIndex == inter.m_segment->m_refractN) {		//������nֵ����Ԫnֵ��ͬ�������������ڵķ���·��
		return false;
	}
	//��������������ⷴ����������
	ray->m_Dir = incident_ray.m_Dir - 2 * (incident_ray.m_Dir * inter.m_segment->m_normal) * inter.m_segment->m_normal;
	ray->m_Ori = inter.m_intersect;
	ray->m_fRefractiveIndex = incident_ray.m_fRefractiveIndex; //��������������ʽ��ʲ���
	ray->m_fMin = incident_ray.m_fMin;
	ray->m_fMax = incident_ray.m_fMax + inter.m_ft;//���Ӵ�������
	ray->m_theta = incident_ray.m_theta;
	ray->m_costheta = incident_ray.m_costheta;
	ray->m_bsplit = incident_ray.m_bsplit;
	ray->m_vWedge = incident_ray.m_vWedge;
	ray->m_sensorDataId = incident_ray.m_sensorDataId;								//����������ID��ֵ
	return true;
}

void GenerateReflectRayOnToughSurface(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray)
{
}
