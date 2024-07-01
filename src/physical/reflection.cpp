#include "reflection.h"


bool GenerateReflectRay(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray) {
	if (inter.m_segment->m_refractN == inter.m_segment->m_refractNOut)//面元折射率与外界折射率相同，无反射
		return false;
	if (!inter.m_propagationProperty.m_hasRelfection)			//若面元不具有反射属性，则返回false
		return false;
	if (incident_ray.m_fRefractiveIndex == inter.m_segment->m_refractN) {		//若射线n值和面元n值相同，不产生介质内的反射路径
		return false;
	}
	//根据入射射线求解反射射线向量
	ray->m_Dir = incident_ray.m_Dir - 2 * (incident_ray.m_Dir * inter.m_segment->m_normal) * inter.m_segment->m_normal;
	ray->m_Ori = inter.m_intersect;
	ray->m_fRefractiveIndex = incident_ray.m_fRefractiveIndex; //反射过程中折射率介质不变
	ray->m_fMin = incident_ray.m_fMin;
	ray->m_fMax = incident_ray.m_fMax + inter.m_ft;//叠加传播距离
	ray->m_theta = incident_ray.m_theta;
	ray->m_costheta = incident_ray.m_costheta;
	ray->m_bsplit = incident_ray.m_bsplit;
	ray->m_vWedge = incident_ray.m_vWedge;
	ray->m_sensorDataId = incident_ray.m_sensorDataId;								//传感器数据ID赋值
	return true;
}

void GenerateReflectRayOnToughSurface(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray)
{
}
