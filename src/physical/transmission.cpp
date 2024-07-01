#include "transmission.h"


bool GenerateTransmitRay(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray, PATHNODETYPE& type) {
	type = NODE_TRANIN;				//默认为透射入状态
	if (!inter.m_propagationProperty.m_hasTransmission)			//若面元无透射属性，则返回false
		return false;
	if (inter.m_segment->m_refractN == inter.m_segment->m_refractNOut) {//介质为“空气”介质，不改变传播方向
		ray->m_Dir = incident_ray.m_Dir;
		ray->m_Ori = inter.m_intersect;
		ray->m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;
		ray->m_fMin = incident_ray.m_fMin;
		ray->m_fMax = incident_ray.m_fMax + inter.m_ft;//叠加传播距离
		ray->m_theta = incident_ray.m_theta;
		ray->m_costheta = incident_ray.m_costheta;
		ray->m_bsplit = incident_ray.m_bsplit;
		ray->m_vWedge = incident_ray.m_vWedge;
		ray->m_sensorDataId = incident_ray.m_sensorDataId;
		return true;
	}
	RtLbsType n1, n2;//n1:前一种介质的折射率，n2:后一种介质的折射率
	n1 = incident_ray.m_fRefractiveIndex;
	n2 = inter.m_segment->m_refractN;                                           //默认射线从外部→内部传播，采用介质折射率
	Vector2D I = incident_ray.m_Dir;
	Vector2D N = inter.m_segment->m_normal;
	if (n1 == n2) {//射线上的折射率与面元内部折射率相同，射线方向：内部→外部
		N = -N;//内部情况下，扭转面元法向
		n2 = inter.m_segment->m_refractNOut;
		type = NODE_TRANOUT;			//从介质中传出时为透射出节点
	}
	double dotIN = I * N;
	double critical_angle = asin(n2 / n1);
	double incident_angle = acos(-dotIN);
	if (incident_angle > critical_angle)//大于临界角，不发生透射
		return false;
	RtLbsType ratio = n1 / n2;


	Vector2D I_prep = I - static_cast<RtLbsType>(dotIN) * N; //入射方向沿法向量垂直方向的投影向量
	Vector2D T_prep = ratio * I_prep; //透射方向沿法向量垂直方向的投影分量
	double cos_theta_T = sqrt(1.0 - T_prep.SquaredLength());
	Vector2D T_N = static_cast<RtLbsType>(-1.0 * cos_theta_T) * N;//透射法向沿法向量方向上的分量


	ray->m_Dir = T_prep + T_N;
	if (isnan(ray->m_Dir.x))
		LOG_ERROR << "计算到临界角，请检查错误" << CRASH;
	ray->m_Ori = inter.m_intersect;
	ray->m_fRefractiveIndex = n2; /** @brief	进入到"出射介质体中"	*/
	ray->m_fMin = incident_ray.m_fMin;
	ray->m_fMax = incident_ray.m_fMax + inter.m_ft;//叠加传播距离
	ray->m_theta = incident_ray.m_theta;
	ray->m_costheta = incident_ray.m_costheta;
	ray->m_bsplit = incident_ray.m_bsplit; 
	ray->m_vWedge = incident_ray.m_vWedge;
	ray->m_sensorDataId = incident_ray.m_sensorDataId;
	return true;
}

bool GenerateEmpiricalTransmitRay(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray, PATHNODETYPE& type)
{
	type = NODE_ETRANIN;													//默认为经验透射入节点
	if (!inter.m_propagationProperty.m_hasEmpiricalTransmission)			//交点不具备经验穿透属性，直接返回
		return false;
	const Vector2D& inDir = incident_ray.m_Dir;								/** @brief	入射方向	*/
	const Vector2D& nDir = inter.m_segment->m_normal;						/** @brief	面元法线方向	*/
	if (inDir * nDir > 0) {													//点乘大于0，表明从介质中传出，为经验透射出节点
		type = NODE_ETRANOUT;
	}
	ray->m_Dir = incident_ray.m_Dir;
	ray->m_Ori = inter.m_intersect;
	ray->m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;
	ray->m_fMin = incident_ray.m_fMin;
	ray->m_fMax = incident_ray.m_fMax + inter.m_ft;//叠加传播距离
	ray->m_theta = incident_ray.m_theta;
	ray->m_costheta = incident_ray.m_costheta;
	ray->m_bsplit = incident_ray.m_bsplit;
	ray->m_vWedge = incident_ray.m_vWedge;
	ray->m_sensorDataId = incident_ray.m_sensorDataId;
	return true;
}
