#include "transmission.h"


bool GenerateTransmitRay(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray, PATHNODETYPE& type) {
	type = NODE_TRANIN;				//Ĭ��Ϊ͸����״̬
	if (!inter.m_propagationProperty.m_hasTransmission)			//����Ԫ��͸�����ԣ��򷵻�false
		return false;
	if (inter.m_segment->m_refractN == inter.m_segment->m_refractNOut) {//����Ϊ�����������ʣ����ı䴫������
		ray->m_Dir = incident_ray.m_Dir;
		ray->m_Ori = inter.m_intersect;
		ray->m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;
		ray->m_fMin = incident_ray.m_fMin;
		ray->m_fMax = incident_ray.m_fMax + inter.m_ft;//���Ӵ�������
		ray->m_theta = incident_ray.m_theta;
		ray->m_costheta = incident_ray.m_costheta;
		ray->m_bsplit = incident_ray.m_bsplit;
		ray->m_vWedge = incident_ray.m_vWedge;
		ray->m_sensorDataId = incident_ray.m_sensorDataId;
		return true;
	}
	RtLbsType n1, n2;//n1:ǰһ�ֽ��ʵ������ʣ�n2:��һ�ֽ��ʵ�������
	n1 = incident_ray.m_fRefractiveIndex;
	n2 = inter.m_segment->m_refractN;                                           //Ĭ�����ߴ��ⲿ���ڲ����������ý���������
	Vector2D I = incident_ray.m_Dir;
	Vector2D N = inter.m_segment->m_normal;
	if (n1 == n2) {//�����ϵ�����������Ԫ�ڲ���������ͬ�����߷����ڲ����ⲿ
		N = -N;//�ڲ�����£�Ťת��Ԫ����
		n2 = inter.m_segment->m_refractNOut;
		type = NODE_TRANOUT;			//�ӽ����д���ʱΪ͸����ڵ�
	}
	double dotIN = I * N;
	double critical_angle = asin(n2 / n1);
	double incident_angle = acos(-dotIN);
	if (incident_angle > critical_angle)//�����ٽ�ǣ�������͸��
		return false;
	RtLbsType ratio = n1 / n2;


	Vector2D I_prep = I - static_cast<RtLbsType>(dotIN) * N; //���䷽���ط�������ֱ�����ͶӰ����
	Vector2D T_prep = ratio * I_prep; //͸�䷽���ط�������ֱ�����ͶӰ����
	double cos_theta_T = sqrt(1.0 - T_prep.SquaredLength());
	Vector2D T_N = static_cast<RtLbsType>(-1.0 * cos_theta_T) * N;//͸�䷨���ط����������ϵķ���


	ray->m_Dir = T_prep + T_N;
	if (isnan(ray->m_Dir.x))
		LOG_ERROR << "���㵽�ٽ�ǣ��������" << CRASH;
	ray->m_Ori = inter.m_intersect;
	ray->m_fRefractiveIndex = n2; /** @brief	���뵽"�����������"	*/
	ray->m_fMin = incident_ray.m_fMin;
	ray->m_fMax = incident_ray.m_fMax + inter.m_ft;//���Ӵ�������
	ray->m_theta = incident_ray.m_theta;
	ray->m_costheta = incident_ray.m_costheta;
	ray->m_bsplit = incident_ray.m_bsplit; 
	ray->m_vWedge = incident_ray.m_vWedge;
	ray->m_sensorDataId = incident_ray.m_sensorDataId;
	return true;
}

bool GenerateEmpiricalTransmitRay(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray, PATHNODETYPE& type)
{
	type = NODE_ETRANIN;													//Ĭ��Ϊ����͸����ڵ�
	if (!inter.m_propagationProperty.m_hasEmpiricalTransmission)			//���㲻�߱����鴩͸���ԣ�ֱ�ӷ���
		return false;
	const Vector2D& inDir = incident_ray.m_Dir;								/** @brief	���䷽��	*/
	const Vector2D& nDir = inter.m_segment->m_normal;						/** @brief	��Ԫ���߷���	*/
	if (inDir * nDir > 0) {													//��˴���0�������ӽ����д�����Ϊ����͸����ڵ�
		type = NODE_ETRANOUT;
	}
	ray->m_Dir = incident_ray.m_Dir;
	ray->m_Ori = inter.m_intersect;
	ray->m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;
	ray->m_fMin = incident_ray.m_fMin;
	ray->m_fMax = incident_ray.m_fMax + inter.m_ft;//���Ӵ�������
	ray->m_theta = incident_ray.m_theta;
	ray->m_costheta = incident_ray.m_costheta;
	ray->m_bsplit = incident_ray.m_bsplit;
	ray->m_vWedge = incident_ray.m_vWedge;
	ray->m_sensorDataId = incident_ray.m_sensorDataId;
	return true;
}
