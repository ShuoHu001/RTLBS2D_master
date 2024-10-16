#include "reflectiongpu.h"


HOST_DEVICE_FUNC bool GenerateReflectRayGPU(const Intersection2DGPU& inter, int interId, const Segment2DGPU* segments, Ray2DGPU* ray) {
	if (!inter.m_propagationProperty.m_hasRelfection)				//���޷������ԣ��򲻲�����������
		return false;
	const Segment2DGPU& segment = segments[inter.m_segmentId];
	const Ray2DGPU& incident_ray = inter.m_ray;
	if (segment.m_refractN == segment.m_refractNOut)//��Ԫ�������������������ͬ���޷���
		return false;
	//��������������ⷴ����������
	ray->m_isValid = true;
	ray->m_limTotl = incident_ray.m_limTotl - 1;
	ray->m_limRefl = incident_ray.m_limRefl - 1;
	ray->m_limTran = incident_ray.m_limTran;
	ray->m_limDiff = incident_ray.m_limDiff;
	ray->m_limScat = incident_ray.m_limScat;
	ray->m_Dir = incident_ray.m_Dir - 2 * (incident_ray.m_Dir * segment.m_normal) * segment.m_normal;
	ray->m_Ori = inter.m_intersect;
	ray->m_fRefractiveIndex = incident_ray.m_fRefractiveIndex; //��������������ʽ��ʲ���
	ray->m_tMin = incident_ray.m_tMin;
	ray->m_tMax = incident_ray.m_tMax + inter.m_ft;//���Ӵ�������
	ray->m_theta = incident_ray.m_theta;
	ray->m_costheta = incident_ray.m_costheta;
	ray->m_bsplit = incident_ray.m_bsplit;
	ray->m_primitiveId = inter.m_segmentId;
	ray->m_nodeType = NODE_REFL;
	ray->m_prevInterId = interId;
	return true;
}

HOST_DEVICE_FUNC bool GenerateReflectRayGPU(const Intersection2DGPU& inter, int interId, const Segment2DGPU* segments, Ray2DGPU* ray, TreeNodeGPU* treenode, int layer)
{
	if (!inter.m_propagationProperty.m_hasRelfection)				//���޷������ԣ��򲻲�����������
		return false;
	const Segment2DGPU& segment = segments[inter.m_segmentId];
	const Ray2DGPU& incident_ray = inter.m_ray;
	if (segment.m_refractN == segment.m_refractNOut)//��Ԫ�������������������ͬ���޷���
		return false;
	if (incident_ray.m_nodeType == NODE_TRANIN ||
		incident_ray.m_nodeType == NODE_ETRANIN) {				//�������߽ڵ�Ϊ͸����ڵ㣬�򲻻���з���
		return false;
	}
	//��������������ⷴ����������
	ray->m_isValid = true;
	ray->m_limTotl = incident_ray.m_limTotl - 1;
	ray->m_limRefl = incident_ray.m_limRefl - 1;
	ray->m_limTran = incident_ray.m_limTran;
	ray->m_limDiff = incident_ray.m_limDiff;
	ray->m_limScat = incident_ray.m_limScat;
	ray->m_Dir = incident_ray.m_Dir - 2 * (incident_ray.m_Dir * segment.m_normal) * segment.m_normal;
	ray->m_Ori = inter.m_intersect;
	ray->m_fRefractiveIndex = incident_ray.m_fRefractiveIndex; //��������������ʽ��ʲ���
	ray->m_tMin = incident_ray.m_tMin;
	ray->m_tMax = incident_ray.m_tMax + inter.m_ft;//���Ӵ�������
	ray->m_theta = incident_ray.m_theta;
	ray->m_costheta = incident_ray.m_costheta;
	ray->m_bsplit = incident_ray.m_bsplit;
	ray->m_primitiveId = inter.m_segmentId;
	ray->m_nodeType = NODE_REFL;
	ray->m_prevInterId = interId;

	//�����������ڵ�
	treenode->m_isValid = true;
	treenode->m_type = NODE_REFL;
	treenode->m_depth = layer;
	treenode->m_t = ray->m_tMax;
	treenode->m_point = inter.m_intersect;
	treenode->m_segmentId = inter.m_segmentId;
	treenode->m_wedgeId = inter.m_wedgeId;
	treenode->m_nextRay = *ray;

	return true;
}
