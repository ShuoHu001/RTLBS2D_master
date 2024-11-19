#include "diffractiongpu.h"


HOST_DEVICE_FUNC bool GenerateDiffractRaysGPU(const Intersection2DGPU& inter, Ray2DGPU& incident_ray, Wedge2DGPU* wedge, int prevInterId, Ray2DGPU* newRays, int raynum) {
	if (!inter.m_propagationProperty.m_hasDiffraction)
		return false;
	if (wedge->m_face1.m_refractN == wedge->m_face1.m_refractNOut)//�ֽ�����������ͬ������������
		return false;
	//unsigned raynum = DIFF_RAYNUM + 2;  //��������������Ϊ�����ֲ�������Ե�Ž�Ϊ0������
	double orientation = 1.0;  /** @brief	��ת���� 1:��ʱ�� -1:˳ʱ�룬Ĭ����ʱ��	*/
	double external_angle = wedge->m_fExternalAngle;
	//�����ʼ�������߷���
	Vector2D oa = wedge->m_vector1;
	Vector2D ob = wedge->m_vector2;
	Vector2D n1 = wedge->m_face1.m_normal;
	if (Cross(oa, n1) < 0)//��Ҫ����n1�ķ�����ת
		orientation = -1.0;//˳ʱ��
	//�ж����ڲ����仹���ⲿ����
	if (incident_ray.m_fRefractiveIndex == wedge->m_face1.m_refractN) {//�������ڲ���������,��Ϊ���,��ת����ȡ�෴����
		external_angle = TWO_PI - external_angle;
		orientation *= -1.0;//�л���ת����
	}
	RtLbsType new_m_ft = incident_ray.m_tMax + (wedge->m_edge - incident_ray.m_Ori).Length();//�����������߾���Դ�ľ���
	double delta_theta = external_angle / (raynum - 2);
	double half_theta = delta_theta / 2.0;
	double cos_halftheta = cos(half_theta);
	Ray2DGPU newRay;
	newRay.m_isValid = true;
	newRay.m_limTotl = incident_ray.m_limTotl - 1;//��������-1
	newRay.m_limDiff = incident_ray.m_limDiff - 1;//����������-1
	newRay.m_limRefl = incident_ray.m_limRefl;
	newRay.m_limTran = incident_ray.m_limTran;
	newRay.m_limScat = incident_ray.m_limScat;
	newRay.m_Ori = wedge->m_edge;
	newRay.m_tMin = new_m_ft;
	newRay.m_tMax = new_m_ft;
	newRay.m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;//��������������ϵ������ʲ���
	newRay.m_nodeType = NODE_DIFF;
	newRay.m_theta = half_theta;
	newRay.m_costheta = cos_halftheta;
	newRay.m_prevInterId = prevInterId;

	newRays[0] = newRay;
	newRays[0].m_Dir = wedge->m_vector1;
	newRays[0].m_theta = 0.0;
	newRays[0].m_costheta = 1.0;//�������߰��
	newRays[0].m_bsplit = false; //ǽ���Ե���߲�����
	
	newRays[1] = newRay;
	newRays[1].m_Dir = oa.Rotate(orientation * half_theta);//������ת����
	newRays[1].m_bsplit = true;              //�������߾��з������ԣ��µĹ���Դ��


	for (unsigned i = 2; i < raynum - 1; i++) {//�м�Ƕȷ���
		newRays[i] = newRay;
		newRays[i].m_Dir = oa.Rotate(orientation * delta_theta);//ÿ����תһ���Ƕȣ���ɢ��
		newRays[i].m_bsplit = true;
	}

	newRays[raynum - 1] = newRay;//���һ������,ǽ�巽��2
	newRays[raynum - 1].m_Dir = wedge->m_vector2;
	newRays[raynum - 1].m_theta = 0.0;
	newRays[raynum - 1].m_costheta = 1.0;
	newRays[raynum - 1].m_bsplit = false;//ǽ���Ե���߲�����
	return true;
}

HOST_DEVICE_FUNC bool GenerateDiffractRaysGPU(const Intersection2DGPU& inter, Ray2DGPU& incident_ray, Wedge2DGPU* wedge, int prevInterId, Ray2DGPU* newRays, TreeNodeGPU* nodes, int raynum, int layer)
{
	if (!inter.m_propagationProperty.m_hasDiffraction)
		return false;
	if (wedge->m_face1.m_refractN == wedge->m_face1.m_refractNOut)//�ֽ�����������ͬ������������
		return false;
	if (incident_ray.m_nodeType == NODE_TRANIN ||
		incident_ray.m_nodeType == NODE_ETRANIN) {					//͸���롢����͸����֮�󲻻��������
		return false;
	}
	//unsigned raynum = DIFF_RAYNUM + 2;  //��������������Ϊ�����ֲ�������Ե�Ž�Ϊ0������
	double orientation = 1.0;  /** @brief	��ת���� 1:��ʱ�� -1:˳ʱ�룬Ĭ����ʱ��	*/
	double external_angle = wedge->m_fExternalAngle;
	//�����ʼ�������߷���
	Vector2D oa = wedge->m_vector1;
	Vector2D ob = wedge->m_vector2;
	Vector2D n1 = wedge->m_face1.m_normal;
	if (Cross(oa, n1) < 0)//��Ҫ����n1�ķ�����ת
		orientation = -1.0;//˳ʱ��
	//�ж����ڲ����仹���ⲿ����
	if (incident_ray.m_fRefractiveIndex == wedge->m_face1.m_refractN) {//�������ڲ���������,��Ϊ���,��ת����ȡ�෴����
		external_angle = TWO_PI - external_angle;
		orientation *= -1.0;//�л���ת����
	}
	RtLbsType new_m_ft = incident_ray.m_tMax + (wedge->m_edge - incident_ray.m_Ori).Length();//�����������߾���Դ�ľ���
	double delta_theta = external_angle / (raynum - 2);
	double half_theta = delta_theta / 2.0;
	double cos_halftheta = cos(half_theta);
	Ray2DGPU newRay;
	newRay.m_isValid = true;
	newRay.m_limTotl = incident_ray.m_limTotl - 1;//��������-1
	newRay.m_limDiff = incident_ray.m_limDiff - 1;//����������-1
	newRay.m_limRefl = incident_ray.m_limRefl;
	newRay.m_limTran = incident_ray.m_limTran;
	newRay.m_limScat = incident_ray.m_limScat;
	newRay.m_Ori = wedge->m_edge;
	newRay.m_tMin = new_m_ft;
	newRay.m_tMax = new_m_ft;
	newRay.m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;//��������������ϵ������ʲ���
	newRay.m_nodeType = NODE_DIFF;
	newRay.m_theta = half_theta;
	newRay.m_costheta = cos_halftheta;
	newRay.m_prevInterId = prevInterId;

	newRays[0] = newRay;
	newRays[0].m_Dir = wedge->m_vector1;
	newRays[0].m_theta = 0.0;
	newRays[0].m_costheta = 1.0;//�������߰��
	newRays[0].m_bsplit = false; //ǽ���Ե���߲�����

	newRays[1] = newRay;
	newRays[1].m_Dir = oa.Rotate(orientation * half_theta);//������ת����
	newRays[1].m_bsplit = true;              //�������߾��з������ԣ��µĹ���Դ��


	for (unsigned i = 2; i < raynum - 1; i++) {//�м�Ƕȷ���
		newRays[i] = newRay;
		newRays[i].m_Dir = oa.Rotate(orientation * delta_theta);//ÿ����תһ���Ƕȣ���ɢ��
		newRays[i].m_bsplit = true;
	}

	newRays[raynum - 1] = newRay;//���һ������,ǽ�巽��2
	newRays[raynum - 1].m_Dir = wedge->m_vector2;
	newRays[raynum - 1].m_theta = 0.0;
	newRays[raynum - 1].m_costheta = 1.0;
	newRays[raynum - 1].m_bsplit = false;//ǽ���Ե���߲�����

	//�����������ĸ�ֵ
	for (int i = 0; i < raynum; ++i) {
		nodes[i].m_isValid = true;
		nodes[i].m_type = NODE_DIFF;
		nodes[i].m_depth = layer;
		nodes[i].m_t = new_m_ft;
		nodes[i].m_point = wedge->m_edge;
		nodes[i].m_segmentId = inter.m_segmentId;
		nodes[i].m_wedgeId = inter.m_wedgeId;
		nodes[i].m_nextRay = newRays[i];
	}

	return true;
}
