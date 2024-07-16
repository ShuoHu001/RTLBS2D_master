#include "diffraction.h"

bool GenerateDiffractRays(const Ray2D& incident_ray, Wedge2D* wedge, std::vector<Ray2D>* rays) {

	if (wedge->m_face1->m_refractN == wedge->m_face1->m_refractNOut)//�ֽ�����������ͬ������������
		return false;
	if (!wedge->m_face1->m_propagationProperty.m_hasDiffraction)		//��û���������ԣ��򷵻�false
		return false;
	if (incident_ray.m_fRefractiveIndex == wedge->m_face1->m_refractN) {	//���������������ʺ�������������ͬ�����ݹ��򣬲����������·��
		return false;
	}
	unsigned raynum = _global_diffractRayNum + RANDINT(0, DIFF_DELTARAYNUM) + 2;  //��������������Ϊ�����ֲ�������Ե�Ž�Ϊ0������
	double orientation = 1.0;  /** @brief	��ת���� 1:��ʱ�� -1:˳ʱ�룬Ĭ����ʱ��	*/
	double external_angle = wedge->m_theta;
	(*rays).resize(raynum);
	//�����ʼ�������߷���
	Vector2D oa = wedge->m_dir1;
	Vector2D ob = wedge->m_dir2;
	Vector2D n1 = wedge->m_face1->m_normal;
	if (Cross(oa, n1) < 0)//��Ҫ����n1�ķ�����ת
		orientation = -1.0;//˳ʱ��
	//�ж����ڲ����仹���ⲿ����
	if (incident_ray.m_fRefractiveIndex == wedge->m_face1->m_refractN) {//�������ڲ���������,��Ϊ���,��ת����ȡ�෴����
		external_angle = TWO_PI - external_angle;
		orientation *= -1.0;//�л���ת����
	}
	RtLbsType new_m_ft = incident_ray.m_fMax + (wedge->m_point - incident_ray.m_Ori).Length();//�����������߾���Դ�ľ���
	double delta_theta = external_angle / (raynum - 2);
	double half_theta = delta_theta / 2.0;
	double cos_halftheta = cos(half_theta);
	(*rays)[0].m_Ori = wedge->m_point;//��һ���Ƕȷ���,ǽ�巽��1
	(*rays)[0].m_Dir = wedge->m_dir1;
	(*rays)[0].m_fMin = new_m_ft;
	(*rays)[0].m_fMax = new_m_ft;
	(*rays)[0].m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;//��������������ϵ������ʲ���
	(*rays)[0].m_theta = 0.0;
	(*rays)[0].m_costheta = 1.0;//�������߰��
	(*rays)[0].m_bsplit = false; //ǽ���Ե���߲�����
	(*rays)[0].m_vWedge = incident_ray.m_vWedge;
	(*rays)[0].m_vWedge.push_back(wedge);
	(*rays)[0].m_sensorDataId = incident_ray.m_sensorDataId;									//����������ID��ֵ

	(*rays)[1].m_Ori = wedge->m_point;  //�ֲ���һ�����ߵ�half_theta�հ�
	(*rays)[1].m_Dir = oa.Rotate(orientation * half_theta);//������ת����
	(*rays)[1].m_fMin = new_m_ft;
	(*rays)[1].m_fMax = new_m_ft;
	(*rays)[1].m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;//��������������ϵ������ʲ���
	(*rays)[1].m_theta = half_theta;
	(*rays)[1].m_costheta = cos_halftheta;//�������߰��
	(*rays)[1].m_bsplit = true;              //�������߾��з������ԣ��µĹ���Դ��
	(*rays)[1].m_vWedge = incident_ray.m_vWedge;
	(*rays)[1].m_vWedge.push_back(wedge);
	(*rays)[1].m_sensorDataId = incident_ray.m_sensorDataId;									//����������ID��ֵ

	for (unsigned i = 2; i < raynum - 1; i++) {//�м�Ƕȷ���
		(*rays)[i].m_Ori = wedge->m_point;
		(*rays)[i].m_Dir = oa.Rotate(orientation * delta_theta);//ÿ����תһ���Ƕȣ���ɢ��
		(*rays)[i].m_fMin = new_m_ft;
		(*rays)[i].m_fMax = new_m_ft;
		(*rays)[i].m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;
		(*rays)[i].m_theta = half_theta;
		(*rays)[i].m_costheta = cos_halftheta;
		(*rays)[i].m_bsplit = true;
		(*rays)[i].m_vWedge = incident_ray.m_vWedge;
		(*rays)[i].m_vWedge.push_back(wedge);
		(*rays)[i].m_sensorDataId = incident_ray.m_sensorDataId;									//����������ID��ֵ
	}

	(*rays)[raynum - 1].m_Ori = wedge->m_point;//���һ������,ǽ�巽��2
	(*rays)[raynum - 1].m_Dir = wedge->m_dir2;
	(*rays)[raynum - 1].m_fMin = new_m_ft;
	(*rays)[raynum - 1].m_fMax = new_m_ft;
	(*rays)[raynum - 1].m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;
	(*rays)[raynum - 1].m_theta = 0.0;
	(*rays)[raynum - 1].m_costheta = 1.0;
	(*rays)[raynum - 1].m_bsplit = false;//ǽ���Ե���߲�����
	(*rays)[raynum - 1].m_vWedge = incident_ray.m_vWedge;
	(*rays)[raynum - 1].m_vWedge.push_back(wedge);
	(*rays)[raynum - 1].m_sensorDataId = incident_ray.m_sensorDataId;								//����������ID��ֵ
	return true;
}
