#include "raysplittinggpu.h"


HOST_DEVICE_FUNC void GenerateSplittingRayGPU(Ray2DGPU& initRay, int splitNum, Ray2DGPU* newRays, Segment2DGPU* segments) {//�ϰ���������������߷���,������͸���ֹ����
	//
	
	//��������߹����Ž�
	double total_theta = 2 * initRay.m_theta;
	
	//��������߰��Ž�
	double dtheta = total_theta / splitNum;

	double halftheta = dtheta / 2.0;
	double coshalftheta = cos(halftheta);
	//�����������ɢ������
	Ray2DGPU rayTemp(initRay);
	Vector2D newDir = initRay.m_Dir;
	rayTemp.m_theta = halftheta; //�����µ�ray����
	rayTemp.m_costheta = coshalftheta;
	int k = 0;
	//�������߷�Ϊ�����֣���һΪ����Դ������ڶ�Ϊ�ǹ���Դ���
	//----------------------------------------����������(����Դ�ڵ�)---------------------------------------------
	//���ﲻ�ٲ��ò����ȷ��ѷ��������þ��ȷ��ѷ���

	rayTemp.m_theta = halftheta; //ȷ���������ߵİ��Ž�
	rayTemp.m_costheta = coshalftheta;


	//����ת��һ��
	newDir = initRay.m_Dir;
	newDir.Rotate(-initRay.m_theta);
	if (initRay.m_nodeType == NODE_ROOT || initRay.m_nodeType == NODE_DIFF) {//��ǰ�ڵ�����Ϊ���ڵ������ڵ�Ĳ���ִ���ཻ�ж��������������ߺ���������

		//��ʼ����
		newDir.Rotate(halftheta); 
		rayTemp.m_Dir = newDir;
		newRays[k] = rayTemp;
		newRays[k].m_isValid = true;
		k++;

		//��������
		for (int i = 1; i < splitNum; ++i) {
			newDir.Rotate(dtheta);
			rayTemp.m_Dir = newDir;
			newRays[k] = rayTemp;
			newRays[k].m_isValid = true;
			k++;
		}
		return;
	}
	//�ǹ���Դ���



	RtLbsType t = initRay.m_tMax - initRay.m_tMin;
	Point2D vSource = initRay(-t);//�������Դ��λ��
	Intersection2DGPU intersect;
	Ray2DGPU tempRay2(initRay);
	tempRay2.m_theta = halftheta;
	tempRay2.m_costheta = coshalftheta;
	newDir = initRay.m_Dir;


	//��ʼ����
	newDir.Rotate(halftheta);
	rayTemp.m_Dir = newDir;
	if (segments[initRay.m_primitiveId].GetIntersect(rayTemp, &intersect)) {
		newRays[k] = rayTemp;
		newRays[k].m_isValid = true;
		k++;
	}

	//��������
	for (int i = 1; i < splitNum; ++i) {
		newDir.Rotate(dtheta);
		rayTemp.m_Dir = newDir;
		if (segments[initRay.m_primitiveId].GetIntersect(rayTemp, &intersect)) {
			newRays[k] = rayTemp;
			newRays[k].m_isValid = true;
			k++;
		}
	}
}


