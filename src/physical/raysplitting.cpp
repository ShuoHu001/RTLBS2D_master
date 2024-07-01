#include "raysplitting.h"




bool IsGenerateSplittingRays(const Ray2D& rayInit, RtLbsType t, bool splitFlag, RtLbsType splitRadius, int& splitNum)
{
	if (rayInit.m_bsplit == false || !splitFlag) {				//�����߲��߱��������ԣ������з���
		return false;
	}
	RtLbsType r = rayInit.GetRayRadius(t);			/** @brief	����������t���İ뾶ֵ	*/
	if (r <= splitRadius) {							//��������t���İ뾶ֵС�ڷ��Ѱ뾶�����϶��������߲�����
		return false;
	}
	//��������߹����Ž�
	double total_theta = 2 * rayInit.m_theta;
	//��������ѵ���������
	splitNum = static_cast<int>(ceil(2 * r / splitRadius)) + 2;
	if (splitNum % 2 != 1) {//�뱣֤������ĿΪ���������ԶԳƻ���
		splitNum += 1;
	}
	return true;
}

void GenerateSplittingRay(const Ray2D& ray, int splitNum, std::vector<Ray2D>* rays) {//�ϰ���������������߷���,������͸���ֹ����
	//��������߹����Ž�
	double total_theta = 2 * ray.m_theta;
	(*rays).resize(splitNum);
	//��������߰��Ž�
	double dtheta = total_theta / (splitNum - 1);
	double ray_halftheta = dtheta / 2.0;
	double coshalftheta = cos(ray_halftheta);
	double ray_quardtheta = dtheta / 4.0;
	double cosquardtheta = cos(ray_quardtheta);
	//�����������ɢ������
	Ray2D newRay1(ray);
	Vector2D newDir = ray.m_Dir;
	newRay1.m_theta = ray_halftheta; //�����µ�ray����
	newRay1.m_costheta = coshalftheta;
	int k1 = 0;
	//----------------------------------------����������(����Դ�ڵ�)---------------------------------------------

	//����ת
	(*rays)[k1++] = newRay1;

	//���� 1 dtheta
	newDir = ray.m_Dir;
	for (int i = 1; i < (splitNum - 1) / 2; ++i) {
		newDir.Rotate(dtheta);
		newRay1.m_Dir = newDir;
		newRay1.m_theta = ray_halftheta;
		newRay1.m_costheta = coshalftheta;
		(*rays)[k1++] = newRay1;
	}
	//���� 3/4 dtheta
	newDir.Rotate(ray_quardtheta + ray_halftheta);
	newRay1.m_Dir = newDir;
	newRay1.m_theta = ray_quardtheta;
	newRay1.m_costheta = cosquardtheta;
	(*rays)[k1++] = newRay1;


	//���� 1 dtheta
	newDir = ray.m_Dir;
	for (int i = 1; i < (splitNum - 1) / 2; ++i) {
		newDir.Rotate(-dtheta);
		newRay1.m_Dir = newDir;
		newRay1.m_theta = ray_halftheta;
		newRay1.m_costheta = coshalftheta;
		(*rays)[k1++] = newRay1;
	}
	//���� 3/4 dtheta
	newDir.Rotate(-ray_quardtheta - ray_halftheta);
	newRay1.m_Dir = newDir;
	newRay1.m_theta = ray_quardtheta;
	newRay1.m_costheta = cosquardtheta;
	(*rays)[k1++] = newRay1;
}
