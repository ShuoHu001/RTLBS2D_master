/** 
* @filename:	ray2d.h 
* @time:		2023/03/18 14:22:49 
* @author:		Hu Shuo
* @Email:		15991797156@163.com
* @location:	China, Shaanxi
* @brief:		Based 2D ray in RTLBS
*/
#ifndef RTLBS_RAY2D
#define RTLBS_RAY2D

#include "vector2d.h"
#include "point2d.h"
#include "wedge2d.h"
#include "utility/define.h"
#include "geometry/gpu/ray2dgpu.h"



class Ray2D {
public:
	Point2D m_Ori;												/** @brief	������ʵ����	*/
	Vector2D m_Dir;												/** @brief	���ߵķ���	*/
	RtLbsType m_fRefractiveIndex;								/** @brief	��ǰ������������ռ��������	*/
	RtLbsType m_fMax;											/** @brief	���ߵĴ������� ��С����	*/
	RtLbsType m_fMin;											/** @brief	���ߵĴ������� ������	*/
	double m_theta;												/** @brief	���߷���ʱ�İ��Ž�	*/
	double m_costheta;											/** @brief	���߷���ʱ�İ��Žǵ�����ֵ	*/
	bool m_bsplit;												/** @brief	�����Ƿ���ѵı�־	*/
	int m_sensorDataId;											/** @brief	������Я���Ĵ���������ID��Ϣ	*/						
	std::vector<Wedge2D*> m_vWedge;								/** @brief	��ǰ���߾��������������飬��ֹ�ظ�����	*/

public:
	explicit Ray2D();
	Ray2D(const Point2D& ori, const Vector2D& dir);
	Ray2D(const Point2D& ori, const Point2D& tar); //����������Ŀ��㹹�����߷���
	Ray2D(const Point2D& ori, const Vector2D& dir, double theta, double costheta);
	Ray2D(const Ray2DGPU& rayGPU);
	Ray2D(const Ray2D& r);
	~Ray2D() {};
	Ray2D operator = (const Ray2D& ray);
	Ray2D operator = (const Ray2DGPU& rayGPU);
	Point2D operator () (RtLbsType t) const;
	Point2D GetRayCoordinate(RtLbsType t); //��ȡ�����߷�����ָ�����ȵ�����
	RtLbsType GetRayRadius(RtLbsType t) const;//��ȡ���ߴ�Դ�㴫����ָ�����ȵķ���뾶,�������tΪ����tmax�ľ���
	RtLbsType GetSquaredDistanceToPoint(const Point2D& p);
	Ray2DGPU Convert2GPU() const;
};

//ȫ�ֺ���

//���������t��������
inline Point2D GetRayCoordinate(const Ray2D& ray, RtLbsType t) {
	return ray.m_Ori + ray.m_Dir * t;
}

//���p��������ߵĴ���ƽ������
inline RtLbsType GetSquaredDistanceToPoint(const Ray2D& ray, Point2D p) {
	Vector2D op = p - ray.m_Ori;
	Vector2D oq = ray.m_Dir * op * ray.m_Dir;
	Vector2D qp = op - oq;
	return qp.x * qp.x + qp.y * qp.y;
}

#endif
