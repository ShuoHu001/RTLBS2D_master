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
	Point2D m_Ori;												/** @brief	射线其实坐标	*/
	Vector2D m_Dir;												/** @brief	射线的方向	*/
	RtLbsType m_fRefractiveIndex;								/** @brief	当前射线所在物体空间的折射率	*/
	RtLbsType m_fMax;											/** @brief	射线的传播距离 最小距离	*/
	RtLbsType m_fMin;											/** @brief	射线的传播距离 最大距离	*/
	double m_theta;												/** @brief	射线发射时的半张角	*/
	double m_costheta;											/** @brief	射线发射时的半张角的余弦值	*/
	bool m_bsplit;												/** @brief	射线是否分裂的标志	*/
	int m_sensorDataId;											/** @brief	射线所携带的传感器数据ID信息	*/						
	std::vector<Wedge2D*> m_vWedge;								/** @brief	当前射线经过的绕射棱数组，防止重复绕射	*/

public:
	explicit Ray2D();
	Ray2D(const Point2D& ori, const Vector2D& dir);
	Ray2D(const Point2D& ori, const Point2D& tar); //根据输入点和目标点构建射线方向
	Ray2D(const Point2D& ori, const Vector2D& dir, double theta, double costheta);
	Ray2D(const Ray2DGPU& rayGPU);
	Ray2D(const Ray2D& r);
	~Ray2D() {};
	Ray2D operator = (const Ray2D& ray);
	Ray2D operator = (const Ray2DGPU& rayGPU);
	Point2D operator () (RtLbsType t) const;
	Point2D GetRayCoordinate(RtLbsType t); //获取沿射线方向上指定长度的坐标
	RtLbsType GetRayRadius(RtLbsType t) const;//获取射线从源点传播到指定长度的辐射半径,输入参数t为距离tmax的距离
	RtLbsType GetSquaredDistanceToPoint(const Point2D& p);
	Ray2DGPU Convert2GPU() const;
};

//全局函数

//获得射线沿t处的坐标
inline Point2D GetRayCoordinate(const Ray2D& ray, RtLbsType t) {
	return ray.m_Ori + ray.m_Dir * t;
}

//获得p点距离射线的垂线平方距离
inline RtLbsType GetSquaredDistanceToPoint(const Ray2D& ray, Point2D p) {
	Vector2D op = p - ray.m_Ori;
	Vector2D oq = ray.m_Dir * op * ray.m_Dir;
	Vector2D qp = op - oq;
	return qp.x * qp.x + qp.y * qp.y;
}

#endif
