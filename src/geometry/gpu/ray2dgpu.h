/** 
* @filename:	ray2d.h 
* @time:		2023/03/18 14:22:49 
* @author:		Hu Shuo
* @Email:		15991797156@163.com
* @location:	China, Shaanxi
* @brief:		Based 2D ray in RTLBS
*/
#ifndef RTLBS_RAY2DGPU
#define RTLBS_RAY2DGPU

#include "rtlbs.h"
#include "geometry/vector2d.h"
#include "geometry/point2d.h"
#include "utility/define.h"



class SignedDistanceFieldGPU;

class Intersection2DGPU;
class Segment2DGPU;

class Ray2DGPU {
public:
	bool m_isValid;									/** @brief	射线是否有效	*/
	Point2D m_Ori;									/** @brief	射线其实坐标	*/
	Vector2D m_Dir;									/** @brief	射线的方向	*/
	RtLbsType m_fMax;								/** @brief	射线的传播距离 最小距离	*/
	RtLbsType m_fMin;								/** @brief	射线的传播距离 最大距离	*/
	double m_costheta;								/** @brief	半张角余弦值	*/
	double m_theta;									/** @brief	半张角	*/
	PATHNODETYPE m_nodeType;						/** @brief	射线原点处所处的节点类型	*/
	int m_primitiveId;								/** @brief	射线原点处所处的节点面元类型	*/
	float m_fRefractiveIndex;						/** @brief	射线所处的介质环境	*/
	bool m_bsplit;									/** @brief	射线是否具有分裂标志	*/
	int m_limTotl;									/** @brief	总限制数	*/
	int m_limRefl;									/** @brief	反射限制数	*/
	int m_limTran;									/** @brief	透射限制数	*/
	int m_limDiff;									/** @brief	绕射限制数	*/
	int m_limScat;									/** @brief	散射限制数	*/
	int m_prevInterId;								/** @brief	射线所依托的交点信息	*/
	int m_sensorDataId;								/** @brief	射线所在的传感器数据ID	*/

public:
	HOST_DEVICE_FUNC  Ray2DGPU();
	HOST_DEVICE_FUNC  Ray2DGPU(const Ray2DGPU& other);
	HOST_DEVICE_FUNC  ~Ray2DGPU() {};
	HOST_DEVICE_FUNC Ray2DGPU& operator = (const Ray2DGPU& ray);
	HOST_DEVICE_FUNC Point2D operator () (RtLbsType t) const;
	HOST_DEVICE_FUNC RtLbsType GetRayRadis(RtLbsType t) const;
	HOST_DEVICE_FUNC bool IsCaptureByWedgePoint(Point2D p, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments) const;
	HOST_DEVICE_FUNC Point2D GetVisualSource() const;  //计算当前射线的广义源坐标
	HOST_DEVICE_FUNC RtLbsType GetSquaredDistanceToPoint(const Point2D& p) const;
};

#endif
