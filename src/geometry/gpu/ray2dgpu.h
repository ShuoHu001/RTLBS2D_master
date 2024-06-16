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
	bool m_isValid;									/** @brief	�����Ƿ���Ч	*/
	Point2D m_Ori;									/** @brief	������ʵ����	*/
	Vector2D m_Dir;									/** @brief	���ߵķ���	*/
	RtLbsType m_fMax;								/** @brief	���ߵĴ������� ��С����	*/
	RtLbsType m_fMin;								/** @brief	���ߵĴ������� ������	*/
	double m_costheta;								/** @brief	���Ž�����ֵ	*/
	double m_theta;									/** @brief	���Ž�	*/
	PATHNODETYPE m_nodeType;						/** @brief	����ԭ�㴦�����Ľڵ�����	*/
	int m_primitiveId;								/** @brief	����ԭ�㴦�����Ľڵ���Ԫ����	*/
	float m_fRefractiveIndex;						/** @brief	���������Ľ��ʻ���	*/
	bool m_bsplit;									/** @brief	�����Ƿ���з��ѱ�־	*/
	int m_limTotl;									/** @brief	��������	*/
	int m_limRefl;									/** @brief	����������	*/
	int m_limTran;									/** @brief	͸��������	*/
	int m_limDiff;									/** @brief	����������	*/
	int m_limScat;									/** @brief	ɢ��������	*/
	int m_prevInterId;								/** @brief	���������еĽ�����Ϣ	*/
	int m_sensorDataId;								/** @brief	�������ڵĴ���������ID	*/

public:
	HOST_DEVICE_FUNC  Ray2DGPU();
	HOST_DEVICE_FUNC  Ray2DGPU(const Ray2DGPU& other);
	HOST_DEVICE_FUNC  ~Ray2DGPU() {};
	HOST_DEVICE_FUNC Ray2DGPU& operator = (const Ray2DGPU& ray);
	HOST_DEVICE_FUNC Point2D operator () (RtLbsType t) const;
	HOST_DEVICE_FUNC RtLbsType GetRayRadis(RtLbsType t) const;
	HOST_DEVICE_FUNC bool IsCaptureByWedgePoint(Point2D p, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments) const;
	HOST_DEVICE_FUNC Point2D GetVisualSource() const;  //���㵱ǰ���ߵĹ���Դ����
	HOST_DEVICE_FUNC RtLbsType GetSquaredDistanceToPoint(const Point2D& p) const;
};

#endif
