#ifndef RTLBS_SEGMENT2DGPU
#define RTLBS_SEGMENT2DGPU
#include "utility/define.h"
#include "bbox2dgpu.h"
#include "physical/propagationproperty.h"

class Intersection2DGPU;//Ԥ�ȶ��壬��ֹ�໥����
class Ray2DGPU;

class Segment2DGPU {
public:
	Point2D m_ps;											/** @brief	��ʼ�ڵ�	*/
	Point2D m_pe;											/** @brief	��ֹ�ڵ�	*/
	Vector2D m_normal;										/** @brief	�ⷨ����	*/
	Vector2D m_dir;											/** @brief	��������	*/
	RtLbsType m_length;										/** @brief	�߶γ���	*/
	BBox2DGPU m_bbox;										/** @brief	��Χ��	*/
	int64_t m_primitive_id;
	RtLbsType m_refractN;
	RtLbsType m_refractNOut;
	PropagationProperty m_propagationProperty;				/** @brief	��������	*/


public:
	HOST_DEVICE_FUNC Segment2DGPU();
	HOST_DEVICE_FUNC Segment2DGPU(Point2D ps, Point2D pe, Vector2D normal, Vector2D dir, RtLbsType length, BBox2DGPU bbox, int primitive_id, RtLbsType refractN, RtLbsType refractNOut);  //��Ԫ����
	HOST_DEVICE_FUNC Segment2DGPU(const Segment2DGPU& segment);
	HOST_DEVICE_FUNC ~Segment2DGPU();
	HOST_DEVICE_FUNC bool operator == (const Segment2DGPU& other) const;
	HOST_DEVICE_FUNC bool operator != (const Segment2DGPU& other) const;
	HOST_DEVICE_FUNC Segment2DGPU& operator = (const Segment2DGPU& segment);
	HOST_DEVICE_FUNC bool GetIntersect(const Ray2DGPU& ray, Intersection2DGPU* intersect) const;
	HOST_DEVICE_FUNC const BBox2DGPU& GetBBox() const;
	 

};




#endif
