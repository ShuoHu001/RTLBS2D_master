#ifndef RTLBS_SEGMENT2DGPU
#define RTLBS_SEGMENT2DGPU
#include "utility/define.h"
#include "bbox2dgpu.h"
#include "physical/propagationproperty.h"

class Intersection2DGPU;//预先定义，防止相互包含
class Ray2DGPU;

class Segment2DGPU {
public:
	Point2D m_ps;											/** @brief	开始节点	*/
	Point2D m_pe;											/** @brief	终止节点	*/
	Vector2D m_normal;										/** @brief	外法向量	*/
	Vector2D m_dir;											/** @brief	方向向量	*/
	RtLbsType m_length;										/** @brief	线段长度	*/
	BBox2DGPU m_bbox;										/** @brief	包围盒	*/
	int64_t m_primitive_id;
	RtLbsType m_refractN;
	RtLbsType m_refractNOut;
	PropagationProperty m_propagationProperty;				/** @brief	传播属性	*/


public:
	HOST_DEVICE_FUNC Segment2DGPU();
	HOST_DEVICE_FUNC Segment2DGPU(Point2D ps, Point2D pe, Vector2D normal, Vector2D dir, RtLbsType length, BBox2DGPU bbox, int primitive_id, RtLbsType refractN, RtLbsType refractNOut);  //面元复制
	HOST_DEVICE_FUNC Segment2DGPU(const Segment2DGPU& segment);
	HOST_DEVICE_FUNC ~Segment2DGPU();
	HOST_DEVICE_FUNC bool operator == (const Segment2DGPU& other) const;
	HOST_DEVICE_FUNC bool operator != (const Segment2DGPU& other) const;
	HOST_DEVICE_FUNC Segment2DGPU& operator = (const Segment2DGPU& segment);
	HOST_DEVICE_FUNC bool GetIntersect(const Ray2DGPU& ray, Intersection2DGPU* intersect) const;
	HOST_DEVICE_FUNC const BBox2DGPU& GetBBox() const;
	 

};




#endif
