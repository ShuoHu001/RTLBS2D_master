#ifndef RTLBS_INTERSECTION2DGPU
#define RTLBS_INTERSECTION2DGPU
#include "utility/enum.h"
#include "segment2dgpu.h"
#include "configuration/radiowave/propagation/propagationproperty.h"

class Point2D;
class Ray2DGPU;

class Intersection2DGPU {

public:
	bool m_isValid;											/** @brief	相交状态	*/
	Point2D m_intersect;									/** @brief	交点坐标	*/
	PATHNODETYPE m_type;									/** @brief	节点类型	*/
	RtLbsType m_ft;											/** @brief	交点端距离原始坐标点的距离	*/
	int m_matId;											/** @brief	节点材质ID	*/
	int m_segmentId;										/** @brief	反射/透射 判定的面元Id	*/
	int m_wedgeId;											/** @brief	绕射判定的楞劈Id	*/
	Ray2DGPU m_ray;											/** @brief	交点信息的射线	*/
	int m_prevId;											/** @brief	前一个节点的Id	*/
	PropagationProperty m_propagationProperty;				/** @brief	交点所在面元的传播属性	*/

public:
	HOST_DEVICE_FUNC Intersection2DGPU();
	HOST_DEVICE_FUNC Intersection2DGPU(const Intersection2DGPU& intersect);				//赋值构造函数
	HOST_DEVICE_FUNC ~Intersection2DGPU();
	HOST_DEVICE_FUNC Intersection2DGPU& operator = (const Intersection2DGPU& intersect);
	HOST_DEVICE_FUNC bool IsCaptureRx(Point2D rx);
	HOST_DEVICE_FUNC Point2D GetVisualSource();  //计算广义源的位置
	

};

#endif
