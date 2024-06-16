#ifndef RTLBS_INTERSECTION2D
#define RTLBS_INTERSECTION2D
#include "utility/enum.h"
#include "wedge2d.h"
#include "ray2d.h"
#include "geometry/gpu/Intersection2Dgpu.h"
#include "physical/propagationproperty.h"

class Point2D;
class Segment2D;

class Intersection2D {

public:
	Point2D m_intersect;									/** @brief	交点坐标	*/
	PATHNODETYPE m_type;									/** @brief	节点类型	*/
	RtLbsType m_ft;											/** @brief	交点端距离原始坐标点的距离	*/
	Segment2D* m_segment;									/** @brief	反射/透射 判定的面元	*/
	std::vector<Wedge2D*> m_wedges;							/** @brief	绕射判定的楔形角,绕射角会有多个的情况	*/
	RtLbsType m_u;											/** @brief	线段的分割参数,即交点所处的位置	*/
	PropagationProperty m_propagationProperty;				/** @brief	传播属性-用于确定传播参数	*/

public:
	Intersection2D();
	Intersection2D(Point2D intersect, PATHNODETYPE type, RtLbsType t, Segment2D* segment); //反射节点配置
	Intersection2D(Point2D intersect, PATHNODETYPE type, RtLbsType t);						//视距节点配置
	~Intersection2D();
	bool Update(Ray2D& ray); //更新交点信息,初级相交判断只是几何结果，需要根据进一步物理信息更新相交信息

	Intersection2DGPU Convert2GPU();
	
private:
	bool ValidWedges(Ray2D& ray);

};

#endif
