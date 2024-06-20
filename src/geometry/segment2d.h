#ifndef RTLBS_SEGMENT2D
#define RTLBS_SEGMENT2D

#include "rtlbs.h"
#include "utility/define.h"
#include "primitive2d.h"
#include "geometry/gpu/segment2dgpu.h"
#include "utility/serializable.h"

class Intersection2D;//预先定义，防止相互包含
class Wedge2D;

class Segment2D:public Primitive2D, Serializable {
public:
	Point2D m_ps;						/** @brief	开始节点	*/
	Point2D m_pe;						/** @brief	终止节点	*/
	Vector2D m_normal;					/** @brief	外法向量	*/
	Vector2D m_dir;						/** @brief	方向向量	*/
	RtLbsType m_length;					/** @brief	线段长度	*/
	Wedge2D* m_ws;						/** @brief	开始节点对应的楔形角信息	*/
	Wedge2D* m_we;						/** @brief	终止节点对应的楔形角信息	*/


public:
	explicit Segment2D();
	Segment2D(const Point2D& ps, const Point2D& pe, OBJECT2DCATEGORY category = BUILDING2D);
	Segment2D(Point2D ps, Point2D pe, int globalId, RtLbsType refractN);  //基本面元配置
	Segment2D(Point2D ps, Point2D pe, int globalId, RtLbsType refractN, RtLbsType refractNOut);  //基本面元配置
	Segment2D(Point2D ps, Point2D pe, int globalId, RtLbsType refractN, Wedge2D* ws, Wedge2D* we);  //具有绕射楔形角的面元配置
	Segment2D(Point2D ps, Point2D pe, int globalId, RtLbsType refractN, RtLbsType refractNOut, Wedge2D* ws, Wedge2D* we);  //具有绕射楔形角的面元配置
	Segment2D(const Segment2D* segment);
	~Segment2D();
	bool operator == (const Segment2D& other) const;
	bool operator != (const Segment2D& other) const;
	Segment2D& operator = (const Segment2D& segment);
	RtLbsType DistanceToPoint(const Point2D& p) const;											//空间中点到线段的距离
	bool IsInSegment(const Point2D& p) const;													//点是否在线段上
	RtLbsType GetCenter(int axis) const;
	bool GetIntersect(const Ray2D& ray, Intersection2D* intersect) const;
	bool GetIntersectNoBBox(const Ray2D& ray, Intersection2D* intersect) const; /** @brief	计算相交不需要与包围盒相交	*/
	const BBox2D& GetBBox() const;

	Segment2DGPU Convert2GPU();
	 

public:
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);

};

//基于给定的面元数组计算边界盒
inline BBox2D computeBBox(const std::vector<Segment2D*>& segments) {
	BBox2D bbox;
	for (auto it = segments.begin(); it != segments.end(); ++it) {
		const Segment2D* segment = *it;
		bbox.Union(segment->m_bbox);
	}
	return bbox;
}

//基于面元索引计算边界盒
inline BBox2D ComputeBBoxByIndex(std::vector<Segment2D*>::iterator start, std::vector<Segment2D*>::iterator end) {
	BBox2D bbox;
	for (auto it = start; it != end; ++it) {
		const Segment2D* segment = *it;
		bbox.Union(segment->m_bbox);
	}
	return bbox;
}




#endif
