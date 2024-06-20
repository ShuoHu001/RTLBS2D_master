#ifndef RTLBS_SEGMENT2D
#define RTLBS_SEGMENT2D

#include "rtlbs.h"
#include "utility/define.h"
#include "primitive2d.h"
#include "geometry/gpu/segment2dgpu.h"
#include "utility/serializable.h"

class Intersection2D;//Ԥ�ȶ��壬��ֹ�໥����
class Wedge2D;

class Segment2D:public Primitive2D, Serializable {
public:
	Point2D m_ps;						/** @brief	��ʼ�ڵ�	*/
	Point2D m_pe;						/** @brief	��ֹ�ڵ�	*/
	Vector2D m_normal;					/** @brief	�ⷨ����	*/
	Vector2D m_dir;						/** @brief	��������	*/
	RtLbsType m_length;					/** @brief	�߶γ���	*/
	Wedge2D* m_ws;						/** @brief	��ʼ�ڵ��Ӧ��Ш�ν���Ϣ	*/
	Wedge2D* m_we;						/** @brief	��ֹ�ڵ��Ӧ��Ш�ν���Ϣ	*/


public:
	explicit Segment2D();
	Segment2D(const Point2D& ps, const Point2D& pe, OBJECT2DCATEGORY category = BUILDING2D);
	Segment2D(Point2D ps, Point2D pe, int globalId, RtLbsType refractN);  //������Ԫ����
	Segment2D(Point2D ps, Point2D pe, int globalId, RtLbsType refractN, RtLbsType refractNOut);  //������Ԫ����
	Segment2D(Point2D ps, Point2D pe, int globalId, RtLbsType refractN, Wedge2D* ws, Wedge2D* we);  //��������Ш�νǵ���Ԫ����
	Segment2D(Point2D ps, Point2D pe, int globalId, RtLbsType refractN, RtLbsType refractNOut, Wedge2D* ws, Wedge2D* we);  //��������Ш�νǵ���Ԫ����
	Segment2D(const Segment2D* segment);
	~Segment2D();
	bool operator == (const Segment2D& other) const;
	bool operator != (const Segment2D& other) const;
	Segment2D& operator = (const Segment2D& segment);
	RtLbsType DistanceToPoint(const Point2D& p) const;											//�ռ��е㵽�߶εľ���
	bool IsInSegment(const Point2D& p) const;													//���Ƿ����߶���
	RtLbsType GetCenter(int axis) const;
	bool GetIntersect(const Ray2D& ray, Intersection2D* intersect) const;
	bool GetIntersectNoBBox(const Ray2D& ray, Intersection2D* intersect) const; /** @brief	�����ཻ����Ҫ���Χ���ཻ	*/
	const BBox2D& GetBBox() const;

	Segment2DGPU Convert2GPU();
	 

public:
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);

};

//���ڸ�������Ԫ�������߽��
inline BBox2D computeBBox(const std::vector<Segment2D*>& segments) {
	BBox2D bbox;
	for (auto it = segments.begin(); it != segments.end(); ++it) {
		const Segment2D* segment = *it;
		bbox.Union(segment->m_bbox);
	}
	return bbox;
}

//������Ԫ��������߽��
inline BBox2D ComputeBBoxByIndex(std::vector<Segment2D*>::iterator start, std::vector<Segment2D*>::iterator end) {
	BBox2D bbox;
	for (auto it = start; it != end; ++it) {
		const Segment2D* segment = *it;
		bbox.Union(segment->m_bbox);
	}
	return bbox;
}




#endif
