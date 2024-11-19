#ifndef RTLBS_TERRAINSEGMENT
#define RTLBS_TERRAINSEGMENT

#include "rtlbs.h"
#include "utility/define.h"
#include "math/point2d.h"
#include "geometry/ray2d.h"
#include "geometry/bbox2d.h"
#include "geometry/ray3dlite.h"

class TerrainFacet;

class TerrainSegment {
public:
	int m_segmentId;						/** @brief	�����߶�ID	*/
	bool m_isShared;						/** @brief	�Ƿ��ǹ����	*/
	Point3D* m_ps;							/** @brief	ָ���߶���ʼ�������ָ�루���ͷ��ڴ棩	*/
	Point3D* m_pe;							/** @brief	ָ���߶���ֹ�������ָ�루���ͷ��ڴ棩	*/
	Vector3D m_dir;							/** @brief	�߶εķ�������	*/
	TerrainFacet* m_facet1;					/** @brief	�߶ζ�Ӧ����Ԫ1	*/
	TerrainFacet* m_facet2;					/** @brief	�߶ζ�Ӧ����Ԫ2	*/
	BBox2D m_bbox;							/** @brief	�������ڵİ�Χ��	*/
public:
	TerrainSegment();
	TerrainSegment(int id, Point3D* ps, Point3D* pe, TerrainFacet* facet1);
	TerrainSegment(int id, Point3D* ps, Point3D* pe);
	TerrainSegment(int id, Point3D* ps, Point3D* pe, TerrainFacet* facet1, TerrainFacet* facet2);
	TerrainSegment(const TerrainSegment& ts);																//ǳ�㿽�����캯������ַԭֵ���ƣ�
	~TerrainSegment();
	TerrainSegment& operator = (const TerrainSegment& ts);
	bool operator == (const TerrainSegment& ts) const;
	bool operator != (const TerrainSegment& ts) const;
	Point2D GetStartPoint2D() const;
	Point2D GetEndPoint2D() const;
	TerrainFacet* GetAdjacentFacet(TerrainFacet* facet); //����õ����ڵĵ���������Ԫ
	bool Intersect(Ray2D* ray, RtLbsType* st, RtLbsType* height) const; //�����Ƿ����߶��ཻ, st:�������߶εĽ����������Դ��ľ��룬su:�������߶ν�������߶����ľ���ٷֱ�
	bool HasIntersect(Ray2D* ray, RtLbsType& out_t) const;										//�ж��������߶��Ƿ��ཻ
	RtLbsType GetLengthXY() const;
	RtLbsType GetLength() const;
	Vector2D GetDirXY() const;
	bool ValidIntersect(const Ray3DLite& ray); //��֤�߶������Ƿ��ཻ
	std::string ToString() const;
	size_t GetHash() const; //��ȡ��ߵĹ�ϣֵ
	void Update();									//���µ����߶�

};


#endif
