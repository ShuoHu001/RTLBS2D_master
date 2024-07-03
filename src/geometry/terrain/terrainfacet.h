#ifndef RTLBS3D_TERRAINFACET
#define RTLBS3D_TERRAINFACET

#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/point3d.h"
#include "geometry/vector3d.h"
#include "geometry/bbox2d.h"
#include "geometry/bbox3d.h"
#include "geometry/ray2d.h"
#include "geometry/ray3dlite.h"
#include "material/material.h"

class TerrainSegment;
class Intersection3D;

class TerrainFacet {
public:
	int m_facetId;							/** @brief	��Ԫ����	*/
	bool m_isEdgeFacet;						/** @brief	�Ƿ��Ǳ߽���Ԫ	*/
	Point3D* m_p1;							/** @brief	�����ε�1	*/
	Point3D* m_p2;							/** @brief	�����ε�2	*/
	Point3D* m_p3;							/** @brief	�����ε�3	*/
	Vector3D m_normal;						/** @brief	��Ԫ���߷���	*/
	TerrainSegment* m_segment1;				//�����ζ�Ӧ���߶�1
	TerrainSegment* m_segment2;				//�����ζ�Ӧ���߶�2
	TerrainSegment* m_segment3;				//�����ζ�Ӧ���߶�3
	Material* m_mat;						/** @brief	��Ԫ����	*/
	BBox3D m_bbox;							/** @brief	��Χ��	*/

public:
	TerrainFacet();
	TerrainFacet(int facetId, Point3D* p1, Point3D* p2, Point3D* p3, Material* mat = nullptr);
	TerrainFacet(int facetId, Point3D* p1, Point3D* p2, Point3D* p3, TerrainSegment* segment1, TerrainSegment* segment2, TerrainSegment* segment3, Material* mat = nullptr);
	TerrainFacet(const TerrainFacet& tf);												//ǳ�㿽�����캯��
	~TerrainFacet();
	TerrainFacet operator = (const TerrainFacet& tf);
	bool operator == (const TerrainFacet& other) const;
	bool operator != (const TerrainFacet& other) const;
	void AssignEdge(TerrainSegment* segment); //���������εı�
	bool GetIntersect(Ray3DLite* ray, Point3D* point = nullptr) const; //��������������ཻ
	bool CheckInside(const Point2D& p) const; //�ж�һ����ά������Ƿ��ڵ��������ε�ͶӰ��
	bool CheckInside(const Point3D& p) const; //�ж�һ����ά������Ƿ��ڵ��������ε�ͶӰ��
	RtLbsType GetMinDistanceToPoint(const Point3D& p) const; //���㵽��Ԫ����ƽ�����С����
	RtLbsType GetVerticleDistanceToPoint(const Point3D& p) const; //���㵽��Ԫ����ƽ��Ĵ��߾���
	RtLbsType GetFacetHeightViaPoint(const Point2D& p) const;			//����ά������ά��Ԫ�ϵ�ӳ���
	Point3D GetPointOnPlane(const Point3D& p) const; //�����Ԫ�Ͽյĵ����Ŵ�������Ԫ�ϵĵ�
	Point3D GetMirrorPoint(const Point3D& p) const;			//����p���ڵ�����Ԫ�ľ����
	TerrainSegment* GetIntersectSegment(Ray2D* ray, TerrainSegment* prevSegment, RtLbsType* maxt, RtLbsType* height); //��ȡ�������������ཻ����һ����,�����ض�ά�Ͼ����������ĳ��Ⱥ��߶��϶�Ӧ�ĸ߶�
	bool HasEdgeSegmentIntersect(Ray2D* ray2d, RtLbsType& t) const;																//�ж���Ԫ���Ƿ���ڱ߽��߶�,���ؾ���ֵ�����ֵ
	void Update();															//������������Ԫ�ĸ���
};
#endif
