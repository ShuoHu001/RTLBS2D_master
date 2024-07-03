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
	int m_facetId;							/** @brief	面元索引	*/
	bool m_isEdgeFacet;						/** @brief	是否是边界面元	*/
	Point3D* m_p1;							/** @brief	三角形点1	*/
	Point3D* m_p2;							/** @brief	三角形点2	*/
	Point3D* m_p3;							/** @brief	三角形点3	*/
	Vector3D m_normal;						/** @brief	面元法线方向	*/
	TerrainSegment* m_segment1;				//三角形对应的线段1
	TerrainSegment* m_segment2;				//三角形对应的线段2
	TerrainSegment* m_segment3;				//三角形对应的线段3
	Material* m_mat;						/** @brief	面元材料	*/
	BBox3D m_bbox;							/** @brief	包围盒	*/

public:
	TerrainFacet();
	TerrainFacet(int facetId, Point3D* p1, Point3D* p2, Point3D* p3, Material* mat = nullptr);
	TerrainFacet(int facetId, Point3D* p1, Point3D* p2, Point3D* p3, TerrainSegment* segment1, TerrainSegment* segment2, TerrainSegment* segment3, Material* mat = nullptr);
	TerrainFacet(const TerrainFacet& tf);												//浅层拷贝构造函数
	~TerrainFacet();
	TerrainFacet operator = (const TerrainFacet& tf);
	bool operator == (const TerrainFacet& other) const;
	bool operator != (const TerrainFacet& other) const;
	void AssignEdge(TerrainSegment* segment); //分配三角形的边
	bool GetIntersect(Ray3DLite* ray, Point3D* point = nullptr) const; //计算射线与地形相交
	bool CheckInside(const Point2D& p) const; //判断一个二维坐标点是否在地形三角形的投影中
	bool CheckInside(const Point3D& p) const; //判断一个三维坐标点是否在地形三角形的投影中
	RtLbsType GetMinDistanceToPoint(const Point3D& p) const; //求解点到面元所在平面的最小距离
	RtLbsType GetVerticleDistanceToPoint(const Point3D& p) const; //求解点到面元所在平面的垂线距离
	RtLbsType GetFacetHeightViaPoint(const Point2D& p) const;			//求解二维点在三维面元上的映射点
	Point3D GetPointOnPlane(const Point3D& p) const; //求解面元上空的点沿着垂线在面元上的点
	Point3D GetMirrorPoint(const Point3D& p) const;			//求解点p关于地形面元的镜像点
	TerrainSegment* GetIntersectSegment(Ray2D* ray, TerrainSegment* prevSegment, RtLbsType* maxt, RtLbsType* height); //获取射线与三角形相交的另一个边,并返回二维上距离射线起点的长度和线段上对应的高度
	bool HasEdgeSegmentIntersect(Ray2D* ray2d, RtLbsType& t) const;																//判定面元内是否存在边界线段,返回距离值最近的值
	void Update();															//进行三角形面元的更新
};
#endif
