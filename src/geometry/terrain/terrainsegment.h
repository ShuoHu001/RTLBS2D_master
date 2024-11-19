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
	int m_segmentId;						/** @brief	地形线段ID	*/
	bool m_isShared;						/** @brief	是否是共享边	*/
	Point3D* m_ps;							/** @brief	指向线段起始点坐标的指针（不释放内存）	*/
	Point3D* m_pe;							/** @brief	指向线段终止点坐标的指针（不释放内存）	*/
	Vector3D m_dir;							/** @brief	线段的方向向量	*/
	TerrainFacet* m_facet1;					/** @brief	线段对应的面元1	*/
	TerrainFacet* m_facet2;					/** @brief	线段对应的面元2	*/
	BBox2D m_bbox;							/** @brief	射线所在的包围盒	*/
public:
	TerrainSegment();
	TerrainSegment(int id, Point3D* ps, Point3D* pe, TerrainFacet* facet1);
	TerrainSegment(int id, Point3D* ps, Point3D* pe);
	TerrainSegment(int id, Point3D* ps, Point3D* pe, TerrainFacet* facet1, TerrainFacet* facet2);
	TerrainSegment(const TerrainSegment& ts);																//浅层拷贝构造函数（地址原值复制）
	~TerrainSegment();
	TerrainSegment& operator = (const TerrainSegment& ts);
	bool operator == (const TerrainSegment& ts) const;
	bool operator != (const TerrainSegment& ts) const;
	Point2D GetStartPoint2D() const;
	Point2D GetEndPoint2D() const;
	TerrainFacet* GetAdjacentFacet(TerrainFacet* facet); //计算得到相邻的地形三角面元
	bool Intersect(Ray2D* ray, RtLbsType* st, RtLbsType* height) const; //计算是否与线段相交, st:射线与线段的交点距离射线源点的距离，su:射线与线段交点距离线段起点的距离百分比
	bool HasIntersect(Ray2D* ray, RtLbsType& out_t) const;										//判定射线与线段是否相交
	RtLbsType GetLengthXY() const;
	RtLbsType GetLength() const;
	Vector2D GetDirXY() const;
	bool ValidIntersect(const Ray3DLite& ray); //验证线段射线是否相交
	std::string ToString() const;
	size_t GetHash() const; //获取棱边的哈希值
	void Update();									//更新地形线段

};


#endif
