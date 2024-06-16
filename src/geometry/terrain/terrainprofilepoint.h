#ifndef RTLBS_TERRAINPROFILEPOINT
#define RTLBS_TERRAINPROFILEPOINT

#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/point3d.h"
#include "geometry/point2d.h"

//地形坐标点对象
class TerrainProfilePoint {
public:
	int pId;									/** @brief	三维点坐标索引	*/
	int m_matId;								/** @brief	材质ID	*/
	const Point3D* m_point3d;					/** @brief	三维点坐标	*/
	Point2D m_point2d;							/** @brief	二维剖面坐标 x:距离 y:高度*/

public:
	TerrainProfilePoint(int id, const Point3D* point, const Point3D* startPoint, int matId = -1);
	TerrainProfilePoint(const TerrainProfilePoint& tpp);
	~TerrainProfilePoint();
	TerrainProfilePoint& operator = (const TerrainProfilePoint& tpp);
	bool operator == (const TerrainProfilePoint& other) const;
	bool operator != (const TerrainProfilePoint& other) const;
	void WriteToFile(std::ofstream& stream) const;

};

#endif
