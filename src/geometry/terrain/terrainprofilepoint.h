#ifndef RTLBS_TERRAINPROFILEPOINT
#define RTLBS_TERRAINPROFILEPOINT

#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/point3d.h"
#include "geometry/point2d.h"
#include "material/material.h"

//������������
class TerrainProfilePoint {
public:
	int pId;									/** @brief	��ά����������	*/
	Material* m_mat;							/** @brief	����	*/
	const Point3D* m_point3d;					/** @brief	��ά������	*/
	Point2D m_point2d;							/** @brief	��ά�������� x:���� y:�߶�*/

public:
	TerrainProfilePoint(int id, const Point3D* point, const Point3D* startPoint, Material* material = nullptr);
	TerrainProfilePoint(const TerrainProfilePoint& tpp);
	~TerrainProfilePoint();
	TerrainProfilePoint& operator = (const TerrainProfilePoint& tpp);
	bool operator == (const TerrainProfilePoint& other) const;
	bool operator != (const TerrainProfilePoint& other) const;
	void WriteToFile(std::ofstream& stream) const;

};

#endif
