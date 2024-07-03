#include "terrainprofilepoint.h"

TerrainProfilePoint::TerrainProfilePoint(int id, const Point3D* point, const Point3D* startPoint, Material* material)
	: pId(id)
	, m_point3d(point)
	, m_mat(material)
{
	Vector3D sp = *point - *startPoint;
	m_point2d.x = sp.LengthXY();
	m_point2d.y = point->z;

}

TerrainProfilePoint::TerrainProfilePoint(const TerrainProfilePoint& tpp)
	: pId(tpp.pId)
	, m_mat(tpp.m_mat)
	, m_point3d(tpp.m_point3d)
	, m_point2d(tpp.m_point2d)
{
}

TerrainProfilePoint::~TerrainProfilePoint()
{
}

TerrainProfilePoint& TerrainProfilePoint::operator=(const TerrainProfilePoint& tpp)
{
	pId = tpp.pId;
	m_mat = tpp.m_mat;
	m_point3d = tpp.m_point3d;
	m_point2d = tpp.m_point2d;
	return *this;
}

bool TerrainProfilePoint::operator==(const TerrainProfilePoint& other) const
{
	return pId == other.pId;
}

bool TerrainProfilePoint::operator!=(const TerrainProfilePoint& other) const
{
	return !(*this == other);
}

void TerrainProfilePoint::WriteToFile(std::ofstream& stream) const
{
	stream << m_point2d.x << "\t" << m_point3d->z << std::endl;
}
