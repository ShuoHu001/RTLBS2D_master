#ifndef RTLBS_TERRAINPROFILESEGMENT
#define RTLBS_TERRAINPROFILESEGMENT

#include "rtlbs.h"
#include "utility/define.h"
#include "math/vector2d.h"
#include "terrainprofilepoint.h"

//���������߶ζ���
class TerrainProfileSegment {
public:
	TerrainProfilePoint* m_ps;
	TerrainProfilePoint* m_pe;
	Vector2D m_dir;

public:
	TerrainProfileSegment();
	TerrainProfileSegment(TerrainProfilePoint* ps, TerrainProfilePoint* pe);
	TerrainProfileSegment(const TerrainProfileSegment& segment);
	~TerrainProfileSegment();
	TerrainProfileSegment& operator = (const TerrainProfileSegment& segment);
	bool operator == (const TerrainProfileSegment& other) const;
	bool operator != (const TerrainProfileSegment& other) const;
	RtLbsType GetHeight(TerrainProfilePoint* p) const; //���ָ���㴦�ĸ߶�
	void Init(TerrainProfilePoint* ps, TerrainProfilePoint* pe);
};

#endif
