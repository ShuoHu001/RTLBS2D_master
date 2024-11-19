#ifndef RTLBS_TERRAINPROFILESEGMENT
#define RTLBS_TERRAINPROFILESEGMENT

#include "rtlbs.h"
#include "utility/define.h"
#include "math/vector2d.h"
#include "terrainprofilepoint.h"

//地形剖面线段对象
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
	RtLbsType GetHeight(TerrainProfilePoint* p) const; //获得指定点处的高度
	void Init(TerrainProfilePoint* ps, TerrainProfilePoint* pe);
};

#endif
