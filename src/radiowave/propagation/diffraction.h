#ifndef RTLBS_DIFFRACTION
#define RTLBS_DIFFRACTION

#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/ray2d.h"
#include "math/vector2d.h"
#include "geometry/segment2d.h"
#include "managers/randomanager.h"
#include "geometry/terrain/terrain.h"
#include "radiowave/raypath/terraindiffractionpath.h"


//产生常规绕射射线
bool GenerateDiffractRays(const Ray2D& incident_ray, int diffractRayNum, Wedge2D* wedge, std::vector<Ray2D>* rays);


//产生地形剖面线上的绕射路径
void GetDiffractPathOverRidges(const TerrainProfile* profile, TerrainDiffractionPath*& outPath);

//产生地形绕射路径
bool GetTerrainDiffractionPath(const Terrain* terrain,const Point3D& tx,const Point3D& rx, TerrainDiffractionPath*& outPath);


#endif
