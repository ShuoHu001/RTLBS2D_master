#ifndef RTLBS_REFLECTION
#define RTLBS_REFLECTION

#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/ray2d.h"
#include "geometry/Intersection2D.h"
#include "geometry/segment2d.h"
#include "geometry/terrain/terrain.h"
#include "radiowave/raypath/raypath3d.h"



//产生常规面的反射射线
bool GenerateReflectRay(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray);

//产生粗糙表面射线
void GenerateReflectRayOnToughSurface(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray);

//产生地形反射路径集合
bool GenerateTerrainReflectionPaths(const Terrain* terrain, const Point3D& tx, const Point3D& rx, std::vector<RayPath3D*>& outPaths);


#endif


