#ifndef RTLBS_REFLECTION
#define RTLBS_REFLECTION

#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/ray2d.h"
#include "geometry/Intersection2D.h"
#include "geometry/segment2d.h"



bool GenerateReflectRay(Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray);

void GenerateReflectRayOnToughSurface(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray);


#endif


