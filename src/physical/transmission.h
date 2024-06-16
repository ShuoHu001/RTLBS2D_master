#ifndef RTLBS_TRANSMISSION
#define RTLBS_TRANSMISSION

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/ray2d.h"
#include "geometry/Intersection2D.h"
#include "geometry/segment2d.h"

bool GenerateTransmitRay(Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray, PATHNODETYPE& type);

//产生经验透射路径
bool GenerateEmpiricalTransmitRay(Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray, PATHNODETYPE& type);

#endif
