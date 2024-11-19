#ifndef RTLBS_SCATTERING
#define RTLBS_SCATTERING

#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/ray2d.h"
#include "geometry/Intersection2D.h"
#include "geometry/segment2d.h"


bool GenerateScatteringRays(Ray2D& incident_ray, const Intersection2D& inter, std::vector<Ray2D*>& ray);

#endif
