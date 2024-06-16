#ifndef RTLBS_DIFFRACTION
#define RTLBS_DIFFRACTION

#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/ray2d.h"
#include "geometry/vector2d.h"
#include "geometry/segment2d.h"
#include "managers/randomanager.h"


bool GenerateDiffractRays(Ray2D& incident_ray, Wedge2D* wedge, std::vector<Ray2D>* rays);


#endif
