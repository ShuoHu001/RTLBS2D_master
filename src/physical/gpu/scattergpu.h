#ifndef RTLBS_SCATTERGPU
#define RTLBS_SCATTERGPU

#include "rtlbs.h"
#include "geometry/gpu/ray2dgpu.h"
#include "utility/define.h"
#include "geometry/gpu/Intersection2Dgpu.h"
#include "geometry/gpu/segment2dgpu.h"

HOST_DEVICE_FUNC void GenerateScatterRayGPU(Ray2DGPU& incident_ray, Intersection2DGPU* intersect, Ray2DGPU* newRays, int raynum);

#endif
