#ifndef RTLBS_DIFFRACTIONGPU
#define RTLBS_DIFFRACTIONGPU

#include "geometry/gpu/ray2dgpu.h"
#include "geometry/vector2d.h"
#include "geometry/gpu/segment2dgpu.h"
#include "geometry/gpu/wedge2dgpu.h"
#include "geometry/gpu/Intersection2Dgpu.h"
#include "tree/gpu/treenodegpu.h"

//“—ÕÍ…∆
HOST_DEVICE_FUNC bool GenerateDiffractRaysGPU(const Intersection2DGPU& inter, Ray2DGPU& incident_ray, Wedge2DGPU* wedge, int prevInterId, Ray2DGPU* newRays, int raynum);

HOST_DEVICE_FUNC bool GenerateDiffractRaysGPU(const Intersection2DGPU& inter, Ray2DGPU& incident_ray, Wedge2DGPU* wedge, int prevInterId, Ray2DGPU* newRays, TreeNodeGPU* nodes, int raynum, int layer);

#endif
