#ifndef RTLBS_REFLECTIONGPU
#define RTLBS_REFLECTIONGPU

#include "geometry/gpu/ray2dgpu.h"
#include "geometry/gpu/Intersection2Dgpu.h"
#include "geometry/gpu/segment2dgpu.h"
#include "tree/gpu/pathnodegpu.h"
#include "tree/gpu/treenodegpu.h"



HOST_DEVICE_FUNC bool GenerateReflectRayGPU(const Intersection2DGPU& inter, int interId, const Segment2DGPU* segments, Ray2DGPU* ray);

HOST_DEVICE_FUNC bool GenerateReflectRayGPU(const Intersection2DGPU& inter, int interId, const Segment2DGPU* segments, Ray2DGPU* ray, TreeNodeGPU* treenode, int layer);
#endif


