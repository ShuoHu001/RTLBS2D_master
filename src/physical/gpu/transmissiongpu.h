#ifndef RTLBS_TRANSMISSIONGPU
#define RTLBS_TRANSMISSIONGPU

#include "geometry/gpu/ray2dgpu.h"
#include "geometry/gpu/segment2dgpu.h"
#include "geometry/gpu/Intersection2Dgpu.h"
#include "tree/gpu/treenodegpu.h"


HOST_DEVICE_FUNC bool GenerateTransmitRayGPU(const Intersection2DGPU& inter, int prevInterId, const Segment2DGPU* segments, Ray2DGPU* newRay);

HOST_DEVICE_FUNC bool GenerateEmpiricalTransmitRayGPU(const Intersection2DGPU& inter, int prevInterId, const Segment2DGPU* segments, Ray2DGPU* newRay);

HOST_DEVICE_FUNC bool GenerateTransmitRayGPU(const Intersection2DGPU& inter, int prevInterId, const Segment2DGPU* segments, Ray2DGPU* newRay, TreeNodeGPU* node, int layer);

HOST_DEVICE_FUNC bool GenerateEmpiricalTransmitRayGPU(const Intersection2DGPU& inter, int prevInterId, const Segment2DGPU* segments, Ray2DGPU* newRay, TreeNodeGPU* node, int layer);

#endif
