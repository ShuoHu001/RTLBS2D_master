#ifndef RTLBS_RAYSPLITGPU
#define RTLBS_RAYSPLITGPU

#include "rtlbs.h"
#include "geometry/gpu/ray2dgpu.h"
#include "utility/define.h"
#include "geometry/gpu/Intersection2Dgpu.h"
#include "geometry/gpu/segment2dgpu.h"



/**
* @brief		GPU���߷��Ѻ���
* @param[in]	
* @return		
* @author		��˶
* @date			2023/05/25
*/
HOST_DEVICE_FUNC void GenerateSplittingRayGPU(Ray2DGPU& initRay, int splitNum, Ray2DGPU* newRays, Segment2DGPU* segments);

#endif