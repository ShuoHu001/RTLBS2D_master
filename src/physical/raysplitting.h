#ifndef RTLBS_RAYSPLIT
#define RTLBS_RAYSPLIT

#include "rtlbs.h"
#include "geometry/ray2d.h"
#include "utility/define.h"
#include "geometry/Intersection2D.h"
#include "geometry/segment2d.h"
#include "tree/pathnode.h"


bool IsGenerateSplittingRays(const Ray2D& rayInit, RtLbsType t, bool splitFlag, RtLbsType splitRadius, int& splitNum);

void GenerateSplittingRay(const Ray2D& ray, int splitNum, std::vector<Ray2D>* rays);

#endif
