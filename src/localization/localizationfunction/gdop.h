#ifndef RTLBS_LOCALIZATION_FUNCTION_GDOP_H
#define RTLBS_LOCALIZATION_FUNCTION_GDOP_H

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "math/point2d.h"
#include "math/matrixfunction.h"


//计算AOA方法的GDOP
RtLbsType ComputeGDOPForAOA(const std::vector<Point2D>& bss, const Point2D& ms);

//计算TOA方法的GDOP
RtLbsType ComputeGDOPForTOA(const std::vector<Point2D>& bss, const Point2D& ms);

//计算AOATOA方法的GDOP
RtLbsType ComputeGDOPForAOATOA(const std::vector<Point2D>& bss, const Point2D& ms);

//计算AOATDOA方法的GDOP
RtLbsType ComputeGDOPForAOATDOA(const std::vector<Point2D>& bss, const Point2D& ms);

#endif
