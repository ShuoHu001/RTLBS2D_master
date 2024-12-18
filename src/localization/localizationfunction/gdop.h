#ifndef RTLBS_LOCALIZATION_FUNCTION_GDOP_H
#define RTLBS_LOCALIZATION_FUNCTION_GDOP_H

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "math/point2d.h"
#include "math/matrixfunction.h"


//����AOA������GDOP
RtLbsType ComputeGDOPForAOA(const std::vector<Point2D>& bss, const Point2D& ms);

//����TOA������GDOP
RtLbsType ComputeGDOPForTOA(const std::vector<Point2D>& bss, const Point2D& ms);

//����AOATOA������GDOP
RtLbsType ComputeGDOPForAOATOA(const std::vector<Point2D>& bss, const Point2D& ms);

//����AOATDOA������GDOP
RtLbsType ComputeGDOPForAOATDOA(const std::vector<Point2D>& bss, const Point2D& ms);

#endif
