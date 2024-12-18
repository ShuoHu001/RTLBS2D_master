#ifndef RTLBS_LOCALIZATION_FUNCTION_CRLB_H
#define RTLBS_LOCALIZATION_FUNCTION_CRLB_H

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "math/point2d.h"
#include "math/matrixfunction.h"

//计算AOA算法的CRLB
RtLbsType ComputeCRLBForAOA(const std::vector<Point2D>& bss, const Point2D& ms, double phi_noise_sigma);

//计算TOA算法的CRLB
RtLbsType ComputeCRLBForTOA(const std::vector<Point2D>& bss, const Point2D& ms, double time_noise_sigma);

//计算AOA/TOA算法的CRLB
RtLbsType ComputeCRLBForAOATOA(const std::vector<Point2D>& bss, const Point2D& ms, double phi_noise_sigma, double time_noise_sigma);

//计算AOA/TDOA算法的CRLB
RtLbsType ComputeCRLBForAOATDOA(const std::vector<Point2D>& bss, const Point2D& ms, double phi_noise_sigma, double time_noise_sigma);

#endif
