#ifndef RTLBS_LOCALIZATION_FUNCTION_CRLB_H
#define RTLBS_LOCALIZATION_FUNCTION_CRLB_H

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "math/point2d.h"
#include "math/matrixfunction.h"

//����AOA�㷨��CRLB
RtLbsType ComputeCRLBForAOA(const std::vector<Point2D>& bss, const Point2D& ms, double phi_noise_sigma);

//����TOA�㷨��CRLB
RtLbsType ComputeCRLBForTOA(const std::vector<Point2D>& bss, const Point2D& ms, double time_noise_sigma);

//����AOA/TOA�㷨��CRLB
RtLbsType ComputeCRLBForAOATOA(const std::vector<Point2D>& bss, const Point2D& ms, double phi_noise_sigma, double time_noise_sigma);

//����AOA/TDOA�㷨��CRLB
RtLbsType ComputeCRLBForAOATDOA(const std::vector<Point2D>& bss, const Point2D& ms, double phi_noise_sigma, double time_noise_sigma);

#endif
