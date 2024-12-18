#ifndef RTLBS_MATRIXFUNCTION_H
#define RTLBS_MATRIXFUNCTION_H

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include <Eigen/Dense>
#include <Eigen/SVD>

//¼ÆËã¾ØÕóµÄÎ±Äæ¾ØÕó
Eigen::MatrixXd ComputePseudoInverser(const Eigen::MatrixXd& matrix, double tolerance = 1e-9);

#endif
