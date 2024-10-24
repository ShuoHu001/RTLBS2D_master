#ifndef RTLBS_TOASOLVER
#define RTLBS_TOASOLVER

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "localization/generalsource.h"
#include "localization/lbsresidual.h"

//#include ceres library 用于求解方程
#include <glog/export.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ceres/ceres.h>


class TOASolver {
public:
	std::vector<GeneralSource*> m_gsData;						/** @brief	广义源	*/
	Point2D m_solution;											/** @brief	TOA定位解	*/

public:
	TOASolver();
	TOASolver(const TOASolver& solver);
	~TOASolver();
	TOASolver& operator = (const TOASolver& solver);
	void SetGeneralSource(const std::vector<GeneralSource*>& gsData);
	void SetGeneralSource(GeneralSource* gs1, GeneralSource* gs2);
	RtLbsType Solving_LS(Point2D& outP);
	Point2D Solving_WIRLS(int iterNum, RtLbsType tol, const Point2D& initPoint);
};

#endif
