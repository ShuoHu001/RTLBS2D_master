#ifndef RTLBS_AOATOASOLVER
#define RTLBS_AOATOASOLVER
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

class AOATOASolver {
public:
	std::vector<GeneralSource*> m_gsData;						/** @brief	广义源	*/
	Point2D m_solution;											/** @brief	AOATOA定位解	*/

public:
	AOATOASolver();
	AOATOASolver(const AOATOASolver& solver);
	~AOATOASolver();
	AOATOASolver& operator = (const AOATOASolver& solver);
	void SetGeneralSource(const std::vector<GeneralSource*>& gsData);
	void SetGeneralSource(GeneralSource* gs1, GeneralSource* gs2);
	RtLbsType Solving_LS(Point2D& outP);
	Point2D Solving_WIRLS(int iterNum, RtLbsType tol, const Point2D& initPoint);
};

#endif