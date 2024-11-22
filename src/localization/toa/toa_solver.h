#ifndef RTLBS_TOASOLVER
#define RTLBS_TOASOLVER

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "localization/generalsource.h"
#include "toa_residual.h"
#include "configuration/localization/solvingconfig.h"
#include "math/vectorfunction.h"
#include "localization/lossfunction/lossfunction.h"

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
	RtLbsType Solving_LS(const BBox2D& bbox, Point2D& outP);
	Point2D Solving_WLS(const BBox2D& bbox, const Point2D& initPoint);
	Point2D Solving_IRLS(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint);
	Point2D Solving_WIRLS(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint);
	Point2D Solving(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint);
	void UpdateResidualWeight(const double* position, std::vector<TOAResidual>& toaResiduals, double& toaResidual_STD);
	double GetResidualSTD(const double* position, std::vector<TOAResidual>& toaResiduals);
};

#endif
