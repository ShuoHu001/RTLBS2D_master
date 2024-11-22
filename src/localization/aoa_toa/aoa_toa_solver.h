#ifndef RTLBS_AOATOASOLVER
#define RTLBS_AOATOASOLVER
#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "localization/generalsource.h"
#include "localization/aoa/aoa_residual.h"
#include "localization/toa/toa_residual.h"
#include "configuration/localization/solvingconfig.h"
#include "math/vectorfunction.h"
#include "localization/lossfunction/lossfunction.h"

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
	RtLbsType Solving_LS(const BBox2D& bbox, Point2D& outP);
	Point2D Solving_WLS(const BBox2D& bbox, const Point2D& initPoint);																//加权最小二乘方法求解器
	Point2D Solving_IRLS(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint);								//迭代加权最小二乘方法求解器
	Point2D Solving_WIRLS(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint);								//初始权重迭代加权最小二乘方法求解器
	Point2D Solving(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint);										//方程求解
	void UpdateResidualWeight(const double* position, std::vector<AOAResidual>& aoaResiduals, std::vector<TOAResidual>& toaResiduals, double& aoaResidual_STD, double& toaResidual_STD);
	double GetAOAResiudalSTD(const double* position, std::vector<AOAResidual>& aoaResiduals);
	double GetTOAResidualSTD(const double* position, std::vector<TOAResidual>& toaResiduals);
};

#endif