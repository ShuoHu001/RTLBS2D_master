#ifndef RTLBS_TDOASOLVER
#define RTLBS_TDOASOLVER

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "localization/generalsource.h"
#include "localization/aoa_tdoa/tdoa_residual.h"
#include "configuration/localization/solvingconfig.h"
#include "math/vectorfunction.h"
#include "localization/lossfunction/lossfunction.h"

//#include ceres library 用于求解方程
#include <glog/export.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ceres/ceres.h>

class TDOASolver {
public:
	GeneralSource* m_refSource;								/** @brief	参考广义源	*/
	std::vector<GeneralSource*> m_gsData;					/** @brief	广义源数据	*/
	Point2D m_solution;										/** @brief	TDOA定位解	*/

public:
	TDOASolver();
	TDOASolver(const TDOASolver& solver);
	~TDOASolver();
	TDOASolver& operator = (const TDOASolver& solver);
	void SetGeneralSource(GeneralSource* refSource, const std::vector<GeneralSource*>& gsData);				//设定广义源基本信息
	void SetGeneralSource(GeneralSource* refSource, GeneralSource* gs1, GeneralSource* gs2);				//设定广义源基本信息-最低三个广义源组合求解
	RtLbsType Solving_LS(const BBox2D& bbox, Point2D& outP);													//使用最小二乘方法求解,返回最后一次迭代的误差
	Point2D Solving_WLS(const BBox2D& bbox, const Point2D& initPoint);																	//WLS方法求解方程
	Point2D Solving_IRLS(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint);		//IRLS方法求解方程
	Point2D Solving_WIRLS(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint);	//WIRLS方法求解方程
	Point2D Solving(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint);		//求解方程
	void UpdateResidualWeight(const double* position, std::vector<TDOAResidual>& tdoaResiduals, double& tdoaResidual_STD);		//更新残差权重
};

#endif
