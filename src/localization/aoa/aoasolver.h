#ifndef RTLBS_AOASOLVER
#define RTLBS_AOASOLVER

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "localization/generalsource.h"
#include "geometry/bbox2d.h"
#include "aoa_residual.h"
#include "configuration/localization/solvingconfig.h"
#include "math/vectorfunction.h"
#include "localization/lossfunction/lossfunction.h"

//#include ceres library 用于求解方程
#include <glog/export.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ceres/ceres.h>



class AOASolver {
public:
	std::vector<GeneralSource*> m_gsData;					/** @brief	广义源数据	*/
	Point2D m_solution;										/** @brief	AOA定位解*/

public:
	AOASolver();
	AOASolver(const AOASolver& solver);
	~AOASolver();
	AOASolver& operator = (const AOASolver& solver);
	void SetGeneralSource(const std::vector<GeneralSource*>& gsData);
	Point2D Solving_LS(const BBox2D& bbox, const Point2D& initPoint);																//最小二乘方法求解器
	Point2D Solving_WLS(const BBox2D& bbox, const Point2D& initPoint);																//加权最小二乘方法求解器
	Point2D Solving_TSWLS(const BBox2D& bbox, const Point2D& initPoint);															//两步最小二乘方法求解器
	Point2D Solving_IRLS(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint);								//迭代加权最小二乘方法求解器
	Point2D Solving_WIRLS(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint);								//初始权重迭代加权最小二乘方法求解器
	Point2D Solving(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint);										//方程求解
	void UpdateResidualWeight(const double* position, std::vector<AOAResidual>& aoaResiduals, double& aoaResidual_STD);				//更新残差权重
	double GetResidualSTD(const double* position, std::vector<AOAResidual>& aoaResiduals);
};


#endif
