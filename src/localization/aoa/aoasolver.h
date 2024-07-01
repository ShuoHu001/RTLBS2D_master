#ifndef RTLBS_AOASOLVER
#define RTLBS_AOASOLVER

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
	void Solving_LS();																//最小二乘方法求解器
	void Solving_WLS();																//加权最小二乘方法求解器
	Point2D Solving_WIRLS(int iterNum, RtLbsType tol, const Point2D& initPoint);				//初始权重迭代加权最小二乘方法求解器
	void Solving_IRLS(int iterNum, RtLbsType tol);									//迭代加权最小二乘方法求解器
	void Solving_ElaspNet(RtLbsType lamda1, RtLbsType lamda2);						//弹性网络求解器（L1正则、L2正则）
};


#endif
