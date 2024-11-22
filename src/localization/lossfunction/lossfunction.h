#ifndef RTLBS_LOSSFUNCTION_H
#define RTLBS_LOSSFUNCTION_H

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include <glog/export.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>

namespace custom_loss {
	class FairLoss : public ceres::LossFunction {
	public:
		explicit FairLoss(double delta);

		// Evaluate 函数计算损失值和导数
		void Evaluate(double s, double* rho) const override;

	private:
		const double delta_;  // 阈值，控制损失函数的形状
	};

	class GemanMcClureLoss : public ceres::LossFunction {
	public:
		explicit GemanMcClureLoss(double delta);

		// Evaluate 函数计算损失值和导数
		void Evaluate(double s, double* rho) const override;

	private:
		const double delta_;  // 阈值，控制损失函数的形状
	};


	class DCSLoss : public ceres::LossFunction {
	public:
		// 构造函数，接受一个参数 delta 作为控制参数
		explicit DCSLoss(double delta);

		// Evaluate 函数计算损失值和导数
		void Evaluate(double s, double* rho) const override;

	private:
		const double delta_;  // 控制损失函数形状的参数
	};

	ceres::LossFunction* CreateLossFunction(LOSSFUNCTIONTYPE lossType, double lossParam);
}

#endif
