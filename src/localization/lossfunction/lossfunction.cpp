#include "lossfunction.h"

custom_loss::FairLoss::FairLoss(double delta)
	: delta_(delta)
{
}

void custom_loss::FairLoss::Evaluate(double s, double* rho) const
{
	double sqrt_s = sqrt(s);
	rho[0] = 2 * delta_ * delta_ * (sqrt_s / delta_ - log(1 + sqrt_s / delta_));  // 损失值
	rho[1] = 1.0 / (1.0 + sqrt_s / delta_);  // 一阶导数
	rho[2] = -0.5 / (delta_ * (1.0 + sqrt_s / delta_));  // 二阶导数
}

custom_loss::GemanMcClureLoss::GemanMcClureLoss(double delta)
	: delta_(delta)
{
}

void custom_loss::GemanMcClureLoss::Evaluate(double s, double* rho) const
{
	double s_plus_delta2 = s + delta_ * delta_;
	rho[0] = s / s_plus_delta2;  // 损失值
	rho[1] = 1.0 / s_plus_delta2; // 一阶导数
	rho[2] = -1.0 / (s_plus_delta2 * s_plus_delta2); // 二阶导数
}

custom_loss::DCSLoss::DCSLoss(double delta)
	: delta_(delta)
{
}

void custom_loss::DCSLoss::Evaluate(double s, double* rho) const
{
	// s 是残差平方, delta_ 是控制参数
	double delta2 = delta_ * delta_;
	double s_over_delta2 = s / delta2;

	// 计算损失值 rho[0]
	rho[0] = delta2 * log(1.0 + s_over_delta2);

	// 计算一阶导数 rho[1]
	rho[1] = 1.0 / (1.0 + s_over_delta2);

	// 计算二阶导数 rho[2]
	rho[2] = -s_over_delta2 / ((1.0 + s_over_delta2) * (1.0 + s_over_delta2));
}

ceres::LossFunction* custom_loss::CreateLossFunction(LOSSFUNCTIONTYPE lossType, double lossParam)
{
	if (lossType == LOSS_NONE) {
		return nullptr;
	}
	else if (lossType == LOSS_HUBER) {
		return new ceres::HuberLoss(lossParam);
	}
	else if (lossType == LOSS_CAUCHY) {
		return new ceres::CauchyLoss(lossParam);
	}
	else if (lossType == LOSS_ARCTAN) {
		return new ceres::ArctanLoss(lossParam);
	}
	else if (lossType == LOSS_TUKEY) {
		return new ceres::TukeyLoss(lossParam);
	}
	else if (lossType == LOSS_FAIR) {
		return new custom_loss::FairLoss(lossParam);
	}
	else if (lossType == LOSS_GEMANMCCLURE) {
		return new custom_loss::GemanMcClureLoss(lossParam);
	}
	else if (lossType == LOSS_DCS) {
		return new custom_loss::DCSLoss(lossParam);
	}
	else {
		return nullptr;
	}
}
