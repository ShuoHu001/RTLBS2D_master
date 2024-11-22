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

		// Evaluate ����������ʧֵ�͵���
		void Evaluate(double s, double* rho) const override;

	private:
		const double delta_;  // ��ֵ��������ʧ��������״
	};

	class GemanMcClureLoss : public ceres::LossFunction {
	public:
		explicit GemanMcClureLoss(double delta);

		// Evaluate ����������ʧֵ�͵���
		void Evaluate(double s, double* rho) const override;

	private:
		const double delta_;  // ��ֵ��������ʧ��������״
	};


	class DCSLoss : public ceres::LossFunction {
	public:
		// ���캯��������һ������ delta ��Ϊ���Ʋ���
		explicit DCSLoss(double delta);

		// Evaluate ����������ʧֵ�͵���
		void Evaluate(double s, double* rho) const override;

	private:
		const double delta_;  // ������ʧ������״�Ĳ���
	};

	ceres::LossFunction* CreateLossFunction(LOSSFUNCTIONTYPE lossType, double lossParam);
}

#endif
