#ifndef RTLBS_VECTORFUNCTION_H
#define RTLBS_VECTORFUNCTION_H

#include "rtlbs.h"
#include "utility/define.h"

namespace vectoroperator {
	// ����ƽ��ֵ��Mean��
		template <typename T>
	T CalculateMean(const std::vector<T>&data) {
		T sum = std::accumulate(data.begin(), data.end(), T(0));  // ���
		return sum / data.size();  // ƽ��ֵ = �ܺ� / ��������
	}

	// ���㷽�Variance��
	template <typename T>
	T CalculateVariance(const std::vector<T>& data) {
		T mean = CalculateMean(data);  // �ȼ���ƽ��ֵ
		T variance = 0.0;

		for (const auto& value : data) {
			variance += (value - mean) * (value - mean);  // ����ÿ��ֵ���ֵ�Ĳ��ƽ��
		}

		variance /= data.size();  // ���� = ���ƽ���� / ��������
		return variance;
	}

	// �����׼�Standard Deviation��
	template <typename T>
	T CalculateStandardDeviation(const std::vector<T>& data) {
		T mean = CalculateMean(data);  // �ȼ���ƽ��ֵ
		T variance = 0.0;

		for (const auto& value : data) {
			variance += (value - mean) * (value - mean);  // ����ÿ��ֵ���ֵ�Ĳ��ƽ��
		}

		variance /= data.size();  // ���� = ���ƽ���� / ��������
		return std::sqrt(variance);  // ��׼�� = �����ƽ����
	}

	// �����������RMSE, Root Mean Square Error��
	// ������������ͬ��С���������ֱ��ʾʵ��ֵ��Ԥ��ֵ
	template <typename T>
	T CalculateRMSE(const std::vector<T>& observed, const std::vector<T>& predicted) {
		if (observed.size() != predicted.size()) {
			throw std::invalid_argument("Input vectors must have the same size");
		}

		T sum = 0.0;
		for (size_t i = 0; i < observed.size(); ++i) {
			T diff = observed[i] - predicted[i];
			sum += diff * diff;  // ���ƽ����
		}

		return std::sqrt(sum / observed.size());  // RMSE = sqrt(���ƽ���� / ��������)
	}
}



#endif
