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

	//��ȡʸ����СԪ�ض�Ӧ������
	template <typename T>
	size_t GetMinIndex(const std::vector<T>& data) {
		size_t minIndex = 0;
		for (size_t i = 1; i < data.size(); ++i) {
			if (data[i] < data[minIndex]) {
				minIndex = i;
			}
		}
		return minIndex;
	}

	//��ȡʸ�����Ԫ�ض�Ӧ������
	template <typename T>
	size_t GetMaxIndex(const std::vector<T>& data) {
		size_t maxIndex = 0;
		for (size_t i = 1; i < data.size(); ++i) {
			if (data[i] > data[maxIndex]) {
				maxIndex = i;
			}
		}
		return maxIndex;
	}

	//��ȡʸ�����ֵ
	template <typename T>
	T GetMax(const std::vector<T>& data) {
		return data[GetMaxIndex(data)];
	}

	//��ȡʸ����Сֵ
	template <typename T>
	T GetMin(const std::vector<T>& data) {
		return data[GetMinIndex(data)];
	}

}



#endif
