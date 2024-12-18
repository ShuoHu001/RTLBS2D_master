#ifndef RTLBS_VECTORFUNCTION_H
#define RTLBS_VECTORFUNCTION_H

#include "rtlbs.h"
#include "utility/define.h"

namespace vectoroperator {
	// 计算平均值（Mean）
		template <typename T>
	T CalculateMean(const std::vector<T>&data) {
		T sum = std::accumulate(data.begin(), data.end(), T(0));  // 求和
		return sum / data.size();  // 平均值 = 总和 / 数据数量
	}

	// 计算方差（Variance）
	template <typename T>
	T CalculateVariance(const std::vector<T>& data) {
		T mean = CalculateMean(data);  // 先计算平均值
		T variance = 0.0;

		for (const auto& value : data) {
			variance += (value - mean) * (value - mean);  // 计算每个值与均值的差的平方
		}

		variance /= data.size();  // 方差 = 差的平方和 / 数据数量
		return variance;
	}

	// 计算标准差（Standard Deviation）
	template <typename T>
	T CalculateStandardDeviation(const std::vector<T>& data) {
		T mean = CalculateMean(data);  // 先计算平均值
		T variance = 0.0;

		for (const auto& value : data) {
			variance += (value - mean) * (value - mean);  // 计算每个值与均值的差的平方
		}

		variance /= data.size();  // 方差 = 差的平方和 / 数据数量
		return std::sqrt(variance);  // 标准差 = 方差的平方根
	}

	// 计算均方根误差（RMSE, Root Mean Square Error）
	// 适用于两个相同大小的向量，分别表示实际值和预测值
	template <typename T>
	T CalculateRMSE(const std::vector<T>& observed, const std::vector<T>& predicted) {
		if (observed.size() != predicted.size()) {
			throw std::invalid_argument("Input vectors must have the same size");
		}

		T sum = 0.0;
		for (size_t i = 0; i < observed.size(); ++i) {
			T diff = observed[i] - predicted[i];
			sum += diff * diff;  // 误差平方和
		}

		return std::sqrt(sum / observed.size());  // RMSE = sqrt(误差平方和 / 数据数量)
	}

	//获取矢量最小元素对应的索引
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

	//获取矢量最大元素对应的索引
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

	//获取矢量最大值
	template <typename T>
	T GetMax(const std::vector<T>& data) {
		return data[GetMaxIndex(data)];
	}

	//获取矢量最小值
	template <typename T>
	T GetMin(const std::vector<T>& data) {
		return data[GetMinIndex(data)];
	}

}



#endif
