#ifndef RTLBS_TDOA_RESIDUAL_H
#define RTLBS_TDOA_RESIDUAL_H

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "localization/generalsource.h"

#include <glog/export.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ceres/ceres.h>

//TDOA WLS 定位残差表达式
class TDOAResidual {
private:
	RtLbsType m_x1;						/** @brief	参考 x 坐标	*/
	RtLbsType m_y1;						/** @brief	参考 y 坐标	*/
	RtLbsType m_xi;						/** @brief	x 坐标	*/
	RtLbsType m_yi;						/** @brief	y 坐标	*/
	RtLbsType m_timeDiff;				/** @brief	测量到的时间差	*/
	RtLbsType m_weight;					/** @brief	方程的权重	*/

public:
	TDOAResidual();
	TDOAResidual(const GeneralSource* refSource, const GeneralSource* dataSource, RtLbsType weight = 1.0);
	TDOAResidual(const TDOAResidual& residual);
	~TDOAResidual();
	TDOAResidual& operator = (const TDOAResidual& residual);
	void Init(const GeneralSource* refSource, const GeneralSource* dataSource, RtLbsType weight = 1.0);
	double GetResidual(const double* position) const;
	RtLbsType GetWeight() const;
	void SetWeight(RtLbsType weight);

	template <typename T> bool operator ()(const T* const position, T* residual) const {
		T p1_x = T(m_x1);							/** @brief	参考点P1 x坐标	*/
		T p1_y = T(m_y1);							/** @brief	参考点P1 y坐标	*/
		T pi_x = T(m_xi);							/** @brief	数据点Pi x坐标	*/
		T pi_y = T(m_yi);							/** @brief	数据点Pi y坐标	*/
		T x = position[0];							/** @brief	预测点 x坐标	*/
		T y = position[1];							/** @brief	预测点 y坐标	*/
		T d1 = ceres::sqrt((x - p1_x) * (x - p1_x) + (y - p1_y) * (y - p1_y));
		T di = ceres::sqrt((x - pi_x) * (x - pi_x) + (y - pi_y) * (y - pi_y));
		residual[0] = ((di - d1) / T(LIGHT_VELOCITY_AIR) - T(m_timeDiff)) *  T(m_weight * 1e9);				//ns 时间差
		return true;
	}

};

#endif
