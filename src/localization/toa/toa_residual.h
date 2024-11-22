#ifndef RTLBS_TOA_RESIDUAL_H
#define RTLBS_TOA_RESIDUAL_H

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "localization/generalsource.h"

#include <glog/export.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ceres/ceres.h>

class TOAResidual {
private:
	RtLbsType m_x;						/** @brief	x 坐标	*/
	RtLbsType m_y;						/** @brief	y 坐标	*/
	RtLbsType m_time;					/** @brief	测量到的时间	*/
	RtLbsType m_weight;					/** @brief	残差权重	*/

public:
	TOAResidual();
	TOAResidual(const GeneralSource* source, RtLbsType weight = 1.0);
	TOAResidual(const TOAResidual& r);
	~TOAResidual();
	TOAResidual& operator = (const TOAResidual& r);
	void Init(const GeneralSource* source, RtLbsType weight = 1.0);
	double GetResidual(const double* position) const;
	RtLbsType GetWeight() const;								//获得权重
	void SetWeight(RtLbsType weight);							//设置权重

	//广义残差表达式，适用于ceres优化库
	template <typename T> bool operator() (const T* const position, T* residual) const {
		T dx = position[0] - T(m_x);
		T dy = position[1] - T(m_y);
		T distance = ceres::sqrt(dx * dx + dy * dy);
		T calTime = distance / T(LIGHT_VELOCITY_AIR);  //单位ns
		residual[0] = (calTime - m_time) * 1e9 * T(m_weight);
		return true;
	}

};

#endif
