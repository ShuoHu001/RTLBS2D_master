#ifndef RTLBS_AOA_RESIDUAL_H
#define RTLBS_AOA_RESIDUAL_H

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

//AOA WLS 定位残差表达式
class AOAResidual {
private:
	RtLbsType m_x;		/** @brief	x 坐标	*/
	RtLbsType m_y;		/** @brief	y 坐标	*/
	RtLbsType m_phi;		/** @brief	测量到的角度值	*/
	RtLbsType m_weight;		/** @brief	方程的权重	*/
	RtLbsType m_cosPhi;		/** @brief	角度值余弦	*/
	RtLbsType m_sinPhi;		/** @brief	角度值正弦	*/

public:
	AOAResidual();
	AOAResidual(const GeneralSource* source, RtLbsType weight = 1.0);
	AOAResidual(const AOAResidual& residual);
	~AOAResidual();
	AOAResidual& operator = (const AOAResidual& residual);

	void Init(const GeneralSource* source, RtLbsType weight = 1.0);						//从广义源初始化
	double GetResidual(const double* position) const;									//计算残差
	RtLbsType GetWeight() const;														//获得权重
	void SetWeight(RtLbsType weight);													//设置权重
	
	//广义残差表达式，适用于ceres优化库
	template <typename T> bool operator()(const T* const position, T* residual) const {
		T dx = position[0] - T(m_x);
		T dy = position[1] - T(m_y);
		residual[0] = (dx * T(m_sinPhi) - dy * T(m_cosPhi)) / T(m_weight);
		return true;
	}
};

#endif
