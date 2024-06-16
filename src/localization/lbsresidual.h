#ifndef RTLBS_LBSRESIDUAL
#define RTLBS_LBSRESIDUAL

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "generalsource.h"


//AOA WLS 定位残差表达式
class AOAWLSResidual {
private:
	RtLbsType m_x;		/** @brief	x 坐标	*/
	RtLbsType m_y;		/** @brief	y 坐标	*/
	RtLbsType m_phi;		/** @brief	测量到的角度值	*/
	RtLbsType m_weight;		/** @brief	方程的权重	*/
	RtLbsType m_cosPhi;		/** @brief	角度值余弦	*/
	RtLbsType m_sinPhi;		/** @brief	角度值正弦	*/

public:
	AOAWLSResidual();
	AOAWLSResidual(RtLbsType x, RtLbsType y, RtLbsType phi, RtLbsType weight);
	AOAWLSResidual(const GeneralSource* source);
	AOAWLSResidual(const AOAWLSResidual& residual);
	~AOAWLSResidual();
	AOAWLSResidual& operator = (const AOAWLSResidual& residual);

	void Init(const GeneralSource* source);						//从广义源初始化
	RtLbsType GetWeight() const;								//获得权重

	//广义残差表达式，适用于ceres优化库
	template <typename T> bool operator()(const T* const position, T* residual) const {
		T dx = position[0] - T(m_x);
		T dy = position[1] - T(m_y);
		residual[0] = (dx * T(m_sinPhi) - dy * T(m_cosPhi)) * T(m_weight);
		return true;
	}
 
	RtLbsType GetResidual(RtLbsType* position) const;	//计算残差
	void SetWeight(RtLbsType weight);					//设置权重
};


#endif
