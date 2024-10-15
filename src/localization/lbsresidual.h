#ifndef RTLBS_LBSRESIDUAL
#define RTLBS_LBSRESIDUAL

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "generalsource.h"

#include <glog/export.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <ceres/internal/jet_traits.h>

template <typename T>
struct is_jet : std::false_type {};

template <typename T, int N>
struct is_jet<ceres::Jet<T, N>> : std::true_type {};

// 从 T 类型获取 double 类型的值
template <typename T>
double get_double_value(const T& value) {
	if constexpr (is_jet<T>::value) {
		return value.a;
	}
	else {
		return static_cast<double>(value);
	}
}


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
	RtLbsType Test(int a) { return a; };

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


//AOA LS 定位残差表达式
class AOALSResidual {
private:
	RtLbsType m_x;			/** @brief	x 坐标	*/
	RtLbsType m_y;			/** @brief	y 坐标	*/
	RtLbsType m_phi;		/** @brief	测量到的角度值	*/
	RtLbsType m_cosPhi;		/** @brief	角度值余弦	*/
	RtLbsType m_sinPhi;		/** @brief	角度值正弦	*/

public:
	AOALSResidual();
	AOALSResidual(RtLbsType x, RtLbsType y, RtLbsType phi);
	AOALSResidual(const GeneralSource* source);
	~AOALSResidual();
	AOALSResidual& operator = (const AOALSResidual& residual);

	void Init(const GeneralSource* source);						//从广义源初始化
	RtLbsType GetResidual(RtLbsType* position) const;	//计算残差

	//广义残差表达式，适用于ceres优化库
	template <typename T> bool operator()(const T* const position, T* residual) const {
		T dx = position[0] - T(m_x);
		T dy = position[1] - T(m_y);
		residual[0] = dx * T(m_sinPhi) - dy * T(m_cosPhi);
		return true;
	}

};



//TDOA LS 定位残差表达式
class TDOALSResidual {
private:
	RtLbsType m_x1;						/** @brief	参考 x 坐标	*/
	RtLbsType m_y1;						/** @brief	参考 y 坐标	*/
	RtLbsType m_xi;						/** @brief	x 坐标	*/
	RtLbsType m_yi;						/** @brief	y 坐标	*/
	RtLbsType m_timeDiff;				/** @brief	测量到的时间差	*/

public:
	TDOALSResidual();
	TDOALSResidual(RtLbsType ref_x, RtLbsType ref_y, RtLbsType x, RtLbsType y, RtLbsType timeDiff);
	TDOALSResidual(const GeneralSource* refSource, const GeneralSource* dataSource);
	TDOALSResidual(const TDOALSResidual& residual);
	~TDOALSResidual();
	TDOALSResidual& operator = (const TDOALSResidual& residual);
	void Init(const GeneralSource* refSource, const GeneralSource* dataSource);

	RtLbsType GetResidual(RtLbsType* position) const;							//计算残差

	template <typename T> bool operator ()(const T* const position, T* residual) const {
		T p1_x = T(m_x1);							/** @brief	参考点P1 x坐标	*/
		T p1_y = T(m_y1);							/** @brief	参考点P1 y坐标	*/
		T pi_x = T(m_xi);							/** @brief	数据点Pi x坐标	*/
		T pi_y = T(m_yi);							/** @brief	数据点Pi y坐标	*/
		T x = position[0];							/** @brief	预测点 x坐标	*/
		T y = position[1];							/** @brief	预测点 y坐标	*/
		T d1 = ceres::sqrt((x - p1_x) * (x - p1_x) + (y - p1_y) * (y - p1_y));
		T di = ceres::sqrt((x - pi_x) * (x - pi_x) + (y - pi_y) * (y - pi_y));
		residual[0] = (di - d1) - T(m_timeDiff) * T(LIGHT_VELOCITY_AIR);
		return true;
	}
};

//TDOA WLS 定位残差表达式
class TDOAWLSResidual {
private:
	RtLbsType m_x1;						/** @brief	参考 x 坐标	*/
	RtLbsType m_y1;						/** @brief	参考 y 坐标	*/
	RtLbsType m_xi;						/** @brief	x 坐标	*/
	RtLbsType m_yi;						/** @brief	y 坐标	*/
	RtLbsType m_timeDiff;				/** @brief	测量到的时间差	*/
	RtLbsType m_weight;					/** @brief	方程的权重	*/

public:
	TDOAWLSResidual();
	TDOAWLSResidual(RtLbsType ref_x, RtLbsType ref_y, RtLbsType x, RtLbsType y, RtLbsType timeDiff, RtLbsType weight);
	TDOAWLSResidual(const GeneralSource* refSource, const GeneralSource* dataSource);
	TDOAWLSResidual(const TDOAWLSResidual& residual);
	~TDOAWLSResidual();
	TDOAWLSResidual& operator = (const TDOAWLSResidual& residual);
	void Init(const GeneralSource* refSource, const GeneralSource* dataSource);
	RtLbsType GetResidual(RtLbsType* position) const;
	void SetWeight(RtLbsType weight);
	RtLbsType GetWeight() const;

	template <typename T> bool operator ()(const T* const position, T* residual) const {
		T p1_x = T(m_x1);							/** @brief	参考点P1 x坐标	*/
		T p1_y = T(m_y1);							/** @brief	参考点P1 y坐标	*/
		T pi_x = T(m_xi);							/** @brief	数据点Pi x坐标	*/
		T pi_y = T(m_yi);							/** @brief	数据点Pi y坐标	*/
		T x = position[0];							/** @brief	预测点 x坐标	*/
		T y = position[1];							/** @brief	预测点 y坐标	*/
		T d1 = ceres::sqrt((x - p1_x) * (x - p1_x) + (y - p1_y) * (y - p1_y));
		T di = ceres::sqrt((x - pi_x) * (x - pi_x) + (y - pi_y) * (y - pi_y));
		residual[0] = ((di - d1) - T(m_timeDiff) * LIGHT_VELOCITY_AIR) * T(m_weight);
		return true;
	}

};



//AOA-TOA LS定位残差表达式
class TOAResidual {
private:
	RtLbsType m_x;						/** @brief	x 坐标	*/
	RtLbsType m_y;						/** @brief	y 坐标	*/
	RtLbsType m_time;					/** @brief	测量到的时间	*/

public:
	TOAResidual();
	TOAResidual(RtLbsType x, RtLbsType y, RtLbsType delay);
	TOAResidual(const GeneralSource* source);
	~TOAResidual();
	TOAResidual& operator = (const TOAResidual& r);

	void Init(const GeneralSource* source);
	RtLbsType GetTOAResidual(RtLbsType* position) const;

	//广义残差表达式，适用于ceres优化库
	template <typename T> bool operator() (const T* const position, T* residual) const {
		T dx = position[0] - T(m_x);
		T dy = position[1] - T(m_y);
		T distance = sqrt(dx * dx + dy * dy);
		T calTime = distance / T(LIGHT_VELOCITY_AIR);  //单位ns
		residual[0] = calTime - m_time;
		return true;
	}
};


//AOA-TOA WLS 定位残差表达式
class TOAWLSResidual {
private:
	RtLbsType m_x;						/** @brief	x 坐标	*/
	RtLbsType m_y;						/** @brief	y 坐标	*/
	RtLbsType m_time;					/** @brief	测量到的时间	*/
	RtLbsType m_weight;					/** @brief	残差权重	*/

public:
	TOAWLSResidual();
	TOAWLSResidual(RtLbsType x, RtLbsType y, RtLbsType delay, RtLbsType weight);
	TOAWLSResidual(const GeneralSource* source);
	~TOAWLSResidual();
	TOAWLSResidual& operator = (const TOAWLSResidual& r);

	void Init(const GeneralSource* source);
	RtLbsType GetResidual(RtLbsType* position) const;
	RtLbsType GetWeight() const;								//获得权重
	void SetWeight(RtLbsType weight);							//设置权重
	//广义残差表达式，适用于ceres优化库
	template <typename T> bool operator() (const T* const position, T* residual) const {
		T dx = position[0] - T(m_x);
		T dy = position[1] - T(m_y);
		T distance = sqrt(dx * dx + dy * dy);
		T calTime = distance / T(LIGHT_VELOCITY_AIR);  //单位ns
		residual[0] = calTime - m_time;
		return true;
	}
};


#endif
