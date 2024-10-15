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

// �� T ���ͻ�ȡ double ���͵�ֵ
template <typename T>
double get_double_value(const T& value) {
	if constexpr (is_jet<T>::value) {
		return value.a;
	}
	else {
		return static_cast<double>(value);
	}
}


//AOA WLS ��λ�в���ʽ
class AOAWLSResidual {
private:
	RtLbsType m_x;		/** @brief	x ����	*/
	RtLbsType m_y;		/** @brief	y ����	*/
	RtLbsType m_phi;		/** @brief	�������ĽǶ�ֵ	*/
	RtLbsType m_weight;		/** @brief	���̵�Ȩ��	*/
	RtLbsType m_cosPhi;		/** @brief	�Ƕ�ֵ����	*/
	RtLbsType m_sinPhi;		/** @brief	�Ƕ�ֵ����	*/

public:
	AOAWLSResidual();
	AOAWLSResidual(RtLbsType x, RtLbsType y, RtLbsType phi, RtLbsType weight);
	AOAWLSResidual(const GeneralSource* source);
	AOAWLSResidual(const AOAWLSResidual& residual);
	~AOAWLSResidual();
	AOAWLSResidual& operator = (const AOAWLSResidual& residual);

	void Init(const GeneralSource* source);						//�ӹ���Դ��ʼ��
	RtLbsType GetWeight() const;								//���Ȩ��
	RtLbsType Test(int a) { return a; };

	//����в���ʽ��������ceres�Ż���
	template <typename T> bool operator()(const T* const position, T* residual) const {
		T dx = position[0] - T(m_x);
		T dy = position[1] - T(m_y);
		residual[0] = (dx * T(m_sinPhi) - dy * T(m_cosPhi)) * T(m_weight);
		return true;
	}
 
	RtLbsType GetResidual(RtLbsType* position) const;	//����в�
	void SetWeight(RtLbsType weight);					//����Ȩ��
};


//AOA LS ��λ�в���ʽ
class AOALSResidual {
private:
	RtLbsType m_x;			/** @brief	x ����	*/
	RtLbsType m_y;			/** @brief	y ����	*/
	RtLbsType m_phi;		/** @brief	�������ĽǶ�ֵ	*/
	RtLbsType m_cosPhi;		/** @brief	�Ƕ�ֵ����	*/
	RtLbsType m_sinPhi;		/** @brief	�Ƕ�ֵ����	*/

public:
	AOALSResidual();
	AOALSResidual(RtLbsType x, RtLbsType y, RtLbsType phi);
	AOALSResidual(const GeneralSource* source);
	~AOALSResidual();
	AOALSResidual& operator = (const AOALSResidual& residual);

	void Init(const GeneralSource* source);						//�ӹ���Դ��ʼ��
	RtLbsType GetResidual(RtLbsType* position) const;	//����в�

	//����в���ʽ��������ceres�Ż���
	template <typename T> bool operator()(const T* const position, T* residual) const {
		T dx = position[0] - T(m_x);
		T dy = position[1] - T(m_y);
		residual[0] = dx * T(m_sinPhi) - dy * T(m_cosPhi);
		return true;
	}

};



//TDOA LS ��λ�в���ʽ
class TDOALSResidual {
private:
	RtLbsType m_x1;						/** @brief	�ο� x ����	*/
	RtLbsType m_y1;						/** @brief	�ο� y ����	*/
	RtLbsType m_xi;						/** @brief	x ����	*/
	RtLbsType m_yi;						/** @brief	y ����	*/
	RtLbsType m_timeDiff;				/** @brief	��������ʱ���	*/

public:
	TDOALSResidual();
	TDOALSResidual(RtLbsType ref_x, RtLbsType ref_y, RtLbsType x, RtLbsType y, RtLbsType timeDiff);
	TDOALSResidual(const GeneralSource* refSource, const GeneralSource* dataSource);
	TDOALSResidual(const TDOALSResidual& residual);
	~TDOALSResidual();
	TDOALSResidual& operator = (const TDOALSResidual& residual);
	void Init(const GeneralSource* refSource, const GeneralSource* dataSource);

	RtLbsType GetResidual(RtLbsType* position) const;							//����в�

	template <typename T> bool operator ()(const T* const position, T* residual) const {
		T p1_x = T(m_x1);							/** @brief	�ο���P1 x����	*/
		T p1_y = T(m_y1);							/** @brief	�ο���P1 y����	*/
		T pi_x = T(m_xi);							/** @brief	���ݵ�Pi x����	*/
		T pi_y = T(m_yi);							/** @brief	���ݵ�Pi y����	*/
		T x = position[0];							/** @brief	Ԥ��� x����	*/
		T y = position[1];							/** @brief	Ԥ��� y����	*/
		T d1 = ceres::sqrt((x - p1_x) * (x - p1_x) + (y - p1_y) * (y - p1_y));
		T di = ceres::sqrt((x - pi_x) * (x - pi_x) + (y - pi_y) * (y - pi_y));
		residual[0] = (di - d1) - T(m_timeDiff) * T(LIGHT_VELOCITY_AIR);
		return true;
	}
};

//TDOA WLS ��λ�в���ʽ
class TDOAWLSResidual {
private:
	RtLbsType m_x1;						/** @brief	�ο� x ����	*/
	RtLbsType m_y1;						/** @brief	�ο� y ����	*/
	RtLbsType m_xi;						/** @brief	x ����	*/
	RtLbsType m_yi;						/** @brief	y ����	*/
	RtLbsType m_timeDiff;				/** @brief	��������ʱ���	*/
	RtLbsType m_weight;					/** @brief	���̵�Ȩ��	*/

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
		T p1_x = T(m_x1);							/** @brief	�ο���P1 x����	*/
		T p1_y = T(m_y1);							/** @brief	�ο���P1 y����	*/
		T pi_x = T(m_xi);							/** @brief	���ݵ�Pi x����	*/
		T pi_y = T(m_yi);							/** @brief	���ݵ�Pi y����	*/
		T x = position[0];							/** @brief	Ԥ��� x����	*/
		T y = position[1];							/** @brief	Ԥ��� y����	*/
		T d1 = ceres::sqrt((x - p1_x) * (x - p1_x) + (y - p1_y) * (y - p1_y));
		T di = ceres::sqrt((x - pi_x) * (x - pi_x) + (y - pi_y) * (y - pi_y));
		residual[0] = ((di - d1) - T(m_timeDiff) * LIGHT_VELOCITY_AIR) * T(m_weight);
		return true;
	}

};



//AOA-TOA LS��λ�в���ʽ
class TOAResidual {
private:
	RtLbsType m_x;						/** @brief	x ����	*/
	RtLbsType m_y;						/** @brief	y ����	*/
	RtLbsType m_time;					/** @brief	��������ʱ��	*/

public:
	TOAResidual();
	TOAResidual(RtLbsType x, RtLbsType y, RtLbsType delay);
	TOAResidual(const GeneralSource* source);
	~TOAResidual();
	TOAResidual& operator = (const TOAResidual& r);

	void Init(const GeneralSource* source);
	RtLbsType GetTOAResidual(RtLbsType* position) const;

	//����в���ʽ��������ceres�Ż���
	template <typename T> bool operator() (const T* const position, T* residual) const {
		T dx = position[0] - T(m_x);
		T dy = position[1] - T(m_y);
		T distance = sqrt(dx * dx + dy * dy);
		T calTime = distance / T(LIGHT_VELOCITY_AIR);  //��λns
		residual[0] = calTime - m_time;
		return true;
	}
};


//AOA-TOA WLS ��λ�в���ʽ
class TOAWLSResidual {
private:
	RtLbsType m_x;						/** @brief	x ����	*/
	RtLbsType m_y;						/** @brief	y ����	*/
	RtLbsType m_time;					/** @brief	��������ʱ��	*/
	RtLbsType m_weight;					/** @brief	�в�Ȩ��	*/

public:
	TOAWLSResidual();
	TOAWLSResidual(RtLbsType x, RtLbsType y, RtLbsType delay, RtLbsType weight);
	TOAWLSResidual(const GeneralSource* source);
	~TOAWLSResidual();
	TOAWLSResidual& operator = (const TOAWLSResidual& r);

	void Init(const GeneralSource* source);
	RtLbsType GetResidual(RtLbsType* position) const;
	RtLbsType GetWeight() const;								//���Ȩ��
	void SetWeight(RtLbsType weight);							//����Ȩ��
	//����в���ʽ��������ceres�Ż���
	template <typename T> bool operator() (const T* const position, T* residual) const {
		T dx = position[0] - T(m_x);
		T dy = position[1] - T(m_y);
		T distance = sqrt(dx * dx + dy * dy);
		T calTime = distance / T(LIGHT_VELOCITY_AIR);  //��λns
		residual[0] = calTime - m_time;
		return true;
	}
};


#endif
