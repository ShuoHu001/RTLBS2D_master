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

class RDOAWLSResidual {
private:
	RtLbsType m_x1;						/** @brief	�ο�x����	*/
	RtLbsType m_y1;						/** @brief	�ο�y����	*/
	RtLbsType m_xi;						/** @brief	x����	*/
	RtLbsType m_yi;						/** @brief	y����	*/
	RtLbsType m_powerDiff;				/** @brief	�������Ĺ��ʲ�	*/
	GeneralSource* m_refSource;			/** @brief	�ο�����Դ	*/
	GeneralSource* m_dataSource;		/** @brief	���ݹ���Դ	*/

public:
	RDOAWLSResidual();
	RDOAWLSResidual(GeneralSource* refSource, GeneralSource* dataSource);
	RDOAWLSResidual(const RDOAWLSResidual& residual);
	~RDOAWLSResidual();
	RDOAWLSResidual& operator = (const RDOAWLSResidual& residual);
	void Init(GeneralSource* refSource, GeneralSource* dataSource);

	RtLbsType GetResidual(RtLbsType* position) const;

	template <typename T> bool operator ()(const T* const position, T* residual) const {
		T p1_x = T(m_x1);							/** @brief	�ο���P1 x����	*/
		T p1_y = T(m_y1);							/** @brief	�ο���P1 y����	*/
		T pi_x = T(m_xi);							/** @brief	���ݵ�Pi x����	*/
		T pi_y = T(m_yi);							/** @brief	���ݵ�Pi y����	*/
		T x = position[0];							/** @brief	Ԥ��� x����	*/
		T y = position[1];							/** @brief	Ԥ��� y����	*/
		RtLbsType powerDiff = m_refSource->CalculateExtraRDOALoss(position) - m_dataSource->CalculateExtraRDOALoss(position);
		residual[0] = T(powerDiff) - m_powerDiff;
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


#endif
