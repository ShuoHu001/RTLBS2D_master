#ifndef RTLBS_AOA_RESIDUAL_H
#define RTLBS_AOA_RESIDUAL_H

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "localization/generalsource.h"
#include "localization/lbsresidual.h"

//#include ceres library ������ⷽ��
#include <glog/export.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ceres/ceres.h>

//AOA WLS ��λ�в���ʽ
class AOAResidual {
private:
	RtLbsType m_x;		/** @brief	x ����	*/
	RtLbsType m_y;		/** @brief	y ����	*/
	RtLbsType m_phi;		/** @brief	�������ĽǶ�ֵ	*/
	RtLbsType m_weight;		/** @brief	���̵�Ȩ��	*/
	RtLbsType m_cosPhi;		/** @brief	�Ƕ�ֵ����	*/
	RtLbsType m_sinPhi;		/** @brief	�Ƕ�ֵ����	*/

public:
	AOAResidual();
	AOAResidual(const GeneralSource* source, RtLbsType weight = 1.0);
	AOAResidual(const AOAResidual& residual);
	~AOAResidual();
	AOAResidual& operator = (const AOAResidual& residual);

	void Init(const GeneralSource* source, RtLbsType weight = 1.0);						//�ӹ���Դ��ʼ��
	double GetResidual(const double* position) const;									//����в�
	RtLbsType GetWeight() const;														//���Ȩ��
	void SetWeight(RtLbsType weight);													//����Ȩ��
	
	//����в���ʽ��������ceres�Ż���
	template <typename T> bool operator()(const T* const position, T* residual) const {
		T dx = position[0] - T(m_x);
		T dy = position[1] - T(m_y);
		residual[0] = (dx * T(m_sinPhi) - dy * T(m_cosPhi)) / T(m_weight);
		return true;
	}
};

#endif
