#ifndef RTLBS_LBSRESIDUAL
#define RTLBS_LBSRESIDUAL

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "generalsource.h"


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


#endif
