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

//TDOA WLS ��λ�в���ʽ
class TDOAResidual {
private:
	RtLbsType m_x1;						/** @brief	�ο� x ����	*/
	RtLbsType m_y1;						/** @brief	�ο� y ����	*/
	RtLbsType m_xi;						/** @brief	x ����	*/
	RtLbsType m_yi;						/** @brief	y ����	*/
	RtLbsType m_timeDiff;				/** @brief	��������ʱ���	*/
	RtLbsType m_weight;					/** @brief	���̵�Ȩ��	*/

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
		T p1_x = T(m_x1);							/** @brief	�ο���P1 x����	*/
		T p1_y = T(m_y1);							/** @brief	�ο���P1 y����	*/
		T pi_x = T(m_xi);							/** @brief	���ݵ�Pi x����	*/
		T pi_y = T(m_yi);							/** @brief	���ݵ�Pi y����	*/
		T x = position[0];							/** @brief	Ԥ��� x����	*/
		T y = position[1];							/** @brief	Ԥ��� y����	*/
		T d1 = ceres::sqrt((x - p1_x) * (x - p1_x) + (y - p1_y) * (y - p1_y));
		T di = ceres::sqrt((x - pi_x) * (x - pi_x) + (y - pi_y) * (y - pi_y));
		residual[0] = ((di - d1) / T(LIGHT_VELOCITY_AIR) - T(m_timeDiff)) *  T(m_weight * 1e9);				//ns ʱ���
		return true;
	}

};

#endif
