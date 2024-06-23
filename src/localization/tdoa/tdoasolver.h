#ifndef RTLBS_TDOASOLVER
#define RTLBS_TDOASOLVER

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

class TDOASolver {
public:
	GeneralSource* m_refSource;								/** @brief	�ο�����Դ	*/
	std::vector<GeneralSource*> m_gsData;					/** @brief	����Դ����	*/
	Point2D m_solution;										/** @brief	TDOA��λ��	*/

public:
	TDOASolver();
	TDOASolver(const TDOASolver& solver);
	~TDOASolver();
	TDOASolver& operator = (const TDOASolver& solver);
	void SetGeneralSource(GeneralSource* refSource, const std::vector<GeneralSource*>& gsData);				//�趨����Դ������Ϣ
	void SetGeneralSource(GeneralSource* refSource, GeneralSource* gs1, GeneralSource* gs2);				//�趨����Դ������Ϣ-�����������Դ������
	RtLbsType Solving_LS(Point2D& outP);													//ʹ����С���˷������,�������һ�ε��������
};

#endif
