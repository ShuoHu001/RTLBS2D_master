#ifndef RTLBS_AOATDOASOLVER
#define RTLBS_AOATDOASOLVER
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

class AOATDOASolver {
public:
	GeneralSource* m_refSource;									/** @brief	�ο�����Դ	*/
	std::vector<GeneralSource*> m_gsData;						/** @brief	����Դ	*/
	Point2D m_solution;											/** @brief	AOATDOA��λ��	*/

public:
	AOATDOASolver();
	AOATDOASolver(const AOATDOASolver& solver);
	~AOATDOASolver();
	AOATDOASolver& operator = (const AOATDOASolver& solver);
	void SetGeneralSource(GeneralSource* refSource, const std::vector<GeneralSource*>& gsData);
	void SetGeneralSource(GeneralSource* refSource, GeneralSource* gs1, GeneralSource* gs2);
	RtLbsType Solving_LS(Point2D& outP);																	//LS������ⷽ��
	Point2D Solving_WIRLS(int iterNum, RtLbsType tol, const Point2D& initPoint);
};


#endif
