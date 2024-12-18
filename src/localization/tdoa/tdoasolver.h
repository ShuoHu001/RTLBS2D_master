#ifndef RTLBS_TDOASOLVER
#define RTLBS_TDOASOLVER

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "localization/generalsource.h"
#include "localization/aoa_tdoa/tdoa_residual.h"
#include "configuration/localization/solvingconfig.h"
#include "math/vectorfunction.h"
#include "localization/lossfunction/lossfunction.h"

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
	RtLbsType Solving_LS(const BBox2D& bbox, Point2D& outP);													//ʹ����С���˷������,�������һ�ε��������
	Point2D Solving_WLS(const BBox2D& bbox, const Point2D& initPoint);																	//WLS������ⷽ��
	Point2D Solving_IRLS(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint);		//IRLS������ⷽ��
	Point2D Solving_WIRLS(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint);	//WIRLS������ⷽ��
	Point2D Solving(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint);		//��ⷽ��
	void UpdateResidualWeight(const double* position, std::vector<TDOAResidual>& tdoaResiduals, double& tdoaResidual_STD);		//���²в�Ȩ��
};

#endif
