#ifndef RTLBS_AOATDOASOLVER
#define RTLBS_AOATDOASOLVER
#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "localization/generalsource.h"
#include "localization/aoa/aoa_residual.h"
#include "tdoa_residual.h"
#include "configuration/localization/solvingconfig.h"
#include "math/vectorfunction.h"
#include "localization/lossfunction/lossfunction.h"

//#include ceres library ������ⷽ��
#include <glog/export.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ceres/ceres.h>

class AOATDOASolver {
public:
	bool m_isValid;												/** @brief	�Ƿ���Ч	*/
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
	double Solving_LS(const BBox2D& bbox, Point2D& outP);																	//LS������ⷽ��
	Point2D Solving_WLS(const BBox2D& bbox, const Point2D& initPoint);																	//WLS������ⷽ��
	Point2D Solving_TSWLS(const BBox2D& bbox, const Point2D& initPoint);		//TSWLS ��ⷽ��
	Point2D Solving_IRLS(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint);		//IRLS������ⷽ��
	Point2D Solving_WIRLS(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint);	//WIRLS������ⷽ��
	Point2D Solving(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint);		//��ⷽ��
	void UpdateResidualWeight(const double* position, const WeightFactor& weightFactor, std::vector<AOAResidual>& aoaResiduals, std::vector<TDOAResidual>& tdoaResiduals, double& aoaResidual_STD, double& tdoaResidual_STD);			//���ݲв�����²в�Ȩ��
	void CalculateResidualSTD(const double* position, const std::vector<AOAResidual>& aoaResiduals, const std::vector<TDOAResidual>& tdoaResiduals, double& aoaResidual_STD, double& tdoaResidual_STD);			//���ݲв�����²в�Ȩ��
};


#endif
