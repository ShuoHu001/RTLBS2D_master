#ifndef RTLBS_AOASOLVER
#define RTLBS_AOASOLVER

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "localization/generalsource.h"
#include "geometry/bbox2d.h"
#include "aoa_residual.h"
#include "configuration/localization/solvingconfig.h"
#include "math/vectorfunction.h"
#include "localization/lossfunction/lossfunction.h"

//#include ceres library ������ⷽ��
#include <glog/export.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ceres/ceres.h>



class AOASolver {
public:
	std::vector<GeneralSource*> m_gsData;					/** @brief	����Դ����	*/
	Point2D m_solution;										/** @brief	AOA��λ��*/

public:
	AOASolver();
	AOASolver(const AOASolver& solver);
	~AOASolver();
	AOASolver& operator = (const AOASolver& solver);
	void SetGeneralSource(const std::vector<GeneralSource*>& gsData);
	Point2D Solving_LS(const BBox2D& bbox, const Point2D& initPoint);																//��С���˷��������
	Point2D Solving_WLS(const BBox2D& bbox, const Point2D& initPoint);																//��Ȩ��С���˷��������
	Point2D Solving_TSWLS(const BBox2D& bbox, const Point2D& initPoint);															//������С���˷��������
	Point2D Solving_IRLS(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint);								//������Ȩ��С���˷��������
	Point2D Solving_WIRLS(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint);								//��ʼȨ�ص�����Ȩ��С���˷��������
	Point2D Solving(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint);										//�������
	void UpdateResidualWeight(const double* position, std::vector<AOAResidual>& aoaResiduals, double& aoaResidual_STD);				//���²в�Ȩ��
	double GetResidualSTD(const double* position, std::vector<AOAResidual>& aoaResiduals);
};


#endif
