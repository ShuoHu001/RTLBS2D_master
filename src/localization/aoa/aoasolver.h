#ifndef RTLBS_AOASOLVER
#define RTLBS_AOASOLVER

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
	void Solving_LS();																//��С���˷��������
	void Solving_WLS();																//��Ȩ��С���˷��������
	Point2D Solving_WIRLS(int iterNum, RtLbsType tol, const Point2D& initPoint);				//��ʼȨ�ص�����Ȩ��С���˷��������
	void Solving_IRLS(int iterNum, RtLbsType tol);									//������Ȩ��С���˷��������
	void Solving_ElaspNet(RtLbsType lamda1, RtLbsType lamda2);						//���������������L1����L2����
};


#endif
