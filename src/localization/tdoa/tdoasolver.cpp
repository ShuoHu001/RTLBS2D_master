#include "tdoasolver.h"

TDOASolver::TDOASolver()
	: m_refSource(nullptr)
{
}

TDOASolver::TDOASolver(const TDOASolver& solver)
	: m_refSource(solver.m_refSource)
	, m_gsData(solver.m_gsData)
	, m_solution(solver.m_solution)
{
}

TDOASolver::~TDOASolver()
{
}

TDOASolver& TDOASolver::operator=(const TDOASolver& solver)
{
	m_refSource = solver.m_refSource;
	m_gsData = solver.m_gsData;
	m_solution = solver.m_solution;
	return *this;
}

void TDOASolver::SetGeneralSource(GeneralSource* refSource, const std::vector<GeneralSource*>& gsData)
{
	m_refSource = refSource;
	m_gsData = gsData;
	if (m_gsData.size() < 2) {
		LOG_ERROR << "TDOASolver: not satisfied TDOA algorithm min equation set size, min size is 2." << ENDL;
	}
}

void TDOASolver::SetGeneralSource(GeneralSource* refSource, GeneralSource* gs1, GeneralSource* gs2)
{
	m_refSource = refSource;
	m_gsData.resize(2);
	m_gsData[0] = gs1;
	m_gsData[1] = gs2;
}

RtLbsType TDOASolver::Solving_LS(Point2D& outP)
{
	//��������
	ceres::Problem problem;

	RtLbsType position[2] = { 0,0 };		//��ʼλ�ù���

	//ָ�����ݼ�
	for (auto& curSource : m_gsData) {
		ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<TDOALSResidual, 1, 2>(new TDOALSResidual(m_refSource, curSource));			//������ʧ����
		problem.AddResidualBlock(costFunction, nullptr, position);
	}

	//���������
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;

	//���
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	outP.x = position[0];
	outP.y = position[1];

	//���һ�ε���λ�����
	const ceres::IterationSummary& last_iteration = summary.iterations.back();
	return last_iteration.cost;
}
