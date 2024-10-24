#include "toa_solver.h"

TOASolver::TOASolver()
{
}

TOASolver::TOASolver(const TOASolver& solver)
	: m_gsData(solver.m_gsData)
	, m_solution(solver.m_solution)
{
}

TOASolver::~TOASolver()
{
}

TOASolver& TOASolver::operator=(const TOASolver& solver)
{
	m_gsData = solver.m_gsData;
	m_solution = solver.m_solution;
	return *this;
}

void TOASolver::SetGeneralSource(const std::vector<GeneralSource*>& gsData)
{
	m_gsData = gsData;
}

void TOASolver::SetGeneralSource(GeneralSource* gs1, GeneralSource* gs2)
{
	m_gsData.resize(2);
	m_gsData[0] = gs1;
	m_gsData[1] = gs2;
}

RtLbsType TOASolver::Solving_LS(Point2D& outP)
{
	//��������
	ceres::Problem problem;

	RtLbsType position[2] = { outP.x,outP.y };		//��ʼλ�ù���

	int dataNum = static_cast<int>(m_gsData.size());

	//ָ�����ݼ�(�в��)
	for (int i = 0; i < dataNum; ++i) {
		ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOALSResidual, 1, 2>(new TOALSResidual(m_gsData[i]));
		problem.AddResidualBlock(costFunc_TOA, nullptr, position);
	}

	//���������
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	options.logging_type = ceres::LoggingType::SILENT; // ��ֹ��־���

	//���
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	outP.x = position[0];
	outP.y = position[1];



	//���һ�ε���λ�����
	const ceres::IterationSummary& last_iteration = summary.iterations.back();


	return last_iteration.cost;
}

Point2D TOASolver::Solving_WIRLS(int iterNum, RtLbsType tol, const Point2D& initPoint)
{
	RtLbsType position[2] = { initPoint.x, initPoint.y };				//��ʼλ�ù���
	RtLbsType prevPosition[2] = { 0,0 };								//ǰһ���ڵ��λ�ù���

	int dataNum = static_cast<int>(m_gsData.size());
	std::vector<TOAWLSResidual> toaResiduals(dataNum);
	for (int i = 0; i < dataNum; ++i) {//��ʼ���в�
		toaResiduals[i].Init(m_gsData[i]);
	}

	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//����ǰһ��Ԥ�����ĸ�ֵ
		prevPosition[1] = position[1];

		//��������
		ceres::Problem problem;

		//ָ�����ݼ�(�в��)
		for (int j = 0; j < dataNum; ++j) {
			ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOAWLSResidual, 1, 2>(new TOAWLSResidual(toaResiduals[j]));
			problem.AddResidualBlock(costFunc_TOA, nullptr, position);
		}

		//���������
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = false;
		options.logging_type = ceres::LoggingType::SILENT; // ��ֹ��־���

		//��ʼ�������
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

		//���Ԥ�����������һ���Ż�������֮���������������tolerance�����break
		RtLbsType deltaDis = sqrt((prevPosition[0] - position[0]) * (prevPosition[0] - position[0]) + (prevPosition[1] - position[1]) * (prevPosition[1] - position[1]));
		if (deltaDis < tol) {
			break;
		}

		//Ȩ�ظ���
		RtLbsType max_aoa_weight = 0.0;
		RtLbsType max_toa_weight = 0.0;
		for (int i = 0; i < dataNum; ++i) {
			RtLbsType res_toa = toaResiduals[i].GetResidual(position);
			RtLbsType cur_toa_weight = toaResiduals[i].GetWeight() / (abs(res_toa) + EPSILON);
			max_toa_weight = std::max(max_toa_weight, cur_toa_weight);
			toaResiduals[i].SetWeight(cur_toa_weight);
		}
		//��һ��Ȩ��
		for (int i = 0; i < dataNum; ++i) {
			RtLbsType cur_toa_weight = toaResiduals[i].GetWeight() / max_toa_weight;
			toaResiduals[i].SetWeight(cur_toa_weight);
		}
	}



	return { position[0], position[1] };
}
