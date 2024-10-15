#include "aoa_toa_solver.h"

AOATOASolver::AOATOASolver()
{
}

AOATOASolver::AOATOASolver(const AOATOASolver& solver)
	: m_gsData(solver.m_gsData)
	, m_solution(solver.m_solution)
{
}

AOATOASolver::~AOATOASolver()
{
}

AOATOASolver& AOATOASolver::operator=(const AOATOASolver& solver)
{
	m_gsData = solver.m_gsData;
	m_solution = solver.m_solution;
	return *this;
}

void AOATOASolver::SetGeneralSource(const std::vector<GeneralSource*>& gsData)
{
	m_gsData = gsData;
}

Point2D AOATOASolver::Solving_WIRLS(int iterNum, RtLbsType tol, const Point2D& initPoint)
{
	RtLbsType position[2] = { initPoint.x, initPoint.y };				//��ʼλ�ù���
	RtLbsType prevPosition[2] = { 0,0 };								//ǰһ���ڵ��λ�ù���

	int dataNum = static_cast<int>(m_gsData.size());
	std::vector<AOAWLSResidual*> aoaResiduals(dataNum);
	std::vector<TOAWLSResidual*> toaResiduals(dataNum);

	//��ʼ���в�
	for (int i = 0; i < dataNum; ++i) {
		aoaResiduals[i] = new AOAWLSResidual(m_gsData[i]);
		toaResiduals[i] = new TOAWLSResidual(m_gsData[i]);
	}

	for (int i = 0; i < iterNum; ++i) {
		//��������
		ceres::Problem problem;

		//ָ�����ݼ�(�в��)
		for (int i = 0; i < dataNum; ++i) {
			ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOAWLSResidual, 1, 2>(aoaResiduals[i]);
			ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOAWLSResidual, 1, 2>(toaResiduals[i]);
			problem.AddResidualBlock(costFunc_AOA, nullptr, position);
			problem.AddResidualBlock(costFunc_TOA, nullptr, position);
		}

		//���������
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = true;
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
			RtLbsType res_aoa = aoaResiduals[i]->GetResidual(position);
			RtLbsType res_toa = toaResiduals[i]->GetResidual(position);
			RtLbsType cur_aoa_weight = aoaResiduals[i]->GetWeight() / (abs(res_aoa) + EPSILON);
			RtLbsType cur_toa_weight = toaResiduals[i]->GetWeight() / (abs(res_toa) + EPSILON);
			max_aoa_weight = std::max(max_aoa_weight, cur_aoa_weight);
			max_toa_weight = std::max(max_toa_weight, cur_toa_weight);
			aoaResiduals[i]->SetWeight(cur_aoa_weight);
			toaResiduals[i]->SetWeight(cur_toa_weight);
		}
		//��һ��Ȩ��
		for (int i = 0; i < dataNum; ++i) {
			RtLbsType cur_aoa_weight = aoaResiduals[i]->GetWeight() / max_aoa_weight;
			RtLbsType cur_toa_weight = toaResiduals[i]->GetWeight() / max_toa_weight;
			aoaResiduals[i]->SetWeight(cur_aoa_weight);
			toaResiduals[i]->SetWeight(cur_toa_weight);
		}
	}

	//�ͷ��ڴ�
	for (int i = 0; i < dataNum; ++i) {
		delete aoaResiduals[i];
		delete toaResiduals[i];
	}
	

	return { position[0], position[1] };
}
