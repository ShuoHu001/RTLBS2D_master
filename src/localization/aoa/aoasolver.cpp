#include "aoasolver.h"

AOASolver::AOASolver()
{
}

AOASolver::AOASolver(const AOASolver& solver)
	: m_gsData(solver.m_gsData)
	, m_solution(solver.m_solution)
{
}

AOASolver::~AOASolver()
{
}

AOASolver& AOASolver::operator=(const AOASolver& solver)
{
	m_gsData = solver.m_gsData;
	m_solution = solver.m_solution;
	return *this;
}

void AOASolver::SetGeneralSource(const std::vector<GeneralSource*>& gsData)
{
	m_gsData = gsData;
}

void AOASolver::Solving_WLS()
{
	//��������
	ceres::Problem problem;

	RtLbsType position[2] = { 0,0 };		//��ʼλ�ù���

	//ָ�����ݼ�
	for (auto it = m_gsData.begin(); it != m_gsData.end(); ++it) {
		const GeneralSource* curSource = *it;
		ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAWLSResidual, 1, 2>(new AOAWLSResidual(curSource));				//������ʧ����
		problem.AddResidualBlock(costFunction, nullptr, position);
	}

	//���������
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	//��ʼ�������
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	//������
	std::cout << summary.FullReport() << std::endl;
	std::cout << "Estimate non-corporate source coordinate is (" << position[0] << "," << position[1] << ")." << std::endl;
}

void AOASolver::Solving_IRLS(int iterNum, RtLbsType tol)
{
	RtLbsType position[2] = { 0,0 };								//��ʼλ�ù���
	RtLbsType prevPosition[2] = { 0,0 };							//ǰһ���ڵ��λ�ù���

	int dataNum = static_cast<int>(m_gsData.size());				//��������
	std::vector<AOAWLSResidual> aoaResiduals(dataNum);				//����aoa�в����
	for (int i = 0; i < dataNum; ++i) {
		aoaResiduals[i].Init(m_gsData[i]);
	}


	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//����ǰһ��Ԥ�����ĸ�ֵ
		prevPosition[1] = position[1];

		//��������
		ceres::Problem problem;

		//�ƶ����ݼ�
		for (int i = 0; i < dataNum; ++i) {
			ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAWLSResidual, 1, 2>(new AOAWLSResidual(aoaResiduals[i]));				//������ʧ����
			problem.AddResidualBlock(costFunction, nullptr, position);
		}

		//���������
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = true;

		//��ʼ�������
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

		//������
		std::cout << summary.FullReport() << std::endl;
		std::cout << "Estimate non-corporate source coordinate is (" << position[0] << "," << position[1] << ")." << std::endl;


		//���Ԥ�����������һ���Ż�������֮���������������tolerance�����break
		RtLbsType deltaDis = sqrt((prevPosition[0] - position[0]) * (prevPosition[0] - position[0]) + (prevPosition[1] - position[1]) * (prevPosition[1] - position[1]));
		if (deltaDis < tol) {
			break;
		}

		//Ȩ�ظ���
		std::vector<RtLbsType> weights(dataNum);								//Ȩ��ϵ��
		for (int i = 0; i < dataNum; ++i) {
			RtLbsType res = aoaResiduals[i].GetResidual(position);
			weights[i] = 1.0 / (res * res + 1e-6);								//Ȩ��
		}

		RtLbsType maxWeight = *std::max_element(weights.begin(), weights.end());
		for (int i = 0; i < dataNum; ++i) {
			RtLbsType cur_w = aoaResiduals[i].GetWeight() * (weights[i] / maxWeight);
			aoaResiduals[i].SetWeight(cur_w);
		}


	}
}

void AOASolver::Solving_ElaspNet(RtLbsType lamda1, RtLbsType lamda2)
{
}
