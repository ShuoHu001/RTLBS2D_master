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

void AOASolver::Solving_LS()
{
	//��������
	ceres::Problem problem;

	RtLbsType position[2] = { 0,0 };		//��ʼλ�ù���

	//ָ�����ݼ�
	for (auto it = m_gsData.begin(); it != m_gsData.end(); ++it) {
		const GeneralSource* curSource = *it;
		ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOALSResidual, 1, 2>(new AOALSResidual(curSource));				//������ʧ����
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
	std::cout << "Estimate non-corporate source coordinate is (" << position[0] << "," << position[1] << ")." << std::endl;
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
	options.minimizer_progress_to_stdout = false;

	//��ʼ�������
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	//������
	std::cout << "Estimate non-corporate source coordinate is (" << position[0] << "," << position[1] << ")." << std::endl;
}

Point2D AOASolver::Solving_WIRLS(int iterNum, RtLbsType tol, const Point2D& initPoint)
{
	RtLbsType position[2] = { initPoint.x, initPoint.y };								//��ʼλ�ù���
	RtLbsType prevPosition[2] = { 0,0 };							//ǰһ���ڵ��λ�ù���

	int aoaDataNum = static_cast<int>(m_gsData.size());				//��������
	std::vector<AOAWLSResidual> aoaResiduals(aoaDataNum);				//����aoa�в����
	for (int i = 0; i < aoaDataNum; ++i) {
		aoaResiduals[i].Init(m_gsData[i]);
	}

	//int rdoaDataNum = aoaDataNum - 1;								//RDOA �в�������
	//std::vector<RDOAWLSResidual> rdoaResiduals(rdoaDataNum);
	//for (int i = 0; i < rdoaDataNum; ++i) {
	//	rdoaResiduals[i].Init(m_gsData[0], m_gsData[i + 1]);
	//}
	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//����ǰһ��Ԥ�����ĸ�ֵ
		prevPosition[1] = position[1];

		//��������
		ceres::Problem problem;


		//�ƶ����ݼ�

		for (int i = 0; i < aoaDataNum; ++i) {
			ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAWLSResidual, 1, 2>(new AOAWLSResidual(aoaResiduals[i]));				//������ʧ����
			problem.AddResidualBlock(costFunction, nullptr, position);
		}

		//for (int i = 0; i < rdoaDataNum; ++i) {
		//	ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<RDOAWLSResidual, 1, 2>(new RDOAWLSResidual(rdoaResiduals[i]));				//������ʧ����
		//	problem.AddResidualBlock(costFunction, nullptr, position);
		//}

		//���������
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_SCHUR;
		options.minimizer_progress_to_stdout = true;
		options.logging_type = ceres::LoggingType::SILENT; // ��ֹ��־���

		//��ʼ�������
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

		//������
		//std::cout << "Estimate non-corporate source coordinate is (" << position[0] << "," << position[1] << ")." << std::endl;


		//���Ԥ�����������һ���Ż�������֮���������������tolerance�����break
		RtLbsType deltaDis = sqrt((prevPosition[0] - position[0]) * (prevPosition[0] - position[0]) + (prevPosition[1] - position[1]) * (prevPosition[1] - position[1]));
		if (deltaDis < tol) {
			break;
		}

		//Ȩ�ظ���
		std::vector<RtLbsType> weights(aoaDataNum);								//Ȩ��ϵ��
		for (int i = 0; i < aoaDataNum; ++i) {
			RtLbsType res = aoaResiduals[i].GetResidual(position);
			weights[i] = aoaResiduals[i].GetWeight() / (res * res + 1e-6);								//Ȩ��
			aoaResiduals[i].SetWeight(weights[i]);
		}

		//for (int i = 0; i < rdoaDataNum; ++i) {
		//	RtLbsType res = rdoaResiduals[i].GetResidual(position);
		//	rdoaResiduals[i].SetWeight(1.0 / (res * res + 1e-6));
		//}

	}
	return Point2D(position[0], position[1]);
}

void AOASolver::Solving_IRLS(int iterNum, RtLbsType tol)
{
	RtLbsType position[2] = { 0,0 };								//��ʼλ�ù���
	RtLbsType prevPosition[2] = { 0,0 };							//ǰһ���ڵ��λ�ù���

	int aoaDataNum = static_cast<int>(m_gsData.size());				//AOA �в���������
	std::vector<AOAWLSResidual> aoaResiduals(aoaDataNum);				//����aoa�в����
	for (int i = 0; i < aoaDataNum; ++i) {
		aoaResiduals[i].Init(m_gsData[i]);
		//aoaResiduals[i].SetWeight(1.0);								//�趨��ʼȨ��Ϊ1
	}

	int rdoaDataNum = aoaDataNum - 1;								//RDOA �в�������
	std::vector<RDOAWLSResidual> rdoaResiduals(rdoaDataNum);
	for (int i = 0; i < rdoaDataNum; ++i) {
		rdoaResiduals[i].Init(m_gsData[0], m_gsData[i + 1]);
	}


	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//����ǰһ��Ԥ�����ĸ�ֵ
		prevPosition[1] = position[1];

		//��������
		ceres::Problem problem;

		//�ƶ����ݼ�
		for (int i = 0; i < aoaDataNum; ++i) {
			ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAWLSResidual, 1, 2>(new AOAWLSResidual(aoaResiduals[i]));				//������ʧ����
			problem.AddResidualBlock(costFunction, nullptr, position);
		}

		for (int i = 0; i < rdoaDataNum; ++i) {
			ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<RDOAWLSResidual, 1, 2>(new RDOAWLSResidual(rdoaResiduals[i]));				//������ʧ����
			problem.AddResidualBlock(costFunction, nullptr, position);
		}

		//���������
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = false;

		//��ʼ�������
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

		//������
		std::cout << "Estimate non-corporate source coordinate is (" << position[0] << "," << position[1] << ")." << std::endl;


		//���Ԥ�����������һ���Ż�������֮���������������tolerance�����break
		RtLbsType deltaDis = sqrt((prevPosition[0] - position[0]) * (prevPosition[0] - position[0]) + (prevPosition[1] - position[1]) * (prevPosition[1] - position[1]));
		if (deltaDis < tol) {
			break;
		}

		//Ȩ�ظ���
		for (int i = 0; i < aoaDataNum; ++i) {
			RtLbsType res = aoaResiduals[i].GetResidual(position);
			aoaResiduals[i].SetWeight(1.0 / (res * res + 1e-6));
		}
		for (int i = 0; i < rdoaDataNum; ++i) {
			RtLbsType res = rdoaResiduals[i].GetResidual(position);
			rdoaResiduals[i].SetWeight(1.0 / (res * res + 1e-6));
		}
	}
}

void AOASolver::Solving_ElaspNet(RtLbsType lamda1, RtLbsType lamda2)
{
}
