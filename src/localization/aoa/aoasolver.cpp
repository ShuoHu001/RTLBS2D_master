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

Point2D AOASolver::Solving_LS(const BBox2D& bbox, const Point2D& initPoint)
{
	//��������
	ceres::Problem problem;

	RtLbsType position[2] = { initPoint.x, initPoint.y };		//��ʼλ�ù���

	//ָ�����ݼ�
	for (auto it = m_gsData.begin(); it != m_gsData.end(); ++it) {
		const GeneralSource* curSource = *it;
		ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(curSource));				//������ʧ����
		problem.AddResidualBlock(costFunction, nullptr, position);
	}

	//���ñ߽�Լ��
	problem.SetParameterLowerBound(position, 0, bbox.m_min.x);
	problem.SetParameterUpperBound(position, 0, bbox.m_max.x);
	problem.SetParameterLowerBound(position, 1, bbox.m_min.y);
	problem.SetParameterUpperBound(position, 1, bbox.m_max.y);

	//���������
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	//��ʼ�������
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);



	//������
	return Point2D(position[0], position[1]);
}

Point2D AOASolver::Solving_WLS(const BBox2D& bbox, const Point2D& initPoint)
{
	//��������
	ceres::Problem problem;

	RtLbsType position[2] = { initPoint.x, initPoint.y };		//��ʼλ�ù���

	//ָ�����ݼ�
	for (auto it = m_gsData.begin(); it != m_gsData.end(); ++it) {
		const GeneralSource* curSource = *it;
		ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(curSource, curSource->m_weight));				//������ʧ����
		problem.AddResidualBlock(costFunction, nullptr, position);
	}

	//���ñ߽�Լ��
	problem.SetParameterLowerBound(position, 0, bbox.m_min.x);
	problem.SetParameterUpperBound(position, 0, bbox.m_max.x);
	problem.SetParameterLowerBound(position, 1, bbox.m_min.y);
	problem.SetParameterUpperBound(position, 1, bbox.m_max.y);

	//���������
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;

	//��ʼ�������
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	//������
	return Point2D(position[0], position[1]);
}

Point2D AOASolver::Solving_TSWLS(const BBox2D& bbox, const Point2D& initPoint)
{
	Point2D tsInitPoint = Solving_LS(bbox, initPoint);

	
	//��������
	ceres::Problem problem;

	RtLbsType position[2] = { tsInitPoint.x, tsInitPoint.y };		//��ʼλ�ù���

	//ָ�����ݼ�
	for (auto it = m_gsData.begin(); it != m_gsData.end(); ++it) {
		const GeneralSource* curSource = *it;
		ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(curSource));				//������ʧ����
		problem.AddResidualBlock(costFunction, nullptr, position);
	}

	//���ñ߽�Լ��
	problem.SetParameterLowerBound(position, 0, bbox.m_min.x);
	problem.SetParameterUpperBound(position, 0, bbox.m_max.x);
	problem.SetParameterLowerBound(position, 1, bbox.m_min.y);
	problem.SetParameterUpperBound(position, 1, bbox.m_max.y);

	//���������
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;

	//��ʼ�������
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	//������
	return Point2D(position[0], position[1]);
}

Point2D AOASolver::Solving_IRLS(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint)
{
	int iterNum = config.m_iterNum;
	double tol = config.m_tolerance;
	LOSSFUNCTIONTYPE lossType = config.m_lossType;

	RtLbsType position[2] = { initPoint.x,initPoint.y };								//��ʼλ�ù���
	RtLbsType prevPosition[2] = { 0,0 };							//ǰһ���ڵ��λ�ù���

	double aoaResidual_STD = 0.01;											/** @brief	AOA�в��׼��	*/

	int aoaDataNum = static_cast<int>(m_gsData.size());				//AOA �в���������
	std::vector<AOAResidual> aoaResiduals(aoaDataNum);				//����aoa�в����
	for (int i = 0; i < aoaDataNum; ++i) {
		aoaResiduals[i].Init(m_gsData[i]);
	}


	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//����ǰһ��Ԥ�����ĸ�ֵ
		prevPosition[1] = position[1];

		//��������
		ceres::Problem problem;

		ceres::LossFunction* aoa_cost_function = custom_loss::CreateLossFunction(lossType, aoaResidual_STD);

		//�ƶ����ݼ�
		for (int j = 0; j < aoaDataNum; ++j) {
			ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(aoaResiduals[j]));				//������ʧ����
			problem.AddResidualBlock(costFunction, aoa_cost_function, position);
		}

		//���ñ߽�Լ��
		problem.SetParameterLowerBound(position, 0, bbox.m_min.x);
		problem.SetParameterUpperBound(position, 0, bbox.m_max.x);
		problem.SetParameterLowerBound(position, 1, bbox.m_min.y);
		problem.SetParameterUpperBound(position, 1, bbox.m_max.y);

		//���������
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = false;

		//��ʼ�������
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);


		//���Ԥ�����������һ���Ż�������֮���������������tolerance�����break
		RtLbsType deltaDis = sqrt((prevPosition[0] - position[0]) * (prevPosition[0] - position[0]) + (prevPosition[1] - position[1]) * (prevPosition[1] - position[1]));
		if (deltaDis < tol) {
			break;
		}

		UpdateResidualWeight(position, aoaResiduals, aoaResidual_STD);
	}
	return Point2D(position[0], position[1]);
}

Point2D AOASolver::Solving_WIRLS(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint)
{
	int iterNum = config.m_iterNum;
	double tol = config.m_tolerance;
	LOSSFUNCTIONTYPE lossType = config.m_lossType;

	RtLbsType position[2] = { initPoint.x, initPoint.y };								//��ʼλ�ù���
	RtLbsType prevPosition[2] = { 0,0 };							//ǰһ���ڵ��λ�ù���

	double aoaResidual_STD = 0.01;											/** @brief	AOA�в��׼��	*/

	int aoaDataNum = static_cast<int>(m_gsData.size());				//��������
	std::vector<AOAResidual> aoaResiduals(aoaDataNum);				//����aoa�в����
	for (int i = 0; i < aoaDataNum; ++i) {
		aoaResiduals[i].Init(m_gsData[i], m_gsData[i]->m_weight);
	}

	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//����ǰһ��Ԥ�����ĸ�ֵ
		prevPosition[1] = position[1];

		//��������
		ceres::Problem problem;

		ceres::LossFunction* aoa_cost_function = custom_loss::CreateLossFunction(lossType, aoaResidual_STD);

		//�ƶ����ݼ�
		for (int j = 0; j < aoaDataNum; ++j) {
			ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(aoaResiduals[j]));				//������ʧ����
			problem.AddResidualBlock(costFunction, aoa_cost_function, position);
		}

		//���ñ߽�Լ��
		problem.SetParameterLowerBound(position, 0, bbox.m_min.x);
		problem.SetParameterUpperBound(position, 0, bbox.m_max.x);
		problem.SetParameterLowerBound(position, 1, bbox.m_min.y);
		problem.SetParameterUpperBound(position, 1, bbox.m_max.y);

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

		UpdateResidualWeight(position, aoaResiduals, aoaResidual_STD);

	}
	return Point2D(position[0], position[1]);
}

Point2D AOASolver::Solving(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint)
{
	const SOLVINGMODE mode = config.m_solvingMode;
	Point2D solution = initPoint;
	if (mode == SOLVING_LS) {
		solution = Solving_LS(bbox, initPoint);
	}
	else if (mode == SOLVING_WLS) {
		solution = Solving_WLS(bbox, initPoint);
	}
	else if (mode == SOLVING_TSWLS) {
		solution = Solving_TSWLS(bbox, initPoint);
	}
	else if (mode == SOLVING_IRLS) {
		solution = Solving_IRLS(config, bbox, initPoint);
	}
	else if (mode == SOLVING_WIRLS) {
		solution = Solving_WIRLS(config, bbox, initPoint);
	}
	return solution;
}

void AOASolver::UpdateResidualWeight(const double* position, std::vector<AOAResidual>& aoaResiduals, double& aoaResidual_STD)
{

	//Ȩ�ظ���
	double max_aoa_weight = 0.0;
	std::vector<double> r_aoas;
	for (auto& curAOAResidual: aoaResiduals) {
		double res = curAOAResidual.GetResidual(position);
		double cur_aoa_weight = curAOAResidual.GetWeight() / (abs(res) + EPSILON);								//Ȩ��
		max_aoa_weight = std::max(max_aoa_weight, cur_aoa_weight);
		curAOAResidual.SetWeight(cur_aoa_weight);
		r_aoas.push_back(res);
	}
	//��һ��Ȩ��
	for (auto& curAOAResidual : aoaResiduals) {
		double cur_aoa_weight = curAOAResidual.GetWeight()/max_aoa_weight;
		curAOAResidual.SetWeight(cur_aoa_weight);
	}

	//���²в��׼��
	aoaResidual_STD = vectoroperator::CalculateStandardDeviation(r_aoas);
}

double AOASolver::GetResidualSTD(const double* position, std::vector<AOAResidual>& aoaResiduals)
{
	std::vector<double> r_aoas;
	for (auto& curAOAResidual : aoaResiduals) {
		double res = curAOAResidual.GetResidual(position);
		r_aoas.push_back(res);
	}
	return vectoroperator::CalculateStandardDeviation(r_aoas);
}
