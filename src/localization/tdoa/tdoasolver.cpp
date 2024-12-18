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

RtLbsType TDOASolver::Solving_LS(const BBox2D& bbox, Point2D& outP)
{
	//��������
	ceres::Problem problem;

	RtLbsType position[2] = { outP.x,outP.y };		//��ʼλ�ù���

	//ָ�����ݼ�
	for (auto& curSource : m_gsData) {
		ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<TDOAResidual, 1, 2>(new TDOAResidual(m_refSource, curSource));			//������ʧ����
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

	//���
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	outP.x = position[0];
	outP.y = position[1];

	//���һ�ε���λ�����
	if (summary.iterations.size() > 0) {
		const ceres::IterationSummary& last_iteration = summary.iterations.back();
		return last_iteration.cost_change;
	}
	return 1e5;
}

Point2D TDOASolver::Solving_WLS(const BBox2D& bbox, const Point2D& initPoint)
{
	//��������
	ceres::Problem problem;

	RtLbsType position[2] = { initPoint.x, initPoint.y };		//��ʼλ�ù���

	//ָ�����ݼ�
	for (auto& curSource : m_gsData) {
		ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<TDOAResidual, 1, 2>(new TDOAResidual(m_refSource, curSource, curSource->m_weight));			//������ʧ����
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

	//���
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	return Point2D(position[0], position[1]);
}

Point2D TDOASolver::Solving_IRLS(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint)
{
	int iterNum = config.m_iterNum;
	double tol = config.m_tolerance;
	LOSSFUNCTIONTYPE lossType = config.m_lossType;

	RtLbsType position[2] = { initPoint.x, initPoint.y };				//��ʼλ�ù���
	RtLbsType prevPosition[2] = { 0,0 };								//ǰһ���ڵ��λ�ù���

	double tdoaResidual_STD = 0.01;											/** @brief	TDOA�в��׼��	*/

	int dataNum = static_cast<int>(m_gsData.size());
	std::vector<TDOAResidual> tdoaResiduals(dataNum);				/** @brief	TDOA �в�	*/

	//��ʼ���в�
	for (int i = 0; i < dataNum; ++i) {
		tdoaResiduals[i].Init(m_refSource, m_gsData[i]);
	}

	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//����ǰһ��Ԥ�����ĸ�ֵ
		prevPosition[1] = position[1];

		//��������
		ceres::Problem problem;

		ceres::LossFunction* tdoa_cost_function = custom_loss::CreateLossFunction(lossType, tdoaResidual_STD);

		//ָ�����ݼ�
		for (int j = 0; j < dataNum; ++j) {
			ceres::CostFunction* costFunc_TDOA = new ceres::AutoDiffCostFunction<TDOAResidual, 1, 2>(new TDOAResidual(tdoaResiduals[j]));
			problem.AddResidualBlock(costFunc_TDOA, tdoa_cost_function, position);

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

		UpdateResidualWeight(position, tdoaResiduals, tdoaResidual_STD);
	}

	return { position[0], position[1] };
}

Point2D TDOASolver::Solving_WIRLS(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint)
{
	int iterNum = config.m_iterNum;
	double tol = config.m_tolerance;
	LOSSFUNCTIONTYPE lossType = config.m_lossType;

	RtLbsType position[2] = { initPoint.x, initPoint.y };				//��ʼλ�ù���
	RtLbsType prevPosition[2] = { 0,0 };								//ǰһ���ڵ��λ�ù���

	double tdoaResidual_STD = 0.01;											/** @brief	TDOA�в��׼��	*/

	int dataNum = static_cast<int>(m_gsData.size());
	std::vector<TDOAResidual> tdoaResiduals(dataNum);				/** @brief	TDOA �в�	*/

	//��ʼ���в�
	for (int i = 0; i < dataNum; ++i) {
		tdoaResiduals[i].Init(m_refSource, m_gsData[i], m_gsData[i]->m_weight);
	}

	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//����ǰһ��Ԥ�����ĸ�ֵ
		prevPosition[1] = position[1];

		//��������
		ceres::Problem problem;

		ceres::LossFunction* tdoa_cost_function = custom_loss::CreateLossFunction(lossType, tdoaResidual_STD);

		//ָ�����ݼ�
		for (int j = 0; j < dataNum; ++j) {
			ceres::CostFunction* costFunc_TDOA = new ceres::AutoDiffCostFunction<TDOAResidual, 1, 2>(new TDOAResidual(tdoaResiduals[j]));
			problem.AddResidualBlock(costFunc_TDOA, tdoa_cost_function, position);

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

		UpdateResidualWeight(position, tdoaResiduals, tdoaResidual_STD);
	}

	return { position[0], position[1] };
}

Point2D TDOASolver::Solving(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint)
{
	const SOLVINGMODE mode = config.m_solvingMode;
	Point2D solution = initPoint;
	if (mode == SOLVING_LS) {
		Solving_LS(bbox, solution);
	}
	else if (mode == SOLVING_WLS) {
		solution = Solving_WLS(bbox, initPoint);
	}
	else if (mode == SOLVING_IRLS) {
		solution = Solving_IRLS(config, bbox, weightFactor, initPoint);
	}
	else if (mode == SOLVING_WIRLS) {
		solution = Solving_WIRLS(config, bbox, weightFactor, initPoint);
	}
	return solution;
}

void TDOASolver::UpdateResidualWeight(const double* position, std::vector<TDOAResidual>& tdoaResiduals, double& tdoaResidual_STD)
{
	double max_tdoa_weight = 0.0;
	std::vector<double> r_tdoas;
	for (auto& curTDOAResidual : tdoaResiduals) {
		double res_tdoa = curTDOAResidual.GetResidual(position);
		double cur_tdoa_weight = curTDOAResidual.GetWeight() / (abs(res_tdoa) + EPSILON);
		max_tdoa_weight = std::max(max_tdoa_weight, cur_tdoa_weight);
		curTDOAResidual.SetWeight(cur_tdoa_weight);
		r_tdoas.push_back(res_tdoa);
	}
	//��һ��Ȩ��
	for (auto& curTDOAResidual : tdoaResiduals) {
		double cur_tdoa_weight = curTDOAResidual.GetWeight() / max_tdoa_weight;
		curTDOAResidual.SetWeight(cur_tdoa_weight);
	}

	//���²в��׼��
	tdoaResidual_STD = vectoroperator::CalculateStandardDeviation(r_tdoas);
}
