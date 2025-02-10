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

RtLbsType TOASolver::Solving_LS(const BBox2D& bbox, Point2D& outP)
{
	//��������
	ceres::Problem problem;

	RtLbsType position[2] = { outP.x, outP.y };		//��ʼλ�ù���

	int dataNum = static_cast<int>(m_gsData.size());

	//ָ�����ݼ�(�в��)
	for (int i = 0; i < dataNum; ++i) {
		ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOAResidual, 1, 2>(new TOAResidual(m_gsData[i]));
		problem.AddResidualBlock(costFunc_TOA, nullptr, position);
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
	options.logging_type = ceres::LoggingType::SILENT; // ��ֹ��־���

	//���
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	outP.x = position[0];
	outP.y = position[1];



	//���һ�ε���λ�����
	if (summary.iterations.size() != 0) {
		const ceres::IterationSummary& last_iteration = summary.iterations.back();
		return last_iteration.cost;
	}
	return 1e5;
}

Point2D TOASolver::Solving_WLS(const BBox2D& bbox, const Point2D& initPoint)
{
	//��������
	ceres::Problem problem;

	RtLbsType position[2] = { 0, 0 };		//��ʼλ�ù���

	int dataNum = static_cast<int>(m_gsData.size());

	//ָ�����ݼ�(�в��)
	for (int i = 0; i < dataNum; ++i) {
		ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOAResidual, 1, 2>(new TOAResidual(m_gsData[i], m_gsData[i]->m_weight));
		problem.AddResidualBlock(costFunc_TOA, nullptr, position);
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
	options.logging_type = ceres::LoggingType::SILENT; // ��ֹ��־���

	//���
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	return Point2D(position[0], position[1]);
}

Point2D TOASolver::Solving_TSWLS(const BBox2D& bbox)
{
	//ʹ��������Ȩ��С���˷���
	Point2D initPoint(10, 10);
	Solving_LS(bbox, initPoint);
	//���ȵ�һ��ʹ��LS���м����ʼλ��
	//����Ȩ�أ����ղв�Ĵ�С���е���

	RtLbsType position[2] = { initPoint.x, initPoint.y };		//��ʼλ�ù���

	int dataNum = static_cast<int>(m_gsData.size());

	std::vector<TOAResidual> toaResiduals(dataNum);
	for (int i = 0; i < dataNum; ++i) {//��ʼ���в�
		toaResiduals[i].Init(m_gsData[i]);
		double res = toaResiduals[i].GetResidual(position);
		double weight = 1.0 / (res * res + EPSILON);
		toaResiduals[i].SetWeight(weight);
	}

	//��������
	ceres::Problem problem;
	//ָ�����ݼ�(�в��)
	for (int i = 0; i < dataNum; ++i) {
		ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOAResidual, 1, 2>(new TOAResidual(toaResiduals[i]));
		problem.AddResidualBlock(costFunc_TOA, nullptr, position);
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
	options.logging_type = ceres::LoggingType::SILENT; // ��ֹ��־���

	//���
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	return Point2D(position[0], position[1]);

}

Point2D TOASolver::Solving_IRLS(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint)
{
	int iterNum = config.m_iterNum;
	double tol = config.m_tolerance;
	LOSSFUNCTIONTYPE lossType = config.m_lossType;
	


	RtLbsType position[2] = { 10.0, 10.0 };				//��ʼλ�ù���,IRLS���ӳ�ֵ��ֵ
	RtLbsType prevPosition[2] = { 0,0 };								//ǰһ���ڵ��λ�ù���

	double toaResidual_STD = 0.01;											/** @brief	AOA�в��׼��	*/

	int dataNum = static_cast<int>(m_gsData.size());
	std::vector<TOAResidual> toaResiduals(dataNum);
	for (int i = 0; i < dataNum; ++i) {//��ʼ���в�
		toaResiduals[i].Init(m_gsData[i]);
	}

	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//����ǰһ��Ԥ�����ĸ�ֵ
		prevPosition[1] = position[1];

		//��������
		ceres::Problem problem;

		if (i == 0) {
			lossType = LOSS_HUBER;                                   //��ʼʱʹ��Huber��ʧ����
		}
		ceres::LossFunction* toa_cost_function = custom_loss::CreateLossFunction(lossType, toaResidual_STD);

		//ָ�����ݼ�(�в��)
		for (int j = 0; j < dataNum; ++j) {
			ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOAResidual, 1, 2>(new TOAResidual(toaResiduals[j]));
			problem.AddResidualBlock(costFunc_TOA, toa_cost_function, position);
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
		options.logging_type = ceres::LoggingType::SILENT; // ��ֹ��־���

		//��ʼ�������
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

		//���Ԥ�����������һ���Ż�������֮���������������tolerance�����break
		RtLbsType deltaDis = sqrt((prevPosition[0] - position[0]) * (prevPosition[0] - position[0]) + (prevPosition[1] - position[1]) * (prevPosition[1] - position[1]));
		if (deltaDis < tol) {
			break;
		}

		UpdateResidualWeight(position, toaResiduals, toaResidual_STD);
	}
	return { position[0], position[1] };
}

Point2D TOASolver::Solving_WIRLS(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint)
{
	int iterNum = config.m_iterNum;
	double tol = config.m_tolerance;
	LOSSFUNCTIONTYPE lossType = config.m_lossType;

	RtLbsType position[2] = { initPoint.x, initPoint.y };				//��ʼλ�ù���
	RtLbsType prevPosition[2] = { 0,0 };								//ǰһ���ڵ��λ�ù���

	double toaResidual_STD = 0.01;											/** @brief	AOA�в��׼��	*/

	int dataNum = static_cast<int>(m_gsData.size());
	std::vector<TOAResidual> toaResiduals(dataNum);
	for (int i = 0; i < dataNum; ++i) {//��ʼ���в�
		toaResiduals[i].Init(m_gsData[i], m_gsData[i]->m_weight);
	}

	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//����ǰһ��Ԥ�����ĸ�ֵ
		prevPosition[1] = position[1];

		//��������
		ceres::Problem problem;
		ceres::LossFunction* aoa_cost_function = custom_loss::CreateLossFunction(lossType, toaResidual_STD);

		//ָ�����ݼ�(�в��)
		for (int j = 0; j < dataNum; ++j) {
			ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOAResidual, 1, 2>(new TOAResidual(toaResiduals[j]));
			problem.AddResidualBlock(costFunc_TOA, aoa_cost_function, position);
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
		options.logging_type = ceres::LoggingType::SILENT; // ��ֹ��־���

		//��ʼ�������
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

		//���Ԥ�����������һ���Ż�������֮���������������tolerance�����break
		RtLbsType deltaDis = sqrt((prevPosition[0] - position[0]) * (prevPosition[0] - position[0]) + (prevPosition[1] - position[1]) * (prevPosition[1] - position[1]));
		if (deltaDis < tol) {
			break;
		}
		double stdTemp = 0;
		UpdateResidualWeight(position, toaResiduals, stdTemp);
		//toaResidual_STD = std::min(toaResidual_STD, stdTemp);
	}
	return { position[0], position[1] };
}

Point2D TOASolver::Solving(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint)
{
	const SOLVINGMODE mode = config.m_solvingMode;
	Point2D solution = initPoint;
	if (mode == SOLVING_LS) {
		Solving_LS(bbox, solution);
	}
	else if (mode == SOLVING_WLS) {
		solution = Solving_WLS(bbox, initPoint);
	}
	else if (mode == SOLVING_TSWLS) {
		solution = Solving_TSWLS(bbox);
	}
	else if (mode == SOLVING_IRLS) {
		solution = Solving_IRLS(config, bbox, initPoint);
	}
	else if (mode == SOLVING_WIRLS) {
		solution = Solving_WIRLS(config, bbox, initPoint);
	}
	return solution;
}

void TOASolver::UpdateResidualWeight(const double* position, std::vector<TOAResidual>& toaResiduals, double& toaResidual_STD)
{
	//Ȩ�ظ���
	double max_toa_weight = 0.0;
	std::vector<double> r_aoas;
	for (auto& curTOAResidual : toaResiduals) {
		double res = curTOAResidual.GetResidual(position);
		double cur_toa_weight = curTOAResidual.GetWeight() / (res * res + EPSILON);								//Ȩ��
		max_toa_weight = std::max(max_toa_weight, cur_toa_weight);
		curTOAResidual.SetWeight(cur_toa_weight);
		r_aoas.push_back(res);
	}
	//��һ��Ȩ��
	for (auto& curTOAResidual : toaResiduals) {
		double cur_toa_weight = curTOAResidual.GetWeight() / max_toa_weight;
		curTOAResidual.SetWeight(cur_toa_weight);
	}

	//���²в��׼��
	toaResidual_STD = vectoroperator::CalculateStandardDeviation(r_aoas);
}

double TOASolver::GetResidualSTD(const double* position, std::vector<TOAResidual>& toaResiduals)
{
	std::vector<double> r_toas;
	for (auto& curTOAResidual : toaResiduals) {
		double res = curTOAResidual.GetResidual(position);
		r_toas.push_back(res);
	}
	return vectoroperator::CalculateStandardDeviation(r_toas);
}

