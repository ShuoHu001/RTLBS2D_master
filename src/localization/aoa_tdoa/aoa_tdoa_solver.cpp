#include "aoa_tdoa_solver.h"

AOATDOASolver::AOATDOASolver()
	: m_isValid(true)
	, m_refSource(nullptr)
{
}

AOATDOASolver::AOATDOASolver(const AOATDOASolver& solver)
	: m_isValid(solver.m_isValid)
	, m_refSource(solver.m_refSource)
	, m_gsData(solver.m_gsData)
	, m_solution(solver.m_solution)
{
}

AOATDOASolver::~AOATDOASolver()
{
}

AOATDOASolver& AOATDOASolver::operator=(const AOATDOASolver& solver)
{
	m_isValid = solver.m_isValid;
	m_refSource = solver.m_refSource;
	m_gsData = solver.m_gsData;
	m_solution = solver.m_solution;
	return *this;
}

void AOATDOASolver::SetGeneralSource(GeneralSource* refSource, const std::vector<GeneralSource*>& gsData)
{
	m_refSource = refSource;
	m_gsData.reserve(gsData.size());
	for (auto& curSource : gsData) {
		if (curSource->m_position != refSource->m_position) {			//������ο�Դλ����ͬ��Դ
			m_gsData.push_back(curSource);
		}
	}
	if (m_gsData.size() < 2) {
		LOG_ERROR<< "AOATDOASolver: not satisfied TDOA algorithm min equation set size, min size is 2." << ENDL;
		m_isValid = false;
	}
}

void AOATDOASolver::SetGeneralSource(GeneralSource* refSource, GeneralSource* gs1, GeneralSource* gs2)
{
	m_refSource = refSource;
	m_gsData.resize(2);
	m_gsData[0] = gs1;
	m_gsData[1] = gs2;
}

double AOATDOASolver::Solving_LS(const BBox2D& bbox, Point2D& outP)
{
	//��������
	ceres::Problem problem;

	RtLbsType position[2] = { outP.x,outP.y };		//��ʼλ�ù���

	int dataNum = static_cast<int>(m_gsData.size());


	//ָ�����ݼ�
	for (int i = 0; i < dataNum; ++i) {
		ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(m_gsData[i]));
		ceres::CostFunction* costFunc_TDOA = new ceres::AutoDiffCostFunction<TDOAResidual, 1, 2>(new TDOAResidual(m_refSource, m_gsData[i]));
		problem.AddResidualBlock(costFunc_AOA, nullptr, position);
		problem.AddResidualBlock(costFunc_TDOA, nullptr, position);
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

Point2D AOATDOASolver::Solving_WLS(const BBox2D& bbox, const Point2D& initPoint)
{
	//��������
	ceres::Problem problem;

	RtLbsType position[2] = { initPoint.x, initPoint.y };		//��ʼλ�ù���

	int dataNum = static_cast<int>(m_gsData.size());


	//ָ�����ݼ�
	for (int i = 0; i < dataNum; ++i) {
		ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(m_gsData[i], m_gsData[i]->m_weight));
		ceres::CostFunction* costFunc_TDOA = new ceres::AutoDiffCostFunction<TDOAResidual, 1, 2>(new TDOAResidual(m_refSource, m_gsData[i], m_gsData[i]->m_weight));
		problem.AddResidualBlock(costFunc_AOA, nullptr, position);
		problem.AddResidualBlock(costFunc_TDOA, nullptr, position);
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

Point2D AOATDOASolver::Solving_TSWLS(const BBox2D& bbox, const Point2D& initPoint)
{
	//ʹ��������Ȩ��С���˷������
	Point2D tsInitPoint = Point2D(0,0);
	Solving_LS(bbox, tsInitPoint);

	RtLbsType position[2] = { tsInitPoint.x, tsInitPoint.y };		//��ʼλ�ù���

	int dataNum = static_cast<int>(m_gsData.size());

	std::vector<AOAResidual> aoaResiduals(dataNum);					/** @brief	AOA �в�	*/
	std::vector<TDOAResidual> tdoaResiduals(dataNum);				/** @brief	TDOA �в�	*/

	//��ʼ���в�
	for (int i = 0; i < dataNum; ++i) {
		aoaResiduals[i].Init(m_gsData[i]);
		tdoaResiduals[i].Init(m_refSource, m_gsData[i]);
		double res_aoa = aoaResiduals[i].GetResidual(position);
		double res_tdoa = tdoaResiduals[i].GetResidual(position);
		double weight_aoa = 1.0 / (res_aoa * res_aoa + EPSILON);
		double weight_tdoa = 1.0 / (res_tdoa * res_tdoa + EPSILON);
		aoaResiduals[i].SetWeight(weight_aoa);
		tdoaResiduals[i].SetWeight(weight_tdoa);
	}

	//��������
	ceres::Problem problem;
	//ָ�����ݼ�(�в��)
	for (int i = 0; i < dataNum; ++i) {
		ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(aoaResiduals[i]));
		ceres::CostFunction* costFunc_TDOA = new ceres::AutoDiffCostFunction<TDOAResidual, 1, 2>(new TDOAResidual(tdoaResiduals[i]));
		problem.AddResidualBlock(costFunc_AOA, nullptr, position);
		problem.AddResidualBlock(costFunc_TDOA, nullptr, position);
	}

	////���ñ߽�Լ��
	//problem.SetParameterLowerBound(position, 0, bbox.m_min.x);
	//problem.SetParameterUpperBound(position, 0, bbox.m_max.x);
	//problem.SetParameterLowerBound(position, 1, bbox.m_min.y);
	//problem.SetParameterUpperBound(position, 1, bbox.m_max.y);

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

Point2D AOATDOASolver::Solving_IRLS(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint)
{
	int iterNum = config.m_iterNum;
	double tol = config.m_tolerance;
	LOSSFUNCTIONTYPE lossType = config.m_lossType;

	RtLbsType position[2] = { initPoint.x, initPoint.y };				//��ʼλ�ù���
	RtLbsType prevPosition[2] = { 0,0 };								//ǰһ���ڵ��λ�ù���

	double aoaResidual_STD = 0.01;											/** @brief	AOA�в��׼��	*/
	double tdoaResidual_STD = 0.01;											/** @brief	TDOA�в��׼��	*/

	int dataNum = static_cast<int>(m_gsData.size());
	std::vector<AOAResidual> aoaResiduals(dataNum);					/** @brief	AOA �в�	*/
	std::vector<TDOAResidual> tdoaResiduals(dataNum);				/** @brief	TDOA �в�	*/

	//��ʼ���в�
	for (int i = 0; i < dataNum; ++i) {
		aoaResiduals[i].Init(m_gsData[i]);
		tdoaResiduals[i].Init(m_refSource, m_gsData[i]);
	}

	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//����ǰһ��Ԥ�����ĸ�ֵ
		prevPosition[1] = position[1];

		//��������
		ceres::Problem problem;

		ceres::LossFunction* aoa_cost_function = custom_loss::CreateLossFunction(lossType, aoaResidual_STD);
		ceres::LossFunction* tdoa_cost_function = custom_loss::CreateLossFunction(lossType, tdoaResidual_STD);

		//ָ�����ݼ�
		for (int j = 0; j < dataNum; ++j) {
			ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(aoaResiduals[j]));
			ceres::CostFunction* costFunc_TDOA = new ceres::AutoDiffCostFunction<TDOAResidual, 1, 2>(new TDOAResidual(tdoaResiduals[j]));
			problem.AddResidualBlock(costFunc_AOA, aoa_cost_function, position);
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

		UpdateResidualWeight(position, weightFactor, aoaResiduals, tdoaResiduals, aoaResidual_STD, tdoaResidual_STD);
	}

	return { position[0], position[1] };
}

Point2D AOATDOASolver::Solving_WIRLS(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint)
{
	int iterNum = config.m_iterNum;
	double tol = config.m_tolerance;
	LOSSFUNCTIONTYPE lossType = config.m_lossType;

	RtLbsType position[2] = { initPoint.x, initPoint.y };				//��ʼλ�ù���
	RtLbsType prevPosition[2] = { 0,0 };								//ǰһ���ڵ��λ�ù���



	int dataNum = static_cast<int>(m_gsData.size());
	std::vector<AOAResidual> aoaResiduals(dataNum);					/** @brief	AOA �в�	*/
	std::vector<TDOAResidual> tdoaResiduals(dataNum);				/** @brief	TDOA �в�	*/

	//��ʼ���в�
	for (int i = 0; i < dataNum; ++i) {
		aoaResiduals[i].Init(m_gsData[i], m_gsData[i]->m_weight * weightFactor.m_phiWeight);
		tdoaResiduals[i].Init(m_refSource, m_gsData[i], m_gsData[i]->m_weight * weightFactor.m_timeWeight);
	}

	//��ʼ���в��׼��
	double aoaResidual_STD = 0.01;											/** @brief	AOA�в��׼��	*/
	double tdoaResidual_STD = 0.01;											/** @brief	TDOA�в��׼��	*/
	//CalculateResidualSTD(position, aoaResiduals, tdoaResiduals, aoaResidual_STD, tdoaResidual_STD);

	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//����ǰһ��Ԥ�����ĸ�ֵ
		prevPosition[1] = position[1];

		//��������
		ceres::Problem problem;
		if (i == 0) {
			lossType = LOSS_GEMANMCCLURE;
		}
		ceres::LossFunction* aoa_cost_function = custom_loss::CreateLossFunction(lossType, aoaResidual_STD);
		ceres::LossFunction* tdoa_cost_function = custom_loss::CreateLossFunction(lossType, tdoaResidual_STD);

		//ָ�����ݼ�
		for (int j = 0; j<dataNum; ++j) {
			ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(aoaResiduals[j]));
			ceres::CostFunction* costFunc_TDOA = new ceres::AutoDiffCostFunction<TDOAResidual, 1, 2>(new TDOAResidual(tdoaResiduals[j]));
			problem.AddResidualBlock(costFunc_AOA, aoa_cost_function, position);
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
		RtLbsType a, b;
		UpdateResidualWeight(position, weightFactor, aoaResiduals, tdoaResiduals, aoaResidual_STD, tdoaResidual_STD);
		//if (i == 0) {
		//	CalculateResidualSTD(position, aoaResiduals, tdoaResiduals, aoaResidual_STD, tdoaResidual_STD);
		//}
	}

	return { position[0], position[1] };
}

Point2D AOATDOASolver::Solving(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint)
{
	if (!m_isValid) {
		return Point2D(FLT_MAX, FLT_MAX);
	}
	const SOLVINGMODE mode = config.m_solvingMode;
	Point2D solution = initPoint;
	if (mode == SOLVING_LS) {
		Solving_LS(bbox, solution);
	}
	else if (mode == SOLVING_WLS) {
		solution = Solving_WLS(bbox, initPoint);
	}
	else if (mode == SOLVING_TSWLS) {
		solution = Solving_TSWLS(bbox, initPoint);
	}
	else if (mode == SOLVING_IRLS) {
		solution = Solving_IRLS(config, bbox, weightFactor, initPoint);
	}
	else if (mode == SOLVING_WIRLS) {
		solution = Solving_WIRLS(config, bbox, weightFactor, initPoint);
	}
	return solution;
}

void AOATDOASolver::UpdateResidualWeight(const double* position, const WeightFactor& weightFactor, std::vector<AOAResidual>& aoaResiduals, std::vector<TDOAResidual>& tdoaResiduals, double& aoaResidual_STD, double& tdoaResidual_STD)
{
	double max_aoa_weight = 0.0;
	double max_tdoa_weight = 0.0;
	std::vector<double> r_aoas;
	std::vector<double> r_tdoas;
	for (auto& curAOAResidual : aoaResiduals) {
		double res_aoa = curAOAResidual.GetResidual(position);
		double cur_aoa_weight = curAOAResidual.GetWeight() / (res_aoa*res_aoa + EPSILON);
		max_aoa_weight = std::max(max_aoa_weight, cur_aoa_weight);
		curAOAResidual.SetWeight(cur_aoa_weight);
		r_aoas.push_back(res_aoa);
	}
	for (auto& curTDOAResidual : tdoaResiduals) {
		double res_tdoa = curTDOAResidual.GetResidual(position);
		double cur_tdoa_weight = curTDOAResidual.GetWeight() / (res_tdoa*res_tdoa + EPSILON);
		max_tdoa_weight = std::max(max_tdoa_weight, cur_tdoa_weight);
		curTDOAResidual.SetWeight(cur_tdoa_weight);
		r_tdoas.push_back(res_tdoa);
	}
	//��һ��Ȩ��
	for (auto& curAOAResidual : aoaResiduals) {
		double cur_aoa_weight = curAOAResidual.GetWeight() / max_aoa_weight;
		curAOAResidual.SetWeight(cur_aoa_weight * weightFactor.m_phiWeight);
	}
	for (auto& curTDOAResidual : tdoaResiduals) {
		double cur_tdoa_weight = curTDOAResidual.GetWeight() / max_tdoa_weight;
		curTDOAResidual.SetWeight(cur_tdoa_weight * weightFactor.m_timeWeight);
	}

	//���²в��׼��
	aoaResidual_STD = vectoroperator::CalculateStandardDeviation(r_aoas);
	tdoaResidual_STD = vectoroperator::CalculateStandardDeviation(r_tdoas);
}

void AOATDOASolver::CalculateResidualSTD(const double* position, const std::vector<AOAResidual>& aoaResiduals, const std::vector<TDOAResidual>& tdoaResiduals, double& aoaResidual_STD, double& tdoaResidual_STD)
{
	double max_aoa_weight = 0.0;
	double max_tdoa_weight = 0.0;
	std::vector<double> r_aoas;
	std::vector<double> r_tdoas;
	for (auto& curAOAResidual : aoaResiduals) {
		double res_aoa = curAOAResidual.GetResidual(position);
		r_aoas.push_back(res_aoa);
	}
	for (auto& curTDOAResidual : tdoaResiduals) {
		double res_tdoa = curTDOAResidual.GetResidual(position);
		r_tdoas.push_back(res_tdoa);
	}

	//���²в��׼��
	aoaResidual_STD = vectoroperator::CalculateStandardDeviation(r_aoas);
	tdoaResidual_STD = vectoroperator::CalculateStandardDeviation(r_tdoas);

}
