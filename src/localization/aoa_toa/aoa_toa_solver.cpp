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

void AOATOASolver::SetGeneralSource(GeneralSource* gs1, GeneralSource* gs2)
{
	m_gsData.resize(2);
	m_gsData[0] = gs1;
	m_gsData[1] = gs2;
}

RtLbsType AOATOASolver::Solving_LS(const BBox2D& bbox, Point2D& outP)
{
	//定义问题
	ceres::Problem problem;

	RtLbsType position[2] = { outP.x,outP.y };		//初始位置估计

	int dataNum = static_cast<int>(m_gsData.size());

	//指定数据集(残差块)
	for (int i = 0; i < dataNum; ++i) {
		ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(m_gsData[i]));
		ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOAResidual, 1, 2>(new TOAResidual(m_gsData[i]));
		problem.AddResidualBlock(costFunc_AOA, nullptr, position);
		problem.AddResidualBlock(costFunc_TOA, nullptr, position);
	}

	//设置边界约束
	problem.SetParameterLowerBound(position, 0, bbox.m_min.x);
	problem.SetParameterUpperBound(position, 0, bbox.m_max.x);
	problem.SetParameterLowerBound(position, 1, bbox.m_min.y);
	problem.SetParameterUpperBound(position, 1, bbox.m_max.y);

	//配置求解器
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;

	//求解
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	outP.x = position[0];
	outP.y = position[1];

	//最后一次迭代位置误差
	if (summary.iterations.size() != 0) {
		const ceres::IterationSummary& last_iteration = summary.iterations.back();
		return last_iteration.cost_change;
	}
	return 1e-5;
}

Point2D AOATOASolver::Solving_WLS(const BBox2D& bbox, const Point2D& initPoint)
{
	//定义问题
	ceres::Problem problem;

	RtLbsType position[2] = { initPoint.x, initPoint.y };		//初始位置估计

	int dataNum = static_cast<int>(m_gsData.size());

	//指定数据集(残差块)
	for (int i = 0; i < dataNum; ++i) {
		ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(m_gsData[i], m_gsData[i]->m_weight));
		ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOAResidual, 1, 2>(new TOAResidual(m_gsData[i], m_gsData[i]->m_weight));
		problem.AddResidualBlock(costFunc_AOA, nullptr, position);
		problem.AddResidualBlock(costFunc_TOA, nullptr, position);
	}

	//设置边界约束
	problem.SetParameterLowerBound(position, 0, bbox.m_min.x);
	problem.SetParameterUpperBound(position, 0, bbox.m_max.x);
	problem.SetParameterLowerBound(position, 1, bbox.m_min.y);
	problem.SetParameterUpperBound(position, 1, bbox.m_max.y);

	//配置求解器
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;

	//求解
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	return Point2D(position[0], position[1]);
}

Point2D AOATOASolver::Solving_TSWLS(const BBox2D& bbox, const Point2D& initPoint)
{
	Point2D tsInitPoint = initPoint;
	Solving_LS(bbox, tsInitPoint);
	//定义问题
	ceres::Problem problem;

	RtLbsType position[2] = { tsInitPoint.x, tsInitPoint.y };		//初始位置估计

	int dataNum = static_cast<int>(m_gsData.size());

	//指定数据集(残差块)
	for (int i = 0; i < dataNum; ++i) {
		ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(m_gsData[i]));
		ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOAResidual, 1, 2>(new TOAResidual(m_gsData[i]));
		problem.AddResidualBlock(costFunc_AOA, nullptr, position);
		problem.AddResidualBlock(costFunc_TOA, nullptr, position);
	}

	//设置边界约束
	problem.SetParameterLowerBound(position, 0, bbox.m_min.x);
	problem.SetParameterUpperBound(position, 0, bbox.m_max.x);
	problem.SetParameterLowerBound(position, 1, bbox.m_min.y);
	problem.SetParameterUpperBound(position, 1, bbox.m_max.y);

	//配置求解器
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;

	//求解
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	return Point2D(position[0], position[1]);
}

Point2D AOATOASolver::Solving_IRLS(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint)
{
	int iterNum = config.m_iterNum;
	double tol = config.m_tolerance;
	LOSSFUNCTIONTYPE lossType = config.m_lossType;

	RtLbsType position[2] = { initPoint.x, initPoint.y };				//初始位置估计
	RtLbsType prevPosition[2] = { 0,0 };								//前一个节点的位置估计

	double aoaResidual_STD = 0.01;											/** @brief	AOA残差标准差	*/
	double toaResidual_STD = 0.01;											/** @brief	TOA残差标准差	*/

	int dataNum = static_cast<int>(m_gsData.size());
	std::vector<AOAResidual> aoaResiduals(dataNum);
	std::vector<TOAResidual> toaResiduals(dataNum);

	//初始化残差
	for (int i = 0; i < dataNum; ++i) {
		aoaResiduals[i].Init(m_gsData[i], weightFactor.m_phiWeight);
		toaResiduals[i].Init(m_gsData[i], weightFactor.m_timeWeight);
	}

	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//进行前一次预测结果的赋值
		prevPosition[1] = position[1];

		//定义问题
		ceres::Problem problem;

		ceres::LossFunction* aoa_cost_function = custom_loss::CreateLossFunction(lossType, aoaResidual_STD);
		ceres::LossFunction* toa_cost_function = custom_loss::CreateLossFunction(lossType, toaResidual_STD);

		//指定数据集(残差块)
		for (int j = 0; j < dataNum; ++j) {
			ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(aoaResiduals[j]));
			ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOAResidual, 1, 2>(new TOAResidual(toaResiduals[j]));
			problem.AddResidualBlock(costFunc_AOA, aoa_cost_function, position);
			problem.AddResidualBlock(costFunc_TOA, toa_cost_function, position);
		}

		//设置边界约束
		problem.SetParameterLowerBound(position, 0, bbox.m_min.x);
		problem.SetParameterUpperBound(position, 0, bbox.m_max.x);
		problem.SetParameterLowerBound(position, 1, bbox.m_min.y);
		problem.SetParameterUpperBound(position, 1, bbox.m_max.y);

		//配置求解器
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = true;
		options.logging_type = ceres::LoggingType::SILENT; // 禁止日志输出

		//开始求解问题
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

		//求解预测坐标点与上一次优化的坐标之间的增量，若满足tolerance则进行break
		RtLbsType deltaDis = sqrt((prevPosition[0] - position[0]) * (prevPosition[0] - position[0]) + (prevPosition[1] - position[1]) * (prevPosition[1] - position[1]));
		if (deltaDis < tol) {
			break;
		}

		UpdateResidualWeight(position, aoaResiduals, toaResiduals, aoaResidual_STD, toaResidual_STD);
	}

	return { position[0], position[1] };
}

Point2D AOATOASolver::Solving_WIRLS(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint)
{
	int iterNum = config.m_iterNum;
	double tol = config.m_tolerance;
	LOSSFUNCTIONTYPE lossType = config.m_lossType;

	RtLbsType position[2] = { initPoint.x, initPoint.y };				//初始位置估计
	RtLbsType prevPosition[2] = { 0,0 };								//前一个节点的位置估计

	double aoaResidual_STD = 0.01;											/** @brief	AOA残差标准差	*/
	double toaResidual_STD = 0.01;											/** @brief	TOA残差标准差	*/

	int dataNum = static_cast<int>(m_gsData.size());
	std::vector<AOAResidual> aoaResiduals(dataNum);
	std::vector<TOAResidual> toaResiduals(dataNum);

	//初始化残差
	for (int i = 0; i < dataNum; ++i) {
		aoaResiduals[i].Init(m_gsData[i], m_gsData[i]->m_weight * weightFactor.m_phiWeight);
		toaResiduals[i].Init(m_gsData[i], m_gsData[i]->m_weight * weightFactor.m_timeWeight);
	}

	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//进行前一次预测结果的赋值
		prevPosition[1] = position[1];

		//定义问题
		ceres::Problem problem;

		ceres::LossFunction* aoa_cost_function = custom_loss::CreateLossFunction(lossType, aoaResidual_STD);
		ceres::LossFunction* toa_cost_function = custom_loss::CreateLossFunction(lossType, toaResidual_STD);

		//指定数据集(残差块)
		for (int j = 0; j < dataNum; ++j) {
			ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(aoaResiduals[j]));
			ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOAResidual, 1, 2>(new TOAResidual(toaResiduals[j]));
			problem.AddResidualBlock(costFunc_AOA, aoa_cost_function, position);
			problem.AddResidualBlock(costFunc_TOA, toa_cost_function, position);
		}

		//设置边界约束
		problem.SetParameterLowerBound(position, 0, bbox.m_min.x);
		problem.SetParameterUpperBound(position, 0, bbox.m_max.x);
		problem.SetParameterLowerBound(position, 1, bbox.m_min.y);
		problem.SetParameterUpperBound(position, 1, bbox.m_max.y);

		//配置求解器
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = true;
		options.logging_type = ceres::LoggingType::SILENT; // 禁止日志输出

		//开始求解问题
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

		//求解预测坐标点与上一次优化的坐标之间的增量，若满足tolerance则进行break
		RtLbsType deltaDis = sqrt((prevPosition[0] - position[0]) * (prevPosition[0] - position[0]) + (prevPosition[1] - position[1]) * (prevPosition[1] - position[1]));
		if (deltaDis < tol) {
			break;
		}

		UpdateResidualWeight(position, aoaResiduals, toaResiduals, aoaResidual_STD, toaResidual_STD);
	}

	return { position[0], position[1] };
}

Point2D AOATOASolver::Solving(const SolvingConfig& config, const BBox2D& bbox, const WeightFactor& weightFactor, const Point2D& initPoint)
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

void AOATOASolver::UpdateResidualWeight(const double* position, std::vector<AOAResidual>& aoaResiduals, std::vector<TOAResidual>& toaResiduals, double& aoaResidual_STD, double& toaResidual_STD)
{
	//权重更新
	double max_aoa_weight = 0.0;
	double max_toa_weight = 0.0;
	int dataNum = static_cast<int>(m_gsData.size());
	for (int i = 0; i < dataNum; ++i) {
		double res_aoa = aoaResiduals[i].GetResidual(position);
		double res_toa = toaResiduals[i].GetResidual(position);
		double cur_aoa_weight = aoaResiduals[i].GetWeight() / (abs(res_aoa) + EPSILON);
		double cur_toa_weight = toaResiduals[i].GetWeight() / (abs(res_toa) + EPSILON);
		max_aoa_weight = std::max(max_aoa_weight, cur_aoa_weight);
		max_toa_weight = std::max(max_toa_weight, cur_toa_weight);
		aoaResiduals[i].SetWeight(cur_aoa_weight);
		toaResiduals[i].SetWeight(cur_toa_weight);
	}
	//归一化权重
	for (int i = 0; i < dataNum; ++i) {
		double cur_aoa_weight = aoaResiduals[i].GetWeight() / max_aoa_weight;
		double cur_toa_weight = toaResiduals[i].GetWeight() / max_toa_weight;
		aoaResiduals[i].SetWeight(cur_aoa_weight);
		toaResiduals[i].SetWeight(cur_toa_weight);
	}
}

double AOATOASolver::GetAOAResiudalSTD(const double* position, std::vector<AOAResidual>& aoaResiduals)
{
	std::vector<double> r_aoas;
	for (auto& curAOAResidual : aoaResiduals) {
		double res = curAOAResidual.GetResidual(position);
		r_aoas.push_back(res);
	}
	return vectoroperator::CalculateStandardDeviation(r_aoas);
}

double AOATOASolver::GetTOAResidualSTD(const double* position, std::vector<TOAResidual>& toaResiduals)
{
	std::vector<double> r_toas;
	for (auto& curTOAResidual : toaResiduals) {
		double res = curTOAResidual.GetResidual(position);
		r_toas.push_back(res);
	}
	return vectoroperator::CalculateStandardDeviation(r_toas);
}
