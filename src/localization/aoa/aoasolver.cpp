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
	//定义问题
	ceres::Problem problem;

	RtLbsType position[2] = { initPoint.x, initPoint.y };		//初始位置估计

	//指定数据集
	for (auto it = m_gsData.begin(); it != m_gsData.end(); ++it) {
		const GeneralSource* curSource = *it;
		ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(curSource));				//定义损失函数
		problem.AddResidualBlock(costFunction, nullptr, position);
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

	//开始求解问题
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);



	//输出结果
	return Point2D(position[0], position[1]);
}

Point2D AOASolver::Solving_WLS(const BBox2D& bbox, const Point2D& initPoint)
{
	//定义问题
	ceres::Problem problem;

	RtLbsType position[2] = { initPoint.x, initPoint.y };		//初始位置估计

	//指定数据集
	for (auto it = m_gsData.begin(); it != m_gsData.end(); ++it) {
		const GeneralSource* curSource = *it;
		ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(curSource, curSource->m_weight));				//定义损失函数
		problem.AddResidualBlock(costFunction, nullptr, position);
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

	//开始求解问题
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	//输出结果
	return Point2D(position[0], position[1]);
}

Point2D AOASolver::Solving_TSWLS(const BBox2D& bbox, const Point2D& initPoint)
{
	Point2D tsInitPoint = Solving_LS(bbox, initPoint);

	
	//定义问题
	ceres::Problem problem;

	RtLbsType position[2] = { tsInitPoint.x, tsInitPoint.y };		//初始位置估计

	//指定数据集
	for (auto it = m_gsData.begin(); it != m_gsData.end(); ++it) {
		const GeneralSource* curSource = *it;
		ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(curSource));				//定义损失函数
		problem.AddResidualBlock(costFunction, nullptr, position);
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

	//开始求解问题
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	//输出结果
	return Point2D(position[0], position[1]);
}

Point2D AOASolver::Solving_IRLS(const SolvingConfig& config, const BBox2D& bbox, const Point2D& initPoint)
{
	int iterNum = config.m_iterNum;
	double tol = config.m_tolerance;
	LOSSFUNCTIONTYPE lossType = config.m_lossType;

	RtLbsType position[2] = { initPoint.x,initPoint.y };								//初始位置估计
	RtLbsType prevPosition[2] = { 0,0 };							//前一个节点的位置估计

	double aoaResidual_STD = 0.01;											/** @brief	AOA残差标准差	*/

	int aoaDataNum = static_cast<int>(m_gsData.size());				//AOA 残差数据数量
	std::vector<AOAResidual> aoaResiduals(aoaDataNum);				//设置aoa残差对象
	for (int i = 0; i < aoaDataNum; ++i) {
		aoaResiduals[i].Init(m_gsData[i]);
	}


	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//进行前一次预测结果的赋值
		prevPosition[1] = position[1];

		//定义问题
		ceres::Problem problem;

		ceres::LossFunction* aoa_cost_function = custom_loss::CreateLossFunction(lossType, aoaResidual_STD);

		//制定数据集
		for (int j = 0; j < aoaDataNum; ++j) {
			ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(aoaResiduals[j]));				//定义损失函数
			problem.AddResidualBlock(costFunction, aoa_cost_function, position);
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

		//开始求解问题
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);


		//求解预测坐标点与上一次优化的坐标之间的增量，若满足tolerance则进行break
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

	RtLbsType position[2] = { initPoint.x, initPoint.y };								//初始位置估计
	RtLbsType prevPosition[2] = { 0,0 };							//前一个节点的位置估计

	double aoaResidual_STD = 0.01;											/** @brief	AOA残差标准差	*/

	int aoaDataNum = static_cast<int>(m_gsData.size());				//数据数量
	std::vector<AOAResidual> aoaResiduals(aoaDataNum);				//设置aoa残差对象
	for (int i = 0; i < aoaDataNum; ++i) {
		aoaResiduals[i].Init(m_gsData[i], m_gsData[i]->m_weight);
	}

	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//进行前一次预测结果的赋值
		prevPosition[1] = position[1];

		//定义问题
		ceres::Problem problem;

		ceres::LossFunction* aoa_cost_function = custom_loss::CreateLossFunction(lossType, aoaResidual_STD);

		//制定数据集
		for (int j = 0; j < aoaDataNum; ++j) {
			ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAResidual, 1, 2>(new AOAResidual(aoaResiduals[j]));				//定义损失函数
			problem.AddResidualBlock(costFunction, aoa_cost_function, position);
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

	//权重更新
	double max_aoa_weight = 0.0;
	std::vector<double> r_aoas;
	for (auto& curAOAResidual: aoaResiduals) {
		double res = curAOAResidual.GetResidual(position);
		double cur_aoa_weight = curAOAResidual.GetWeight() / (abs(res) + EPSILON);								//权重
		max_aoa_weight = std::max(max_aoa_weight, cur_aoa_weight);
		curAOAResidual.SetWeight(cur_aoa_weight);
		r_aoas.push_back(res);
	}
	//归一化权重
	for (auto& curAOAResidual : aoaResiduals) {
		double cur_aoa_weight = curAOAResidual.GetWeight()/max_aoa_weight;
		curAOAResidual.SetWeight(cur_aoa_weight);
	}

	//更新残差标准差
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
