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
	//定义问题
	ceres::Problem problem;

	RtLbsType position[2] = { 0,0 };		//初始位置估计

	//指定数据集
	for (auto it = m_gsData.begin(); it != m_gsData.end(); ++it) {
		const GeneralSource* curSource = *it;
		ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOALSResidual, 1, 2>(new AOALSResidual(curSource));				//定义损失函数
		problem.AddResidualBlock(costFunction, nullptr, position);
	}

	//配置求解器
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	//开始求解问题
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	//输出结果
	std::cout << "Estimate non-corporate source coordinate is (" << position[0] << "," << position[1] << ")." << std::endl;
}

void AOASolver::Solving_WLS()
{
	//定义问题
	ceres::Problem problem;

	RtLbsType position[2] = { 0,0 };		//初始位置估计

	//指定数据集
	for (auto it = m_gsData.begin(); it != m_gsData.end(); ++it) {
		const GeneralSource* curSource = *it;
		ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAWLSResidual, 1, 2>(new AOAWLSResidual(curSource));				//定义损失函数
		problem.AddResidualBlock(costFunction, nullptr, position);
	}

	//配置求解器
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;

	//开始求解问题
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	//输出结果
	std::cout << "Estimate non-corporate source coordinate is (" << position[0] << "," << position[1] << ")." << std::endl;
}

Point2D AOASolver::Solving_WIRLS(int iterNum, RtLbsType tol, const Point2D& initPoint)
{
	RtLbsType position[2] = { initPoint.x, initPoint.y };								//初始位置估计
	RtLbsType prevPosition[2] = { 0,0 };							//前一个节点的位置估计

	int aoaDataNum = static_cast<int>(m_gsData.size());				//数据数量
	std::vector<AOAWLSResidual> aoaResiduals(aoaDataNum);				//设置aoa残差对象
	for (int i = 0; i < aoaDataNum; ++i) {
		aoaResiduals[i].Init(m_gsData[i]);
	}

	//int rdoaDataNum = aoaDataNum - 1;								//RDOA 残差数据量
	//std::vector<RDOAWLSResidual> rdoaResiduals(rdoaDataNum);
	//for (int i = 0; i < rdoaDataNum; ++i) {
	//	rdoaResiduals[i].Init(m_gsData[0], m_gsData[i + 1]);
	//}
	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//进行前一次预测结果的赋值
		prevPosition[1] = position[1];

		//定义问题
		ceres::Problem problem;


		//制定数据集

		for (int i = 0; i < aoaDataNum; ++i) {
			ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAWLSResidual, 1, 2>(new AOAWLSResidual(aoaResiduals[i]));				//定义损失函数
			problem.AddResidualBlock(costFunction, nullptr, position);
		}

		//for (int i = 0; i < rdoaDataNum; ++i) {
		//	ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<RDOAWLSResidual, 1, 2>(new RDOAWLSResidual(rdoaResiduals[i]));				//定义损失函数
		//	problem.AddResidualBlock(costFunction, nullptr, position);
		//}

		//配置求解器
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_SCHUR;
		options.minimizer_progress_to_stdout = true;
		options.logging_type = ceres::LoggingType::SILENT; // 禁止日志输出

		//开始求解问题
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

		//输出结果
		//std::cout << "Estimate non-corporate source coordinate is (" << position[0] << "," << position[1] << ")." << std::endl;


		//求解预测坐标点与上一次优化的坐标之间的增量，若满足tolerance则进行break
		RtLbsType deltaDis = sqrt((prevPosition[0] - position[0]) * (prevPosition[0] - position[0]) + (prevPosition[1] - position[1]) * (prevPosition[1] - position[1]));
		if (deltaDis < tol) {
			break;
		}

		//权重更新
		std::vector<RtLbsType> weights(aoaDataNum);								//权重系数
		for (int i = 0; i < aoaDataNum; ++i) {
			RtLbsType res = aoaResiduals[i].GetResidual(position);
			weights[i] = aoaResiduals[i].GetWeight() / (res * res + 1e-6);								//权重
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
	RtLbsType position[2] = { 0,0 };								//初始位置估计
	RtLbsType prevPosition[2] = { 0,0 };							//前一个节点的位置估计

	int aoaDataNum = static_cast<int>(m_gsData.size());				//AOA 残差数据数量
	std::vector<AOAWLSResidual> aoaResiduals(aoaDataNum);				//设置aoa残差对象
	for (int i = 0; i < aoaDataNum; ++i) {
		aoaResiduals[i].Init(m_gsData[i]);
		//aoaResiduals[i].SetWeight(1.0);								//设定初始权重为1
	}

	int rdoaDataNum = aoaDataNum - 1;								//RDOA 残差数据量
	std::vector<RDOAWLSResidual> rdoaResiduals(rdoaDataNum);
	for (int i = 0; i < rdoaDataNum; ++i) {
		rdoaResiduals[i].Init(m_gsData[0], m_gsData[i + 1]);
	}


	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//进行前一次预测结果的赋值
		prevPosition[1] = position[1];

		//定义问题
		ceres::Problem problem;

		//制定数据集
		for (int i = 0; i < aoaDataNum; ++i) {
			ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<AOAWLSResidual, 1, 2>(new AOAWLSResidual(aoaResiduals[i]));				//定义损失函数
			problem.AddResidualBlock(costFunction, nullptr, position);
		}

		for (int i = 0; i < rdoaDataNum; ++i) {
			ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<RDOAWLSResidual, 1, 2>(new RDOAWLSResidual(rdoaResiduals[i]));				//定义损失函数
			problem.AddResidualBlock(costFunction, nullptr, position);
		}

		//配置求解器
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = false;

		//开始求解问题
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

		//输出结果
		std::cout << "Estimate non-corporate source coordinate is (" << position[0] << "," << position[1] << ")." << std::endl;


		//求解预测坐标点与上一次优化的坐标之间的增量，若满足tolerance则进行break
		RtLbsType deltaDis = sqrt((prevPosition[0] - position[0]) * (prevPosition[0] - position[0]) + (prevPosition[1] - position[1]) * (prevPosition[1] - position[1]));
		if (deltaDis < tol) {
			break;
		}

		//权重更新
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
