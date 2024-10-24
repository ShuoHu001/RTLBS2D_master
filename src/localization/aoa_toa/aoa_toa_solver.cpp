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

RtLbsType AOATOASolver::Solving_LS(Point2D& outP)
{
	//定义问题
	ceres::Problem problem;

	RtLbsType position[2] = { outP.x,outP.y };		//初始位置估计

	int dataNum = static_cast<int>(m_gsData.size());
	std::vector<AOALSResidual*> aoaResiduals(dataNum);
	std::vector<TOALSResidual*> toaResiduals(dataNum);

	//初始化残差
	for (int i = 0; i < dataNum; ++i) {
		aoaResiduals[i] = new AOALSResidual(m_gsData[i]);
		toaResiduals[i] = new TOALSResidual(m_gsData[i]);
	}

	//指定数据集(残差块)
	for (int i = 0; i < dataNum; ++i) {
		ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOALSResidual, 1, 2>(aoaResiduals[i]);
		ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOALSResidual, 1, 2>(toaResiduals[i]);
		problem.AddResidualBlock(costFunc_AOA, nullptr, position);
		problem.AddResidualBlock(costFunc_TOA, nullptr, position);
	}

	//配置求解器
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;

	//求解
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	outP.x = position[0];
	outP.y = position[1];

	////删除残差所申请的内存
	//for (int i = 0; i < dataNum; ++i) {
	//	delete aoaResiduals[i];
	//	delete toaResiduals[i];
	//}

	//最后一次迭代位置误差
	const ceres::IterationSummary& last_iteration = summary.iterations.back();
	return last_iteration.cost_change;
}

Point2D AOATOASolver::Solving_WIRLS(int iterNum, RtLbsType tol, const Point2D& initPoint)
{
	RtLbsType position[2] = { initPoint.x, initPoint.y };				//初始位置估计
	RtLbsType prevPosition[2] = { 0,0 };								//前一个节点的位置估计

	int dataNum = static_cast<int>(m_gsData.size());
	std::vector<AOAWLSResidual*> aoaResiduals(dataNum);
	std::vector<TOAWLSResidual*> toaResiduals(dataNum);

	//初始化残差
	for (int i = 0; i < dataNum; ++i) {
		aoaResiduals[i] = new AOAWLSResidual(m_gsData[i]);
		toaResiduals[i] = new TOAWLSResidual(m_gsData[i]);
	}

	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//进行前一次预测结果的赋值
		prevPosition[1] = position[1];

		//定义问题
		ceres::Problem problem;

		//指定数据集(残差块)
		for (int j = 0; j < dataNum; ++j) {
			ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOAWLSResidual, 1, 2>(new AOAWLSResidual(m_gsData[j]));
			ceres::CostFunction* costFunc_TOA = new ceres::AutoDiffCostFunction<TOAWLSResidual, 1, 2>(new TOAWLSResidual(m_gsData[j]));
			problem.AddResidualBlock(costFunc_AOA, nullptr, position);
			problem.AddResidualBlock(costFunc_TOA, nullptr, position);
		}

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

		//权重更新
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
		//归一化权重
		for (int i = 0; i < dataNum; ++i) {
			RtLbsType cur_aoa_weight = aoaResiduals[i]->GetWeight() / max_aoa_weight;
			RtLbsType cur_toa_weight = toaResiduals[i]->GetWeight() / max_toa_weight;
			aoaResiduals[i]->SetWeight(cur_aoa_weight);
			toaResiduals[i]->SetWeight(cur_toa_weight);
		}
	}

	////释放内存
	//for (int i = 0; i < dataNum; ++i) {
	//	delete aoaResiduals[i];
	//	delete toaResiduals[i];
	//}
	

	return { position[0], position[1] };
}
