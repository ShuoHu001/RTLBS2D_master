#include "aoa_tdoa_solver.h"

AOATDOASolver::AOATDOASolver()
{
}

AOATDOASolver::AOATDOASolver(const AOATDOASolver& solver)
	: m_refSource(solver.m_refSource)
	, m_gsData(solver.m_gsData)
	, m_solution(solver.m_solution)
{
}

AOATDOASolver::~AOATDOASolver()
{
}

AOATDOASolver& AOATDOASolver::operator=(const AOATDOASolver& solver)
{
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
		if (curSource != refSource) {
			m_gsData.push_back(curSource);
		}
	}
	if (m_gsData.size() < 2) {
		LOG_ERROR<< "AOATDOASolver: not satisfied TDOA algorithm min equation set size, min size is 2." << ENDL;
	}
}

void AOATDOASolver::SetGeneralSource(GeneralSource* refSource, GeneralSource* gs1, GeneralSource* gs2)
{
	m_refSource = refSource;
	m_gsData.resize(2);
	m_gsData[0] = gs1;
	m_gsData[1] = gs2;
}

RtLbsType AOATDOASolver::Solving_LS(Point2D& outP)
{
	//定义问题
	ceres::Problem problem;

	RtLbsType position[2] = { outP.x,outP.y };		//初始位置估计

	int dataNum = static_cast<int>(m_gsData.size());
	std::vector<AOALSResidual*> aoaResiduals(dataNum);
	std::vector<TDOALSResidual*> tdoaResiduals(dataNum);

	//初始化残差
	for (int i = 0; i < dataNum; ++i) {
		aoaResiduals[i] = new AOALSResidual(m_gsData[i]);
		tdoaResiduals[i] = new TDOALSResidual(m_refSource, m_gsData[i]);
	}


	//指定数据集
	for (int i = 0; i < dataNum; ++i) {
		ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOALSResidual, 1, 2>(aoaResiduals[i]);
		ceres::CostFunction* costFunc_TDOA = new ceres::AutoDiffCostFunction<TDOALSResidual, 1, 2>(tdoaResiduals[i]);
		problem.AddResidualBlock(costFunc_AOA, nullptr, position);
		problem.AddResidualBlock(costFunc_TDOA, nullptr, position);
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

	//最后一次迭代位置误差
	const ceres::IterationSummary& last_iteration = summary.iterations.back();
	return last_iteration.cost_change;
}

Point2D AOATDOASolver::Solving_WIRLS(int iterNum, RtLbsType tol, const Point2D& initPoint)
{
	RtLbsType position[2] = { initPoint.x, initPoint.y };				//初始位置估计
	RtLbsType prevPosition[2] = { 0,0 };								//前一个节点的位置估计

	int dataNum = static_cast<int>(m_gsData.size());
	std::vector<AOAWLSResidual*> aoaResiduals(dataNum + 1);					/** @brief	AOA 残差	*/
	std::vector<TDOAWLSResidual*> tdoaResiduals(dataNum);					/** @brief	TDOA 残差	*/

	//初始化残差
	aoaResiduals[0] = new AOAWLSResidual(m_refSource);
	for (int i = 1; i < dataNum + 1; ++i) {
		aoaResiduals[i] = new AOAWLSResidual(m_gsData[i]);
	}
	for (int i = 0; i < dataNum; ++i) {
		tdoaResiduals[i] = new TDOAWLSResidual(m_refSource, m_gsData[i]);
	}

	for (int i = 0; i < iterNum; ++i) {

		prevPosition[0] = position[0];								//进行前一次预测结果的赋值
		prevPosition[1] = position[1];

		//定义问题
		ceres::Problem problem;

		//指定数据集
		for (auto& curAOAResidual : aoaResiduals) {
			ceres::CostFunction* costFunc_AOA = new ceres::AutoDiffCostFunction<AOAWLSResidual, 1, 2>(curAOAResidual);
			problem.AddResidualBlock(costFunc_AOA, nullptr, position);
		}
		for (auto& curTDOAResidual : tdoaResiduals) {
			ceres::CostFunction* costFunc_TDOA = new ceres::AutoDiffCostFunction<TDOAWLSResidual, 1, 2>(curTDOAResidual);
			problem.AddResidualBlock(costFunc_TDOA, nullptr, position);
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
		RtLbsType max_tdoa_weight = 0.0;
		for (auto& curAOAResidual : aoaResiduals) {
			RtLbsType res_aoa = curAOAResidual->GetResidual(position);
			RtLbsType cur_aoa_weight = curAOAResidual->GetWeight() / (abs(res_aoa) + EPSILON);
			max_aoa_weight = std::max(max_aoa_weight, cur_aoa_weight);
			curAOAResidual->SetWeight(cur_aoa_weight);
		}
		for (auto& curTDOAResidual : tdoaResiduals) {
			RtLbsType res_tdoa = curTDOAResidual->GetResidual(position);
			RtLbsType cur_tdoa_weight = curTDOAResidual->GetWeight() / (abs(res_tdoa) + EPSILON);
			max_tdoa_weight = std::max(max_tdoa_weight, cur_tdoa_weight);
			tdoaResiduals[i]->SetWeight(cur_tdoa_weight);
		}
		//归一化权重
		for (auto& curAOAResidual : aoaResiduals) {
			RtLbsType cur_aoa_weight = curAOAResidual->GetWeight() / max_aoa_weight;
			curAOAResidual->SetWeight(cur_aoa_weight);
		}
		for (auto& curTDOAResidual : tdoaResiduals) {
			RtLbsType cur_tdoa_weight = curTDOAResidual->GetWeight() / max_tdoa_weight;
			curTDOAResidual->SetWeight(cur_tdoa_weight);
		}
	}

	//释放内存
	for (auto& curAOAResidual : aoaResiduals) {
		delete curAOAResidual;
	}
	for (auto& curTDOAResidual : tdoaResiduals) {
		delete curTDOAResidual;
	}

	return { position[0], position[1] };
}
