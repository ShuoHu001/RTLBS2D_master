#include "sensordataprocessing.h"


void CalculateResidual_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi)
{
	RtLbsType cur_r_phi = std::abs(d1.m_phi - d2.m_phi);
	if (cur_r_phi > PI) {										//修正越界角度残差
		cur_r_phi  = TWO_PI - cur_r_phi;
	}
	r_phi = cur_r_phi * cur_r_phi;								//计算角度残差平方
}

void CalculateResidual_TOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_time, RtLbsType& r_power)
{
	RtLbsType cur_r_time = std::abs(d1.m_time - d2.m_time) * 1e9;
	RtLbsType cur_r_power = std::abs(d1.m_power - d2.m_power);
	r_time = cur_r_time * cur_r_time;
	r_power = cur_r_power * cur_r_power;
}

void CalculateResidual_TDOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_timediff)
{
	RtLbsType cur_r_timeDiff = (d1.m_timeDiff - d2.m_timeDiff) * 1e9;
	r_timediff = cur_r_timeDiff * cur_r_timeDiff;		//计算时间差残差平方
}

void CalculateResidual_AOA_TOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi, RtLbsType& r_time, RtLbsType& r_power)
{
	RtLbsType cur_r_phi = std::abs(d1.m_phi - d2.m_phi);
	if (cur_r_phi > PI) {
		cur_r_phi -= PI;
	}
	RtLbsType cur_r_time = std::abs(d1.m_time - d2.m_time) * 1e9;
	RtLbsType cur_r_power = std::abs(d1.m_power - d2.m_power);
	r_phi = cur_r_phi * cur_r_phi;
	r_time = cur_r_time * cur_r_time;
	r_power = cur_r_power * cur_r_power;
}

void CalculateResidual_TDOA_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi, RtLbsType& r_timediff)
{
	RtLbsType cur_r_phi = std::abs(d1.m_phi - d2.m_phi);
	RtLbsType cur_r_timeDiff = (d1.m_timeDiff - d2.m_timeDiff) * 1e9;
	if (cur_r_phi > PI) {
		cur_r_phi -= PI;
	}
	r_phi = cur_r_phi * cur_r_phi;								//计算角度残差平方
	r_timediff = cur_r_timeDiff * cur_r_timeDiff;				//计算时间差残差平方
}

bool ComparedByPower_SensorDataCollection(const SensorDataCollection& c1, const SensorDataCollection& c2) {
	return c1.m_datas[0].m_power > c2.m_datas[0].m_power;
}

bool ComparedByDelay_SensorDataCollection(const SensorDataCollection& c1, const SensorDataCollection& c2)
{
	return c1.m_datas[0].m_time > c2.m_datas[0].m_time;
}

bool ComparedByTimeDifference_SensorDataCollection(const SensorDataCollection& c1, const SensorDataCollection& c2)
{
	return c1.m_datas[0].m_timeDiff < c2.m_datas[0].m_timeDiff;
}

void CalculateSensorResidual_AOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi)
{
	//计算角度残差二范数和
	for (int i = 0; i < static_cast<int>(c1.m_datas.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateResidual_AOA(c1.m_datas[i], c2.m_datas[i], cur_r_phi);
		r_phi += cur_r_phi;
	}
}

void CalculateSensorCollectionResidual_AOA_SingleData(std::vector<SensorDataCollection>& c1, std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_powerDiff, int& nullDataNum)
{

	//计算角度残差二范数和
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		if (c2[i].m_datas.size() == 0) {
			nullDataNum++;
			continue;
		}
		RtLbsType cur_r_phi = 0;
		CalculateSensorResidual_AOA_SingleData(c1[i], c2[i], cur_r_phi);
		r_phi += cur_r_phi;
	}

	//计算功率差残差二范数和
	if (c2[0].m_datas.size() == 0) {			//参考站无功率
		nullDataNum = c2.size();
	}
	else {
		for (int i = 1; i < static_cast<int>(c1.size()); ++i) {
			if (c2[i].m_datas.size() == 0) {
				continue;
			}
			RtLbsType powerDiff1 = c1[i].m_datas[0].m_power - c1[0].m_datas[0].m_power;
			RtLbsType powerDiff2 = c2[i].m_datas[0].m_power - c2[0].m_datas[0].m_power;
			r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
		}
	}
	

}

void CalculateSensorResidual_AOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, const WeightFactor& w, RtLbsType& r_phi, RtLbsType& r_powerdiff, RtLbsType& r_angularSpread, int& nullDataNum)
{
	/** @brief	残差表达	*/
	struct Residual {
		RtLbsType r_phi;						/** @brief	角度残差	*/
		RtLbsType r_powerDiff;					/** @brief	功率差残差	*/
		RtLbsType r_norm;						/** @brief	归一化残差	*/
		void CalNormResidual(RtLbsType max_r_phi, RtLbsType max_r_powerDiff, const WeightFactor& w) {			//计算归一化残差
			r_norm = w.m_phiWeight * r_phi / max_r_phi + w.m_powerWeight * r_powerDiff / max_r_powerDiff;
		}
	};
	//构建残差矩阵
	int n = static_cast<int>(c1.m_datas.size());									/** @brief	原始传感器数据维度	*/
	int m = static_cast<int>(c2.m_datas.size());									/** @brief	目标传感器数据维度	*/

	RtLbsType max_r_phi = 0.0;													/** @brief	最大角度残差	*/
	RtLbsType max_r_powerDiff = 0.0;											/** @brief	最大功率差残差	*/

	std::vector<std::vector<Residual>> cost(n, std::vector<Residual>(m));					/** @brief	残差矩阵	*/
	std::vector<std::vector<RtLbsType>> norm_cost(n, std::vector<RtLbsType>(m));				/** @brief	归一化代价矩阵	*/
	for (int i = 0; i < n; ++i) {
		norm_cost[i].assign(m, 0);
		for (int j = 0; j < m; ++j) {

			RtLbsType cur_r_phi = std::abs(c1.m_datas[i].m_phi - c2.m_datas[j].m_phi);
			if (cur_r_phi > PI) {			//保证残差在0-PI内(最小残差准则)
				cur_r_phi = TWO_PI - cur_r_phi;
			}
			RtLbsType cur_r_powerDiff = abs(c1.m_datas[i].m_power - c2.m_datas[j].m_power);
			cost[i][j].r_phi = cur_r_phi;
			cost[i][j].r_powerDiff = cur_r_powerDiff;
			max_r_phi = std::max(max_r_phi, cost[i][j].r_phi);
			max_r_powerDiff = std::max(max_r_powerDiff, cost[i][j].r_powerDiff);
		}
	}

	//将矩阵进行归一化处理
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			cost[i][j].CalNormResidual(max_r_phi, max_r_powerDiff, w);
			norm_cost[i][j] = cost[i][j].r_norm;
		}
	}
	int new_m = m;
	//处理特殊情况-m的值小于n的值，属于欠缺分配问题，需要进行补全
	if (m < n) {
		new_m = n;
		for (int i = 0; i < n; ++i) {
			for (int j = m - 1; j < n; ++j) {
				norm_cost[i].push_back(2.0);					//增广代价矩阵
			}
		}
		nullDataNum = n - m;									//赋值无数据数量
	}

	//选取残差相对较少的一组作为最优解(匈牙利算法，基于glpk求解)
	std::vector<std::pair<int, int>> assigns = AssignOprimization(norm_cost);

	//计算残差和
	for (auto assign : assigns) {
		int& ni = assign.first;
		int& mi = assign.second;
		if (mi >= m) {										//防止在欠定指派问题情况下的越界访问无效数据
			continue;
		}
		r_phi += cost[ni][mi].r_phi;
		r_powerdiff += cost[ni][mi].r_powerDiff;
	}

	////计算相对角度扩展

	//RtLbsType conTemp1 = 0.0;
	//RtLbsType conTemp2 = 0.0;
	//for (auto assign : assigns) {
	//	int& ni = assign.first;
	//	int& mi = assign.second;
	//	if (mi >= m) { continue; }
	//	RtLbsType powerDiffLin = std::pow(10.0, cost[ni][mi].r_powerDiff / 10.0);
	//	conTemp1 += powerDiffLin * cost[ni][mi].r_phi;
	//	conTemp2 += powerDiffLin;
	//}
	//RtLbsType meanAoA = conTemp1 / conTemp2;

	//conTemp1 = 0.0;
	//for (auto assign : assigns) {
	//	int& ni = assign.first;
	//	int& mi = assign.second;
	//	if (mi >= m) { continue; }
	//	RtLbsType powerDiffLin = std::pow(10.0, cost[ni][mi].r_powerDiff / 10.0);
	//	conTemp1 += powerDiffLin * (cost[ni][mi].r_phi - meanAoA) * (cost[ni][mi].r_phi - meanAoA);
	//}
	//r_angularSpread = sqrt(conTemp1 / conTemp2);

	//计算角度扩展残差
	RtLbsType conTemp1 = 0.0;
	RtLbsType conTemp2 = 0.0;
	for (auto& assign : assigns) {
		int mi = assign.second;
		if (mi >= m) { continue; }
		conTemp1 += c2.m_datas[mi].m_powerLin * c2.m_datas[mi].m_phi;
		conTemp2 += c2.m_datas[mi].m_powerLin;
	}
	RtLbsType meanAoA = conTemp1 / conTemp2;

	conTemp1 = 0.0;
	for (auto& assign : assigns) {
		int mi = assign.second;
		if (mi >= m) { continue; }
		conTemp1 += c2.m_datas[mi].m_powerLin * (c2.m_datas[mi].m_phi - meanAoA) * (c2.m_datas[mi].m_phi - meanAoA);
	}

	RtLbsType angularSpread2 = sqrt(conTemp1 / conTemp2);
	RtLbsType angularSpread1 = c1.CalculateRMSAngularSpread();
	r_angularSpread = abs(angularSpread1 - angularSpread2);
}

void CalculateSensorCollectionResidual_AOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, const WeightFactor& w, RtLbsType& r_phi, RtLbsType& r_powerDiff, RtLbsType& r_angularSpread, int& nullDataNum)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0.0;
		RtLbsType cur_r_powerDiff = 0.0;
		RtLbsType cur_r_angularSpread = 0.0;
		int cur_nullDataNum = 0;
		CalculateSensorResidual_AOA_MultiData(c1[i], c2[i], w, cur_r_phi, cur_r_powerDiff, cur_r_angularSpread, cur_nullDataNum);
		r_phi += cur_r_phi;
		r_powerDiff += cur_r_powerDiff;
		nullDataNum += cur_nullDataNum;
		r_angularSpread += cur_r_angularSpread;
	}
}

void CalculateSensorResidual_TOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_time, RtLbsType& r_power)
{
	for (int i = 0; i < static_cast<int>(c1.m_datas.size()); ++i) {
		RtLbsType cur_r_time = 0;
		RtLbsType cur_r_power = 0;
		CalculateResidual_TOA(c1.m_datas[i], c2.m_datas[i], cur_r_time, cur_r_power);
		r_time += cur_r_time;
		r_power += cur_r_power;
	}
}

void CalculateSensorCollectionResidual_TOA_SingleData(std::vector<SensorDataCollection>& c1, std::vector<SensorDataCollection>& c2, RtLbsType& r_time, RtLbsType& r_power, int& nullDataNum)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		if (c2[i].m_datas.size() == 0) {
			nullDataNum++;
			continue;
		}
		RtLbsType cur_r_time = 0;
		RtLbsType cur_r_power = 0;
		CalculateSensorResidual_TOA_SingleData(c1[i], c2[i], cur_r_time, cur_r_power);
		r_time += cur_r_time;
		r_power += cur_r_power;
	}
}

void CalculateSensorResidual_TOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, const WeightFactor& w, RtLbsType& r_time, RtLbsType& r_power, int& nullDataNum)
{
	/** @brief	残差表达	*/
	struct Residual {
		RtLbsType r_time;						/** @brief	角度残差	*/
		RtLbsType r_power;						/** @brief	功率残差	*/
		RtLbsType r_norm;						/** @brief	归一化残差	*/
		void CalNormResidual(RtLbsType max_r_time, RtLbsType max_r_power, const WeightFactor& w) {			//计算归一化残差
			r_norm = w.m_timeWeight * r_time / max_r_time + w.m_powerWeight * r_power / max_r_power;
		}
	};

	//构建残差矩阵
	int n = static_cast<int>(c1.m_datas.size());									/** @brief	原始传感器数据维度	*/
	int m = static_cast<int>(c2.m_datas.size());									/** @brief	目标传感器数据维度	*/

	RtLbsType max_r_time = 0.0;													/** @brief	最大时间残差	*/
	RtLbsType max_r_power = 0.0;												/** @brief	最大功率残差	*/

	std::vector<std::vector<Residual>> cost(n, std::vector<Residual>(m));						/** @brief	残差矩阵	*/
	std::vector<std::vector<RtLbsType>> norm_cost(n, std::vector<RtLbsType>(m));				/** @brief	归一化代价矩阵	*/
	for (int i = 0; i < n; ++i) {
		norm_cost[i].assign(m, 0);
		for (int j = 0; j < m; ++j) {

			RtLbsType cur_r_time = std::abs(c1.m_datas[i].m_time - c2.m_datas[j].m_time) * 1e9;
			RtLbsType cur_r_power = abs(c1.m_datas[i].m_power - c2.m_datas[j].m_power);
			cost[i][j].r_time = cur_r_time;
			cost[i][j].r_power = cur_r_power;
			max_r_time = std::max(max_r_time, cost[i][j].r_time);
			max_r_power = std::max(max_r_power, cost[i][j].r_power);
		}
	}

	//将矩阵进行归一化处理
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			cost[i][j].CalNormResidual(max_r_time, max_r_power, w);
			norm_cost[i][j] = cost[i][j].r_norm;
		}
	}
	int new_m = m;
	//处理特殊情况-m的值小于n的值，属于欠缺分配问题，需要进行补全
	if (m < n) {
		new_m = n;
		for (int i = 0; i < n; ++i) {
			for (int j = m - 1; j < n; ++j) {
				norm_cost[i].push_back(2.0);					//增广代价矩阵
			}
		}
		nullDataNum = n - m;									//赋值无数据数量
	}

	//选取残差相对较少的一组作为最优解(匈牙利算法，基于glpk求解)
	std::vector<std::pair<int, int>> assigns = AssignOprimization(norm_cost);

	//计算残差和
	for (auto assign : assigns) {
		int& ni = assign.first;
		int& mi = assign.second;
		if (mi >= m) {										//防止在欠定指派问题情况下的越界访问无效数据
			continue;
		}
		r_time += cost[ni][mi].r_time;
		r_power += cost[ni][mi].r_power;
	}
}

void CalculateSensorCollectionResidual_TOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, const WeightFactor& w, RtLbsType& r_time, RtLbsType& r_power, int& nullDataNum)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_time = 0.0;
		RtLbsType cur_r_power = 0.0;
		int cur_nullDataNum = 0;
		CalculateSensorResidual_TOA_MultiData(c1[i], c2[i], w, cur_r_time, cur_r_power, cur_nullDataNum);
		r_time += cur_r_time;
		r_power += cur_r_power;
		nullDataNum += cur_nullDataNum;
	}
}

void CalculateSensorResidual_AOATOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_time, RtLbsType& r_power)
{
	for (int i = 0; i < static_cast<int>(c1.m_datas.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		RtLbsType cur_r_time = 0;
		RtLbsType cur_r_power = 0;
		CalculateResidual_AOA_TOA(c1.m_datas[i], c2.m_datas[i], cur_r_phi, cur_r_time, cur_r_power);
		r_phi += cur_r_phi;
		r_time += cur_r_time;
		r_power += cur_r_power;
	}
}

void CalculateSensorCollectionResidual_AOATOA_SingleData(std::vector<SensorDataCollection>& c1, std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_time, RtLbsType& r_power, int& nullDataNum)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		if (c2[i].m_datas.size() == 0) {
			nullDataNum++;
			continue;
		}
		RtLbsType cur_r_phi = 0;
		RtLbsType cur_r_time = 0;
		RtLbsType cur_r_power = 0;
		CalculateSensorResidual_AOATOA_SingleData(c1[i], c2[i], cur_r_phi, cur_r_time, cur_r_power);
		r_phi += cur_r_phi;
		r_time += cur_r_time;
		r_power += cur_r_power;
	}
}

void CalculateSensorResidual_AOATOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, const WeightFactor& w, RtLbsType& r_phi, RtLbsType& r_time, RtLbsType& r_power, int& nullDataNum)
{
	/** @brief	残差表达	*/
	struct Residual {
		RtLbsType r_phi;						/** @brief	角度残差	*/
		RtLbsType r_time;						/** @brief	时间残差	*/
		RtLbsType r_power;						/** @brief	功率残差	*/
		RtLbsType r_norm;						/** @brief	归一化残差	*/
		void CalNormResidual(RtLbsType max_r_phi, RtLbsType max_r_time, RtLbsType max_r_power, const WeightFactor& w) {			//计算归一化残差
			r_norm = w.m_phiWeight*r_phi/max_r_phi + w.m_timeWeight * r_time / max_r_time + w.m_powerWeight * r_power / max_r_power;
		}
	};

	//构建残差矩阵
	int n = static_cast<int>(c1.m_datas.size());									/** @brief	原始传感器数据维度	*/
	int m = static_cast<int>(c2.m_datas.size());									/** @brief	目标传感器数据维度	*/

	RtLbsType max_r_phi = 0.0;													/** @brief	最大角度残差	*/
	RtLbsType max_r_time = 0.0;													/** @brief	最大时间残差	*/
	RtLbsType max_r_power = 0.0;												/** @brief	最大功率残差	*/

	std::vector<std::vector<Residual>> cost(n, std::vector<Residual>(m));						/** @brief	残差矩阵	*/
	std::vector<std::vector<RtLbsType>> norm_cost(n, std::vector<RtLbsType>(m));				/** @brief	归一化代价矩阵	*/
	for (int i = 0; i < n; ++i) {
		norm_cost[i].assign(m, 0);
		for (int j = 0; j < m; ++j) {
			RtLbsType cur_r_phi = std::abs(c1.m_datas[i].m_phi - c2.m_datas[j].m_phi);
			if (cur_r_phi > PI) { cur_r_phi -= PI; }
			RtLbsType cur_r_time = std::abs(c1.m_datas[i].m_time - c2.m_datas[j].m_time) * 1e9;
			RtLbsType cur_r_power = abs(c1.m_datas[i].m_power - c2.m_datas[j].m_power);
			cost[i][j].r_phi = cur_r_phi;
			cost[i][j].r_time = cur_r_time;
			cost[i][j].r_power = cur_r_power;
			max_r_phi = std::max(max_r_phi, cost[i][j].r_phi);
			max_r_time = std::max(max_r_time, cost[i][j].r_time);
			max_r_power = std::max(max_r_power, cost[i][j].r_power);
		}
	}

	//将矩阵进行归一化处理
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			cost[i][j].CalNormResidual(max_r_phi, max_r_time, max_r_power, w);
			norm_cost[i][j] = cost[i][j].r_norm;
		}
	}
	int new_m = m;
	//处理特殊情况-m的值小于n的值，属于欠缺分配问题，需要进行补全
	if (m < n) {
		new_m = n;
		for (int i = 0; i < n; ++i) {
			for (int j = m - 1; j < n; ++j) {
				norm_cost[i].push_back(2.0);					//增广代价矩阵
			}
		}
		nullDataNum = n - m;									//赋值无数据数量
	}

	//选取残差相对较少的一组作为最优解(匈牙利算法，基于glpk求解)
	std::vector<std::pair<int, int>> assigns = AssignOprimization(norm_cost);

	//计算残差和
	for (auto assign : assigns) {
		int& ni = assign.first;
		int& mi = assign.second;
		if (mi >= m) {										//防止在欠定指派问题情况下的越界访问无效数据
			continue;
		}
		r_phi += cost[ni][mi].r_phi;
		r_time += cost[ni][mi].r_time;
		r_power += cost[ni][mi].r_power;
	}
}

void CalculateSensorCollectionResidual_AOATOA_MultiData(std::vector<SensorDataCollection>& c1, std::vector<SensorDataCollection>& c2, const WeightFactor& w, RtLbsType& r_phi, RtLbsType& r_time, RtLbsType& r_power, int& nullDataNum)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0.0;
		RtLbsType cur_r_time = 0.0;
		RtLbsType cur_r_power = 0.0;
		int cur_nullDataNum = 0;
		CalculateSensorResidual_AOATOA_MultiData(c1[i], c2[i], w, cur_r_phi, cur_r_time, cur_r_power, cur_nullDataNum);
		r_phi += cur_r_phi;
		r_time += cur_r_time;
		r_power += cur_r_power;
		nullDataNum += cur_nullDataNum;
	}
}

void CalculateSensorResidual_TDOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_timeDiff)
{
	//计算时延差残差二范数和
	for (int i = 0; i < static_cast<int>(c1.m_datas.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_datas[i], c2.m_datas[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
}

void CalculateSensorCollectionResidual_TDOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff, int& nullDataNum)
{
	//计算时延差残差二范数和
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		if (c2[i].m_datas.size() == 0) {
			nullDataNum++;
			continue;
		}
		RtLbsType cur_r_timeDiff = 0;
		CalculateSensorResidual_TDOA_SingleData(c1[i], c2[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}

	//计算功率差残差二范数和
	for (int i = 1; i < static_cast<int>(c1.size()); ++i) {
		if (c2[i].m_datas.size() == 0) {
			continue;
		}
		RtLbsType powerDiff1 = c1[i].m_datas[0].m_power - c1[0].m_datas[0].m_power;
		RtLbsType powerDiff2 = c2[i].m_datas[0].m_power - c2[0].m_datas[0].m_power;
		r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
	}
}

void CalculateSensorResidual_TDOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, const WeightFactor& w, RtLbsType& r_timeDiff, RtLbsType& r_powerdiff, int& nullDataNum)
{
	//比较前需要确定c1和出的顺序都要按照功率从高到低的排序进行，并且以功率最大的多径作为能量主参考径来估计时延差

	/** @brief	残差表达	*/
	struct Residual {
		RtLbsType r_timeDiff;					/** @brief	时差残差	*/
		RtLbsType r_powerDiff;					/** @brief	功率差残差	*/
		RtLbsType r_norm;						/** @brief	归一化残差	*/
		void CalNormResidual(const WeightFactor& w, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff) {			//计算归一化残差
			r_norm = w.m_timeWeight * r_timeDiff / max_r_timeDiff + w.m_powerWeight * r_powerDiff / max_r_powerDiff;
		}
	};

	//求解残差代价矩阵
	int n = static_cast<int>(c1.m_datas.size());															/** @brief	原始数据的长度	*/
	int m = static_cast<int>(c2.m_datas.size());															/** @brief	预测数据的长度	*/


	RtLbsType max_r_timeDiff = 0.0;												/** @brief	最大时间残差	*/
	RtLbsType max_r_powerDiff = 0.0;											/** @brief	最大功率差残差	*/

	std::vector<std::vector<Residual>> cost(n, std::vector<Residual>(m));
	std::vector<std::vector<RtLbsType>> norm_cost(n, std::vector<RtLbsType>(m));							/** @brief	归一化代价矩阵	*/
	for (int i = 0; i < n; ++i) {
		norm_cost[i].assign(m, 0);
		for (int j = 0; j < m; ++j) {
			RtLbsType cur_r_timeDiff = (c1.m_datas[i].m_timeDiff - c2.m_datas[j].m_timeDiff) * 1e9;
			RtLbsType cur_r_powerDiff = (c1.m_datas[i].m_power - c2.m_datas[j].m_power);
			cost[i][j].r_timeDiff = cur_r_timeDiff * cur_r_timeDiff;
			cost[i][j].r_powerDiff = cur_r_powerDiff * cur_r_powerDiff;
			max_r_timeDiff = std::max(max_r_timeDiff, cost[i][j].r_timeDiff);
			max_r_powerDiff = std::max(max_r_powerDiff, cost[i][j].r_powerDiff);
		}
	}

	//防止出现除0错误
	if (max_r_timeDiff == 0) {
		max_r_timeDiff = 1.0;
	}
	if (max_r_powerDiff == 0) {
		max_r_powerDiff = 1.0;
	}

	//将矩阵进行归一化处理
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			cost[i][j].CalNormResidual(w, max_r_timeDiff, max_r_powerDiff);
			norm_cost[i][j] = cost[i][j].r_norm;
		}
	}
	int new_m = m;
	//处理特殊情况-m的值小于n的值，属于欠缺分配问题，需要进行补全
	if (m < n) {
		new_m = n;
		for (int i = 0; i < n; ++i) {
			for (int j = m - 1; j < n; ++j) {
				norm_cost[i].push_back(2.0);					//增广代价矩阵
			}
		}
		nullDataNum = n - m;									//赋值无数据数量
	}

	//选取残差相对较少的一组作为最优解(匈牙利算法，基于glpk求解)
	std::vector<std::pair<int, int>> assigns = AssignOprimization(norm_cost);

	//计算残差和
	for (auto assign : assigns) {
		int& ni = assign.first;
		int& mi = assign.second;
		if (mi >= m) {											//防止在欠定指派问题情况下的越界访问无效数据
			continue;
		}
		r_timeDiff += cost[ni][mi].r_timeDiff;
		r_powerdiff += cost[ni][mi].r_powerDiff;
	}
}

void CalculateSensorCollectionResidual_TDOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, const WeightFactor& w, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff, int& nullDataNum)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0.0;
		RtLbsType cur_r_powerDiff = 0.0;
		int cur_nullDataNum = 0;
		CalculateSensorResidual_TDOA_MultiData(c1[i], c2[i], w, cur_r_timeDiff, cur_r_powerDiff, cur_nullDataNum);
		r_timeDiff += cur_r_timeDiff;
		r_powerDiff += cur_r_powerDiff;
		nullDataNum += cur_nullDataNum;
	}
	//TDOA 算法需要增加限制条件，若仿真多径远大于实测多径，则增加nulldata的数量，用以限制缺陷
	int tuning_pathinfosize = c2[0].m_datas.size() - c1[0].m_datas.size();
	if (tuning_pathinfosize >= 2) {							//设置多径差额数量为2
		nullDataNum += tuning_pathinfosize - 1;
	}
}

void CalculateSensorResidual_AOATDOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff)
{
	//计算角度残差二范数和
	for (int i = 0; i < static_cast<int>(c1.m_datas.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateResidual_AOA(c1.m_datas[i], c2.m_datas[i], cur_r_phi);
		r_phi += cur_r_phi;
	}

	//计算时延差残差二范数和
	for (int i = 0; i < static_cast<int>(c1.m_datas.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_datas[i], c2.m_datas[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
}

void CalculateSensorCollectionResidual_AOATDOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff, int& nullDataNum)
{
	//计算时延差、角度残差二范数和
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		if (c2[i].m_datas.size() == 0) {
			nullDataNum++;
			continue;
		}
		RtLbsType cur_r_phi = 0;
		RtLbsType cur_r_timeDiff = 0;
		CalculateSensorResidual_AOATDOA_SingleData(c1[i], c2[i], cur_r_phi, cur_r_timeDiff);
		r_phi += cur_r_phi;
		r_timeDiff += cur_r_timeDiff;

		if (i >= 1) {//计算功率差残差二范数和
			RtLbsType powerDiff1 = c1[i].m_datas[0].m_power - c1[0].m_datas[0].m_power;
			RtLbsType powerDiff2 = 0.0;
			if (c2[0].m_datas.size() == 0) {
				powerDiff2 = c2[i].m_datas[0].m_power;
			}
			else {
				powerDiff2 = c2[i].m_datas[0].m_power - c2[0].m_datas[0].m_power;
			}
			r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
		}
	}
}


void CalculateSensorResidual_AOATDOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, const WeightFactor& w, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerdiff, int& nullDataNum)
{
	//比较前需要确定c1和出的顺序都要按照时延从高到低的排序进行，并且以时延最小的多径作为主参考径来估计时延差

	/** @brief	残差表达	*/
	struct Residual {
		RtLbsType r_phi;						/** @brief	角度残差	*/
		RtLbsType r_timeDiff;					/** @brief	时差残差	*/
		RtLbsType r_powerDiff;					/** @brief	功率差残差	*/
		RtLbsType r_norm;						/** @brief	归一化残差	*/
		void CalNormResidual(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w) {			//计算归一化残差
			r_norm = w.m_phiWeight * r_phi / max_r_phi + w.m_timeWeight * r_timeDiff / max_r_timeDiff + w.m_powerWeight * r_powerDiff / max_r_powerDiff;
		}
	};


	//求解残差代价矩阵,TDOA算法中，第一元素为参考时延，固定指派
	int n = static_cast<int>(c1.m_datas.size() - 1);															/** @brief	原始数据的长度	*/
	int m = static_cast<int>(c2.m_datas.size() - 1);															/** @brief	预测数据的长度	*/

	RtLbsType max_r_phi = 0.0;													/** @brief	最大角度残差	*/
	RtLbsType max_r_timeDiff = 0.0;												/** @brief	最大时间残差	*/
	RtLbsType max_r_powerDiff = 0.0;											/** @brief	最大功率差残差	*/

	RtLbsType first_r_phi = c1.m_datas[0].m_phi - c2.m_datas[0].m_phi;
	RtLbsType first_r_timeDiff = c1.m_datas[0].m_timeDiff - c2.m_datas[0].m_timeDiff;
	RtLbsType first_r_powerDiff = c1.m_datas[0].m_power - c2.m_datas[0].m_power;

	max_r_phi = first_r_phi* first_r_phi;
	max_r_timeDiff = first_r_timeDiff* first_r_timeDiff;
	max_r_powerDiff = first_r_powerDiff* first_r_powerDiff;

	r_phi += max_r_phi;
	r_timeDiff += max_r_timeDiff;
	r_powerdiff += max_r_powerDiff;


	std::vector<std::vector<Residual>> cost(n, std::vector<Residual>(m));
	std::vector<std::vector<RtLbsType>> norm_cost(n, std::vector<RtLbsType>(m));							/** @brief	归一化代价矩阵	*/
	for (int i = 0; i < n; ++i) {
		norm_cost[i].assign(m, 0);
		for (int j = 0; j < m; ++j) {
			RtLbsType cur_r_phi = std::abs(c1.m_datas[i + 1].m_phi - c2.m_datas[j + 1].m_phi);
			RtLbsType cur_r_timeDiff = (c1.m_datas[i + 1].m_timeDiff - c2.m_datas[j + 1].m_timeDiff) * 1e9;
			RtLbsType cur_r_powerDiff = c1.m_datas[i + 1].m_power - c2.m_datas[j + 1].m_power;
			cost[i][j].r_phi = cur_r_phi * cur_r_phi;
			cost[i][j].r_timeDiff = cur_r_timeDiff * cur_r_timeDiff;
			cost[i][j].r_powerDiff = cur_r_powerDiff * cur_r_powerDiff;
			max_r_phi = std::max(max_r_phi, cost[i][j].r_phi);
			max_r_timeDiff = std::max(max_r_timeDiff, cost[i][j].r_timeDiff);
			max_r_powerDiff = std::max(max_r_powerDiff, cost[i][j].r_powerDiff);
		}
	}

	//防止出现除0错误
	if (max_r_phi == 0) {
		max_r_phi = 1.0;
	}
	if (max_r_timeDiff == 0) {
		max_r_timeDiff = 1.0;
	}
	if (max_r_powerDiff == 0) {
		max_r_powerDiff = 1.0;
	}

	//将矩阵进行归一化处理
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			cost[i][j].CalNormResidual(max_r_phi, max_r_timeDiff, max_r_powerDiff, w);
			norm_cost[i][j] = cost[i][j].r_norm;
		}
	}
	int new_m = m;
	//处理特殊情况-m的值小于n的值，属于欠缺分配问题，需要进行补全
	if (m < n) {
		new_m = n;
		for (int i = 0; i < n; ++i) {
			for (int j = m - 1; j < n; ++j) {
				norm_cost[i].push_back(2.0);					//增广代价矩阵
			}
		}
		nullDataNum = n - m;									//赋值无数据数量
	}

	//选取残差相对较少的一组作为最优解(匈牙利算法，基于glpk求解)
	std::vector<std::pair<int, int>> assigns = AssignOprimization(norm_cost);

	//计算残差和
	for (auto assign : assigns) {
		int& ni = assign.first;
		int& mi = assign.second;
		if (mi >= m) {												//防止在欠定指派问题情况下的越界访问无效数据
			continue;
		}
		r_phi += cost[ni][mi].r_phi;
		r_timeDiff += cost[ni][mi].r_timeDiff;
		r_powerdiff += cost[ni][mi].r_powerDiff;
	}

}

void CalculateSensorCollectionResidual_AOATDOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, const WeightFactor& w, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff, int& nullDataNum)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0.0;
		RtLbsType cur_r_timeDiff = 0.0;
		RtLbsType cur_r_powerDiff = 0.0;
		int cur_nullDataNum = 0;
		CalculateSensorResidual_AOATDOA_MultiData(c1[i], c2[i], w, cur_r_phi, cur_r_timeDiff, cur_r_powerDiff, cur_nullDataNum);
		r_phi += cur_r_phi;
		r_timeDiff += cur_r_timeDiff;
		r_powerDiff += cur_r_powerDiff;
		nullDataNum += cur_nullDataNum;
	}
}