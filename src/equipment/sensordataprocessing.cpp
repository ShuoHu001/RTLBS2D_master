#include "sensordataprocessing.h"


void CalculateResidual_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi)
{
	r_phi = (d1.m_phi - d2.m_phi) * (d1.m_phi - d2.m_phi);								//计算角度残差平方
}

void CalculateResidual_TDOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_timediff)
{
	r_timediff = (d1.m_timeDiff - d2.m_timeDiff) * (d1.m_timeDiff - d2.m_timeDiff);		//计算时间差残差平方
}

void CalculateResidual_TDOA_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi, RtLbsType& r_timediff)
{
	r_phi = (d1.m_phi - d2.m_phi) * (d1.m_phi - d2.m_phi);								//计算角度残差平方
	r_timediff = (d1.m_timeDiff - d2.m_timeDiff) * (d1.m_timeDiff - d2.m_timeDiff);		//计算时间差残差平方
}

bool ComparedByPower_SensorDataCollection(const SensorDataCollection& c1, const SensorDataCollection& c2) {
	return c1.m_data[0].m_power > c2.m_data[0].m_power;
}

void CalculateSensorResidual_AOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi)
{
	//计算角度残差二范数和
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateResidual_AOA(c1.m_data[i], c2.m_data[i], cur_r_phi);
		r_phi += cur_r_phi;
	}
}

void CalculateSensorCollectionResidual_AOA_SingleData(std::vector<SensorDataCollection>& c1, std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_powerDiff)
{

	//计算角度残差二范数和
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateSensorResidual_AOA_SingleData(c1[i], c2[i], cur_r_phi);
		r_phi += cur_r_phi;
	}

	//计算功率差残差二范数和
	for (int i = 1; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType powerDiff1 = c1[i].m_data[0].m_power - c1[0].m_data[0].m_power;
		RtLbsType powerDiff2 = c2[i].m_data[0].m_power - c2[0].m_data[0].m_power;
		r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
	}

}

inline void CalculateSensorResidual_AOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_powerdiff, int nullDataNum)
{
	/** @brief	残差表达	*/
	struct Residual {
		RtLbsType r_phi;						/** @brief	角度残差	*/
		RtLbsType r_powerDiff;					/** @brief	功率差残差	*/
		RtLbsType r_norm;						/** @brief	归一化残差	*/
		void CalNormResidual(RtLbsType max_r_phi, RtLbsType max_r_powerDiff) {			//计算归一化残差
			r_norm = 0.5 * r_phi / max_r_phi + 0.5 * r_powerDiff / max_r_powerDiff;
		}
	};
	//构建残差矩阵
	int n = static_cast<int>(c1.m_data.size());									/** @brief	原始传感器数据维度	*/
	int m = static_cast<int>(c2.m_data.size());									/** @brief	目标传感器数据维度	*/

	RtLbsType max_r_phi = 0.0;													/** @brief	最大角度残差	*/
	RtLbsType max_r_powerDiff = 0.0;											/** @brief	最大功率差残差	*/

	std::vector<std::vector<Residual>> cost(n, std::vector<Residual>(m));					/** @brief	残差矩阵	*/
	std::vector<std::vector<RtLbsType>> norm_cost(n, std::vector<RtLbsType>(m));				/** @brief	归一化代价矩阵	*/
	for (int i = 0; i < n; ++i) {
		norm_cost[i].assign(m, 0);
		for (int j = 0; j < m; ++j) {
			RtLbsType cur_r_phi = (c1.m_data[i].m_phi - c2.m_data[j].m_phi) * (c1.m_data[i].m_phi - c2.m_data[j].m_phi);
			RtLbsType cur_r_powerDiff = (c1.m_data[i].m_power - c2.m_data[j].m_power) * (c1.m_data[i].m_power - c2.m_data[j].m_power);
			cost[i][j].r_phi = cur_r_phi;
			cost[i][j].r_powerDiff = cur_r_powerDiff;
			if (max_r_phi < cur_r_phi) {
				max_r_phi = cur_r_phi;
			}
			if (max_r_powerDiff < cur_r_powerDiff) {
				max_r_powerDiff = cur_r_powerDiff;
			}
		}
	}

	//将矩阵进行归一化处理
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			cost[i][j].CalNormResidual(max_r_phi, max_r_powerDiff);
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
}

void CalculateSensorCollectionResidual_AOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_powerDiff, int& nullDataNum)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0.0;
		RtLbsType cur_r_powerDiff = 0.0;
		int cur_nullDataNum = 0;
		CalculateSensorResidual_AOA_MultiData(c1[i], c2[i], cur_r_phi, cur_r_powerDiff, cur_nullDataNum);
		r_phi += cur_r_phi;
		r_powerDiff += cur_r_powerDiff;
		nullDataNum += cur_nullDataNum;
	}
}

void CalculateSensorResidual_TDOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_timeDiff)
{
	//计算时延差残差二范数和
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_data[i], c2.m_data[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
}

void CalculateSensorCollectionResidual_TDOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff)
{
	//计算时延差残差二范数和
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateSensorResidual_TDOA_SingleData(c1[i], c2[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}

	//计算功率差残差二范数和
	for (int i = 1; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType powerDiff1 = c1[i].m_data[0].m_power - c1[0].m_data[0].m_power;
		RtLbsType powerDiff2 = c2[i].m_data[0].m_power - c2[0].m_data[0].m_power;
		r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
	}
}

void CalculateSensorResidual_TDOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerdiff, int& nullDataNum)
{
	//比较前需要确定c1和出的顺序都要按照功率从高到低的排序进行，并且以功率最大的多径作为能量主参考径来估计时延差

	/** @brief	残差表达	*/
	struct Residual {
		RtLbsType r_timeDiff;					/** @brief	时差残差	*/
		RtLbsType r_powerDiff;					/** @brief	功率差残差	*/
		RtLbsType r_norm;						/** @brief	归一化残差	*/
		void CalNormResidual(RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff) {			//计算归一化残差
			r_norm = 0.5 * r_timeDiff / max_r_timeDiff + 0.5 * r_powerDiff / max_r_powerDiff;
		}
	};

	//求解残差代价矩阵
	int n = static_cast<int>(c1.m_data.size());															/** @brief	原始数据的长度	*/
	int m = static_cast<int>(c2.m_data.size());															/** @brief	预测数据的长度	*/


	RtLbsType max_r_timeDiff = 0.0;												/** @brief	最大时间残差	*/
	RtLbsType max_r_powerDiff = 0.0;											/** @brief	最大功率差残差	*/

	std::vector<std::vector<Residual>> cost(n);
	std::vector<std::vector<RtLbsType>> norm_cost(n);							/** @brief	归一化代价矩阵	*/
	for (int i = 0; i < n; ++i) {
		cost[n].resize(m);
		norm_cost[n].resize(m);
		norm_cost[n].assign(m, 0);
		for (int j = 0; j < m; ++j) {
			RtLbsType cur_r_timeDiff = (c1.m_data[i].m_timeDiff - c2.m_data[j].m_timeDiff) * (c1.m_data[i].m_timeDiff - c2.m_data[j].m_timeDiff);
			RtLbsType cur_r_powerDiff = (c1.m_data[i].m_power - c2.m_data[j].m_power) * (c1.m_data[i].m_power - c2.m_data[j].m_power);
			cost[i][j].r_timeDiff = cur_r_timeDiff;
			cost[i][j].r_powerDiff = cur_r_powerDiff;
			if (max_r_timeDiff < cur_r_timeDiff) {
				max_r_timeDiff = cur_r_timeDiff;
			}
			if (max_r_powerDiff < cur_r_powerDiff) {
				max_r_powerDiff = cur_r_powerDiff;
			}
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
			cost[i][j].CalNormResidual(max_r_timeDiff, max_r_powerDiff);
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
		int& ni = assign.second;
		int& mi = assign.second;
		if (mi >= m) {											//防止在欠定指派问题情况下的越界访问无效数据
			continue;
		}
		r_timeDiff += cost[ni][mi].r_timeDiff;
		r_powerdiff += cost[ni][mi].r_powerDiff;
	}
}

void CalculateSensorCollectionResidual_TDOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff, int& nullDataNum)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0.0;
		RtLbsType cur_r_powerDiff = 0.0;
		int cur_nullDataNum = 0;
		CalculateSensorResidual_TDOA_MultiData(c1[i], c2[i], cur_r_timeDiff, cur_r_powerDiff, cur_nullDataNum);
		r_timeDiff += cur_r_timeDiff;
		r_powerDiff += cur_r_powerDiff;
		nullDataNum += cur_nullDataNum;
	}
}

void CalculateSensorResidual_TDOA_AOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff)
{
	//计算角度残差二范数和
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateResidual_AOA(c1.m_data[i], c2.m_data[i], cur_r_phi);
		r_phi += cur_r_phi;
	}

	//计算时延差残差二范数和
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_data[i], c2.m_data[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
}

void CalculateSensorCollectionResidual_TDOA_AOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff)
{
	//计算时延差、角度残差二范数和
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		RtLbsType cur_r_timeDiff = 0;
		CalculateSensorResidual_TDOA_AOA_SingleData(c1[i], c2[i], cur_r_phi, cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}

	//计算功率差残差二范数和
	for (int i = 1; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType powerDiff1 = c1[i].m_data[0].m_power - c1[0].m_data[0].m_power;
		RtLbsType powerDiff2 = c2[i].m_data[0].m_power - c2[0].m_data[0].m_power;
		r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
	}
}


void CalculateSensorResidual_TDOA_AOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerdiff, int& nullDataNum)
{
	//比较前需要确定c1和出的顺序都要按照功率从高到低的排序进行，并且以功率最大的多径作为能量主参考径来估计时延差

	/** @brief	残差表达	*/
	struct Residual {
		RtLbsType r_phi;						/** @brief	角度残差	*/
		RtLbsType r_timeDiff;					/** @brief	时差残差	*/
		RtLbsType r_powerDiff;					/** @brief	功率差残差	*/
		RtLbsType r_norm;						/** @brief	归一化残差	*/
		void CalNormResidual(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff) {			//计算归一化残差
			r_norm = ONE_THIRD * r_phi / max_r_phi + ONE_THIRD * r_timeDiff / max_r_timeDiff + ONE_THIRD * r_powerDiff / max_r_powerDiff;
		}
	};


	//求解残差代价矩阵
	int n = static_cast<int>(c1.m_data.size());															/** @brief	原始数据的长度	*/
	int m = static_cast<int>(c2.m_data.size());															/** @brief	预测数据的长度	*/

	RtLbsType max_r_phi = 0.0;													/** @brief	最大角度残差	*/
	RtLbsType max_r_timeDiff = 0.0;												/** @brief	最大时间残差	*/
	RtLbsType max_r_powerDiff = 0.0;											/** @brief	最大功率差残差	*/

	std::vector<std::vector<Residual>> cost(n);
	std::vector<std::vector<RtLbsType>> norm_cost(n);							/** @brief	归一化代价矩阵	*/
	for (int i = 0; i < n; ++i) {
		cost[n].resize(m);
		norm_cost[n].resize(m);
		norm_cost[n].assign(m, 0);
		for (int j = 0; j < m; ++j) {
			RtLbsType cur_r_phi = (c1.m_data[i].m_phi - c2.m_data[j].m_phi) * (c1.m_data[i].m_phi - c2.m_data[j].m_phi);
			RtLbsType cur_r_timeDiff = (c1.m_data[i].m_timeDiff - c2.m_data[j].m_timeDiff) * (c1.m_data[i].m_timeDiff - c2.m_data[j].m_timeDiff);
			RtLbsType cur_r_powerDiff = (c1.m_data[i].m_power - c2.m_data[j].m_power) * (c1.m_data[i].m_power - c2.m_data[j].m_power);
			cost[i][j].r_phi = cur_r_phi;
			cost[i][j].r_timeDiff = cur_r_timeDiff;
			cost[i][j].r_powerDiff = cur_r_powerDiff;
			if (max_r_phi < cur_r_phi) {
				max_r_phi = cur_r_phi;
			}
			if (max_r_timeDiff < cur_r_timeDiff) {
				max_r_timeDiff = cur_r_timeDiff;
			}
			if (max_r_powerDiff < cur_r_powerDiff) {
				max_r_powerDiff = cur_r_powerDiff;
			}
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
			cost[i][j].CalNormResidual(max_r_phi, max_r_timeDiff, max_r_powerDiff);
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
		int& ni = assign.second;
		int& mi = assign.second;
		if (mi >= m) {												//防止在欠定指派问题情况下的越界访问无效数据
			continue;
		}
		r_phi += cost[ni][mi].r_phi;
		r_timeDiff += cost[ni][mi].r_timeDiff;
		r_powerdiff += cost[ni][mi].r_powerDiff;
	}

}

void CalculateSensorCollectionResidual_TDOA_AOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff, int& nullDataNum)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0.0;
		RtLbsType cur_r_timeDiff = 0.0;
		RtLbsType cur_r_powerDiff = 0.0;
		int cur_nullDataNum = 0;
		CalculateSensorResidual_TDOA_AOA_MultiData(c1[i], c2[i], cur_r_phi, cur_r_timeDiff, cur_r_powerDiff, cur_nullDataNum);
		r_phi += cur_r_phi;
		r_timeDiff += cur_r_timeDiff;
		r_powerDiff += cur_r_powerDiff;
		nullDataNum += cur_nullDataNum;
	}
}