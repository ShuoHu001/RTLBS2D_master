#include "sensordataprocessing.h"


void CalculateResidual_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi)
{
	RtLbsType cur_r_phi = std::abs(d1.m_phi - d2.m_phi);
	if (cur_r_phi > PI) {										//����Խ��ǶȲв�
		cur_r_phi  = TWO_PI - cur_r_phi;
	}
	r_phi = cur_r_phi * cur_r_phi;								//����ǶȲв�ƽ��
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
	r_timediff = cur_r_timeDiff * cur_r_timeDiff;		//����ʱ���в�ƽ��
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
	r_phi = cur_r_phi * cur_r_phi;								//����ǶȲв�ƽ��
	r_timediff = cur_r_timeDiff * cur_r_timeDiff;				//����ʱ���в�ƽ��
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
	//����ǶȲв��������
	for (int i = 0; i < static_cast<int>(c1.m_datas.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateResidual_AOA(c1.m_datas[i], c2.m_datas[i], cur_r_phi);
		r_phi += cur_r_phi;
	}
}

void CalculateSensorCollectionResidual_AOA_SingleData(std::vector<SensorDataCollection>& c1, std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_powerDiff, int& nullDataNum)
{

	//����ǶȲв��������
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		if (c2[i].m_datas.size() == 0) {
			nullDataNum++;
			continue;
		}
		RtLbsType cur_r_phi = 0;
		CalculateSensorResidual_AOA_SingleData(c1[i], c2[i], cur_r_phi);
		r_phi += cur_r_phi;
	}

	//���㹦�ʲ�в��������
	if (c2[0].m_datas.size() == 0) {			//�ο�վ�޹���
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
	/** @brief	�в���	*/
	struct Residual {
		RtLbsType r_phi;						/** @brief	�ǶȲв�	*/
		RtLbsType r_powerDiff;					/** @brief	���ʲ�в�	*/
		RtLbsType r_norm;						/** @brief	��һ���в�	*/
		void CalNormResidual(RtLbsType max_r_phi, RtLbsType max_r_powerDiff, const WeightFactor& w) {			//�����һ���в�
			r_norm = w.m_phiWeight * r_phi / max_r_phi + w.m_powerWeight * r_powerDiff / max_r_powerDiff;
		}
	};
	//�����в����
	int n = static_cast<int>(c1.m_datas.size());									/** @brief	ԭʼ����������ά��	*/
	int m = static_cast<int>(c2.m_datas.size());									/** @brief	Ŀ�괫��������ά��	*/

	RtLbsType max_r_phi = 0.0;													/** @brief	���ǶȲв�	*/
	RtLbsType max_r_powerDiff = 0.0;											/** @brief	����ʲ�в�	*/

	std::vector<std::vector<Residual>> cost(n, std::vector<Residual>(m));					/** @brief	�в����	*/
	std::vector<std::vector<RtLbsType>> norm_cost(n, std::vector<RtLbsType>(m));				/** @brief	��һ�����۾���	*/
	for (int i = 0; i < n; ++i) {
		norm_cost[i].assign(m, 0);
		for (int j = 0; j < m; ++j) {

			RtLbsType cur_r_phi = std::abs(c1.m_datas[i].m_phi - c2.m_datas[j].m_phi);
			if (cur_r_phi > PI) {			//��֤�в���0-PI��(��С�в�׼��)
				cur_r_phi = TWO_PI - cur_r_phi;
			}
			RtLbsType cur_r_powerDiff = abs(c1.m_datas[i].m_power - c2.m_datas[j].m_power);
			cost[i][j].r_phi = cur_r_phi;
			cost[i][j].r_powerDiff = cur_r_powerDiff;
			max_r_phi = std::max(max_r_phi, cost[i][j].r_phi);
			max_r_powerDiff = std::max(max_r_powerDiff, cost[i][j].r_powerDiff);
		}
	}

	//��������й�һ������
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			cost[i][j].CalNormResidual(max_r_phi, max_r_powerDiff, w);
			norm_cost[i][j] = cost[i][j].r_norm;
		}
	}
	int new_m = m;
	//�����������-m��ֵС��n��ֵ������Ƿȱ�������⣬��Ҫ���в�ȫ
	if (m < n) {
		new_m = n;
		for (int i = 0; i < n; ++i) {
			for (int j = m - 1; j < n; ++j) {
				norm_cost[i].push_back(2.0);					//������۾���
			}
		}
		nullDataNum = n - m;									//��ֵ����������
	}

	//ѡȡ�в���Խ��ٵ�һ����Ϊ���Ž�(�������㷨������glpk���)
	std::vector<std::pair<int, int>> assigns = AssignOprimization(norm_cost);

	//����в��
	for (auto assign : assigns) {
		int& ni = assign.first;
		int& mi = assign.second;
		if (mi >= m) {										//��ֹ��Ƿ��ָ����������µ�Խ�������Ч����
			continue;
		}
		r_phi += cost[ni][mi].r_phi;
		r_powerdiff += cost[ni][mi].r_powerDiff;
	}

	////������ԽǶ���չ

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

	//����Ƕ���չ�в�
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
	/** @brief	�в���	*/
	struct Residual {
		RtLbsType r_time;						/** @brief	�ǶȲв�	*/
		RtLbsType r_power;						/** @brief	���ʲв�	*/
		RtLbsType r_norm;						/** @brief	��һ���в�	*/
		void CalNormResidual(RtLbsType max_r_time, RtLbsType max_r_power, const WeightFactor& w) {			//�����һ���в�
			r_norm = w.m_timeWeight * r_time / max_r_time + w.m_powerWeight * r_power / max_r_power;
		}
	};

	//�����в����
	int n = static_cast<int>(c1.m_datas.size());									/** @brief	ԭʼ����������ά��	*/
	int m = static_cast<int>(c2.m_datas.size());									/** @brief	Ŀ�괫��������ά��	*/

	RtLbsType max_r_time = 0.0;													/** @brief	���ʱ��в�	*/
	RtLbsType max_r_power = 0.0;												/** @brief	����ʲв�	*/

	std::vector<std::vector<Residual>> cost(n, std::vector<Residual>(m));						/** @brief	�в����	*/
	std::vector<std::vector<RtLbsType>> norm_cost(n, std::vector<RtLbsType>(m));				/** @brief	��һ�����۾���	*/
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

	//��������й�һ������
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			cost[i][j].CalNormResidual(max_r_time, max_r_power, w);
			norm_cost[i][j] = cost[i][j].r_norm;
		}
	}
	int new_m = m;
	//�����������-m��ֵС��n��ֵ������Ƿȱ�������⣬��Ҫ���в�ȫ
	if (m < n) {
		new_m = n;
		for (int i = 0; i < n; ++i) {
			for (int j = m - 1; j < n; ++j) {
				norm_cost[i].push_back(2.0);					//������۾���
			}
		}
		nullDataNum = n - m;									//��ֵ����������
	}

	//ѡȡ�в���Խ��ٵ�һ����Ϊ���Ž�(�������㷨������glpk���)
	std::vector<std::pair<int, int>> assigns = AssignOprimization(norm_cost);

	//����в��
	for (auto assign : assigns) {
		int& ni = assign.first;
		int& mi = assign.second;
		if (mi >= m) {										//��ֹ��Ƿ��ָ����������µ�Խ�������Ч����
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
	/** @brief	�в���	*/
	struct Residual {
		RtLbsType r_phi;						/** @brief	�ǶȲв�	*/
		RtLbsType r_time;						/** @brief	ʱ��в�	*/
		RtLbsType r_power;						/** @brief	���ʲв�	*/
		RtLbsType r_norm;						/** @brief	��һ���в�	*/
		void CalNormResidual(RtLbsType max_r_phi, RtLbsType max_r_time, RtLbsType max_r_power, const WeightFactor& w) {			//�����һ���в�
			r_norm = w.m_phiWeight*r_phi/max_r_phi + w.m_timeWeight * r_time / max_r_time + w.m_powerWeight * r_power / max_r_power;
		}
	};

	//�����в����
	int n = static_cast<int>(c1.m_datas.size());									/** @brief	ԭʼ����������ά��	*/
	int m = static_cast<int>(c2.m_datas.size());									/** @brief	Ŀ�괫��������ά��	*/

	RtLbsType max_r_phi = 0.0;													/** @brief	���ǶȲв�	*/
	RtLbsType max_r_time = 0.0;													/** @brief	���ʱ��в�	*/
	RtLbsType max_r_power = 0.0;												/** @brief	����ʲв�	*/

	std::vector<std::vector<Residual>> cost(n, std::vector<Residual>(m));						/** @brief	�в����	*/
	std::vector<std::vector<RtLbsType>> norm_cost(n, std::vector<RtLbsType>(m));				/** @brief	��һ�����۾���	*/
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

	//��������й�һ������
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			cost[i][j].CalNormResidual(max_r_phi, max_r_time, max_r_power, w);
			norm_cost[i][j] = cost[i][j].r_norm;
		}
	}
	int new_m = m;
	//�����������-m��ֵС��n��ֵ������Ƿȱ�������⣬��Ҫ���в�ȫ
	if (m < n) {
		new_m = n;
		for (int i = 0; i < n; ++i) {
			for (int j = m - 1; j < n; ++j) {
				norm_cost[i].push_back(2.0);					//������۾���
			}
		}
		nullDataNum = n - m;									//��ֵ����������
	}

	//ѡȡ�в���Խ��ٵ�һ����Ϊ���Ž�(�������㷨������glpk���)
	std::vector<std::pair<int, int>> assigns = AssignOprimization(norm_cost);

	//����в��
	for (auto assign : assigns) {
		int& ni = assign.first;
		int& mi = assign.second;
		if (mi >= m) {										//��ֹ��Ƿ��ָ����������µ�Խ�������Ч����
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
	//����ʱ�Ӳ�в��������
	for (int i = 0; i < static_cast<int>(c1.m_datas.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_datas[i], c2.m_datas[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
}

void CalculateSensorCollectionResidual_TDOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff, int& nullDataNum)
{
	//����ʱ�Ӳ�в��������
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		if (c2[i].m_datas.size() == 0) {
			nullDataNum++;
			continue;
		}
		RtLbsType cur_r_timeDiff = 0;
		CalculateSensorResidual_TDOA_SingleData(c1[i], c2[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}

	//���㹦�ʲ�в��������
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
	//�Ƚ�ǰ��Ҫȷ��c1�ͳ���˳��Ҫ���չ��ʴӸߵ��͵�������У������Թ������Ķྶ��Ϊ�������ο���������ʱ�Ӳ�

	/** @brief	�в���	*/
	struct Residual {
		RtLbsType r_timeDiff;					/** @brief	ʱ��в�	*/
		RtLbsType r_powerDiff;					/** @brief	���ʲ�в�	*/
		RtLbsType r_norm;						/** @brief	��һ���в�	*/
		void CalNormResidual(const WeightFactor& w, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff) {			//�����һ���в�
			r_norm = w.m_timeWeight * r_timeDiff / max_r_timeDiff + w.m_powerWeight * r_powerDiff / max_r_powerDiff;
		}
	};

	//���в���۾���
	int n = static_cast<int>(c1.m_datas.size());															/** @brief	ԭʼ���ݵĳ���	*/
	int m = static_cast<int>(c2.m_datas.size());															/** @brief	Ԥ�����ݵĳ���	*/


	RtLbsType max_r_timeDiff = 0.0;												/** @brief	���ʱ��в�	*/
	RtLbsType max_r_powerDiff = 0.0;											/** @brief	����ʲ�в�	*/

	std::vector<std::vector<Residual>> cost(n, std::vector<Residual>(m));
	std::vector<std::vector<RtLbsType>> norm_cost(n, std::vector<RtLbsType>(m));							/** @brief	��һ�����۾���	*/
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

	//��ֹ���ֳ�0����
	if (max_r_timeDiff == 0) {
		max_r_timeDiff = 1.0;
	}
	if (max_r_powerDiff == 0) {
		max_r_powerDiff = 1.0;
	}

	//��������й�һ������
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			cost[i][j].CalNormResidual(w, max_r_timeDiff, max_r_powerDiff);
			norm_cost[i][j] = cost[i][j].r_norm;
		}
	}
	int new_m = m;
	//�����������-m��ֵС��n��ֵ������Ƿȱ�������⣬��Ҫ���в�ȫ
	if (m < n) {
		new_m = n;
		for (int i = 0; i < n; ++i) {
			for (int j = m - 1; j < n; ++j) {
				norm_cost[i].push_back(2.0);					//������۾���
			}
		}
		nullDataNum = n - m;									//��ֵ����������
	}

	//ѡȡ�в���Խ��ٵ�һ����Ϊ���Ž�(�������㷨������glpk���)
	std::vector<std::pair<int, int>> assigns = AssignOprimization(norm_cost);

	//����в��
	for (auto assign : assigns) {
		int& ni = assign.first;
		int& mi = assign.second;
		if (mi >= m) {											//��ֹ��Ƿ��ָ����������µ�Խ�������Ч����
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
	//TDOA �㷨��Ҫ��������������������ྶԶ����ʵ��ྶ��������nulldata����������������ȱ��
	int tuning_pathinfosize = c2[0].m_datas.size() - c1[0].m_datas.size();
	if (tuning_pathinfosize >= 2) {							//���öྶ�������Ϊ2
		nullDataNum += tuning_pathinfosize - 1;
	}
}

void CalculateSensorResidual_AOATDOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff)
{
	//����ǶȲв��������
	for (int i = 0; i < static_cast<int>(c1.m_datas.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateResidual_AOA(c1.m_datas[i], c2.m_datas[i], cur_r_phi);
		r_phi += cur_r_phi;
	}

	//����ʱ�Ӳ�в��������
	for (int i = 0; i < static_cast<int>(c1.m_datas.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_datas[i], c2.m_datas[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
}

void CalculateSensorCollectionResidual_AOATDOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff, int& nullDataNum)
{
	//����ʱ�Ӳ�ǶȲв��������
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

		if (i >= 1) {//���㹦�ʲ�в��������
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
	//�Ƚ�ǰ��Ҫȷ��c1�ͳ���˳��Ҫ����ʱ�ӴӸߵ��͵�������У�������ʱ����С�Ķྶ��Ϊ���ο���������ʱ�Ӳ�

	/** @brief	�в���	*/
	struct Residual {
		RtLbsType r_phi;						/** @brief	�ǶȲв�	*/
		RtLbsType r_timeDiff;					/** @brief	ʱ��в�	*/
		RtLbsType r_powerDiff;					/** @brief	���ʲ�в�	*/
		RtLbsType r_norm;						/** @brief	��һ���в�	*/
		void CalNormResidual(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w) {			//�����һ���в�
			r_norm = w.m_phiWeight * r_phi / max_r_phi + w.m_timeWeight * r_timeDiff / max_r_timeDiff + w.m_powerWeight * r_powerDiff / max_r_powerDiff;
		}
	};


	//���в���۾���,TDOA�㷨�У���һԪ��Ϊ�ο�ʱ�ӣ��̶�ָ��
	int n = static_cast<int>(c1.m_datas.size() - 1);															/** @brief	ԭʼ���ݵĳ���	*/
	int m = static_cast<int>(c2.m_datas.size() - 1);															/** @brief	Ԥ�����ݵĳ���	*/

	RtLbsType max_r_phi = 0.0;													/** @brief	���ǶȲв�	*/
	RtLbsType max_r_timeDiff = 0.0;												/** @brief	���ʱ��в�	*/
	RtLbsType max_r_powerDiff = 0.0;											/** @brief	����ʲ�в�	*/

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
	std::vector<std::vector<RtLbsType>> norm_cost(n, std::vector<RtLbsType>(m));							/** @brief	��һ�����۾���	*/
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

	//��ֹ���ֳ�0����
	if (max_r_phi == 0) {
		max_r_phi = 1.0;
	}
	if (max_r_timeDiff == 0) {
		max_r_timeDiff = 1.0;
	}
	if (max_r_powerDiff == 0) {
		max_r_powerDiff = 1.0;
	}

	//��������й�һ������
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			cost[i][j].CalNormResidual(max_r_phi, max_r_timeDiff, max_r_powerDiff, w);
			norm_cost[i][j] = cost[i][j].r_norm;
		}
	}
	int new_m = m;
	//�����������-m��ֵС��n��ֵ������Ƿȱ�������⣬��Ҫ���в�ȫ
	if (m < n) {
		new_m = n;
		for (int i = 0; i < n; ++i) {
			for (int j = m - 1; j < n; ++j) {
				norm_cost[i].push_back(2.0);					//������۾���
			}
		}
		nullDataNum = n - m;									//��ֵ����������
	}

	//ѡȡ�в���Խ��ٵ�һ����Ϊ���Ž�(�������㷨������glpk���)
	std::vector<std::pair<int, int>> assigns = AssignOprimization(norm_cost);

	//����в��
	for (auto assign : assigns) {
		int& ni = assign.first;
		int& mi = assign.second;
		if (mi >= m) {												//��ֹ��Ƿ��ָ����������µ�Խ�������Ч����
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