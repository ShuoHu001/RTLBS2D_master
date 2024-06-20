#include "sensordataprocessing.h"


void CalculateResidual_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi)
{
	r_phi = (d1.m_phi - d2.m_phi) * (d1.m_phi - d2.m_phi);								//����ǶȲв�ƽ��
}

void CalculateResidual_TDOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_timediff)
{
	r_timediff = (d1.m_timeDiff - d2.m_timeDiff) * (d1.m_timeDiff - d2.m_timeDiff);		//����ʱ���в�ƽ��
}

void CalculateResidual_TDOA_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi, RtLbsType& r_timediff)
{
	r_phi = (d1.m_phi - d2.m_phi) * (d1.m_phi - d2.m_phi);								//����ǶȲв�ƽ��
	r_timediff = (d1.m_timeDiff - d2.m_timeDiff) * (d1.m_timeDiff - d2.m_timeDiff);		//����ʱ���в�ƽ��
}

bool ComparedByPower_SensorDataCollection(const SensorDataCollection& c1, const SensorDataCollection& c2) {
	return c1.m_data[0].m_power > c2.m_data[0].m_power;
}

void CalculateSensorResidual_AOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi)
{
	//����ǶȲв��������
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateResidual_AOA(c1.m_data[i], c2.m_data[i], cur_r_phi);
		r_phi += cur_r_phi;
	}
}

void CalculateSensorCollectionResidual_AOA_SingleData(std::vector<SensorDataCollection>& c1, std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_powerDiff)
{

	//����ǶȲв��������
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateSensorResidual_AOA_SingleData(c1[i], c2[i], cur_r_phi);
		r_phi += cur_r_phi;
	}

	//���㹦�ʲ�в��������
	for (int i = 1; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType powerDiff1 = c1[i].m_data[0].m_power - c1[0].m_data[0].m_power;
		RtLbsType powerDiff2 = c2[i].m_data[0].m_power - c2[0].m_data[0].m_power;
		r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
	}

}

inline void CalculateSensorResidual_AOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_powerdiff, int nullDataNum)
{
	/** @brief	�в���	*/
	struct Residual {
		RtLbsType r_phi;						/** @brief	�ǶȲв�	*/
		RtLbsType r_powerDiff;					/** @brief	���ʲ�в�	*/
		RtLbsType r_norm;						/** @brief	��һ���в�	*/
		void CalNormResidual(RtLbsType max_r_phi, RtLbsType max_r_powerDiff) {			//�����һ���в�
			r_norm = 0.5 * r_phi / max_r_phi + 0.5 * r_powerDiff / max_r_powerDiff;
		}
	};
	//�����в����
	int n = static_cast<int>(c1.m_data.size());									/** @brief	ԭʼ����������ά��	*/
	int m = static_cast<int>(c2.m_data.size());									/** @brief	Ŀ�괫��������ά��	*/

	RtLbsType max_r_phi = 0.0;													/** @brief	���ǶȲв�	*/
	RtLbsType max_r_powerDiff = 0.0;											/** @brief	����ʲ�в�	*/

	std::vector<std::vector<Residual>> cost(n, std::vector<Residual>(m));					/** @brief	�в����	*/
	std::vector<std::vector<RtLbsType>> norm_cost(n, std::vector<RtLbsType>(m));				/** @brief	��һ�����۾���	*/
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

	//��������й�һ������
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			cost[i][j].CalNormResidual(max_r_phi, max_r_powerDiff);
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
	//����ʱ�Ӳ�в��������
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_data[i], c2.m_data[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
}

void CalculateSensorCollectionResidual_TDOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff)
{
	//����ʱ�Ӳ�в��������
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateSensorResidual_TDOA_SingleData(c1[i], c2[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}

	//���㹦�ʲ�в��������
	for (int i = 1; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType powerDiff1 = c1[i].m_data[0].m_power - c1[0].m_data[0].m_power;
		RtLbsType powerDiff2 = c2[i].m_data[0].m_power - c2[0].m_data[0].m_power;
		r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
	}
}

void CalculateSensorResidual_TDOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerdiff, int& nullDataNum)
{
	//�Ƚ�ǰ��Ҫȷ��c1�ͳ���˳��Ҫ���չ��ʴӸߵ��͵�������У������Թ������Ķྶ��Ϊ�������ο���������ʱ�Ӳ�

	/** @brief	�в���	*/
	struct Residual {
		RtLbsType r_timeDiff;					/** @brief	ʱ��в�	*/
		RtLbsType r_powerDiff;					/** @brief	���ʲ�в�	*/
		RtLbsType r_norm;						/** @brief	��һ���в�	*/
		void CalNormResidual(RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff) {			//�����һ���в�
			r_norm = 0.5 * r_timeDiff / max_r_timeDiff + 0.5 * r_powerDiff / max_r_powerDiff;
		}
	};

	//���в���۾���
	int n = static_cast<int>(c1.m_data.size());															/** @brief	ԭʼ���ݵĳ���	*/
	int m = static_cast<int>(c2.m_data.size());															/** @brief	Ԥ�����ݵĳ���	*/


	RtLbsType max_r_timeDiff = 0.0;												/** @brief	���ʱ��в�	*/
	RtLbsType max_r_powerDiff = 0.0;											/** @brief	����ʲ�в�	*/

	std::vector<std::vector<Residual>> cost(n);
	std::vector<std::vector<RtLbsType>> norm_cost(n);							/** @brief	��һ�����۾���	*/
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
			cost[i][j].CalNormResidual(max_r_timeDiff, max_r_powerDiff);
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
		int& ni = assign.second;
		int& mi = assign.second;
		if (mi >= m) {											//��ֹ��Ƿ��ָ����������µ�Խ�������Ч����
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
	//����ǶȲв��������
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateResidual_AOA(c1.m_data[i], c2.m_data[i], cur_r_phi);
		r_phi += cur_r_phi;
	}

	//����ʱ�Ӳ�в��������
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_data[i], c2.m_data[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
}

void CalculateSensorCollectionResidual_TDOA_AOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff)
{
	//����ʱ�Ӳ�ǶȲв��������
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		RtLbsType cur_r_timeDiff = 0;
		CalculateSensorResidual_TDOA_AOA_SingleData(c1[i], c2[i], cur_r_phi, cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}

	//���㹦�ʲ�в��������
	for (int i = 1; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType powerDiff1 = c1[i].m_data[0].m_power - c1[0].m_data[0].m_power;
		RtLbsType powerDiff2 = c2[i].m_data[0].m_power - c2[0].m_data[0].m_power;
		r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
	}
}


void CalculateSensorResidual_TDOA_AOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerdiff, int& nullDataNum)
{
	//�Ƚ�ǰ��Ҫȷ��c1�ͳ���˳��Ҫ���չ��ʴӸߵ��͵�������У������Թ������Ķྶ��Ϊ�������ο���������ʱ�Ӳ�

	/** @brief	�в���	*/
	struct Residual {
		RtLbsType r_phi;						/** @brief	�ǶȲв�	*/
		RtLbsType r_timeDiff;					/** @brief	ʱ��в�	*/
		RtLbsType r_powerDiff;					/** @brief	���ʲ�в�	*/
		RtLbsType r_norm;						/** @brief	��һ���в�	*/
		void CalNormResidual(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff) {			//�����һ���в�
			r_norm = ONE_THIRD * r_phi / max_r_phi + ONE_THIRD * r_timeDiff / max_r_timeDiff + ONE_THIRD * r_powerDiff / max_r_powerDiff;
		}
	};


	//���в���۾���
	int n = static_cast<int>(c1.m_data.size());															/** @brief	ԭʼ���ݵĳ���	*/
	int m = static_cast<int>(c2.m_data.size());															/** @brief	Ԥ�����ݵĳ���	*/

	RtLbsType max_r_phi = 0.0;													/** @brief	���ǶȲв�	*/
	RtLbsType max_r_timeDiff = 0.0;												/** @brief	���ʱ��в�	*/
	RtLbsType max_r_powerDiff = 0.0;											/** @brief	����ʲ�в�	*/

	std::vector<std::vector<Residual>> cost(n);
	std::vector<std::vector<RtLbsType>> norm_cost(n);							/** @brief	��һ�����۾���	*/
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
			cost[i][j].CalNormResidual(max_r_phi, max_r_timeDiff, max_r_powerDiff);
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
		int& ni = assign.second;
		int& mi = assign.second;
		if (mi >= m) {												//��ֹ��Ƿ��ָ����������µ�Խ�������Ч����
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