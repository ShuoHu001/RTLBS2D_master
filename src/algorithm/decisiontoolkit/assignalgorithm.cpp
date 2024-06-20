#include "assignalgorithm.h"

std::vector<std::pair<int, int>> AssignOprimization(const std::vector<std::vector<RtLbsType>>& cost) {
	int n = static_cast<int>(cost.size());
	int m = static_cast<int>(cost[0].size());

	// �����������
	glp_prob* lp = glp_create_prob();
	glp_set_prob_name(lp, "assignment");
	glp_set_obj_dir(lp, GLP_MIN);

	// ����У�ÿ��һ��Լ����
	glp_add_rows(lp, n);
	for (int i = 1; i <= n; ++i) {
		glp_set_row_bnds(lp, i, GLP_FX, 1.0, 1.0);
	}

	// ����У�ÿ������һ��Լ����
	glp_add_cols(lp, n * m);
	int index = 1;
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			glp_set_col_bnds(lp, index, GLP_DB, 0.0, 1.0);
			glp_set_col_kind(lp, index, GLP_BV);
			glp_set_obj_coef(lp, index, cost[i][j]);
			++index;
		}
	}

	// ������㵥Ԫ��
	std::vector<int> ia(1 + n * m * 2);
	std::vector<int> ja(1 + n * m * 2);
	std::vector<double> ar(1 + n * m * 2);

	index = 1;
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < m; ++j) {
			ia[index] = i + 1;
			ja[index] = i * m + j + 1;
			ar[index] = 1.0;
			++index;
		}
	}

	for (int j = 0; j < m; ++j) {
		for (int i = 0; i < n; ++i) {
			ia[index] = n + j + 1;
			ja[index] = i * m + j + 1;
			ar[index] = 1.0;
			++index;
		}
	}

	// �����Լ��
	glp_add_rows(lp, m);
	for (int j = 1; j <= m; ++j) {
		glp_set_row_bnds(lp, n + j, GLP_UP, 0.0, 1.0);
	}

	// ���ؾ���
	glp_load_matrix(lp, n * m * 2, ia.data(), ja.data(), ar.data());

	glp_term_out(GLP_OFF);		//�ر�GLPK���ն���ʾ

	// �������
	glp_simplex(lp, NULL);
	glp_intopt(lp, NULL);


	std::vector<std::pair<int, int>> reVal;
	// ������
	//std::cout << "��С�ܾ��룺" << glp_mip_obj_val(lp) << std::endl;
	for (int i = 1; i <= n; ++i) {
		for (int j = 1; j <= m; ++j) {
			if (glp_mip_col_val(lp, (i - 1) * m + j) == 1) {
				reVal.push_back({ i - 1,j - 1 });
				//std::cout << "�� " << i << " -> �� " << j << std::endl;
			}
		}
	}


	// �ͷ��ڴ�
	glp_delete_prob(lp);
	glp_free_env();

	return reVal;
}