#include "assignalgorithm.h"

std::vector<std::pair<int, int>> AssignOprimization(const std::vector<std::vector<RtLbsType>>& cost) {
	int n = static_cast<int>(cost.size());
	int m = static_cast<int>(cost[0].size());

	// 创建问题对象
	glp_prob* lp = glp_create_prob();
	glp_set_prob_name(lp, "assignment");
	glp_set_obj_dir(lp, GLP_MIN);

	// 添加行（每行一个约束）
	glp_add_rows(lp, n);
	for (int i = 1; i <= n; ++i) {
		glp_set_row_bnds(lp, i, GLP_FX, 1.0, 1.0);
	}

	// 添加列（每个变量一个约束）
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

	// 定义非零单元格
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

	// 添加列约束
	glp_add_rows(lp, m);
	for (int j = 1; j <= m; ++j) {
		glp_set_row_bnds(lp, n + j, GLP_UP, 0.0, 1.0);
	}

	// 加载矩阵
	glp_load_matrix(lp, n * m * 2, ia.data(), ja.data(), ar.data());

	glp_term_out(GLP_OFF);		//关闭GLPK的终端显示

	// 解决问题
	glp_simplex(lp, NULL);
	glp_intopt(lp, NULL);


	std::vector<std::pair<int, int>> reVal;
	// 输出结果
	//std::cout << "最小总距离：" << glp_mip_obj_val(lp) << std::endl;
	for (int i = 1; i <= n; ++i) {
		for (int j = 1; j <= m; ++j) {
			if (glp_mip_col_val(lp, (i - 1) * m + j) == 1) {
				reVal.push_back({ i - 1,j - 1 });
				//std::cout << "行 " << i << " -> 列 " << j << std::endl;
			}
		}
	}


	// 释放内存
	glp_delete_prob(lp);
	glp_free_env();

	return reVal;
}