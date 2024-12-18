#include "matrixfunction.h"

Eigen::MatrixXd ComputePseudoInverser(const Eigen::MatrixXd& matrix, double tolerance)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::VectorXd singularValues = svd.singularValues(); // 奇异值
	Eigen::MatrixXd singularValuesInv = Eigen::MatrixXd::Zero(matrix.cols(), matrix.rows());

	// 计算伪逆的奇异值部分
	for (int i = 0; i < singularValues.size(); ++i) {
		if (singularValues(i) > tolerance) {
			singularValuesInv(i, i) = 1.0 / singularValues(i);
		}
	}

	// 伪逆公式： V * S⁻¹ * U^T
	return svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
}
