#include "gdop.h"

RtLbsType ComputeGDOPForAOA(const std::vector<Point2D>& bss, const Point2D& ms)
{
	int n = bss.size();                                     /** @brief	站点的数量	*/
	if (n < 2) {                                            //数量小于2，无法计算CRLB，返回最大值
		return -1;
	}

	Eigen::Matrix2d FIM = Eigen::Matrix2d::Zero();           /** @brief	Fisher 信息矩阵	*/

	for (const auto& bs : bss) {
		//计算几何关系
		double d2 = (ms - bs).SquaredLength();
		double dx = ms.x - bs.x;
		double dy = ms.y - bs.y;

		//计算雅可比矩阵
		Eigen::Vector2d Jacob;
		Jacob(0) = -dy / d2;                                //∂θ/∂x
		Jacob(1) = dx / d2;                                 //∂θ/∂y

		//累加 Fisher 信息矩阵
		FIM += (Jacob * Jacob.transpose()) ;
	}

	//使用伪逆计算 GDOP
	Eigen::Matrix2d GDOP = FIM.completeOrthogonalDecomposition().pseudoInverse();
	return std::sqrt(GDOP(0, 0) + GDOP(1, 1));
}

RtLbsType ComputeGDOPForTOA(const std::vector<Point2D>& bss, const Point2D& ms)
{
	int n = bss.size();                                     /** @brief	站点的数量	*/
	if (n < 2) {                                            //数量小于2，无法计算CRLB，返回最大值
		return -1;
	}

	Eigen::Matrix2d FIM = Eigen::Matrix2d::Zero();           /** @brief	Fisher 信息矩阵	*/

	for (const auto& bs : bss) {
		//计算几何关系
		double d = (ms - bs).Length();
		double dx = ms.x - bs.x;
		double dy = ms.y - bs.y;

		//计算雅可比矩阵
		Eigen::Vector2d Jacob;
		Jacob(0) = dx / d;                                //∂θ/∂x
		Jacob(1) = dy / d;                                //∂θ/∂y

		//累加 Fisher 信息矩阵
		FIM += (Jacob * Jacob.transpose());
	}

	//使用伪逆计算 CRLB
	Eigen::Matrix2d CRLB = ComputePseudoInverser(FIM);
	return std::sqrt((CRLB(0, 0) + CRLB(1, 1)));
}

RtLbsType ComputeGDOPForAOATOA(const std::vector<Point2D>& bss, const Point2D& ms)
{
	int n = bss.size();                                     /** @brief	站点的数量	*/
	if (n < 2) {                                            //数量小于2，无法计算CRLB，返回最大值
		return -1;
	}

	Eigen::Matrix2d FIM = Eigen::Matrix2d::Zero();           /** @brief	Fisher 信息矩阵	*/

	for (const auto& bs : bss) {
		//计算几何关系
		double d = (ms - bs).Length();
		double dx = ms.x - bs.x;
		double dy = ms.y - bs.y;


		//计算雅可比矩阵-AOA
		Eigen::Vector2d Jacob_aoa;
		Jacob_aoa(0) = -dy / (d * d);
		Jacob_aoa(1) = dx / (d * d);

		//计算雅可比矩阵-TOA
		Eigen::Vector2d Jacob_toa;
		Jacob_toa(0) = dx / d;
		Jacob_toa(1) = dy / d;

		//累加 Fisher 信息矩阵;
		FIM += (Jacob_aoa * Jacob_aoa.transpose());
		FIM += (Jacob_toa * Jacob_toa.transpose());
	}

	//使用伪逆计算 CRLB
	Eigen::Matrix2d GDOP = ComputePseudoInverser(FIM);
	return std::sqrt((GDOP(0, 0) + GDOP(1, 1)));
}

RtLbsType ComputeGDOPForAOATDOA(const std::vector<Point2D>& bss, const Point2D& ms)
{
	int n = bss.size();                                     /** @brief	站点的数量	*/
	if (n < 3) {                                            //数量小于2，无法计算CRLB，返回最大值
		return -1;
	}

	Eigen::Matrix2d FIM = Eigen::Matrix2d::Zero();           /** @brief	Fisher 信息矩阵	*/

	//计算AOA算法的FIM矩阵
	for (auto& bs : bss) {
		//计算几何关系
		double d2 = (ms - bs).SquaredLength();
		double dx = ms.x - bs.x;
		double dy = ms.y - bs.y;

		//计算雅可比矩阵
		Eigen::Vector2d Jacob_aoa;
		Jacob_aoa(0) = -dy / d2;
		Jacob_aoa(1) = dx / d2;

		//累加 Fisher 信息矩阵
		FIM += (Jacob_aoa * Jacob_aoa.transpose());
	}

	//计算TDOA算法的FIM矩阵
	for (int i = 1; i < n; ++i) {
		//计算几何关系
		double d_ref = (ms - bss[0]).Length();
		double dx_ref = ms.x - bss[0].x;
		double dy_ref = ms.y - bss[0].y;

		double d = (ms - bss[i]).Length();
		double dx = ms.x - bss[i].x;
		double dy = ms.y - bss[i].y;

		//计算雅可比矩阵
		Eigen::Vector2d Jacob_tdoa;
		Jacob_tdoa(0) = (dx / d) - (dx_ref / d_ref);
		Jacob_tdoa(1) = (dy / d) - (dy_ref / d_ref);

		//累加 Fisher 信息矩阵
		FIM += (Jacob_tdoa * Jacob_tdoa.transpose());
	}

	//使用伪逆计算 CRLB
	Eigen::Matrix2d GDOP = ComputePseudoInverser(FIM);
	return std::sqrt((GDOP(0, 0) + GDOP(1, 1)));
}
