#include "raysplitting.h"




bool IsGenerateSplittingRays(const Ray2D& rayInit, RtLbsType t, bool splitFlag, RtLbsType splitRadius, int& splitNum)
{
	if (rayInit.m_bsplit == false || !splitFlag) {				//若射线不具备分裂属性，不进行分裂
		return false;
	}
	RtLbsType r = rayInit.GetRayRadius(t);			/** @brief	计算射线在t处的半径值	*/
	if (r <= splitRadius) {							//若射线在t处的半径值小于分裂半径，则认定该条射线不分裂
		return false;
	}
	//计算出射线管总张角
	double total_theta = 2 * rayInit.m_theta;
	//计算出分裂的射线数量
	splitNum = static_cast<int>(ceil(2 * r / splitRadius)) + 2;
	if (splitNum % 2 != 1) {//须保证分裂数目为奇数，可以对称划分
		splitNum += 1;
	}
	return true;
}

void GenerateSplittingRay(const Ray2D& ray, int splitNum, std::vector<Ray2D>* rays) {//障碍物存在条件下射线分裂,修正，透射禁止分裂
	//计算出射线管总张角
	double total_theta = 2 * ray.m_theta;
	(*rays).resize(splitNum);
	//计算出射线半张角
	double dtheta = total_theta / (splitNum - 1);
	double ray_halftheta = dtheta / 2.0;
	double coshalftheta = cos(ray_halftheta);
	double ray_quardtheta = dtheta / 4.0;
	double cosquardtheta = cos(ray_quardtheta);
	//计算出均匀离散的射线
	Ray2D newRay1(ray);
	Vector2D newDir = ray.m_Dir;
	newRay1.m_theta = ray_halftheta; //创建新的ray对象
	newRay1.m_costheta = coshalftheta;
	int k1 = 0;
	//----------------------------------------产生新射线(广义源节点)---------------------------------------------

	//不旋转
	(*rays)[k1++] = newRay1;

	//左旋 1 dtheta
	newDir = ray.m_Dir;
	for (int i = 1; i < (splitNum - 1) / 2; ++i) {
		newDir.Rotate(dtheta);
		newRay1.m_Dir = newDir;
		newRay1.m_theta = ray_halftheta;
		newRay1.m_costheta = coshalftheta;
		(*rays)[k1++] = newRay1;
	}
	//左旋 3/4 dtheta
	newDir.Rotate(ray_quardtheta + ray_halftheta);
	newRay1.m_Dir = newDir;
	newRay1.m_theta = ray_quardtheta;
	newRay1.m_costheta = cosquardtheta;
	(*rays)[k1++] = newRay1;


	//右旋 1 dtheta
	newDir = ray.m_Dir;
	for (int i = 1; i < (splitNum - 1) / 2; ++i) {
		newDir.Rotate(-dtheta);
		newRay1.m_Dir = newDir;
		newRay1.m_theta = ray_halftheta;
		newRay1.m_costheta = coshalftheta;
		(*rays)[k1++] = newRay1;
	}
	//右旋 3/4 dtheta
	newDir.Rotate(-ray_quardtheta - ray_halftheta);
	newRay1.m_Dir = newDir;
	newRay1.m_theta = ray_quardtheta;
	newRay1.m_costheta = cosquardtheta;
	(*rays)[k1++] = newRay1;
}
