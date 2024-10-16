#include "raysplittinggpu.h"


HOST_DEVICE_FUNC void GenerateSplittingRayGPU(Ray2DGPU& initRay, int splitNum, Ray2DGPU* newRays, Segment2DGPU* segments) {//障碍物存在条件下射线分裂,修正，透射禁止分裂
	//
	
	//计算出射线管总张角
	double total_theta = 2 * initRay.m_theta;
	
	//计算出射线半张角
	double dtheta = total_theta / splitNum;

	double halftheta = dtheta / 2.0;
	double coshalftheta = cos(halftheta);
	//计算出均匀离散的射线
	Ray2DGPU rayTemp(initRay);
	Vector2D newDir = initRay.m_Dir;
	rayTemp.m_theta = halftheta; //创建新的ray对象
	rayTemp.m_costheta = coshalftheta;
	int k = 0;
	//产生射线分为两部分，第一为广义源情况，第二为非广义源情况
	//----------------------------------------产生新射线(广义源节点)---------------------------------------------
	//这里不再采用不均匀分裂方法，采用均匀分裂方法

	rayTemp.m_theta = halftheta; //确定所有射线的半张角
	rayTemp.m_costheta = coshalftheta;


	//先旋转到一边
	newDir = initRay.m_Dir;
	newDir.Rotate(-initRay.m_theta);
	if (initRay.m_nodeType == NODE_ROOT || initRay.m_nodeType == NODE_DIFF) {//当前节点类型为根节点和绕射节点的不必执行相交判定，在生成完射线后立即返回

		//起始射线
		newDir.Rotate(halftheta); 
		rayTemp.m_Dir = newDir;
		newRays[k] = rayTemp;
		newRays[k].m_isValid = true;
		k++;

		//后续射线
		for (int i = 1; i < splitNum; ++i) {
			newDir.Rotate(dtheta);
			rayTemp.m_Dir = newDir;
			newRays[k] = rayTemp;
			newRays[k].m_isValid = true;
			k++;
		}
		return;
	}
	//非广义源情况



	RtLbsType t = initRay.m_tMax - initRay.m_tMin;
	Point2D vSource = initRay(-t);//计算广义源的位置
	Intersection2DGPU intersect;
	Ray2DGPU tempRay2(initRay);
	tempRay2.m_theta = halftheta;
	tempRay2.m_costheta = coshalftheta;
	newDir = initRay.m_Dir;


	//起始射线
	newDir.Rotate(halftheta);
	rayTemp.m_Dir = newDir;
	if (segments[initRay.m_primitiveId].GetIntersect(rayTemp, &intersect)) {
		newRays[k] = rayTemp;
		newRays[k].m_isValid = true;
		k++;
	}

	//后续射线
	for (int i = 1; i < splitNum; ++i) {
		newDir.Rotate(dtheta);
		rayTemp.m_Dir = newDir;
		if (segments[initRay.m_primitiveId].GetIntersect(rayTemp, &intersect)) {
			newRays[k] = rayTemp;
			newRays[k].m_isValid = true;
			k++;
		}
	}
}


