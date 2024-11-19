#include "diffractiongpu.h"


HOST_DEVICE_FUNC bool GenerateDiffractRaysGPU(const Intersection2DGPU& inter, Ray2DGPU& incident_ray, Wedge2DGPU* wedge, int prevInterId, Ray2DGPU* newRays, int raynum) {
	if (!inter.m_propagationProperty.m_hasDiffraction)
		return false;
	if (wedge->m_face1.m_refractN == wedge->m_face1.m_refractNOut)//分界面折射率相同，不发生绕射
		return false;
	//unsigned raynum = DIFF_RAYNUM + 2;  //这里多加两根射线为的是弥补两根边缘张角为0的射线
	double orientation = 1.0;  /** @brief	旋转方向 1:逆时针 -1:顺时针，默认逆时针	*/
	double external_angle = wedge->m_fExternalAngle;
	//构造初始出射射线方向
	Vector2D oa = wedge->m_vector1;
	Vector2D ob = wedge->m_vector2;
	Vector2D n1 = wedge->m_face1.m_normal;
	if (Cross(oa, n1) < 0)//需要沿着n1的方向旋转
		orientation = -1.0;//顺时针
	//判断是内部绕射还是外部绕射
	if (incident_ray.m_fRefractiveIndex == wedge->m_face1.m_refractN) {//射线由内部发生绕射,改为余角,旋转方向取相反方向
		external_angle = TWO_PI - external_angle;
		orientation *= -1.0;//切换旋转方向
	}
	RtLbsType new_m_ft = incident_ray.m_tMax + (wedge->m_edge - incident_ray.m_Ori).Length();//更新绕射射线距离源的距离
	double delta_theta = external_angle / (raynum - 2);
	double half_theta = delta_theta / 2.0;
	double cos_halftheta = cos(half_theta);
	Ray2DGPU newRay;
	newRay.m_isValid = true;
	newRay.m_limTotl = incident_ray.m_limTotl - 1;//总限制数-1
	newRay.m_limDiff = incident_ray.m_limDiff - 1;//绕射限制数-1
	newRay.m_limRefl = incident_ray.m_limRefl;
	newRay.m_limTran = incident_ray.m_limTran;
	newRay.m_limScat = incident_ray.m_limScat;
	newRay.m_Ori = wedge->m_edge;
	newRay.m_tMin = new_m_ft;
	newRay.m_tMax = new_m_ft;
	newRay.m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;//绕射过程中射线上的折射率不变
	newRay.m_nodeType = NODE_DIFF;
	newRay.m_theta = half_theta;
	newRay.m_costheta = cos_halftheta;
	newRay.m_prevInterId = prevInterId;

	newRays[0] = newRay;
	newRays[0].m_Dir = wedge->m_vector1;
	newRays[0].m_theta = 0.0;
	newRays[0].m_costheta = 1.0;//设置射线半角
	newRays[0].m_bsplit = false; //墙体边缘射线不分裂
	
	newRays[1] = newRay;
	newRays[1].m_Dir = oa.Rotate(orientation * half_theta);//加上旋转方向
	newRays[1].m_bsplit = true;              //绕射射线具有分裂属性（新的广义源）


	for (unsigned i = 2; i < raynum - 1; i++) {//中间角度方向
		newRays[i] = newRay;
		newRays[i].m_Dir = oa.Rotate(orientation * delta_theta);//每次旋转一定角度，离散化
		newRays[i].m_bsplit = true;
	}

	newRays[raynum - 1] = newRay;//最后一个方向,墙体方向2
	newRays[raynum - 1].m_Dir = wedge->m_vector2;
	newRays[raynum - 1].m_theta = 0.0;
	newRays[raynum - 1].m_costheta = 1.0;
	newRays[raynum - 1].m_bsplit = false;//墙体边缘射线不分裂
	return true;
}

HOST_DEVICE_FUNC bool GenerateDiffractRaysGPU(const Intersection2DGPU& inter, Ray2DGPU& incident_ray, Wedge2DGPU* wedge, int prevInterId, Ray2DGPU* newRays, TreeNodeGPU* nodes, int raynum, int layer)
{
	if (!inter.m_propagationProperty.m_hasDiffraction)
		return false;
	if (wedge->m_face1.m_refractN == wedge->m_face1.m_refractNOut)//分界面折射率相同，不发生绕射
		return false;
	if (incident_ray.m_nodeType == NODE_TRANIN ||
		incident_ray.m_nodeType == NODE_ETRANIN) {					//透射入、经验透射入之后不会进行绕射
		return false;
	}
	//unsigned raynum = DIFF_RAYNUM + 2;  //这里多加两根射线为的是弥补两根边缘张角为0的射线
	double orientation = 1.0;  /** @brief	旋转方向 1:逆时针 -1:顺时针，默认逆时针	*/
	double external_angle = wedge->m_fExternalAngle;
	//构造初始出射射线方向
	Vector2D oa = wedge->m_vector1;
	Vector2D ob = wedge->m_vector2;
	Vector2D n1 = wedge->m_face1.m_normal;
	if (Cross(oa, n1) < 0)//需要沿着n1的方向旋转
		orientation = -1.0;//顺时针
	//判断是内部绕射还是外部绕射
	if (incident_ray.m_fRefractiveIndex == wedge->m_face1.m_refractN) {//射线由内部发生绕射,改为余角,旋转方向取相反方向
		external_angle = TWO_PI - external_angle;
		orientation *= -1.0;//切换旋转方向
	}
	RtLbsType new_m_ft = incident_ray.m_tMax + (wedge->m_edge - incident_ray.m_Ori).Length();//更新绕射射线距离源的距离
	double delta_theta = external_angle / (raynum - 2);
	double half_theta = delta_theta / 2.0;
	double cos_halftheta = cos(half_theta);
	Ray2DGPU newRay;
	newRay.m_isValid = true;
	newRay.m_limTotl = incident_ray.m_limTotl - 1;//总限制数-1
	newRay.m_limDiff = incident_ray.m_limDiff - 1;//绕射限制数-1
	newRay.m_limRefl = incident_ray.m_limRefl;
	newRay.m_limTran = incident_ray.m_limTran;
	newRay.m_limScat = incident_ray.m_limScat;
	newRay.m_Ori = wedge->m_edge;
	newRay.m_tMin = new_m_ft;
	newRay.m_tMax = new_m_ft;
	newRay.m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;//绕射过程中射线上的折射率不变
	newRay.m_nodeType = NODE_DIFF;
	newRay.m_theta = half_theta;
	newRay.m_costheta = cos_halftheta;
	newRay.m_prevInterId = prevInterId;

	newRays[0] = newRay;
	newRays[0].m_Dir = wedge->m_vector1;
	newRays[0].m_theta = 0.0;
	newRays[0].m_costheta = 1.0;//设置射线半角
	newRays[0].m_bsplit = false; //墙体边缘射线不分裂

	newRays[1] = newRay;
	newRays[1].m_Dir = oa.Rotate(orientation * half_theta);//加上旋转方向
	newRays[1].m_bsplit = true;              //绕射射线具有分裂属性（新的广义源）


	for (unsigned i = 2; i < raynum - 1; i++) {//中间角度方向
		newRays[i] = newRay;
		newRays[i].m_Dir = oa.Rotate(orientation * delta_theta);//每次旋转一定角度，离散化
		newRays[i].m_bsplit = true;
	}

	newRays[raynum - 1] = newRay;//最后一个方向,墙体方向2
	newRays[raynum - 1].m_Dir = wedge->m_vector2;
	newRays[raynum - 1].m_theta = 0.0;
	newRays[raynum - 1].m_costheta = 1.0;
	newRays[raynum - 1].m_bsplit = false;//墙体边缘射线不分裂

	//进行射线树的赋值
	for (int i = 0; i < raynum; ++i) {
		nodes[i].m_isValid = true;
		nodes[i].m_type = NODE_DIFF;
		nodes[i].m_depth = layer;
		nodes[i].m_t = new_m_ft;
		nodes[i].m_point = wedge->m_edge;
		nodes[i].m_segmentId = inter.m_segmentId;
		nodes[i].m_wedgeId = inter.m_wedgeId;
		nodes[i].m_nextRay = newRays[i];
	}

	return true;
}
