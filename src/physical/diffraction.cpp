#include "diffraction.h"

bool GenerateDiffractRays(const Ray2D& incident_ray, Wedge2D* wedge, std::vector<Ray2D>* rays) {

	if (wedge->m_face1->m_refractN == wedge->m_face1->m_refractNOut)//分界面折射率相同，不发生绕射
		return false;
	if (!wedge->m_face1->m_propagationProperty.m_hasDiffraction)		//若没有绕射属性，则返回false
		return false;
	if (incident_ray.m_fRefractiveIndex == wedge->m_face1->m_refractN) {	//若入射射线折射率和棱劈折射率相同，根据规则，不会产生绕射路径
		return false;
	}
	unsigned raynum = _global_diffractRayNum + RANDINT(0, DIFF_DELTARAYNUM) + 2;  //这里多加两根射线为的是弥补两根边缘张角为0的射线
	double orientation = 1.0;  /** @brief	旋转方向 1:逆时针 -1:顺时针，默认逆时针	*/
	double external_angle = wedge->m_theta;
	(*rays).resize(raynum);
	//构造初始出射射线方向
	Vector2D oa = wedge->m_dir1;
	Vector2D ob = wedge->m_dir2;
	Vector2D n1 = wedge->m_face1->m_normal;
	if (Cross(oa, n1) < 0)//需要沿着n1的方向旋转
		orientation = -1.0;//顺时针
	//判断是内部绕射还是外部绕射
	if (incident_ray.m_fRefractiveIndex == wedge->m_face1->m_refractN) {//射线由内部发生绕射,改为余角,旋转方向取相反方向
		external_angle = TWO_PI - external_angle;
		orientation *= -1.0;//切换旋转方向
	}
	RtLbsType new_m_ft = incident_ray.m_fMax + (wedge->m_point - incident_ray.m_Ori).Length();//更新绕射射线距离源的距离
	double delta_theta = external_angle / (raynum - 2);
	double half_theta = delta_theta / 2.0;
	double cos_halftheta = cos(half_theta);
	(*rays)[0].m_Ori = wedge->m_point;//第一个角度方向,墙体方向1
	(*rays)[0].m_Dir = wedge->m_dir1;
	(*rays)[0].m_fMin = new_m_ft;
	(*rays)[0].m_fMax = new_m_ft;
	(*rays)[0].m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;//绕射过程中射线上的折射率不变
	(*rays)[0].m_theta = 0.0;
	(*rays)[0].m_costheta = 1.0;//设置射线半角
	(*rays)[0].m_bsplit = false; //墙体边缘射线不分裂
	(*rays)[0].m_vWedge = incident_ray.m_vWedge;
	(*rays)[0].m_vWedge.push_back(wedge);
	(*rays)[0].m_sensorDataId = incident_ray.m_sensorDataId;									//传感器数据ID赋值

	(*rays)[1].m_Ori = wedge->m_point;  //弥补第一条射线的half_theta空白
	(*rays)[1].m_Dir = oa.Rotate(orientation * half_theta);//加上旋转方向
	(*rays)[1].m_fMin = new_m_ft;
	(*rays)[1].m_fMax = new_m_ft;
	(*rays)[1].m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;//绕射过程中射线上的折射率不变
	(*rays)[1].m_theta = half_theta;
	(*rays)[1].m_costheta = cos_halftheta;//设置射线半角
	(*rays)[1].m_bsplit = true;              //绕射射线具有分裂属性（新的广义源）
	(*rays)[1].m_vWedge = incident_ray.m_vWedge;
	(*rays)[1].m_vWedge.push_back(wedge);
	(*rays)[1].m_sensorDataId = incident_ray.m_sensorDataId;									//传感器数据ID赋值

	for (unsigned i = 2; i < raynum - 1; i++) {//中间角度方向
		(*rays)[i].m_Ori = wedge->m_point;
		(*rays)[i].m_Dir = oa.Rotate(orientation * delta_theta);//每次旋转一定角度，离散化
		(*rays)[i].m_fMin = new_m_ft;
		(*rays)[i].m_fMax = new_m_ft;
		(*rays)[i].m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;
		(*rays)[i].m_theta = half_theta;
		(*rays)[i].m_costheta = cos_halftheta;
		(*rays)[i].m_bsplit = true;
		(*rays)[i].m_vWedge = incident_ray.m_vWedge;
		(*rays)[i].m_vWedge.push_back(wedge);
		(*rays)[i].m_sensorDataId = incident_ray.m_sensorDataId;									//传感器数据ID赋值
	}

	(*rays)[raynum - 1].m_Ori = wedge->m_point;//最后一个方向,墙体方向2
	(*rays)[raynum - 1].m_Dir = wedge->m_dir2;
	(*rays)[raynum - 1].m_fMin = new_m_ft;
	(*rays)[raynum - 1].m_fMax = new_m_ft;
	(*rays)[raynum - 1].m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;
	(*rays)[raynum - 1].m_theta = 0.0;
	(*rays)[raynum - 1].m_costheta = 1.0;
	(*rays)[raynum - 1].m_bsplit = false;//墙体边缘射线不分裂
	(*rays)[raynum - 1].m_vWedge = incident_ray.m_vWedge;
	(*rays)[raynum - 1].m_vWedge.push_back(wedge);
	(*rays)[raynum - 1].m_sensorDataId = incident_ray.m_sensorDataId;								//传感器数据ID赋值
	return true;
}
