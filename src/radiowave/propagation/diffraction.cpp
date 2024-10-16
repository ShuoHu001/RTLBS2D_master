#include "diffraction.h"

bool GenerateDiffractRays(const Ray2D& incident_ray, int diffractRayNum, Wedge2D* wedge, std::vector<Ray2D>* rays) {

	if (wedge->m_face1->m_refractN == wedge->m_face1->m_refractNOut)//分界面折射率相同，不发生绕射
		return false;
	if (!wedge->m_face1->m_propagationProperty.m_hasDiffraction)		//若没有绕射属性，则返回false
		return false;
	if (incident_ray.m_fRefractiveIndex == wedge->m_face1->m_refractN) {	//若入射射线折射率和棱劈折射率相同，根据规则，不会产生绕射路径
		return false;
	}

	RtLbsType tNew = (wedge->m_point - incident_ray.m_Ori).Length();		/** @brief	新传播段距离	*/
	if ((incident_ray.m_tMax + tNew) > incident_ray.m_tLimit) {				//超过最远距离限制，不产生透射
		return false;
	}
	unsigned raynum = diffractRayNum + RANDINT(0, DIFF_DELTARAYNUM) + 2;  //这里多加两根射线为的是弥补两根边缘张角为0的射线
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
	RtLbsType new_m_ft = incident_ray.m_tMax + tNew;//更新绕射射线距离源的距离
	double delta_theta = external_angle / (raynum - 2);
	double half_theta = delta_theta / 2.0;
	double cos_halftheta = cos(half_theta);
	(*rays)[0].m_Ori = wedge->m_point;//第一个角度方向,墙体方向1
	(*rays)[0].m_Dir = wedge->m_dir1;
	(*rays)[0].m_tMin = new_m_ft;
	(*rays)[0].m_tMax = new_m_ft;
	(*rays)[0].m_tLimit = incident_ray.m_tLimit;
	(*rays)[0].m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;//绕射过程中射线上的折射率不变
	(*rays)[0].m_theta = 0.0;
	(*rays)[0].m_costheta = 1.0;//设置射线半角
	(*rays)[0].m_bsplit = false; //墙体边缘射线不分裂
	(*rays)[0].m_vWedge = incident_ray.m_vWedge;
	(*rays)[0].m_vWedge.push_back(wedge);
	(*rays)[0].m_sensorDataId = incident_ray.m_sensorDataId;									//传感器数据ID赋值

	(*rays)[1].m_Ori = wedge->m_point;  //弥补第一条射线的half_theta空白
	(*rays)[1].m_Dir = oa.Rotate(orientation * half_theta);//加上旋转方向
	(*rays)[1].m_tMin = new_m_ft;
	(*rays)[1].m_tMax = new_m_ft;
	(*rays)[1].m_tLimit = incident_ray.m_tLimit;
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
		(*rays)[i].m_tMin = new_m_ft;
		(*rays)[i].m_tMax = new_m_ft;
		(*rays)[i].m_tLimit = incident_ray.m_tLimit;
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
	(*rays)[raynum - 1].m_tMin = new_m_ft;
	(*rays)[raynum - 1].m_tMax = new_m_ft;
	(*rays)[raynum - 1].m_tLimit = incident_ray.m_tLimit;
	(*rays)[raynum - 1].m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;
	(*rays)[raynum - 1].m_theta = 0.0;
	(*rays)[raynum - 1].m_costheta = 1.0;
	(*rays)[raynum - 1].m_bsplit = false;//墙体边缘射线不分裂
	(*rays)[raynum - 1].m_vWedge = incident_ray.m_vWedge;
	(*rays)[raynum - 1].m_vWedge.push_back(wedge);
	(*rays)[raynum - 1].m_sensorDataId = incident_ray.m_sensorDataId;								//传感器数据ID赋值
	return true;
}

bool GetTerrainDiffractionPath(const Terrain* terrain, const Point3D& tx, const Point3D& rx, TerrainDiffractionPath*& outPath)
{
	//修正1-无论是否被遮挡，均返回地面投影路径

	TerrainFacet* txFacet = terrain->GetTerrainFacetViaPoint(tx);
	TerrainFacet* rxFacet = terrain->GetTerrainFacetViaPoint(rx);
	if (txFacet == rxFacet)//0-确定收发坐标点所在的面元，若相同，则返回false
		return false;

	std::vector<Point3D> routeAboveRay;//高于射线高度的路径点集合
	std::vector<Point3D> route;		//收发机间的地形点集合
	//求解发射点在对应面元上的投影坐标
	Point3D txOnFacet = txFacet->GetPointOnPlane(tx); //发射机在面元上的坐标   
	Point3D rxOnFacet = rxFacet->GetPointOnPlane(rx); //接收机在面元上的坐标
	route.push_back(txOnFacet); //先将发射坐标点纳入路径中
	//基于半边搜索方法确定射线所经过的地形表面起伏信息
	//构建基础三维射线
	Vector3D rt = rx - tx;
	Vector3D rt_dir = Normalize(rt);
	Ray3DLite* ray3d = new Ray3DLite();
	ray3d->m_ori = tx;
	ray3d->m_dir = rt_dir;
	//构建基础二维射线
	Ray2D* ray2d = new Ray2D();
	ray2d->m_Ori.x = ray3d->m_ori.x;
	ray2d->m_Ori.y = ray3d->m_ori.y;
	ray2d->m_Dir.x = ray3d->m_dir.x;
	ray2d->m_Dir.y = ray3d->m_dir.y;
	ray2d->m_Dir.Normalize(); //赋值后需要归一化数值

	TerrainFacet* curFacet = txFacet; /** @brief	当前运算到的面元	*/
	TerrainSegment* curSegment = nullptr; /** @brief	当前运算到的线段	*/
	TerrainSegment* prevSegment = nullptr; /** @brief	上一个运算到的线段	*/
	RtLbsType t_cur; /** @brief	当前射线与线段相交的距离二维	*/
	RtLbsType h_cur; /** @brief	当前射线与线段相交的交点高度（用于比较是否与射线相交）	*/
	RtLbsType t_max = rt.LengthXY();
	bool hasIntersect = false; //射线被环境遮挡状态
	while (true) { //遍历到相等的位置
		prevSegment = curSegment;
		curSegment = curFacet->GetIntersectSegment(ray2d, prevSegment, &t_cur, &h_cur);//求解射线与面元内部的相交线段
		if (curSegment == nullptr)	//迭代到无法解决的误差问题，终止路径循迹
			break;
		RtLbsType rayHeight = ray3d->GetRayHeight(t_cur);
		//计算二维射线与线段的交点并根据h_cur转换为三维交点
		Point2D pInTerrain2D = (*ray2d)(t_cur);
		Point3D pInTerrain3D(pInTerrain2D.x, pInTerrain2D.y, h_cur);
		route.push_back(pInTerrain3D);
		if (rayHeight <= h_cur) {//将路径添加至高于接收天线的路径中
			routeAboveRay.push_back(pInTerrain3D);
		}
		if (rayHeight <= h_cur || (rayHeight - h_cur) <= 1) { //射线和线段的高度差在1m以内，采用射线与三角面元求交模式求解精确解
			if (curFacet->GetIntersect(ray3d))//计算射线与当前面元的相交结果
				hasIntersect = true; //与环境相交
		}
		curFacet = curSegment->GetAdjacentFacet(curFacet); //计算下一个面元
		if (curFacet == rxFacet || t_cur >= t_max) //遍历到相等面元或距离超过最大距离限制则进行break
			break;
	}
	route.push_back(rxOnFacet); //添加接收机表面路径

	TerrainProfile* profile = new TerrainProfile();
	//获得路径上每个节点的电磁材质信息
	std::vector<Material*> mats(route.size());															/** @brief	地形点材质数组	*/
	for (int i = 0; i < route.size(); ++i)
		mats[i] = terrain->GetMaterial(route[i]);														//获得路径上每个点的材质ID
	profile->InitParameters(route, mats, tx, rx, terrain->m_averageRidgeGap);
	GetDiffractPathOverRidges(profile, outPath);	//由profile生成绕射路径
	if (outPath != nullptr) {								//当且仅当outPath不为nullptr时使用
		outPath->m_terrainDiffractionMode = terrain->m_propagationProperty.m_terrainDiffractionMode;			//地形绕射计算模式判别
	}
	//profile.WriteRidgesToFile("ridges.txt");		//将profile中的峰峦写入到文件中-调试用
	profile->WriteProfileToFile("profile.txt");		//将profile中的地形剖面写入到文件中-调试用
	delete profile;
	delete ray2d;
	delete ray3d;

	return true;
}

void GetDiffractPathOverRidges(const TerrainProfile* profile, TerrainDiffractionPath*& outPath)
{
	if (profile->m_validRidgeNum == 0)
		return;
	outPath = new TerrainDiffractionPath();

	TerrainPathNode* txNode = new TerrainPathNode(profile->m_txPosition);				/** @brief	tx节点	*/
	outPath->m_nodes.push_back(txNode);											//添加tx节点

	for (int i = 0; i < profile->m_ridges.size(); ++i) {									//添加path中的路径节点
		if (profile->m_ridges[i]->m_isValid == false)
			continue;
		TerrainPathNode* newNode = new TerrainPathNode(profile->m_ridges[i]);
		outPath->m_nodes.push_back(newNode);
	}

	TerrainPathNode* rxNode = new TerrainPathNode(profile->m_rxPosition);				/** @brief	rx节点	*/
	outPath->m_nodes.push_back(rxNode);											//添加rx节点

	//更新绕射路径的s参数
	outPath->RectifySParameters();
}
