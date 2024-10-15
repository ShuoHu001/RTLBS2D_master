#include "reflection.h"


bool GenerateReflectRay(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray) {
	if (inter.m_segment->m_refractN == inter.m_segment->m_refractNOut)//面元折射率与外界折射率相同，无反射
		return false;
	if (!inter.m_propagationProperty.m_hasRelfection)			//若面元不具有反射属性，则返回false
		return false;
	if (incident_ray.m_fRefractiveIndex == inter.m_segment->m_refractN) {		//若射线n值和面元n值相同，不产生介质内的反射路径
		return false;
	}
	//根据入射射线求解反射射线向量
	ray->m_Dir = incident_ray.m_Dir - 2 * (incident_ray.m_Dir * inter.m_segment->m_normal) * inter.m_segment->m_normal;
	ray->m_Ori = inter.m_intersect;
	ray->m_fRefractiveIndex = incident_ray.m_fRefractiveIndex; //反射过程中折射率介质不变
	ray->m_fMin = incident_ray.m_fMin;
	ray->m_fMax = incident_ray.m_fMax + inter.m_ft;//叠加传播距离
	ray->m_theta = incident_ray.m_theta;
	ray->m_costheta = incident_ray.m_costheta;
	ray->m_bsplit = incident_ray.m_bsplit;
	ray->m_vWedge = incident_ray.m_vWedge;
	ray->m_sensorDataId = incident_ray.m_sensorDataId;								//传感器数据ID赋值
	return true;
}

void GenerateReflectRayOnToughSurface(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray)
{
}

bool GenerateTerrainReflectionPaths(const Terrain* terrain, const Point3D& tx, const Point3D& rx, std::vector<RayPath3D*>& outPaths)
{
	std::vector<TerrainFacet*> attachGroundFacets;																	/** @brief	贴近地表的面元集合	*/
	if (!terrain->GetTerrainProfileFacets(tx, rx, attachGroundFacets))														//若产生不了地表面元集合，则返回false
		return false;
	std::vector<Point3D> intersectPoints;																			/** @brief	有效的交点，即地面反射点	*/
	//遍历地表面元集合，计算镜像点并求解射线与面元的交点
	for (auto it = attachGroundFacets.begin(); it != attachGroundFacets.end(); ++it) {
		const TerrainFacet* curFacet = *it;
		Point3D mirrorPoint = curFacet->GetMirrorPoint(tx);															/** @brief	求解关于tx的镜像点	*/
		Ray3DLite ray3d(mirrorPoint, rx);																			/** @brief	构造基础三维射线	*/
		Point3D intersectPoint;																						/** @brief	射线与面元的交点	*/
		if (curFacet->GetIntersect(&ray3d, &intersectPoint)) {														//当前面元与地形有交点，则表明反射存在
			RayPath3D* newRayPath = new RayPath3D();																/** @brief	新路径	*/
			PathNode3D* txNode = new PathNode3D(tx, NODE_ROOT);														/** @brief	构造发射节点	*/
			PathNode3D* reflNode = new PathNode3D(intersectPoint, NODE_REFL, curFacet);								/** @brief	构造中间反射节点	*/
			PathNode3D* rxNode = new PathNode3D(rx, NODE_STOP);													/** @brief	构造接收节点	*/
			newRayPath->Union(txNode);
			newRayPath->Union(reflNode);
			newRayPath->Union(rxNode);
			newRayPath->m_type = RAYPATH_TERRAIN_REFLECTION;
			outPaths.push_back(newRayPath);
		}
	}
	if (outPaths.size() == 0)																						//若out的路径数量为0, 则返回false, 寻找地形反射路径无效
		return false;
	return true;
}
