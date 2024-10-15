#include "reflection.h"


bool GenerateReflectRay(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray) {
	if (inter.m_segment->m_refractN == inter.m_segment->m_refractNOut)//��Ԫ�������������������ͬ���޷���
		return false;
	if (!inter.m_propagationProperty.m_hasRelfection)			//����Ԫ�����з������ԣ��򷵻�false
		return false;
	if (incident_ray.m_fRefractiveIndex == inter.m_segment->m_refractN) {		//������nֵ����Ԫnֵ��ͬ�������������ڵķ���·��
		return false;
	}
	//��������������ⷴ����������
	ray->m_Dir = incident_ray.m_Dir - 2 * (incident_ray.m_Dir * inter.m_segment->m_normal) * inter.m_segment->m_normal;
	ray->m_Ori = inter.m_intersect;
	ray->m_fRefractiveIndex = incident_ray.m_fRefractiveIndex; //��������������ʽ��ʲ���
	ray->m_fMin = incident_ray.m_fMin;
	ray->m_fMax = incident_ray.m_fMax + inter.m_ft;//���Ӵ�������
	ray->m_theta = incident_ray.m_theta;
	ray->m_costheta = incident_ray.m_costheta;
	ray->m_bsplit = incident_ray.m_bsplit;
	ray->m_vWedge = incident_ray.m_vWedge;
	ray->m_sensorDataId = incident_ray.m_sensorDataId;								//����������ID��ֵ
	return true;
}

void GenerateReflectRayOnToughSurface(const Ray2D& incident_ray, const Intersection2D& inter, Ray2D* ray)
{
}

bool GenerateTerrainReflectionPaths(const Terrain* terrain, const Point3D& tx, const Point3D& rx, std::vector<RayPath3D*>& outPaths)
{
	std::vector<TerrainFacet*> attachGroundFacets;																	/** @brief	�����ر����Ԫ����	*/
	if (!terrain->GetTerrainProfileFacets(tx, rx, attachGroundFacets))														//���������˵ر���Ԫ���ϣ��򷵻�false
		return false;
	std::vector<Point3D> intersectPoints;																			/** @brief	��Ч�Ľ��㣬�����淴���	*/
	//�����ر���Ԫ���ϣ����㾵��㲢�����������Ԫ�Ľ���
	for (auto it = attachGroundFacets.begin(); it != attachGroundFacets.end(); ++it) {
		const TerrainFacet* curFacet = *it;
		Point3D mirrorPoint = curFacet->GetMirrorPoint(tx);															/** @brief	������tx�ľ����	*/
		Ray3DLite ray3d(mirrorPoint, rx);																			/** @brief	���������ά����	*/
		Point3D intersectPoint;																						/** @brief	��������Ԫ�Ľ���	*/
		if (curFacet->GetIntersect(&ray3d, &intersectPoint)) {														//��ǰ��Ԫ������н��㣬������������
			RayPath3D* newRayPath = new RayPath3D();																/** @brief	��·��	*/
			PathNode3D* txNode = new PathNode3D(tx, NODE_ROOT);														/** @brief	���췢��ڵ�	*/
			PathNode3D* reflNode = new PathNode3D(intersectPoint, NODE_REFL, curFacet);								/** @brief	�����м䷴��ڵ�	*/
			PathNode3D* rxNode = new PathNode3D(rx, NODE_STOP);													/** @brief	������սڵ�	*/
			newRayPath->Union(txNode);
			newRayPath->Union(reflNode);
			newRayPath->Union(rxNode);
			newRayPath->m_type = RAYPATH_TERRAIN_REFLECTION;
			outPaths.push_back(newRayPath);
		}
	}
	if (outPaths.size() == 0)																						//��out��·������Ϊ0, �򷵻�false, Ѱ�ҵ��η���·����Ч
		return false;
	return true;
}
