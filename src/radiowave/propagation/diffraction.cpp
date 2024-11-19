#include "diffraction.h"

bool GenerateDiffractRays(const Ray2D& incident_ray, int diffractRayNum, Wedge2D* wedge, std::vector<Ray2D>* rays) {

	if (wedge->m_face1->m_refractN == wedge->m_face1->m_refractNOut)//�ֽ�����������ͬ������������
		return false;
	if (!wedge->m_face1->m_propagationProperty.m_hasDiffraction)		//��û���������ԣ��򷵻�false
		return false;
	if (incident_ray.m_fRefractiveIndex == wedge->m_face1->m_refractN) {	//���������������ʺ�������������ͬ�����ݹ��򣬲����������·��
		return false;
	}

	RtLbsType tNew = (wedge->m_point - incident_ray.m_Ori).Length();		/** @brief	�´����ξ���	*/
	if ((incident_ray.m_tMax + tNew) > incident_ray.m_tLimit) {				//������Զ�������ƣ�������͸��
		return false;
	}
	unsigned raynum = diffractRayNum + RANDINT(0, DIFF_DELTARAYNUM) + 2;  //��������������Ϊ�����ֲ�������Ե�Ž�Ϊ0������
	double orientation = 1.0;  /** @brief	��ת���� 1:��ʱ�� -1:˳ʱ�룬Ĭ����ʱ��	*/
	double external_angle = wedge->m_theta;
	(*rays).resize(raynum);
	//�����ʼ�������߷���
	Vector2D oa = wedge->m_dir1;
	Vector2D ob = wedge->m_dir2;
	Vector2D n1 = wedge->m_face1->m_normal;
	if (Cross(oa, n1) < 0)//��Ҫ����n1�ķ�����ת
		orientation = -1.0;//˳ʱ��
	//�ж����ڲ����仹���ⲿ����
	if (incident_ray.m_fRefractiveIndex == wedge->m_face1->m_refractN) {//�������ڲ���������,��Ϊ���,��ת����ȡ�෴����
		external_angle = TWO_PI - external_angle;
		orientation *= -1.0;//�л���ת����
	}
	RtLbsType new_m_ft = incident_ray.m_tMax + tNew;//�����������߾���Դ�ľ���
	double delta_theta = external_angle / (raynum - 2);
	double half_theta = delta_theta / 2.0;
	double cos_halftheta = cos(half_theta);
	(*rays)[0].m_Ori = wedge->m_point;//��һ���Ƕȷ���,ǽ�巽��1
	(*rays)[0].m_Dir = wedge->m_dir1;
	(*rays)[0].m_tMin = new_m_ft;
	(*rays)[0].m_tMax = new_m_ft;
	(*rays)[0].m_tLimit = incident_ray.m_tLimit;
	(*rays)[0].m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;//��������������ϵ������ʲ���
	(*rays)[0].m_theta = 0.0;
	(*rays)[0].m_costheta = 1.0;//�������߰��
	(*rays)[0].m_bsplit = false; //ǽ���Ե���߲�����
	(*rays)[0].m_vWedge = incident_ray.m_vWedge;
	(*rays)[0].m_vWedge.push_back(wedge);
	(*rays)[0].m_sensorDataId = incident_ray.m_sensorDataId;									//����������ID��ֵ

	(*rays)[1].m_Ori = wedge->m_point;  //�ֲ���һ�����ߵ�half_theta�հ�
	(*rays)[1].m_Dir = oa.Rotate(orientation * half_theta);//������ת����
	(*rays)[1].m_tMin = new_m_ft;
	(*rays)[1].m_tMax = new_m_ft;
	(*rays)[1].m_tLimit = incident_ray.m_tLimit;
	(*rays)[1].m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;//��������������ϵ������ʲ���
	(*rays)[1].m_theta = half_theta;
	(*rays)[1].m_costheta = cos_halftheta;//�������߰��
	(*rays)[1].m_bsplit = true;              //�������߾��з������ԣ��µĹ���Դ��
	(*rays)[1].m_vWedge = incident_ray.m_vWedge;
	(*rays)[1].m_vWedge.push_back(wedge);
	(*rays)[1].m_sensorDataId = incident_ray.m_sensorDataId;									//����������ID��ֵ

	for (unsigned i = 2; i < raynum - 1; i++) {//�м�Ƕȷ���
		(*rays)[i].m_Ori = wedge->m_point;
		(*rays)[i].m_Dir = oa.Rotate(orientation * delta_theta);//ÿ����תһ���Ƕȣ���ɢ��
		(*rays)[i].m_tMin = new_m_ft;
		(*rays)[i].m_tMax = new_m_ft;
		(*rays)[i].m_tLimit = incident_ray.m_tLimit;
		(*rays)[i].m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;
		(*rays)[i].m_theta = half_theta;
		(*rays)[i].m_costheta = cos_halftheta;
		(*rays)[i].m_bsplit = true;
		(*rays)[i].m_vWedge = incident_ray.m_vWedge;
		(*rays)[i].m_vWedge.push_back(wedge);
		(*rays)[i].m_sensorDataId = incident_ray.m_sensorDataId;									//����������ID��ֵ
	}

	(*rays)[raynum - 1].m_Ori = wedge->m_point;//���һ������,ǽ�巽��2
	(*rays)[raynum - 1].m_Dir = wedge->m_dir2;
	(*rays)[raynum - 1].m_tMin = new_m_ft;
	(*rays)[raynum - 1].m_tMax = new_m_ft;
	(*rays)[raynum - 1].m_tLimit = incident_ray.m_tLimit;
	(*rays)[raynum - 1].m_fRefractiveIndex = incident_ray.m_fRefractiveIndex;
	(*rays)[raynum - 1].m_theta = 0.0;
	(*rays)[raynum - 1].m_costheta = 1.0;
	(*rays)[raynum - 1].m_bsplit = false;//ǽ���Ե���߲�����
	(*rays)[raynum - 1].m_vWedge = incident_ray.m_vWedge;
	(*rays)[raynum - 1].m_vWedge.push_back(wedge);
	(*rays)[raynum - 1].m_sensorDataId = incident_ray.m_sensorDataId;								//����������ID��ֵ
	return true;
}

bool GetTerrainDiffractionPath(const Terrain* terrain, const Point3D& tx, const Point3D& rx, TerrainDiffractionPath*& outPath)
{
	//����1-�����Ƿ��ڵ��������ص���ͶӰ·��

	TerrainFacet* txFacet = terrain->GetTerrainFacetViaPoint(tx);
	TerrainFacet* rxFacet = terrain->GetTerrainFacetViaPoint(rx);
	if (txFacet == rxFacet)//0-ȷ���շ���������ڵ���Ԫ������ͬ���򷵻�false
		return false;

	std::vector<Point3D> routeAboveRay;//�������߸߶ȵ�·���㼯��
	std::vector<Point3D> route;		//�շ�����ĵ��ε㼯��
	//��ⷢ����ڶ�Ӧ��Ԫ�ϵ�ͶӰ����
	Point3D txOnFacet = txFacet->GetPointOnPlane(tx); //���������Ԫ�ϵ�����   
	Point3D rxOnFacet = rxFacet->GetPointOnPlane(rx); //���ջ�����Ԫ�ϵ�����
	route.push_back(txOnFacet); //�Ƚ��������������·����
	//���ڰ����������ȷ�������������ĵ��α��������Ϣ
	//����������ά����
	Vector3D rt = rx - tx;
	Vector3D rt_dir = Normalize(rt);
	Ray3DLite* ray3d = new Ray3DLite();
	ray3d->m_ori = tx;
	ray3d->m_dir = rt_dir;
	//����������ά����
	Ray2D* ray2d = new Ray2D();
	ray2d->m_Ori.x = ray3d->m_ori.x;
	ray2d->m_Ori.y = ray3d->m_ori.y;
	ray2d->m_Dir.x = ray3d->m_dir.x;
	ray2d->m_Dir.y = ray3d->m_dir.y;
	ray2d->m_Dir.Normalize(); //��ֵ����Ҫ��һ����ֵ

	TerrainFacet* curFacet = txFacet; /** @brief	��ǰ���㵽����Ԫ	*/
	TerrainSegment* curSegment = nullptr; /** @brief	��ǰ���㵽���߶�	*/
	TerrainSegment* prevSegment = nullptr; /** @brief	��һ�����㵽���߶�	*/
	RtLbsType t_cur; /** @brief	��ǰ�������߶��ཻ�ľ����ά	*/
	RtLbsType h_cur; /** @brief	��ǰ�������߶��ཻ�Ľ���߶ȣ����ڱȽ��Ƿ��������ཻ��	*/
	RtLbsType t_max = rt.LengthXY();
	bool hasIntersect = false; //���߱������ڵ�״̬
	while (true) { //��������ȵ�λ��
		prevSegment = curSegment;
		curSegment = curFacet->GetIntersectSegment(ray2d, prevSegment, &t_cur, &h_cur);//�����������Ԫ�ڲ����ཻ�߶�
		if (curSegment == nullptr)	//�������޷������������⣬��ֹ·��ѭ��
			break;
		RtLbsType rayHeight = ray3d->GetRayHeight(t_cur);
		//�����ά�������߶εĽ��㲢����h_curת��Ϊ��ά����
		Point2D pInTerrain2D = (*ray2d)(t_cur);
		Point3D pInTerrain3D(pInTerrain2D.x, pInTerrain2D.y, h_cur);
		route.push_back(pInTerrain3D);
		if (rayHeight <= h_cur) {//��·����������ڽ������ߵ�·����
			routeAboveRay.push_back(pInTerrain3D);
		}
		if (rayHeight <= h_cur || (rayHeight - h_cur) <= 1) { //���ߺ��߶εĸ߶Ȳ���1m���ڣ�����������������Ԫ��ģʽ��⾫ȷ��
			if (curFacet->GetIntersect(ray3d))//���������뵱ǰ��Ԫ���ཻ���
				hasIntersect = true; //�뻷���ཻ
		}
		curFacet = curSegment->GetAdjacentFacet(curFacet); //������һ����Ԫ
		if (curFacet == rxFacet || t_cur >= t_max) //�����������Ԫ����볬�����������������break
			break;
	}
	route.push_back(rxOnFacet); //��ӽ��ջ�����·��

	TerrainProfile* profile = new TerrainProfile();
	//���·����ÿ���ڵ�ĵ�Ų�����Ϣ
	std::vector<Material*> mats(route.size());															/** @brief	���ε��������	*/
	for (int i = 0; i < route.size(); ++i)
		mats[i] = terrain->GetMaterial(route[i]);														//���·����ÿ����Ĳ���ID
	profile->InitParameters(route, mats, tx, rx, terrain->m_averageRidgeGap);
	GetDiffractPathOverRidges(profile, outPath);	//��profile��������·��
	if (outPath != nullptr) {								//���ҽ���outPath��Ϊnullptrʱʹ��
		outPath->m_terrainDiffractionMode = terrain->m_propagationProperty.m_terrainDiffractionMode;			//�����������ģʽ�б�
	}
	//profile.WriteRidgesToFile("ridges.txt");		//��profile�еķ���д�뵽�ļ���-������
	profile->WriteProfileToFile("profile.txt");		//��profile�еĵ�������д�뵽�ļ���-������
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

	TerrainPathNode* txNode = new TerrainPathNode(profile->m_txPosition);				/** @brief	tx�ڵ�	*/
	outPath->m_nodes.push_back(txNode);											//���tx�ڵ�

	for (int i = 0; i < profile->m_ridges.size(); ++i) {									//���path�е�·���ڵ�
		if (profile->m_ridges[i]->m_isValid == false)
			continue;
		TerrainPathNode* newNode = new TerrainPathNode(profile->m_ridges[i]);
		outPath->m_nodes.push_back(newNode);
	}

	TerrainPathNode* rxNode = new TerrainPathNode(profile->m_rxPosition);				/** @brief	rx�ڵ�	*/
	outPath->m_nodes.push_back(rxNode);											//���rx�ڵ�

	//��������·����s����
	outPath->RectifySParameters();
}
