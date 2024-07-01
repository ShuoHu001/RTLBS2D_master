#include "scene.h"


Scene::Scene()
    : m_loadingTerrainFlag(false)
    , m_pAccelerator(nullptr)
{
}

Scene::~Scene()
{
    for (auto& element : m_segmentBuf) {
        delete element;
        element = nullptr;
    }
    m_segmentBuf.clear();
    std::vector<Segment2D*>().swap(m_segmentBuf);

	for (auto& element : m_wedgeBuf) {
		delete element;
        element = nullptr;
	}
    m_wedgeBuf.clear();
    std::vector<Wedge2D*>().swap(m_wedgeBuf);

    m_gpuSegmentBuf.clear();
    std::vector<Segment2DGPU>().swap(m_gpuSegmentBuf);

    m_gpuWedgeBuf.clear();
    std::vector<Wedge2DGPU>().swap(m_gpuWedgeBuf);

	for (auto& element : m_objects) {
		delete element;
        element = nullptr;
	}
    m_objects.clear();
    std::vector<Object2D*>().swap(m_objects);

	for (auto& element : m_transmitters) {
		delete element;
        element = nullptr;
	}
    m_transmitters.clear();
    std::vector<Transmitter*>().swap(m_transmitters);

	for (auto& element : m_receivers) {
		delete element;
        element = nullptr;
	}
    m_receivers.clear();
    std::vector<Receiver*>().swap(m_receivers);

	for (auto& element : m_sensors) {
		delete element;
        element = nullptr;
	}
    m_sensors.clear();
    std::vector<Sensor*>().swap(m_sensors);
    delete m_pAccelerator;
}

bool Scene::LoadScene(const SimConfig& config)
{
    bool loadBuildingFlag = false;                  /** @brief	��ȡ������״̬	*/
    bool loadVegetationFlag = false;                /** @brief	��ȡ�̻�ֲ��״̬	*/
    bool loadWallFlag = false;                      /** @brief	��ȡǽ��״̬	*/
    bool loadTerrainFlag = false;                   /** @brief	��ȡ����״̬	*/

    int objectNum = 0;                              /** @brief	��������	*/
    int segmentNum = 0;                             /** @brief	�߶�����	*/
    int wedgeNum = 0;                               /** @brief	��������	*/

    //���ز��ʿ�
    if (!m_materialLibrary.Init(config.m_materialLibraryConfig)) {
		LOG_WARNING << "Scene: material library config failed." << ENDL;
		return false;
    }

    //�������߿�
    if (!m_antennaLibrary.Init(config.m_antennaLibraryConfig)) {
		LOG_WARNING << "Scene: antenna library config failed." << ENDL;
		return false;
    }

    //��ȡ������
    if (!config.m_geometryConfig.m_buildingFile.empty() && !config.m_geometryConfig.m_buildingAttributeFile.empty()) {
        std::vector<Building2D*> buildings;         /** @brief	����ȡ�Ľ����Ｏ��	*/
        if (!LoadBuildingsFromFile(config.m_geometryConfig.m_buildingFile, config.m_geometryConfig.m_buildingAttributeFile, buildings))
            return false;
        for (int i = 0; i < buildings.size(); ++i) {
            buildings[i]->m_objectId = objectNum;
            m_objects.push_back(buildings[i]);
            RtLbsType refractiveN = m_materialLibrary.GetMaterial(buildings[i]->m_matId)->GetRefractiveN();
            for (auto it = buildings[i]->m_segments.begin(); it != buildings[i]->m_segments.end(); ++it) {          //�߶θ�ֵ
                (*it)->m_id = segmentNum++;                                                                   //�߶�ȫ��ID����
                (*it)->m_objectId = objectNum;
                (*it)->m_refractN = refractiveN;
                (*it)->m_propagationProperty = buildings[i]->m_propagationProperty;
                m_segmentBuf.push_back(*it);
            }
            for (auto it = buildings[i]->m_wedges.begin(); it != buildings[i]->m_wedges.end(); ++it) {              //������ֵ
                (*it)->m_globalId = wedgeNum++;                                                                     //����ȫ��ID����
                m_wedgeBuf.push_back(*it);
            }
            objectNum++;
        }
        loadBuildingFlag = true;
    }

    //��ȡ�̻���
    if (!config.m_geometryConfig.m_vegetationFile.empty() && !config.m_geometryConfig.m_vegetationAttributeFile.empty()) {
        std::vector<Vegetation2D*> vegetations;     /** @brief	����ȡ���̻�����	*/
        if (!LoadVegetationsFromFile(config.m_geometryConfig.m_vegetationFile, config.m_geometryConfig.m_vegetationAttributeFile, vegetations))
            return false;
        for (int i = 0; i < vegetations.size(); ++i) {
            vegetations[i]->m_objectId = static_cast<int>(m_objects.size());
            m_objects.push_back(vegetations[i]);
            RtLbsType refractiveN = m_materialLibrary.GetMaterial(vegetations[i]->m_matId)->GetRefractiveN();
            for (auto it = vegetations[i]->m_segments.begin(); it != vegetations[i]->m_segments.end(); ++it) {      //�߶θ�ֵ
                (*it)->m_id = segmentNum++;                                                                   //�߶�ȫ��ID����
                (*it)->m_refractN = refractiveN;
                (*it)->m_propagationProperty = vegetations[i]->m_propagationProperty;
                m_segmentBuf.push_back(*it);
            }
        }
        loadVegetationFlag = true;
    }

    //��ȡǽ��
    if (!config.m_geometryConfig.m_wallFile.empty() && !config.m_geometryConfig.m_wallAttributeFile.empty()) {
        std::vector<Wall2D*> walls;                 /** @brief	����ȡ��ǽ�弯��	*/
        if (!LoadWallsFromFile(config.m_geometryConfig.m_wallFile, config.m_geometryConfig.m_wallAttributeFile, walls))
            return false;
        for (int i = 0; i < walls.size(); ++i) {
            walls[i]->m_objectId = static_cast<int>(m_objects.size());
            m_objects.push_back(walls[i]);
            RtLbsType refractiveN = m_materialLibrary.GetMaterial(walls[i]->m_matId)->GetRefractiveN();
            for (auto it = walls[i]->m_segments.begin(); it != walls[i]->m_segments.end(); ++it) {                  //�߶θ�ֵ
                (*it)->m_id = segmentNum++;                                                                   //�߶�ȫ��ID����
                (*it)->m_refractN = refractiveN;
                (*it)->m_propagationProperty = walls[i]->m_propagationProperty;
                m_segmentBuf.push_back(*it);
            }
            for (auto it = walls[i]->m_wedges.begin(); it != walls[i]->m_wedges.end(); ++it) {
                (*it)->m_globalId = wedgeNum++;                                                                     //����ȫ��ID����
                m_wedgeBuf.push_back(*it);
            }
        }
        loadWallFlag = true;
    }


    //��ȡ���ָ߳���Ϣ��Terrain���Σ�
    if (config.m_geometryConfig.m_loadingTerrainFlag) {
        if (!m_terrain.Init(config.m_geometryConfig.m_terrainConfig, m_materialLibrary))
            return false;
        loadTerrainFlag = true;

        //��������object�ڵ�foundationHeight��ֵ
        for (int i = 0; i < objectNum; ++i) {
            RtLbsType foundationHeight = m_terrain.GetObjectFoundationHeight(m_objects[i]);
            m_objects[i]->m_foundationHeight = foundationHeight;
        }

    }




    //����ϵͳ�İ�Χ����Ϣ
    for (auto it = m_segmentBuf.begin(); it != m_segmentBuf.end(); ++it) {
        m_bbox.Union((*it)->m_bbox);
    }

    if (!loadBuildingFlag && !loadVegetationFlag && !loadWallFlag) {                                //������̻�����ǽ���������һ�ſ�
		LOG_ERROR << "Scene: inadequate geometry information." << ENDL;
		return false;
    }

    
    if (!InitSceneReceivers(config.m_receiverConfig.m_receiverconfigs, &m_antennaLibrary)) {        //��ʼ���������ջ�
		LOG_ERROR << "Scene: Receiver loading failed." << ENDL;
		return false;
    }
    if (config.m_systemMode == MODE_RT) {                                                           //����׷��ģʽ��ʼ�������
		if (!InitSceneTransmitters(config.m_transmitterConfig, &m_antennaLibrary)) {
			LOG_ERROR << "Scene: Transmitter loading failed." << ENDL;
			return false;
		}
    }
    else if (config.m_systemMode == MODE_LBS) {          //ϵͳΪ��λ����ʱ
        bool hasSimuError = config.m_lbsConfig.m_hasSimuError;
        if (!InitSceneSensors(config.m_sensorConfig, &m_antennaLibrary, hasSimuError)) {                           //��λģʽ��ʼ��������
			LOG_ERROR << "Scene: Sensor loading failed." << ENDL;
			return false;
        }
    }

	LOG_INFO << "Scene: configure scene success." << ENDL;
	return true;
    
}

void Scene::ConvertToGPUHostScene()
{
    //��Ԫ��ת��
    m_gpuSegmentBuf.resize(m_segmentBuf.size());
    for (size_t i = 0; i < m_segmentBuf.size(); ++i) {
        m_gpuSegmentBuf[i] = m_segmentBuf[i]->Convert2GPU();            //��CPU�е�����ת��ΪGPU�е�����
    }
    //�������ת��
    m_gpuWedgeBuf.resize(m_wedgeBuf.size());
    for (size_t i = 0; i < m_wedgeBuf.size(); ++i) {
        m_gpuWedgeBuf[i] = m_wedgeBuf[i]->Convert2GPU();
    }
    return;
}

bool Scene::GetIntersect(const Ray2D& r, Intersection2D* intersect) const
{
    if (m_pAccelerator == nullptr) {//��û�м��ٽṹ���������
        return _bfIntersect(r, intersect);
    }
    return m_pAccelerator->GetIntersect(r, intersect);//���м��ٽṹ����ö�Ӧ���ٽṹ���㷨
}

bool Scene::GetIntersect(const Segment2D& segment, Intersection2D* intersect) const
{
    Ray2D ray;
    ray.m_Ori = segment.m_ps + EPSILON * segment.m_dir;             //����EPSILONλ�ƣ���ֹ������
    ray.m_Dir = segment.m_dir;
    Intersection2D interTemp;
    if (GetIntersect(ray, &interTemp)) {
        if ((segment.m_length - interTemp.m_ft) > 2*EPSILON) {        //���߶γ��ȴ��ڽ�����룬��֤���߶μ����ϰ������
            if (intersect != nullptr) {
                *intersect = interTemp;
            }
            return true;
        }
    }
    return false;
}

bool Scene::IsBlock(const Point3D& ps, const Point3D& pe) const
{
    //�ж��Ƿ������ڵ�
    Point2D ps2d(ps.x, ps.y);                       /** @brief	��ʼ�������ά	*/
    Point2D pe2d(pe.x, pe.y);                       /** @brief	��ֹ�������ά	*/
    Ray2D ray2d(ps2d, pe2d);
    Intersection2D intersect2d;                     /** @brief	������Ϣ	*/  
    if (GetIntersect(ray2d, &intersect2d)) {
        RtLbsType seLen = (pe2d - ps2d).Length();                       /** @brief	��ά��ʼ��ֹ�㳤��	*/
        RtLbsType siLen = intersect2d.m_ft;                             /** @brief	��ʼ����뽻��ĳ���	*/
        if (siLen < seLen) {                                            //���ҽ����������С���ܾ���ʱ��������Ч
			if (intersect2d.m_segment->m_objectCategory == BUILDING2D ||
				intersect2d.m_segment->m_objectCategory == WALL2D) {
                int objectId = intersect2d.m_segment->m_id;                     /** @brief	�߶ζ�Ӧ������ID	*/
                Object2D* object = m_objects[objectId];                         /** @brief	�߶ζ�Ӧ������	*/
                RtLbsType objectHeight = object->GetObjectHeight();             /** @brief	��ȡ����߶�	*/
                RtLbsType ratio = siLen / seLen;                                /** @brief	�������ʼ����߳�����	*/
                RtLbsType deltaH = pe.z - ps.z;                                 /** @brief	��ĩ��ĸ߶Ȳ�	*/
                RtLbsType intersectPointHeight = ps.z + deltaH * ratio;         /** @brief	��������ľ��Ը߶�	*/
                if (intersectPointHeight < objectHeight)                        //������߶�С������ĸ߶ȣ�����Ϊ���ڵ�
                    return true;
			}
        }
        
    }
    //�ж��Ƿ񱻵����ڵ�
    if (m_loadingTerrainFlag) {
        if (m_terrain.IsBlock(ps, pe)) {                                        //���������ڵ����򷵻�true
            return true;
        }
    }
    return false;                                                               //���ڵ�,����false
}

bool Scene::IsBlockOnlyObject(const Point3D& ps, const Point3D& pe) const
{
	//�ж��Ƿ������ڵ�
	Point2D ps2d(ps.x, ps.y);                                                               /** @brief	��ʼ�������ά	*/
	Point2D pe2d(pe.x, pe.y);                                                               /** @brief	��ֹ�������ά	*/
	Ray2D ray2d(ps2d, pe2d);
	Intersection2D intersect2d;                                                             /** @brief	������Ϣ	*/
	if (GetIntersect(ray2d, &intersect2d)) {
		RtLbsType seLen = (pe2d - ps2d).Length();                                           /** @brief	��ά��ʼ��ֹ�㳤��	*/
		RtLbsType siLen = intersect2d.m_ft;                                                 /** @brief	��ʼ����뽻��ĳ���	*/
		if (siLen < seLen) {                                                                //���ҽ����������С���ܾ���ʱ��������Ч
			if (intersect2d.m_segment->m_objectCategory == BUILDING2D ||
				intersect2d.m_segment->m_objectCategory == WALL2D) {
				int objectId = intersect2d.m_segment->m_objectId;                           /** @brief	�߶ζ�Ӧ������ID	*/
                if (objectId < 0 || objectId > m_objects.size()) {
                    LOG_ERROR << "Scene: wrong object Id" << ENDL;
                    return false;
                }
				Object2D* object = m_objects[objectId];                                     /** @brief	�߶ζ�Ӧ������	*/
				RtLbsType objectHeight = object->GetObjectHeight();                         /** @brief	��ȡ����߶�	*/
				RtLbsType ratio = siLen / seLen;                                            /** @brief	�������ʼ����߳�����	*/
				RtLbsType deltaH = pe.z - ps.z;                                             /** @brief	��ĩ��ĸ߶Ȳ�	*/
				RtLbsType intersectPointHeight = ps.z + deltaH * ratio;                     /** @brief	��������ľ��Ը߶�	*/
				if (intersectPointHeight < objectHeight)                                    //������߶�С������ĸ߶ȣ�����Ϊ���ڵ�
					return true;
			}
		}

	}
    return false;
}

bool Scene::GetRayTubeWedges(const Ray2D& r, RayTreeNode* treenode, Intersection2D* intersect) const
{
    //�����ÿռ���ٽṹ���м���
    if (r.m_theta == 0.0 || r.m_bsplit == false)//�����߲����йܹ��ܻ����߲����з��ѹ��ܣ�����false
        return false;
    bool reValue = false;
    
    
    //���Vsource ����Դ
    RtLbsType t = r.m_fMax - r.m_fMin;//�����Դ�ľ���
    Point2D vSource = GetRayCoordinate(r, -t);
    std::vector<Wedge2D*> temp_wedges;
    
    Ray2D t_ray(r);//��������
    

    PathNode* node = treenode->m_data;

    for (Wedge2D* wedge : m_wedgeBuf) {
        if (node->m_type == NODE_DIFF) {
			if (wedge == node->m_wedge)//�����ظ������
				continue;
        }
        //���߹ܲ�׽�׶�
        Vector2D op = (wedge->m_point - vSource).Normalize();
        double p_costheta = op * r.m_Dir;
        if (p_costheta < r.m_costheta)//δ�����߹ܲ�׽
            continue;
		t_ray.m_Ori = vSource;//�����ɹ���Դ����������
		t_ray.m_Dir = op;


        //1-����Դ�������
        if (t < EPSILON) {//��ǰ�ڵ�Ϊ����Դ
            Intersection2D t_inter;//���Խ���
            if (!GetIntersect(t_ray, &t_inter)) {                                                                       //�����߹���ȴû�н��㣬�������
                LOG_WARNING << "reached numeric-limit:"<< EPSILON << ENDL;
                continue;
            }  
			if (t_inter.m_intersect != wedge->m_point)
				continue;
            temp_wedges.push_back(wedge);
            continue;
        }



        //2-�ǹ���Դ�������
        Intersection2D t_inter1;//���Խ���
        if (!node->m_segment->GetIntersect(t_ray, &t_inter1))                                                           //���������������Ԫ�޽��㣬������������ʵ���ϲ�����
            continue;
        if (t_inter1.m_intersect == wedge->m_point)//����Ԫ������Ŀ�������ͬ�����ų�����������ɼ��������ɵģ��������·����
            continue;
        Vector2D ip = (wedge->m_point - t_inter1.m_intersect).Normalize();
        if (op * ip < 0)//����Ԫ�ĺ󷽣�������
            continue;
        t_ray.m_Ori = t_inter1.m_intersect;//�����������λ�߶��ϵĵ�
        Intersection2D t_inter2;//���Խ���
		if (!GetIntersect(t_ray, &t_inter2)) {                                                                       //�����߹���ȴû�н��㣬�������
			LOG_WARNING << "reached numeric-limit:" << EPSILON << ENDL;
			continue;
		}
		if (t_inter2.m_intersect != wedge->m_point)
			continue;
		temp_wedges.push_back(wedge);
    }

    if(temp_wedges.size()>0)
        reValue = true;

    if (reValue) { //����intersect����,�����ظ�����
        intersect->m_wedges.clear();
        if (r.m_vWedge.size() != 0) {
			for (Wedge2D* t_wedge : temp_wedges) {
				auto it = std::find(r.m_vWedge.begin(), r.m_vWedge.end(), t_wedge);
				if (it == r.m_vWedge.end()) {//δ�ҵ���Ӧ��Ԫ��
					intersect->m_wedges.push_back(t_wedge);//�����ظ�Ԫ��ѹ�뽻��wedges�д洢
				}
			}
			if (intersect->m_wedges.size() == 0) {
				return false;
			}
        }
        else {
            intersect->m_wedges = temp_wedges;
        }
		

        if (intersect->m_type == NODE_LOS||intersect->m_type == NODE_INIT) {//�Ӿ�ڵ�����ǳ�ʼ���ڵ㣬intersectδ��ֵ
            intersect->m_type = NODE_DIFF;
            return reValue;
        }
        if (intersect->m_type == NODE_REFL) {
            intersect->m_type = NODE_REFLDIFF;
            return reValue;
        }
    } 
    return reValue;
}

void Scene::Release()
{
    SAFE_DELETE(m_pAccelerator);
    std::vector<Segment2D*>::iterator it = m_segmentBuf.begin();
    while (it != m_segmentBuf.end()) {
        delete* it;
        it++;
    }
}

void Scene::OutputLog() const
{
    //LOG_HEADER("Geometry Information");
    //LOG << "Scene File Name:\t" << m_filename << ENDL;
    //LOG << "Segment Count:\t" << (unsigned)m_segBuf.size() << ENDL 
    //if (m_pAccelerator)
    //    m_pAccelerator->OutputLog();
}

void Scene::PreProcess(ACCEL_TYPE type)
{
    switch (type){
    case ACCEL_NONE:
        break;
    case ACCEL_BVH:
        m_pAccelerator = new BVH();
        break;
    case ACCEL_KD:
        m_pAccelerator = new KDTree();
        break;
    case ACCEL_UG:
        m_pAccelerator = new UniGrid();
        break;
    case ACCEL_FastUG:
        m_pAccelerator = new UniformGrid();
        break;
    case ACCEL_SDF:
        m_pAccelerator = new SignedDistanceField();
        break;
    default:
        m_pAccelerator = new BVH();
        break;
    }

    //Ԥ��������Ԫ��Ϣ,������ṹ
    if (m_pAccelerator) {
        m_pAccelerator->SetPrimitives(&m_segmentBuf);
        m_pAccelerator->Build();
    }
}

//��ó�������ײ����
const BBox2D Scene::GetBBox() const
{
    if (m_pAccelerator != 0) {//���м����壬��ֱ�ӻ�ü�����İ�Χ��
        return m_pAccelerator->GetBBox();
    }
	return m_bbox;

}

RtLbsType Scene::_getPositionRefractN(Point2D& p)
{
	//��p��Ϊ�������꣬�������ߣ�������δ���л�������Ϊ�������ʣ������л�������Ϊ�����еĽ���
	Ray2D ray(p, Vector2D(static_cast<RtLbsType>(0.0), static_cast<RtLbsType>(1.0)));
	Intersection2D* inter = new Intersection2D();
    if (GetIntersect(ray, inter)) {
        if (ray.m_Dir * inter->m_segment->m_normal > 0)
            return inter->m_segment->m_refractN; //���������ڲ�
        else
            return inter->m_segment->m_refractNOut; //���������ⲿ�����ⲿ�Ĳ���
    }
	return 1.0f; // �뻷���޽���,�ÿ����Ĳ���
}

bool Scene::IsValidPoint(const Point3D& p) const
{
    if (!m_bbox.IsContainPoint(p))
        return false;
    //���������Ƿ�λ�ڽ�����֮��
    for (auto it = m_objects.begin(); it != m_objects.end(); ++it) {
        const Object2D* obejct = *it;
        if (obejct->IsContain(p))           //���������������㣬�򷵻�false,�õ���Ч
            return false;
    }

    if (m_loadingTerrainFlag) {
        //���������Ƿ�λ�ڵ���֮��
		if (!m_terrain.IsValidPoint(p))
			return false;
    }
    return true;
}

bool Scene::IsValidPoint(const Point2D& p) const
{
	if (!m_bbox.IsContainPoint(p))
		return false;
	//���������Ƿ�λ�ڽ�����֮��
	for (auto it = m_objects.begin(); it != m_objects.end(); ++it) {
		const Object2D* obejct = *it;
		if (obejct->IsContain(p))           //���������������㣬�򷵻�false,�õ���Ч
			return false;
	}
    return true;
}

bool Scene::InitSceneTransmitters(const TransmitterCollectionConfig& config, AntennaLibrary* antLibrary)
{
    int validTxNum = 0;                                                 /** @brief	��Ч�ķ��������	*/
    InitTransmitter(config, antLibrary, m_transmitters);                //��ʼ�������
    for (int i = 0; i < m_transmitters.size(); ++i) {
        if (!IsValidPoint(m_transmitters[i]->m_position)) {
            m_transmitters[i]->m_isValid = false;                       //�����������ķ��������Ϊ��Ч
            continue;
        }
        validTxNum++;
    }
	if (validTxNum == 0) {
		LOG_WARNING << "Scene: tx num up to " << m_transmitters.size() << ", valid num is " << validTxNum << "." << ENDL;
		return false;
	}
	LOG_INFO << "Scene: tx num up to " << m_transmitters.size() << ", valid num is " << validTxNum << "." << ENDL;
	return true;
}

bool Scene::InitSceneReceivers(const std::vector<ReceiverUnitConfig>& configs, AntennaLibrary* antLibrary)
{
    int validRxNum = 0;                                                 /** @brief	��Ч�Ľ��ջ�����	*/
    InitReceiver(configs, antLibrary, m_receivers);                     //��ʼ�����ջ�
    for (int i = 0; i < m_receivers.size(); ++i) {
        if (!IsValidPoint(m_receivers[i]->m_position) || _isRepeatToTransmitter(m_receivers[i]->m_position)) {      //�����������ķ�����趨Ϊ��Ч
            m_receivers[i]->m_isValid = false;
            continue;
        }
        validRxNum++;
    }
	if (validRxNum == 0) {
		LOG_WARNING << "Scene: rx num up to " << m_receivers.size() << ", valid num is " << validRxNum << "." << ENDL;
		return false;
	}
	LOG_INFO << "Scene: rx num up to " << m_receivers.size() << ", valid num is " << validRxNum << "." << ENDL;
	return true;
}

bool Scene::InitSceneSensors(const SensorCollectionConfig& config, AntennaLibrary* antLibrary, bool hasSimuError)
{
	int validSensorNum = 0;                                                 /** @brief	��Ч�Ĵ���������	*/
    int sensorDataId = 0;                                                   /** @brief	����������ID	*/
    InitSensors(config, antLibrary, m_sensors);                             //��ʼ���𴫸���
	for (int i = 0; i < m_sensors.size(); ++i) {
		if (!IsValidPoint(m_sensors[i]->m_position)) {
            m_sensors[i]->m_isValid = false;                                //�����������Ĵ���������Ϊ��Ч
			continue;
		}
        validSensorNum++;

        //��sensor���������Ӳ����������ģ����ʵ���������flagΪfalse�򲻽���ģ��������ӳ�
        if (hasSimuError) {
            m_sensors[i]->AddSimulationError();
        }
        //��sensor�е����ݿ�����sensordatalibrary�У����޸Ķ�Ӧ����ID�ʹ�����ID
        m_sensorDataLibrary.AddNew(m_sensors[i]->m_sensorDataCollection);
	}
	if (validSensorNum == 0) {
		LOG_WARNING << "Scene: Sensor num up to " << m_sensors.size() << ", valid num is " << validSensorNum << "." << ENDL;
		return false;
	}
	LOG_INFO << "Scene: Sensor num up to " << m_sensors.size() << ", valid num is " << validSensorNum << "." << ENDL;
	return true;
}

bool Scene::GetGroundReflectPaths(const Point3D& ps, const Point3D& pe, std::vector<RayPath3D*>& outPaths) const
{
	if (m_loadingTerrainFlag == true) {                                                                             //�������˵��Σ����ǵ��η���
		m_terrain.GetTerrainReflectionPaths(ps, pe, outPaths);
	}
	else {                                                                                                          //��δ���ص��Σ������0ƽ����м�����淴�侶
		outPaths.resize(1);                                                                                         //0ƽ�����ҽ���һ�����侶
		Point2D ps2d(ps.x, ps.y);                                                                                   /** @brief	��ʼ����XOYƽ���ϵ�����	*/
		Point3D pe1(pe.x, pe.y, -1 * pe.z);                                                                         /** @brief	pe����0ƽ��ľ����	*/
		RtLbsType ratio = pe.z / (ps.z + pe.z);                                                                     /** @brief	�ȱȱ�ֵ	*/
		Vector2D dir = Vector2D(pe1.x - ps.x, pe1.y - ps.y).Normalize();                                            /** @brief	�����ָ��Ĩ�㾵��ĵ�λ���򣨶�ά��	*/
		RtLbsType len = (pe - ps).LengthXY();                                                                       /** @brief	�߶���XOYƽ���ϵ�ͶӰ	*/
		RtLbsType srLen = len - len * ratio;                                                                        /** @brief	��㵽�����Ķ�ά����	*/
		Point2D rp2d = ps2d + dir * srLen;                                                                          /** @brief	XOYƽ���ϵķ��������	*/
		Point3D reflPoint(rp2d.x, rp2d.y, 0);                                                                       /** @brief	��ά�ռ��еķ��������	*/
		//��������·��
		PathNode3D* sNode = new PathNode3D(ps, NODE_ROOT);                                                          /** @brief	����·����ʼ�ڵ�	*/
		PathNode3D* eNode = new PathNode3D(pe, NODE_STOP);                                                          /** @brief	����·����ֹ�ڵ�	*/
        PathNode3D* rNode = new PathNode3D(reflPoint, NODE_REFL);                                                   /** @brief	����·������ڵ�	*/
        RayPath3D* newPath = new RayPath3D();
        newPath->Union(sNode);
        newPath->Union(rNode);
        newPath->Union(eNode);
        newPath->m_type = RAYPATH_TERRAIN_REFLECTION;
        outPaths[0] = newPath;
    }
    return true;
}

bool Scene::GetGroundDiffractionPath(const Point3D& ps, const Point3D& pe, TerrainDiffractionPath*& outPath) const
{
    if (m_loadingTerrainFlag) {
        return m_terrain.GetTerrainDiffractionPath(ps, pe, outPath);
    }
    return false;
}

bool Scene::IsValidRayPath(const RayPath3D* path) const
{
    bool isValidPath = true;                                   /** @brief	�Ƿ�����Ч·��	*/  
    if (path->m_type == RAYPATH_COMMON) {
		//���ÿ��node�е��Ƿ񳬹���Ӧ������ĸ߶�
		for (int i = 1; i < path->m_nodes.size() - 1; ++i) {        //��Ҫ������β�ڵ�
            if (path->m_nodes[i]->m_type == NODE_ETRANIN ||
                path->m_nodes[i]->m_type == NODE_ETRANOUT) {
                continue;
            }
				
			int nodeObjectId = 0;
			if (path->m_nodes[i]->m_type == NODE_REFL ||
                path->m_nodes[i]->m_type == NODE_TRANIN ||
                path->m_nodes[i]->m_type == NODE_TRANOUT) {
				nodeObjectId = path->m_nodes[i]->m_primitive->m_objectId;
			}
			else if (path->m_nodes[i]->m_type == NODE_DIFF) {
				nodeObjectId = path->m_nodes[i]->m_wedge->m_face1->m_objectId;
			}

			RtLbsType objHMin = m_objects[nodeObjectId]->m_foundationHeight;                /** @brief	������С�߶�	*/
			RtLbsType objHMax = m_objects[nodeObjectId]->GetObjectHeight();                 /** @brief	�������߶� 	*/
			if (path->m_nodes[i]->m_point.z > objHMax) {
				isValidPath = false;
			}
		}

		if (isValidPath == false) {                                                    //��·������Ч�������������һ���ж�
			return false;
		}

		//������·�����зֶΣ��ж��Ƿ񱻵����ڵ�
		if (m_loadingTerrainFlag) {
			for (int i = 0; i < path->m_nodes.size() - 1; ++i) {
				const Point3D ps = path->m_nodes[i]->m_point;                   /** @brief	·����֧��ʼ��	*/
				const Point3D pe = path->m_nodes[i + 1]->m_point;               /** @brief	·����֧��ֹ��	*/
				if (m_terrain.IsBlock(ps, pe)) {                                //������·����֧�������ڵ���·����Ч
					isValidPath = false;
				}
			}
		}
    }
    else if (path->m_type == RAYPATH_TERRAIN_REFLECTION) {
		//��·�����зֶΣ��ж��Ƿ񱻻��������ڵ�
		for (int i = 0; i < path->m_nodes.size() - 1; ++i) {
			const Point3D ps = path->m_nodes[i]->m_point;                   /** @brief	·����֧��ʼ��	*/
			const Point3D pe = path->m_nodes[i + 1]->m_point;               /** @brief	·����֧��ֹ��	*/
			if (IsBlockOnlyObject(ps, pe)) {                                //������·����֧�������ڵ���·����Ч
                isValidPath = false;
			}
		}
    }
   
    return isValidPath;

}

bool Scene::IsValidRayPath(const TerrainDiffractionPath* path) const
{
    //��·�����зֶΣ��ж��Ƿ񱻻��������ڵ�
    for (int i = 0; i < path->m_nodes.size() - 1; ++i) {
        const Point3D ps = path->m_nodes[i]->m_point;                   /** @brief	·����֧��ʼ��	*/
        const Point3D pe = path->m_nodes[i + 1]->m_point;               /** @brief	·����֧��ֹ��	*/
        if (IsBlockOnlyObject(ps, pe)) {                                //������·����֧�������ڵ���·����Ч
            return false;
        }
    }
    return true;
}

bool Scene::_bfIntersect(const Ray2D& r, Intersection2D* intersect) const
{
    
    if (intersect)//���³�ʼ��intersect��ֹ����
        intersect->m_ft = FLT_MAX;
    int n = static_cast<int>(m_segmentBuf.size());
    Intersection2D curIntersect;//��ǰ���뽻��
    bool hasIntersect = false;
    for (Segment2D* segment : m_segmentBuf) {
        bool flag = segment->GetIntersect(r, &curIntersect);
        if (flag && intersect == nullptr)//�ڲ���⽻�������£�flagΪ������뻷���ཻ
            return true;
        if (flag && curIntersect.m_ft < intersect->m_ft) {//���½�����Ϣ״̬��ʼ�ձ��־������
            hasIntersect = true;
            *intersect = curIntersect;
        }
    }
    
    return hasIntersect;
}

bool Scene::_isRepeatToTransmitter(const Point3D& p) const
{
	for (int i = 0; i < m_transmitters.size(); ++i) {
		if (m_transmitters[i]->m_position == p)
			return true;
	}
	return false;
}




