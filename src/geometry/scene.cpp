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
    bool loadBuildingFlag = false;                  /** @brief	读取建筑物状态	*/
    bool loadVegetationFlag = false;                /** @brief	读取绿化植被状态	*/
    bool loadWallFlag = false;                      /** @brief	读取墙体状态	*/
    bool loadTerrainFlag = false;                   /** @brief	读取地形状态	*/

    int objectNum = 0;                              /** @brief	物体数量	*/
    int segmentNum = 0;                             /** @brief	线段数量	*/
    int wedgeNum = 0;                               /** @brief	棱劈数量	*/

    //加载材质库
    if (!m_materialLibrary.Init(config.m_materialLibraryConfig)) {
		LOG_WARNING << "Scene: material library config failed." << ENDL;
		return false;
    }

    //加载天线库
    if (!m_antennaLibrary.Init(config.m_antennaLibraryConfig)) {
		LOG_WARNING << "Scene: antenna library config failed." << ENDL;
		return false;
    }

    //读取建筑物
    if (!config.m_geometryConfig.m_buildingFile.empty() && !config.m_geometryConfig.m_buildingAttributeFile.empty()) {
        std::vector<Building2D*> buildings;         /** @brief	待读取的建筑物集合	*/
        if (!LoadBuildingsFromFile(config.m_geometryConfig.m_buildingFile, config.m_geometryConfig.m_buildingAttributeFile, buildings))
            return false;
        for (int i = 0; i < buildings.size(); ++i) {
            buildings[i]->m_objectId = objectNum;
            m_objects.push_back(buildings[i]);
            RtLbsType refractiveN = m_materialLibrary.GetMaterial(buildings[i]->m_matId)->GetRefractiveN();
            for (auto it = buildings[i]->m_segments.begin(); it != buildings[i]->m_segments.end(); ++it) {          //线段赋值
                (*it)->m_id = segmentNum++;                                                                   //线段全局ID更新
                (*it)->m_objectId = objectNum;
                (*it)->m_refractN = refractiveN;
                (*it)->m_propagationProperty = buildings[i]->m_propagationProperty;
                m_segmentBuf.push_back(*it);
            }
            for (auto it = buildings[i]->m_wedges.begin(); it != buildings[i]->m_wedges.end(); ++it) {              //棱劈赋值
                (*it)->m_globalId = wedgeNum++;                                                                     //棱劈全局ID更新
                m_wedgeBuf.push_back(*it);
            }
            objectNum++;
        }
        loadBuildingFlag = true;
    }

    //读取绿化带
    if (!config.m_geometryConfig.m_vegetationFile.empty() && !config.m_geometryConfig.m_vegetationAttributeFile.empty()) {
        std::vector<Vegetation2D*> vegetations;     /** @brief	待读取的绿化集合	*/
        if (!LoadVegetationsFromFile(config.m_geometryConfig.m_vegetationFile, config.m_geometryConfig.m_vegetationAttributeFile, vegetations))
            return false;
        for (int i = 0; i < vegetations.size(); ++i) {
            vegetations[i]->m_objectId = static_cast<int>(m_objects.size());
            m_objects.push_back(vegetations[i]);
            RtLbsType refractiveN = m_materialLibrary.GetMaterial(vegetations[i]->m_matId)->GetRefractiveN();
            for (auto it = vegetations[i]->m_segments.begin(); it != vegetations[i]->m_segments.end(); ++it) {      //线段赋值
                (*it)->m_id = segmentNum++;                                                                   //线段全局ID更新
                (*it)->m_refractN = refractiveN;
                (*it)->m_propagationProperty = vegetations[i]->m_propagationProperty;
                m_segmentBuf.push_back(*it);
            }
        }
        loadVegetationFlag = true;
    }

    //读取墙体
    if (!config.m_geometryConfig.m_wallFile.empty() && !config.m_geometryConfig.m_wallAttributeFile.empty()) {
        std::vector<Wall2D*> walls;                 /** @brief	待读取的墙体集合	*/
        if (!LoadWallsFromFile(config.m_geometryConfig.m_wallFile, config.m_geometryConfig.m_wallAttributeFile, walls))
            return false;
        for (int i = 0; i < walls.size(); ++i) {
            walls[i]->m_objectId = static_cast<int>(m_objects.size());
            m_objects.push_back(walls[i]);
            RtLbsType refractiveN = m_materialLibrary.GetMaterial(walls[i]->m_matId)->GetRefractiveN();
            for (auto it = walls[i]->m_segments.begin(); it != walls[i]->m_segments.end(); ++it) {                  //线段赋值
                (*it)->m_id = segmentNum++;                                                                   //线段全局ID更新
                (*it)->m_refractN = refractiveN;
                (*it)->m_propagationProperty = walls[i]->m_propagationProperty;
                m_segmentBuf.push_back(*it);
            }
            for (auto it = walls[i]->m_wedges.begin(); it != walls[i]->m_wedges.end(); ++it) {
                (*it)->m_globalId = wedgeNum++;                                                                     //棱劈全局ID更新
                m_wedgeBuf.push_back(*it);
            }
        }
        loadWallFlag = true;
    }


    //读取数字高程信息（Terrain地形）
    if (config.m_geometryConfig.m_loadingTerrainFlag) {
        if (!m_terrain.Init(config.m_geometryConfig.m_terrainConfig, m_materialLibrary))
            return false;
        loadTerrainFlag = true;

        //更新所有object在的foundationHeight数值
        for (int i = 0; i < objectNum; ++i) {
            RtLbsType foundationHeight = m_terrain.GetObjectFoundationHeight(m_objects[i]);
            m_objects[i]->m_foundationHeight = foundationHeight;
        }

    }




    //更新系统的包围盒信息
    for (auto it = m_segmentBuf.begin(); it != m_segmentBuf.end(); ++it) {
        m_bbox.Union((*it)->m_bbox);
    }

    if (!loadBuildingFlag && !loadVegetationFlag && !loadWallFlag) {                                //建筑物、绿化带和墙体必须有其一才可
		LOG_ERROR << "Scene: inadequate geometry information." << ENDL;
		return false;
    }

    
    if (!InitSceneReceivers(config.m_receiverConfig.m_receiverconfigs, &m_antennaLibrary)) {        //初始化场景接收机
		LOG_ERROR << "Scene: Receiver loading failed." << ENDL;
		return false;
    }
    if (config.m_systemMode == MODE_RT) {                                                           //射线追踪模式初始化发射机
		if (!InitSceneTransmitters(config.m_transmitterConfig, &m_antennaLibrary)) {
			LOG_ERROR << "Scene: Transmitter loading failed." << ENDL;
			return false;
		}
    }
    else if (config.m_systemMode == MODE_LBS) {          //系统为定位服务时
        bool hasSimuError = config.m_lbsConfig.m_hasSimuError;
        if (!InitSceneSensors(config.m_sensorConfig, &m_antennaLibrary, hasSimuError)) {                           //定位模式初始化传感器
			LOG_ERROR << "Scene: Sensor loading failed." << ENDL;
			return false;
        }
    }

	LOG_INFO << "Scene: configure scene success." << ENDL;
	return true;
    
}

void Scene::ConvertToGPUHostScene()
{
    //面元的转换
    m_gpuSegmentBuf.resize(m_segmentBuf.size());
    for (size_t i = 0; i < m_segmentBuf.size(); ++i) {
        m_gpuSegmentBuf[i] = m_segmentBuf[i]->Convert2GPU();            //将CPU中的数据转换为GPU中的数据
    }
    //绕射棱的转换
    m_gpuWedgeBuf.resize(m_wedgeBuf.size());
    for (size_t i = 0; i < m_wedgeBuf.size(); ++i) {
        m_gpuWedgeBuf[i] = m_wedgeBuf[i]->Convert2GPU();
    }
    return;
}

bool Scene::GetIntersect(const Ray2D& r, Intersection2D* intersect) const
{
    if (m_pAccelerator == nullptr) {//若没有加速结构，则暴力求解
        return _bfIntersect(r, intersect);
    }
    return m_pAccelerator->GetIntersect(r, intersect);//若有加速结构则采用对应加速结构的算法
}

bool Scene::GetIntersect(const Segment2D& segment, Intersection2D* intersect) const
{
    Ray2D ray;
    ray.m_Ori = segment.m_ps + EPSILON * segment.m_dir;             //扩大EPSILON位移，防止求解起点
    ray.m_Dir = segment.m_dir;
    Intersection2D interTemp;
    if (GetIntersect(ray, &interTemp)) {
        if ((segment.m_length - interTemp.m_ft) > 2*EPSILON) {        //若线段长度大于交点距离，则证明线段间有障碍物存在
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
    //判定是否被物体遮挡
    Point2D ps2d(ps.x, ps.y);                       /** @brief	起始点坐标二维	*/
    Point2D pe2d(pe.x, pe.y);                       /** @brief	终止点坐标二维	*/
    Ray2D ray2d(ps2d, pe2d);
    Intersection2D intersect2d;                     /** @brief	交点信息	*/  
    if (GetIntersect(ray2d, &intersect2d)) {
        RtLbsType seLen = (pe2d - ps2d).Length();                       /** @brief	二维起始终止点长度	*/
        RtLbsType siLen = intersect2d.m_ft;                             /** @brief	起始点距离交点的长度	*/
        if (siLen < seLen) {                                            //当且仅当交点距离小于总距离时，交点有效
			if (intersect2d.m_segment->m_objectCategory == BUILDING2D ||
				intersect2d.m_segment->m_objectCategory == WALL2D) {
                int objectId = intersect2d.m_segment->m_id;                     /** @brief	线段对应的物体ID	*/
                Object2D* object = m_objects[objectId];                         /** @brief	线段对应的物体	*/
                RtLbsType objectHeight = object->GetObjectHeight();             /** @brief	获取物体高度	*/
                RtLbsType ratio = siLen / seLen;                                /** @brief	交点距起始点的线长比例	*/
                RtLbsType deltaH = pe.z - ps.z;                                 /** @brief	起末点的高度差	*/
                RtLbsType intersectPointHeight = ps.z + deltaH * ratio;         /** @brief	交点坐标的绝对高度	*/
                if (intersectPointHeight < objectHeight)                        //若交点高度小于物体的高度，则视为被遮挡
                    return true;
			}
        }
        
    }
    //判定是否被地形遮挡
    if (m_loadingTerrainFlag) {
        if (m_terrain.IsBlock(ps, pe)) {                                        //若被地形遮挡，则返回true
            return true;
        }
    }
    return false;                                                               //无遮挡,返回false
}

bool Scene::IsBlockOnlyObject(const Point3D& ps, const Point3D& pe) const
{
	//判定是否被物体遮挡
	Point2D ps2d(ps.x, ps.y);                                                               /** @brief	起始点坐标二维	*/
	Point2D pe2d(pe.x, pe.y);                                                               /** @brief	终止点坐标二维	*/
	Ray2D ray2d(ps2d, pe2d);
	Intersection2D intersect2d;                                                             /** @brief	交点信息	*/
	if (GetIntersect(ray2d, &intersect2d)) {
		RtLbsType seLen = (pe2d - ps2d).Length();                                           /** @brief	二维起始终止点长度	*/
		RtLbsType siLen = intersect2d.m_ft;                                                 /** @brief	起始点距离交点的长度	*/
		if (siLen < seLen) {                                                                //当且仅当交点距离小于总距离时，交点有效
			if (intersect2d.m_segment->m_objectCategory == BUILDING2D ||
				intersect2d.m_segment->m_objectCategory == WALL2D) {
				int objectId = intersect2d.m_segment->m_objectId;                           /** @brief	线段对应的物体ID	*/
                if (objectId < 0 || objectId > m_objects.size()) {
                    LOG_ERROR << "Scene: wrong object Id" << ENDL;
                    return false;
                }
				Object2D* object = m_objects[objectId];                                     /** @brief	线段对应的物体	*/
				RtLbsType objectHeight = object->GetObjectHeight();                         /** @brief	获取物体高度	*/
				RtLbsType ratio = siLen / seLen;                                            /** @brief	交点距起始点的线长比例	*/
				RtLbsType deltaH = pe.z - ps.z;                                             /** @brief	起末点的高度差	*/
				RtLbsType intersectPointHeight = ps.z + deltaH * ratio;                     /** @brief	交点坐标的绝对高度	*/
				if (intersectPointHeight < objectHeight)                                    //若交点高度小于物体的高度，则视为被遮挡
					return true;
			}
		}

	}
    return false;
}

bool Scene::GetRayTubeWedges(const Ray2D& r, RayTreeNode* treenode, Intersection2D* intersect) const
{
    //考虑用空间加速结构进行加速
    if (r.m_theta == 0.0 || r.m_bsplit == false)//若射线不具有管功能或射线不具有分裂功能，返回false
        return false;
    bool reValue = false;
    
    
    //求解Vsource 广义源
    RtLbsType t = r.m_fMax - r.m_fMin;//与广义源的距离
    Point2D vSource = GetRayCoordinate(r, -t);
    std::vector<Wedge2D*> temp_wedges;
    
    Ray2D t_ray(r);//测试射线
    

    PathNode* node = treenode->m_data;

    for (Wedge2D* wedge : m_wedgeBuf) {
        if (node->m_type == NODE_DIFF) {
			if (wedge == node->m_wedge)//避免重复的棱角
				continue;
        }
        //射线管捕捉阶段
        Vector2D op = (wedge->m_point - vSource).Normalize();
        double p_costheta = op * r.m_Dir;
        if (p_costheta < r.m_costheta)//未被射线管捕捉
            continue;
		t_ray.m_Ori = vSource;//射线由广义源引出的射线
		t_ray.m_Dir = op;


        //1-广义源情况讨论
        if (t < EPSILON) {//当前节点为广义源
            Intersection2D t_inter;//测试交点
            if (!GetIntersect(t_ray, &t_inter)) {                                                                       //在射线管内却没有交点，计算误差
                LOG_WARNING << "reached numeric-limit:"<< EPSILON << ENDL;
                continue;
            }  
			if (t_inter.m_intersect != wedge->m_point)
				continue;
            temp_wedges.push_back(wedge);
            continue;
        }



        //2-非广义源情况讨论
        Intersection2D t_inter1;//测试交点
        if (!node->m_segment->GetIntersect(t_ray, &t_inter1))                                                           //若构造的射线与面元无交点，表明该条射线实际上不存在
            continue;
        if (t_inter1.m_intersect == wedge->m_point)//若面元交点与目标棱角相同，则排除该种情况（由计算误差造成的，不合理的路径）
            continue;
        Vector2D ip = (wedge->m_point - t_inter1.m_intersect).Normalize();
        if (op * ip < 0)//在面元的后方，不存在
            continue;
        t_ray.m_Ori = t_inter1.m_intersect;//射线起点修正位线段上的点
        Intersection2D t_inter2;//测试交点
		if (!GetIntersect(t_ray, &t_inter2)) {                                                                       //在射线管内却没有交点，计算误差
			LOG_WARNING << "reached numeric-limit:" << EPSILON << ENDL;
			continue;
		}
		if (t_inter2.m_intersect != wedge->m_point)
			continue;
		temp_wedges.push_back(wedge);
    }

    if(temp_wedges.size()>0)
        reValue = true;

    if (reValue) { //修正intersect属性,避免重复绕射
        intersect->m_wedges.clear();
        if (r.m_vWedge.size() != 0) {
			for (Wedge2D* t_wedge : temp_wedges) {
				auto it = std::find(r.m_vWedge.begin(), r.m_vWedge.end(), t_wedge);
				if (it == r.m_vWedge.end()) {//未找到对应的元素
					intersect->m_wedges.push_back(t_wedge);//将不重复元素压入交点wedges中存储
				}
			}
			if (intersect->m_wedges.size() == 0) {
				return false;
			}
        }
        else {
            intersect->m_wedges = temp_wedges;
        }
		

        if (intersect->m_type == NODE_LOS||intersect->m_type == NODE_INIT) {//视距节点或者是初始化节点，intersect未赋值
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

    //预处理环境面元信息,加速体结构
    if (m_pAccelerator) {
        m_pAccelerator->SetPrimitives(&m_segmentBuf);
        m_pAccelerator->Build();
    }
}

//获得场景的碰撞盒子
const BBox2D Scene::GetBBox() const
{
    if (m_pAccelerator != 0) {//若有加速体，则直接获得加速体的包围盒
        return m_pAccelerator->GetBBox();
    }
	return m_bbox;

}

RtLbsType Scene::_getPositionRefractN(Point2D& p)
{
	//以p点为中心坐标，发射射线，若射线未击中环境，则为空气介质，若击中环境，则为环境中的介质
	Ray2D ray(p, Vector2D(static_cast<RtLbsType>(0.0), static_cast<RtLbsType>(1.0)));
	Intersection2D* inter = new Intersection2D();
    if (GetIntersect(ray, inter)) {
        if (ray.m_Dir * inter->m_segment->m_normal > 0)
            return inter->m_segment->m_refractN; //点在物体内部
        else
            return inter->m_segment->m_refractNOut; //点在物体外部，用外部的参数
    }
	return 1.0f; // 与环境无交点,用空气的参数
}

bool Scene::IsValidPoint(const Point3D& p) const
{
    if (!m_bbox.IsContainPoint(p))
        return false;
    //检测坐标点是否位于建筑物之内
    for (auto it = m_objects.begin(); it != m_objects.end(); ++it) {
        const Object2D* obejct = *it;
        if (obejct->IsContain(p))           //若物体包含该坐标点，则返回false,该点无效
            return false;
    }

    if (m_loadingTerrainFlag) {
        //检测坐标点是否位于地形之下
		if (!m_terrain.IsValidPoint(p))
			return false;
    }
    return true;
}

bool Scene::IsValidPoint(const Point2D& p) const
{
	if (!m_bbox.IsContainPoint(p))
		return false;
	//检测坐标点是否位于建筑物之内
	for (auto it = m_objects.begin(); it != m_objects.end(); ++it) {
		const Object2D* obejct = *it;
		if (obejct->IsContain(p))           //若物体包含该坐标点，则返回false,该点无效
			return false;
	}
    return true;
}

bool Scene::InitSceneTransmitters(const TransmitterCollectionConfig& config, AntennaLibrary* antLibrary)
{
    int validTxNum = 0;                                                 /** @brief	有效的发射机数量	*/
    InitTransmitter(config, antLibrary, m_transmitters);                //初始化发射机
    for (int i = 0; i < m_transmitters.size(); ++i) {
        if (!IsValidPoint(m_transmitters[i]->m_position)) {
            m_transmitters[i]->m_isValid = false;                       //不满足条件的发射机设置为无效
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
    int validRxNum = 0;                                                 /** @brief	有效的接收机数量	*/
    InitReceiver(configs, antLibrary, m_receivers);                     //初始化接收机
    for (int i = 0; i < m_receivers.size(); ++i) {
        if (!IsValidPoint(m_receivers[i]->m_position) || _isRepeatToTransmitter(m_receivers[i]->m_position)) {      //不满足条件的发射机设定为无效
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
	int validSensorNum = 0;                                                 /** @brief	有效的传感器数量	*/
    int sensorDataId = 0;                                                   /** @brief	传感器数据ID	*/
    InitSensors(config, antLibrary, m_sensors);                             //初始化拆传感器
	for (int i = 0; i < m_sensors.size(); ++i) {
		if (!IsValidPoint(m_sensors[i]->m_position)) {
            m_sensors[i]->m_isValid = false;                                //不满足条件的传感器设置为无效
			continue;
		}
        validSensorNum++;

        //在sensor基础上增加测量误差用于模拟真实的情况，若flag为false则不进行模拟仿真误差加持
        if (hasSimuError) {
            m_sensors[i]->AddSimulationError();
        }
        //将sensor中的数据拷贝至sensordatalibrary中，并修改对应数据ID和传感器ID
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
	if (m_loadingTerrainFlag == true) {                                                                             //若加载了地形，则考虑地形反射
		m_terrain.GetTerrainReflectionPaths(ps, pe, outPaths);
	}
	else {                                                                                                          //若未加载地形，则基于0平面进行计算地面反射径
		outPaths.resize(1);                                                                                         //0平面有且仅有一条反射径
		Point2D ps2d(ps.x, ps.y);                                                                                   /** @brief	起始点在XOY平面上的坐标	*/
		Point3D pe1(pe.x, pe.y, -1 * pe.z);                                                                         /** @brief	pe关于0平面的镜像点	*/
		RtLbsType ratio = pe.z / (ps.z + pe.z);                                                                     /** @brief	等比比值	*/
		Vector2D dir = Vector2D(pe1.x - ps.x, pe1.y - ps.y).Normalize();                                            /** @brief	由起点指向抹点镜像的单位方向（二维）	*/
		RtLbsType len = (pe - ps).LengthXY();                                                                       /** @brief	线段在XOY平面上的投影	*/
		RtLbsType srLen = len - len * ratio;                                                                        /** @brief	起点到反射点的二维距离	*/
		Point2D rp2d = ps2d + dir * srLen;                                                                          /** @brief	XOY平面上的反射点坐标	*/
		Point3D reflPoint(rp2d.x, rp2d.y, 0);                                                                       /** @brief	三维空间中的反射点坐标	*/
		//构造射线路径
		PathNode3D* sNode = new PathNode3D(ps, NODE_ROOT);                                                          /** @brief	构造路径起始节点	*/
		PathNode3D* eNode = new PathNode3D(pe, NODE_STOP);                                                          /** @brief	构造路径终止节点	*/
        PathNode3D* rNode = new PathNode3D(reflPoint, NODE_REFL);                                                   /** @brief	构造路径反射节点	*/
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
    bool isValidPath = true;                                   /** @brief	是否是有效路径	*/  
    if (path->m_type == RAYPATH_COMMON) {
		//检查每个node中点是否超过对应建筑物的高度
		for (int i = 1; i < path->m_nodes.size() - 1; ++i) {        //需要跳过首尾节点
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

			RtLbsType objHMin = m_objects[nodeObjectId]->m_foundationHeight;                /** @brief	物体最小高度	*/
			RtLbsType objHMax = m_objects[nodeObjectId]->GetObjectHeight();                 /** @brief	物体最大高度 	*/
			if (path->m_nodes[i]->m_point.z > objHMax) {
				isValidPath = false;
			}
		}

		if (isValidPath == false) {                                                    //若路径已无效，则无需进行下一步判定
			return false;
		}

		//将射线路径进行分段，判定是否被地形遮挡
		if (m_loadingTerrainFlag) {
			for (int i = 0; i < path->m_nodes.size() - 1; ++i) {
				const Point3D ps = path->m_nodes[i]->m_point;                   /** @brief	路径分支起始点	*/
				const Point3D pe = path->m_nodes[i + 1]->m_point;               /** @brief	路径分支终止点	*/
				if (m_terrain.IsBlock(ps, pe)) {                                //若该条路径分支被环境遮挡，路径无效
					isValidPath = false;
				}
			}
		}
    }
    else if (path->m_type == RAYPATH_TERRAIN_REFLECTION) {
		//将路径进行分段，判定是否被环境物体遮挡
		for (int i = 0; i < path->m_nodes.size() - 1; ++i) {
			const Point3D ps = path->m_nodes[i]->m_point;                   /** @brief	路径分支起始点	*/
			const Point3D pe = path->m_nodes[i + 1]->m_point;               /** @brief	路径分支终止点	*/
			if (IsBlockOnlyObject(ps, pe)) {                                //若该条路径分支被环境遮挡，路径无效
                isValidPath = false;
			}
		}
    }
   
    return isValidPath;

}

bool Scene::IsValidRayPath(const TerrainDiffractionPath* path) const
{
    //将路径进行分段，判定是否被环境物体遮挡
    for (int i = 0; i < path->m_nodes.size() - 1; ++i) {
        const Point3D ps = path->m_nodes[i]->m_point;                   /** @brief	路径分支起始点	*/
        const Point3D pe = path->m_nodes[i + 1]->m_point;               /** @brief	路径分支终止点	*/
        if (IsBlockOnlyObject(ps, pe)) {                                //若该条路径分支被环境遮挡，路径无效
            return false;
        }
    }
    return true;
}

bool Scene::_bfIntersect(const Ray2D& r, Intersection2D* intersect) const
{
    
    if (intersect)//重新初始化intersect防止出错
        intersect->m_ft = FLT_MAX;
    int n = static_cast<int>(m_segmentBuf.size());
    Intersection2D curIntersect;//当前距离交点
    bool hasIntersect = false;
    for (Segment2D* segment : m_segmentBuf) {
        bool flag = segment->GetIntersect(r, &curIntersect);
        if (flag && intersect == nullptr)//在不求解交点的情况下，flag为真代表与环境相交
            return true;
        if (flag && curIntersect.m_ft < intersect->m_ft) {//更新交点信息状态，始终保持距离最近
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




