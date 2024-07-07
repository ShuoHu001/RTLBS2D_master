#include "raytracing.h"

void RayTracing_CPUSingleThread(SYSTEM_MODE systemMode, const std::vector<std::vector<Ray2D>>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, std::vector<RayTreeNode*>& vroots)
{
	struct StackItem {
		RayTreeNode* node;
	};
	if (systemMode == MODE_RT) {		//构建射线追踪树结构
		for (size_t i = 0; i < scene->m_transmitters.size(); ++i) {
			const Transmitter* transmitter = scene->m_transmitters[i];										/** @brief	当前所遍历到的的发射机	*/
			if (!transmitter->m_isValid) {																	//跳过无效的发射机
				continue;
			}
			std::stack<StackItem> stack; //初始化栈
			Point2D txPosition = transmitter->GetPosition2D();												/** @brief	发射机坐标	*/
			RayTreeNode* tempNode = new RayTreeNode();														/** @brief	临时节点挂载	*/
			tempNode->m_isValid = false;
			RayTreeNode* visualTempNode = tempNode;															/** @brief	临时节点的复制	*/
			for (const auto& ray:rays[i]) {																	
				RayTreeNode* vrootTreeNode = new RayTreeNode();												/** @brief	根树节点	*/
				vrootTreeNode->m_data = new PathNode(limitInfo, NODE_ROOT, txPosition);
				vrootTreeNode->m_data->m_nextRay = ray;
				tempNode->m_pRight = vrootTreeNode;															/** @brief	将根树节点挂载至虚拟根节点右兄弟节点	*/
				tempNode = tempNode->m_pRight;																	//继续迭代并赋值当前根节点的下一右兄弟节点
				stack.push({ tempNode });																		//将数据压入栈
			}
			//执行射线追踪
			while (!stack.empty()) {																		//将栈中所有虚拟根节点进行弹出并进行射线追踪
				StackItem& curItem = stack.top();
				stack.pop();
				PathTrace(raySplitFlag, raySplitRadius, scene, curItem.node);
			}
			//进行数据综合
			RayTreeNode* curVRootNode = vroots[i];
			RayTreeNode* curNode = visualTempNode;
			while (curNode != nullptr) {
				if (!curNode->m_isValid) {
					curNode = curNode->m_pRight;															//跳过无效节点
					continue;
				}
				curVRootNode->m_pRight = curNode;															//将有效节点挂载至当前虚拟根节点右兄节点处
				curVRootNode = curVRootNode->m_pRight;														//进行下一个挂载点迭代
				curNode = curNode->m_pRight;																//迭代下一个挂载位置
			}
			delete visualTempNode;
		}
	}
	else if (systemMode == MODE_LBS) {		//定位模式中构建伴随的射线追踪树结构，用于权重计算
		for (size_t i = 0; i < scene->m_sensors.size(); ++i) {
			const Sensor* sensor = scene->m_sensors[i];														/** @brief	当前所遍历到的的传感器	*/
			if (!sensor->m_isValid) {																		//跳过无效的传感器
				continue;
			}
			std::stack<StackItem> stack; //初始化栈
			Point2D txPosition = sensor->GetPosition2D();													/** @brief	传感器坐标	*/
			RayTreeNode* tempNode = new RayTreeNode();														/** @brief	临时节点挂载	*/
			tempNode->m_isValid = false;
			RayTreeNode* visualTempNode = tempNode;															/** @brief	临时节点的复制	*/
			for (auto it = rays[i].begin(); it != rays[i].end(); ++it) {
				const Ray2D& ray = *it;																		/** @brief	当前遍历到的射线	*/
				PathNode* vrootPathNode = new PathNode(limitInfo, NODE_ROOT, txPosition);									/** @brief	根路径节点	*/
				vrootPathNode->m_nextRay = ray;
				RayTreeNode* vrootTreeNode = new RayTreeNode(vrootPathNode);								/** @brief	根树节点	*/
				tempNode->m_pRight = vrootTreeNode;															/** @brief	将根树节点挂载至虚拟根节点右兄弟节点	*/
				tempNode = tempNode->m_pRight;																//继续迭代并赋值当前根节点的下一右兄弟节点
				stack.push({ tempNode });																	//将数据压入栈
			}
			//执行射线追踪
			while (!stack.empty()) {																		//将栈中所有虚拟根节点进行弹出并进行射线追踪
				StackItem& curItem = stack.top();
				stack.pop();
				PathTrace(raySplitFlag, raySplitRadius, scene, curItem.node);
			}
			//进行数据综合
			RayTreeNode* curVRootNode = vroots[i];
			RayTreeNode* curNode = visualTempNode;
			while (curNode != nullptr) {
				if (!curNode->m_isValid) {
					curNode = curNode->m_pRight;															//跳过无效节点
					continue;
				}
				curVRootNode->m_pRight = curNode;															//将有效节点挂载至当前虚拟根节点右兄节点处
				curVRootNode = curVRootNode->m_pRight;														//进行下一个挂载点迭代
				curNode = curNode->m_pRight;																//迭代下一个挂载位置
			}
			delete visualTempNode;
		}
	}
	
}

void RayTracingLBS_CPUSingleThread(SYSTEM_MODE systemMode, const std::vector<std::vector<Ray2D>>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, std::vector<RayTreeNode*>& vroots)
{
	struct StackItem {
		RayTreeNode* node;
	};
	for (size_t i = 0; i < scene->m_sensors.size(); ++i) {
		const Sensor* curSensor = scene->m_sensors[i];													/** @brief	当前所遍历到的的发射机	*/
		if (!curSensor->m_isValid) {																	//跳过无效的接收机
			continue;
		}
		std::stack<StackItem> stack; //初始化栈
		Point2D txPosition = curSensor->GetPosition2D();												/** @brief	传感器坐标	*/
		RayTreeNode* tempNode = new RayTreeNode();														/** @brief	临时节点挂载	*/
		tempNode->m_isValid = false;
		RayTreeNode* visualTempNode = tempNode;															/** @brief	临时节点的复制	*/
		for (auto it = rays[i].begin(); it != rays[i].end(); ++it) {
			const Ray2D& ray = *it;																		/** @brief	当前遍历到的射线	*/
			PathNode* vrootPathNode = new PathNode(limitInfo, NODE_ROOT, txPosition);									/** @brief	根路径节点	*/
			vrootPathNode->m_nextRay = ray;
			RayTreeNode* vrootTreeNode = new RayTreeNode(vrootPathNode);								/** @brief	根树节点	*/
			tempNode->m_pRight = vrootTreeNode;															/** @brief	将根树节点挂载至虚拟根节点右兄弟节点	*/
			tempNode = tempNode->m_pRight;																	//继续迭代并赋值当前根节点的下一右兄弟节点
			stack.push({ tempNode });																		//将数据压入栈
		}
		//执行射线追踪
		while (!stack.empty()) {																		//将栈中所有虚拟根节点进行弹出并进行射线追踪
			StackItem& curItem = stack.top();
			stack.pop();
			PathTraceLBS(raySplitFlag, raySplitRadius, scene, curItem.node);
		}
		//进行数据综合
		RayTreeNode* curVRootNode = vroots[i];
		RayTreeNode* curNode = visualTempNode;
		while (curNode != nullptr) {
			if (!curNode->m_isValid) {
				curNode = curNode->m_pRight;															//跳过无效节点
				continue;
			}
			curVRootNode->m_pRight = curNode;															//将有效节点挂载至当前虚拟根节点右兄节点处
			curVRootNode = curVRootNode->m_pRight;														//进行下一个挂载点迭代
			curNode = curNode->m_pRight;																//迭代下一个挂载位置
		}
		delete visualTempNode;
	}
}

void RayTracing_CPUMultiThread(SYSTEM_MODE systemMode, const std::vector<std::vector<Ray2D>>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, uint16_t threadNum, std::vector<RayTreeNode*>& vroots)
{
	ThreadPool pool(threadNum);									//创建线程池

	if (systemMode == MODE_RT) {		//射线追踪模式构建树结构
		for (int i = 0; i < scene->m_transmitters.size(); ++i) {
			if (!scene->m_transmitters[i]->m_isValid) {																	//跳过无效接收机
				continue;
			}
			Point2D txPosition = scene->m_transmitters[i]->GetPosition2D();												/** @brief	发射机位置	*/
			std::vector<RayTreeNode*> vrootTemp(rays[i].size());														/** @brief	临时存储	*/
			for (size_t j = 0; j < rays[i].size(); ++j) {
				PathNode* vrootPathNode = new PathNode(limitInfo, NODE_ROOT, txPosition);
				vrootPathNode->m_nextRay = rays[i][j];
				RayTreeNode* vrootTreeNode = new RayTreeNode(vrootPathNode);
				auto future = pool.enqueue(PathTrace, raySplitFlag, raySplitRadius, scene, vrootTreeNode);

				vrootTemp[j] = vrootTreeNode;
			}

			while (true) {
				if (pool.getTaskCount() == 0) {
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(100));		//每100ms检查一次
			}

			//将vroots中所有节点纳入至根节点
			RayTreeNode* tempNode = vroots[i];																		/** @brief	当前引用的根节点	*/
			for (auto it = vrootTemp.begin(); it != vrootTemp.end(); ++it) {										//将所有的节点挂载至右兄弟节点上
				RayTreeNode* curNode = *it;
				while (curNode != nullptr) {
					if (!curNode->m_isValid) {
						curNode = curNode->m_pRight;
						continue;
					}
					tempNode->m_pRight = curNode;
					tempNode = tempNode->m_pRight;
					curNode = curNode->m_pRight;
				}
			}
		}
	}
	else if (systemMode == MODE_LBS) {		//定位模式中构建伴随的射线追踪树结构，用于权重计算
		for (int i = 0; i < scene->m_sensors.size(); ++i) {
			if (!scene->m_sensors[i]->m_isValid) {																		//跳过无效传感器
				continue;
			}
			Point2D txPosition = scene->m_sensors[i]->GetPosition2D();												/** @brief	发射机位置	*/
			std::vector<RayTreeNode*> vrootTemp(rays[i].size());														/** @brief	临时存储	*/
			for (size_t j = 0; j < rays[i].size(); ++j) {
				PathNode* vrootPathNode = new PathNode(limitInfo, NODE_ROOT, txPosition);
				vrootPathNode->m_nextRay = rays[i][j];
				RayTreeNode* vrootTreeNode = new RayTreeNode(vrootPathNode);
				auto future = pool.enqueue(PathTrace, raySplitFlag, raySplitRadius, scene, vrootTreeNode);

				vrootTemp[j] = vrootTreeNode;
			}

			while (true) {
				if (pool.getTaskCount() == 0) {
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(100));		//每100ms检查一次
			}

			//将vroots中所有节点纳入至根节点
			RayTreeNode* tempNode = vroots[i];																		/** @brief	当前引用的根节点	*/
			for (auto it = vrootTemp.begin(); it != vrootTemp.end(); ++it) {										//将所有的节点挂载至右兄弟节点上
				RayTreeNode* curNode = *it;
				while (curNode != nullptr) {
					if (!curNode->m_isValid) {
						curNode = curNode->m_pRight;
						continue;
					}
					tempNode->m_pRight = curNode;
					tempNode = tempNode->m_pRight;
					curNode = curNode->m_pRight;
				}
			}
		}
	}
}

void RayTracingLBS_CPUMultiThread(SYSTEM_MODE systemMode, const std::vector<std::vector<Ray2D>>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, uint16_t threadNum, std::vector<RayTreeNode*>& vroots)
{
	ThreadPool pool(threadNum);									//创建线程池
	for (int i = 0; i < scene->m_sensors.size(); ++i) {
		if (!scene->m_sensors[i]->m_isValid) {																	//跳过无效接收机
			continue;
		}
		const Point2D& sensorPosition = scene->m_sensors[i]->GetPosition2D();										/** @brief	发射机位置	*/
		std::vector<RayTreeNode*> vrootTemp(rays[i].size());														/** @brief	临时存储	*/
		for (size_t j = 0; j < rays[i].size(); ++j) {
			PathNode* vrootPathNode = new PathNode(limitInfo, NODE_ROOT, sensorPosition);
			vrootPathNode->m_nextRay = rays[i][j];
			RayTreeNode* vrootTreeNode = new RayTreeNode(vrootPathNode);
			auto future = pool.enqueue(PathTraceLBS, raySplitFlag, raySplitRadius, scene, vrootTreeNode);
			vrootTemp[j] = vrootTreeNode;
		}

		while (true) {
			if (pool.getTaskCount() == 0) {
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));		//每100ms检查一次
		}

		//将vroots中所有节点纳入至根节点
		RayTreeNode* tempNode = vroots[i];																		/** @brief	当前引用的根节点	*/
		for (auto it = vrootTemp.begin(); it != vrootTemp.end(); ++it) {										//将所有的节点挂载至右兄弟节点上
			RayTreeNode* curNode = *it;
			while (curNode != nullptr) {
				if (!curNode->m_isValid) {
					curNode = curNode->m_pRight;
					continue;
				}
				tempNode->m_pRight = curNode;
				tempNode = tempNode->m_pRight;
				curNode = curNode->m_pRight;
			}
		}
	}
}

void RayTracing_GPUMultiThread(const std::vector<std::vector<Ray2D>>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, std::vector<std::vector<PathNodeGPU*>>& gpuPathNodes)
{
	//------------------------------------------------------------------环境加速场的导入与处理--------------------------------------------------------------------------//
	if (scene->m_pAccelerator->GetAccelType() != ACCEL_SDF) {
		LOG_ERROR << "RayTracing: scene accelerator must be signed distance field." << ENDL;
		return;
	}
	SignedDistanceField* sdf = dynamic_cast<SignedDistanceField*>(scene->m_pAccelerator);							//获取场景中的加速结构（转换为距离场）
	if (sdf == nullptr) {
		LOG_ERROR << "RayTracing: GPU SDF Convert Failed." << ENDL;
		return;
	}
	//----------------------------------------------------------------------GPU面元的导入与处理---------------------------------------------------------------------------//
	size_t segmentNum = scene->m_gpuSegmentBuf.size();
	const std::vector<Segment2DGPU>& host_segments = scene->m_gpuSegmentBuf;										/** @brief	主机上GPU面元数组	*/
	thrust::device_vector<Segment2DGPU> device_segments(segmentNum);
	device_segments = host_segments;
	//---------------------------------------------------------------------GPU绕射棱的导入与处理--------------------------------------------------------------------------//
	size_t wedgeNum = scene->m_gpuWedgeBuf.size();																	/** @brief	GPU棱劈数量	*/
	const std::vector<Wedge2DGPU>& host_wedges = scene->m_gpuWedgeBuf;												/** @brief	主机上GPU棱劈数组	*/
	thrust::device_vector<Wedge2DGPU> device_wedges(wedgeNum);														/** @brief	设备上GPU棱劈数组	*/
	device_wedges = host_wedges;

	

	SignedDistanceFieldGPU sdfGPU = sdf->Convert2GPU();
	sdfGPU.AllocateOnDevice();

	//构建发射点集合
	int txNum = static_cast<int>(scene->m_transmitters.size());
	std::vector<Point2D> txPositions(txNum);
	for (int i = 0; i < txNum; ++i) {
		txPositions[i] = scene->m_transmitters[i]->GetPosition2D();
	}
	//构建接收点集合
	size_t rxNum = static_cast<int>(scene->m_receivers.size());
	std::vector<Point2D> rxPositions(rxNum);
	for (size_t i = 0; i < rxNum; ++i) {
		rxPositions[i] = scene->m_receivers[i]->GetPosition2D();
	}

	thrust::device_vector<Point2D> device_RxPositions(rxNum);
	device_RxPositions = rxPositions;

	//执行GPU射线追踪
	gpuPathNodes.resize(txNum);
	for (size_t i = 0; i < txNum; ++i) {
		PathTraceGPU(rays[i], limitInfo, raySplitFlag, raySplitRadius, thrust::raw_pointer_cast(device_RxPositions.data()), rxNum, thrust::raw_pointer_cast(device_segments.data()), segmentNum,
			thrust::raw_pointer_cast(device_wedges.data()), wedgeNum, sdfGPU.GetDevicePointer(), gpuPathNodes[i]);
	}
	device_segments.swap(device_segments);
	device_wedges.swap(device_wedges);
	device_RxPositions.swap(device_RxPositions);
	sdfGPU.ReleaseOnDevice();
}

void RayTracing_GPUMultiThreadWithNode(const std::vector<std::vector<Ray2D>>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, std::vector<std::vector<TreeNodeGPU*>>& gpuTreeNodes)
{
	//------------------------------------------------------------------环境加速场的导入与处理--------------------------------------------------------------------------//
	if (scene->m_pAccelerator->GetAccelType() != ACCEL_SDF) {
		LOG_ERROR << "RayTracing: scene accelerator must be signed distance field." << ENDL;
		return;
	}
	SignedDistanceField* sdf = dynamic_cast<SignedDistanceField*>(scene->m_pAccelerator);							//获取场景中的加速结构（转换为距离场）
	if (sdf == nullptr) {
		LOG_ERROR << "RayTracing: GPU SDF Convert Failed." << ENDL;
		return;
	}
	//----------------------------------------------------------------------GPU面元的导入与处理---------------------------------------------------------------------------//
	size_t segmentNum = scene->m_gpuSegmentBuf.size();
	const std::vector<Segment2DGPU>& host_segments = scene->m_gpuSegmentBuf;										/** @brief	主机上GPU面元数组	*/
	thrust::device_vector<Segment2DGPU> device_segments(segmentNum);
	device_segments = host_segments;
	//---------------------------------------------------------------------GPU绕射棱的导入与处理--------------------------------------------------------------------------//
	size_t wedgeNum = scene->m_gpuWedgeBuf.size();																	/** @brief	GPU棱劈数量	*/
	const std::vector<Wedge2DGPU>& host_wedges = scene->m_gpuWedgeBuf;												/** @brief	主机上GPU棱劈数组	*/
	thrust::device_vector<Wedge2DGPU> device_wedges(wedgeNum);														/** @brief	设备上GPU棱劈数组	*/
	device_wedges = host_wedges;



	SignedDistanceFieldGPU sdfGPU = sdf->Convert2GPU();
	sdfGPU.AllocateOnDevice();

	//构建传感器节点集合
	//构建发射点集合
	int sensorNum = static_cast<int>(scene->m_sensors.size());
	std::vector<Point2D> sensorPositions(sensorNum);
	for (int i = 0; i < sensorNum; ++i) {
		sensorPositions[i] = scene->m_sensors[i]->GetPosition2D();
	}

	thrust::device_vector<Point2D> device_SensorPositions(sensorNum);
	device_SensorPositions = sensorPositions;

	//执行GPU射线追踪
	gpuTreeNodes.resize(sensorNum);
	for (size_t i = 0; i < sensorNum; ++i) {
		PathTraceGPUOnlyTree(rays[i], limitInfo, raySplitFlag, raySplitRadius, thrust::raw_pointer_cast(device_segments.data()), segmentNum,
			thrust::raw_pointer_cast(device_wedges.data()), wedgeNum, sdfGPU.GetDevicePointer(), gpuTreeNodes[i]);
	}
	device_segments.swap(device_segments);
	device_wedges.swap(device_wedges);
	sdfGPU.ReleaseOnDevice();
}


