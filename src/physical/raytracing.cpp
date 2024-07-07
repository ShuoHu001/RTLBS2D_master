#include "raytracing.h"

void RayTracing_CPUSingleThread(SYSTEM_MODE systemMode, const std::vector<std::vector<Ray2D>>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, std::vector<RayTreeNode*>& vroots)
{
	struct StackItem {
		RayTreeNode* node;
	};
	if (systemMode == MODE_RT) {		//��������׷�����ṹ
		for (size_t i = 0; i < scene->m_transmitters.size(); ++i) {
			const Transmitter* transmitter = scene->m_transmitters[i];										/** @brief	��ǰ���������ĵķ����	*/
			if (!transmitter->m_isValid) {																	//������Ч�ķ����
				continue;
			}
			std::stack<StackItem> stack; //��ʼ��ջ
			Point2D txPosition = transmitter->GetPosition2D();												/** @brief	���������	*/
			RayTreeNode* tempNode = new RayTreeNode();														/** @brief	��ʱ�ڵ����	*/
			tempNode->m_isValid = false;
			RayTreeNode* visualTempNode = tempNode;															/** @brief	��ʱ�ڵ�ĸ���	*/
			for (const auto& ray:rays[i]) {																	
				RayTreeNode* vrootTreeNode = new RayTreeNode();												/** @brief	�����ڵ�	*/
				vrootTreeNode->m_data = new PathNode(limitInfo, NODE_ROOT, txPosition);
				vrootTreeNode->m_data->m_nextRay = ray;
				tempNode->m_pRight = vrootTreeNode;															/** @brief	�������ڵ������������ڵ����ֵܽڵ�	*/
				tempNode = tempNode->m_pRight;																	//������������ֵ��ǰ���ڵ����һ���ֵܽڵ�
				stack.push({ tempNode });																		//������ѹ��ջ
			}
			//ִ������׷��
			while (!stack.empty()) {																		//��ջ������������ڵ���е�������������׷��
				StackItem& curItem = stack.top();
				stack.pop();
				PathTrace(raySplitFlag, raySplitRadius, scene, curItem.node);
			}
			//���������ۺ�
			RayTreeNode* curVRootNode = vroots[i];
			RayTreeNode* curNode = visualTempNode;
			while (curNode != nullptr) {
				if (!curNode->m_isValid) {
					curNode = curNode->m_pRight;															//������Ч�ڵ�
					continue;
				}
				curVRootNode->m_pRight = curNode;															//����Ч�ڵ��������ǰ������ڵ����ֽڵ㴦
				curVRootNode = curVRootNode->m_pRight;														//������һ�����ص����
				curNode = curNode->m_pRight;																//������һ������λ��
			}
			delete visualTempNode;
		}
	}
	else if (systemMode == MODE_LBS) {		//��λģʽ�й������������׷�����ṹ������Ȩ�ؼ���
		for (size_t i = 0; i < scene->m_sensors.size(); ++i) {
			const Sensor* sensor = scene->m_sensors[i];														/** @brief	��ǰ���������ĵĴ�����	*/
			if (!sensor->m_isValid) {																		//������Ч�Ĵ�����
				continue;
			}
			std::stack<StackItem> stack; //��ʼ��ջ
			Point2D txPosition = sensor->GetPosition2D();													/** @brief	����������	*/
			RayTreeNode* tempNode = new RayTreeNode();														/** @brief	��ʱ�ڵ����	*/
			tempNode->m_isValid = false;
			RayTreeNode* visualTempNode = tempNode;															/** @brief	��ʱ�ڵ�ĸ���	*/
			for (auto it = rays[i].begin(); it != rays[i].end(); ++it) {
				const Ray2D& ray = *it;																		/** @brief	��ǰ������������	*/
				PathNode* vrootPathNode = new PathNode(limitInfo, NODE_ROOT, txPosition);									/** @brief	��·���ڵ�	*/
				vrootPathNode->m_nextRay = ray;
				RayTreeNode* vrootTreeNode = new RayTreeNode(vrootPathNode);								/** @brief	�����ڵ�	*/
				tempNode->m_pRight = vrootTreeNode;															/** @brief	�������ڵ������������ڵ����ֵܽڵ�	*/
				tempNode = tempNode->m_pRight;																//������������ֵ��ǰ���ڵ����һ���ֵܽڵ�
				stack.push({ tempNode });																	//������ѹ��ջ
			}
			//ִ������׷��
			while (!stack.empty()) {																		//��ջ������������ڵ���е�������������׷��
				StackItem& curItem = stack.top();
				stack.pop();
				PathTrace(raySplitFlag, raySplitRadius, scene, curItem.node);
			}
			//���������ۺ�
			RayTreeNode* curVRootNode = vroots[i];
			RayTreeNode* curNode = visualTempNode;
			while (curNode != nullptr) {
				if (!curNode->m_isValid) {
					curNode = curNode->m_pRight;															//������Ч�ڵ�
					continue;
				}
				curVRootNode->m_pRight = curNode;															//����Ч�ڵ��������ǰ������ڵ����ֽڵ㴦
				curVRootNode = curVRootNode->m_pRight;														//������һ�����ص����
				curNode = curNode->m_pRight;																//������һ������λ��
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
		const Sensor* curSensor = scene->m_sensors[i];													/** @brief	��ǰ���������ĵķ����	*/
		if (!curSensor->m_isValid) {																	//������Ч�Ľ��ջ�
			continue;
		}
		std::stack<StackItem> stack; //��ʼ��ջ
		Point2D txPosition = curSensor->GetPosition2D();												/** @brief	����������	*/
		RayTreeNode* tempNode = new RayTreeNode();														/** @brief	��ʱ�ڵ����	*/
		tempNode->m_isValid = false;
		RayTreeNode* visualTempNode = tempNode;															/** @brief	��ʱ�ڵ�ĸ���	*/
		for (auto it = rays[i].begin(); it != rays[i].end(); ++it) {
			const Ray2D& ray = *it;																		/** @brief	��ǰ������������	*/
			PathNode* vrootPathNode = new PathNode(limitInfo, NODE_ROOT, txPosition);									/** @brief	��·���ڵ�	*/
			vrootPathNode->m_nextRay = ray;
			RayTreeNode* vrootTreeNode = new RayTreeNode(vrootPathNode);								/** @brief	�����ڵ�	*/
			tempNode->m_pRight = vrootTreeNode;															/** @brief	�������ڵ������������ڵ����ֵܽڵ�	*/
			tempNode = tempNode->m_pRight;																	//������������ֵ��ǰ���ڵ����һ���ֵܽڵ�
			stack.push({ tempNode });																		//������ѹ��ջ
		}
		//ִ������׷��
		while (!stack.empty()) {																		//��ջ������������ڵ���е�������������׷��
			StackItem& curItem = stack.top();
			stack.pop();
			PathTraceLBS(raySplitFlag, raySplitRadius, scene, curItem.node);
		}
		//���������ۺ�
		RayTreeNode* curVRootNode = vroots[i];
		RayTreeNode* curNode = visualTempNode;
		while (curNode != nullptr) {
			if (!curNode->m_isValid) {
				curNode = curNode->m_pRight;															//������Ч�ڵ�
				continue;
			}
			curVRootNode->m_pRight = curNode;															//����Ч�ڵ��������ǰ������ڵ����ֽڵ㴦
			curVRootNode = curVRootNode->m_pRight;														//������һ�����ص����
			curNode = curNode->m_pRight;																//������һ������λ��
		}
		delete visualTempNode;
	}
}

void RayTracing_CPUMultiThread(SYSTEM_MODE systemMode, const std::vector<std::vector<Ray2D>>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, uint16_t threadNum, std::vector<RayTreeNode*>& vroots)
{
	ThreadPool pool(threadNum);									//�����̳߳�

	if (systemMode == MODE_RT) {		//����׷��ģʽ�������ṹ
		for (int i = 0; i < scene->m_transmitters.size(); ++i) {
			if (!scene->m_transmitters[i]->m_isValid) {																	//������Ч���ջ�
				continue;
			}
			Point2D txPosition = scene->m_transmitters[i]->GetPosition2D();												/** @brief	�����λ��	*/
			std::vector<RayTreeNode*> vrootTemp(rays[i].size());														/** @brief	��ʱ�洢	*/
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
				std::this_thread::sleep_for(std::chrono::milliseconds(100));		//ÿ100ms���һ��
			}

			//��vroots�����нڵ����������ڵ�
			RayTreeNode* tempNode = vroots[i];																		/** @brief	��ǰ���õĸ��ڵ�	*/
			for (auto it = vrootTemp.begin(); it != vrootTemp.end(); ++it) {										//�����еĽڵ���������ֵܽڵ���
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
	else if (systemMode == MODE_LBS) {		//��λģʽ�й������������׷�����ṹ������Ȩ�ؼ���
		for (int i = 0; i < scene->m_sensors.size(); ++i) {
			if (!scene->m_sensors[i]->m_isValid) {																		//������Ч������
				continue;
			}
			Point2D txPosition = scene->m_sensors[i]->GetPosition2D();												/** @brief	�����λ��	*/
			std::vector<RayTreeNode*> vrootTemp(rays[i].size());														/** @brief	��ʱ�洢	*/
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
				std::this_thread::sleep_for(std::chrono::milliseconds(100));		//ÿ100ms���һ��
			}

			//��vroots�����нڵ����������ڵ�
			RayTreeNode* tempNode = vroots[i];																		/** @brief	��ǰ���õĸ��ڵ�	*/
			for (auto it = vrootTemp.begin(); it != vrootTemp.end(); ++it) {										//�����еĽڵ���������ֵܽڵ���
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
	ThreadPool pool(threadNum);									//�����̳߳�
	for (int i = 0; i < scene->m_sensors.size(); ++i) {
		if (!scene->m_sensors[i]->m_isValid) {																	//������Ч���ջ�
			continue;
		}
		const Point2D& sensorPosition = scene->m_sensors[i]->GetPosition2D();										/** @brief	�����λ��	*/
		std::vector<RayTreeNode*> vrootTemp(rays[i].size());														/** @brief	��ʱ�洢	*/
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
			std::this_thread::sleep_for(std::chrono::milliseconds(100));		//ÿ100ms���һ��
		}

		//��vroots�����нڵ����������ڵ�
		RayTreeNode* tempNode = vroots[i];																		/** @brief	��ǰ���õĸ��ڵ�	*/
		for (auto it = vrootTemp.begin(); it != vrootTemp.end(); ++it) {										//�����еĽڵ���������ֵܽڵ���
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
	//------------------------------------------------------------------�������ٳ��ĵ����봦��--------------------------------------------------------------------------//
	if (scene->m_pAccelerator->GetAccelType() != ACCEL_SDF) {
		LOG_ERROR << "RayTracing: scene accelerator must be signed distance field." << ENDL;
		return;
	}
	SignedDistanceField* sdf = dynamic_cast<SignedDistanceField*>(scene->m_pAccelerator);							//��ȡ�����еļ��ٽṹ��ת��Ϊ���볡��
	if (sdf == nullptr) {
		LOG_ERROR << "RayTracing: GPU SDF Convert Failed." << ENDL;
		return;
	}
	//----------------------------------------------------------------------GPU��Ԫ�ĵ����봦��---------------------------------------------------------------------------//
	size_t segmentNum = scene->m_gpuSegmentBuf.size();
	const std::vector<Segment2DGPU>& host_segments = scene->m_gpuSegmentBuf;										/** @brief	������GPU��Ԫ����	*/
	thrust::device_vector<Segment2DGPU> device_segments(segmentNum);
	device_segments = host_segments;
	//---------------------------------------------------------------------GPU������ĵ����봦��--------------------------------------------------------------------------//
	size_t wedgeNum = scene->m_gpuWedgeBuf.size();																	/** @brief	GPU��������	*/
	const std::vector<Wedge2DGPU>& host_wedges = scene->m_gpuWedgeBuf;												/** @brief	������GPU��������	*/
	thrust::device_vector<Wedge2DGPU> device_wedges(wedgeNum);														/** @brief	�豸��GPU��������	*/
	device_wedges = host_wedges;

	

	SignedDistanceFieldGPU sdfGPU = sdf->Convert2GPU();
	sdfGPU.AllocateOnDevice();

	//��������㼯��
	int txNum = static_cast<int>(scene->m_transmitters.size());
	std::vector<Point2D> txPositions(txNum);
	for (int i = 0; i < txNum; ++i) {
		txPositions[i] = scene->m_transmitters[i]->GetPosition2D();
	}
	//�������յ㼯��
	size_t rxNum = static_cast<int>(scene->m_receivers.size());
	std::vector<Point2D> rxPositions(rxNum);
	for (size_t i = 0; i < rxNum; ++i) {
		rxPositions[i] = scene->m_receivers[i]->GetPosition2D();
	}

	thrust::device_vector<Point2D> device_RxPositions(rxNum);
	device_RxPositions = rxPositions;

	//ִ��GPU����׷��
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
	//------------------------------------------------------------------�������ٳ��ĵ����봦��--------------------------------------------------------------------------//
	if (scene->m_pAccelerator->GetAccelType() != ACCEL_SDF) {
		LOG_ERROR << "RayTracing: scene accelerator must be signed distance field." << ENDL;
		return;
	}
	SignedDistanceField* sdf = dynamic_cast<SignedDistanceField*>(scene->m_pAccelerator);							//��ȡ�����еļ��ٽṹ��ת��Ϊ���볡��
	if (sdf == nullptr) {
		LOG_ERROR << "RayTracing: GPU SDF Convert Failed." << ENDL;
		return;
	}
	//----------------------------------------------------------------------GPU��Ԫ�ĵ����봦��---------------------------------------------------------------------------//
	size_t segmentNum = scene->m_gpuSegmentBuf.size();
	const std::vector<Segment2DGPU>& host_segments = scene->m_gpuSegmentBuf;										/** @brief	������GPU��Ԫ����	*/
	thrust::device_vector<Segment2DGPU> device_segments(segmentNum);
	device_segments = host_segments;
	//---------------------------------------------------------------------GPU������ĵ����봦��--------------------------------------------------------------------------//
	size_t wedgeNum = scene->m_gpuWedgeBuf.size();																	/** @brief	GPU��������	*/
	const std::vector<Wedge2DGPU>& host_wedges = scene->m_gpuWedgeBuf;												/** @brief	������GPU��������	*/
	thrust::device_vector<Wedge2DGPU> device_wedges(wedgeNum);														/** @brief	�豸��GPU��������	*/
	device_wedges = host_wedges;



	SignedDistanceFieldGPU sdfGPU = sdf->Convert2GPU();
	sdfGPU.AllocateOnDevice();

	//�����������ڵ㼯��
	//��������㼯��
	int sensorNum = static_cast<int>(scene->m_sensors.size());
	std::vector<Point2D> sensorPositions(sensorNum);
	for (int i = 0; i < sensorNum; ++i) {
		sensorPositions[i] = scene->m_sensors[i]->GetPosition2D();
	}

	thrust::device_vector<Point2D> device_SensorPositions(sensorNum);
	device_SensorPositions = sensorPositions;

	//ִ��GPU����׷��
	gpuTreeNodes.resize(sensorNum);
	for (size_t i = 0; i < sensorNum; ++i) {
		PathTraceGPUOnlyTree(rays[i], limitInfo, raySplitFlag, raySplitRadius, thrust::raw_pointer_cast(device_segments.data()), segmentNum,
			thrust::raw_pointer_cast(device_wedges.data()), wedgeNum, sdfGPU.GetDevicePointer(), gpuTreeNodes[i]);
	}
	device_segments.swap(device_segments);
	device_wedges.swap(device_wedges);
	sdfGPU.ReleaseOnDevice();
}


