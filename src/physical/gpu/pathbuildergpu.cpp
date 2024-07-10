#include "pathbuildergpu.h"

__constant__ int D_CONST_MAXNODESIZE;												/** @brief	L1�豸�����ڴ�-������	*/
__constant__ RtLbsType D_CONST_RAYSPLITRADIUS1;									/** @brief	L1�豸�����ڴ�-���߷��Ѱ뾶	*/


//����GPU�����ν�������ṹ
struct IsValidNodeId {
	HOST_DEVICE_FUNC
		bool operator () (const int id) {
		return id == -1;
	}
};

__global__ void CheckTargetInsideKernel(CPUConverterPathNode* allNodes, int numNodes, Point2D* target, int* nodeIds)
{
	int nodeIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (nodeIdx < numNodes) {
		const CPUConverterPathNode& curNode = allNodes[nodeIdx];
		CPUConverterPathNode* fatherNode = nullptr;
		if (curNode.m_fatherNodeId != -1) {
			fatherNode = &allNodes[curNode.m_fatherNodeId];
		}
		if (curNode.IsCapturedByPoint(*target, D_CONST_RAYSPLITRADIUS1, fatherNode)) {			//�������գ������nodesId��
			nodeIds[nodeIdx] = nodeIdx;												//��ֵ���յ�ID
		}
	}
}

__global__ void FindPathNodeKernel(CPUConverterPathNode* allNodes, int numNodes, int* nodeIds, int numId, int* pathNodeIds, int numPathId)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx < numId) {
		int curNodeId = nodeIds[idx];															/** @brief	��ǰ�Ľڵ�ID	*/
		int pathNodeId = (idx + 1) * D_CONST_MAXNODESIZE - 1;										/** @brief	��ƴ�ĩβ��ʼ����д�룬����׷����Դ	*/
		pathNodeIds[pathNodeId] = curNodeId;
		int fatherNodeId = allNodes[curNodeId].m_fatherNodeId;
		for (int i = 0; i < D_CONST_MAXNODESIZE - 1; ++i) {
			if (fatherNodeId == -1) {															
				break;
			}
			pathNodeIds[--pathNodeId] = fatherNodeId;
			fatherNodeId = allNodes[fatherNodeId].m_fatherNodeId;								//������һ�ָ��ڵ�Ѱ��
		}
	}
}

void DirectlySetResultPath_GPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRaidus, std::vector<GSPairCluster>& clusters)
{
	//����vroots�е��������ݣ�����CPUConverterPathNode,��CPUConverterPathNode��Ӵ�����ID���ԣ����ڱ�ʶ��������һ������������GPU�����У�����ѭ����ѭ���Ĳ����ǽ��յ������

	//1-����GPU�ϵ� CPUConverterPathNode����
	int maxDepth = 0;																					/** @brief	������	*/
	int maxNodeSize = 0;																				/** @brief	���ڵ�����	*/
	std::vector<CPUConverterPathNode> host_AllPathNodes;
	GenerateAllTreeNodeAndConvertToCPUConvertPathNode(vroots, host_AllPathNodes, maxDepth);
	maxNodeSize = maxDepth + 1;																			/** @brief	���ڵ�����Ϊ������+1	*/

	//�ϲ�cluster�����е����������
	std::vector<Point2D> targetPoints;
	targetPoints.reserve(5 * clusters.size());
	for (auto& cluster : clusters) {
		for (auto& point : cluster.m_aroundPoints) {
			targetPoints.push_back(point);
		}
	}

	//�����Ӧ����������׷�ٽ������
	std::vector<std::vector<RaytracingResult>*> targetRtResults;
	for (auto& cluster : clusters) {
		for (auto& results : cluster.m_rtResult) {
			results.resize(scene->m_sensors.size());
			targetRtResults.push_back(&results);
		}
	}


	int num_Nodes = static_cast<int>(host_AllPathNodes.size());											/** @brief	�ڵ�����	*/
	int num_Targets = static_cast<int>(targetPoints.size());											/** @brief	Ŀ������	*/

	//2-�������ϴ���GPU�ڴ���
	thrust::device_vector< CPUConverterPathNode> dev_AllPathNodes(num_Nodes);							/** @brief	GPU �ϵ����нڵ�����	*/
	dev_AllPathNodes = host_AllPathNodes;																/** @brief	��CPU�ڴ�������GPU�ڴ���	*/

	//3-��λ�������ϴ���GPU�ڴ���
	thrust::device_vector<Point2D> dev_TargetPositions(num_Targets);									/** @brief	GPU �����е�Ŀ�������	*/
	dev_TargetPositions = targetPoints;																

	//�����ڴ�����
	thrust::device_vector<int> dev_Temp_TargetNodeIds;													/** @brief	ÿ��NODE��target���յ�����	*/
	dev_Temp_TargetNodeIds.resize(num_Nodes);															/** @brief	����Ϊ�ڵ�����	*/

	thrust::device_vector<int> dev_TargetNodeIds;														/** @brief	�洢����Ŀ�������յ���NodeId	*/
	std::vector<int> everyTargetNodesNum(num_Targets);													/** @brief	ÿ��Ŀ����յ��Ľڵ������	*/

	thrust::device_vector<int> dev_TargetPathNodeIds;													/** @brief	�洢����Ŀ����յ���NodeId��׷��·��	*/

	//GPU ������趨����
	int threadPerBlock = 256;																			/** @brief	�����߳̿鸺����߳�����	*/
	int numBlock1 = (num_Nodes + threadPerBlock - 1) / threadPerBlock;									/** @brief	�߳̿�����1	*/
	cudaError_t cudaerr;																				/** @brief	cuda������	*/

	//����������GPU�����ڴ���
	cudaMemcpyToSymbol(D_CONST_MAXNODESIZE, &maxNodeSize, sizeof(int));
	cudaMemcpyToSymbol(D_CONST_RAYSPLITRADIUS1, &splitRaidus, sizeof(RtLbsType));


	//4-���ú˺�������������������node��rx�����ж�
	for (int i = 0; i < static_cast<int>(dev_TargetPositions.size()); ++i) {
		//����ʱ�ڵ�ID��Ϊ��Ч��ID �趨Ϊ-1
		thrust::fill(dev_Temp_TargetNodeIds.begin(), dev_Temp_TargetNodeIds.end(), -1);

		//����Ŀ�겶���ж�
		CheckTargetInsideKernel CUDA_KERNEL(numBlock1, threadPerBlock)(thrust::raw_pointer_cast(dev_AllPathNodes.data()), num_Nodes, thrust::raw_pointer_cast(&dev_TargetPositions[i]), thrust::raw_pointer_cast(dev_Temp_TargetNodeIds.data()));
		cudaerr = cudaDeviceSynchronize();
		if (cudaerr != cudaSuccess) {
			LOG_ERROR << "PathBuilderGPU: CheckTargetInsideKernel " << cudaGetErrorString(cudaerr) << ENDL;
		}

		//���ù�Լ��ʽ��ɾ��temp�е���Ч����
		int validNodesNum = static_cast<int>(thrust::distance(dev_Temp_TargetNodeIds.begin(), thrust::remove_if(dev_Temp_TargetNodeIds.begin(), dev_Temp_TargetNodeIds.end(), IsValidNodeId())));		/** @brief	��Ч�Ľڵ�����	*/

		//��ID��������Ӧ��������
		everyTargetNodesNum[i] = validNodesNum;
		if (validNodesNum != 0) {
			dev_TargetNodeIds.resize(dev_TargetNodeIds.size() + validNodesNum);
			thrust::copy(dev_Temp_TargetNodeIds.begin(), dev_Temp_TargetNodeIds.begin() + validNodesNum, dev_TargetNodeIds.end() - validNodesNum);
		}
	}

	std::vector<int> host_TargetNodeIds(dev_TargetNodeIds.size());
	thrust::copy(dev_TargetNodeIds.begin(), dev_TargetNodeIds.end(), host_TargetNodeIds.begin());

	int validTargetNodesNum = static_cast<int>(dev_TargetNodeIds.size());								/** @brief	��Ч��Ŀ��ڵ�����	*/
	//׷����Դ
	dev_TargetPathNodeIds.resize(validTargetNodesNum * maxNodeSize);										//����ڵ�·�������ڵ�����ΪmaxDepth
	thrust::fill(dev_TargetPathNodeIds.begin(), dev_TargetPathNodeIds.end(), -1);							//Ĭ��ȫ��Ϊ-1

	int numBlock2 = (validTargetNodesNum + threadPerBlock - 1) / threadPerBlock;						/** @brief	�߳̿�����2	*/
	FindPathNodeKernel CUDA_KERNEL(numBlock2, threadPerBlock)(thrust::raw_pointer_cast(dev_AllPathNodes.data()), num_Nodes, thrust::raw_pointer_cast(dev_TargetNodeIds.data()), dev_TargetNodeIds.size(), thrust::raw_pointer_cast(dev_TargetPathNodeIds.data()), dev_TargetPathNodeIds.size());
	cudaerr = cudaDeviceSynchronize();
	if (cudaerr != cudaSuccess) {
		LOG_ERROR << "PathBuilderGPU: FindPathNodeKernel " << cudaGetErrorString(cudaerr) << ENDL;
	}

	

	//�����ݿ����������ڴ���
	std::vector<int> host_TargetPathNodeIds(dev_TargetPathNodeIds.size());
	thrust::copy(dev_TargetPathNodeIds.begin(), dev_TargetPathNodeIds.end(), host_TargetPathNodeIds.begin());

	//ɾ����GPU�е�����
	dev_AllPathNodes.swap(dev_AllPathNodes);
	dev_TargetPositions.swap(dev_TargetPositions);
	dev_Temp_TargetNodeIds.swap(dev_Temp_TargetNodeIds);
	dev_TargetNodeIds.swap(dev_TargetNodeIds);
	dev_TargetPathNodeIds.swap(dev_TargetPathNodeIds);

	//�����������˹�ϵ
	int nId = 0;																						/** @brief	��ʼ�Ľڵ�ID	*/
	int pId = 0;																						/** @brief	Ŀ������� ID	*/
	for (auto curTargetNodeNum : everyTargetNodesNum) {													//����ÿ��Ŀ��
		std::vector<std::vector<RayPath*>> commonPath2Ds;												//����ྶ
		commonPath2Ds.resize(scene->m_sensors.size(), std::vector<RayPath*>());							//���䳣��·���ڴ�
		for (int i = 0; i < curTargetNodeNum; ++i) {
			std::vector<CPUConverterPathNode*> pathNodes;												/** @brief	CPUNode���ɵ�path	*/
			pathNodes.reserve(maxNodeSize);
			for (int j = 0; j < maxNodeSize; ++j) {
				int curNodeId = host_TargetPathNodeIds[nId++];
				if (curNodeId == -1) {																	//������Ч�ڵ�
					continue;
				}
				pathNodes.push_back(&host_AllPathNodes[curNodeId]);
			}
			int sensorId = pathNodes[0]->m_sensorId;
			//��pathת��ΪRayPath
			RayPath* newRayPath = new RayPath();
			newRayPath->ConvertFrom(pathNodes, scene->m_segmentBuf, scene->m_wedgeBuf);
			//��֤·������Ч��
			if (newRayPath->IsValidAndRectify(targetPoints[pId], scene)) {
				commonPath2Ds[sensorId].push_back(newRayPath);
			}
			else {
				delete newRayPath;
			}
		}

		RtLbsType sensorHeight = scene->m_sensors[0]->m_position.z;
		Point3D targetPoint3D(targetPoints[pId].x, targetPoints[pId].y, sensorHeight);
		for (int i = 0; i < static_cast<int>(scene->m_sensors.size()); ++i) {
			const Sensor* curSensor = scene->m_sensors[i];
			//������·��ת��Ϊ��ά·��
			std::vector<RayPath3D*> commonPath3D(commonPath2Ds[i].size());
			for (int j = 0; j < commonPath2Ds[i].size(); ++j) {
				commonPath3D[j] = new RayPath3D(*commonPath2Ds[i][j], curSensor->m_position, targetPoint3D);
				commonPath3D[j]->ReverseRayPath();																//���ڴ�������Ŀ��Դ֮���·���Ŀ����ԣ���˶�·������������
			}

			//���淴�侶
			std::vector<RayPath3D*> terrainReflectPaths;														/** @brief	���淴��·��(��ת�͵�·������Ŀ�괫����������)	*/
			if (scene->GetGroundReflectPaths(targetPoint3D, curSensor->m_position, terrainReflectPaths)) {
				for (auto it = terrainReflectPaths.begin(); it != terrainReflectPaths.end(); ++it) {			//�������˵��淴��·�����򽫵��淴��·�����������·����
					commonPath3D.push_back(*it);
				}
			}

			//ɾ����Ч��·��
			std::vector<RayPath3D*> validPath;																	/** @brief	��Ч·��	*/
			for (auto it = commonPath3D.begin(); it != commonPath3D.end(); ++it) {
				RayPath3D* curPath = *it;
				if (!scene->IsValidRayPath(curPath)) {															//·����֤��ͨ������Ч·����ֱ��ɾ��
					delete curPath;
					curPath = nullptr;
				}
				else {
					validPath.push_back(curPath);
				}
			}
			(*targetRtResults[pId])[i].SetRayPath(validPath);


			//�����������·��
			TerrainDiffractionPath* terrainDifftactionPath = nullptr;
			if (scene->GetGroundDiffractionPath(targetPoint3D, curSensor->m_position, terrainDifftactionPath)) {
				if (scene->IsValidRayPath(terrainDifftactionPath))
					(*targetRtResults[pId])[i].SetRayPath(terrainDifftactionPath);												//�趨��������·��
				else																							//����������·����Ч�������ɾ��
					delete terrainDifftactionPath;
			}

			//ɾ������
			for (auto it = commonPath2Ds[i].begin(); it != commonPath2Ds[i].end(); ++it) {
				delete* it;
			}
			commonPath2Ds[i].clear();
			std::vector<RayPath*>().swap(commonPath2Ds[i]);

			terrainReflectPaths.clear();
			std::vector<RayPath3D*>().swap(terrainReflectPaths);

			commonPath3D.clear();
			std::vector<RayPath3D*>().swap(commonPath3D);

			validPath.clear();
			std::vector<RayPath3D*>().swap(validPath);
		}
		
		//��·���趨��result��
		pId++;
	}



	targetRtResults.clear();
	std::vector <std::vector<RaytracingResult>*>().swap(targetRtResults);

	targetPoints.clear();
	std::vector<Point2D>().swap(targetPoints);

	host_AllPathNodes.clear();
	std::vector<CPUConverterPathNode>().swap(host_AllPathNodes);

	host_TargetPathNodeIds.clear();
	std::vector<int>().swap(host_TargetPathNodeIds);
}

void PathBuilder_CPUGPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, Result& result)
{
	//1-����GPU�ϵ� CPUConverterPathNode����
	int maxDepth = 0;																					/** @brief	������	*/
	int maxNodeSize = 0;																				/** @brief	���ڵ�����	*/
	std::vector<CPUConverterPathNode> host_AllPathNodes;
	GenerateAllTreeNodeAndConvertToCPUConvertPathNode(vroots, host_AllPathNodes, maxDepth);
	maxNodeSize = maxDepth + 1;																			/** @brief	���ڵ�����Ϊ������+1	*/

	//�ϲ�rx���������
	std::vector<Point2D> targetPoints;																/** @brief	���н��յ�Ķ�ά����	*/
	std::vector<Point3D> targetPoints3D;																/** @brief	���н��յ����ά����	*/
	for (auto& curRx : scene->m_receivers) {
		if (!curRx->m_isValid) {					//������Ч�ڵ�
			continue;
		}
		targetPoints.push_back(curRx->GetPosition2D());
		targetPoints3D.push_back(curRx->m_position);
	}

	//����ȫ�����ջ�������׷�ٽ������
	int rxNum = static_cast<int>(scene->m_receivers.size());
	int txNum = static_cast<int>(scene->m_transmitters.size());
	std::vector<std::vector<RaytracingResult*>> rtResults;												/** @brief	��һά���ǽ��յ㣬�ڶ�ά���Ƿ����	*/
	for (int j = 0; j < rxNum; ++j) {
		const Receiver* curRx = scene->m_receivers[j];
		if (!curRx->m_isValid) {continue;}
		std::vector<RaytracingResult*> curRTResults;													/** @brief	��ǰ������׷�ٽ��	*/
		for (int i = 0; i < txNum; ++i) {
			const Transmitter* curTx = scene->m_transmitters[i];
			if (!curTx->m_isValid) {continue;}
			int offset = i * rxNum + j;
			curRTResults.push_back(&result.m_raytracingResult[offset]);
		}
		rtResults.push_back(curRTResults);
	}

	int num_Nodes = static_cast<int>(host_AllPathNodes.size());											/** @brief	�ڵ�����	*/
	int num_Targets = static_cast<int>(targetPoints.size());											/** @brief	Ŀ������	*/

	//2-�������ϴ���GPU�ڴ���
	thrust::device_vector< CPUConverterPathNode> dev_AllPathNodes(num_Nodes);							/** @brief	GPU �ϵ����нڵ�����	*/
	dev_AllPathNodes = host_AllPathNodes;																/** @brief	��CPU�ڴ�������GPU�ڴ���	*/

	//3-��λ�������ϴ���GPU�ڴ���
	thrust::device_vector<Point2D> dev_TargetPositions(num_Targets);									/** @brief	GPU �����е�Ŀ�������	*/
	dev_TargetPositions = targetPoints;


	//�����ڴ�����
	thrust::device_vector<int> dev_Temp_TargetNodeIds;													/** @brief	ÿ��NODE��target���յ�����	*/
	dev_Temp_TargetNodeIds.resize(num_Nodes);															/** @brief	����Ϊ�ڵ�����	*/

	thrust::device_vector<int> dev_TargetNodeIds;														/** @brief	�洢����Ŀ�������յ���NodeId	*/
	std::vector<int> everyTargetNodesNum(num_Targets);													/** @brief	ÿ��Ŀ����յ��Ľڵ������	*/

	thrust::device_vector<int> dev_TargetPathNodeIds;													/** @brief	�洢����Ŀ����յ���NodeId��׷��·��	*/

	//GPU ������趨����
	int threadPerBlock = 256;																			/** @brief	�����߳̿鸺����߳�����	*/
	int numBlock1 = (num_Nodes + threadPerBlock - 1) / threadPerBlock;									/** @brief	�߳̿�����1	*/
	cudaError_t cudaerr;																				/** @brief	cuda������	*/

	//����������GPU�����ڴ���
	cudaMemcpyToSymbol(D_CONST_MAXNODESIZE, &maxNodeSize, sizeof(int));
	cudaMemcpyToSymbol(D_CONST_RAYSPLITRADIUS1, &splitRadius, sizeof(RtLbsType));


	//4-���ú˺�������������������node��rx�����ж�
	for (int i = 0; i < num_Targets; ++i) {
		//����ʱ�ڵ�ID��Ϊ��Ч��ID �趨Ϊ-1
		thrust::fill(dev_Temp_TargetNodeIds.begin(), dev_Temp_TargetNodeIds.end(), -1);

		//����Ŀ�겶���ж�
		CheckTargetInsideKernel CUDA_KERNEL(numBlock1, threadPerBlock)(thrust::raw_pointer_cast(dev_AllPathNodes.data()), num_Nodes, thrust::raw_pointer_cast(&dev_TargetPositions[i]), thrust::raw_pointer_cast(dev_Temp_TargetNodeIds.data()));
		cudaerr = cudaDeviceSynchronize();
		if (cudaerr != cudaSuccess) {
			LOG_ERROR << "PathBuilderGPU: CheckTargetInsideKernel " << cudaGetErrorString(cudaerr) << ENDL;
		}

		//���ù�Լ��ʽ��ɾ��temp�е���Ч����
		int validNodesNum = static_cast<int>(thrust::distance(dev_Temp_TargetNodeIds.begin(), thrust::remove_if(dev_Temp_TargetNodeIds.begin(), dev_Temp_TargetNodeIds.end(), IsValidNodeId())));		/** @brief	��Ч�Ľڵ�����	*/

		//��ID��������Ӧ��������
		everyTargetNodesNum[i] = validNodesNum;
		if (validNodesNum != 0) {
			dev_TargetNodeIds.resize(dev_TargetNodeIds.size() + validNodesNum);
			thrust::copy(dev_Temp_TargetNodeIds.begin(), dev_Temp_TargetNodeIds.begin() + validNodesNum, dev_TargetNodeIds.end() - validNodesNum);
		}
	}

	std::vector<int> host_TargetNodeIds(dev_TargetNodeIds.size());
	thrust::copy(dev_TargetNodeIds.begin(), dev_TargetNodeIds.end(), host_TargetNodeIds.begin());

	int validTargetNodesNum = static_cast<int>(dev_TargetNodeIds.size());								/** @brief	��Ч��Ŀ��ڵ�����	*/
	//׷����Դ
	dev_TargetPathNodeIds.resize(validTargetNodesNum * maxNodeSize);										//����ڵ�·�������ڵ�����ΪmaxDepth
	thrust::fill(dev_TargetPathNodeIds.begin(), dev_TargetPathNodeIds.end(), -1);							//Ĭ��ȫ��Ϊ-1

	int numBlock2 = (validTargetNodesNum + threadPerBlock - 1) / threadPerBlock;						/** @brief	�߳̿�����2	*/
	FindPathNodeKernel CUDA_KERNEL(numBlock2, threadPerBlock)(thrust::raw_pointer_cast(dev_AllPathNodes.data()), num_Nodes, thrust::raw_pointer_cast(dev_TargetNodeIds.data()), dev_TargetNodeIds.size(), thrust::raw_pointer_cast(dev_TargetPathNodeIds.data()), dev_TargetPathNodeIds.size());
	cudaerr = cudaDeviceSynchronize();
	if (cudaerr != cudaSuccess) {
		LOG_ERROR << "PathBuilderGPU: FindPathNodeKernel " << cudaGetErrorString(cudaerr) << ENDL;
	}

	//�����ݿ����������ڴ���
	std::vector<int> host_TargetPathNodeIds(dev_TargetPathNodeIds.size());
	thrust::copy(dev_TargetPathNodeIds.begin(), dev_TargetPathNodeIds.end(), host_TargetPathNodeIds.begin());

	//ɾ����GPU�е�����
	dev_AllPathNodes.swap(dev_AllPathNodes);
	dev_TargetPositions.swap(dev_TargetPositions);
	dev_Temp_TargetNodeIds.swap(dev_Temp_TargetNodeIds);
	dev_TargetNodeIds.swap(dev_TargetNodeIds);
	dev_TargetPathNodeIds.swap(dev_TargetPathNodeIds);

	//�����������˹�ϵ
	int nId = 0;																						/** @brief	��ʼ�Ľڵ�ID	*/
	int pId = 0;																						/** @brief	Ŀ������� ID	*/
	for (auto curTargetNodeNum : everyTargetNodesNum) {													//����ÿ��Ŀ��
		std::vector<std::vector<RayPath*>> commonPath2Ds;												//����ྶ
		commonPath2Ds.resize(txNum, std::vector<RayPath*>());											//���䳣��·���ڴ�
		for (int i = 0; i < curTargetNodeNum; ++i) {
			std::vector<CPUConverterPathNode*> pathNodes;												/** @brief	CPUNode���ɵ�path	*/
			pathNodes.reserve(maxNodeSize);
			for (int j = 0; j < maxNodeSize; ++j) {
				int curNodeId = host_TargetPathNodeIds[nId++];
				if (curNodeId == -1) {																	//������Ч�ڵ�
					continue;
				}
				pathNodes.push_back(&host_AllPathNodes[curNodeId]);
			}
			int sensorId = pathNodes[0]->m_sensorId;
			//��pathת��ΪRayPath
			RayPath* newRayPath = new RayPath();
			newRayPath->ConvertFrom(pathNodes, scene->m_segmentBuf, scene->m_wedgeBuf);
			//��֤·������Ч��
			if (newRayPath->IsValidAndRectify(targetPoints[pId], scene)) {
				commonPath2Ds[sensorId].push_back(newRayPath);
			}
			else {
				delete newRayPath;
			}
		}

		//·����ֵ
		for (int i = 0; i < txNum; ++i) {
			const Transmitter* curTx = scene->m_transmitters[i];
			if (!curTx->m_isValid) { continue; }
			//����ά·��ת��Ϊ��ά·��
			std::vector<RayPath3D*> commonPath3D(commonPath2Ds[i].size());
			for (int j = 0; j < commonPath2Ds[i].size(); ++j) {
				commonPath3D[j] = new RayPath3D(*commonPath2Ds[i][j], curTx->m_position, targetPoints3D[pId]);
			}

			//���淴�侶
			std::vector<RayPath3D*> terrainReflectPaths;														/** @brief	���淴��·��(��ת�͵�·������Ŀ�괫����������)	*/
			if (scene->GetGroundReflectPaths(curTx->m_position, targetPoints3D[pId], terrainReflectPaths)) {
				for (auto it = terrainReflectPaths.begin(); it != terrainReflectPaths.end(); ++it) {			//�������˵��淴��·�����򽫵��淴��·�����������·����
					commonPath3D.push_back(*it);
				}
			}

			//ɾ����Ч��·��
			std::vector<RayPath3D*> validPath;																	/** @brief	��Ч·��	*/
			for (auto it = commonPath3D.begin(); it != commonPath3D.end(); ++it) {
				RayPath3D* curPath = *it;
				if (!scene->IsValidRayPath(curPath)) {															//·����֤��ͨ������Ч·����ֱ��ɾ��
					delete curPath;
					curPath = nullptr;
				}
				else {
					validPath.push_back(curPath);
				}
			}
			rtResults[pId][i]->SetRayPath(validPath);

			//�������侶
			TerrainDiffractionPath* terrainDifftactionPath = nullptr;
			if (scene->GetGroundDiffractionPath(curTx->m_position, targetPoints3D[pId], terrainDifftactionPath)) {
				if (scene->IsValidRayPath(terrainDifftactionPath))
					rtResults[pId][i]->SetRayPath(terrainDifftactionPath);												//�趨��������·��
				else																							//����������·����Ч�������ɾ��
					delete terrainDifftactionPath;
			}

			//ɾ������
			for (auto it = commonPath2Ds[i].begin(); it != commonPath2Ds[i].end(); ++it) {
				delete* it;
			}
			commonPath2Ds[i].clear();
			std::vector<RayPath*>().swap(commonPath2Ds[i]);

			terrainReflectPaths.clear();
			std::vector<RayPath3D*>().swap(terrainReflectPaths);

			commonPath3D.clear();
			std::vector<RayPath3D*>().swap(commonPath3D);

			validPath.clear();
			std::vector<RayPath3D*>().swap(validPath);
		}

		pId++;
	}

	rtResults.clear();
	std::vector <std::vector<RaytracingResult*>>().swap(rtResults);

	targetPoints.clear();
	std::vector<Point2D>().swap(targetPoints);

	host_AllPathNodes.clear();
	std::vector<CPUConverterPathNode>().swap(host_AllPathNodes);

	host_TargetPathNodeIds.clear();
	std::vector<int>().swap(host_TargetPathNodeIds);
}
