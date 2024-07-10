#include "pathbuildergpu.h"

__constant__ int D_CONST_MAXNODESIZE;												/** @brief	L1设备快速内存-最大深度	*/
__constant__ RtLbsType D_CONST_RAYSPLITRADIUS1;									/** @brief	L1设备快速内存-射线分裂半径	*/


//满足GPU运算的谓词条件结构
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
		if (curNode.IsCapturedByPoint(*target, D_CONST_RAYSPLITRADIUS1, fatherNode)) {			//若被接收，则加入nodesId中
			nodeIds[nodeIdx] = nodeIdx;												//赋值接收点ID
		}
	}
}

__global__ void FindPathNodeKernel(CPUConverterPathNode* allNodes, int numNodes, int* nodeIds, int numId, int* pathNodeIds, int numPathId)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx < numId) {
		int curNodeId = nodeIds[idx];															/** @brief	当前的节点ID	*/
		int pathNodeId = (idx + 1) * D_CONST_MAXNODESIZE - 1;										/** @brief	设计从末尾开始进行写入，进行追根溯源	*/
		pathNodeIds[pathNodeId] = curNodeId;
		int fatherNodeId = allNodes[curNodeId].m_fatherNodeId;
		for (int i = 0; i < D_CONST_MAXNODESIZE - 1; ++i) {
			if (fatherNodeId == -1) {															
				break;
			}
			pathNodeIds[--pathNodeId] = fatherNodeId;
			fatherNodeId = allNodes[fatherNodeId].m_fatherNodeId;								//进行下一轮父节点寻找
		}
	}
}

void DirectlySetResultPath_GPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRaidus, std::vector<GSPairCluster>& clusters)
{
	//遍历vroots中的所有数据，生成CPUConverterPathNode,在CPUConverterPathNode添加传感器ID属性，用于标识其属于哪一个传感器，在GPU代码中，增加循环，循环的参数是接收点的数量

	//1-生成GPU上的 CPUConverterPathNode数组
	int maxDepth = 0;																					/** @brief	最大深度	*/
	int maxNodeSize = 0;																				/** @brief	最大节点数量	*/
	std::vector<CPUConverterPathNode> host_AllPathNodes;
	GenerateAllTreeNodeAndConvertToCPUConvertPathNode(vroots, host_AllPathNodes, maxDepth);
	maxNodeSize = maxDepth + 1;																			/** @brief	最大节点数量为最大深度+1	*/

	//合并cluster中所有的坐标点数量
	std::vector<Point2D> targetPoints;
	targetPoints.reserve(5 * clusters.size());
	for (auto& cluster : clusters) {
		for (auto& point : cluster.m_aroundPoints) {
			targetPoints.push_back(point);
		}
	}

	//构造对应坐标点的射线追踪结果数组
	std::vector<std::vector<RaytracingResult>*> targetRtResults;
	for (auto& cluster : clusters) {
		for (auto& results : cluster.m_rtResult) {
			results.resize(scene->m_sensors.size());
			targetRtResults.push_back(&results);
		}
	}


	int num_Nodes = static_cast<int>(host_AllPathNodes.size());											/** @brief	节点数量	*/
	int num_Targets = static_cast<int>(targetPoints.size());											/** @brief	目标数量	*/

	//2-将数组上传至GPU内存中
	thrust::device_vector< CPUConverterPathNode> dev_AllPathNodes(num_Nodes);							/** @brief	GPU 上的所有节点数据	*/
	dev_AllPathNodes = host_AllPathNodes;																/** @brief	将CPU内存上载至GPU内存中	*/

	//3-将位置数组上传至GPU内存中
	thrust::device_vector<Point2D> dev_TargetPositions(num_Targets);									/** @brief	GPU 上所有的目标点数据	*/
	dev_TargetPositions = targetPoints;																

	//定义内存区域
	thrust::device_vector<int> dev_Temp_TargetNodeIds;													/** @brief	每个NODE对target接收的数据	*/
	dev_Temp_TargetNodeIds.resize(num_Nodes);															/** @brief	数量为节点总数	*/

	thrust::device_vector<int> dev_TargetNodeIds;														/** @brief	存储所有目标所接收到的NodeId	*/
	std::vector<int> everyTargetNodesNum(num_Targets);													/** @brief	每个目标接收到的节点的数量	*/

	thrust::device_vector<int> dev_TargetPathNodeIds;													/** @brief	存储所有目标接收到的NodeId并追根路径	*/

	//GPU 程序块设定参数
	int threadPerBlock = 256;																			/** @brief	单个线程块负责的线程数量	*/
	int numBlock1 = (num_Nodes + threadPerBlock - 1) / threadPerBlock;									/** @brief	线程块数量1	*/
	cudaError_t cudaerr;																				/** @brief	cuda错误处理	*/

	//拷贝常量至GPU快速内存中
	cudaMemcpyToSymbol(D_CONST_MAXNODESIZE, &maxNodeSize, sizeof(int));
	cudaMemcpyToSymbol(D_CONST_RAYSPLITRADIUS1, &splitRaidus, sizeof(RtLbsType));


	//4-调用核函数，计算所有数组中node于rx接收判定
	for (int i = 0; i < static_cast<int>(dev_TargetPositions.size()); ++i) {
		//将临时节点ID设为无效，ID 设定为-1
		thrust::fill(dev_Temp_TargetNodeIds.begin(), dev_Temp_TargetNodeIds.end(), -1);

		//进行目标捕获判定
		CheckTargetInsideKernel CUDA_KERNEL(numBlock1, threadPerBlock)(thrust::raw_pointer_cast(dev_AllPathNodes.data()), num_Nodes, thrust::raw_pointer_cast(&dev_TargetPositions[i]), thrust::raw_pointer_cast(dev_Temp_TargetNodeIds.data()));
		cudaerr = cudaDeviceSynchronize();
		if (cudaerr != cudaSuccess) {
			LOG_ERROR << "PathBuilderGPU: CheckTargetInsideKernel " << cudaGetErrorString(cudaerr) << ENDL;
		}

		//采用规约形式，删除temp中的无效数据
		int validNodesNum = static_cast<int>(thrust::distance(dev_Temp_TargetNodeIds.begin(), thrust::remove_if(dev_Temp_TargetNodeIds.begin(), dev_Temp_TargetNodeIds.end(), IsValidNodeId())));		/** @brief	有效的节点数量	*/

		//将ID拷贝至对应的数据中
		everyTargetNodesNum[i] = validNodesNum;
		if (validNodesNum != 0) {
			dev_TargetNodeIds.resize(dev_TargetNodeIds.size() + validNodesNum);
			thrust::copy(dev_Temp_TargetNodeIds.begin(), dev_Temp_TargetNodeIds.begin() + validNodesNum, dev_TargetNodeIds.end() - validNodesNum);
		}
	}

	std::vector<int> host_TargetNodeIds(dev_TargetNodeIds.size());
	thrust::copy(dev_TargetNodeIds.begin(), dev_TargetNodeIds.end(), host_TargetNodeIds.begin());

	int validTargetNodesNum = static_cast<int>(dev_TargetNodeIds.size());								/** @brief	有效的目标节点数量	*/
	//追根溯源
	dev_TargetPathNodeIds.resize(validTargetNodesNum * maxNodeSize);										//定义节点路径，最大节点数量为maxDepth
	thrust::fill(dev_TargetPathNodeIds.begin(), dev_TargetPathNodeIds.end(), -1);							//默认全置为-1

	int numBlock2 = (validTargetNodesNum + threadPerBlock - 1) / threadPerBlock;						/** @brief	线程块数量2	*/
	FindPathNodeKernel CUDA_KERNEL(numBlock2, threadPerBlock)(thrust::raw_pointer_cast(dev_AllPathNodes.data()), num_Nodes, thrust::raw_pointer_cast(dev_TargetNodeIds.data()), dev_TargetNodeIds.size(), thrust::raw_pointer_cast(dev_TargetPathNodeIds.data()), dev_TargetPathNodeIds.size());
	cudaerr = cudaDeviceSynchronize();
	if (cudaerr != cudaSuccess) {
		LOG_ERROR << "PathBuilderGPU: FindPathNodeKernel " << cudaGetErrorString(cudaerr) << ENDL;
	}

	

	//将数据拷贝至主机内存中
	std::vector<int> host_TargetPathNodeIds(dev_TargetPathNodeIds.size());
	thrust::copy(dev_TargetPathNodeIds.begin(), dev_TargetPathNodeIds.end(), host_TargetPathNodeIds.begin());

	//删除在GPU中的数据
	dev_AllPathNodes.swap(dev_AllPathNodes);
	dev_TargetPositions.swap(dev_TargetPositions);
	dev_Temp_TargetNodeIds.swap(dev_Temp_TargetNodeIds);
	dev_TargetNodeIds.swap(dev_TargetNodeIds);
	dev_TargetPathNodeIds.swap(dev_TargetPathNodeIds);

	//分析数据拓扑关系
	int nId = 0;																						/** @brief	初始的节点ID	*/
	int pId = 0;																						/** @brief	目标坐标点 ID	*/
	for (auto curTargetNodeNum : everyTargetNodesNum) {													//遍历每个目标
		std::vector<std::vector<RayPath*>> commonPath2Ds;												//常规多径
		commonPath2Ds.resize(scene->m_sensors.size(), std::vector<RayPath*>());							//分配常规路径内存
		for (int i = 0; i < curTargetNodeNum; ++i) {
			std::vector<CPUConverterPathNode*> pathNodes;												/** @brief	CPUNode构成的path	*/
			pathNodes.reserve(maxNodeSize);
			for (int j = 0; j < maxNodeSize; ++j) {
				int curNodeId = host_TargetPathNodeIds[nId++];
				if (curNodeId == -1) {																	//跳过无效节点
					continue;
				}
				pathNodes.push_back(&host_AllPathNodes[curNodeId]);
			}
			int sensorId = pathNodes[0]->m_sensorId;
			//将path转换为RayPath
			RayPath* newRayPath = new RayPath();
			newRayPath->ConvertFrom(pathNodes, scene->m_segmentBuf, scene->m_wedgeBuf);
			//验证路径的有效性
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
			//将常规路径转换为三维路径
			std::vector<RayPath3D*> commonPath3D(commonPath2Ds[i].size());
			for (int j = 0; j < commonPath2Ds[i].size(); ++j) {
				commonPath3D[j] = new RayPath3D(*commonPath2Ds[i][j], curSensor->m_position, targetPoint3D);
				commonPath3D[j]->ReverseRayPath();																//由于传感器和目标源之间的路径的可逆性，因此对路径进行逆向处理
			}

			//地面反射径
			std::vector<RayPath3D*> terrainReflectPaths;														/** @brief	地面反射路径(逆转型的路径，从目标传播到传感器)	*/
			if (scene->GetGroundReflectPaths(targetPoint3D, curSensor->m_position, terrainReflectPaths)) {
				for (auto it = terrainReflectPaths.begin(); it != terrainReflectPaths.end(); ++it) {			//若产生了地面反射路径，则将地面反射路径纳入进常规路径中
					commonPath3D.push_back(*it);
				}
			}

			//删除无效的路径
			std::vector<RayPath3D*> validPath;																	/** @brief	有效路径	*/
			for (auto it = commonPath3D.begin(); it != commonPath3D.end(); ++it) {
				RayPath3D* curPath = *it;
				if (!scene->IsValidRayPath(curPath)) {															//路径验证不通过，无效路径，直接删除
					delete curPath;
					curPath = nullptr;
				}
				else {
					validPath.push_back(curPath);
				}
			}
			(*targetRtResults[pId])[i].SetRayPath(validPath);


			//处理地形绕射路径
			TerrainDiffractionPath* terrainDifftactionPath = nullptr;
			if (scene->GetGroundDiffractionPath(targetPoint3D, curSensor->m_position, terrainDifftactionPath)) {
				if (scene->IsValidRayPath(terrainDifftactionPath))
					(*targetRtResults[pId])[i].SetRayPath(terrainDifftactionPath);												//设定地形绕射路径
				else																							//若地形绕射路径无效，则进行删除
					delete terrainDifftactionPath;
			}

			//删除缓存
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
		
		//将路径设定在result中
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
	//1-生成GPU上的 CPUConverterPathNode数组
	int maxDepth = 0;																					/** @brief	最大深度	*/
	int maxNodeSize = 0;																				/** @brief	最大节点数量	*/
	std::vector<CPUConverterPathNode> host_AllPathNodes;
	GenerateAllTreeNodeAndConvertToCPUConvertPathNode(vroots, host_AllPathNodes, maxDepth);
	maxNodeSize = maxDepth + 1;																			/** @brief	最大节点数量为最大深度+1	*/

	//合并rx坐标点数量
	std::vector<Point2D> targetPoints;																/** @brief	所有接收点的二维坐标	*/
	std::vector<Point3D> targetPoints3D;																/** @brief	所有接收点的三维坐标	*/
	for (auto& curRx : scene->m_receivers) {
		if (!curRx->m_isValid) {					//跳过无效节点
			continue;
		}
		targetPoints.push_back(curRx->GetPosition2D());
		targetPoints3D.push_back(curRx->m_position);
	}

	//构造全部接收机的射线追踪结果数组
	int rxNum = static_cast<int>(scene->m_receivers.size());
	int txNum = static_cast<int>(scene->m_transmitters.size());
	std::vector<std::vector<RaytracingResult*>> rtResults;												/** @brief	第一维度是接收点，第二维度是发射点	*/
	for (int j = 0; j < rxNum; ++j) {
		const Receiver* curRx = scene->m_receivers[j];
		if (!curRx->m_isValid) {continue;}
		std::vector<RaytracingResult*> curRTResults;													/** @brief	当前的射线追踪结果	*/
		for (int i = 0; i < txNum; ++i) {
			const Transmitter* curTx = scene->m_transmitters[i];
			if (!curTx->m_isValid) {continue;}
			int offset = i * rxNum + j;
			curRTResults.push_back(&result.m_raytracingResult[offset]);
		}
		rtResults.push_back(curRTResults);
	}

	int num_Nodes = static_cast<int>(host_AllPathNodes.size());											/** @brief	节点数量	*/
	int num_Targets = static_cast<int>(targetPoints.size());											/** @brief	目标数量	*/

	//2-将数组上传至GPU内存中
	thrust::device_vector< CPUConverterPathNode> dev_AllPathNodes(num_Nodes);							/** @brief	GPU 上的所有节点数据	*/
	dev_AllPathNodes = host_AllPathNodes;																/** @brief	将CPU内存上载至GPU内存中	*/

	//3-将位置数组上传至GPU内存中
	thrust::device_vector<Point2D> dev_TargetPositions(num_Targets);									/** @brief	GPU 上所有的目标点数据	*/
	dev_TargetPositions = targetPoints;


	//定义内存区域
	thrust::device_vector<int> dev_Temp_TargetNodeIds;													/** @brief	每个NODE对target接收的数据	*/
	dev_Temp_TargetNodeIds.resize(num_Nodes);															/** @brief	数量为节点总数	*/

	thrust::device_vector<int> dev_TargetNodeIds;														/** @brief	存储所有目标所接收到的NodeId	*/
	std::vector<int> everyTargetNodesNum(num_Targets);													/** @brief	每个目标接收到的节点的数量	*/

	thrust::device_vector<int> dev_TargetPathNodeIds;													/** @brief	存储所有目标接收到的NodeId并追根路径	*/

	//GPU 程序块设定参数
	int threadPerBlock = 256;																			/** @brief	单个线程块负责的线程数量	*/
	int numBlock1 = (num_Nodes + threadPerBlock - 1) / threadPerBlock;									/** @brief	线程块数量1	*/
	cudaError_t cudaerr;																				/** @brief	cuda错误处理	*/

	//拷贝常量至GPU快速内存中
	cudaMemcpyToSymbol(D_CONST_MAXNODESIZE, &maxNodeSize, sizeof(int));
	cudaMemcpyToSymbol(D_CONST_RAYSPLITRADIUS1, &splitRadius, sizeof(RtLbsType));


	//4-调用核函数，计算所有数组中node于rx接收判定
	for (int i = 0; i < num_Targets; ++i) {
		//将临时节点ID设为无效，ID 设定为-1
		thrust::fill(dev_Temp_TargetNodeIds.begin(), dev_Temp_TargetNodeIds.end(), -1);

		//进行目标捕获判定
		CheckTargetInsideKernel CUDA_KERNEL(numBlock1, threadPerBlock)(thrust::raw_pointer_cast(dev_AllPathNodes.data()), num_Nodes, thrust::raw_pointer_cast(&dev_TargetPositions[i]), thrust::raw_pointer_cast(dev_Temp_TargetNodeIds.data()));
		cudaerr = cudaDeviceSynchronize();
		if (cudaerr != cudaSuccess) {
			LOG_ERROR << "PathBuilderGPU: CheckTargetInsideKernel " << cudaGetErrorString(cudaerr) << ENDL;
		}

		//采用规约形式，删除temp中的无效数据
		int validNodesNum = static_cast<int>(thrust::distance(dev_Temp_TargetNodeIds.begin(), thrust::remove_if(dev_Temp_TargetNodeIds.begin(), dev_Temp_TargetNodeIds.end(), IsValidNodeId())));		/** @brief	有效的节点数量	*/

		//将ID拷贝至对应的数据中
		everyTargetNodesNum[i] = validNodesNum;
		if (validNodesNum != 0) {
			dev_TargetNodeIds.resize(dev_TargetNodeIds.size() + validNodesNum);
			thrust::copy(dev_Temp_TargetNodeIds.begin(), dev_Temp_TargetNodeIds.begin() + validNodesNum, dev_TargetNodeIds.end() - validNodesNum);
		}
	}

	std::vector<int> host_TargetNodeIds(dev_TargetNodeIds.size());
	thrust::copy(dev_TargetNodeIds.begin(), dev_TargetNodeIds.end(), host_TargetNodeIds.begin());

	int validTargetNodesNum = static_cast<int>(dev_TargetNodeIds.size());								/** @brief	有效的目标节点数量	*/
	//追根溯源
	dev_TargetPathNodeIds.resize(validTargetNodesNum * maxNodeSize);										//定义节点路径，最大节点数量为maxDepth
	thrust::fill(dev_TargetPathNodeIds.begin(), dev_TargetPathNodeIds.end(), -1);							//默认全置为-1

	int numBlock2 = (validTargetNodesNum + threadPerBlock - 1) / threadPerBlock;						/** @brief	线程块数量2	*/
	FindPathNodeKernel CUDA_KERNEL(numBlock2, threadPerBlock)(thrust::raw_pointer_cast(dev_AllPathNodes.data()), num_Nodes, thrust::raw_pointer_cast(dev_TargetNodeIds.data()), dev_TargetNodeIds.size(), thrust::raw_pointer_cast(dev_TargetPathNodeIds.data()), dev_TargetPathNodeIds.size());
	cudaerr = cudaDeviceSynchronize();
	if (cudaerr != cudaSuccess) {
		LOG_ERROR << "PathBuilderGPU: FindPathNodeKernel " << cudaGetErrorString(cudaerr) << ENDL;
	}

	//将数据拷贝至主机内存中
	std::vector<int> host_TargetPathNodeIds(dev_TargetPathNodeIds.size());
	thrust::copy(dev_TargetPathNodeIds.begin(), dev_TargetPathNodeIds.end(), host_TargetPathNodeIds.begin());

	//删除在GPU中的数据
	dev_AllPathNodes.swap(dev_AllPathNodes);
	dev_TargetPositions.swap(dev_TargetPositions);
	dev_Temp_TargetNodeIds.swap(dev_Temp_TargetNodeIds);
	dev_TargetNodeIds.swap(dev_TargetNodeIds);
	dev_TargetPathNodeIds.swap(dev_TargetPathNodeIds);

	//分析数据拓扑关系
	int nId = 0;																						/** @brief	初始的节点ID	*/
	int pId = 0;																						/** @brief	目标坐标点 ID	*/
	for (auto curTargetNodeNum : everyTargetNodesNum) {													//遍历每个目标
		std::vector<std::vector<RayPath*>> commonPath2Ds;												//常规多径
		commonPath2Ds.resize(txNum, std::vector<RayPath*>());											//分配常规路径内存
		for (int i = 0; i < curTargetNodeNum; ++i) {
			std::vector<CPUConverterPathNode*> pathNodes;												/** @brief	CPUNode构成的path	*/
			pathNodes.reserve(maxNodeSize);
			for (int j = 0; j < maxNodeSize; ++j) {
				int curNodeId = host_TargetPathNodeIds[nId++];
				if (curNodeId == -1) {																	//跳过无效节点
					continue;
				}
				pathNodes.push_back(&host_AllPathNodes[curNodeId]);
			}
			int sensorId = pathNodes[0]->m_sensorId;
			//将path转换为RayPath
			RayPath* newRayPath = new RayPath();
			newRayPath->ConvertFrom(pathNodes, scene->m_segmentBuf, scene->m_wedgeBuf);
			//验证路径的有效性
			if (newRayPath->IsValidAndRectify(targetPoints[pId], scene)) {
				commonPath2Ds[sensorId].push_back(newRayPath);
			}
			else {
				delete newRayPath;
			}
		}

		//路径赋值
		for (int i = 0; i < txNum; ++i) {
			const Transmitter* curTx = scene->m_transmitters[i];
			if (!curTx->m_isValid) { continue; }
			//将二维路径转换为三维路径
			std::vector<RayPath3D*> commonPath3D(commonPath2Ds[i].size());
			for (int j = 0; j < commonPath2Ds[i].size(); ++j) {
				commonPath3D[j] = new RayPath3D(*commonPath2Ds[i][j], curTx->m_position, targetPoints3D[pId]);
			}

			//地面反射径
			std::vector<RayPath3D*> terrainReflectPaths;														/** @brief	地面反射路径(逆转型的路径，从目标传播到传感器)	*/
			if (scene->GetGroundReflectPaths(curTx->m_position, targetPoints3D[pId], terrainReflectPaths)) {
				for (auto it = terrainReflectPaths.begin(); it != terrainReflectPaths.end(); ++it) {			//若产生了地面反射路径，则将地面反射路径纳入进常规路径中
					commonPath3D.push_back(*it);
				}
			}

			//删除无效的路径
			std::vector<RayPath3D*> validPath;																	/** @brief	有效路径	*/
			for (auto it = commonPath3D.begin(); it != commonPath3D.end(); ++it) {
				RayPath3D* curPath = *it;
				if (!scene->IsValidRayPath(curPath)) {															//路径验证不通过，无效路径，直接删除
					delete curPath;
					curPath = nullptr;
				}
				else {
					validPath.push_back(curPath);
				}
			}
			rtResults[pId][i]->SetRayPath(validPath);

			//地面绕射径
			TerrainDiffractionPath* terrainDifftactionPath = nullptr;
			if (scene->GetGroundDiffractionPath(curTx->m_position, targetPoints3D[pId], terrainDifftactionPath)) {
				if (scene->IsValidRayPath(terrainDifftactionPath))
					rtResults[pId][i]->SetRayPath(terrainDifftactionPath);												//设定地形绕射路径
				else																							//若地形绕射路径无效，则进行删除
					delete terrainDifftactionPath;
			}

			//删除缓存
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
