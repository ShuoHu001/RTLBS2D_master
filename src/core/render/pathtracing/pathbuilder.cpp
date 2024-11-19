#include "pathbuilder.h"
#include "result/result.h"
#include "result/raytracingresult.h"

void PathBuilder_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, Result& result)
{
	size_t txNum = scene->m_transmitters.size();
	size_t rxNum = scene->m_receivers.size();
	for (size_t i = 0; i < txNum; ++i) {			//遍历每个发射天线
		LOG_INFO << "PathBuilder: starting tx-" << i << "path builder." << ENDL;
		const Transmitter* curTransmitter = scene->m_transmitters[i];		/** @brief	当前遍历到的发射机	*/
		if (!curTransmitter->m_isValid) {									//跳过无效的发射机
			continue;
		}
		for (size_t j = 0; j < rxNum; ++j) {
			std::cout << j << std::endl;
			const Receiver* curReceiver = scene->m_receivers[j];			/** @brief	当前遍历到的接收机	*/
			if (!curReceiver->m_isValid) {									//跳过无效的接收机
				continue;
			}
			size_t offset = i * rxNum + j;																							//
			//-----------------------------------------------------常规路径设定-------------------------------------------------------
			std::vector<RayPath*> commonPath2D;
			GenerateMultipathofPoint(vroots[i], curReceiver->GetPosition2D(), scene, splitRadius, commonPath2D);

			//将常规路径转换为3D路径
			std::vector<RayPath3D*> commonPath3D(commonPath2D.size());																//常规3D路径分配内存（包含常规二维路径转换+地形反射路径）
			for (int i = 0; i < commonPath2D.size(); ++i) {																			//转换常规二维路径
				commonPath3D[i] = new RayPath3D(*commonPath2D[i], curTransmitter->m_position, curReceiver->m_position);
			}

			std::vector<RayPath3D*> terrainReflectPaths;																			/** @brief	地面反射路径	*/
			if (scene->GetGroundReflectPaths(curTransmitter->m_position, curReceiver->m_position, terrainReflectPaths)) {
				for (auto it = terrainReflectPaths.begin(); it != terrainReflectPaths.end(); ++it) {								//若产生了地面反射路径，则将地面反射路径纳入进常规路径中
					commonPath3D.push_back(*it);
				}
			}

			//查验并设定常规路径
			std::vector<RayPath3D*> validCommonPath;																				/** @brief	有效的常规路径集合	*/
			for (auto it = commonPath3D.begin(); it != commonPath3D.end(); ++it) {													//筛选路径
				RayPath3D*& curPath = *it;
				if (!scene->IsValidRayPath(curPath)) {																				//路径无效
					delete curPath;
					curPath = nullptr;
				}
				else																												//路径有效
					validCommonPath.push_back(curPath);
			}
			result.m_raytracingResult[offset].SetRayPath(validCommonPath);														//设定常规三维路径


			//------------------------------------------------------------地形绕射路径设定-----------------------------------------------------------------
			TerrainDiffractionPath* terrainDifftactionPath = nullptr;																/** @brief	地形绕射路径	*/
			if (scene->GetGroundDiffractionPath(curTransmitter->m_position, curReceiver->m_position, terrainDifftactionPath)) {
				if (scene->IsValidRayPath(terrainDifftactionPath))
					result.m_raytracingResult[offset].SetRayPath(terrainDifftactionPath);											//设定地形绕射路径
				else																												//若地形绕射路径无效，则进行删除
					delete terrainDifftactionPath;
			}

			//------------------------------------------------------------释放内存------------------------------------------------------------------------
			for (auto& path : commonPath2D) {													//释放二维路径内存
				delete path;
			}
			commonPath2D.clear();
			std::vector<RayPath*>().swap(commonPath2D);

			commonPath3D.clear();
			std::vector<RayPath3D*>().swap(commonPath3D);

			terrainReflectPaths.clear();
			std::vector<RayPath3D*>().swap(terrainReflectPaths);

			validCommonPath.clear();
			std::vector<RayPath3D*>().swap(validCommonPath);
		}
	}
}

void PathBuilder_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, int16_t threadNum, Result& result)
{
	size_t txNum = scene->m_transmitters.size();
	size_t rxNum = scene->m_receivers.size();
	for (size_t i = 0; i < txNum; ++i) {
		LOG_INFO << "PathBuilder: starting tx-" << i << "path builder." << ENDL;
		const Transmitter* curTransmitter = scene->m_transmitters[i];		/** @brief	当前遍历到的发射机	*/
		if (!curTransmitter->m_isValid) {									//跳过无效的发射机
			continue;
		}

		//--------------------------------------------------------------------CPU并行处理路径收集---------------------------------------------------------------
		ThreadPool pool(threadNum);
		std::vector<std::vector<RayPath*>> commonPaths(scene->m_receivers.size(), std::vector<RayPath*>());						/** @brief	所有接收点的二维路径数组	*/
		for (size_t j = 0; j < scene->m_receivers.size(); ++j) {																//将任务压入线程池中
			auto future = pool.enqueue(GenerateMultipathofPoint, vroots[i], scene->m_receivers[j]->GetPosition2D(), scene, splitRadius, commonPaths[j]);
		}
		while (true) {
			if (pool.getTaskCount() == 0) {
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10000)); // 每100毫秒检查一次
			std::cout << pool.getTaskCount() << std::endl;
		}

		//-----------------------------------------------------------------常规路径转换并判定是否有效-----------------------------------------------------------
		for (int j = 0; j < rxNum; ++j) {
			Receiver* curReceiver = scene->m_receivers[i];																		/** @brief	当前的接收机	*/
			size_t offset = i * rxNum + j;												/** @brief	设定结果在结果数组中的偏移量	*/
			std::vector<RayPath3D*> commanPath3D;																					/** @brief	每个接收机所接收到的三维路径	*/
			commanPath3D.reserve(commonPaths[j].size() + 2);																			//预留大小
			int validCommonPathId = 0;
			for (size_t k = 0; k < commonPaths[j].size(); ++k) {																		//遍历路径进行3D路径转换
				RayPath3D* newPath = new RayPath3D(*commonPaths[j][k], curTransmitter->m_position, curReceiver->m_position);			//转换路径
				if (scene->IsValidRayPath(newPath)) {																				//若路径有效，则纳入至路径数组中
					commanPath3D[validCommonPathId++] = newPath;
				}
				else {																												//若路径无效，则进行内存删除
					delete newPath;
				}
			}

			std::vector<RayPath3D*> groundReflectionPath;																			//地面反射路径
			if (scene->GetGroundReflectPaths(curTransmitter->m_position, curReceiver->m_position, groundReflectionPath)) {
				for (auto it = groundReflectionPath.begin(); it != groundReflectionPath.end(); ++it) {
					if (scene->IsValidRayPath(*it))																				//有效保留，无效删除
						commanPath3D[validCommonPathId++] = *it;
					else
						delete* it;
				}
			}

			result.m_raytracingResult[offset].SetRayPath(commanPath3D);															//设定常规路径


			//地形绕射路径
			TerrainDiffractionPath* terrainDifftactionPath = nullptr;																/** @brief	地形绕射路径	*/
			if (scene->GetGroundDiffractionPath(curTransmitter->m_position, curReceiver->m_position, terrainDifftactionPath)) {
				if (scene->IsValidRayPath(terrainDifftactionPath))
					result.m_raytracingResult[offset].SetRayPath(terrainDifftactionPath);
				else
					delete terrainDifftactionPath;
			}

			commanPath3D.clear();
			std::vector<RayPath3D*>().swap(commanPath3D);

			groundReflectionPath.clear();
			std::vector<RayPath3D*>().swap(groundReflectionPath);
		}

		for (auto& path2Ds : commonPaths) {
			for (auto& path2D : path2Ds) {
				delete path2D;
				path2D = nullptr;
			}
			path2Ds.clear();
			std::vector<RayPath*>().swap(path2Ds);
		}
		commonPaths.clear();
		std::vector<std::vector<RayPath*>>().swap(commonPaths);

	}
}

void PathBuilder_GPUMultiThread(const std::vector<std::vector<PathNodeGPU*>>& gpuPathNodes, const Scene* scene, Result& result)
{
	int txNum = static_cast<int>(scene->m_transmitters.size());
	int rxNum = static_cast<int>(scene->m_receivers.size());
	for (int i = 0; i < txNum; ++i) {
		LOG_INFO << "PathBuilder: starting tx-" << i << "path builder." << ENDL;
		const Transmitter* curTransmitter = scene->m_transmitters[i];		/** @brief	当前遍历到的发射机	*/
		if (!curTransmitter->m_isValid) {									//跳过无效的发射机
			continue;
		}


		//读取GPU运算结果，分离出多径信息
		std::vector<std::vector<RayPathGPU*>> commonPaths;																			/** @brief	常规GPU路径，第一层为rx数量，第二层为多径数量	*/
		commonPaths.resize(rxNum);
		Point2D txPosition2D(curTransmitter->m_position.x, curTransmitter->m_position.y);													/** @brief	发射机坐标（二维）	*/
		std::vector<Point2D> rxPositons2D(scene->m_receivers.size());																/** @brief	接收机坐标（二维）	*/
		for (size_t i = 0; i < scene->m_receivers.size(); ++i) {
			rxPositons2D[i].x = scene->m_receivers[i]->m_position.x;
			rxPositons2D[i].y = scene->m_receivers[i]->m_position.y;
		}
		GenerateMultiPathofRxSingleTxGPU(gpuPathNodes[i], txPosition2D, rxPositons2D, scene->m_gpuSegmentBuf.data(), scene, commonPaths);		//基于GPU节点解析出多径信息

		//将GPU多径转换为CPU中的三维多径信息
		for (int j = 0; j < rxNum; ++j) {																		//遍历每个rx，转换并筛选路径
			Receiver* curReceiver = scene->m_receivers[j];																		/** @brief	当前的接收机	*/
			int offset = i * rxNum + j;												/** @brief	设定结果在结果数组中的偏移量	*/
			std::vector<RayPath3D*> commanPath3D;																					/** @brief	每个接收机所接收到的三维路径	*/
			commanPath3D.resize(commonPaths[j].size() + 2);																			//预留大小
			int validCommonPathId = 0;
			for (int k = 0; k < commonPaths[j].size(); ++k) {
				RayPath3D* newPath = new RayPath3D(*commonPaths[j][k], curTransmitter->m_position, curReceiver->m_position, scene->m_segmentBuf, scene->m_wedgeBuf);
				if (scene->IsValidRayPath(newPath)) {																				//若路径有效，则纳入至路径数组中
					commanPath3D[validCommonPathId++] = newPath;
				}
				else {																												//若路径无效，则进行内存删除
					delete newPath;
				}
			}


			std::vector<RayPath3D*> groundReflectionPath;																			//地面反射路径
			if (scene->GetGroundReflectPaths(curTransmitter->m_position, curReceiver->m_position, groundReflectionPath)) {
				for (auto it = groundReflectionPath.begin(); it != groundReflectionPath.end(); ++it) {
					if (scene->IsValidRayPath(*it))																				//有效保留，无效删除
						commanPath3D[validCommonPathId++] = *it;
					else
						delete* it;
				}
			}
			commanPath3D.resize(validCommonPathId);
			result.m_raytracingResult[offset].SetRayPath(commanPath3D);															//设定常规路径


			//地形绕射路径
			TerrainDiffractionPath* terrainDifftactionPath = nullptr;																/** @brief	地形绕射路径	*/
			if (scene->GetGroundDiffractionPath(curTransmitter->m_position, curReceiver->m_position, terrainDifftactionPath)) {
				if (scene->IsValidRayPath(terrainDifftactionPath))
					result.m_raytracingResult[offset].SetRayPath(terrainDifftactionPath);
				else
					delete terrainDifftactionPath;
			}
		}

		for (auto& paths : commonPaths) {
			for (auto& path : paths) {
				delete path;
				path = nullptr;
			}
			paths.clear();
			std::vector<RayPathGPU*>().swap(paths);
		}
		std::vector< std::vector < RayPathGPU*>>().swap(commonPaths);


	}
}

void PathBuilder_DEBUG(const std::vector<RayTreeNode*>& vroots, const Scene* scene) {
	size_t txNum = scene->m_transmitters.size();
	size_t rxNum = scene->m_receivers.size();
	for (size_t i = 0; i < txNum; ++i) {
		LOG_INFO << "PathBuilder: starting tx-" << i << "path builder." << ENDL;
		const Transmitter* curTransmitter = scene->m_transmitters[i];		/** @brief	当前遍历到的发射机	*/
		if (!curTransmitter->m_isValid) {									//跳过无效的发射机
			continue;
		}
		std::vector<RayPath*> commonPath2D;
		GenerateMultiPath(vroots[i], commonPath2D);
		std::string filename = "fullmultipath";
		filename.append(std::to_string(i));
		filename.append(".txt");
		std::ofstream stream(filename);
		if (stream.is_open()) {
			for (auto it = commonPath2D.begin(); it != commonPath2D.end(); ++it) {
				const RayPath* path = *it;
				path->Write2File(stream);
			}
			stream.close();
		}

	}
}

void GenerateMultiPathofRxSingleTxGPU(const std::vector<PathNodeGPU*>& nodes, const Point2D& txPosition, const std::vector<Point2D>& rxPositions, const Segment2DGPU* segments, const Scene* scene, std::vector<std::vector<RayPathGPU*>>& outRayPath) {
	PathNodeGPU* rootNode = new PathNodeGPU();//构建根节点
	rootNode->m_inter.m_intersect = txPosition;
	rootNode->m_inter.m_type = NODE_ROOT;
	std::vector<PathNodeGPU*> newNodes;
	int curRxId = -1;//初始化rx编号
	bool containReftact = false;
	for (int i = 0; i < nodes.size(); ++i) {
		newNodes.push_back(nodes[i]);
		if (nodes[i]->m_inter.m_type == NODE_TRANIN ||
			nodes[i]->m_inter.m_type == NODE_TRANOUT ||
			nodes[i]->m_inter.m_type == NODE_ETRANIN ||
			nodes[i]->m_inter.m_type == NODE_ETRANOUT) {
			containReftact = true;
		}

		if (nodes[i]->m_layer == 0) {//寻找到新增路径
			curRxId = nodes[i]->m_rxId;
			newNodes.push_back(rootNode);
			RayPathGPU* newPath = new RayPathGPU(newNodes, containReftact);
			if (RectifyGPURayPath(scene, *newPath, rxPositions[curRxId])) {
				outRayPath[curRxId].push_back(newPath);
			}
			newNodes.clear();//重置数据
			containReftact = false;
		}
	}
}

void GenerateMultiPathofRxGPU(std::vector<std::vector<PathNodeGPU*>> nodes, std::vector<Point2D> txPositions, std::vector<Point2D> rxPositions, Segment2DGPU* segments, const Scene* scene, std::vector<std::vector<std::vector<RayPathGPU*>>>& outRayPaths) {
	//基于输入的nodes，将有效的raypath提取出来
	//将提取出的数据写入至outRayPath中并进行校正处理
	for (int i = 0; i < nodes.size(); ++i) {//第一层循环为每个tx, outRayPaths中第一层为tx数量，outRayPaths第二层为rx数量，outRayPaths第三层为多径数量
		GenerateMultiPathofRxSingleTxGPU(nodes[i], txPositions[i], rxPositions, segments, scene, outRayPaths[i]);
	}
	////将多径文件写入到文件
	//for (int i = 0; i < txPositions.size(); ++i) {
	//	for (int j = 0; j < rxPositions.size(); ++j) {
	//		std::stringstream filename;
	//		filename << "path-tx" << i + 1 << "-rx" << j + 1 << ".txt";
	//		std::ofstream outFile(filename.str());
	//		if (outFile.is_open()) {
	//			for (int k = 0; k < outRayPaths[i][j].size(); ++k) {
	//				RayPathGPU& newPath = outRayPaths[i][j][k];
	//				outFile << newPath.m_nodes.size() << ",";
	//				for (int m = static_cast<int>(newPath.m_nodes.size()) - 1; m >= 0; --m) {
	//					Intersection2DGPU& inter = newPath.m_nodes[m].m_inter;
	//					outFile << inter.m_intersect.x << "," << inter.m_intersect.y << ",";
	//				}
	//				outFile << std::endl;
	//			}
	//			outFile.close();
	//		}
	//	}
	//}
}
