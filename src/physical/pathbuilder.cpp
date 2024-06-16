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
			std::vector<RayPath3D*> invalidCommonPath;																				/** @brief	无效的常规路径集合	*/
			std::vector<RayPath3D*> validCommonPath;																				/** @brief	有效的常规路径集合	*/
			for (auto it = commonPath3D.begin(); it != commonPath3D.end(); ++it) {													//筛选路径
				RayPath3D* curPath = *it;
				if (!scene->IsValidRayPath(curPath))																				//路径无效
					invalidCommonPath.push_back(curPath);
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
			for (auto it = commonPath2D.begin(); it != commonPath2D.end(); ++it)													//释放二维路径内存
				delete* it;
			for (auto it = invalidCommonPath.begin(); it != invalidCommonPath.end(); ++it)											//释放无效的三维路径内存
				delete* it;
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
			std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 每100毫秒检查一次
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
		}
	}
}

void PathBuilder_GPUMultiThread(const std::vector<std::vector<PathNodeGPU*>>& gpuPathNodes, const Scene* scene, Result& result)
{
	size_t txNum = scene->m_transmitters.size();
	size_t rxNum = scene->m_receivers.size();
	for (size_t i = 0; i < txNum; ++i) {
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

void DirectlySetResultPath_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, const Point2D& targetPoint, std::vector<RaytracingResult>& rtResult)
{
	//首先产生从每个传感器射线树到目标坐标点p的路径集合
	//将路径放入对应的rtResult中并标记为逆向路径
	int sensorNum = static_cast<int>(vroots.size());														/** @brief	传感器数量	*/
	RtLbsType sensorHeight = scene->m_sensors[0]->m_position.z;												/** @brief	传感器所在高度，二维情况下所有传感器均在一个平面上	*/
	Point3D targetPoint3D(targetPoint.x, targetPoint.y, sensorHeight);										/** @brief	三维目标坐标	*/

	rtResult.resize(sensorNum);																				//需要输出的射线追踪结果
	for (int i = 0; i < sensorNum; ++i) {
		RaytracingResult& curRTResult = rtResult[i];														/** @brief	当前需要填充的射线追踪结果	*/
		const Sensor* curSensor = scene->m_sensors[i];
		std::vector<RayPath*> commonPath2D;
		GenerateMultipathofPoint(vroots[i], targetPoint, scene, splitRadius, commonPath2D);

		//获得二维路径后需要转换为三维路径
		//将常规路径转换为3D路径
		std::vector<RayPath3D*> commonPath3D(commonPath2D.size());											//常规3D路径分配内存（包含常规二维路径转换+地形反射路径）
		for (int i = 0; i < commonPath2D.size(); ++i) {														//转换常规二维路径
			commonPath3D[i] = new RayPath3D(*commonPath2D[i], curSensor->m_position, targetPoint3D);		//将常规路径转换为三维路径
			commonPath3D[i]->ReverseRayPath();																//由于传感器和目标源之间的路径的可逆性，因此对路径进行逆向处理
		}

		std::vector<RayPath3D*> terrainReflectPaths;														/** @brief	地面反射路径(逆转型的路径，从目标传播到传感器)	*/
		if (scene->GetGroundReflectPaths(targetPoint3D, curSensor->m_position, terrainReflectPaths)) {
			for (auto it = terrainReflectPaths.begin(); it != terrainReflectPaths.end(); ++it) {			//若产生了地面反射路径，则将地面反射路径纳入进常规路径中
				commonPath3D.push_back(*it);
			}
		}


		//删除无效的路径
		std::vector<RayPath3D*> validPath, inValidPath;														/** @brief	有效路径和无效路径	*/
		for (auto it = commonPath3D.begin(); it != commonPath3D.end(); ++it) {
			RayPath3D* curPath = *it;
			if (!scene->IsValidRayPath(curPath)) {															//路径验证不通过，无效路径
				inValidPath.push_back(curPath);
			}
			else {
				validPath.push_back(curPath);
			}
		}

		curRTResult.SetRayPath(validPath);

		//处理地形绕射路径
		TerrainDiffractionPath* terrainDifftactionPath = nullptr;
		if (scene->GetGroundDiffractionPath(targetPoint3D, curSensor->m_position, terrainDifftactionPath)) {
			if (scene->IsValidRayPath(terrainDifftactionPath))
				curRTResult.SetRayPath(terrainDifftactionPath);												//设定地形绕射路径
			else																							//若地形绕射路径无效，则进行删除
				delete terrainDifftactionPath;
		}

		//删除路径计算中的缓存
		for (auto it = commonPath2D.begin(); it != commonPath2D.end(); ++it) {
			delete* it;
		}

		for (auto it = inValidPath.begin(); it != inValidPath.end(); ++it) {
			delete* it;
		}

	}
}
