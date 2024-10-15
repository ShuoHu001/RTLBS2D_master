#include "raypathsetter.h"

void DirectlySetResultPath_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, GSPairCluster* cluster)
{
	//首先产生从每个传感器射线树到目标坐标点p的路径集合
	//将路径放入对应的rtResult中并标记为逆向路径
	int sensorNum = static_cast<int>(vroots.size());														/** @brief	传感器数量	*/
	RtLbsType sensorHeight = scene->m_sensors[0]->m_position.z;												/** @brief	传感器所在高度，二维情况下所有传感器均在一个平面上	*/
	for (int j = 0; j < cluster->m_aroundPoints.size(); ++j) {
		const Point2D& targetPoint = cluster->m_aroundPoints[j];
		Point3D targetPoint3D(cluster->m_aroundPoints[j].x, cluster->m_aroundPoints[j].y, sensorHeight);										/** @brief	三维目标坐标	*/
		std::vector<RaytracingResult>& rtResult = cluster->m_rtResult[j];
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
}

void DirectlySetResultPath_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, uint16_t threadNum, std::vector<GSPairCluster>& clusters)
{
	ThreadPool pool(threadNum);
	for (int i = 0; i < static_cast<int>(clusters.size()); ++i) {
		auto future = pool.enqueue(DirectlySetResultPath_CPUSingleThread, vroots, scene, splitRadius, &clusters[i]);
	}

	while (true) {
		if (pool.getTaskCount() == 0) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));						//每100ms检查一次
		std::cout << pool.getTaskCount() << std::endl;
	}
}