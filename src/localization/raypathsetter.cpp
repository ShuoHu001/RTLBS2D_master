#include "raypathsetter.h"

void DirectlySetResultPath_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, GSPairCluster* cluster)
{
	//���Ȳ�����ÿ����������������Ŀ�������p��·������
	//��·�������Ӧ��rtResult�в����Ϊ����·��
	int sensorNum = static_cast<int>(vroots.size());														/** @brief	����������	*/
	RtLbsType sensorHeight = scene->m_sensors[0]->m_position.z;												/** @brief	���������ڸ߶ȣ���ά��������д���������һ��ƽ����	*/
	for (int j = 0; j < cluster->m_aroundPoints.size(); ++j) {
		const Point2D& targetPoint = cluster->m_aroundPoints[j];
		Point3D targetPoint3D(cluster->m_aroundPoints[j].x, cluster->m_aroundPoints[j].y, sensorHeight);										/** @brief	��άĿ������	*/
		std::vector<RaytracingResult>& rtResult = cluster->m_rtResult[j];
		rtResult.resize(sensorNum);																				//��Ҫ���������׷�ٽ��
		for (int i = 0; i < sensorNum; ++i) {
			RaytracingResult& curRTResult = rtResult[i];														/** @brief	��ǰ��Ҫ��������׷�ٽ��	*/
			const Sensor* curSensor = scene->m_sensors[i];
			std::vector<RayPath*> commonPath2D;
			GenerateMultipathofPoint(vroots[i], targetPoint, scene, splitRadius, commonPath2D);

			//��ö�ά·������Ҫת��Ϊ��ά·��
			//������·��ת��Ϊ3D·��
			std::vector<RayPath3D*> commonPath3D(commonPath2D.size());											//����3D·�������ڴ棨���������ά·��ת��+���η���·����
			for (int i = 0; i < commonPath2D.size(); ++i) {														//ת�������ά·��
				commonPath3D[i] = new RayPath3D(*commonPath2D[i], curSensor->m_position, targetPoint3D);		//������·��ת��Ϊ��ά·��
				commonPath3D[i]->ReverseRayPath();																//���ڴ�������Ŀ��Դ֮���·���Ŀ����ԣ���˶�·������������
			}

			std::vector<RayPath3D*> terrainReflectPaths;														/** @brief	���淴��·��(��ת�͵�·������Ŀ�괫����������)	*/
			if (scene->GetGroundReflectPaths(targetPoint3D, curSensor->m_position, terrainReflectPaths)) {
				for (auto it = terrainReflectPaths.begin(); it != terrainReflectPaths.end(); ++it) {			//�������˵��淴��·�����򽫵��淴��·�����������·����
					commonPath3D.push_back(*it);
				}
			}


			//ɾ����Ч��·��
			std::vector<RayPath3D*> validPath, inValidPath;														/** @brief	��Ч·������Ч·��	*/
			for (auto it = commonPath3D.begin(); it != commonPath3D.end(); ++it) {
				RayPath3D* curPath = *it;
				if (!scene->IsValidRayPath(curPath)) {															//·����֤��ͨ������Ч·��
					inValidPath.push_back(curPath);
				}
				else {
					validPath.push_back(curPath);
				}
			}

			curRTResult.SetRayPath(validPath);

			//�����������·��
			TerrainDiffractionPath* terrainDifftactionPath = nullptr;
			if (scene->GetGroundDiffractionPath(targetPoint3D, curSensor->m_position, terrainDifftactionPath)) {
				if (scene->IsValidRayPath(terrainDifftactionPath))
					curRTResult.SetRayPath(terrainDifftactionPath);												//�趨��������·��
				else																							//����������·����Ч�������ɾ��
					delete terrainDifftactionPath;
			}

			//ɾ��·�������еĻ���
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
		std::this_thread::sleep_for(std::chrono::milliseconds(50));						//ÿ100ms���һ��
		std::cout << pool.getTaskCount() << std::endl;
	}
}