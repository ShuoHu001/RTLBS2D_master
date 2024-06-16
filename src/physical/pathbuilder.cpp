#include "pathbuilder.h"
#include "result/result.h"
#include "result/raytracingresult.h"

void PathBuilder_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, Result& result)
{
	size_t txNum = scene->m_transmitters.size();
	size_t rxNum = scene->m_receivers.size();
	for (size_t i = 0; i < txNum; ++i) {			//����ÿ����������
		LOG_INFO << "PathBuilder: starting tx-" << i << "path builder." << ENDL;
		const Transmitter* curTransmitter = scene->m_transmitters[i];		/** @brief	��ǰ�������ķ����	*/
		if (!curTransmitter->m_isValid) {									//������Ч�ķ����
			continue;
		}
		for (size_t j = 0; j < rxNum; ++j) {
			const Receiver* curReceiver = scene->m_receivers[j];			/** @brief	��ǰ�������Ľ��ջ�	*/
			if (!curReceiver->m_isValid) {									//������Ч�Ľ��ջ�
				continue;
			}
			size_t offset = i * rxNum + j;																							//
			//-----------------------------------------------------����·���趨-------------------------------------------------------
			std::vector<RayPath*> commonPath2D;
			GenerateMultipathofPoint(vroots[i], curReceiver->GetPosition2D(), scene, splitRadius, commonPath2D);

			//������·��ת��Ϊ3D·��
			std::vector<RayPath3D*> commonPath3D(commonPath2D.size());																//����3D·�������ڴ棨���������ά·��ת��+���η���·����
			for (int i = 0; i < commonPath2D.size(); ++i) {																			//ת�������ά·��
				commonPath3D[i] = new RayPath3D(*commonPath2D[i], curTransmitter->m_position, curReceiver->m_position);
			}

			std::vector<RayPath3D*> terrainReflectPaths;																			/** @brief	���淴��·��	*/
			if (scene->GetGroundReflectPaths(curTransmitter->m_position, curReceiver->m_position, terrainReflectPaths)) {
				for (auto it = terrainReflectPaths.begin(); it != terrainReflectPaths.end(); ++it) {								//�������˵��淴��·�����򽫵��淴��·�����������·����
					commonPath3D.push_back(*it);
				}
			}

			//���鲢�趨����·��
			std::vector<RayPath3D*> invalidCommonPath;																				/** @brief	��Ч�ĳ���·������	*/
			std::vector<RayPath3D*> validCommonPath;																				/** @brief	��Ч�ĳ���·������	*/
			for (auto it = commonPath3D.begin(); it != commonPath3D.end(); ++it) {													//ɸѡ·��
				RayPath3D* curPath = *it;
				if (!scene->IsValidRayPath(curPath))																				//·����Ч
					invalidCommonPath.push_back(curPath);
				else																												//·����Ч
					validCommonPath.push_back(curPath);
			}
			result.m_raytracingResult[offset].SetRayPath(validCommonPath);														//�趨������ά·��


			//------------------------------------------------------------��������·���趨-----------------------------------------------------------------
			TerrainDiffractionPath* terrainDifftactionPath = nullptr;																/** @brief	��������·��	*/
			if (scene->GetGroundDiffractionPath(curTransmitter->m_position, curReceiver->m_position, terrainDifftactionPath)) {
				if (scene->IsValidRayPath(terrainDifftactionPath))
					result.m_raytracingResult[offset].SetRayPath(terrainDifftactionPath);											//�趨��������·��
				else																												//����������·����Ч�������ɾ��
					delete terrainDifftactionPath;
			}

			//------------------------------------------------------------�ͷ��ڴ�------------------------------------------------------------------------
			for (auto it = commonPath2D.begin(); it != commonPath2D.end(); ++it)													//�ͷŶ�ά·���ڴ�
				delete* it;
			for (auto it = invalidCommonPath.begin(); it != invalidCommonPath.end(); ++it)											//�ͷ���Ч����ά·���ڴ�
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
		const Transmitter* curTransmitter = scene->m_transmitters[i];		/** @brief	��ǰ�������ķ����	*/
		if (!curTransmitter->m_isValid) {									//������Ч�ķ����
			continue;
		}

		//--------------------------------------------------------------------CPU���д���·���ռ�---------------------------------------------------------------
		ThreadPool pool(threadNum);
		std::vector<std::vector<RayPath*>> commonPaths(scene->m_receivers.size(), std::vector<RayPath*>());						/** @brief	���н��յ�Ķ�ά·������	*/
		for (size_t j = 0; j < scene->m_receivers.size(); ++j) {																//������ѹ���̳߳���
			auto future = pool.enqueue(GenerateMultipathofPoint, vroots[i], scene->m_receivers[j]->GetPosition2D(), scene, splitRadius, commonPaths[j]);
		}
		while (true) {
			if (pool.getTaskCount() == 0) {
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100)); // ÿ100������һ��
		}

		//-----------------------------------------------------------------����·��ת�����ж��Ƿ���Ч-----------------------------------------------------------
		for (int j = 0; j < rxNum; ++j) {
			Receiver* curReceiver = scene->m_receivers[i];																		/** @brief	��ǰ�Ľ��ջ�	*/
			size_t offset = i * rxNum + j;												/** @brief	�趨����ڽ�������е�ƫ����	*/
			std::vector<RayPath3D*> commanPath3D;																					/** @brief	ÿ�����ջ������յ�����ά·��	*/
			commanPath3D.reserve(commonPaths[j].size() + 2);																			//Ԥ����С
			int validCommonPathId = 0;
			for (size_t k = 0; k < commonPaths[j].size(); ++k) {																		//����·������3D·��ת��
				RayPath3D* newPath = new RayPath3D(*commonPaths[j][k], curTransmitter->m_position, curReceiver->m_position);			//ת��·��
				if (scene->IsValidRayPath(newPath)) {																				//��·����Ч����������·��������
					commanPath3D[validCommonPathId++] = newPath;
				}
				else {																												//��·����Ч��������ڴ�ɾ��
					delete newPath;
				}
			}

			std::vector<RayPath3D*> groundReflectionPath;																			//���淴��·��
			if (scene->GetGroundReflectPaths(curTransmitter->m_position, curReceiver->m_position, groundReflectionPath)) {
				for (auto it = groundReflectionPath.begin(); it != groundReflectionPath.end(); ++it) {
					if (scene->IsValidRayPath(*it))																				//��Ч��������Чɾ��
						commanPath3D[validCommonPathId++] = *it;
					else
						delete* it;
				}
			}

			result.m_raytracingResult[offset].SetRayPath(commanPath3D);															//�趨����·��


			//��������·��
			TerrainDiffractionPath* terrainDifftactionPath = nullptr;																/** @brief	��������·��	*/
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
		const Transmitter* curTransmitter = scene->m_transmitters[i];		/** @brief	��ǰ�������ķ����	*/
		if (!curTransmitter->m_isValid) {									//������Ч�ķ����
			continue;
		}


		//��ȡGPU��������������ྶ��Ϣ
		std::vector<std::vector<RayPathGPU*>> commonPaths;																			/** @brief	����GPU·������һ��Ϊrx�������ڶ���Ϊ�ྶ����	*/
		commonPaths.resize(rxNum);
		Point2D txPosition2D(curTransmitter->m_position.x, curTransmitter->m_position.y);													/** @brief	��������꣨��ά��	*/
		std::vector<Point2D> rxPositons2D(scene->m_receivers.size());																/** @brief	���ջ����꣨��ά��	*/
		for (size_t i = 0; i < scene->m_receivers.size(); ++i) {
			rxPositons2D[i].x = scene->m_receivers[i]->m_position.x;
			rxPositons2D[i].y = scene->m_receivers[i]->m_position.y;
		}
		GenerateMultiPathofRxSingleTxGPU(gpuPathNodes[i], txPosition2D, rxPositons2D, scene->m_gpuSegmentBuf.data(), scene, commonPaths);		//����GPU�ڵ�������ྶ��Ϣ

		//��GPU�ྶת��ΪCPU�е���ά�ྶ��Ϣ
		for (int j = 0; j < rxNum; ++j) {																		//����ÿ��rx��ת����ɸѡ·��
			Receiver* curReceiver = scene->m_receivers[j];																		/** @brief	��ǰ�Ľ��ջ�	*/
			int offset = i * rxNum + j;												/** @brief	�趨����ڽ�������е�ƫ����	*/
			std::vector<RayPath3D*> commanPath3D;																					/** @brief	ÿ�����ջ������յ�����ά·��	*/
			commanPath3D.resize(commonPaths[j].size() + 2);																			//Ԥ����С
			int validCommonPathId = 0;
			for (int k = 0; k < commonPaths[j].size(); ++k) {
				RayPath3D* newPath = new RayPath3D(*commonPaths[j][k], curTransmitter->m_position, curReceiver->m_position, scene->m_segmentBuf, scene->m_wedgeBuf);
				if (scene->IsValidRayPath(newPath)) {																				//��·����Ч����������·��������
					commanPath3D[validCommonPathId++] = newPath;
				}
				else {																												//��·����Ч��������ڴ�ɾ��
					delete newPath;
				}
			}


			std::vector<RayPath3D*> groundReflectionPath;																			//���淴��·��
			if (scene->GetGroundReflectPaths(curTransmitter->m_position, curReceiver->m_position, groundReflectionPath)) {
				for (auto it = groundReflectionPath.begin(); it != groundReflectionPath.end(); ++it) {
					if (scene->IsValidRayPath(*it))																				//��Ч��������Чɾ��
						commanPath3D[validCommonPathId++] = *it;
					else
						delete* it;
				}
			}
			commanPath3D.resize(validCommonPathId);
			result.m_raytracingResult[offset].SetRayPath(commanPath3D);															//�趨����·��


			//��������·��
			TerrainDiffractionPath* terrainDifftactionPath = nullptr;																/** @brief	��������·��	*/
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
		const Transmitter* curTransmitter = scene->m_transmitters[i];		/** @brief	��ǰ�������ķ����	*/
		if (!curTransmitter->m_isValid) {									//������Ч�ķ����
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
	//���Ȳ�����ÿ����������������Ŀ�������p��·������
	//��·�������Ӧ��rtResult�в����Ϊ����·��
	int sensorNum = static_cast<int>(vroots.size());														/** @brief	����������	*/
	RtLbsType sensorHeight = scene->m_sensors[0]->m_position.z;												/** @brief	���������ڸ߶ȣ���ά��������д���������һ��ƽ����	*/
	Point3D targetPoint3D(targetPoint.x, targetPoint.y, sensorHeight);										/** @brief	��άĿ������	*/

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
