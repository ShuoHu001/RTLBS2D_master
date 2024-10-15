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
			std::cout << j << std::endl;
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
			std::vector<RayPath3D*> validCommonPath;																				/** @brief	��Ч�ĳ���·������	*/
			for (auto it = commonPath3D.begin(); it != commonPath3D.end(); ++it) {													//ɸѡ·��
				RayPath3D*& curPath = *it;
				if (!scene->IsValidRayPath(curPath)) {																				//·����Ч
					delete curPath;
					curPath = nullptr;
				}
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
			for (auto& path : commonPath2D) {													//�ͷŶ�ά·���ڴ�
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
			std::this_thread::sleep_for(std::chrono::milliseconds(10000)); // ÿ100������һ��
			std::cout << pool.getTaskCount() << std::endl;
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

void GenerateMultiPathofRxSingleTxGPU(const std::vector<PathNodeGPU*>& nodes, const Point2D& txPosition, const std::vector<Point2D>& rxPositions, const Segment2DGPU* segments, const Scene* scene, std::vector<std::vector<RayPathGPU*>>& outRayPath) {
	PathNodeGPU* rootNode = new PathNodeGPU();//�������ڵ�
	rootNode->m_inter.m_intersect = txPosition;
	rootNode->m_inter.m_type = NODE_ROOT;
	std::vector<PathNodeGPU*> newNodes;
	int curRxId = -1;//��ʼ��rx���
	bool containReftact = false;
	for (int i = 0; i < nodes.size(); ++i) {
		newNodes.push_back(nodes[i]);
		if (nodes[i]->m_inter.m_type == NODE_TRANIN ||
			nodes[i]->m_inter.m_type == NODE_TRANOUT ||
			nodes[i]->m_inter.m_type == NODE_ETRANIN ||
			nodes[i]->m_inter.m_type == NODE_ETRANOUT) {
			containReftact = true;
		}

		if (nodes[i]->m_layer == 0) {//Ѱ�ҵ�����·��
			curRxId = nodes[i]->m_rxId;
			newNodes.push_back(rootNode);
			RayPathGPU* newPath = new RayPathGPU(newNodes, containReftact);
			if (RectifyGPURayPath(scene, *newPath, rxPositions[curRxId])) {
				outRayPath[curRxId].push_back(newPath);
			}
			newNodes.clear();//��������
			containReftact = false;
		}
	}
}

void GenerateMultiPathofRxGPU(std::vector<std::vector<PathNodeGPU*>> nodes, std::vector<Point2D> txPositions, std::vector<Point2D> rxPositions, Segment2DGPU* segments, const Scene* scene, std::vector<std::vector<std::vector<RayPathGPU*>>>& outRayPaths) {
	//���������nodes������Ч��raypath��ȡ����
	//����ȡ��������д����outRayPath�в�����У������
	for (int i = 0; i < nodes.size(); ++i) {//��һ��ѭ��Ϊÿ��tx, outRayPaths�е�һ��Ϊtx������outRayPaths�ڶ���Ϊrx������outRayPaths������Ϊ�ྶ����
		GenerateMultiPathofRxSingleTxGPU(nodes[i], txPositions[i], rxPositions, segments, scene, outRayPaths[i]);
	}
	////���ྶ�ļ�д�뵽�ļ�
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
