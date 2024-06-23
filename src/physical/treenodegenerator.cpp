#include "treenodegenerator.h"
#include "tree/gpu/cpuconverterpathnode.h"
#include "result/result.h"

void TreeNodeGenerator_AOA_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, Result& result)
{
	size_t sensorNum = scene->m_sensors.size();
	for (size_t i = 0; i < sensorNum; ++i) {
		const Sensor* curSensor = scene->m_sensors[i];
		if (!curSensor->m_isValid) {								//跳过无效传感器
			continue;
		}

		std::vector<PathNode*> nodes;
		GenerateAllTreeNode(vroots[i], nodes);
		std::vector<LBSTreeNode*> lbsNodes(nodes.size());
		for (int j = 0; j < nodes.size(); ++j) {
			const PathNode* curPathNode = nodes[j];
			int sensorDataId = curPathNode->m_nextRay.m_sensorDataId;
			SensorData* sensorData = scene->m_sensorDataLibrary.GetData(sensorDataId);
			lbsNodes[j] = new LBSTreeNode(*curPathNode, sensorData);
		}

		//添加根节点,每个数据都是一个节点
		std::vector<SensorData>& sensorDatas = scene->m_sensors[i]->m_sensorDataCollection.m_data;
		for (auto it = sensorDatas.begin(); it != sensorDatas.end(); ++it) {
			int sensorDataId = (*it).m_id;
			SensorData* sensorData = scene->m_sensorDataLibrary.GetData(sensorDataId);
			lbsNodes.push_back(new LBSTreeNode(vroots[i]->m_data, sensorData));
		}

		result.m_lbsGSResult[i].SetNodes(lbsNodes);
	}
}


void TreeNodeGenerator_TDOA_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, Result& result)
{
	size_t sensorNum = scene->m_sensors.size();
	for (size_t i = 0; i < sensorNum; ++i) {
		const Sensor* curSensor = scene->m_sensors[i];
		if (!curSensor->m_isValid) {								//跳过无效传感器
			continue;
		}

		std::vector<PathNode*> nodes;
		GenerateAllTreeNode(vroots[i], nodes);
		std::vector<LBSTreeNode*> lbsNodes(nodes.size());
		for (int j = 0; j < nodes.size(); ++j) {
			const PathNode* curPathNode = nodes[j];
			lbsNodes[j] = new LBSTreeNode(*curPathNode);
		}

		result.m_lbsGSResult[i].SetNodes(lbsNodes);
	}
}

void TreeNodeGenerator_AOA_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, int16_t threadNum, Result& result)
{
	size_t sensorNum = scene->m_sensors.size();
	if (sensorNum < threadNum) {
		threadNum = static_cast<int16_t>(sensorNum);			//若传感器数量小于线程数量，则按照传感器数量为线程数量
	}

	ThreadPool pool(threadNum);
	std::vector<std::vector<PathNode*>> nodes;
	for (int i = 0; i < sensorNum; ++i) {
		auto future = pool.enqueue(GenerateAllTreeNode, vroots[i], nodes[i]);
	}
	while (true) {
		if (pool.getTaskCount() == 0) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 每100毫秒检查一次
	}
	for (int i = 0; i < sensorNum; ++i) {
		std::vector<LBSTreeNode*> lbsNodes(nodes[i].size());
		for (int j = 0; j < nodes.size(); ++j) {
			const PathNode* curPathNode = nodes[i][j];
			int sensorDataId = curPathNode->m_nextRay.m_sensorDataId;
			SensorData* sensorData = scene->m_sensorDataLibrary.GetData(sensorDataId);
			lbsNodes[j] = new LBSTreeNode(*curPathNode, sensorData);
		}

		//添加根节点,每个数据都是一个节点
		std::vector<SensorData>& sensorDatas = scene->m_sensors[i]->m_sensorDataCollection.m_data;
		for (auto it = sensorDatas.begin(); it != sensorDatas.end(); ++it) {
			int sensorDataId = (*it).m_id;
			SensorData* sensorData = scene->m_sensorDataLibrary.GetData(sensorDataId);
			lbsNodes.push_back(new LBSTreeNode(vroots[i]->m_data, sensorData));
		}

		result.m_lbsGSResult[i].SetNodes(lbsNodes);
	}
}

void TreeNodeGenerator_TDOA_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, int16_t threadNum, Result& result)
{
	size_t sensorNum = scene->m_sensors.size();
	if (sensorNum < threadNum) {
		threadNum = static_cast<int16_t>(sensorNum);			//若传感器数量小于线程数量，则按照传感器数量为线程数量
	}

	ThreadPool pool(threadNum);
	std::vector<std::vector<PathNode*>> nodes;
	for (int i = 0; i < sensorNum; ++i) {
		auto future = pool.enqueue(GenerateAllTreeNode, vroots[i], nodes[i]);
	}
	while (true) {
		if (pool.getTaskCount() == 0) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 每100毫秒检查一次
	}
	for (int i = 0; i < sensorNum; ++i) {
		std::vector<LBSTreeNode*> lbsNodes(nodes[i].size());
		for (int j = 0; j < nodes.size(); ++j) {
			const PathNode* curPathNode = nodes[i][j];
			int sensorDataId = curPathNode->m_nextRay.m_sensorDataId;
			SensorData* sensorData = scene->m_sensorDataLibrary.GetData(sensorDataId);
			lbsNodes[j] = new LBSTreeNode(*curPathNode, sensorData);
		}

		//添加根节点,每个数据都是一个节点
		std::vector<SensorData>& sensorDatas = scene->m_sensors[i]->m_sensorDataCollection.m_data;
		for (auto it = sensorDatas.begin(); it != sensorDatas.end(); ++it) {
			int sensorDataId = (*it).m_id;
			SensorData* sensorData = scene->m_sensorDataLibrary.GetData(sensorDataId);
			lbsNodes.push_back(new LBSTreeNode(vroots[i]->m_data, sensorData));
		}

		result.m_lbsGSResult[i].SetNodes(lbsNodes);
	}
}



void TreeNodeGenerator_GPUMultiThread(const std::vector<std::vector<TreeNodeGPU*>>& gpuTreeNodes, const Scene* scene, Result& result)//未完成修改
{
	for (int i = 0; i < gpuTreeNodes.size(); ++i) {
		std::vector<LBSTreeNode*> lbsNodes(gpuTreeNodes[i].size());
		for (int j = 0; j < gpuTreeNodes[i].size(); ++j) {
			const TreeNodeGPU* curTreeNodeGPU = gpuTreeNodes[i][j];
			int segmentId = curTreeNodeGPU->m_segmentId;
			int wedgeId = curTreeNodeGPU->m_wedgeId;
			Segment2D* segment = nullptr;
			Wedge2D* wedge = nullptr;
			if (segmentId != -1) {
				segment = scene->m_segmentBuf[segmentId];
			}
			if (wedgeId != -1) {
				wedge = scene->m_wedgeBuf[wedgeId];
			}
			int sensorDataId = curTreeNodeGPU->m_nextRay.m_sensorDataId;
			SensorData* sensorData = scene->m_sensorDataLibrary.GetData(sensorDataId);
			lbsNodes[j] = new LBSTreeNode(*gpuTreeNodes[i][j], segment, wedge, sensorData);
		}
		result.m_lbsGSResult[i].SetNodes(lbsNodes);
	}
}

void GenerateAllTreeNodeAndConvertToCPUConvertPathNode(const std::vector<RayTreeNode*>& roots, std::vector<CPUConverterPathNode>& outNodes, int& maxDepth)
{
	struct StackItem {
		RayTreeNode* cur_node;					/** @brief	当前节点	*/
		int father_nodeId;						/** @brief	当前节点父节点在数组中的ID	*/
		int layerId;							/** @brief	当前节点所在的层ID	*/
	};
	std::stack<StackItem> stack;

	//遍历每个树根，将树节点元素转换为CPUConverterPathNode
	for (int sensorId = 0; sensorId < static_cast<int>(roots.size()); ++sensorId) {
		RayTreeNode* curRoot = roots[sensorId];

		if (curRoot == nullptr) {				//跳过无效根节点
			continue;
		}
		//确定虚拟根节点的数量
		int vrootNum = 0;
		RayTreeNode* tempNode = curRoot;		/** @brief	临时节点交换指针	*/	
		while (tempNode != nullptr) {
			tempNode = tempNode->m_pRight;
			vrootNum++;
		}

		tempNode = curRoot;
		for (int i = 0; i < vrootNum; ++i) {
			if (!tempNode->m_isValid) {			//跳过无效节点
				tempNode = tempNode->m_pRight;
				continue;
			}
			stack.push({ tempNode, -1, 0 });
			tempNode = tempNode->m_pRight;
		}

		while (!stack.empty()) {
			StackItem curItem = stack.top();
			stack.pop();

			RayTreeNode* curNode = curItem.cur_node;
			int cur_faterNodeId = curItem.father_nodeId;
			int cur_layerId = curItem.layerId;
			if (maxDepth < cur_layerId) {											//寻找最大深度
				maxDepth = cur_layerId;
			}

			CPUConverterPathNode newNode(curNode->m_data, cur_faterNodeId, sensorId, cur_layerId);
			outNodes.push_back(newNode);

			if (curNode->m_pLeft) {
				int next_fatherNodeId = static_cast<int>(outNodes.size() - 1);			/** @brief	下一节点的父节点ID	*/
				int next_layerId = cur_layerId + 1;									/** @brief	下一节点的层 ID	*/
				RayTreeNode* childNode = curNode->m_pLeft;
				while (childNode) {
					if (!childNode->m_isValid) {					//跳过无效节点
						childNode = childNode->m_pRight;
						continue;
					}
					stack.push({ childNode, next_fatherNodeId, next_layerId });
					childNode = childNode->m_pRight;
				}
			}
		}
	}
}
