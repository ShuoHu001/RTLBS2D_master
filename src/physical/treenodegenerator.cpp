#include "treenodegenerator.h"

void TreeNodeGenerator_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, Result& result)
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

void TreeNodeGenerator_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, int16_t threadNum, Result& result)
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
