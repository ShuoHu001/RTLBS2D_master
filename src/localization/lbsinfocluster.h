#ifndef RTLBS_LBSINFOCLUSTER
#define RTLBS_LBSINFOCLUSTER

#include "rtlbs.h"
#include "utility/define.h"
#include "lbsinfo.h"
#include "equipment/sensor/sensor.h"
#include "core/render/pathtracing/treenodegenerator.h"

class LBSInfoCluster {
public:
	std::vector<LBSInfo*> m_infos;

public:
	LBSInfoCluster();
	LBSInfoCluster(const std::vector<Sensor*>& sensors);
	~LBSInfoCluster();
	void Init(const std::vector<Sensor*>& sensors);
};

//AOA�����������߽ṹ���ڵ㡪������CPU���� ������AOA��AOA-TDOA�㷨
inline void TreeNodeGenerator_AOA_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, LBSInfoCluster& infoCluster)
{
	size_t sensorNum = scene->m_sensors.size();
	for (size_t i = 0; i < sensorNum; ++i) {
		const Sensor* curSensor = scene->m_sensors[i];
		if (!curSensor->m_isValid) {								//������Ч������
			continue;
		}

		std::vector<PathNode*> nodes;
		GenerateAllPathNode(vroots[i], nodes);
		std::vector<LBSTreeNode*> lbsNodes(nodes.size());
		for (int j = 0; j < nodes.size(); ++j) {
			const PathNode* curPathNode = nodes[j];
			int sensorDataId = curPathNode->m_nextRay.m_sensorDataId;
			SensorData* sensorData = scene->m_sensorDataLibrary.GetData(sensorDataId);
			lbsNodes[j] = new LBSTreeNode(*curPathNode, sensorData);
		}
		for (auto& node : nodes) {
			delete node;
		}

		infoCluster.m_infos[i]->SetNodes(lbsNodes);
	}
}

//TDOA�����������߽ṹ���ڵ㡪������CPU���� ������TDOA�㷨
inline void TreeNodeGenerator_TDOA_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, LBSInfoCluster& infoCluster)
{
	size_t sensorNum = scene->m_sensors.size();
	for (size_t i = 0; i < sensorNum; ++i) {
		const Sensor* curSensor = scene->m_sensors[i];
		if (!curSensor->m_isValid) {								//������Ч������
			continue;
		}

		std::vector<PathNode*> nodes;
		GenerateAllPathNode(vroots[i], nodes);
		std::vector<LBSTreeNode*> lbsNodes(nodes.size());
		for (int j = 0; j < nodes.size(); ++j) {
			const PathNode* curPathNode = nodes[j];
			lbsNodes[j] = new LBSTreeNode(*curPathNode);
		}
		for (auto& node : nodes) {
			delete node;
		}
		infoCluster.m_infos[i]->SetNodes(lbsNodes);
	}
}

//TOA�����������߽ṹ���ڵ㡪������CPU���� ������TOA����(��վ������)
inline void TreeNodeGenerator_TOA_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, LBSInfoCluster& infoCluster) {
	size_t sensorNum = scene->m_sensors.size();
	for (size_t i = 0; i < sensorNum; ++i) {
		const Sensor* curSensor = scene->m_sensors[i];
		if (!curSensor->m_isValid) {								//������Ч������
			continue;
		}
		std::vector<RayTreeNode*> treeNodes;
		GenerateAllTreeNode(vroots[i], treeNodes);					
		std::vector<LBSTreeNode*> lbsNodes(treeNodes.size());
		for (int j = 0; j < treeNodes.size(); ++j) {
			const RayTreeNode* curTreeNode = treeNodes[j];					/** @brief	��ǰ�ڵ�	*/
			const RayTreeNode* nextTreeNode = curTreeNode->m_pLeft;			/** @brief	��һ�ڵ�	*/
			lbsNodes[j] = new LBSTreeNode(*curTreeNode->m_data, *nextTreeNode->m_data);
		}
		infoCluster.m_infos[i]->SetNodes(lbsNodes);
	}
}

//AOATOA�����������߽ṹ���ڵ㡪������CPU���� ������TOA��AOA-TOA�㷨
inline void TreeNodeGenerator_AOATOA_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, LBSInfoCluster& infoCluster) {
	size_t sensorNum = scene->m_sensors.size();
	for (size_t i = 0; i < sensorNum; ++i) {
		const Sensor* curSensor = scene->m_sensors[i];
		if (!curSensor->m_isValid) {								//������Ч������
			continue;
		}

		std::vector<PathNode*> nodes;
		GenerateAllLeafTreeNode(vroots[i], nodes);
		std::vector<LBSTreeNode*> lbsNodes(nodes.size());
		for (int j = 0; j < nodes.size(); ++j) {
			const PathNode* curPathNode = nodes[j];
			int sensorDataId = curPathNode->m_nextRay.m_sensorDataId;
			SensorData* sensorData = scene->m_sensorDataLibrary.GetData(sensorDataId);
			lbsNodes[j] = new LBSTreeNode(*curPathNode, sensorData);
		}
		for (auto& node : nodes) {
			delete node;
		}

		infoCluster.m_infos[i]->SetNodes(lbsNodes);
	}
}

inline void TreeNodeGenerator_AOATDOA_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, LBSInfoCluster& infoCluster) {
	size_t sensorNum = scene->m_sensors.size();
	for (size_t i = 0; i < sensorNum; ++i) {
		const Sensor* curSensor = scene->m_sensors[i];
		if (!curSensor->m_isValid) {								//������Ч������
			continue;
		}

		std::vector<PathNode*> nodes;
		GenerateAllPathNode(vroots[i], nodes);
		std::vector<LBSTreeNode*> lbsNodes(nodes.size());
		for (int j = 0; j < nodes.size(); ++j) {
			const PathNode* curPathNode = nodes[j];
			int sensorDataId = curPathNode->m_nextRay.m_sensorDataId;
			SensorData* sensorData = scene->m_sensorDataLibrary.GetData(sensorDataId);
			lbsNodes[j] = new LBSTreeNode(*curPathNode, sensorData);
		}
		for (auto& node : nodes) {
			delete node;
		}

		infoCluster.m_infos[i]->SetNodes(lbsNodes);
	}
}

//AOA�����������߽ṹ���ڵ㡪������CPU���
inline void TreeNodeGenerator_AOA_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, int16_t threadNum, LBSInfoCluster& infoCluster)
{
	size_t sensorNum = scene->m_sensors.size();
	if (sensorNum < threadNum) {
		threadNum = static_cast<int16_t>(sensorNum);			//������������С���߳����������մ���������Ϊ�߳�����
	}

	ThreadPool pool(threadNum);
	std::vector<std::vector<PathNode*>> nodes(sensorNum);
	for (int i = 0; i < sensorNum; ++i) {
		auto future = pool.enqueue(GenerateAllPathNode, vroots[i], nodes[i]);
	}
	while (true) {
		if (pool.getTaskCount() == 0) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50)); // ÿ100������һ��
	}
	for (int i = 0; i < sensorNum; ++i) {
		std::vector<LBSTreeNode*> lbsNodes(nodes[i].size());
		for (int j = 0; j < nodes[i].size(); ++j) {
			const PathNode* curPathNode = nodes[i][j];
			int sensorDataId = curPathNode->m_nextRay.m_sensorDataId;
			SensorData* sensorData = scene->m_sensorDataLibrary.GetData(sensorDataId);
			lbsNodes[j] = new LBSTreeNode(*curPathNode, sensorData);
		}

		for (auto& node : nodes[i]) {
			delete node;
		}

		//��Ӹ��ڵ�,ÿ�����ݶ���һ���ڵ�
		std::vector<SensorData>& sensorDatas = scene->m_sensors[i]->m_sensorDataCollection.m_datas;
		for (auto it = sensorDatas.begin(); it != sensorDatas.end(); ++it) {
			int sensorDataId = (*it).m_id;
			SensorData* sensorData = scene->m_sensorDataLibrary.GetData(sensorDataId);
			lbsNodes.push_back(new LBSTreeNode(*vroots[i]->m_data, sensorData));
		}

		infoCluster.m_infos[i]->SetNodes(lbsNodes);
	}
}


//TDOA�����������߽ṹ���ڵ㡪������CPU���
inline void TreeNodeGenerator_TDOA_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, int16_t threadNum, LBSInfoCluster& infoCluster)
{
	size_t sensorNum = scene->m_sensors.size();
	if (sensorNum < threadNum) {
		threadNum = static_cast<int16_t>(sensorNum);			//������������С���߳����������մ���������Ϊ�߳�����
	}

	ThreadPool pool(threadNum);
	std::vector<std::vector<PathNode*>> nodes(sensorNum);
	for (int i = 0; i < sensorNum; ++i) {
		auto future = pool.enqueue(GenerateAllPathNode, vroots[i], nodes[i]);
	}
	while (true) {
		if (pool.getTaskCount() == 0) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50)); // ÿ100������һ��
	}
	for (int i = 0; i < sensorNum; ++i) {
		std::vector<LBSTreeNode*> lbsNodes(nodes[i].size());
		for (int j = 0; j < nodes[i].size(); ++j) {
			const PathNode* curPathNode = nodes[i][j];
			int sensorDataId = curPathNode->m_nextRay.m_sensorDataId;
			SensorData* sensorData = scene->m_sensorDataLibrary.GetData(sensorDataId);
			lbsNodes[j] = new LBSTreeNode(*curPathNode, sensorData);
		}

		for (auto& node : nodes[i]) {
			delete node;
		}

		//��Ӹ��ڵ�,ÿ�����ݶ���һ���ڵ�
		std::vector<SensorData>& sensorDatas = scene->m_sensors[i]->m_sensorDataCollection.m_datas;
		for (auto it = sensorDatas.begin(); it != sensorDatas.end(); ++it) {
			int sensorDataId = (*it).m_id;
			SensorData* sensorData = scene->m_sensorDataLibrary.GetData(sensorDataId);
			lbsNodes.push_back(new LBSTreeNode(*vroots[i]->m_data, sensorData));
		}

		infoCluster.m_infos[i]->SetNodes(lbsNodes);
	}
}

//����GPU�����ڵ���Ϣ
inline void TreeNodeGenerator_GPUMultiThread(const std::vector<std::vector<TreeNodeGPU*>>& gpuTreeNodes, const Scene* scene, LBSInfoCluster& infoCluster)//δ����޸�
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
		infoCluster.m_infos[i]->SetNodes(lbsNodes);
	}
}
#endif
