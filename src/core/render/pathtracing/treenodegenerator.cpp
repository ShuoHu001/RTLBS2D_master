#include "treenodegenerator.h"
#include "radiowave/raypath/gpu/cpuconverterpathnode.h"
#include "result/result.h"

void GenerateAllTreeNodeAndConvertToCPUConvertPathNode(const std::vector<RayTreeNode*>& roots, std::vector<CPUConverterPathNode>& outNodes, int& maxDepth)
{
	struct StackItem {
		RayTreeNode* cur_node;					/** @brief	��ǰ�ڵ�	*/
		int father_nodeId;						/** @brief	��ǰ�ڵ㸸�ڵ��������е�ID	*/
		int layerId;							/** @brief	��ǰ�ڵ����ڵĲ�ID	*/
	};
	std::stack<StackItem> stack;

	//����ÿ�������������ڵ�Ԫ��ת��ΪCPUConverterPathNode
	for (int sensorId = 0; sensorId < static_cast<int>(roots.size()); ++sensorId) {
		RayTreeNode* curRoot = roots[sensorId];

		if (curRoot == nullptr) {				//������Ч���ڵ�
			continue;
		}
		//ȷ��������ڵ������
		int vrootNum = 0;
		RayTreeNode* tempNode = curRoot;		/** @brief	��ʱ�ڵ㽻��ָ��	*/	
		while (tempNode != nullptr) {
			tempNode = tempNode->m_pRight;
			vrootNum++;
		}

		tempNode = curRoot;
		for (int i = 0; i < vrootNum; ++i) {
			if (!tempNode->m_isValid) {			//������Ч�ڵ�
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
			if (maxDepth < cur_layerId) {											//Ѱ��������
				maxDepth = cur_layerId;
			}

			CPUConverterPathNode newNode(*curNode->m_data, cur_faterNodeId, sensorId, cur_layerId);
			outNodes.push_back(newNode);

			if (curNode->m_pLeft) {
				int next_fatherNodeId = static_cast<int>(outNodes.size() - 1);			/** @brief	��һ�ڵ�ĸ��ڵ�ID	*/
				int next_layerId = cur_layerId + 1;									/** @brief	��һ�ڵ�Ĳ� ID	*/
				RayTreeNode* childNode = curNode->m_pLeft;
				while (childNode) {
					if (!childNode->m_isValid) {					//������Ч�ڵ�
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

void GenerateAllTreeNode(RayTreeNode* root, std::vector<PathNode*>* outNodes)
{

	struct StackItem {
		RayTreeNode* node;									/** @brief	��ǰ�ڵ�	*/
		int fatherNodeId;									/** @brief	���ڵ��������е�ID	*/
	};
	std::stack<StackItem> stack;
	if (root == nullptr)
		return;
	//������������ڵ���ӵ�ջ��

	RayTreeNode* tempNode = root;							/** @brief	��ʱ�ڵ㣬���ڽ��е���	*/

	//ȷ��������ڵ������
	int vrootNum = 0;
	tempNode = tempNode->m_pRight;							//���˵���һ����Чroot,ֻ����vroot
	while (tempNode != nullptr) {
		tempNode = tempNode->m_pRight;
		vrootNum++;
	}
	tempNode = root->m_pRight;
	for (int i = 0; i < vrootNum; ++i) {
		if (!tempNode->m_isValid) {							//��ֹ��Ч�ڵ���ջ
			tempNode = tempNode->m_pRight;
			continue;
		}
		stack.push({ tempNode, -1 });						//���ڵ�ĸ��ڵ�IDΪ-1
		tempNode = tempNode->m_pRight;
	}

	while (!stack.empty()) {
		StackItem curItem = stack.top();
		stack.pop();

		RayTreeNode* curNode = curItem.node;				/** @brief	��ǰ�ڵ�	*/
		int curFatherNodeId = curItem.fatherNodeId;			/** @brief	��ǰ�ڵ�ĸ��ڵ�ID	*/

		if (!curNode->m_isValid ||
			curNode->m_data->m_type == NODE_LOS ||
			curNode->m_data->m_type == NODE_STOP ||
			curNode->m_data->m_type == NODE_TRANIN ||
			curNode->m_data->m_type == NODE_ETRANIN) { //��Ч�ڵ㣺ֹͣ�ڵ㡢͸����ڵ㡢����͸����ڵ��Ϊ��Ч�ڵ�(����������Դ��˵����Ч��),������ڵ�
		}
		else {
			outNodes->push_back(new PathNode(*curNode->m_data));
			outNodes->back()->m_fatherNodeId = curFatherNodeId;		//�޸���������Ԫ�صĸ��ڵ�ID
		}
		if (curNode->m_pLeft) {
			RayTreeNode* child = curNode->m_pLeft;
			int curNodeId = static_cast<int>(outNodes->size()) - 1;			/** @brief	��ǰ������ڵ��ID	*/
			while (child) {
				if (!child->m_isValid) {					//��ֹ��Ч�ڵ���ջ
					child = child->m_pRight;
					continue;
				}
				stack.push({ child, curNodeId });
				child = child->m_pRight;
			}
		}
	}

}

void GenerateMultiPath(RayTreeNode* root, std::vector<RayPath*>& outPaths)
{
	struct StackItem {
		RayTreeNode* node;
		RayPath* path;
	};

	std::stack<StackItem> stack;
	if (root == nullptr)
		return;
	//������������ڵ���ӵ�ջ��

	RayTreeNode* tempNode = root;							/** @brief	��ʱ�ڵ㣬���ڽ��е���	*/

	//ȷ��������ڵ������
	int vrootNum = 0;
	while (tempNode != nullptr) {
		tempNode = tempNode->m_pRight;
		vrootNum++;
	}

	std::vector<RayPath*> tempPaths(vrootNum);				/** @brief	�洢��ʱ����·�����ڴ棬����ģ��ջ�н��е����ݹ�	*/
	std::vector<RayPath*> extraTempPaths;					/** @brief	�洢��������·�����ڴ棬�������ӽڵ㣨ͬ���ڵ�ĵ�����	*/
	tempNode = root;
	for (int i = 0; i < vrootNum; ++i) {
		tempPaths[i] = new RayPath();
		if (!tempNode->m_isValid) {							//��ֹ��Ч�ڵ���ջ
			tempNode = tempNode->m_pRight;
			continue;
		}
		stack.push({ tempNode, tempPaths[i] });
		tempNode = tempNode->m_pRight;
	}

	while (!stack.empty()) {
		StackItem curItem = stack.top();
		stack.pop();

		RayTreeNode* curNode = curItem.node;
		RayPath* curPath = curItem.path;

		curPath->m_nodes.push_back(new PathNode(*curNode->m_data));
		if (!curNode->m_pLeft) {//��ǰ�ڵ���Ҷ�ӽڵ�(�������¼��ڵ�-����ӽڵ�),����·��
			//��ȸ���raypath
			if (curPath->m_nodes.size() < 2)
				continue;
			RayPath* newPath = new RayPath(*curPath);
			outPaths.push_back(newPath);
		}
		else {
			RayTreeNode* child = curNode->m_pLeft;
			while (child) {
				if (!child->m_isValid) {					//��ֹ��Ч�ڵ���ջ
					child = child->m_pRight;
					continue;
				}
				RayPath* newRayPath = new RayPath(*curPath);
				extraTempPaths.push_back(newRayPath);
				stack.push({ child,newRayPath });
				child = child->m_pRight;
			}
		}

	}


	//�ͷ���ģ��ջ������������������ڴ�
	for (auto& path : tempPaths) {
		if (path != nullptr) {
			delete path;
			path = nullptr;
		}
	}
	tempPaths.clear();
	std::vector<RayPath*>().swap(tempPaths);

	for (auto& path : extraTempPaths) {
		if (path != nullptr) {
			delete path;
			path = nullptr;
		}
	}
	extraTempPaths.clear();
	std::vector<RayPath*>().swap(extraTempPaths);
}

void GenerateMultipathofPoint(RayTreeNode* root, Point2D rx, const Scene* scene, RtLbsType splitRadius, std::vector<RayPath*>& outPaths)
{
	struct StackItem {
		RayTreeNode* cur_node; /** @brief	�洢��ǰ�ڵ�	*/
		RayTreeNode* prev_node; /** @brief	�洢ǰһ���ڵ�	*/
		RayPath* path;
	};

	std::stack<StackItem> stack;
	if (root == nullptr)
		return;
	//������������ڵ���ӵ�ջ��
	RayTreeNode* tempNode = root;
	//����·�����������ڵ�����
	size_t vRootNum = 0;														/** @brief	������ڵ�����	*/
	while (tempNode != nullptr) {
		tempNode = tempNode->m_pRight;
		vRootNum++;
	}
	std::vector<RayPath*> tempPaths(vRootNum);									/** @brief	��ʱ·���ڴ棬����ģ��ջ�е���·���ڴ�	*/
	std::vector<RayPath*> extraTempPaths;										/** @brief	����������ڴ棬���ڴ洢��Ч����·��	*/
	tempNode = root;
	for (int i = 0; i < vRootNum; ++i) {
		tempPaths[i] = new RayPath();
		if (!tempNode->m_isValid) {							//��ֹ��Ч�ڵ���ջ
			tempNode = tempNode->m_pRight;
			continue;
		}
		stack.push({ tempNode, nullptr, tempPaths[i] });						//ѹ��������ڵ�
		tempNode = tempNode->m_pRight;
	}

	int debugId = 0;
	while (!stack.empty()) {
		StackItem current_item = stack.top();
		stack.pop();

		RayTreeNode* cur_node = current_item.cur_node;
		RayTreeNode* prev_node = current_item.prev_node;
		RayPath* current_path = current_item.path;

		if (cur_node->m_data->m_type == NODE_TRANOUT) {//����ǰ�ڵ�Ϊ͸��ڵ㣬�����·�����϶�Ϊ͸��·��, ����͸��·��������Ϊ����Ҫ��·��������͸��·��
			current_path->m_bContainRefract = true;
		}
		current_path->m_nodes.push_back(new PathNode(*cur_node->m_data));

		debugId++;
		if (cur_node->IsCaptureByPoint(rx, splitRadius, prev_node)) {//��ǰ�ڵ�rx�ڵ�λ��
			//��ǰ current_path�����Լ죬���Լ�ɹ���������ǰ·��
			if (RectifyRayPath(scene, current_path, rx)) {
				RayPath* newPath = new RayPath(*current_path);
				outPaths.push_back(newPath);
			}
			continue;
		}
		RayTreeNode* child_node = cur_node->m_pLeft;					//����ǰ�ڵ�û�У�������һ���ڵ����뿼��
		while (child_node) {
			if (!child_node->m_isValid) {					//��ֹ��Ч�ڵ���ջ
				child_node = child_node->m_pRight;
				continue;
			}
			RayPath* newPath = new RayPath(*current_path);
			extraTempPaths.push_back(newPath);
			stack.push({ child_node, cur_node, newPath });
			child_node = child_node->m_pRight;
		}
	}

	//���͸��·�����ܳ����ظ���·������ɸ��
	for (auto it = outPaths.begin(); it != outPaths.end(); ++it) {
		for (auto it2 = it + 1; it2 != outPaths.end();) {
			if (RepatePathComparison(*(*it), *(*it2))) {
				it2 = outPaths.erase(it2);
			}
			else {
				++it2;
			}
		}
	}

	//�ͷ���ģ��ջ������������������ڴ�
	for (auto& path : tempPaths) {
		if (path != nullptr) {
			delete path;
			path = nullptr;
		}
	}
	tempPaths.clear();
	std::vector<RayPath*>().swap(tempPaths);

	for (auto& path : extraTempPaths) {
		if (path != nullptr) {
			delete path;
			path = nullptr;
		}
	}
	extraTempPaths.clear();
	std::vector<RayPath*>().swap(extraTempPaths);
}
