#include "treenodegenerator.h"
#include "radiowave/raypath/gpu/cpuconverterpathnode.h"
#include "result/result.h"

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

			CPUConverterPathNode newNode(*curNode->m_data, cur_faterNodeId, sensorId, cur_layerId);
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

void GenerateAllTreeNode(RayTreeNode* root, std::vector<PathNode*>* outNodes)
{

	struct StackItem {
		RayTreeNode* node;									/** @brief	当前节点	*/
		int fatherNodeId;									/** @brief	父节点在数组中的ID	*/
	};
	std::stack<StackItem> stack;
	if (root == nullptr)
		return;
	//将所有虚拟根节点添加到栈中

	RayTreeNode* tempNode = root;							/** @brief	临时节点，用于进行迭代	*/

	//确定虚拟根节点的数量
	int vrootNum = 0;
	tempNode = tempNode->m_pRight;							//过滤掉第一个无效root,只保留vroot
	while (tempNode != nullptr) {
		tempNode = tempNode->m_pRight;
		vrootNum++;
	}
	tempNode = root->m_pRight;
	for (int i = 0; i < vrootNum; ++i) {
		if (!tempNode->m_isValid) {							//禁止无效节点入栈
			tempNode = tempNode->m_pRight;
			continue;
		}
		stack.push({ tempNode, -1 });						//根节点的父节点ID为-1
		tempNode = tempNode->m_pRight;
	}

	while (!stack.empty()) {
		StackItem curItem = stack.top();
		stack.pop();

		RayTreeNode* curNode = curItem.node;				/** @brief	当前节点	*/
		int curFatherNodeId = curItem.fatherNodeId;			/** @brief	当前节点的父节点ID	*/

		if (!curNode->m_isValid ||
			curNode->m_data->m_type == NODE_LOS ||
			curNode->m_data->m_type == NODE_STOP ||
			curNode->m_data->m_type == NODE_TRANIN ||
			curNode->m_data->m_type == NODE_ETRANIN) { //无效节点：停止节点、透射入节点、经验透射入节点均为无效节点(对于求解广义源来说是无效的),不纳入节点
		}
		else {
			outNodes->push_back(new PathNode(*curNode->m_data));
			outNodes->back()->m_fatherNodeId = curFatherNodeId;		//修改新入数组元素的父节点ID
		}
		if (curNode->m_pLeft) {
			RayTreeNode* child = curNode->m_pLeft;
			int curNodeId = static_cast<int>(outNodes->size()) - 1;			/** @brief	当前入数组节点的ID	*/
			while (child) {
				if (!child->m_isValid) {					//禁止无效节点入栈
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
	//将所有虚拟根节点添加到栈中

	RayTreeNode* tempNode = root;							/** @brief	临时节点，用于进行迭代	*/

	//确定虚拟根节点的数量
	int vrootNum = 0;
	while (tempNode != nullptr) {
		tempNode = tempNode->m_pRight;
		vrootNum++;
	}

	std::vector<RayPath*> tempPaths(vrootNum);				/** @brief	存储临时申请路径的内存，用于模拟栈中进行迭代递归	*/
	std::vector<RayPath*> extraTempPaths;					/** @brief	存储额外申请路径的内存，用于右子节点（同级节点的迭代）	*/
	tempNode = root;
	for (int i = 0; i < vrootNum; ++i) {
		tempPaths[i] = new RayPath();
		if (!tempNode->m_isValid) {							//禁止无效节点入栈
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
		if (!curNode->m_pLeft) {//当前节点是叶子节点(不存在下级节点-左二子节点),纳入路径
			//深度复制raypath
			if (curPath->m_nodes.size() < 2)
				continue;
			RayPath* newPath = new RayPath(*curPath);
			outPaths.push_back(newPath);
		}
		else {
			RayTreeNode* child = curNode->m_pLeft;
			while (child) {
				if (!child->m_isValid) {					//禁止无效节点入栈
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


	//释放在模拟栈迭代过程中所申请的内存
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
		RayTreeNode* cur_node; /** @brief	存储当前节点	*/
		RayTreeNode* prev_node; /** @brief	存储前一个节点	*/
		RayPath* path;
	};

	std::stack<StackItem> stack;
	if (root == nullptr)
		return;
	//将所有虚拟根节点添加到栈中
	RayTreeNode* tempNode = root;
	//遍历路径获得虚拟根节点数量
	size_t vRootNum = 0;														/** @brief	虚拟根节点数量	*/
	while (tempNode != nullptr) {
		tempNode = tempNode->m_pRight;
		vRootNum++;
	}
	std::vector<RayPath*> tempPaths(vRootNum);									/** @brief	临时路径内存，用于模拟栈中的新路径内存	*/
	std::vector<RayPath*> extraTempPaths;										/** @brief	额外申请的内存，用于存储无效的新路径	*/
	tempNode = root;
	for (int i = 0; i < vRootNum; ++i) {
		tempPaths[i] = new RayPath();
		if (!tempNode->m_isValid) {							//禁止无效节点入栈
			tempNode = tempNode->m_pRight;
			continue;
		}
		stack.push({ tempNode, nullptr, tempPaths[i] });						//压入虚拟根节点
		tempNode = tempNode->m_pRight;
	}

	int debugId = 0;
	while (!stack.empty()) {
		StackItem current_item = stack.top();
		stack.pop();

		RayTreeNode* cur_node = current_item.cur_node;
		RayTreeNode* prev_node = current_item.prev_node;
		RayPath* current_path = current_item.path;

		if (cur_node->m_data->m_type == NODE_TRANOUT) {//若当前节点为透射节点，则该条路径被认定为透射路径, 经验透射路径不被认为是需要被路径修正的透射路径
			current_path->m_bContainRefract = true;
		}
		current_path->m_nodes.push_back(new PathNode(*cur_node->m_data));

		debugId++;
		if (cur_node->IsCaptureByPoint(rx, splitRadius, prev_node)) {//当前节点rx节点位置
			//当前 current_path进行自检，若自检成功，则保留当前路径
			if (RectifyRayPath(scene, current_path, rx)) {
				RayPath* newPath = new RayPath(*current_path);
				outPaths.push_back(newPath);
			}
			continue;
		}
		RayTreeNode* child_node = cur_node->m_pLeft;					//若当前节点没有，则将其下一级节点纳入考虑
		while (child_node) {
			if (!child_node->m_isValid) {					//禁止无效节点入栈
				child_node = child_node->m_pRight;
				continue;
			}
			RayPath* newPath = new RayPath(*current_path);
			extraTempPaths.push_back(newPath);
			stack.push({ child_node, cur_node, newPath });
			child_node = child_node->m_pRight;
		}
	}

	//针对透射路径可能出现重复的路径进行筛除
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

	//释放在模拟栈迭代过程中所申请的内存
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
