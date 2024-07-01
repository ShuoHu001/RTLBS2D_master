#ifndef RTLBS_BINARYTREE
#define RTLBS_BINARYTREE

#include "rtlbs.h"
#include "utility/define.h"
#include "pathnode.h"
#include "raypath.h"


class Scene;

class RayTreeNode {
public:
	bool m_isValid;								/** @brief	节点是否有效	*/
	PathNode* m_data;							/** @brief	节点信息	*/					
	RayTreeNode* m_pLeft;						/** @brief	左子节点指针	*/
	RayTreeNode* m_pRight;						/** @brief	右兄节点指针	*/
	RayTreeNode* m_pGeneralFather;				/** @brief	广义父节点指针	*/
public:
	RayTreeNode();
	RayTreeNode(PathNode*& data);
	~RayTreeNode();
	void SetGeneralFatherNode(RayTreeNode* prevNode);						//设置广义父节点
	bool IsCaptureByPoint(const Point2D& p, RtLbsType splitRadius, RayTreeNode* prev_treenode); //是否被当前节点捕获
};

inline void delete_tree_recursive(RayTreeNode* node)
{
	if (!node) return;
	if (node->m_pLeft != nullptr) {
		delete_tree_recursive(node->m_pLeft);
	}
	if (node->m_pRight != nullptr) {
		delete_tree_recursive(node->m_pRight);
	}
	delete node;
	node = nullptr;
}

inline void delete_tree_iterative(RayTreeNode* root)
{
	struct StackItem {
		RayTreeNode* node;									/** @brief	当前节点	*/
	};

	std::stack<StackItem> stack;
	if (root == nullptr)
		return;
	std::vector<RayTreeNode*> allNodes;						/** @brief	所有节点数据	*/
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
		stack.push({ tempNode });						//根节点的父节点ID为-1
		tempNode = tempNode->m_pRight;
	}

	while (!stack.empty()) {
		StackItem curItem = stack.top();
		stack.pop();

		RayTreeNode* curNode = curItem.node;				/** @brief	当前节点	*/
		allNodes.push_back(curNode);
		if (curNode->m_pLeft) {
			RayTreeNode* child = curNode->m_pLeft;
			while (child) {
				if (!child->m_isValid) {					//禁止无效节点入栈
					child = child->m_pRight;
					continue;
				}
				stack.push({ child });
				child = child->m_pRight;
			}
		}
	}

	for (auto& node : allNodes) {
		delete node;
		node = nullptr;
	}
	allNodes.clear();
	std::vector<RayTreeNode*>().swap(allNodes);
}

void GenerateAllTreeNode(RayTreeNode* root, std::vector<PathNode*>& outNodes);				//产生所有树节点
void GenerateMultiPath(RayTreeNode* root, std::vector<RayPath*>& outPaths); //产生完全路径
void GenerateMultipathofPoint(RayTreeNode* root, Point2D rx, const Scene* scene, RtLbsType splitRadius, std::vector<RayPath*>& outPaths); //产生rx周围的路径



#endif
