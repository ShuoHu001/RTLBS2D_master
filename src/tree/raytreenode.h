#ifndef RTLBS_BINARYTREE
#define RTLBS_BINARYTREE

#include "rtlbs.h"
#include "utility/define.h"
#include "pathnode.h"
#include "raypath.h"


class Scene;

class RayTreeNode {
public:
	bool m_isValid;								/** @brief	�ڵ��Ƿ���Ч	*/
	PathNode* m_data;							/** @brief	�ڵ���Ϣ	*/					
	RayTreeNode* m_pLeft;						/** @brief	���ӽڵ�ָ��	*/
	RayTreeNode* m_pRight;						/** @brief	���ֽڵ�ָ��	*/
	RayTreeNode* m_pGeneralFather;				/** @brief	���常�ڵ�ָ��	*/
public:
	RayTreeNode();
	RayTreeNode(PathNode*& data);
	~RayTreeNode();
	void SetGeneralFatherNode(RayTreeNode* prevNode);						//���ù��常�ڵ�
	bool IsCaptureByPoint(const Point2D& p, RtLbsType splitRadius, RayTreeNode* prev_treenode); //�Ƿ񱻵�ǰ�ڵ㲶��
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
		RayTreeNode* node;									/** @brief	��ǰ�ڵ�	*/
	};

	std::stack<StackItem> stack;
	if (root == nullptr)
		return;
	std::vector<RayTreeNode*> allNodes;						/** @brief	���нڵ�����	*/
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
		stack.push({ tempNode });						//���ڵ�ĸ��ڵ�IDΪ-1
		tempNode = tempNode->m_pRight;
	}

	while (!stack.empty()) {
		StackItem curItem = stack.top();
		stack.pop();

		RayTreeNode* curNode = curItem.node;				/** @brief	��ǰ�ڵ�	*/
		allNodes.push_back(curNode);
		if (curNode->m_pLeft) {
			RayTreeNode* child = curNode->m_pLeft;
			while (child) {
				if (!child->m_isValid) {					//��ֹ��Ч�ڵ���ջ
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

void GenerateAllTreeNode(RayTreeNode* root, std::vector<PathNode*>& outNodes);				//�����������ڵ�
void GenerateMultiPath(RayTreeNode* root, std::vector<RayPath*>& outPaths); //������ȫ·��
void GenerateMultipathofPoint(RayTreeNode* root, Point2D rx, const Scene* scene, RtLbsType splitRadius, std::vector<RayPath*>& outPaths); //����rx��Χ��·��



#endif
