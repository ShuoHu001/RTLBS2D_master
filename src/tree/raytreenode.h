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
	PathNode m_data;							/** @brief	�ڵ���Ϣ	*/					
	RayTreeNode* m_pLeft;						/** @brief	���ӽڵ�ָ��	*/
	RayTreeNode* m_pRight;						/** @brief	���ֽڵ�ָ��	*/
	RayTreeNode* m_pGeneralFather;				/** @brief	���常�ڵ�ָ��	*/
public:
	RayTreeNode();
	RayTreeNode(PathNode data);
	~RayTreeNode();
	void SetGeneralFatherNode(RayTreeNode* prevNode);						//���ù��常�ڵ�
	bool IsCaptureByPoint(const Point2D& p, RtLbsType splitRadius, RayTreeNode* prev_treenode); //�Ƿ񱻵�ǰ�ڵ㲶��
	
private:
	void delete_tree_recursive(RayTreeNode* node);
	void delete_tree_iterative(RayTreeNode* node);
	
};

void GenerateAllTreeNode(RayTreeNode* root, std::vector<PathNode*>& outNodes);				//�����������ڵ�
void GenerateMultiPath(RayTreeNode* root, std::vector<RayPath*>& outPaths); //������ȫ·��
void GenerateMultipathofPoint(RayTreeNode* root, Point2D rx, const Scene* scene, RtLbsType splitRadius, std::vector<RayPath*>& outPaths); //����rx��Χ��·��


#endif
