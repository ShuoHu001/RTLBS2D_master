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
	PathNode m_data;							/** @brief	节点信息	*/					
	RayTreeNode* m_pLeft;						/** @brief	左子节点指针	*/
	RayTreeNode* m_pRight;						/** @brief	右兄节点指针	*/
	RayTreeNode* m_pGeneralFather;				/** @brief	广义父节点指针	*/
public:
	RayTreeNode();
	RayTreeNode(PathNode data);
	~RayTreeNode();
	void SetGeneralFatherNode(RayTreeNode* prevNode);						//设置广义父节点
	bool IsCaptureByPoint(const Point2D& p, RtLbsType splitRadius, RayTreeNode* prev_treenode); //是否被当前节点捕获
	
private:
	void delete_tree_recursive(RayTreeNode* node);
	void delete_tree_iterative(RayTreeNode* node);
	
};

void GenerateAllTreeNode(RayTreeNode* root, std::vector<PathNode*>& outNodes);				//产生所有树节点
void GenerateMultiPath(RayTreeNode* root, std::vector<RayPath*>& outPaths); //产生完全路径
void GenerateMultipathofPoint(RayTreeNode* root, Point2D rx, const Scene* scene, RtLbsType splitRadius, std::vector<RayPath*>& outPaths); //产生rx周围的路径


#endif
