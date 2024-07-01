#include "raytreenode.h"
#include "geometry/scene.h"

RayTreeNode::RayTreeNode()
	: m_isValid(true)
	, m_data(nullptr)
	, m_pLeft(nullptr)
	, m_pRight(nullptr)
	, m_pGeneralFather(nullptr)
{
}

RayTreeNode::RayTreeNode(PathNode*& data) 
	: m_isValid(true)
	, m_data(data)
	, m_pLeft(nullptr)
	, m_pRight(nullptr)
	, m_pGeneralFather(nullptr)
{
}

RayTreeNode::~RayTreeNode()
{
	if (m_data != nullptr) {
		delete m_data;
		m_data = nullptr;
	}
}

void RayTreeNode::SetGeneralFatherNode(RayTreeNode* prevNode)
{
	if (m_data->m_type == NODE_ROOT || m_data->m_type == NODE_DIFF) {		//�����ѽڵ㱾��Ϊ���ڵ㣬���϶��䱾��Ϊ���常�ڵ�
		m_pGeneralFather = this;
	}
	else {																//�������������ԭ�ڵ�Ĺ��常�ڵ�
		m_pGeneralFather = prevNode->m_pGeneralFather;
	}
}

bool RayTreeNode::IsCaptureByPoint(const Point2D& p, RtLbsType splitRadius, RayTreeNode* prev_treenode) 
{
	if (!m_isValid)						//����ǰ�ڵ���Ч����ֱ�ӷ���false
		return false;
	if (prev_treenode == nullptr)//����ǰ�ڵ�Ϊ���ڵ㣬�����м�֦
		return false;
	if (m_data->m_type == NODE_DIFF)			//����ǰ�ڵ�Ϊ����ڵ㣬�򲻽��н����ж���������·��Ϊ�������·�����Ѵ������߹ܣ�
		return false;
	PathNode* cur_node = m_data;
	PathNode* prev_node = prev_treenode->m_data;

	Vector2D op = p - prev_node->m_source ;   //����Դ����ǰ�ڵ�����������������ǰһ���ڵ������ֵ 
	RtLbsType t_op = op.Length();				//��ǰ�ڵ�������Դ�ľ���
	RtLbsType t = t_op + cur_node->m_prevRay.m_fMin;//����ΪrootԴ�ľ���  p������Դ����+����Դ���ڽڵ�������ߵ���С����ֵ
	RtLbsType tmin = prev_node->m_ft;//������Сֵ
	RtLbsType tmax = cur_node->m_ft + splitRadius;//�������ֵ ��������splitRadius(����)

	//20240521 �����������tֵ���ж�



	if (t > tmax || t < tmin) {//���ڷ�Χ��
		return false;
	}
		
	//����-1 ���нǶ��ж�
	
	//const Ray2D& ray = cur_node.m_prevRay;
	//double costheta_op = ray.m_Dir * op / t;
	//if (acos(costheta_op) > ray.m_theta)
	//	return false;
	//����-2 ���������ж�
	const Ray2D& ray = cur_node->m_prevRay;
	double costheta_op = ray.m_Dir * op / t_op;//��������ֵ
	if (costheta_op < ray.m_costheta)//�������߹���
		return false;
	return true;
}

void GenerateAllTreeNode(RayTreeNode* root, std::vector<PathNode*>& outNodes)
{

	//�Ȳ�������·��������·������ӽڵ�

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
			outNodes.push_back(new PathNode(*curNode->m_data));
			outNodes.back()->m_fatherNodeId = curFatherNodeId;		//�޸���������Ԫ�صĸ��ڵ�ID
		}
		if (curNode->m_pLeft) {
			RayTreeNode* child = curNode->m_pLeft;
			int curNodeId = static_cast<int>(outNodes.size()) - 1;			/** @brief	��ǰ������ڵ��ID	*/
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
	struct StackItem{
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
		if (!curNode->m_pLeft ) {//��ǰ�ڵ���Ҷ�ӽڵ�(�������¼��ڵ�-����ӽڵ�),����·��
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
			if (current_path->IsValidAndRectify(rx, scene)) {
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
