#include "pathtracing.h"

//���ӿ��ǣ���RTģʽ�еļ��ֹ���
//����͸��In��͸��Out���ж�����ǰ�ڵ�Ϊ͸��In����ô�����з�������� -- ��ɹ����ƶ�
//·��׷��ģ��
void PathTrace(bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, RayTreeNode*& vroot) {
	struct StackItem {
		RayTreeNode* node;			//��ǰ�ڵ�
	};
	const LimitInfo limitInfo = vroot->m_data.m_limitInfo;
	const Ray2D& initRay = vroot->m_data.m_nextRay;
	EndStopInfo endStopInfo = limitInfo.CalEndStopInfo();
	//��ʼ��ջ�ռ�
	std::stack<StackItem> stack;
	vroot->m_pGeneralFather = vroot;
	stack.push({ vroot});


	while (!stack.empty()) {//����ѭ��ֱ��ջΪ��
		StackItem current_item = stack.top();
		stack.pop();
		
		RayTreeNode* curNode = current_item.node;						/** @brief	��ǰ�ڵ�	*/

		Ray2D& curRay = curNode->m_data.m_nextRay;
		LimitInfo curLimitInfo = curNode->m_data.m_limitInfo;
		EndStopInfo curStopInfo = curLimitInfo.CalEndStopInfo();

		if (!curNode->m_pGeneralFather->m_isValid) {						//��ջ�нڵ�Ĺ��常�ڵ�Ϊ��Ч�ڵ㣬���������ѹջ����·��
			continue;
		}

		bool IsBlockByEnvironment = false;//�����Ƿ񱻻����ڵ�
		bool rayTubeHasWedges = false;//���߹����Ƿ���������
		bool IsRaySplit = false; //�����Ƿ���з�������
		RtLbsType t;//���ڼ������߷�������
		bool hasLeftChild = false;//��ǰ�ڵ��Ƿ�������ӽڵ㣬Ĭ�ϲ��������ӽڵ�
		RayTreeNode* temp_node = curNode;//��ʱдָ��ڵ�

		Intersection2D intersect;
		IsBlockByEnvironment = scene->GetIntersect(curRay, &intersect);//���������뻷�����ཻ�ж�
		if (IsBlockByEnvironment) {																									//�ڵ����߷���ģ��+intersect ����ģ��
			if (!intersect.Update(curRay)) {//��֤��ǰ�ڵ��Ƿ���֮ǰ�Ľڵ��д��ڹ�����㣬�����ڣ������׷�ٸ���·������ֹ�ظ�����·��
				continue;
			}
			t = intersect.m_ft;
		}
		else {																														//�Ӿ����߷���ģ��
			BBox2D bbox = scene->GetBBox();//��������볡���а�Χ�еĽ���,���������ǰ�ڵ���
			t = bbox.Intersect(curRay, nullptr);
		}

		int raySplitNum = 0;
		if (IsGenerateSplittingRays(curRay, t, raySplitFlag, raySplitRadius, raySplitNum)) {
			RayTreeNode* generalFatherNode = curNode->m_pGeneralFather;								//������常�ڵ�
			RayTreeNode* generalFatherRightNode = generalFatherNode->m_pRight;								//���常�ڵ�����ֽڵ���ʱ�洢
			temp_node = generalFatherNode;																	//��ȡ���常�ڵ�ֵ�����ڷ��ѽڵ�д��
			std::vector<Ray2D> splitRays;
			GenerateSplittingRay(generalFatherNode->m_data.m_nextRay, raySplitNum, &splitRays);
			generalFatherNode->m_isValid = false;															//�ѷ��ѵĹ��常�ڵ�����Ϊ��Ч�ڵ�
			for (auto it = splitRays.begin(); it != splitRays.end(); ++it) {
				Ray2D& splitRay = *it;
				PathNode splitPathNode(generalFatherNode->m_data);
				splitPathNode.m_nextRay = splitRay;
				RayTreeNode* splitTreeNode = new RayTreeNode(splitPathNode);
				splitTreeNode->SetGeneralFatherNode(curNode);
				temp_node->m_pRight = splitTreeNode;
				temp_node = temp_node->m_pRight;
				stack.push({ splitTreeNode });
			}
			temp_node->m_pRight = generalFatherRightNode;
			continue;
		}

		EndStopInfo curEndStopInfo = curLimitInfo.CalEndStopInfo();								//��ȡ��ǰֹͣ��Ϣ

		//-----------------------------------------��������������������-----------------------------------------------------------------------------------------------
		if (!curEndStopInfo.EndStopDiffract) {//�����������ԣ���Ѱ�ҹ��ڵ���������
			rayTubeHasWedges = scene->GetRayTubeWedges(curRay, curNode, &intersect);//���߹����뻷��wedge���ཻ�ж�
		}

		//-----------------------------------------�Ӿ���㲿��-----------------------------------------------------------------------------------------------
		if (IsBlockByEnvironment == false) {//�������ڵ���д���Ӿ�ڵ�
			Point2D bbox_inter_point = GetRayCoordinate(curRay, t);
			PathNode pathnode(curLimitInfo, NODE_LOS, bbox_inter_point, curRay);
			RayTreeNode* loss_treenode = new RayTreeNode(pathnode);
			temp_node->m_pLeft = loss_treenode;
			temp_node = temp_node->m_pLeft;
			hasLeftChild = true;
			if (rayTubeHasWedges == false) {//������������������ִ�к�������
				continue;
			}
		}
		//-----------------------------------------�ڵ����㲿��----------------------------------------------------------------------------------------------	
		const PropagationProperty& propagationProperty = intersect.m_segment->m_propagationProperty;							//��ȡ������Ϣ�е���Ԫ��������״̬

		if (intersect.m_type == NODE_REFL) {
			if (curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {								//�޷�����͸, д����ֹ�ڵ�
				PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);
				RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
				endTreeNode->SetGeneralFatherNode(curNode);
				if (hasLeftChild) {
					temp_node->m_pRight = endTreeNode;
					temp_node = temp_node->m_pRight;
				}
				else {
					temp_node->m_pLeft = endTreeNode;
					temp_node = temp_node->m_pLeft;
				}
				hasLeftChild = true;
			}
			else if (!curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {						//�з�����͸��д�뷴��ڵ�
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//����ڵ�
					LimitInfo reflectLimitInfo = curLimitInfo;
					reflectLimitInfo.MinusReflectLimit();
					PathNode reflect_pathnode(reflectLimitInfo, NODE_REFL, intersect.m_intersect, intersect.m_segment, curRay, reflect_ray);
					RayTreeNode* reflect_treenode = new RayTreeNode(reflect_pathnode);
					reflect_treenode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = reflect_treenode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = reflect_treenode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
					stack.push({ temp_node });
				}
				else {				//���������˷������ߣ���д����ֹ�ڵ�
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //Ĭ��ֻ��һ���ڵ���Ҫ�޸�
					RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
					endTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = endTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = endTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
				}
			}
			else if (curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {						//�޷�����͸��д��͸��ڵ�
				Ray2D transmit_ray;
				PATHNODETYPE tranType;										/** @brief	͸������	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType)) {//��������͸������													
					LimitInfo transmitLimitInfo = curLimitInfo;
					transmitLimitInfo.MinusTransmitLimit();
					PathNode transmit_pathnode(transmitLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* transmit_treenode = new RayTreeNode(transmit_pathnode);
					transmit_treenode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = transmit_treenode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = transmit_treenode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
					stack.push({ temp_node });
				}
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType)) {//��������͸������
					PathNode empiricalTransmitPathNode(curLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* empiricalTransmitTreeNode = new RayTreeNode(empiricalTransmitPathNode);
					empiricalTransmitTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = empiricalTransmitTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = empiricalTransmitTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
					stack.push({ temp_node });
				}
				else {				//����������͸�����ߣ���д����ֹ�ڵ�
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);
					RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
					endTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = endTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = endTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
				}
			}
			else if (!curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {						//�з�����͸, д�뷴��ڵ㡢͸��ڵ�
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//����ڵ�
					LimitInfo reflectLimitInfo = curLimitInfo;
					reflectLimitInfo.MinusReflectLimit();
					PathNode reflect_pathnode(reflectLimitInfo, NODE_REFL, intersect.m_intersect, intersect.m_segment, curRay, reflect_ray);
					RayTreeNode* reflect_treenode = new RayTreeNode(reflect_pathnode);
					reflect_treenode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = reflect_treenode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = reflect_treenode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
					stack.push({ temp_node });
				}
				else {				//���������˷������ߣ���д����ֹ�ڵ�
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //Ĭ��ֻ��һ���ڵ���Ҫ�޸�
					RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
					endTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = endTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = endTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
				}

				Ray2D transmit_ray;
				PATHNODETYPE tranType;										/** @brief	͸������	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType)) {															//��������͸������													
					LimitInfo transmitLimitInfo = curLimitInfo;
					transmitLimitInfo.MinusTransmitLimit();
					PathNode transmit_pathnode(transmitLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* transmit_treenode = new RayTreeNode(transmit_pathnode);
					transmit_treenode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = transmit_treenode;
					temp_node = temp_node->m_pRight;
					stack.push({ temp_node });
				}
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType)) {//��������͸������
					PathNode empiricalTransmitPathNode(curLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* empiricalTransmitTreeNode = new RayTreeNode(empiricalTransmitPathNode);
					empiricalTransmitTreeNode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = empiricalTransmitTreeNode;
					temp_node = temp_node->m_pRight;
					stack.push({ temp_node });
				}
			}
			continue;
		}

		if (intersect.m_type == NODE_DIFF) {																										//������ڵ�-���Ӷ�wedge��������
			if (curEndStopInfo.EndStopDiffract) {//�����ֹ�ڵ�
				for (Wedge2D* wedeg : intersect.m_wedges) {
					PathNode end_pathnode(curLimitInfo, NODE_STOP, wedeg->m_point, wedeg, curRay);
					RayTreeNode* end_treenode = new RayTreeNode(end_pathnode);
					end_treenode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {			
						temp_node->m_pRight = end_treenode;
						temp_node = temp_node->m_pRight;
					}
					else {							
						temp_node->m_pLeft = end_treenode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
				}
			}
			else {
				for (Wedge2D* wedeg : intersect.m_wedges) {
					std::vector<Ray2D> diffract_rays;
					if (GenerateDiffractRays(curRay, wedeg, &diffract_rays)) {
						LimitInfo diffractLimitInfo = curLimitInfo;
						diffractLimitInfo.MinusDiffractLimit();
						if (hasLeftChild) {//��ǰ�����ӽڵ��Ѿ���д�룬ֱ��д��ڵ�����ֽڵ㼴��
							for (int i = 0; i < diffract_rays.size(); i++) {
								PathNode diffract_node(diffractLimitInfo, NODE_DIFF, wedeg->m_point, wedeg, curRay, diffract_rays[i]);//ÿ���ڵ�洢ǰһ������
								RayTreeNode* diffract_treenode = new RayTreeNode(diffract_node);
								diffract_treenode->SetGeneralFatherNode(curNode);
								temp_node->m_pRight = diffract_treenode;
								temp_node = temp_node->m_pRight;
								stack.push({ temp_node });
							}
						}
						else {
							PathNode firstdiffract_node(diffractLimitInfo, NODE_DIFF, wedeg->m_point, wedeg, curRay, diffract_rays[0]);			//��һ������ڵ�Ϊ�ӽڵ�
							RayTreeNode* firstdiffract_treenode = new RayTreeNode(firstdiffract_node);
							firstdiffract_treenode->SetGeneralFatherNode(curNode);
							temp_node->m_pLeft = firstdiffract_treenode;
							temp_node = temp_node->m_pLeft;
							hasLeftChild = true;
							stack.push({ temp_node });
							for (int i = 1; i < diffract_rays.size(); i++) {
								PathNode diffract_node(diffractLimitInfo, NODE_DIFF, wedeg->m_point, wedeg, curRay, diffract_rays[i]);
								RayTreeNode* diffract_treenode = new RayTreeNode(diffract_node);
								diffract_treenode->SetGeneralFatherNode(curNode);
								temp_node->m_pRight = diffract_treenode;
								temp_node = temp_node->m_pRight;
								stack.push({ temp_node });
							}
						}
					}
					else {				//���޷������������ߣ���д��������ֹ�ڵ�
						for (Wedge2D* wedeg : intersect.m_wedges) {
							PathNode end_pathnode(curLimitInfo, NODE_STOP, wedeg->m_point, wedeg, curRay);
							RayTreeNode* end_treenode = new RayTreeNode(end_pathnode);
							end_treenode->SetGeneralFatherNode(curNode);
							if (!hasLeftChild) {			
								temp_node->m_pRight = end_treenode;
								temp_node = temp_node->m_pRight;
							}
							else {							
								temp_node->m_pLeft = end_treenode;
								temp_node = temp_node->m_pLeft;
								hasLeftChild = true;
							}
						}
					}
				}
			}
			continue;
		}

		if (intersect.m_type == NODE_REFLDIFF) {																						//����+����ڵ�
			if (curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {				//���1 �޷�����͸, ֱ��д����ֹ�ڵ�
				PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay); 
				RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
				endTreeNode->SetGeneralFatherNode(curNode);
				if (hasLeftChild) {
					temp_node->m_pRight = endTreeNode;
					temp_node = temp_node->m_pRight;
				}
				else {
					temp_node->m_pLeft = endTreeNode;
					temp_node = temp_node->m_pLeft;
				}
				hasLeftChild = true;
			}
			else if (!curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {		//���2 �з�����͸, д�뷴��ڵ�
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//����ڵ�
					LimitInfo reflectLimitInfo = curLimitInfo;
					reflectLimitInfo.MinusReflectLimit();
					PathNode reflect_pathnode(reflectLimitInfo, NODE_REFL, intersect.m_intersect, intersect.m_segment, curRay, reflect_ray);
					RayTreeNode* reflect_treenode = new RayTreeNode(reflect_pathnode);
					reflect_treenode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = reflect_treenode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = reflect_treenode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
					stack.push({ temp_node });
				}
				else {				//���������˷������ߣ���д����ֹ�ڵ�
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //Ĭ��ֻ��һ���ڵ���Ҫ�޸�
					RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
					endTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = endTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = endTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
				}
			}
			else if (curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {		//���3 �޷�����͸, д��͸��ڵ�
				Ray2D transmit_ray;
				PATHNODETYPE tranType;										/** @brief	͸������	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType)) {//��������͸������													
					LimitInfo transmitLimitInfo = curLimitInfo;
					transmitLimitInfo.MinusTransmitLimit();
					PathNode transmit_pathnode(transmitLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* transmit_treenode = new RayTreeNode(transmit_pathnode);
					transmit_treenode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = transmit_treenode; 
						temp_node = temp_node->m_pRight;										
					}
					else {
						temp_node->m_pLeft = transmit_treenode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;											
					}
					stack.push({ temp_node });
				}
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType)) {//��������͸������
					PathNode empiricalTransmitPathNode(curLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* empiricalTransmitTreeNode = new RayTreeNode(empiricalTransmitPathNode);
					empiricalTransmitTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = empiricalTransmitTreeNode;
						temp_node = temp_node->m_pRight;										
					}
					else {
						temp_node->m_pLeft = empiricalTransmitTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;											
					}
					stack.push({ temp_node });
				}
				else {				//����������͸�����ߣ���д��͸����ֹ�ڵ�
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);
					RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
					endTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = endTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = endTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
				}
			}
			else if(!curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {		//���4 �з�����͸, ��д�뷴��ڵ㣬��д��͸��ڵ�
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//����ڵ�
					LimitInfo reflectLimitInfo = curLimitInfo;
					reflectLimitInfo.MinusReflectLimit();
					PathNode reflect_pathnode(reflectLimitInfo, NODE_REFL, intersect.m_intersect, intersect.m_segment, curRay, reflect_ray);
					RayTreeNode* reflect_treenode = new RayTreeNode(reflect_pathnode);
					reflect_treenode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = reflect_treenode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = reflect_treenode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
					stack.push({ temp_node });
				}
				else {				//���������˷������ߣ���д����ֹ�ڵ�
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //Ĭ��ֻ��һ���ڵ���Ҫ�޸�
					RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
					endTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = endTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = endTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
				}

				Ray2D transmit_ray;
				PATHNODETYPE tranType;										/** @brief	͸������	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType)) {															//��������͸������													
					LimitInfo transmitLimitInfo = curLimitInfo;
					transmitLimitInfo.MinusTransmitLimit();
					PathNode transmit_pathnode(transmitLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* transmit_treenode = new RayTreeNode(transmit_pathnode);
					transmit_treenode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = transmit_treenode;
					temp_node = temp_node->m_pRight;
					stack.push({ temp_node });
				}
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType)) {//��������͸������
					PathNode empiricalTransmitPathNode(curLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* empiricalTransmitTreeNode = new RayTreeNode(empiricalTransmitPathNode);
					empiricalTransmitTreeNode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = empiricalTransmitTreeNode;
					temp_node = temp_node->m_pRight;
					stack.push({ temp_node });
				}
			}

			//����������ƣ���͸���Ȼд�����ӽڵ㣬����ֱ��д�����ֽڵ�
			if (curEndStopInfo.EndStopDiffract) {							//��������ֹ��д��������ֹ�ڵ�
				for (Wedge2D* wedeg : intersect.m_wedges) {
					PathNode end_pathnode(curLimitInfo, NODE_STOP, wedeg->m_point, wedeg, curRay);
					RayTreeNode* end_treenode = new RayTreeNode(end_pathnode);
					end_treenode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = end_treenode;
					temp_node = temp_node->m_pRight;
				}
			}
			else {															//�����������ڵ�
				LimitInfo diffractLimitInfo = curLimitInfo;
				diffractLimitInfo.MinusDiffractLimit();
				for (Wedge2D* wedeg : intersect.m_wedges) {
					std::vector<Ray2D> diffract_rays;
					if (GenerateDiffractRays(curRay, wedeg, &diffract_rays)) {
						for (int i = 0; i < diffract_rays.size(); i++) {
							PathNode diffract_node(diffractLimitInfo, NODE_DIFF, wedeg->m_point, wedeg, curRay, diffract_rays[i]);
							RayTreeNode* diffract_treenode = new RayTreeNode(diffract_node);
							diffract_treenode->SetGeneralFatherNode(curNode);
							temp_node->m_pRight = diffract_treenode;
							temp_node = temp_node->m_pRight;
							stack.push({ temp_node });
						}
					}
					else {					//��������������·������д��������ֹ�ڵ�
						for (Wedge2D* wedeg : intersect.m_wedges) {
							PathNode end_pathnode(curLimitInfo, NODE_STOP, wedeg->m_point, wedeg, curRay);
							RayTreeNode* end_treenode = new RayTreeNode(end_pathnode);
							end_treenode->SetGeneralFatherNode(curNode);
							temp_node->m_pRight = end_treenode;
							temp_node = temp_node->m_pRight;
						}
					}
				}
			}
			continue;
		}
		
		if (intersect.m_type == NODE_SCAT) {
			//��һ���ڵ�Ϊ�Ӷ��ӽڵ�
			//�����ڵ�Ϊ�Ӷ��ӽڵ��sibling�ڵ�
			continue;
		}
	}
}

//���ӿ��ǣ��ڶ�λģʽ�еļ��ֹ���
//����1-͸��������ٽ��з��䡢����  ����ƶ�
//����2-͸����������߽��з���	   ����ƶ�	
//����3-������������͸��		   ����ƶ�
void PathTraceLBS(bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, RayTreeNode*& vroot) {
	struct StackItem {
		RayTreeNode* node;			//��ǰ�ڵ�
		RayTreeNode* fatherNode;	//��ǰ�ڵ�ĸ��ڵ�
	};
	const LimitInfo limitInfo = vroot->m_data.m_limitInfo;
	const Ray2D& initRay = vroot->m_data.m_nextRay;
	EndStopInfo endStopInfo = limitInfo.CalEndStopInfo();
	//��ʼ��ջ�ռ�
	std::stack<StackItem> stack;
	vroot->m_pGeneralFather = vroot;
	stack.push({ vroot, nullptr });


	while (!stack.empty()) {//����ѭ��ֱ��ջΪ��
		StackItem current_item = stack.top();
		stack.pop();

		RayTreeNode* curNode = current_item.node;						/** @brief	��ǰ�ڵ�	*/
		RayTreeNode* fatherNode = current_item.fatherNode;				/** @brief	��ǰ�ڵ�ĸ��ڵ�	*/
		PATHNODETYPE fatherNodeType = NODE_INIT;						/** @brief	��ǰ�ڵ�ĸ��ڵ�Ľڵ�����	*/
		if (fatherNode != nullptr) {
			fatherNodeType = fatherNode->m_data.m_type;
		}

		Ray2D& curRay = curNode->m_data.m_nextRay;
		LimitInfo curLimitInfo = curNode->m_data.m_limitInfo;
		EndStopInfo curStopInfo = curLimitInfo.CalEndStopInfo();

		if (!curNode->m_pGeneralFather->m_isValid) {						//��ջ�нڵ�Ĺ��常�ڵ�Ϊ��Ч�ڵ㣬���������ѹջ����·��
			continue;
		}

		bool IsBlockByEnvironment = false;//�����Ƿ񱻻����ڵ�
		bool rayTubeHasWedges = false;//���߹����Ƿ���������
		bool IsRaySplit = false; //�����Ƿ���з�������
		RtLbsType t;//���ڼ������߷�������
		bool hasLeftChild = false;//��ǰ�ڵ��Ƿ�������ӽڵ㣬Ĭ�ϲ��������ӽڵ�
		RayTreeNode* temp_node = curNode;//��ʱдָ��ڵ�

		Intersection2D intersect;
		IsBlockByEnvironment = scene->GetIntersect(curRay, &intersect);//���������뻷�����ཻ�ж�
		if (IsBlockByEnvironment) {																									//�ڵ����߷���ģ��+intersect ����ģ��
			if (!intersect.Update(curRay)) {//��֤��ǰ�ڵ��Ƿ���֮ǰ�Ľڵ��д��ڹ�����㣬�����ڣ������׷�ٸ���·������ֹ�ظ�����·��
				continue;
			}
			t = intersect.m_ft;
		}
		else {																														//�Ӿ����߷���ģ��
			BBox2D bbox = scene->GetBBox();//��������볡���а�Χ�еĽ���,���������ǰ�ڵ���
			t = bbox.Intersect(curRay, nullptr);
		}

		//���ӹ���-͸�����߲����з�������
		int raySplitNum = 0;
		if (fatherNodeType != NODE_TRANIN &&
			fatherNodeType != NODE_TRANOUT &&
			fatherNodeType != NODE_ETRANIN &&
			fatherNodeType != NODE_ETRANOUT) {
			if (IsGenerateSplittingRays(curRay, t, raySplitFlag, raySplitRadius, raySplitNum)) {
				RayTreeNode* generalFatherNode = curNode->m_pGeneralFather;								//������常�ڵ�
				RayTreeNode* generalFatherRightNode = generalFatherNode->m_pRight;								//���常�ڵ�����ֽڵ���ʱ�洢
				temp_node = generalFatherNode;																	//��ȡ���常�ڵ�ֵ�����ڷ��ѽڵ�д��
				std::vector<Ray2D> splitRays;
				GenerateSplittingRay(generalFatherNode->m_data.m_nextRay, raySplitNum, &splitRays);
				generalFatherNode->m_isValid = false;															//�ѷ��ѵĹ��常�ڵ�����Ϊ��Ч�ڵ�
				for (auto it = splitRays.begin(); it != splitRays.end(); ++it) {
					Ray2D& splitRay = *it;
					PathNode splitPathNode(generalFatherNode->m_data);
					splitPathNode.m_nextRay = splitRay;
					RayTreeNode* splitTreeNode = new RayTreeNode(splitPathNode);
					splitTreeNode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = splitTreeNode;
					temp_node = temp_node->m_pRight;
					stack.push({ splitTreeNode, fatherNode });
				}
				temp_node->m_pRight = generalFatherRightNode;
				continue;
			}
		}
		

		EndStopInfo curEndStopInfo = curLimitInfo.CalEndStopInfo();								//��ȡ��ǰֹͣ��Ϣ

		//-----------------------------------------��������������������-----------------------------------------------------------------------------------------------
		if (!curEndStopInfo.EndStopDiffract) {//�����������ԣ���Ѱ�ҹ��ڵ���������
			rayTubeHasWedges = scene->GetRayTubeWedges(curRay, curNode, &intersect);//���߹����뻷��wedge���ཻ�ж�
		}

		//-----------------------------------------�Ӿ���㲿��-----------------------------------------------------------------------------------------------
		if (IsBlockByEnvironment == false) {//�������ڵ���д���Ӿ�ڵ�
			Point2D bbox_inter_point = GetRayCoordinate(curRay, t);
			PathNode pathnode(curLimitInfo, NODE_LOS, bbox_inter_point, curRay);
			RayTreeNode* loss_treenode = new RayTreeNode(pathnode);
			temp_node->m_pLeft = loss_treenode;
			temp_node = temp_node->m_pLeft;
			hasLeftChild = true;
			if (rayTubeHasWedges == false) {//������������������ִ�к�������
				continue;
			}
		}
		//-----------------------------------------�ڵ����㲿��----------------------------------------------------------------------------------------------	
		const PropagationProperty& propagationProperty = intersect.m_segment->m_propagationProperty;							//��ȡ������Ϣ�е���Ԫ��������״̬

		if (intersect.m_type == NODE_REFL) {
			if (curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {								//�޷�����͸, д����ֹ�ڵ�
				PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);
				RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
				endTreeNode->SetGeneralFatherNode(curNode);
				if (hasLeftChild) {
					temp_node->m_pRight = endTreeNode;
					temp_node = temp_node->m_pRight;
				}
				else {
					temp_node->m_pLeft = endTreeNode;
					temp_node = temp_node->m_pLeft;
				}
				hasLeftChild = true;
			}
			else if (!curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {						//�з�����͸��д�뷴��ڵ�
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//����ڵ�
					LimitInfo reflectLimitInfo = curLimitInfo;
					reflectLimitInfo.MinusReflectLimit();
					PathNode reflect_pathnode(reflectLimitInfo, NODE_REFL, intersect.m_intersect, intersect.m_segment, curRay, reflect_ray);
					RayTreeNode* reflect_treenode = new RayTreeNode(reflect_pathnode);
					reflect_treenode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = reflect_treenode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = reflect_treenode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
					stack.push({ temp_node, curNode });
				}
				else {				//���������˷������ߣ���д����ֹ�ڵ�
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //Ĭ��ֻ��һ���ڵ���Ҫ�޸�
					RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
					endTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = endTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = endTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
				}
			}
			else if (curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {						//�޷�����͸��д��͸��ڵ�
				Ray2D transmit_ray;
				PATHNODETYPE tranType;										/** @brief	͸������	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {//��������͸������,���������͸��											
					LimitInfo transmitLimitInfo = curLimitInfo;
					transmitLimitInfo.MinusTransmitLimit();
					PathNode transmit_pathnode(transmitLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* transmit_treenode = new RayTreeNode(transmit_pathnode);
					transmit_treenode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = transmit_treenode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = transmit_treenode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
					stack.push({ temp_node, curNode });
				}
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {//��������͸�����ߣ����������͸��
					PathNode empiricalTransmitPathNode(curLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* empiricalTransmitTreeNode = new RayTreeNode(empiricalTransmitPathNode);
					empiricalTransmitTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = empiricalTransmitTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = empiricalTransmitTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
					stack.push({ temp_node, curNode });
				}
				else {				//����������͸�����ߣ���д����ֹ�ڵ�
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);
					RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
					endTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = endTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = endTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
				}
			}
			else if (!curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {						//�з�����͸, д�뷴��ڵ㡢͸��ڵ�
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//����ڵ�
					LimitInfo reflectLimitInfo = curLimitInfo;
					reflectLimitInfo.MinusReflectLimit();
					PathNode reflect_pathnode(reflectLimitInfo, NODE_REFL, intersect.m_intersect, intersect.m_segment, curRay, reflect_ray);
					RayTreeNode* reflect_treenode = new RayTreeNode(reflect_pathnode);
					reflect_treenode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = reflect_treenode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = reflect_treenode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
					stack.push({ temp_node, curNode });
				}
				else {				//���������˷������ߣ���д����ֹ�ڵ�
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //Ĭ��ֻ��һ���ڵ���Ҫ�޸�
					RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
					endTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = endTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = endTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
				}

				Ray2D transmit_ray;
				PATHNODETYPE tranType;										/** @brief	͸������	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {															//��������͸������													
					LimitInfo transmitLimitInfo = curLimitInfo;
					transmitLimitInfo.MinusTransmitLimit();
					PathNode transmit_pathnode(transmitLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* transmit_treenode = new RayTreeNode(transmit_pathnode);
					transmit_treenode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = transmit_treenode;
					temp_node = temp_node->m_pRight;
					stack.push({ temp_node, curNode });
				}
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {//��������͸������
					PathNode empiricalTransmitPathNode(curLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* empiricalTransmitTreeNode = new RayTreeNode(empiricalTransmitPathNode);
					empiricalTransmitTreeNode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = empiricalTransmitTreeNode;
					temp_node = temp_node->m_pRight;
					stack.push({ temp_node, curNode });
				}
			}
			continue;
		}

		if (intersect.m_type == NODE_DIFF) {																										//������ڵ�-���Ӷ�wedge��������
			if (curEndStopInfo.EndStopDiffract) {//�����ֹ�ڵ�
				for (Wedge2D* wedeg : intersect.m_wedges) {
					PathNode end_pathnode(curLimitInfo, NODE_STOP, wedeg->m_point, wedeg, curRay);
					RayTreeNode* end_treenode = new RayTreeNode(end_pathnode);
					end_treenode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = end_treenode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = end_treenode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
				}
			}
			else {
				for (Wedge2D* wedeg : intersect.m_wedges) {
					std::vector<Ray2D> diffract_rays;
					if (GenerateDiffractRays(curRay, wedeg, &diffract_rays)) {
						LimitInfo diffractLimitInfo = curLimitInfo;
						diffractLimitInfo.MinusDiffractLimit();
						if (hasLeftChild) {//��ǰ�����ӽڵ��Ѿ���д�룬ֱ��д��ڵ�����ֽڵ㼴��
							for (int i = 0; i < diffract_rays.size(); i++) {
								PathNode diffract_node(diffractLimitInfo, NODE_DIFF, wedeg->m_point, wedeg, curRay, diffract_rays[i]);//ÿ���ڵ�洢ǰһ������
								RayTreeNode* diffract_treenode = new RayTreeNode(diffract_node);
								diffract_treenode->SetGeneralFatherNode(curNode);
								temp_node->m_pRight = diffract_treenode;
								temp_node = temp_node->m_pRight;
								stack.push({ temp_node, curNode });
							}
						}
						else {
							PathNode firstdiffract_node(diffractLimitInfo, NODE_DIFF, wedeg->m_point, wedeg, curRay, diffract_rays[0]);			//��һ������ڵ�Ϊ�ӽڵ�
							RayTreeNode* firstdiffract_treenode = new RayTreeNode(firstdiffract_node);
							firstdiffract_treenode->SetGeneralFatherNode(curNode);
							temp_node->m_pLeft = firstdiffract_treenode;
							temp_node = temp_node->m_pLeft;
							hasLeftChild = true;
							stack.push({ temp_node, curNode });
							for (int i = 1; i < diffract_rays.size(); i++) {
								PathNode diffract_node(diffractLimitInfo, NODE_DIFF, wedeg->m_point, wedeg, curRay, diffract_rays[i]);
								RayTreeNode* diffract_treenode = new RayTreeNode(diffract_node);
								diffract_treenode->SetGeneralFatherNode(curNode);
								temp_node->m_pRight = diffract_treenode;
								temp_node = temp_node->m_pRight;
								stack.push({ temp_node, curNode });
							}
						}
					}
					else {				//���޷������������ߣ���д��������ֹ�ڵ�
						for (Wedge2D* wedeg : intersect.m_wedges) {
							PathNode end_pathnode(curLimitInfo, NODE_STOP, wedeg->m_point, wedeg, curRay);
							RayTreeNode* end_treenode = new RayTreeNode(end_pathnode);
							end_treenode->SetGeneralFatherNode(curNode);
							if (!hasLeftChild) {
								temp_node->m_pRight = end_treenode;
								temp_node = temp_node->m_pRight;
							}
							else {
								temp_node->m_pLeft = end_treenode;
								temp_node = temp_node->m_pLeft;
								hasLeftChild = true;
							}
						}
					}
				}
			}
			continue;
		}

		if (intersect.m_type == NODE_REFLDIFF) {																						//����+����ڵ�
			if (curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {				//���1 �޷�����͸, ֱ��д����ֹ�ڵ�
				PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);
				RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
				endTreeNode->SetGeneralFatherNode(curNode);
				if (hasLeftChild) {
					temp_node->m_pRight = endTreeNode;
					temp_node = temp_node->m_pRight;
				}
				else {
					temp_node->m_pLeft = endTreeNode;
					temp_node = temp_node->m_pLeft;
				}
				hasLeftChild = true;
			}
			else if (!curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {		//���2 �з�����͸, д�뷴��ڵ�
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//����ڵ�
					LimitInfo reflectLimitInfo = curLimitInfo;
					reflectLimitInfo.MinusReflectLimit();
					PathNode reflect_pathnode(reflectLimitInfo, NODE_REFL, intersect.m_intersect, intersect.m_segment, curRay, reflect_ray);
					RayTreeNode* reflect_treenode = new RayTreeNode(reflect_pathnode);
					reflect_treenode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = reflect_treenode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = reflect_treenode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
					stack.push({ temp_node, curNode });
				}
				else {				//���������˷������ߣ���д����ֹ�ڵ�
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //Ĭ��ֻ��һ���ڵ���Ҫ�޸�
					RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
					endTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = endTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = endTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
				}
			}
			else if (curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {		//���3 �޷�����͸, д��͸��ڵ�
				Ray2D transmit_ray;
				PATHNODETYPE tranType;										/** @brief	͸������	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {//��������͸������													
					LimitInfo transmitLimitInfo = curLimitInfo;
					transmitLimitInfo.MinusTransmitLimit();
					PathNode transmit_pathnode(transmitLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* transmit_treenode = new RayTreeNode(transmit_pathnode);
					transmit_treenode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = transmit_treenode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = transmit_treenode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
					stack.push({ temp_node, curNode });
				}
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {//��������͸������
					PathNode empiricalTransmitPathNode(curLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* empiricalTransmitTreeNode = new RayTreeNode(empiricalTransmitPathNode);
					empiricalTransmitTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = empiricalTransmitTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = empiricalTransmitTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
					stack.push({ temp_node, curNode });
				}
				else {				//����������͸�����ߣ���д��͸����ֹ�ڵ�
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);
					RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
					endTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = endTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = endTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
				}
			}
			else if (!curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {		//���4 �з�����͸, ��д�뷴��ڵ㣬��д��͸��ڵ�
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//����ڵ�
					LimitInfo reflectLimitInfo = curLimitInfo;
					reflectLimitInfo.MinusReflectLimit();
					PathNode reflect_pathnode(reflectLimitInfo, NODE_REFL, intersect.m_intersect, intersect.m_segment, curRay, reflect_ray);
					RayTreeNode* reflect_treenode = new RayTreeNode(reflect_pathnode);
					reflect_treenode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = reflect_treenode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = reflect_treenode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
					stack.push({ temp_node, curNode });
				}
				else {				//���������˷������ߣ���д����ֹ�ڵ�
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //Ĭ��ֻ��һ���ڵ���Ҫ�޸�
					RayTreeNode* endTreeNode = new RayTreeNode(endPathNode);
					endTreeNode->SetGeneralFatherNode(curNode);
					if (hasLeftChild) {
						temp_node->m_pRight = endTreeNode;
						temp_node = temp_node->m_pRight;
					}
					else {
						temp_node->m_pLeft = endTreeNode;
						temp_node = temp_node->m_pLeft;
						hasLeftChild = true;
					}
				}

				Ray2D transmit_ray;
				PATHNODETYPE tranType;										/** @brief	͸������	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {															//��������͸������													
					LimitInfo transmitLimitInfo = curLimitInfo;
					transmitLimitInfo.MinusTransmitLimit();
					PathNode transmit_pathnode(transmitLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* transmit_treenode = new RayTreeNode(transmit_pathnode);
					transmit_treenode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = transmit_treenode;
					temp_node = temp_node->m_pRight;
					stack.push({ temp_node, curNode });
				}
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {//��������͸������
					PathNode empiricalTransmitPathNode(curLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* empiricalTransmitTreeNode = new RayTreeNode(empiricalTransmitPathNode);
					empiricalTransmitTreeNode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = empiricalTransmitTreeNode;
					temp_node = temp_node->m_pRight;
					stack.push({ temp_node, curNode });
				}
			}

			//����������ƣ���͸���Ȼд�����ӽڵ㣬����ֱ��д�����ֽڵ�
			if (curEndStopInfo.EndStopDiffract) {							//��������ֹ��д��������ֹ�ڵ�
				for (Wedge2D* wedeg : intersect.m_wedges) {
					PathNode end_pathnode(curLimitInfo, NODE_STOP, wedeg->m_point, wedeg, curRay);
					RayTreeNode* end_treenode = new RayTreeNode(end_pathnode);
					end_treenode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = end_treenode;
					temp_node = temp_node->m_pRight;
				}
			}
			else {															//�����������ڵ�
				LimitInfo diffractLimitInfo = curLimitInfo;
				diffractLimitInfo.MinusDiffractLimit();
				for (Wedge2D* wedeg : intersect.m_wedges) {
					std::vector<Ray2D> diffract_rays;
					if (GenerateDiffractRays(curRay, wedeg, &diffract_rays)) {
						for (int i = 0; i < diffract_rays.size(); i++) {
							PathNode diffract_node(diffractLimitInfo, NODE_DIFF, wedeg->m_point, wedeg, curRay, diffract_rays[i]);
							RayTreeNode* diffract_treenode = new RayTreeNode(diffract_node);
							diffract_treenode->SetGeneralFatherNode(curNode);
							temp_node->m_pRight = diffract_treenode;
							temp_node = temp_node->m_pRight;
							stack.push({ temp_node, curNode });
						}
					}
					else {					//��������������·������д��������ֹ�ڵ�
						for (Wedge2D* wedeg : intersect.m_wedges) {
							PathNode end_pathnode(curLimitInfo, NODE_STOP, wedeg->m_point, wedeg, curRay);
							RayTreeNode* end_treenode = new RayTreeNode(end_pathnode);
							end_treenode->SetGeneralFatherNode(curNode);
							temp_node->m_pRight = end_treenode;
							temp_node = temp_node->m_pRight;
						}
					}
				}
			}
			continue;
		}

		if (intersect.m_type == NODE_SCAT) {
			//��һ���ڵ�Ϊ�Ӷ��ӽڵ�
			//�����ڵ�Ϊ�Ӷ��ӽڵ��sibling�ڵ�
			continue;
		}
	}
}

bool PathTraceLite(RayPath& inpath) {
	//�������ߣ�������Ž�
	std::vector<Ray2D> splitRays; //��Ҫ���ѵ�����
	const std::vector<PathNode*>& nodes = inpath.m_nodes;

	PathNode* frontNode = nodes.front();
	PathNode* backNode = nodes.back();
	PathNode* secondNode = nodes[1];
	const Ray2D& init_ray = secondNode->m_prevRay;
	RtLbsType t = backNode->m_prevRay.m_fMax - backNode->m_prevRay.m_fMin + (backNode->m_point - backNode->m_prevRay.m_Ori).Length();//��������
	RtLbsType r = init_ray.GetRayRadius(t);
	int splitNum = static_cast<int>(ceil(r / TRAN_EPSILON));
	double theta = init_ray.m_theta * 2.0;
	double dtheta = theta / splitNum;
	double halftheta = dtheta / 2.0;
	double coshalftheta = cos(halftheta);
	Vector2D newDir = Rotate(init_ray.m_Dir, -init_ray.m_theta);//��ת��������Сֵ
	splitRays.resize(splitNum);
	//���ߵĳ�ʼ��
	newDir.Rotate(halftheta);//��һ������
	splitRays[0] = init_ray;
	splitRays[0].m_Dir = newDir;
	splitRays[0].m_theta = halftheta;
	splitRays[0].m_costheta = coshalftheta;

	for (size_t i = 1; i < splitRays.size(); ++i) {//�м�����
		newDir.Rotate(dtheta);
		splitRays[i] = init_ray;
		splitRays[i].m_Dir = newDir;
		splitRays[i].m_theta = halftheta;
		splitRays[i].m_costheta = coshalftheta;
	}

	std::vector<RayPath> outpaths;//�����·��
	//���㲢��������
	for (Ray2D& splitray : splitRays) {//������������
		RayPath newPath;
		newPath.m_nodes.push_back(frontNode);
		Ray2D iterateRay(splitray);//���ڵ���������
		for (auto it = next(nodes.begin()); it != prev(nodes.end()); ++it) {//�ٳ���βԪ�أ������ڵ㣬����������׷��
			PathNode* node = *it;

			Intersection2D inter;
			const Segment2D& segment = node->m_segment;
			if (!segment.GetIntersect(iterateRay, &inter)) {//����������Ԫ���ཻ���������ٸ�������
				break;
			}
			if (node->m_type == NODE_REFL) {//����ڵ�
				Ray2D reflectRay;
				if (!GenerateReflectRay(iterateRay, inter, &reflectRay))//���������˷���·�����������������
					break;
				PathNode* reflectNode = new PathNode(node->m_limitInfo, NODE_REFL, inter.m_intersect, node->m_segment, splitray, reflectRay);
				newPath.m_nodes.push_back(reflectNode);
				iterateRay = reflectRay;
			}
			if (node->m_type == NODE_TRANIN || node->m_type == NODE_TRANOUT) {//͸��ڵ�
				Ray2D transmitRay;
				PATHNODETYPE tranType;
				if (!GenerateTransmitRay(iterateRay, inter, &transmitRay, tranType))//����������͸��·�����������������
					break;
				PathNode* transmitNode = new PathNode(node->m_limitInfo, tranType, inter.m_intersect, node->m_segment, splitray, transmitRay);
				newPath.m_nodes.push_back(transmitNode);
				iterateRay = transmitRay;
			}
		}

		if (newPath.m_nodes.size() != (inpath.m_nodes.size() - 1))//�ٳ�ĩβԪ�غ����·����������ԭʼ����·������ͬ����������Ч·��
			continue;
		//if (!newPath.m_path.back().IsContainPointByAngle(backNode.m_point))
		//	continue;
		//����ǰ·����ȫ������·����
		//newPath.m_path.push_back(backNode);
		outpaths.push_back(newPath);
	}

	if (outpaths.empty())//����ǰ·������Ϊ�գ�����·��������
		return false;

	//��������rx�����·��
	RayPath targetPath;
	RtLbsType tmin = FLT_MAX;
	for (auto it = outpaths.begin(); it != outpaths.end(); ++it) {
		RayPath& cur_path = *it;
		PathNode* cur_node = cur_path.m_nodes.back();
		RtLbsType tcur = cur_node->m_nextRay.GetSquaredDistanceToPoint(backNode->m_point);
		if (tcur < tmin) {
			tmin = tcur;
			targetPath = cur_path;
		}

	}
	targetPath.m_nodes.push_back(backNode);
	inpath = targetPath;//�޸�path��ֵ
	return true;
}
