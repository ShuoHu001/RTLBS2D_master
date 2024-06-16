#include "pathtracing.h"

//增加考虑，在RT模式中的几种规则
//增加透射In和透射Out的判定，若前节点为透射In，那么不进行反射和绕射 -- 完成规则制定
//路径追踪模块
void PathTrace(bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, RayTreeNode*& vroot) {
	struct StackItem {
		RayTreeNode* node;			//当前节点
	};
	const LimitInfo limitInfo = vroot->m_data.m_limitInfo;
	const Ray2D& initRay = vroot->m_data.m_nextRay;
	EndStopInfo endStopInfo = limitInfo.CalEndStopInfo();
	//初始化栈空间
	std::stack<StackItem> stack;
	vroot->m_pGeneralFather = vroot;
	stack.push({ vroot});


	while (!stack.empty()) {//迭代循环直到栈为空
		StackItem current_item = stack.top();
		stack.pop();
		
		RayTreeNode* curNode = current_item.node;						/** @brief	当前节点	*/

		Ray2D& curRay = curNode->m_data.m_nextRay;
		LimitInfo curLimitInfo = curNode->m_data.m_limitInfo;
		EndStopInfo curStopInfo = curLimitInfo.CalEndStopInfo();

		if (!curNode->m_pGeneralFather->m_isValid) {						//若栈中节点的广义父节点为无效节点，则无需进行压栈计算路径
			continue;
		}

		bool IsBlockByEnvironment = false;//射线是否被环境遮挡
		bool rayTubeHasWedges = false;//射线管内是否包含绕射点
		bool IsRaySplit = false; //射线是否具有分裂特性
		RtLbsType t;//用于计算射线分裂条件
		bool hasLeftChild = false;//当前节点是否具有左子节点，默认不具有左子节点
		RayTreeNode* temp_node = curNode;//临时写指针节点

		Intersection2D intersect;
		IsBlockByEnvironment = scene->GetIntersect(curRay, &intersect);//单条射线与环境的相交判定
		if (IsBlockByEnvironment) {																									//遮挡射线分裂模组+intersect 修正模组
			if (!intersect.Update(curRay)) {//验证当前节点是否在之前的节点中存在过绕射点，若存在，则放弃追踪该条路径，防止重复绕射路径
				continue;
			}
			t = intersect.m_ft;
		}
		else {																														//视距射线分裂模组
			BBox2D bbox = scene->GetBBox();//求解射线与场景中包围盒的交点,并添加至当前节点下
			t = bbox.Intersect(curRay, nullptr);
		}

		int raySplitNum = 0;
		if (IsGenerateSplittingRays(curRay, t, raySplitFlag, raySplitRadius, raySplitNum)) {
			RayTreeNode* generalFatherNode = curNode->m_pGeneralFather;								//计算广义父节点
			RayTreeNode* generalFatherRightNode = generalFatherNode->m_pRight;								//广义父节点的右兄节点临时存储
			temp_node = generalFatherNode;																	//获取广义父节点值，用于分裂节点写入
			std::vector<Ray2D> splitRays;
			GenerateSplittingRay(generalFatherNode->m_data.m_nextRay, raySplitNum, &splitRays);
			generalFatherNode->m_isValid = false;															//已分裂的广义父节点设置为无效节点
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

		EndStopInfo curEndStopInfo = curLimitInfo.CalEndStopInfo();								//获取当前停止信息

		//-----------------------------------------管内绕射棱劈搜索部分-----------------------------------------------------------------------------------------------
		if (!curEndStopInfo.EndStopDiffract) {//若有绕射属性，则寻找管内的绕射棱劈
			rayTubeHasWedges = scene->GetRayTubeWedges(curRay, curNode, &intersect);//射线管内与环境wedge的相交判定
		}

		//-----------------------------------------视距计算部分-----------------------------------------------------------------------------------------------
		if (IsBlockByEnvironment == false) {//射线无遮挡，写入视距节点
			Point2D bbox_inter_point = GetRayCoordinate(curRay, t);
			PathNode pathnode(curLimitInfo, NODE_LOS, bbox_inter_point, curRay);
			RayTreeNode* loss_treenode = new RayTreeNode(pathnode);
			temp_node->m_pLeft = loss_treenode;
			temp_node = temp_node->m_pLeft;
			hasLeftChild = true;
			if (rayTubeHasWedges == false) {//管内无绕射棱劈，不执行后续操作
				continue;
			}
		}
		//-----------------------------------------遮挡计算部分----------------------------------------------------------------------------------------------	
		const PropagationProperty& propagationProperty = intersect.m_segment->m_propagationProperty;							//读取交点信息中的面元传播属性状态

		if (intersect.m_type == NODE_REFL) {
			if (curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {								//无反、无透, 写入终止节点
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
			else if (!curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {						//有反、无透，写入反射节点
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//反射节点
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
				else {				//若产生不了反射射线，则写入终止节点
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //默认只推一个节点需要修改
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
			else if (curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {						//无反、有透，写入透射节点
				Ray2D transmit_ray;
				PATHNODETYPE tranType;										/** @brief	透射类型	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType)) {//产生常规透射射线													
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
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType)) {//产生经验透射射线
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
				else {				//若产生不了透射射线，则写入终止节点
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
			else if (!curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {						//有反、有透, 写入反射节点、透射节点
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//反射节点
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
				else {				//若产生不了反射射线，则写入终止节点
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //默认只推一个节点需要修改
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
				PATHNODETYPE tranType;										/** @brief	透射类型	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType)) {															//产生常规透射射线													
					LimitInfo transmitLimitInfo = curLimitInfo;
					transmitLimitInfo.MinusTransmitLimit();
					PathNode transmit_pathnode(transmitLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* transmit_treenode = new RayTreeNode(transmit_pathnode);
					transmit_treenode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = transmit_treenode;
					temp_node = temp_node->m_pRight;
					stack.push({ temp_node });
				}
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType)) {//产生经验透射射线
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

		if (intersect.m_type == NODE_DIFF) {																										//单绕射节点-增加多wedge绕射的情况
			if (curEndStopInfo.EndStopDiffract) {//绕射截止节点
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
						if (hasLeftChild) {//当前的左子节点已经被写入，直接写入节点的右兄节点即可
							for (int i = 0; i < diffract_rays.size(); i++) {
								PathNode diffract_node(diffractLimitInfo, NODE_DIFF, wedeg->m_point, wedeg, curRay, diffract_rays[i]);//每个节点存储前一条射线
								RayTreeNode* diffract_treenode = new RayTreeNode(diffract_node);
								diffract_treenode->SetGeneralFatherNode(curNode);
								temp_node->m_pRight = diffract_treenode;
								temp_node = temp_node->m_pRight;
								stack.push({ temp_node });
							}
						}
						else {
							PathNode firstdiffract_node(diffractLimitInfo, NODE_DIFF, wedeg->m_point, wedeg, curRay, diffract_rays[0]);			//第一个绕射节点为子节点
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
					else {				//若无法产生绕射射线，则写入绕射终止节点
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

		if (intersect.m_type == NODE_REFLDIFF) {																						//反射+绕射节点
			if (curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {				//情况1 无反、无透, 直接写入终止节点
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
			else if (!curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {		//情况2 有反、无透, 写入反射节点
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//反射节点
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
				else {				//若产生不了反射射线，则写入终止节点
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //默认只推一个节点需要修改
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
			else if (curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {		//情况3 无反、有透, 写入透射节点
				Ray2D transmit_ray;
				PATHNODETYPE tranType;										/** @brief	透射类型	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType)) {//产生常规透射射线													
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
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType)) {//产生经验透射射线
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
				else {				//若产生不了透射射线，则写入透射终止节点
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
			else if(!curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {		//情况4 有反、有透, 先写入反射节点，再写入透射节点
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//反射节点
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
				else {				//若产生不了反射射线，则写入终止节点
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //默认只推一个节点需要修改
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
				PATHNODETYPE tranType;										/** @brief	透射类型	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType)) {															//产生常规透射射线													
					LimitInfo transmitLimitInfo = curLimitInfo;
					transmitLimitInfo.MinusTransmitLimit();
					PathNode transmit_pathnode(transmitLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* transmit_treenode = new RayTreeNode(transmit_pathnode);
					transmit_treenode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = transmit_treenode;
					temp_node = temp_node->m_pRight;
					stack.push({ temp_node });
				}
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType)) {//产生经验透射射线
					PathNode empiricalTransmitPathNode(curLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* empiricalTransmitTreeNode = new RayTreeNode(empiricalTransmitPathNode);
					empiricalTransmitTreeNode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = empiricalTransmitTreeNode;
					temp_node = temp_node->m_pRight;
					stack.push({ temp_node });
				}
			}

			//处理绕射机制，反透射必然写入左子节点，后续直接写入右兄节点
			if (curEndStopInfo.EndStopDiffract) {							//若绕射终止则写入绕射终止节点
				for (Wedge2D* wedeg : intersect.m_wedges) {
					PathNode end_pathnode(curLimitInfo, NODE_STOP, wedeg->m_point, wedeg, curRay);
					RayTreeNode* end_treenode = new RayTreeNode(end_pathnode);
					end_treenode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = end_treenode;
					temp_node = temp_node->m_pRight;
				}
			}
			else {															//否则产生绕射节点
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
					else {					//若产生不了绕射路径，则写入绕射终止节点
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
			//第一个节点为子儿子节点
			//后续节点为子儿子节点的sibling节点
			continue;
		}
	}
}

//增加考虑，在定位模式中的几种规则
//规则1-透射后不允许再进行反射、绕射  完成制定
//规则2-透射后不允许射线进行分裂	   完成制定	
//规则3-绕射后不允许进行透射		   完成制定
void PathTraceLBS(bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, RayTreeNode*& vroot) {
	struct StackItem {
		RayTreeNode* node;			//当前节点
		RayTreeNode* fatherNode;	//当前节点的父节点
	};
	const LimitInfo limitInfo = vroot->m_data.m_limitInfo;
	const Ray2D& initRay = vroot->m_data.m_nextRay;
	EndStopInfo endStopInfo = limitInfo.CalEndStopInfo();
	//初始化栈空间
	std::stack<StackItem> stack;
	vroot->m_pGeneralFather = vroot;
	stack.push({ vroot, nullptr });


	while (!stack.empty()) {//迭代循环直到栈为空
		StackItem current_item = stack.top();
		stack.pop();

		RayTreeNode* curNode = current_item.node;						/** @brief	当前节点	*/
		RayTreeNode* fatherNode = current_item.fatherNode;				/** @brief	当前节点的父节点	*/
		PATHNODETYPE fatherNodeType = NODE_INIT;						/** @brief	当前节点的父节点的节点类型	*/
		if (fatherNode != nullptr) {
			fatherNodeType = fatherNode->m_data.m_type;
		}

		Ray2D& curRay = curNode->m_data.m_nextRay;
		LimitInfo curLimitInfo = curNode->m_data.m_limitInfo;
		EndStopInfo curStopInfo = curLimitInfo.CalEndStopInfo();

		if (!curNode->m_pGeneralFather->m_isValid) {						//若栈中节点的广义父节点为无效节点，则无需进行压栈计算路径
			continue;
		}

		bool IsBlockByEnvironment = false;//射线是否被环境遮挡
		bool rayTubeHasWedges = false;//射线管内是否包含绕射点
		bool IsRaySplit = false; //射线是否具有分裂特性
		RtLbsType t;//用于计算射线分裂条件
		bool hasLeftChild = false;//当前节点是否具有左子节点，默认不具有左子节点
		RayTreeNode* temp_node = curNode;//临时写指针节点

		Intersection2D intersect;
		IsBlockByEnvironment = scene->GetIntersect(curRay, &intersect);//单条射线与环境的相交判定
		if (IsBlockByEnvironment) {																									//遮挡射线分裂模组+intersect 修正模组
			if (!intersect.Update(curRay)) {//验证当前节点是否在之前的节点中存在过绕射点，若存在，则放弃追踪该条路径，防止重复绕射路径
				continue;
			}
			t = intersect.m_ft;
		}
		else {																														//视距射线分裂模组
			BBox2D bbox = scene->GetBBox();//求解射线与场景中包围盒的交点,并添加至当前节点下
			t = bbox.Intersect(curRay, nullptr);
		}

		//增加规则-透射射线不具有分裂属性
		int raySplitNum = 0;
		if (fatherNodeType != NODE_TRANIN &&
			fatherNodeType != NODE_TRANOUT &&
			fatherNodeType != NODE_ETRANIN &&
			fatherNodeType != NODE_ETRANOUT) {
			if (IsGenerateSplittingRays(curRay, t, raySplitFlag, raySplitRadius, raySplitNum)) {
				RayTreeNode* generalFatherNode = curNode->m_pGeneralFather;								//计算广义父节点
				RayTreeNode* generalFatherRightNode = generalFatherNode->m_pRight;								//广义父节点的右兄节点临时存储
				temp_node = generalFatherNode;																	//获取广义父节点值，用于分裂节点写入
				std::vector<Ray2D> splitRays;
				GenerateSplittingRay(generalFatherNode->m_data.m_nextRay, raySplitNum, &splitRays);
				generalFatherNode->m_isValid = false;															//已分裂的广义父节点设置为无效节点
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
		

		EndStopInfo curEndStopInfo = curLimitInfo.CalEndStopInfo();								//获取当前停止信息

		//-----------------------------------------管内绕射棱劈搜索部分-----------------------------------------------------------------------------------------------
		if (!curEndStopInfo.EndStopDiffract) {//若有绕射属性，则寻找管内的绕射棱劈
			rayTubeHasWedges = scene->GetRayTubeWedges(curRay, curNode, &intersect);//射线管内与环境wedge的相交判定
		}

		//-----------------------------------------视距计算部分-----------------------------------------------------------------------------------------------
		if (IsBlockByEnvironment == false) {//射线无遮挡，写入视距节点
			Point2D bbox_inter_point = GetRayCoordinate(curRay, t);
			PathNode pathnode(curLimitInfo, NODE_LOS, bbox_inter_point, curRay);
			RayTreeNode* loss_treenode = new RayTreeNode(pathnode);
			temp_node->m_pLeft = loss_treenode;
			temp_node = temp_node->m_pLeft;
			hasLeftChild = true;
			if (rayTubeHasWedges == false) {//管内无绕射棱劈，不执行后续操作
				continue;
			}
		}
		//-----------------------------------------遮挡计算部分----------------------------------------------------------------------------------------------	
		const PropagationProperty& propagationProperty = intersect.m_segment->m_propagationProperty;							//读取交点信息中的面元传播属性状态

		if (intersect.m_type == NODE_REFL) {
			if (curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {								//无反、无透, 写入终止节点
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
			else if (!curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {						//有反、无透，写入反射节点
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//反射节点
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
				else {				//若产生不了反射射线，则写入终止节点
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //默认只推一个节点需要修改
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
			else if (curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {						//无反、有透，写入透射节点
				Ray2D transmit_ray;
				PATHNODETYPE tranType;										/** @brief	透射类型	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {//产生常规透射射线,绕射后不允许透射											
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
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {//产生经验透射射线，绕射后不允许透射
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
				else {				//若产生不了透射射线，则写入终止节点
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
			else if (!curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {						//有反、有透, 写入反射节点、透射节点
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//反射节点
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
				else {				//若产生不了反射射线，则写入终止节点
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //默认只推一个节点需要修改
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
				PATHNODETYPE tranType;										/** @brief	透射类型	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {															//产生常规透射射线													
					LimitInfo transmitLimitInfo = curLimitInfo;
					transmitLimitInfo.MinusTransmitLimit();
					PathNode transmit_pathnode(transmitLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* transmit_treenode = new RayTreeNode(transmit_pathnode);
					transmit_treenode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = transmit_treenode;
					temp_node = temp_node->m_pRight;
					stack.push({ temp_node, curNode });
				}
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {//产生经验透射射线
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

		if (intersect.m_type == NODE_DIFF) {																										//单绕射节点-增加多wedge绕射的情况
			if (curEndStopInfo.EndStopDiffract) {//绕射截止节点
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
						if (hasLeftChild) {//当前的左子节点已经被写入，直接写入节点的右兄节点即可
							for (int i = 0; i < diffract_rays.size(); i++) {
								PathNode diffract_node(diffractLimitInfo, NODE_DIFF, wedeg->m_point, wedeg, curRay, diffract_rays[i]);//每个节点存储前一条射线
								RayTreeNode* diffract_treenode = new RayTreeNode(diffract_node);
								diffract_treenode->SetGeneralFatherNode(curNode);
								temp_node->m_pRight = diffract_treenode;
								temp_node = temp_node->m_pRight;
								stack.push({ temp_node, curNode });
							}
						}
						else {
							PathNode firstdiffract_node(diffractLimitInfo, NODE_DIFF, wedeg->m_point, wedeg, curRay, diffract_rays[0]);			//第一个绕射节点为子节点
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
					else {				//若无法产生绕射射线，则写入绕射终止节点
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

		if (intersect.m_type == NODE_REFLDIFF) {																						//反射+绕射节点
			if (curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {				//情况1 无反、无透, 直接写入终止节点
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
			else if (!curEndStopInfo.EndStopReflect && curEndStopInfo.EndStopTransmit) {		//情况2 有反、无透, 写入反射节点
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//反射节点
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
				else {				//若产生不了反射射线，则写入终止节点
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //默认只推一个节点需要修改
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
			else if (curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {		//情况3 无反、有透, 写入透射节点
				Ray2D transmit_ray;
				PATHNODETYPE tranType;										/** @brief	透射类型	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {//产生常规透射射线													
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
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {//产生经验透射射线
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
				else {				//若产生不了透射射线，则写入透射终止节点
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
			else if (!curEndStopInfo.EndStopReflect && !curEndStopInfo.EndStopTransmit) {		//情况4 有反、有透, 先写入反射节点，再写入透射节点
				Ray2D reflect_ray;
				if (GenerateReflectRay(curRay, intersect, &reflect_ray)) {																//反射节点
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
				else {				//若产生不了反射射线，则写入终止节点
					PathNode endPathNode(curLimitInfo, NODE_STOP, intersect.m_intersect, intersect.m_segment, curRay);       //默认只推一个节点需要修改
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
				PATHNODETYPE tranType;										/** @brief	透射类型	*/
				if (GenerateTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {															//产生常规透射射线													
					LimitInfo transmitLimitInfo = curLimitInfo;
					transmitLimitInfo.MinusTransmitLimit();
					PathNode transmit_pathnode(transmitLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* transmit_treenode = new RayTreeNode(transmit_pathnode);
					transmit_treenode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = transmit_treenode;
					temp_node = temp_node->m_pRight;
					stack.push({ temp_node, curNode });
				}
				else if (GenerateEmpiricalTransmitRay(curRay, intersect, &transmit_ray, tranType) && fatherNodeType != NODE_DIFF) {//产生经验透射射线
					PathNode empiricalTransmitPathNode(curLimitInfo, tranType, intersect.m_intersect, intersect.m_segment, curRay, transmit_ray);
					RayTreeNode* empiricalTransmitTreeNode = new RayTreeNode(empiricalTransmitPathNode);
					empiricalTransmitTreeNode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = empiricalTransmitTreeNode;
					temp_node = temp_node->m_pRight;
					stack.push({ temp_node, curNode });
				}
			}

			//处理绕射机制，反透射必然写入左子节点，后续直接写入右兄节点
			if (curEndStopInfo.EndStopDiffract) {							//若绕射终止则写入绕射终止节点
				for (Wedge2D* wedeg : intersect.m_wedges) {
					PathNode end_pathnode(curLimitInfo, NODE_STOP, wedeg->m_point, wedeg, curRay);
					RayTreeNode* end_treenode = new RayTreeNode(end_pathnode);
					end_treenode->SetGeneralFatherNode(curNode);
					temp_node->m_pRight = end_treenode;
					temp_node = temp_node->m_pRight;
				}
			}
			else {															//否则产生绕射节点
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
					else {					//若产生不了绕射路径，则写入绕射终止节点
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
			//第一个节点为子儿子节点
			//后续节点为子儿子节点的sibling节点
			continue;
		}
	}
}

bool PathTraceLite(RayPath& inpath) {
	//构造射线，计算半张角
	std::vector<Ray2D> splitRays; //需要分裂的射线
	const std::vector<PathNode*>& nodes = inpath.m_nodes;

	PathNode* frontNode = nodes.front();
	PathNode* backNode = nodes.back();
	PathNode* secondNode = nodes[1];
	const Ray2D& init_ray = secondNode->m_prevRay;
	RtLbsType t = backNode->m_prevRay.m_fMax - backNode->m_prevRay.m_fMin + (backNode->m_point - backNode->m_prevRay.m_Ori).Length();//传播长度
	RtLbsType r = init_ray.GetRayRadius(t);
	int splitNum = static_cast<int>(ceil(r / TRAN_EPSILON));
	double theta = init_ray.m_theta * 2.0;
	double dtheta = theta / splitNum;
	double halftheta = dtheta / 2.0;
	double coshalftheta = cos(halftheta);
	Vector2D newDir = Rotate(init_ray.m_Dir, -init_ray.m_theta);//旋转到方向最小值
	splitRays.resize(splitNum);
	//射线的初始化
	newDir.Rotate(halftheta);//第一条射线
	splitRays[0] = init_ray;
	splitRays[0].m_Dir = newDir;
	splitRays[0].m_theta = halftheta;
	splitRays[0].m_costheta = coshalftheta;

	for (size_t i = 1; i < splitRays.size(); ++i) {//中间射线
		newDir.Rotate(dtheta);
		splitRays[i] = init_ray;
		splitRays[i].m_Dir = newDir;
		splitRays[i].m_theta = halftheta;
		splitRays[i].m_costheta = coshalftheta;
	}

	std::vector<RayPath> outpaths;//输出的路径
	//计算并跟踪射线
	for (Ray2D& splitray : splitRays) {//遍历所有射线
		RayPath newPath;
		newPath.m_nodes.push_back(frontNode);
		Ray2D iterateRay(splitray);//用于迭代的射线
		for (auto it = next(nodes.begin()); it != prev(nodes.end()); ++it) {//刨除首尾元素，遍历节点，并进行射线追踪
			PathNode* node = *it;

			Intersection2D inter;
			const Segment2D& segment = node->m_segment;
			if (!segment.GetIntersect(iterateRay, &inter)) {//若射线与面元不相交，放弃跟踪该条射线
				break;
			}
			if (node->m_type == NODE_REFL) {//反射节点
				Ray2D reflectRay;
				if (!GenerateReflectRay(iterateRay, inter, &reflectRay))//若产生不了反射路径，则放弃该条射线
					break;
				PathNode* reflectNode = new PathNode(node->m_limitInfo, NODE_REFL, inter.m_intersect, node->m_segment, splitray, reflectRay);
				newPath.m_nodes.push_back(reflectNode);
				iterateRay = reflectRay;
			}
			if (node->m_type == NODE_TRANIN || node->m_type == NODE_TRANOUT) {//透射节点
				Ray2D transmitRay;
				PATHNODETYPE tranType;
				if (!GenerateTransmitRay(iterateRay, inter, &transmitRay, tranType))//若产生不了透射路径，则放弃该条射线
					break;
				PathNode* transmitNode = new PathNode(node->m_limitInfo, tranType, inter.m_intersect, node->m_segment, splitray, transmitRay);
				newPath.m_nodes.push_back(transmitNode);
				iterateRay = transmitRay;
			}
		}

		if (newPath.m_nodes.size() != (inpath.m_nodes.size() - 1))//刨除末尾元素后的新路径若数量与原始射线路径不相同，则舍弃无效路径
			continue;
		//if (!newPath.m_path.back().IsContainPointByAngle(backNode.m_point))
		//	continue;
		//将当前路径补全并加入路径中
		//newPath.m_path.push_back(backNode);
		outpaths.push_back(newPath);
	}

	if (outpaths.empty())//若当前路径集合为空，表明路径不存在
		return false;

	//搜索距离rx最近的路径
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
	inpath = targetPath;//修改path的值
	return true;
}
