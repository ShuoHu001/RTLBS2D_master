#include "raypathgpu.h"

RayPathGPU::RayPathGPU()
	:m_bContainRefract(false)
{
}

RayPathGPU::RayPathGPU(std::vector<PathNodeGPU*> nodes, bool containRefract)
	: m_nodes(nodes)
	, m_bContainRefract(containRefract)
{
	//计算节点传播累积距离
	RtLbsType st = 0;
	for (int i = static_cast<int>(m_nodes.size()) - 1; i >= 0; --i) {
		if (i == 0) {									//第一个节点为真实路径的末尾节点，在GPU程序中未考虑到上一个交点到接收点的距离，因此需要重新计算距离
			st += (m_nodes[1]->m_inter.m_intersect - m_nodes[0]->m_inter.m_intersect).Length();
		}
		else {
			st += m_nodes[i]->m_inter.m_ft;
		}
		m_nodes[i]->m_ft = st;
	}
}

RayPathGPU::~RayPathGPU()
{
}

bool RayPathGPU::operator==(const RayPathGPU& path) const
{
	const std::vector<PathNodeGPU*>& nodesA = m_nodes;
	const std::vector<PathNodeGPU*>& nodesB = path.m_nodes;
	if (nodesA.size() != nodesB.size())
		return false;
	for (int i = 0; i < nodesA.size(); ++i) {
		if (nodesA[i]->m_inter.m_type != nodesB[i]->m_inter.m_type)
			return false;
		if (nodesA[i]->m_inter.m_type == NODE_REFL ||
			nodesA[i]->m_inter.m_type == NODE_TRANIN ||
			nodesA[i]->m_inter.m_type == NODE_TRANOUT || 
			nodesA[i]->m_inter.m_type == NODE_ETRANIN ||
			nodesA[i]->m_inter.m_type == NODE_ETRANOUT) {
			if (nodesA[i]->m_inter.m_segmentId != nodesB[i]->m_inter.m_segmentId)
				return false;
		}
		if (nodesA[i]->m_inter.m_type == NODE_DIFF) {
			if (nodesA[i]->m_inter.m_wedgeId != nodesB[i]->m_inter.m_wedgeId)
				return false;
		}
	}
	return true;
}

bool RayPathGPU::operator!=(const RayPathGPU& path) const
{
	return !(*this == path);
}

bool RayPathGPU::IsValidAndRectifyCommon(const Point2D& p, const Scene* scene, const Segment2DGPU* segments)//基于反向射线追踪算法进行修正
{
	//从末尾节点出发，逐步修正前面各个广义源的路径
	if (m_nodes.empty())
		return false;
	PathNodeGPU* endNode = m_nodes.front(); endNode->m_inter.m_intersect = p;//更新路径末尾节点的坐标，这里的“末尾”指的是正向路径的末尾
	
	Point2D endPoint = p;//末尾节点的坐标
	Ray2DGPU tRay;//用于相交测试的ray
	Intersection2DGPU testInter;//用于相交测试的intersect
	for (auto it = next(m_nodes.begin()); it != m_nodes.end(); ++it) {
		PathNodeGPU* curNode = *it;			/** @brief	当前节点	*/
		PathNodeGPU* prevNode = *prev(it);	/** @brief	前一个节点	*/
		if (curNode->m_inter.m_type == NODE_ROOT)//跳过开始节点
			continue;
		if (curNode->m_inter.m_type == NODE_DIFF) {//广义源节点
			endPoint = curNode->m_inter.m_intersect;
			continue;
		}
		if (curNode->m_inter.m_type == NODE_REFL) {//反射节点
			//求解广义源坐标
			Point2D vSource = prevNode->m_inter.GetVisualSource();
			tRay.m_Ori = vSource;
			tRay.m_Dir = (endPoint - vSource).Normalize();
			const Segment2DGPU* segment = &segments[curNode->m_inter.m_segmentId];
			if (!segment->GetIntersect(tRay, &testInter))//不存在交点，该条射线不存在
				return false;
			curNode->m_inter.m_intersect = testInter.m_intersect;//更新当前节点的交点信息
			endPoint = curNode->m_inter.m_intersect; //下一次迭代采用当前更新过的节点坐标
			continue;
		}
	}

	//20240521增加判定末尾和前节点构成的线段与环境相交，若相交且不是节点所在的面元，则路径无效
	Segment2D rayTestSegment(m_nodes[0]->m_inter.m_intersect, m_nodes[1]->m_inter.m_intersect);               /** @brief	用于检测节点是否与环境相交	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //若线段与环境相交，则路径无效
		return false;
	}

	return true;
}

bool RayPathGPU::IsValidAndRectifyRefractMixed(const Point2D& p, const Scene* scene, const Segment2DGPU* segments)
{
	//总体思路：
	// 1.将总路径分为常规路径组合透射路径组，分水岭为广义源；
	// 2.常规路径按照一般方法进行
	// 3.透射路径按照发射射线方法进行

	if (m_nodes.empty())
		return false;
	m_nodes.front()->m_inter.m_intersect = p; //更新路径末尾节点的坐标

	std::vector<RayPathGPU> splitPaths; //根据广义源分离的路径
	std::vector<PathNodeGPU*> nodes;
	bool containRefract = false;
	//1-将path按照广义源进行分段处理
	for (auto it = m_nodes.begin(); it != m_nodes.end(); ++it) {
		PathNodeGPU* node = *it;
		nodes.push_back(node);
		if (node->m_inter.m_type == NODE_DIFF) {//当前节点为广义源节点
			RayPathGPU path(nodes, containRefract);
			nodes.clear();
			nodes.push_back(node);//重置nodes并更新nodes中的初始值
			containRefract = false;
			continue;
		}
		if (next(it) == m_nodes.end()) {//是否到达序列的尾部
			RayPathGPU path(nodes, containRefract);
			splitPaths.push_back(path);
			nodes.clear();
			std::vector<PathNodeGPU*>().swap(nodes);
		}
		if (node->m_inter.m_type == NODE_TRANIN ||
			node->m_inter.m_type == NODE_TRANOUT ||
			node->m_inter.m_type == NODE_ETRANIN ||
			node->m_inter.m_type == NODE_ETRANOUT) {//当前节点为透射节点
			containRefract = true;
			continue;
		}
	}

	//2-针对不同的路径进行不同的处理方法，针对包含透射的路径，认为末尾节点为其需要优化的节点

	for (auto it = splitPaths.begin(); it != splitPaths.end(); ++it) {
		RayPathGPU& path = *it;
		PathNodeGPU* endNode = path.m_nodes.front();
		if (!path.m_bContainRefract) {//非透射路径
			if (!path.IsValidAndRectifyCommon(p, scene, segments))
				return false;
			continue;
		}
		//透射路径修正模块
		if (!path.IsValidAndRectifyRefract(segments))
			return false;
		continue;
	}

	//3- 将修正完成的路径进行拼接合成
	std::vector<PathNodeGPU*> newNodes;
	for (auto it = splitPaths.begin(); it != splitPaths.end();++it) {
		if (it != prev(splitPaths.end())) { //不是最后一段路径
			const RayPathGPU& newPath = *it;
			for (auto it1 = newPath.m_nodes.begin(); it1 != prev(newPath.m_nodes.end()); ++it1) {
				PathNodeGPU* node = *it1;
				newNodes.push_back(node);
			}
			continue;
		}
		//最后一段路径
		const RayPathGPU& newPath = *it;
		for (auto it1 = newPath.m_nodes.begin(); it1 != newPath.m_nodes.end(); ++it) {//最后一条路径保留最后一个元素
			PathNodeGPU* node = *it1;
			newNodes.push_back(node);
		}
	}
	m_nodes = newNodes;//更新路径，完成透射路径的修正

	//20240521增加判定末尾和前节点构成的线段与环境相交，若相交且不是节点所在的面元，则路径无效
	Segment2D rayTestSegment(m_nodes[0]->m_inter.m_intersect, m_nodes[1]->m_inter.m_intersect);               /** @brief	用于检测节点是否与环境相交	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //若线段与环境相交，则路径无效
		return false;
	}

	return true;
}


bool RayPathGPU::IsValidAndRectifyRefract(const Segment2DGPU* segments) {
	if (!PathTraceGPULite((*this), segments))
		return false;
	return true;
}

void GenerateMultiPathofRxSingleTxGPU(const std::vector<PathNodeGPU*>& nodes, const Point2D& txPosition, const std::vector<Point2D>& rxPositions, const Segment2DGPU* segments, const Scene* scene, std::vector<std::vector<RayPathGPU*>>& outRayPath) {
	PathNodeGPU* rootNode = new PathNodeGPU();//构建根节点
	rootNode->m_inter.m_intersect = txPosition;
	rootNode->m_inter.m_type = NODE_ROOT;
	std::vector<PathNodeGPU*> newNodes;
	int curRxId = -1;//初始化rx编号
	bool containReftact = false;
	for (int i = 0; i < nodes.size(); ++i) {
		newNodes.push_back(nodes[i]);
		if (nodes[i]->m_inter.m_type == NODE_TRANIN ||
			nodes[i]->m_inter.m_type == NODE_TRANOUT ||
			nodes[i]->m_inter.m_type == NODE_ETRANIN ||
			nodes[i]->m_inter.m_type == NODE_ETRANOUT) {
			containReftact = true;
		}
			
		if (nodes[i]->m_layer == 0) {//寻找到新增路径
			curRxId = nodes[i]->m_rxId;
			newNodes.push_back(rootNode);
			RayPathGPU* newPath = new RayPathGPU(newNodes, containReftact);
			if (!containReftact) {
				if (newPath->IsValidAndRectifyCommon(rxPositions[curRxId], scene, segments))
					outRayPath[curRxId].push_back(newPath);
			}
			else {
				if (newPath->IsValidAndRectifyRefractMixed(rxPositions[curRxId], scene, segments))
					outRayPath[curRxId].push_back(newPath);
			}
			newNodes.clear();//重置数据
			containReftact = false;
		}
	}
}

void GenerateMultiPathofRxGPU(std::vector<std::vector<PathNodeGPU*>> nodes, std::vector<Point2D> txPositions, std::vector<Point2D> rxPositions, Segment2DGPU* segments, const Scene* scene, std::vector<std::vector<std::vector<RayPathGPU*>>>& outRayPaths) {
	//基于输入的nodes，将有效的raypath提取出来
	//将提取出的数据写入至outRayPath中并进行校正处理
	for (int i = 0; i < nodes.size(); ++i) {//第一层循环为每个tx, outRayPaths中第一层为tx数量，outRayPaths第二层为rx数量，outRayPaths第三层为多径数量
		GenerateMultiPathofRxSingleTxGPU(nodes[i], txPositions[i], rxPositions, segments, scene, outRayPaths[i]);
	}
	////将多径文件写入到文件
	//for (int i = 0; i < txPositions.size(); ++i) {
	//	for (int j = 0; j < rxPositions.size(); ++j) {
	//		std::stringstream filename;
	//		filename << "path-tx" << i + 1 << "-rx" << j + 1 << ".txt";
	//		std::ofstream outFile(filename.str());
	//		if (outFile.is_open()) {
	//			for (int k = 0; k < outRayPaths[i][j].size(); ++k) {
	//				RayPathGPU& newPath = outRayPaths[i][j][k];
	//				outFile << newPath.m_nodes.size() << ",";
	//				for (int m = static_cast<int>(newPath.m_nodes.size()) - 1; m >= 0; --m) {
	//					Intersection2DGPU& inter = newPath.m_nodes[m].m_inter;
	//					outFile << inter.m_intersect.x << "," << inter.m_intersect.y << ",";
	//				}
	//				outFile << std::endl;
	//			}
	//			outFile.close();
	//		}
	//	}
	//}
}
