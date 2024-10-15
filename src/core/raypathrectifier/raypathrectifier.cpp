#include "raypathrectifier.h"

bool RectifyRayPath(const Scene* scene, RayPath*& path, const Point2D& p)
{
	if (!path->m_bContainRefract) {
		return _isValidAndRectifyCommonRayPath(scene, path, p);
	}
	return _isValidAndRectifyRefractedRayPath(scene, path, p);
}

bool RectifyGPURayPath(const Scene* scene, RayPathGPU& path, const Point2D& p)
{
	if (!path.m_bContainRefract) {
		return _isValidAndRectifyCommonGPURayPath(scene, path, p);
	}
	return _isValidAndRectifyRefractedGPURayPath(scene, path, p);
}

bool _isValidAndRectifyCommonRayPath(const Scene* scene, RayPath*& path, const Point2D& p)
{
	//整体思路是从末尾节点出发，修正到前面各个广义源的路径
	if (path->m_nodes.empty())
		return false;
	PathNode*& endNode = path->m_nodes.back();
	endNode->m_point = p;//更新末尾节点的节点坐标
	endNode->m_type = NODE_STOP;
	Point2D endPoint = p;//末尾节点的坐标
	Ray2D t_ray;//用于相交测试的ray
	Intersection2D t_inter;//用于相交测试的intersect

	for (auto it = std::next(path->m_nodes.rbegin()); it != path->m_nodes.rend(); ++it) { //由于透射具有发散性，采用特殊方法进行校正，需要修改算法
		PathNode*& curNode = *it;
		if (curNode->m_type == NODE_ROOT)
			continue;
		if (curNode->m_type == NODE_DIFF) {//广义源节点
			endPoint = curNode->m_point;//更新末尾节点坐标，进行新一轮的迭代计算
			continue;
		}
		if (curNode->m_type == NODE_REFL) {//反射节点
			//1-求解广义源
			Point2D& vSource = curNode->m_source;
			t_ray.m_Ori = vSource;
			t_ray.m_Dir = (endPoint - vSource).Normalize();
			Segment2D* segment = curNode->m_segment;
			if (!segment->GetIntersect(t_ray, &t_inter))//不存在交点，该条射线不存在
				return false;
			curNode->m_point = t_inter.m_intersect;//更新当前节点的交点信息
			endPoint = curNode->m_point;//下一次迭代采用当前更新过的节点坐标
			continue;
		}
	}

	//20240521增加判定末尾和前节点构成的线段与环境相交，若相交且不是节点所在的面元，则路径无效
	Segment2D rayTestSegment(path->m_nodes[path->m_nodes.size() - 1]->m_point, path->m_nodes[path->m_nodes.size() - 2]->m_point);               /** @brief	用于检测节点是否与环境相交	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //若线段与环境相交，则路径无效
		return false;
	}

	return true;
}

bool _isValidAndRectifyRefractedRayPath(const Scene* scene, RayPath*& path, const Point2D& p)
{
	//总体思路：
	// 1.将总路径分为常规路径组合透射路径组，分水岭为广义源；
	// 2.常规路径按照一般方法进行
	// 3.透射路径按照发射射线方法进行

	if (path->m_nodes.empty())
		return false;
	path->m_nodes.back()->m_point = p;//更新末尾节点的节点坐标

	std::vector<RayPath*> splitpaths; //分离的路径
	std::vector<PathNode*> nodes;
	bool containRefract = false;
	//1-将path按照广义源进行分段
	for (auto it = path->m_nodes.begin(); it != path->m_nodes.end(); ++it) {
		PathNode*& node = *it;
		nodes.push_back(node);
		if (node->m_type == NODE_DIFF) { //当前节点为广义源节点，
			RayPath* splitPath = new RayPath(nodes, containRefract);
			splitpaths.push_back(splitPath);
			nodes.clear();
			nodes.push_back(node);//重置nodes
			containRefract = false;//重置透射路径包含状态
			continue;
		}
		if (std::next(it) == path->m_nodes.end()) {//尾部节点
			RayPath* splitPath = new RayPath(nodes, containRefract);
			splitpaths.push_back(splitPath);
			nodes.clear();
		}
		if (node->m_type == NODE_TRANOUT) {			//当且仅当在透射出时，该条路径才被认为是透射路径
			containRefract = true;
			continue;
		}

	}

	//2-将不同的路径进行不同的处理方法,针对包含透射的路径，认为末尾节点(STOP节点或者是DIFF节点)为其需要优化的节点

	for (auto it = splitpaths.begin(); it != splitpaths.end(); ++it) {
		RayPath*& curSplitPath = *it;
		PathNode*& endNode = path->m_nodes.back();
		if (!path->m_bContainRefract) {//非透射路径
			if (!_isValidAndRectifyCommonRayPath(scene, curSplitPath, p))//常规路径不满足条件直接退出处理程序
				return false;
			continue;
		}
		//透射路径修正模块
		if (!PathTraceLite(path)) {
			return false;
		}
		continue;
	}

	//3-将修正完成的路径进行拼接合成
	std::vector<PathNode*> newNodes;
	for (RayPath*& newPath : splitpaths) {
		if (&newPath != &splitpaths.back()) {//不是最后一段路径
			for (auto it = newPath->m_nodes.begin(); it != prev(newPath->m_nodes.end()); ++it) {//舍弃最后一个元素
				PathNode* node = *it;
				newNodes.push_back(node);
			}
			continue;
		}
		//是最后一段路径
		for (auto it = newPath->m_nodes.begin(); it != newPath->m_nodes.end(); ++it) {//保留最后一个元素
			PathNode* node = *it;
			newNodes.push_back(node);
		}
	}

	path->m_nodes = newNodes;//更新路径，完成透射路径的修正

	//20240521增加判定末尾和前节点构成的线段与环境相交，若相交且不是节点所在的面元，则路径无效
	Segment2D rayTestSegment(path->m_nodes[path->m_nodes.size() - 1]->m_point, path->m_nodes[path->m_nodes.size() - 2]->m_point);               /** @brief	用于检测节点是否与环境相交	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //若线段与环境相交，则路径无效
		return false;
	}

	return true;
}

bool _isValidAndRectifyCommonGPURayPath(const Scene* scene, RayPathGPU& path, const Point2D& p)
{
	//从末尾节点出发，逐步修正前面各个广义源的路径
	if (path.m_nodes.empty())
		return false;
	PathNodeGPU* endNode = path.m_nodes.front(); endNode->m_inter.m_intersect = p;//更新路径末尾节点的坐标，这里的“末尾”指的是正向路径的末尾

	Point2D endPoint = p;//末尾节点的坐标
	Ray2DGPU tRay;//用于相交测试的ray
	Intersection2DGPU testInter;//用于相交测试的intersect
	for (auto it = std::next(path.m_nodes.begin()); it != path.m_nodes.end(); ++it) {
		PathNodeGPU* curNode = *it;			/** @brief	当前节点	*/
		PathNodeGPU* prevNode = *std::prev(it);	/** @brief	前一个节点	*/
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
			const Segment2DGPU* segment = &scene->m_gpuSegmentBuf[curNode->m_inter.m_segmentId];
			if (!segment->GetIntersect(tRay, &testInter))//不存在交点，该条射线不存在
				return false;
			curNode->m_inter.m_intersect = testInter.m_intersect;//更新当前节点的交点信息
			endPoint = curNode->m_inter.m_intersect; //下一次迭代采用当前更新过的节点坐标
			continue;
		}
	}

	//20240521增加判定末尾和前节点构成的线段与环境相交，若相交且不是节点所在的面元，则路径无效
	Segment2D rayTestSegment(path.m_nodes[0]->m_inter.m_intersect, path.m_nodes[1]->m_inter.m_intersect);               /** @brief	用于检测节点是否与环境相交	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //若线段与环境相交，则路径无效
		return false;
	}

	return true;
}

bool _isValidAndRectifyRefractedGPURayPath(const Scene* scene, RayPathGPU& path, const Point2D& p)
{
	//总体思路：
	// 1.将总路径分为常规路径组合透射路径组，分水岭为广义源；
	// 2.常规路径按照一般方法进行
	// 3.透射路径按照发射射线方法进行

	if (path.m_nodes.empty())
		return false;
	path.m_nodes.front()->m_inter.m_intersect = p; //更新路径末尾节点的坐标

	std::vector<RayPathGPU> splitPaths; //根据广义源分离的路径
	std::vector<PathNodeGPU*> nodes;
	bool containRefract = false;
	//1-将path按照广义源进行分段处理
	for (auto it = path.m_nodes.begin(); it != path.m_nodes.end(); ++it) {
		PathNodeGPU* node = *it;
		nodes.push_back(node);
		if (node->m_inter.m_type == NODE_DIFF) {//当前节点为广义源节点
			RayPathGPU newPath(nodes, containRefract);
			splitPaths.push_back(newPath);
			nodes.clear();
			nodes.push_back(node);//重置nodes并更新nodes中的初始值
			containRefract = false;
			continue;
		}
		if (next(it) == path.m_nodes.end()) {//是否到达序列的尾部
			RayPathGPU newPath(nodes, containRefract);
			splitPaths.push_back(newPath);
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
		RayPathGPU& splitPath = *it;
		PathNodeGPU* endNode = splitPath.m_nodes.front();
		if (!splitPath.m_bContainRefract) {//非透射路径
			if (!_isValidAndRectifyCommonGPURayPath(scene, splitPath, p))
				return false;
			continue;
		}
		//透射路径修正模块
		if (!PathTraceGPULite(splitPath, nullptr))
			return false;
		continue;
	}

	//3- 将修正完成的路径进行拼接合成
	std::vector<PathNodeGPU*> newNodes;
	for (auto it = splitPaths.begin(); it != splitPaths.end(); ++it) {
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
	path.m_nodes = newNodes;//更新路径，完成透射路径的修正

	//20240521增加判定末尾和前节点构成的线段与环境相交，若相交且不是节点所在的面元，则路径无效
	Segment2D rayTestSegment(path.m_nodes[0]->m_inter.m_intersect, path.m_nodes[1]->m_inter.m_intersect);               /** @brief	用于检测节点是否与环境相交	*/
	if (scene->GetIntersect(rayTestSegment, nullptr)) {         //若线段与环境相交，则路径无效
		return false;
	}

	return true;
}
