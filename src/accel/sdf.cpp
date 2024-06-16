#include "sdf.h"
#include <fstream>
#include <queue>

SDFNode::SDFNode()
	: x(-1)
	, y(-1)
	, id(-1)
	, m_value(FLT_MAX)
{
}

SDFNode::~SDFNode()
{
}

bool SDFNode::operator<(const SDFNode& node) const
{
	return m_value > node.m_value;
}

SDFNodeGPU SDFNode::Convert2GPU()
{
	SDFNodeGPU nodeGPU;
	nodeGPU.id = id;
	nodeGPU.x = x;
	nodeGPU.y = y;
	nodeGPU.m_segmentCount = m_segemnts.size();
	if (!m_segemnts.empty()) {
		nodeGPU.m_segmentIds = new int64_t[nodeGPU.m_segmentCount];
		for (size_t i = 0; i < nodeGPU.m_segmentCount; ++i) {
			nodeGPU.m_segmentIds[i] = m_segemnts[i]->m_id;
		}
	}
	nodeGPU.m_cornerPoint = m_cornerPoint;
	nodeGPU.m_value = m_value;
	return nodeGPU;
}

SignedDistanceField::SignedDistanceField()
{
	_init();
}

SignedDistanceField::~SignedDistanceField()
{
}

void SignedDistanceField::Build()
{
	_constructBasicGrid(*m_segments); //构建基础均匀网格
	_constructSDFGrid();//构建距离场
}

bool SignedDistanceField::GetIntersect(const Ray2D& ray, Intersection2D* intersect) const
{
	if (m_voxelCount == 0 || m_segments == 0)
		return false;
	RtLbsType maxt;
	RtLbsType cur_t = Intersect_BBox2D(ray, m_bbox, &maxt);
	if (cur_t < 0.0)
		return false;
	if (cur_t == maxt)
		cur_t = 0;//射线在框内
	if (intersect)
		intersect->m_ft = std::min(intersect->m_ft, maxt);

	int64_t curGrid[2], dir[2];
	RtLbsType delta[2], next[2];

	for (unsigned i = 0; i < 2; i++) {
		curGrid[i] = _point2VoxelId(ray(cur_t), i);
		dir[i] = (ray.m_Dir[i] > 0.0) ? 1 : -1;
		if (ray.m_Dir[i] != 0.0)
			delta[i] = abs(m_voxelExtent[i] / ray.m_Dir[i]);
		else
			delta[i] = FLT_MAX;
	}

	int64_t voxelId = _offset(curGrid[0], curGrid[1]);
	Point2D curGridCorner = m_voxels[voxelId].m_cornerPoint;
	for (unsigned i = 0; i < 2; i++) {
		// get the next t
		RtLbsType target = curGridCorner[i] + ((dir[i] + 1) >> 1) * m_voxelExtent[i];
		next[i] = (target - ray.m_Ori[i]) / ray.m_Dir[i];
		if (delta[i] == FLT_MAX)
			next[i] = FLT_MAX;
	}
	//遍历网格
	RtLbsType t_total = cur_t;
	const unsigned array[] = { 0, 0, 1, 1 };//[0],[3]不可能相交
	RtLbsType jumpThreshold = 10 * m_voxelExtent[0];
	RtLbsType tracebackThrershold = 5 * m_voxelExtent[0];
	while (intersect && cur_t < intersect->m_ft || (intersect == 0)) {

		//得到体素ID curGrid
		voxelId = _offset(curGrid[0], curGrid[1]);
		RtLbsType distanceToObstacle = m_voxels[voxelId].m_value;
		if (distanceToObstacle != 0 && distanceToObstacle > jumpThreshold) {
			distanceToObstacle -= tracebackThrershold;
			cur_t += distanceToObstacle; //先更新cur_t的值
			//跳转至体素网格,附带更新迭代部分内容
			GridId nextGrid;
			for (unsigned i = 0; i < 2; ++i) {
				nextGrid[i] = _point2VoxelId(ray(cur_t), i);
				next[i] += delta[i] * abs(nextGrid[i] - curGrid[i]);
				if (nextGrid[i] < 0 || (unsigned)nextGrid[i] >= m_voxelNum[i])
					return (intersect && intersect->m_ft < maxt && (intersect->m_segment != 0));
				curGrid[i] = nextGrid[i];
			}
		}
		voxelId = _offset(curGrid[0], curGrid[1]);
		if (_getIntersect(voxelId, ray, intersect))
			return true;
		//if (distanceToObstacle <= next[nextAxis]) {
		//	if (_getIntersect(curGrid, ray, intersect))
		//		return true;
		//}


		//计算下一个t值
		unsigned nextAxis = (next[0] <= next[1]) + ((unsigned)(next[1] <= next[0])) * 2;
		nextAxis = array[nextAxis];

		// get to the next voxel
		curGrid[nextAxis] += dir[nextAxis];

		if (curGrid[nextAxis] < 0 || (unsigned)curGrid[nextAxis] >= m_voxelNum[nextAxis])
			return (intersect && intersect->m_ft < maxt && (intersect->m_segment != 0));

		//更新下一个单元
		cur_t = next[nextAxis];
		next[nextAxis] += delta[nextAxis];
	}

	return (intersect && intersect->m_ft < maxt && (intersect->m_segment != 0));
}

SignedDistanceFieldGPU SignedDistanceField::Convert2GPU()
{

	SignedDistanceFieldGPU sdfGPU;
	sdfGPU.m_bbox = m_bbox.Convert2GPU();
	for (unsigned i = 0; i < 2; ++i) {
		sdfGPU.m_voxelNum[i] = m_voxelNum[i];
		sdfGPU.m_voxelExtent[i] = m_voxelExtent[i];
		sdfGPU.m_voxelInvExtent[i] = m_voxelInvExtent[i];
	}
	sdfGPU.m_cornerPoint = m_cornerPoint;
	sdfGPU.m_voxelCount = m_voxelCount;
	sdfGPU.m_voxels = new SDFNodeGPU[m_voxelCount];
	for (size_t i = 0; i < m_voxelCount; ++i) {
		sdfGPU.m_voxels[i] = m_voxels[i].Convert2GPU();
	}
	return sdfGPU;
}

ACCEL_TYPE SignedDistanceField::GetAccelType() const
{
	return ACCEL_SDF;
}




void SignedDistanceField::_init()
{
	for (unsigned i = 0; i < 2; i++) {
		m_voxelNum[i] = 0;
		m_voxelExtent[i] = 0.0;
		m_voxelInvExtent[i] = 0.0;
	}
	m_voxelCount = 0;
}

void SignedDistanceField::_constructBasicGrid(std::vector<Segment2D*>& segments)
{
	//计算网格大小
	Vector2D delta = m_bbox.m_max - m_bbox.m_min;
	RtLbsType extent = delta.MaxComponent();
	size_t count = segments.size();
	RtLbsType gridPerDistance = static_cast<RtLbsType>(sqrt(count / extent));

	for (unsigned i = 0; i < 2; i++) {//x轴、y轴
		m_voxelNum[i] = static_cast<int64_t>(ceil(std::min(512.0, gridPerDistance * delta[i])));
		m_voxelInvExtent[i] = m_voxelNum[i] / delta[i];
		m_voxelExtent[i] = 1.0 / m_voxelInvExtent[i];
	}

	//网格初始化
	m_voxelCount = m_voxelNum[0] * m_voxelNum[1];
	m_cornerPoint = m_bbox.m_min;
	m_voxels.resize(m_voxelCount);
	int64_t k = 0;
	for (int64_t i = 0; i < m_voxelNum[0]; ++i) { //角点赋值
		for (int64_t j = 0; j < m_voxelNum[1]; ++j) {
			k = j * m_voxelNum[0] + i;
			m_voxels[k].m_cornerPoint[0] = m_cornerPoint[0] + i * m_voxelExtent[0];
			m_voxels[k].m_cornerPoint[1] = m_cornerPoint[1] + j * m_voxelExtent[1];
			m_voxels[k].x = i;
			m_voxels[k].y = j;
			m_voxels[k].id = k;
		}
	}
	

	//循环面元进行赋值
	for (auto it = segments.begin(); it != segments.end(); ++it) {
		Segment2D* segment = *it;
		std::vector<int64_t> Ids = _getGridCoordAlongSegment(*segment);
		m_source.insert(m_source.end(), Ids.begin(), Ids.end());//给距离场的点源赋值
		if (Ids.empty())
			LOG_WARNING << "empty vector" << ENDL;
		for (size_t i = 0; i < Ids.size(); ++i) {
			m_voxels[Ids[i]].m_segemnts.push_back(segment);
		}
	}


	std::ofstream outFile;
	outFile.open("UniGrid-1.txt");
	for (int64_t j = 0; j < m_voxelNum[1]; ++j) {
		for (int64_t i = 0; i < m_voxelNum[0]; ++i) {
			outFile << static_cast<int64_t>(m_voxels[j * m_voxelNum[0] + i].m_segemnts.size()) << " ";
		}
		outFile << "\n";
	}
	outFile.close();

}

void SignedDistanceField::_constructSDFGrid()
{
	//1-初始化源值
	for (auto it = m_source.begin(); it != m_source.end(); ++it) {
		const size_t& id = *it;
		m_voxels[id].m_value = 0.0; /** @brief	源点的场值初始化为0.0	*/
	}

	//2-通过优先队列迭代更新计算距离场
	std::priority_queue<SDFNode> nodeQueue;
	for (auto it = m_source.begin(); it != m_source.end(); ++it) {
		const size_t& id = *it;
		nodeQueue.push(m_voxels[id]);
	}

	//3-FastMarchingMethod 主循环
	while (!nodeQueue.empty()) {
		SDFNode current = nodeQueue.top();
		nodeQueue.pop();

		//检查周围的节点
		for (int dx = -1; dx <= 1; ++dx) {
			for (int dy = -1; dy <= 1; ++dy) {
				if (dx == 0 && dy == 0) 
					continue; //跳过本身节点

				int64_t newX = current.x + dx;
				int64_t newY = current.y + dy;

				//检查边界
				if (newX<0 || newX>(m_voxelNum[0] - 1) || newY<0 || newY>(m_voxelNum[1] - 1))
					continue; //禁止检查超过边界的数据
				//计算新的距离
				int64_t voxelId = _offset(newX, newY);
				SDFNode& nextNode = m_voxels[voxelId];
				if (nextNode.m_value == 0.0) /** @brief	下一个节点为新源节点，直接跳过，不进行距离计算	*/
					continue;
				RtLbsType newDist = current.m_value + (nextNode.m_cornerPoint - current.m_cornerPoint).Length();

				if (newDist < nextNode.m_value) { //若下一个节点的距离小于当前节点的距离，则跟新距离并将下一个节点纳入到队列中
					nextNode.m_value = newDist;
					nodeQueue.push(nextNode);
				}
			}
		}
	}


	//test 部分，将当前的距离场写入文件中

	std::ofstream outFile;
	outFile.open("SDF.txt");
	for (int64_t j = 0; j < m_voxelNum[1]; ++j) {
		for (int64_t i = 0; i < m_voxelNum[0]; ++i) {
			outFile << m_voxels[j* m_voxelNum[0] + i].m_value << " ";
		}
		outFile << "\n";
	}
	outFile.close();

}

std::vector<int64_t>  SignedDistanceField::_getGridCoordAlongSegment(Segment2D& segment)
{
	std::vector<int64_t>  Ids;

	////方法-1 直接对包围盒进行赋值
	//for (int i = sId.x; i <= eId.x; ++i) {
	//	for (int j = sId.y; j <= eId.y; ++j) {
	//		Ids.push_back(GridId(i, j));
	//	}
	//}

	//方法2-基于网格迭代算法进行赋值(精确)
	int64_t curGrid[2], endGrid[2], dir[2];//当前的网格编号、终止的网格编号、行进方向
	RtLbsType delta[2], next[2];
	RtLbsType maxt = segment.m_length;		/** @brief	网格追踪的最大距离	*/
	RtLbsType curt = 0;						/** @brief	当前的距离	*/
	Vector2D& s_dir = segment.m_dir;
	Point2D& sp = segment.m_ps;
	Point2D& ep = segment.m_pe;

	//计算单位增量值
	for (unsigned i = 0; i < 2; i++) {
		curGrid[i] = _point2VoxelId(sp, i);
		endGrid[i] = _point2VoxelId(ep, i);
		dir[i] = (s_dir[i] > 0.0) ? 1 : -1;
		if (s_dir[i] != 0.0)
			delta[i] = abs(m_voxelExtent[i] / s_dir[i]);
		else
			delta[i] = FLT_MAX;
	}

	//计算目标值
	int64_t voxelId = _offset(curGrid[0], curGrid[1]);
	Point2D& curGridCorner = m_voxels[voxelId].m_cornerPoint;
	for (unsigned i = 0; i < 2; i++) {
		// get the next t
		RtLbsType target = curGridCorner[i] + ((dir[i] + 1) >> 1) * m_voxelExtent[i];
		next[i] = (target - sp[i]) / s_dir[i];
		if (delta[i] == FLT_MAX)
			next[i] = FLT_MAX;
	}

	//遍历网格求解体素编号
	//遍历网格
	const unsigned array[] = { 0, 0, 1, 1 };//[0],[3]不可能相交
	while (curt < maxt) {
		voxelId = _offset(curGrid[0], curGrid[1]);
		Ids.push_back(voxelId);//将当前探测到的网格编号记录在vector中
		//计算下一个t值
		unsigned nextAxis = (next[0] <= next[1]) + ((unsigned)(next[1] <= next[0])) * 2;
		nextAxis = array[nextAxis];

		if (curt < 0) {
			LOG_ERROR << "_getGridCoordAlongSegment:" << "curt < 0" << CRASH;
		}

		// get to the next voxel
		curGrid[nextAxis] += dir[nextAxis];
		curt = next[nextAxis];
		next[nextAxis] += delta[nextAxis];
	}
	return Ids;
}


bool SignedDistanceField::_getIntersect(int64_t id, const Ray2D& ray, Intersection2D* intersect) const
{
	if (m_voxels[id].m_segemnts.empty())
		return false;
	if (intersect)
		intersect->m_ft = FLT_MAX;//重置intersect的值
	bool hasIntersect = false;
	Intersection2D curIntersect;//当前的交点
	for (auto it = m_voxels[id].m_segemnts.begin(); it != m_voxels[id].m_segemnts.end(); ++it) {
		Segment2D* segment = *it;
		if (segment->GetIntersectNoBBox(ray, &curIntersect)) {
			if (!intersect) //若intersect为nullptr直接返回true
				return true;
			if (curIntersect.m_ft < intersect->m_ft)
				*intersect = curIntersect;
			hasIntersect = true;
		}
	}
	return hasIntersect;
}

int64_t SignedDistanceField::_point2VoxelId(const Point2D& p, unsigned axis) const
{
	return std::min(static_cast<int64_t>(m_voxelNum[axis] - 1), static_cast<int64_t>((p[axis] - m_bbox.m_min[axis]) * m_voxelInvExtent[axis]));
}

int64_t SignedDistanceField::_offset(int64_t x, int64_t y) const
{
	return y * m_voxelNum[0] + x;
}
