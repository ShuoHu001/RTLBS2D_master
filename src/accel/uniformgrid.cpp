#include "uniformgrid.h"
#include <fstream>

GridNode::GridNode()
	: m_bHasContent(false)
{
}

GridNode::~GridNode()
{
}

UniformGrid::UniformGrid()
{
	_init();
}

UniformGrid::~UniformGrid()
{
	for (auto& voxel : m_voxels) {
		voxel.clear();
		std::vector<GridNode>().swap(voxel);
	}
	m_voxels.clear();
	std::vector<std::vector< GridNode>>().swap(m_voxels);
}

void UniformGrid::Build()
{
	_construct(*m_segments);
}

bool UniformGrid::GetIntersect(const Ray2D& ray, Intersection2D* intersect) const
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

	GridId curGrid, dir;
	RtLbsType delta[2], next[2];

	for (unsigned i = 0; i < 2; i++) {
		curGrid[i] = _point2VoxelId(ray(cur_t), i);
		dir[i] = (ray.m_Dir[i] > 0.0) ? 1 : -1;
		if (ray.m_Dir[i] != 0.0)
			delta[i] = std::abs(m_voxelExtent[i] / ray.m_Dir[i]);
		else
			delta[i] = FLT_MAX;
	}
	Point2D curGridCorner = m_voxels[curGrid.x][curGrid.y].m_cornerPoint;
	for (unsigned i = 0; i < 2; i++) {
		// get the next t
		RtLbsType target = curGridCorner[i] + ((dir[i] + 1) >> 1) * m_voxelExtent[i];
		next[i] = (target - ray.m_Ori[i]) / ray.m_Dir[i];
		if (delta[i] == FLT_MAX)
			next[i] = FLT_MAX;
	}

	//遍历网格
	const unsigned array[] = { 0, 0, 1, 1 };//[0],[3]不可能相交
	while (intersect && cur_t < intersect->m_ft || (intersect == 0)) {

		//得到体素ID curGrid


		//计算下一个t值
		unsigned nextAxis = (next[0] <= next[1]) + ((unsigned)(next[1] <= next[0])) * 2;
		nextAxis = array[nextAxis];

		if (_getIntersect(curGrid, ray, intersect))
			return true;

		// get to the next voxel
		curGrid[nextAxis] += dir[nextAxis];

		if (curGrid[nextAxis] < 0 || (unsigned)curGrid[nextAxis] >= m_voxelNum[nextAxis])
			return (intersect && intersect->m_ft < maxt && (intersect->m_segment != 0));

		// update next
		cur_t = next[nextAxis];
		next[nextAxis] += delta[nextAxis];
	}

	return (intersect && intersect->m_ft < maxt && (intersect->m_segment != 0));
}

ACCEL_TYPE UniformGrid::GetAccelType() const
{
	return ACCEL_FastUG;
}




void UniformGrid::_init()
{
	for (unsigned i = 0; i < 2; i++) {
		m_voxelNum[i] = -1;
		m_voxelExtent[i] = 0.0;
		m_voxelInvExtent[i] = 0.0;
	}
	m_voxelCount = 0;
}

void UniformGrid::_construct(std::vector<Segment2D*>& segments)
{
	//计算网格大小
	Vector2D delta = m_bbox.m_max - m_bbox.m_min;
	RtLbsType extent = delta.MaxComponent();
	size_t count = segments.size();
	RtLbsType gridPerDistance = static_cast<RtLbsType>(sqrt(count / extent));

	for (unsigned i = 0; i < 2; i++) {//x轴、y轴
		m_voxelNum[i] = static_cast<int64_t>(std::ceil(std::min(256.0, gridPerDistance * delta[i])));
		m_voxelInvExtent[i] = m_voxelNum[i] / delta[i];
		m_voxelExtent[i] = 1.0 / m_voxelInvExtent[i];
	}

	//网格初始化
	m_cornerPoint = m_bbox.m_min;
	m_voxels.resize(m_voxelNum[0], std::vector<GridNode>(m_voxelNum[1]));
	for (int64_t i = 0; i < m_voxelNum[0]; ++i) { //角点赋值
		for (int64_t j = 0; j < m_voxelNum[1]; ++j) {
			m_voxels[i][j].m_cornerPoint[0] = m_cornerPoint[0] + i * m_voxelExtent[0];
			m_voxels[i][j].m_cornerPoint[1] = m_cornerPoint[1] + j * m_voxelExtent[1];
		}
	}
	m_voxelCount = m_voxelNum[0] * m_voxelNum[1];

	//循环面元进行赋值
	for (auto it = segments.begin(); it != segments.end(); ++it) {
		Segment2D* segment = *it;
		std::vector<GridId> Ids = _getGridCoordAlongSegment(*segment);
		if (Ids.empty())
			LOG_WARNING << "empty vector" << ENDL;
		for (size_t i = 0; i < Ids.size(); ++i) {
			int64_t x = Ids[i][0];
			int64_t y = Ids[i][1];
			m_voxels[x][y].m_segemnts.push_back(segment);
			m_voxels[x][y].m_bHasContent = true;
		}
	}

	std::ofstream outFile;
	outFile.open("UniGrid-1.txt");
	for (int64_t j = 0; j < m_voxelNum[1]; ++j) {
		for (int64_t i = 0; i < m_voxelNum[0]; ++i) {
			outFile << static_cast<int64_t>(m_voxels[i][j].m_segemnts.size())<<" ";
		}
		outFile << "\n";
	}
	outFile.close();

}

std::vector<GridId>  UniformGrid::_getGridCoordAlongSegment(Segment2D& segment)
{
	std::vector<GridId>  Ids;

	////方法-1 直接对包围盒进行赋值
	//for (int i = sId.x; i <= eId.x; ++i) {
	//	for (int j = sId.y; j <= eId.y; ++j) {
	//		Ids.push_back(GridId(i, j));
	//	}
	//}

	//方法2-基于网格迭代算法进行赋值(精确)
	GridId curGrid, endGrid, dir;//当前的网格编号、终止的网格编号、行进方向
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
	Point2D& curGridCorner = m_voxels[curGrid.x][curGrid.y].m_cornerPoint;
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
		Ids.push_back(curGrid);//将当前探测到的网格编号记录在vector中
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


bool UniformGrid::_getIntersect(GridId id, const Ray2D& ray, Intersection2D* intersect) const
{
	if (!m_voxels[id.x][id.y].m_bHasContent)
		return false;
	if (intersect)
		intersect->m_ft = FLT_MAX;//重置intersect的值
	bool hasIntersect = false;
	Intersection2D curIntersect;//当前的交点
	for (auto it = m_voxels[id.x][id.y].m_segemnts.begin(); it != m_voxels[id.x][id.y].m_segemnts.end(); ++it) {
		Segment2D* segment = *it;
		if (segment->GetIntersect(ray, &curIntersect)) {
			if (!intersect) //若intersect为nullptr直接返回true
				return true;
			if (curIntersect.m_ft < intersect->m_ft)
				*intersect = curIntersect;
			hasIntersect = true;
		}
	}
	return hasIntersect;
}

int64_t UniformGrid::_point2VoxelId(const Point2D& p, unsigned axis) const
{
	return std::min((int64_t)m_voxelNum[axis] - 1, (int64_t)((p[axis] - m_bbox.m_min[axis]) * m_voxelInvExtent[axis]));
}

