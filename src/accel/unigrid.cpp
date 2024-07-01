#include "unigrid.h"
#include <iostream>
#include <fstream>

UniGrid::UniGrid()
	: m_delta(0.01)
{
	_init();
}

UniGrid::~UniGrid()
{
	_release();
}

void UniGrid::Construct(std::vector<Segment2D*>& segments)
{
	if (segments.size() == 0) {
		LOG_WARNING << "no segments in uniform grid!" << ENDL;
		return;
	}
	//计算网格大小和数量
	Vector2D delta = m_bbox.m_max - m_bbox.m_min;
	RtLbsType extent = delta.MaxComponent();
	size_t count = segments.size();
	RtLbsType gridPerDistance = static_cast<RtLbsType>(sqrt(count / extent));

	for (unsigned i = 0; i < 2; i++) {//x轴、y轴
		m_voxelNum[i] = static_cast<unsigned>(std::ceil(std::min(256.0, gridPerDistance * delta[i])));
		m_voxelInvExtent[i] = m_voxelNum[i] / delta[i];
		m_voxelExtent[i] = 1.0 / m_voxelInvExtent[i];
	}

	//初始化网格
	m_voxelCount = m_voxelNum[0] * m_voxelNum[1];
	m_pVoxels.resize(m_voxelCount, std::vector<Segment2D*>());

	//将图元分配到网格,这里按照包围盒进行赋值
	// distribute the primitives
	for (auto it = segments.begin(); it != segments.end(); ++it) {
		Segment2D* segment = *it;
		int64_t maxGridId[2];
		int64_t minGridId[2];
		for (unsigned i = 0; i < 2; i++) // Only 2 dimensions
		{
			minGridId[i] = _point2VoxelId(segment->GetBBox().m_min, i);
			maxGridId[i] = _point2VoxelId(segment->GetBBox().m_max, i);
		}

		for (int64_t j = minGridId[1]; j <= maxGridId[1]; j++) {
			for (int64_t k = minGridId[0]; k <= maxGridId[0]; k++)
			{
				int64_t offset = _offset(k, j);
				m_pVoxels[offset].push_back(segment);
			}
		}
	}
	//将体素写入文件中（二维形式）
	std::ofstream outputFile("UniGrid.txt");
	if (outputFile.is_open()) {
		for (int64_t j = m_voxelNum[1] - 1; j > 0; --j) { //y 轴
			for (int64_t i = 0; i < m_voxelNum[0]; ++i) { //x 轴
				int64_t offset = _offset(i, j);
				if (m_pVoxels[offset].empty())
					outputFile << 0 << " ";
				else
					outputFile << 1 << " ";
			}
			outputFile << std::endl;
		}
	}
	outputFile.close();
}

void UniGrid::Build()
{
	Construct(*m_segments);
}

bool UniGrid::GetIntersect(const Ray2D& ray, Intersection2D* intersect) const
{
	if (m_pVoxels.size() == 0 || m_segments == 0)
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

	Point2D gridCorner = _voxelId2Point(curGrid);
	for (unsigned i = 0; i < 2; i++) {
		// get the next t
		RtLbsType target = gridCorner[i] + ((dir[i] + 1) >> 1) * m_voxelExtent[i];
		next[i] = (target - ray.m_Ori[i]) / ray.m_Dir[i];
	}

	//遍历网格
	const unsigned array[] = { 0, 0, 1, 1 };//[0],[3]不可能相交
	while (intersect && cur_t < intersect->m_ft || (intersect == 0)) {

		//得到体素ID
		int64_t voxelId = _offset(curGrid[0], curGrid[1]);

		//计算下一个t值
		unsigned nextAxis = (next[0] <= next[1]) + ((unsigned)(next[1] <= next[0])) * 2;
		nextAxis = array[nextAxis];

		if (_getIntersect(ray, intersect, voxelId, next[nextAxis]))
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

ACCEL_TYPE UniGrid::GetAccelType() const
{
	return ACCEL_UG;
}

void UniGrid::_init()
{
	for (unsigned i = 0; i < 2; i++) {
		m_voxelNum[i] = 0;
		m_voxelExtent[i] = 0.0;
		m_voxelInvExtent[i] = 0.0;
	}
	m_voxelCount = 0;
}

void UniGrid::_release()
{
	for (int i = 0; i < m_voxelCount; ++i) {
		m_pVoxels[i].clear();
		std::vector<Segment2D*>().swap(m_pVoxels[i]);
	}
	// 清空指针向量
	m_pVoxels.clear();
	std::vector<std::vector<Segment2D*>>().swap(m_pVoxels);
}

int64_t UniGrid::_point2VoxelId(const Point2D& p, unsigned axis) const
{
	return std::min(m_voxelNum[axis] - 1, (int64_t)((p[axis] - m_bbox.m_min[axis]) * m_voxelInvExtent[axis]));
}

Point2D UniGrid::_voxelId2Point(int64_t voxel[2]) const
{
	Point2D p;
	p.x = m_bbox.m_min.x + voxel[0] * m_voxelExtent[0];
	p.y = m_bbox.m_min.y + voxel[1] * m_voxelExtent[1];
	return p;
}

int64_t UniGrid::_offset(int64_t x, int64_t y) const
{
	return y * m_voxelNum[0] + x;
}

bool UniGrid::_getIntersect(const Ray2D& ray, Intersection2D* intersect, int64_t voxelId, RtLbsType nextT) const
{
	if (voxelId >= m_voxelCount)
		LOG_ERROR << "Voxel id is out of range.(" << voxelId << "/" << m_voxelCount << ")" << CRASH;
	bool hasIntersection = false;
	Intersection2D curintersect;
	for (auto it = m_pVoxels[voxelId].begin(); it != m_pVoxels[voxelId].end(); ++it) {
		Segment2D* segment = *it;
		if (segment->GetIntersect(ray, &curintersect)) {
			hasIntersection = true;
			if (curintersect.m_ft < intersect->m_ft)
				*intersect = curintersect;
			if (!intersect)
				return true;
		}
	}
	return hasIntersection;
}
