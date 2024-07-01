#ifndef RTLBS_UNIGRID
#define RTLBS_UNIGRID

#include "accelerator.h"

class UniGrid :public Accelerator {
public:
	UniGrid();
	~UniGrid();
	void Construct(std::vector<Segment2D*>& segments);
	void Build() override;
	bool GetIntersect(const Ray2D& ray, Intersection2D* intersect) const override;
	ACCEL_TYPE GetAccelType() const override;

private:
	std::vector<std::vector<Segment2D*>> m_pVoxels;
	int64_t m_voxelNum[2];
	RtLbsType m_voxelExtent[2];
	RtLbsType m_voxelInvExtent[2];
	size_t m_voxelCount;
	RtLbsType m_delta;


	void _init();//初始化数据
	void _release();//释放数据
	int64_t _point2VoxelId(const Point2D& p, unsigned axis) const;//坐标点转换为voxel
	Point2D _voxelId2Point(int64_t voxel[2]) const;//voxel 转换为坐标点
	int64_t _offset(int64_t x, int64_t y) const;//计算二维数组在一维数组之间的偏移量
	bool _getIntersect(const Ray2D& ray, Intersection2D* intersect, int64_t voxelId, RtLbsType nextT) const;//射线与voxel求交
};


#endif
