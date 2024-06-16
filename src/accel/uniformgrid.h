#ifndef RTLBS_UNIFORMGRID
#define RTLBS_UNIFORMGRID
#include "rtlbs.h"
#include "utility/define.h"
#include "accelerator.h"
#include "gridid.h"



class GridNode {
public:
	std::vector<Segment2D*> m_segemnts; /** @brief	节点中包含的面元	*/
	Point2D m_cornerPoint; /** @brief	网格左下角角点坐标	*/
	bool m_bHasContent; /** @brief	节点中是否包含面元	*/

public:
	GridNode();
	~GridNode();

};



class UniformGrid :public Accelerator {
public:
	std::vector<std::vector<GridNode>> m_voxels;		/** @brief	均匀网格	*/
	int64_t m_voxelNum[2];						/** @brief	网格数量	*/
	RtLbsType m_voxelExtent[2];				/** @brief	网格边长	*/
	RtLbsType m_voxelInvExtent[2];			/** @brief	网格边长的倒数	*/
	Point2D m_cornerPoint;					/** @brief	网格左下角的坐标	*/
	size_t m_voxelCount;						/** @brief	网格总数	*/
public:
	UniformGrid();
	~UniformGrid();
	void Build() override;
	bool GetIntersect(const Ray2D& ray, Intersection2D* intersect) const override;
	ACCEL_TYPE GetAccelType() const override;

private:
	void _init();
	void _construct(std::vector<Segment2D*>& segments);
	std::vector<GridId> _getGridCoordAlongSegment(Segment2D& segment); //基于segment获得一系列的网格编号
	bool _getIntersect(GridId id, const Ray2D& ray, Intersection2D* intersect) const;//射线与voxel求交
	int64_t _point2VoxelId(const Point2D& p, unsigned axis) const;
};

#endif
