#ifndef RTLBS_UNIFORMGRID
#define RTLBS_UNIFORMGRID
#include "rtlbs.h"
#include "utility/define.h"
#include "accelerator.h"
#include "gridid.h"



class GridNode {
public:
	std::vector<Segment2D*> m_segemnts; /** @brief	�ڵ��а�������Ԫ	*/
	Point2D m_cornerPoint; /** @brief	�������½ǽǵ�����	*/
	bool m_bHasContent; /** @brief	�ڵ����Ƿ������Ԫ	*/

public:
	GridNode();
	~GridNode();

};



class UniformGrid :public Accelerator {
public:
	std::vector<std::vector<GridNode>> m_voxels;		/** @brief	��������	*/
	int64_t m_voxelNum[2];						/** @brief	��������	*/
	RtLbsType m_voxelExtent[2];				/** @brief	����߳�	*/
	RtLbsType m_voxelInvExtent[2];			/** @brief	����߳��ĵ���	*/
	Point2D m_cornerPoint;					/** @brief	�������½ǵ�����	*/
	size_t m_voxelCount;						/** @brief	��������	*/
public:
	UniformGrid();
	~UniformGrid();
	void Build() override;
	bool GetIntersect(const Ray2D& ray, Intersection2D* intersect) const override;
	ACCEL_TYPE GetAccelType() const override;

private:
	void _init();
	void _construct(std::vector<Segment2D*>& segments);
	std::vector<GridId> _getGridCoordAlongSegment(Segment2D& segment); //����segment���һϵ�е�������
	bool _getIntersect(GridId id, const Ray2D& ray, Intersection2D* intersect) const;//������voxel��
	int64_t _point2VoxelId(const Point2D& p, unsigned axis) const;
};

#endif
