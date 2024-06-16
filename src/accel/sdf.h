#ifndef RTLBS_SDF
#define RTLBS_SDF

#include "rtlbs.h"
#include "utility/define.h"
#include "accelerator.h"
#include "gridid.h"
#include "accel/gpu/sdfgpu.h"



class SDFNode {
public:
	std::vector<Segment2D*> m_segemnts; /** @brief	节点中包含的面元	*/
	Point2D m_cornerPoint; /** @brief	网格左下角角点坐标	*/
	int64_t x, y;           /** @brief	节点的二维索引值，用于计算距离场	*/
	int64_t id;				/** @brief	节点的一维索引值	*/
	RtLbsType m_value; /** @brief	距离场值, 初始值为FLTMAX	*/
public:
	SDFNode();
	~SDFNode();
	bool operator<(const SDFNode& node) const; /** @brief	距离场反向比较，用于优先队列是最小堆	*/

	SDFNodeGPU Convert2GPU();
};



class SignedDistanceField :public Accelerator {
public:
	std::vector<SDFNode> m_voxels;		/** @brief	均匀网格	*/
	int64_t m_voxelNum[2];						/** @brief	网格数量	*/
	RtLbsType m_voxelExtent[2];				/** @brief	网格边长	*/
	RtLbsType m_voxelInvExtent[2];			/** @brief	网格边长的倒数	*/
	Point2D m_cornerPoint;					/** @brief	网格左下角的坐标	*/
	size_t m_voxelCount;						/** @brief	网格总数	*/
public:
	SignedDistanceField();
	~SignedDistanceField();
	void Build() override;
	bool GetIntersect(const Ray2D& ray, Intersection2D* intersect) const override;
	SignedDistanceFieldGPU Convert2GPU();// 将距离场转换为GPU上的距离场
	ACCEL_TYPE GetAccelType() const override;

private:
	std::vector<int64_t> m_source; /** @brief	距离场中的点源(包含有面元对象的网格)	*/

private:
	void _init();
	void _constructBasicGrid(std::vector<Segment2D*>& segments);		/** @brief	构建均匀网格	*/
	void _constructSDFGrid();									/** @brief	构建距离场	*/
	std::vector<int64_t> _getGridCoordAlongSegment(Segment2D& segment); //基于segment获得一系列的网格编号
	bool _getIntersect(int64_t id, const Ray2D& ray, Intersection2D* intersect) const;//射线与voxel求交
	int64_t _point2VoxelId(const Point2D& p, unsigned axis) const;
	int64_t _offset(int64_t x, int64_t y) const; //计算二维网格在一维数组之间的映射关系
};

#endif
