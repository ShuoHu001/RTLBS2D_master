#ifndef RTLBS_SDFGPU
#define RTLBS_SDFGPU
#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/gpu/segment2dgpu.h"
#include "geometry/gpu/Intersection2Dgpu.h"
#include "geometry/gpu/bbox2dgpu.h"
#include "geometry/gpu/ray2dgpu.h"




class SDFNodeGPU {  //该类改为CPU预计算再赋值模式
public:
	int64_t id;           /** @brief	节点的索引值，用于计算距离场	*/
	int64_t x, y;			/** @brief	节点的二维索引值	*/
	int64_t* m_segmentIds; /** @brief	节点中包含的面元	, 该地址为GPU内存引用地址，禁止对内存操作*/
	size_t m_segmentCount;		/** @brief	节点中的面元数量	*/
	Point2D m_cornerPoint; /** @brief	网格左下角角点坐标	*/
	RtLbsType m_value; /** @brief	距离场值, 初始值为FLTMAX	*/
public:
	HOST_DEVICE_FUNC SDFNodeGPU();
	HOST_DEVICE_FUNC  SDFNodeGPU(int64_t* segmentIds, size_t segmentCount, Point2D cornerPoint, RtLbsType value, int64_t _id, int64_t _x, int64_t _y);
	HOST_DEVICE_FUNC  ~SDFNodeGPU();
	HOST_DEVICE_FUNC SDFNodeGPU& operator = (const SDFNodeGPU& node);
};



class SignedDistanceFieldGPU {
public:
	BBox2DGPU m_bbox;								/** @brief	场景面元的包围盒	*/
	SDFNodeGPU* m_voxels;						/** @brief	均匀网格	*/
	int64_t m_voxelNum[2];						/** @brief	网格数量	*/
	RtLbsType m_voxelExtent[2];					/** @brief	网格边长	*/
	RtLbsType m_voxelInvExtent[2];				/** @brief	网格边长的倒数	*/
	Point2D m_cornerPoint;						/** @brief	网格左下角的坐标	*/
	size_t m_voxelCount;						/** @brief	网格总数	*/
	
private:
	SignedDistanceFieldGPU* device_sdfGPU = nullptr;
	int64_t** old_segmentIds; //原始的每个节点的old_segmentIds数组地址(CPU 内存地址)
	SDFNodeGPU* old_voxels; //原始的每个体素的old_voxel数组地址(CPU 内存地址)

public:
	HOST_DEVICE_FUNC SignedDistanceFieldGPU();
	HOST_DEVICE_FUNC  SignedDistanceFieldGPU(BBox2DGPU bbox, SDFNodeGPU* voxels, size_t voxelNum[2], RtLbsType voxelExtent[2], RtLbsType voxelInvExtent[2], Point2D conorPoint, size_t voxelCount);
	HOST_DEVICE_FUNC  ~SignedDistanceFieldGPU();
	HOST_DEVICE_FUNC  SignedDistanceFieldGPU& operator = (const SignedDistanceFieldGPU& sdf);
	HOST_DEVICE_FUNC void GetIntersect(const Ray2DGPU& ray, Intersection2DGPU* intersect, Segment2DGPU* segmentsGPU);
	HOST_DEVICE_FUNC bool IsRayCaptureByWedgePoint(const Ray2DGPU& ray, Point2D& p, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments) const;
	void AllocateOnDevice(); //Allocate memory on GPU
	void ReleaseOnDevice();//Release the memory on GPU
	SignedDistanceFieldGPU* GetDevicePointer() const; // return the pointer to the device memory
private:
	HOST_DEVICE_FUNC bool _getIntersect(int64_t id, const Ray2DGPU& ray, Intersection2DGPU* intersect, Segment2DGPU* segmentsGPU) const;//射线与voxel求交
	HOST_DEVICE_FUNC int64_t _point2VoxelId(const Point2D& p, unsigned axis) const;
	HOST_DEVICE_FUNC int64_t _offset(int64_t x, int64_t y) const; //获得二维网格编号在voxel内的坐标值
};

#endif
