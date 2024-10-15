#ifndef RTLBS_SDFGPU
#define RTLBS_SDFGPU
#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/gpu/segment2dgpu.h"
#include "geometry/gpu/Intersection2Dgpu.h"
#include "geometry/gpu/bbox2dgpu.h"
#include "geometry/gpu/ray2dgpu.h"




class SDFNodeGPU {  //�����ΪCPUԤ�����ٸ�ֵģʽ
public:
	int64_t id;           /** @brief	�ڵ������ֵ�����ڼ�����볡	*/
	int64_t x, y;			/** @brief	�ڵ�Ķ�ά����ֵ	*/
	int64_t* m_segmentIds; /** @brief	�ڵ��а�������Ԫ	, �õ�ַΪGPU�ڴ����õ�ַ����ֹ���ڴ����*/
	size_t m_segmentCount;		/** @brief	�ڵ��е���Ԫ����	*/
	Point2D m_cornerPoint; /** @brief	�������½ǽǵ�����	*/
	RtLbsType m_value; /** @brief	���볡ֵ, ��ʼֵΪFLTMAX	*/
public:
	HOST_DEVICE_FUNC SDFNodeGPU();
	HOST_DEVICE_FUNC  SDFNodeGPU(int64_t* segmentIds, size_t segmentCount, Point2D cornerPoint, RtLbsType value, int64_t _id, int64_t _x, int64_t _y);
	HOST_DEVICE_FUNC  ~SDFNodeGPU();
	HOST_DEVICE_FUNC SDFNodeGPU& operator = (const SDFNodeGPU& node);
};



class SignedDistanceFieldGPU {
public:
	BBox2DGPU m_bbox;								/** @brief	������Ԫ�İ�Χ��	*/
	SDFNodeGPU* m_voxels;						/** @brief	��������	*/
	int64_t m_voxelNum[2];						/** @brief	��������	*/
	RtLbsType m_voxelExtent[2];					/** @brief	����߳�	*/
	RtLbsType m_voxelInvExtent[2];				/** @brief	����߳��ĵ���	*/
	Point2D m_cornerPoint;						/** @brief	�������½ǵ�����	*/
	size_t m_voxelCount;						/** @brief	��������	*/
	
private:
	SignedDistanceFieldGPU* device_sdfGPU = nullptr;
	int64_t** old_segmentIds; //ԭʼ��ÿ���ڵ��old_segmentIds�����ַ(CPU �ڴ��ַ)
	SDFNodeGPU* old_voxels; //ԭʼ��ÿ�����ص�old_voxel�����ַ(CPU �ڴ��ַ)

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
	HOST_DEVICE_FUNC bool _getIntersect(int64_t id, const Ray2DGPU& ray, Intersection2DGPU* intersect, Segment2DGPU* segmentsGPU) const;//������voxel��
	HOST_DEVICE_FUNC int64_t _point2VoxelId(const Point2D& p, unsigned axis) const;
	HOST_DEVICE_FUNC int64_t _offset(int64_t x, int64_t y) const; //��ö�ά��������voxel�ڵ�����ֵ
};

#endif
