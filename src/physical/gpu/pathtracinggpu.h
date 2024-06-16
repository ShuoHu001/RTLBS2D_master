#ifndef RTLBS_PATHTRACINGGPU
#define RTLBS_PATHTRACINGGPU
#include "utility/define.h"
#include "managers/logmanager.h"

#include "tree/raytreenode.h"
#include "tree/pathnode.h"
#include "geometry/ray2d.h"
#include "geometry/scene.h"
#include "geometry/Intersection2D.h"
#include "geometry/bbox2d.h"

#include "physical/gpu/reflectiongpu.h"
#include "physical/gpu/transmissiongpu.h"
#include "physical/gpu/diffractiongpu.h"
#include "physical/gpu/raysplittinggpu.h"

#include "accel/gpu/sdfgpu.h"

#include "geometry/gpu/ray2dgpu.h"
#include "geometry/gpu/Intersection2Dgpu.h"
#include "thrust/device_vector.h"
#include "geometry/gpu/wedge2dgpu.h"
#include "raysplittinggpu.h"
#include "tree/gpu/pathnodegpu.h"


#include "physical/limitinfo.h"

#include "utility/gpuutil.h"


class RayPathGPU;


struct GPUIntersectCalculate :public thrust::unary_function<Ray2DGPU, Intersection2DGPU> {
	SignedDistanceFieldGPU* m_sdf;
	Segment2DGPU* m_segments;
	HOST_DEVICE_FUNC GPUIntersectCalculate(SignedDistanceFieldGPU* sdf, Segment2DGPU* segments) :m_sdf(sdf), m_segments(segments) {}
	HOST_DEVICE_FUNC Intersection2DGPU operator()(const Ray2DGPU& ray) {
		Intersection2DGPU intersect;
		m_sdf->GetIntersect(ray, &intersect, m_segments);
		return intersect;
	}
};

//谓词1-交点信息无效依据
struct IsInValidIntersectGPU {
	HOST_DEVICE_FUNC
		bool operator()(const Intersection2DGPU& intersect) {
		return !intersect.m_isValid;
	}
};

//谓词-交点信息视距依据
struct isLosIntersectGPU {
	HOST_DEVICE_FUNC
		bool operator()(const Intersection2DGPU& inter) {
		return inter.m_type == NODE_LOS;
	}
};

//谓词2-射线信息无效依据
struct IsInValidRayGPU {
	HOST_DEVICE_FUNC
		bool operator()(const Ray2DGPU& ray) {
		return !ray.m_isValid;
	}
};

//谓词3-路径节点无效依据
struct IsInValidPathNodeGPU {
	HOST_DEVICE_FUNC
		bool operator()(const PathNodeGPU& pathNode) {
		return !pathNode.m_isValid;
	}
};

//谓词4-数据为0
struct is_zero
{
	HOST_DEVICE_FUNC
		bool operator()(const int x)
	{
		return x == 0;
	}
};

//谓词5-树节点信息为无效信息
struct IsInValidTreeNodeGPU {
	HOST_DEVICE_FUNC
		bool operator()(const TreeNodeGPU& treeNode) {
		return !treeNode.m_isValid;
	}
};




/**
* @brief		并行计算每条射线中的绕射点-[每条射线最大分配50个]
* @param[in]	
* @return		
* @author		胡硕
* @date			2023/05/25
*/
__global__ void GPUDiffFinderKernel(Ray2DGPU* ray, Intersection2DGPU* intersect, int* everyIntersectNum, Wedge2DGPU* wedge, int wedgeCount, int size);


/**
* @brief		并行计算绕射点-一维
* @param[in]	
* @return		
* @author		胡硕
* @date			2023/05/27
*/
__global__ void diffractionFindKernel1D(Ray2DGPU* rays, int numRays, Wedge2DGPU* wedge, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments, Intersection2DGPU* intersects);



/**
* @brief		基于所有射线的绕射点结果，使用前缀和结果进行去除空数据
* @param[in]	
* @return		
* @author		胡硕
* @date			2023/05/25
*/
__global__ void rearrangeDiffractionNodeKernel(Intersection2DGPU* oldIntersects, Intersection2DGPU* newIntersects, int* prefixSum, int* everyIntersectNum, int size);


__global__ void intersectKernel(Ray2DGPU* rays, int numRays, Intersection2DGPU* intersects, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments);

/**
* @brief		计算射线与场景的相交结果并检测分裂的个数,存储至splitNum中，计算完成后得到intersect结果，然后也得到到每条射线的分裂数目，用前缀合相加得到总数
* @param[in]	
* @return		
* @author		胡硕
* @date			2023/05/25
*/
__global__ void intersectAndCalculateRaySplitNumKernel(Ray2DGPU* ray, Intersection2DGPU* intersect, int* everySplitNum, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments, int size);


/**
* @brief		(定位)计算射线与场景的相交结果并检测分裂的个数,存储至splitNum中，计算完成后得到intersect结果，然后也得到到每条射线的分裂数目，用前缀合相加得到总数
* @param[in]
* @return
* @author		胡硕
* @date			2023/05/25
*/
__global__ void intersectAndCalculateRaySplitNumKernelLBS(Ray2DGPU* ray, Intersection2DGPU* intersect, int* everySplitNum, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments, int size);

//使用此函数之前需要使用前缀和进行所有新分裂的射线的求和

/**
* @brief		基于所有射线的结果，进行射线分裂，并写入到newRays中
* @param[in]	
* @return		
* @author		胡硕
* @date			2023/05/25
*/
__global__ void raySplitKernel(Ray2DGPU* rays, int numRays, Ray2DGPU* newRays, int* everySplitNum, Segment2DGPU* segments);

//使用此函数后需要基于thrust::earser进行无效值剔除


/**
* @brief		一维射线管判定包含接收点函数
* @param[in]	
* @return		
* @author		胡硕
* @date			2023/05/27
*/
__global__ void checkRxInsideKernel1D(Intersection2DGPU* allIntersects, Intersection2DGPU* intersects, int numIntersects, Point2D* rx, int rxId, int offset, int layer, PathNodeGPU* pathNode);



//使此函数后需要基于captureNum进行前缀和，删除pathnode中的空数据(isValid为false的数据)

__global__ void generateNewReflectionRayKernel(Intersection2DGPU* intersects, int numIntersects, int prevInterSize, Ray2DGPU* newRays, Segment2DGPU* segments);

//产生新的反射射线并顺带得到射线树节点数据
__global__ void generateNewReflectionRayWithNodeKernel(Intersection2DGPU* intersects, int numIntersects, int prevInterSize, Ray2DGPU* newRays, TreeNodeGPU* newNodes, Segment2DGPU* segments, int layer);

//使用此函数后需要去除不满足条件的射线

__global__ void generateNewTransmitRayKernel(Intersection2DGPU* intersects, int numIntersects, int prevInterSize, Ray2DGPU* newRays, Segment2DGPU* segments);

//产生新的透射射线并顺带得到射线树节点数据
__global__ void generateNewTransmitRayWithNodeKernel(Intersection2DGPU* intersects, int numIntersects, int prevInterSize, Ray2DGPU* newRays, TreeNodeGPU* newNodes, Segment2DGPU* segments, int layer);


//使用此函数后需要去除不满足条件的射线

__global__ void generateNewDifftactRayKernel(Intersection2DGPU* intersects, int numIntersects, int prevInterSize, Ray2DGPU* newRays, Wedge2DGPU* wedges);

//产生新的绕射射线并顺带得到射线树节点数据
__global__ void generateNewDifftactRayWithNodeKernel(Intersection2DGPU* intersects, int numIntersects, int prevInterSize, Ray2DGPU* newRays, TreeNodeGPU* newNodes, Wedge2DGPU* wedges, int layer);

__global__ void generateNewScatRayKernel(Intersection2DGPU* intersects, int numIntersects, Ray2DGPU* newRays, Segment2DGPU* segments);


__global__ void _resetRayGPUKernel(Ray2DGPU* rays, int numRays);

void ResetRay2DGPU(thrust::device_vector<Ray2DGPU>& rays);

__global__ void _resetIntersectGPUKernel(Intersection2DGPU* intersects, int numIntersects);

void ResetIntersect2DGPU(thrust::device_vector<Intersection2DGPU>& intersects);

__global__ void _resetPathNodeGPUKernel(PathNodeGPU* pathNodes, int numPathNodes);


void ResetPathNodeGPU(thrust::device_vector<PathNodeGPU>& pathNodes);
//使用此函数后需要去除不满足条件的射线
//并且要把射线进行合并
//最后释放内存




//全局函数-射线追踪GPU版本，采用GPU并行填满pathNodeTree,调试阶段，只采用固定单个Tx作为测试
void PathTraceGPU(const std::vector<Ray2D>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, Point2D* rxPositions, size_t numRxPositions, Segment2DGPU* segmentsGPU, size_t numSegments,
	Wedge2DGPU* wedges, size_t numWedges, SignedDistanceFieldGPU* sdfGPU, std::vector<PathNodeGPU*>& outPathNode);

//全局函数-射线追踪GPU版本,只生成射线追踪树结构，不生成路径
void PathTraceGPUOnlyTree(const std::vector<Ray2D>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, Segment2DGPU* segmentsGPU, size_t numSegments,
	Wedge2DGPU* wedges, size_t numWedges, SignedDistanceFieldGPU* sdfGPU, std::vector<TreeNodeGPU*>& outTreeNode);

//基于path中的路径进行计算，校正透射路径
bool PathTraceGPULite(RayPathGPU& inpath, const Segment2DGPU* segments);

#endif
