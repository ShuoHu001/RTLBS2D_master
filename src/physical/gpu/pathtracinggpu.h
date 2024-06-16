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

//ν��1-������Ϣ��Ч����
struct IsInValidIntersectGPU {
	HOST_DEVICE_FUNC
		bool operator()(const Intersection2DGPU& intersect) {
		return !intersect.m_isValid;
	}
};

//ν��-������Ϣ�Ӿ�����
struct isLosIntersectGPU {
	HOST_DEVICE_FUNC
		bool operator()(const Intersection2DGPU& inter) {
		return inter.m_type == NODE_LOS;
	}
};

//ν��2-������Ϣ��Ч����
struct IsInValidRayGPU {
	HOST_DEVICE_FUNC
		bool operator()(const Ray2DGPU& ray) {
		return !ray.m_isValid;
	}
};

//ν��3-·���ڵ���Ч����
struct IsInValidPathNodeGPU {
	HOST_DEVICE_FUNC
		bool operator()(const PathNodeGPU& pathNode) {
		return !pathNode.m_isValid;
	}
};

//ν��4-����Ϊ0
struct is_zero
{
	HOST_DEVICE_FUNC
		bool operator()(const int x)
	{
		return x == 0;
	}
};

//ν��5-���ڵ���ϢΪ��Ч��Ϣ
struct IsInValidTreeNodeGPU {
	HOST_DEVICE_FUNC
		bool operator()(const TreeNodeGPU& treeNode) {
		return !treeNode.m_isValid;
	}
};




/**
* @brief		���м���ÿ�������е������-[ÿ������������50��]
* @param[in]	
* @return		
* @author		��˶
* @date			2023/05/25
*/
__global__ void GPUDiffFinderKernel(Ray2DGPU* ray, Intersection2DGPU* intersect, int* everyIntersectNum, Wedge2DGPU* wedge, int wedgeCount, int size);


/**
* @brief		���м��������-һά
* @param[in]	
* @return		
* @author		��˶
* @date			2023/05/27
*/
__global__ void diffractionFindKernel1D(Ray2DGPU* rays, int numRays, Wedge2DGPU* wedge, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments, Intersection2DGPU* intersects);



/**
* @brief		�����������ߵ����������ʹ��ǰ׺�ͽ������ȥ��������
* @param[in]	
* @return		
* @author		��˶
* @date			2023/05/25
*/
__global__ void rearrangeDiffractionNodeKernel(Intersection2DGPU* oldIntersects, Intersection2DGPU* newIntersects, int* prefixSum, int* everyIntersectNum, int size);


__global__ void intersectKernel(Ray2DGPU* rays, int numRays, Intersection2DGPU* intersects, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments);

/**
* @brief		���������볡�����ཻ����������ѵĸ���,�洢��splitNum�У�������ɺ�õ�intersect�����Ȼ��Ҳ�õ���ÿ�����ߵķ�����Ŀ����ǰ׺����ӵõ�����
* @param[in]	
* @return		
* @author		��˶
* @date			2023/05/25
*/
__global__ void intersectAndCalculateRaySplitNumKernel(Ray2DGPU* ray, Intersection2DGPU* intersect, int* everySplitNum, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments, int size);


/**
* @brief		(��λ)���������볡�����ཻ����������ѵĸ���,�洢��splitNum�У�������ɺ�õ�intersect�����Ȼ��Ҳ�õ���ÿ�����ߵķ�����Ŀ����ǰ׺����ӵõ�����
* @param[in]
* @return
* @author		��˶
* @date			2023/05/25
*/
__global__ void intersectAndCalculateRaySplitNumKernelLBS(Ray2DGPU* ray, Intersection2DGPU* intersect, int* everySplitNum, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments, int size);

//ʹ�ô˺���֮ǰ��Ҫʹ��ǰ׺�ͽ��������·��ѵ����ߵ����

/**
* @brief		�����������ߵĽ�����������߷��ѣ���д�뵽newRays��
* @param[in]	
* @return		
* @author		��˶
* @date			2023/05/25
*/
__global__ void raySplitKernel(Ray2DGPU* rays, int numRays, Ray2DGPU* newRays, int* everySplitNum, Segment2DGPU* segments);

//ʹ�ô˺�������Ҫ����thrust::earser������Чֵ�޳�


/**
* @brief		һά���߹��ж��������յ㺯��
* @param[in]	
* @return		
* @author		��˶
* @date			2023/05/27
*/
__global__ void checkRxInsideKernel1D(Intersection2DGPU* allIntersects, Intersection2DGPU* intersects, int numIntersects, Point2D* rx, int rxId, int offset, int layer, PathNodeGPU* pathNode);



//ʹ�˺�������Ҫ����captureNum����ǰ׺�ͣ�ɾ��pathnode�еĿ�����(isValidΪfalse������)

__global__ void generateNewReflectionRayKernel(Intersection2DGPU* intersects, int numIntersects, int prevInterSize, Ray2DGPU* newRays, Segment2DGPU* segments);

//�����µķ������߲�˳���õ��������ڵ�����
__global__ void generateNewReflectionRayWithNodeKernel(Intersection2DGPU* intersects, int numIntersects, int prevInterSize, Ray2DGPU* newRays, TreeNodeGPU* newNodes, Segment2DGPU* segments, int layer);

//ʹ�ô˺�������Ҫȥ������������������

__global__ void generateNewTransmitRayKernel(Intersection2DGPU* intersects, int numIntersects, int prevInterSize, Ray2DGPU* newRays, Segment2DGPU* segments);

//�����µ�͸�����߲�˳���õ��������ڵ�����
__global__ void generateNewTransmitRayWithNodeKernel(Intersection2DGPU* intersects, int numIntersects, int prevInterSize, Ray2DGPU* newRays, TreeNodeGPU* newNodes, Segment2DGPU* segments, int layer);


//ʹ�ô˺�������Ҫȥ������������������

__global__ void generateNewDifftactRayKernel(Intersection2DGPU* intersects, int numIntersects, int prevInterSize, Ray2DGPU* newRays, Wedge2DGPU* wedges);

//�����µ��������߲�˳���õ��������ڵ�����
__global__ void generateNewDifftactRayWithNodeKernel(Intersection2DGPU* intersects, int numIntersects, int prevInterSize, Ray2DGPU* newRays, TreeNodeGPU* newNodes, Wedge2DGPU* wedges, int layer);

__global__ void generateNewScatRayKernel(Intersection2DGPU* intersects, int numIntersects, Ray2DGPU* newRays, Segment2DGPU* segments);


__global__ void _resetRayGPUKernel(Ray2DGPU* rays, int numRays);

void ResetRay2DGPU(thrust::device_vector<Ray2DGPU>& rays);

__global__ void _resetIntersectGPUKernel(Intersection2DGPU* intersects, int numIntersects);

void ResetIntersect2DGPU(thrust::device_vector<Intersection2DGPU>& intersects);

__global__ void _resetPathNodeGPUKernel(PathNodeGPU* pathNodes, int numPathNodes);


void ResetPathNodeGPU(thrust::device_vector<PathNodeGPU>& pathNodes);
//ʹ�ô˺�������Ҫȥ������������������
//����Ҫ�����߽��кϲ�
//����ͷ��ڴ�




//ȫ�ֺ���-����׷��GPU�汾������GPU��������pathNodeTree,���Խ׶Σ�ֻ���ù̶�����Tx��Ϊ����
void PathTraceGPU(const std::vector<Ray2D>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, Point2D* rxPositions, size_t numRxPositions, Segment2DGPU* segmentsGPU, size_t numSegments,
	Wedge2DGPU* wedges, size_t numWedges, SignedDistanceFieldGPU* sdfGPU, std::vector<PathNodeGPU*>& outPathNode);

//ȫ�ֺ���-����׷��GPU�汾,ֻ��������׷�����ṹ��������·��
void PathTraceGPUOnlyTree(const std::vector<Ray2D>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, Segment2DGPU* segmentsGPU, size_t numSegments,
	Wedge2DGPU* wedges, size_t numWedges, SignedDistanceFieldGPU* sdfGPU, std::vector<TreeNodeGPU*>& outTreeNode);

//����path�е�·�����м��㣬У��͸��·��
bool PathTraceGPULite(RayPathGPU& inpath, const Segment2DGPU* segments);

#endif
