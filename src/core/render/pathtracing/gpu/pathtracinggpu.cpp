#include "pathtracinggpu.h"
#include "radiowave/raypath/gpu/raypathgpu.h"
#include <fstream>
#include <chrono>

__constant__ uint16_t D_CONST_LIM_TOTL; /** @brief	�豸��ȫ��--��������	*/
__constant__ uint16_t D_CONST_LIM_REFL;	/** @brief	�豸��ȫ��--����������	*/
__constant__ uint16_t D_CONST_LIM_TRAN; /** @brief	�豸��ȫ��--͸��������	*/
__constant__ uint16_t D_CONST_LIM_DIFF; /** @brief	�豸��ȫ��--����������	*/
__constant__ uint16_t D_CONST_LIM_SCAT; /** @brief	�豸��ȫ��--ɢ��������	*/
__constant__ RtLbsType D_CONST_RAYSPLITRADIUS;	/** @brief	�豸��ȫ��--���߷��Ѱ뾶	*/
__constant__ int _D_CONST_DIFFTACTRAYNUM;		/** @brief	�豸��ȫ��--������������	*/


__device__ int G_DIFFID = 0; /** @brief	���������Ѱ�ҵ�ԭ������������	*/
__device__ int G_RXID = 0; /** @brief	���ڽ��յ�rx�����ж���ԭ������������	*/
const int C_ZERO = 0;		/** @brief	��������ÿ���������	*/

__global__ void GPUDiffFinderKernel(Ray2DGPU* ray, Intersection2DGPU* intersect, int* everyIntersectNum, Wedge2DGPU* wedge, int wedgeCount, int size)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;//����̱߳��
	//����ÿһ���̵߳ı�ţ�Ȼ����intersect�ڴ���Ѱ�ҵ���Ӧ���ڴ�λ�ã���д������
	if (tid < size) {
		int containWedgesNum = 0;
		for (int i = 0; i < wedgeCount; ++i) {
			if (/*��ǰ�ڵ���������*/true) {
				Intersection2DGPU newIntersect;//�´����Ľ�����Ϣ
				intersect[tid * 50 + containWedgesNum] = newIntersect;//�����50����ÿ���������ɰ����������
				containWedgesNum++;
			}
		}
		everyIntersectNum[tid] = containWedgesNum;
	}
}


/**
* @brief		�����Ѱ�Һ���1ά��
* @param[in]	
* @return		
* @author		��˶
* @date			2023/05/27
*/
__global__ void diffractionFindKernel1D(Ray2DGPU* rays, size_t numRays, Wedge2DGPU* wedge, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments, Intersection2DGPU* intersects) {
	
	int rayIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (rayIdx < numRays) {
		if (rays[rayIdx].m_limTotl <= 0 || rays[rayIdx].m_limDiff <= 0)
			return;
		if (sdf->IsRayCaptureByWedgePoint(rays[rayIdx], wedge->m_edge, sdf, segments)) {
			int idx = atomicAdd(&G_DIFFID, 1);
			intersects[idx].m_isValid = true;
			intersects[idx].m_type = NODE_DIFF;
			intersects[idx].m_wedgeId = wedge->m_wedge_id;
			intersects[idx].m_intersect = wedge->m_edge;
			intersects[idx].m_ray = rays[rayIdx];
			intersects[idx].m_ft = (wedge->m_edge - rays[rayIdx].m_Ori).Length();
			intersects[idx].m_prevId = rays[rayIdx].m_prevInterId;//��һ��������Ϣ��Ŵ���
			intersects[idx].m_propagationProperty = wedge->m_face1.m_propagationProperty;
		}
	}
}



__global__ void rearrangeDiffractionNodeKernel(Intersection2DGPU* oldIntersects, Intersection2DGPU* newIntersects, int* prefixSum, int* everyIntersectNum, int size)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid < size) {
		for (int i = 0; i < everyIntersectNum[tid]; ++i) {
			newIntersects[prefixSum[tid] + i] = oldIntersects[tid * 50 + i];
		}
	}
}

__global__ void intersectKernel(Ray2DGPU* rays, size_t numRays, Intersection2DGPU* intersects, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments)
{
	int rayIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (rayIdx < numRays) {
		sdf->GetIntersect(rays[rayIdx], &intersects[rayIdx], segments);
		intersects[rayIdx].m_prevId = rays[rayIdx].m_prevInterId;
	}
}

__global__ void intersectAndCalculateRaySplitNumKernel(Ray2DGPU* ray, Intersection2DGPU* intersect, int* everySplitNum, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments, int size)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid < size) {
		if (ray[tid].m_isValid == false) {
			everySplitNum[tid] = 0;
			return;
		}

		sdf->GetIntersect(ray[tid], &intersect[tid], segments);
		//���Ӽ���ÿ�����ߵķ�����Ŀ
		RtLbsType radius = ray[tid].GetRayRadis(intersect[tid].m_ft);
		intersect[tid].m_prevId = ray[tid].m_prevInterId;
		if (radius > D_CONST_RAYSPLITRADIUS) {
			everySplitNum[tid] = static_cast<int>(ceil(2 * radius / D_CONST_RAYSPLITRADIUS));
			intersect[tid].m_isValid = false;//�������ѷ��ѵĽ�����Ϣ���Ժ�������
		}
		else {
			everySplitNum[tid] = 0;
			ray[tid].m_isValid = false;//������δ���ѵ����ߣ��Է�����������
		}
	}

}

__global__ void intersectAndCalculateRaySplitNumKernelLBS(Ray2DGPU* ray, Intersection2DGPU* intersect, int* everySplitNum, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments, size_t size)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid < size) {
		if (ray[tid].m_isValid == false) {
			everySplitNum[tid] = 0;
			return;
		}
		if (ray[tid].m_nodeType == NODE_TRANIN ||
			ray[tid].m_nodeType == NODE_TRANOUT ||
			ray[tid].m_nodeType == NODE_ETRANIN ||
			ray[tid].m_nodeType == NODE_ETRANOUT) {
			return;											//͸��󲻽��з��Ѳ���
		}
		sdf->GetIntersect(ray[tid], &intersect[tid], segments);
		//���Ӽ���ÿ�����ߵķ�����Ŀ
		RtLbsType radius = ray[tid].GetRayRadis(intersect[tid].m_ft);
		intersect[tid].m_prevId = ray[tid].m_prevInterId;
		if (radius > D_CONST_RAYSPLITRADIUS) {
			everySplitNum[tid] = static_cast<int>(ceil(2 * radius / D_CONST_RAYSPLITRADIUS));
			intersect[tid].m_isValid = false;//�������ѷ��ѵĽ�����Ϣ���Ժ�������
		}
		else {
			everySplitNum[tid] = 0;
			ray[tid].m_isValid = false;//������δ���ѵ����ߣ��Է�����������
		}
	}
}

__global__ void raySplitKernel(Ray2DGPU* rays, size_t numRays, Ray2DGPU* newRays, int* everySplitNum, Segment2DGPU* segments)
{
	int rayIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (rayIdx < numRays) {
		if (everySplitNum[rayIdx] > 0) {
			int startIdx = 0;
			for (int i = 0; i < rayIdx; ++i) {//���㵽��ǰ���߷�������������ֵ
				startIdx += everySplitNum[i];
			}
			GenerateSplittingRayGPU(rays[rayIdx], everySplitNum[rayIdx], &newRays[startIdx], segments);//�ҵ���ַ֮��ֱ���������Ͻ��и�ֵ
		}
	}
}

__global__ void checkRxInsideKernel1D(Intersection2DGPU* allIntersects, Intersection2DGPU* intersects, int numIntersects, Point2D* rx, int rxId, int offset, int layer, PathNodeGPU* pathNode) {
	int intersectIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (intersectIdx < numIntersects) {
		Intersection2DGPU* intersect = &intersects[intersectIdx];
		if (intersect->IsCaptureRx(*rx)) {
			int idx = atomicAdd(&G_RXID, (layer + 1)) + offset;//��ԭ�����Ļ����ϼ������ÿ��rx��Ƶĵ�ַƫ������offset=rxId*300*(layer+1);,�����layer+1��Ϊ�˱���ÿ�����յ������׷��·������
			pathNode[idx].m_inter = intersects[intersectIdx];
			pathNode[idx].m_isValid = true;
			pathNode[idx].m_layer = layer;
			pathNode[idx].m_rxId = rxId;
			Intersection2DGPU *curInter = &intersects[intersectIdx];//��ǰ�����Ľ�����Ϣָ��
			for (int i = 1; i < layer + 1; ++i) {//�ٸ��ʵ�׷�ݵ���������,�����i��1��ʼֻ��Ϊ������pathNode���ӵı��
				curInter = &allIntersects[curInter->m_prevId];
				pathNode[idx + i].m_inter = *curInter;
				pathNode[idx + i].m_isValid = true;
				pathNode[idx + i].m_layer = layer - i;
				pathNode[idx + i].m_rxId = rxId;
			}
		}
	}
}


__global__ void generateNewReflectionRayKernel(Intersection2DGPU* intersects, size_t numIntersects, size_t prevInterSize, Ray2DGPU* newRays, Segment2DGPU* segments)
{
	int intersectIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (intersectIdx < numIntersects) {
		Intersection2DGPU& intersect = intersects[intersectIdx];
		if (intersect.m_type == NODE_REFL) {
			Ray2DGPU& ray = intersect.m_ray;
			if (ray.m_limRefl <= 0 || ray.m_limTotl <= 0)			// �����������������ﵽ���ƻ����������ﵽ���ƣ���ֱ�ӷ���
				return;
			if (!intersect.m_propagationProperty.m_hasRelfection)	//���������ڵĴ����������޷������ԣ���ֱ�ӷ���
				return;
			int interId = intersectIdx + prevInterSize;
			GenerateReflectRayGPU(intersect, interId, segments, &newRays[intersectIdx]);
		}
		
	}
}

__global__ void generateNewReflectionRayWithNodeKernel(Intersection2DGPU* intersects, size_t numIntersects, size_t prevInterSize, Ray2DGPU* newRays, TreeNodeGPU* newNodes, Segment2DGPU* segments, int layer)
{
	int intersectIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (intersectIdx < numIntersects) {
		Intersection2DGPU& intersect = intersects[intersectIdx];
		if (intersect.m_type == NODE_REFL) {
			Ray2DGPU& ray = intersect.m_ray;
			if (ray.m_limRefl <= 0 || ray.m_limTotl <= 0)			// �����������������ﵽ���ƻ����������ﵽ���ƣ���ֱ�ӷ���
				return;
			if (!intersect.m_propagationProperty.m_hasRelfection)	//���������ڵĴ����������޷������ԣ���ֱ�ӷ���
				return;
			int interId = intersectIdx + prevInterSize;
			GenerateReflectRayGPU(intersect, interId, segments, &newRays[intersectIdx], &newNodes[intersectIdx], layer);
		}
	}
}

__global__ void generateNewTransmitRayKernel(Intersection2DGPU* intersects, size_t numIntersects, size_t prevInterSize, Ray2DGPU* newRays, Segment2DGPU* segments)
{
	int intersectIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (intersectIdx < numIntersects) {

		Intersection2DGPU& intersect = intersects[intersectIdx];
		if (intersect.m_type == NODE_REFL) {
			Ray2DGPU& ray = intersect.m_ray;
			int interId = intersectIdx + prevInterSize;
			if (intersect.m_propagationProperty.m_hasTransmission) {	//���������ڵĴ�����������͸�����ԣ���ֱ�ӷ���
				if (ray.m_limTran <= 0 || ray.m_limTotl <= 0) {			//��͸�������������ﵽ���ƻ����������ﵽ���ƣ���ֱ�ӷ���
					return;
				}
				GenerateTransmitRayGPU(intersect, interId, segments, &newRays[intersectIdx]);
			}
			else if (intersect.m_propagationProperty.m_hasEmpiricalTransmission) {
				GenerateEmpiricalTransmitRayGPU(intersect, interId, segments, &newRays[intersectIdx]);
			}
		}
	}
}

__global__ void generateNewTransmitRayWithNodeKernel(Intersection2DGPU* intersects, size_t numIntersects, size_t prevInterSize, Ray2DGPU* newRays, TreeNodeGPU* newNodes, Segment2DGPU* segments, int layer)
{
	int intersectIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (intersectIdx < numIntersects) {

		Intersection2DGPU& intersect = intersects[intersectIdx];
		if (intersect.m_type == NODE_REFL) {
			Ray2DGPU& ray = intersect.m_ray;

			int interId = intersectIdx + prevInterSize;
			if (intersect.m_propagationProperty.m_hasTransmission) {	//���������ڵĴ�����������͸�����ԣ���ֱ�ӷ���
				if (ray.m_limTran <= 0 || ray.m_limTotl <= 0) {			//��͸�������������ﵽ���ƻ����������ﵽ���ƣ���ֱ�ӷ���
					return;
				}
				GenerateTransmitRayGPU(intersect, interId, segments, &newRays[intersectIdx], &newNodes[intersectIdx], layer);
			}
			else if (intersect.m_propagationProperty.m_hasEmpiricalTransmission) {
				GenerateEmpiricalTransmitRayGPU(intersect, interId, segments, &newRays[intersectIdx], &newNodes[intersectIdx], layer);
			}
		}
	}
}

__global__ void generateNewDifftactRayKernel(Intersection2DGPU* intersects, size_t numIntersects, size_t prevInterSize, Ray2DGPU* newRays, Wedge2DGPU* wedges)
{
	int intersectIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (intersectIdx < numIntersects) {
		
		Intersection2DGPU& intersect = intersects[intersectIdx];
		Wedge2DGPU* wedge = &wedges[intersect.m_wedgeId];
		Ray2DGPU& ray = intersect.m_ray;
		if (intersect.m_type == NODE_DIFF) {
			
			if (ray.m_limDiff <= 0 || ray.m_limTotl <= 0)			//�����������������ﵽ���ƻ����������ﵽ���ƣ���ֱ�ӷ���
				return;
			if (!intersect.m_propagationProperty.m_hasDiffraction)	//���������ڵĴ������������������ԣ���ֱ�ӷ���
				return;
			
			int interId = intersectIdx + prevInterSize;
			GenerateDiffractRaysGPU(intersect, ray, wedge, interId, &newRays[intersectIdx * _D_CONST_DIFFTACTRAYNUM], _D_CONST_DIFFTACTRAYNUM);//���ҽ��������������������²ſɽ��в���������
		}
	}
}

__global__ void generateNewDifftactRayWithNodeKernel(Intersection2DGPU* intersects, size_t numIntersects, size_t prevInterSize, Ray2DGPU* newRays, TreeNodeGPU* newNodes, Wedge2DGPU* wedges, int layer)
{
	int intersectIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (intersectIdx < numIntersects) {

		Intersection2DGPU& intersect = intersects[intersectIdx];
		Wedge2DGPU* wedge = &wedges[intersect.m_wedgeId];
		Ray2DGPU& ray = intersect.m_ray;
		if (intersect.m_type == NODE_DIFF) {

			if (ray.m_limDiff <= 0 || ray.m_limTotl <= 0)			//�����������������ﵽ���ƻ����������ﵽ���ƣ���ֱ�ӷ���
				return;
			if (!intersect.m_propagationProperty.m_hasDiffraction)	//���������ڵĴ������������������ԣ���ֱ�ӷ���
				return;

			int interId = intersectIdx + prevInterSize;
			GenerateDiffractRaysGPU(intersect, ray, wedge, interId, &newRays[intersectIdx * _D_CONST_DIFFTACTRAYNUM], &newNodes[intersectIdx * _D_CONST_DIFFTACTRAYNUM], _D_CONST_DIFFTACTRAYNUM, layer);//���ҽ��������������������²ſɽ��в���������
		}
	}
}

__global__ void generateNewScatRayKernel(Intersection2DGPU* intersects, int numIntersects, Ray2DGPU* newRays, Segment2DGPU* segments)
{
	int intersectIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (intersectIdx < numIntersects) {
		Intersection2DGPU& intersect = intersects[intersectIdx];
		if (intersect.m_type == NODE_SCAT) {
			Ray2DGPU& ray = intersect.m_ray;
		}
		return;			//��δ���ǣ��ʲ�������
	}
}


__global__ void _resetRayGPUKernel(Ray2DGPU* rays, int numRays) {
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx < numRays) {
		rays[idx].m_isValid = false;
	}
}

void ResetRay2DGPU(thrust::device_vector<Ray2DGPU>& rays) {
	int threadPerBlock = 256;
	int numblocks = (static_cast<int>(rays.size()) + threadPerBlock - 1) / threadPerBlock;
	_resetRayGPUKernel CUDA_KERNEL(numblocks, threadPerBlock)(thrust::raw_pointer_cast(rays.data()), static_cast<int>(rays.size()));
	cudaError_t cudaerr = cudaDeviceSynchronize();
	if (cudaerr != cudaSuccess) {
		//LOG_ERROR << "PathTracingGPU: _resetRayGPUKernel " << cudaGetErrorString(cudaerr) << ENDL;
	}
}

__global__ void _resetIntersectGPUKernel(Intersection2DGPU* intersects, int numIntersects) {
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx < numIntersects) {
		intersects[idx].m_isValid = false;
	}
}

void ResetIntersect2DGPU(thrust::device_vector<Intersection2DGPU>& intersects) {
	int threadPerBlock = 256;
	int numblocks = (static_cast<int>(intersects.size()) + threadPerBlock - 1) / threadPerBlock;
	_resetIntersectGPUKernel CUDA_KERNEL(numblocks, threadPerBlock)(thrust::raw_pointer_cast(intersects.data()), static_cast<int>(intersects.size()));
	cudaError_t cudaerr = cudaDeviceSynchronize();
	if (cudaerr != cudaSuccess) {
		//LOG_ERROR << "PathTracingGPU: _resetIntersectGPUKernel " << cudaGetErrorString(cudaerr) << ENDL;
	}
}

__global__ void _resetPathNodeGPUKernel(PathNodeGPU* pathNodes, int numPathNodes) {
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx < numPathNodes) {
		pathNodes[idx].m_isValid = false;
	}
}

void ResetPathNodeGPU(thrust::device_vector<PathNodeGPU>& pathNodes) {
	int threadPerBlock = 256;
	int numblocks = (static_cast<int>(pathNodes.size()) + threadPerBlock - 1) / threadPerBlock;
	_resetPathNodeGPUKernel CUDA_KERNEL(numblocks, threadPerBlock)(thrust::raw_pointer_cast(pathNodes.data()), static_cast<int>(pathNodes.size()));
	cudaError_t cudaerr = cudaDeviceSynchronize();
	if (cudaerr != cudaSuccess) {
		//LOG_ERROR << "PathTracingGPU: _resetPathNodeGPUKernel " << cudaGetErrorString(cudaerr) << ENDL;
	}
}


//ȫ�ֺ���-����׷��GPU�汾������GPU��������pathNodeTree,���Խ׶Σ�ֻ���ù̶�����Tx��Ϊ����
void PathTraceGPU(const std::vector<Ray2D>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, int diffractRayNum, Point2D* dev_rxPositions, size_t numRxPositions, Segment2DGPU* segmentsGPU, size_t numSegments, Wedge2DGPU* wedges, size_t numWedges, SignedDistanceFieldGPU* sdfGPU, std::vector<PathNodeGPU*>& outPathNode) {
	int threadsPerBlock = 128;

	std::chrono::high_resolution_clock::time_point start;
	std::chrono::high_resolution_clock::time_point end;
	std::chrono::milliseconds duration;
	//���pathNodeTree���д���

	//task-0 ת��ԭʼ����
	size_t numRays = rays.size();
	std::vector<Ray2DGPU> host_initRays(numRays);//ִ�����ߵ�ת��
	for (size_t i = 0; i < numRays; ++i) {
		host_initRays[i] = rays[i].Convert2GPU();
		host_initRays[i].m_isValid = true;
		host_initRays[i].m_nodeType = NODE_ROOT;
		host_initRays[i].m_limTotl = limitInfo.m_limitTotal;
		host_initRays[i].m_limRefl = limitInfo.m_limitReflect;
		host_initRays[i].m_limTran = limitInfo.m_limitTransmit;
		host_initRays[i].m_limDiff = limitInfo.m_limitDiffract;
		host_initRays[i].m_limScat = limitInfo.m_limitScatter;
	}

	//�漰������ת��


	start = std::chrono::high_resolution_clock::now();
	//LOG_INFO << "running gpu code" << ENDL;

	cudaMemcpyToSymbol(D_CONST_LIM_TOTL, &limitInfo.m_limitTotal, sizeof(uint16_t));
	cudaMemcpyToSymbol(D_CONST_LIM_REFL, &limitInfo.m_limitReflect, sizeof(uint16_t));
	cudaMemcpyToSymbol(D_CONST_LIM_TRAN, &limitInfo.m_limitTransmit, sizeof(uint16_t));
	cudaMemcpyToSymbol(D_CONST_LIM_DIFF, &limitInfo.m_limitDiffract, sizeof(uint16_t));
	cudaMemcpyToSymbol(D_CONST_LIM_SCAT, &limitInfo.m_limitScatter, sizeof(uint16_t));
	cudaMemcpyToSymbol(D_CONST_RAYSPLITRADIUS, &raySplitRadius, sizeof(RtLbsType));
	cudaMemcpyToSymbol(_D_CONST_DIFFTACTRAYNUM, &diffractRayNum, sizeof(int));



	bool stateHasDiff = limitInfo.m_limitDiffract == 0 ? false : true;
	bool stateHasRefl = limitInfo.m_limitReflect == 0 ? false : true;
	bool stateHasTran = limitInfo.m_limitTransmit == 0 ? false : true;
	bool stateHasScat = limitInfo.m_limitScatter == 0 ? false : true;


	
	thrust::device_vector<Ray2DGPU> dev_initRays;//��ֹ��ѭ�����ж�dev_newRays�����ͷ�
	dev_initRays.reserve(numRays * 5);//Ԥ����5���ڴ�
	dev_initRays.resize(numRays);
	dev_initRays = host_initRays;


	thrust::device_vector<Intersection2DGPU> dev_interInit(numRays);//ԭʼ�����ཻ�������ѭ���н�ֹ�Դ����ݽ���ɾ������������

	thrust::device_vector<Intersection2DGPU> dev_interDiffSeries(numWedges*1000);	/** @brief	��ǰ�����������Ѱ��������Ĭ����1000��ԭʼ���������	*/

	thrust::device_vector<int> everySplitNum(numRays); /** @brief	��¼ÿ�����ߵķ�����Ŀ	*/
	thrust::device_vector<Ray2DGPU> dev_splitRays;		/** @brief	���ѵ�����	*/

	thrust::device_vector<Intersection2DGPU> dev_InterSplit; /** @brief	�������ߵĽ�����Ϣ	*/

	thrust::device_vector<Intersection2DGPU> dev_interReflSeries; /** @brief	�ϲ��ķ���ϵ�н�����Ϣ,�������в���Ϣ��ԭʼ�������߽�����Ϣ+�������ߵĽ�����Ϣ��,��Ҫ���ڲ���rx	*/

	int maxSingleRxPathPerLayer = 1500; //ÿ��ÿ��rx���500��·��
	int validRxNodeSize = 0;//����ÿһ����Ч��rxPathNode����
	thrust::device_vector<PathNodeGPU> dev_rxPathNode;/** @brief	�洢����Rx���㲶��ı���*/
	
	cudaError_t cudaerr;										/** @brief	cuda������	*/

	int layer = 0;
	while (dev_initRays.size() != 0) {
		//task-1 ---------------���м��������ཻ�����ڵ������(�������)----------------------------------------------------------------------------------------------

		numRays = dev_initRays.size();

		int numInterDiff = 0;
		if (stateHasDiff) {
			int numBlocks1 = (static_cast<int>(numRays) + threadsPerBlock - 1) / threadsPerBlock;
			ResetIntersect2DGPU(dev_interDiffSeries);
			cudaMemcpyToSymbol(G_DIFFID, &C_ZERO, sizeof(int));//��ʼ��ԭ������
			
			for (int i = 0; i < numWedges; ++i) {//�������������������������
				diffractionFindKernel1D CUDA_KERNEL(numBlocks1, threadsPerBlock)(thrust::raw_pointer_cast(dev_initRays.data()), static_cast<int>(numRays),
					&wedges[i], sdfGPU, segmentsGPU, thrust::raw_pointer_cast(dev_interDiffSeries.data()));
				cudaerr = cudaDeviceSynchronize();
				if (cudaerr != cudaSuccess) {
					//LOG_ERROR <<"PathTracingGPU: diffractionFindKernel1D " << cudaGetErrorString(cudaerr) << ENDL;	
				}
			}
			auto iter_end = thrust::remove_if(dev_interDiffSeries.begin(), dev_interDiffSeries.end(), IsInValidIntersectGPU());
			numInterDiff = static_cast<int>(thrust::distance(dev_interDiffSeries.begin(), iter_end));//������Ч����ƫ��ֵ
			//LOG_INFO <<layer<<": " << numInterDiff << ENDL;
			//LOG_INFO << "end find diff" << ENDL;
		}
		


		//task-2----------------- ���м������߷���---------------------------------------------------------------------------------------------------------
		//2-1	Ԥ������Ѹ���
		dev_interInit.resize(numRays);//��չΪ��������
		ResetIntersect2DGPU(dev_interInit);///--dev_initIntersects���г�ʼ��

		//����ԭʼ���߽�����Ϣ�����߷����������������з������Ե�ԭʼ���ߣ��������߱��������ԵĽ�����Ϣ
		size_t numInterNoSplit = numRays;
		size_t numInterSplit = 0;
		size_t numRaySplit = 0;
		if (raySplitFlag == true) {
			int numBlocks2 = (static_cast<int>(numRays) + threadsPerBlock - 1) / threadsPerBlock;
			everySplitNum.resize(numRays);
			thrust::fill(everySplitNum.begin(), everySplitNum.end(), 0);//����ÿ�����߷�������
			intersectAndCalculateRaySplitNumKernel CUDA_KERNEL(numBlocks2, threadsPerBlock)(thrust::raw_pointer_cast(dev_initRays.data()),
				thrust::raw_pointer_cast(dev_interInit.data()), thrust::raw_pointer_cast(everySplitNum.data()), sdfGPU, segmentsGPU, static_cast<int>(numRays));
			cudaerr = cudaDeviceSynchronize();//�ȴ��̼߳������
			if (cudaerr != cudaSuccess) {
				//LOG_ERROR << "PathTracingGPU: intersectAndCalculateRaySplitNumKernel " << cudaGetErrorString(cudaerr) << ENDL;
			}

			//������Ч�Ľ�����Ϣ-����������������ʹ��
			auto iterEnd_validInter = thrust::remove_if(dev_interInit.begin(), dev_interInit.end(), IsInValidIntersectGPU());
			numInterNoSplit = static_cast<int>(thrust::distance(dev_interInit.begin(), iterEnd_validInter));//δ���ѵ�ԭʼ������Ϣ��Ŀ

			//������Ч������-������ʹ��
			auto iterEnd_validRay = thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU());
			numInterSplit = static_cast<int>(thrust::distance(dev_initRays.begin(), iterEnd_validRay));//�ѷ��ѵ�ԭʼ������Ϣ��Ŀ

			//��������Ŀ��Ϊ0��ǰ��
			auto iterEnd_validEverySplitNum = thrust::remove_if(everySplitNum.begin(), everySplitNum.end(), is_zero());//�����ѵ�������ԭʼ�ķ���������м���������ߣ��õ�splitRays

			//�������������Ŀ����
			numRaySplit = thrust::reduce(everySplitNum.begin(), iterEnd_validEverySplitNum, 0);

			if (numRaySplit != 0) {													//��Ԥ�������������Ϊ0���������ʵ���Ѽ���
				dev_splitRays.resize(numRaySplit);//�����ѵ����߽���resize
				ResetRay2DGPU(dev_splitRays);

				size_t numblocks_split = (numInterSplit + threadsPerBlock - 1) / threadsPerBlock;//ע�⣬�����ԭʼ�ɷ��������Ѿ�ǰ�ã�������ƥ��˼����д���
				raySplitKernel CUDA_KERNEL(numblocks_split, threadsPerBlock)(thrust::raw_pointer_cast(dev_initRays.data()), numInterSplit,
					thrust::raw_pointer_cast(dev_splitRays.data()), thrust::raw_pointer_cast(everySplitNum.data()), segmentsGPU);
				cudaerr = cudaDeviceSynchronize();
				if (cudaerr != cudaSuccess) {
					//LOG_ERROR << "PathTracingGPU: raySplitKernel " << cudaGetErrorString(cudaerr) << ENDL;
				}

				//������Ч�ķ�������
				auto interEnd_validSplitRay = thrust::remove_if(dev_splitRays.begin(), dev_splitRays.end(), IsInValidRayGPU());
				numRaySplit = static_cast<int>(thrust::distance(dev_splitRays.begin(), interEnd_validSplitRay));//���·������ߵ���Ŀ
				//task-3------------------ ���м�����������ཻ------------------------------------------------------------------------------------------------------
				dev_InterSplit.resize(numRaySplit);//�Է��ѵ����߽�����Ϣ����resize
				ResetIntersect2DGPU(dev_InterSplit);

				size_t numBlocks3 = (numRaySplit + threadsPerBlock - 1) / threadsPerBlock;
				intersectKernel CUDA_KERNEL(numBlocks3, threadsPerBlock)(thrust::raw_pointer_cast(dev_splitRays.data()), numRaySplit,
					thrust::raw_pointer_cast(dev_InterSplit.data()), sdfGPU, segmentsGPU);
				cudaerr = cudaDeviceSynchronize();
				if (cudaerr != cudaSuccess) {
					//LOG_ERROR << "PathTracingGPU: intersectKernel " << cudaGetErrorString(cudaerr) << ENDL;
				}
			}
		}
		


		

		//task-4------------------ �ϲ�����/δ�������ߵ�Inter ���--------------------------------------------------------------------------------------------
		size_t oldInterSize = dev_interReflSeries.size(); //��һ�㽻����Ϣ����
		size_t newInterReflSize = numInterNoSplit + numRaySplit;// ��ǰ��ķ���ϵ�н�����Ϣ����
		size_t newInterDiffSize = numInterDiff;   //��ǰ�������ϵ�еĽ�����Ϣ����

		dev_interReflSeries.resize(oldInterSize + newInterReflSize + newInterDiffSize);//�Է���ϵ������Ϣ����resize

		//��δ�������߽�����Ϣ�ͷ������߽�����Ϣ�ж�ȡ����
		thrust::copy(dev_interInit.begin(), dev_interInit.begin() + numInterNoSplit, dev_interReflSeries.begin() + oldInterSize);											//����ʼδ���ѽ��㸽��������ϵ�н���
		thrust::copy(dev_InterSplit.begin(), dev_InterSplit.begin() + numRaySplit, dev_interReflSeries.begin() + oldInterSize + numInterNoSplit);							//�����ѽ��㸽��������ϵ�н����
		thrust::copy(dev_interDiffSeries.begin(), dev_interDiffSeries.begin() + newInterDiffSize, dev_interReflSeries.begin() + oldInterSize + numInterNoSplit + numRaySplit);	//������ϵ���㸽��������ϵ�н���֮��

		
		//task-4------------------------------ ���м��㱻rx����������------------------------------------------------------------------------------
		//4-1 ִ��rx�ػ���
		//LOG_INFO << "start capture" << ENDL;
		int numBlocks4 = (static_cast<int>(newInterReflSize) + threadsPerBlock - 1) / threadsPerBlock;
		//ResetPathNodeGPU(dev_rxPathNode);//��ʼ��rx����ֵ
		int oldPathNodeSize = validRxNodeSize;
		int newPathNodeSize = static_cast<int>(numRxPositions) * maxSingleRxPathPerLayer * (layer + 1);//ÿ��Ԥ�����������
		int pathNodeSize = oldPathNodeSize + newPathNodeSize;
		dev_rxPathNode.resize(pathNodeSize);
		
		for (int i = 0; i < numRxPositions; ++i) {
			//ԭ����ģʽ
			cudaMemcpyToSymbol(G_RXID, &C_ZERO, sizeof(int));//��ʼ��ԭ������
			//maxLayer* rx_positions.size() * maxPathInLayer* maxPathLength
			int offset = i * maxSingleRxPathPerLayer * (layer + 1);
			checkRxInsideKernel1D CUDA_KERNEL(numBlocks4, threadsPerBlock)(thrust::raw_pointer_cast(dev_interReflSeries.data()), thrust::raw_pointer_cast(dev_interReflSeries.data() + oldInterSize), newInterReflSize,
				thrust::raw_pointer_cast(&dev_rxPositions[i]), i, offset, layer, thrust::raw_pointer_cast(dev_rxPathNode.data() + oldPathNodeSize));//ֻ������ϵ�в���
			cudaerr = cudaDeviceSynchronize();
			if (cudaerr != cudaSuccess) {
				//LOG_ERROR << "PathTracingGPU: checkRxInsideKernel1D " << cudaGetErrorString(cudaerr) << ENDL;
			}
			//����ÿ��rx��·���ڵ�ֵ���ྶ��������һ�£�����������ͬ
		}
		auto iterEnd = thrust::remove_if(dev_rxPathNode.begin() + oldPathNodeSize, dev_rxPathNode.end(), IsInValidPathNodeGPU());
		validRxNodeSize += static_cast<int>(thrust::distance(dev_rxPathNode.begin() + oldPathNodeSize, iterEnd));



		size_t numValidReflRay = 0;
		size_t numValidTranRay = 0;
		size_t numValidDiffRay = 0;
		size_t numEvaluateReflRay = newInterReflSize;								/** @brief	Ԥ��������������	*/
		size_t numEvaluateTranRay = newInterReflSize;								/** @brief	Ԥ��͸����������	*/
		size_t numEvaluateDiffRay = numInterDiff * diffractRayNum;							/** @brief	Ԥ��������������	*/

		size_t evaluateRayNum = numEvaluateReflRay * static_cast<int>(stateHasRefl) +		/** @brief	Ԥ����������	*/
							 numEvaluateTranRay * static_cast<int>(stateHasTran) +
							 numEvaluateDiffRay * static_cast<int>(stateHasDiff);
		if (dev_initRays.capacity() < evaluateRayNum) {
			dev_initRays.reserve(static_cast<int>(evaluateRayNum * 1.2));//����1.2Ԥ���ڴ�
		}
		dev_initRays.resize(evaluateRayNum);
		ResetRay2DGPU(dev_initRays);//��ʼ����������ǰ��Ҫ��ԭʼ������Ч�Խ�������

		
		//������·����Ϊ����-һ��Ϊ��������·����һ��Ϊ��Ч����·����������ֹ·������������ֹ·����ֹ������һ��������

		if (stateHasRefl) {//��������·��
			int numBlocks5 = (static_cast<int>(numEvaluateReflRay) * threadsPerBlock - 1) / threadsPerBlock;
			generateNewReflectionRayKernel CUDA_KERNEL(numBlocks5, threadsPerBlock)(thrust::raw_pointer_cast(dev_interReflSeries.data() + oldInterSize), newInterReflSize, oldInterSize,
				thrust::raw_pointer_cast(dev_initRays.data()), segmentsGPU);
			cudaerr = cudaDeviceSynchronize();
			if (cudaerr != cudaSuccess) {
				//LOG_ERROR << "PathTracingGPU: generateNewReflectionRayKernel " << cudaGetErrorString(cudaerr) << ENDL;
			}
			numValidReflRay = static_cast<int>(thrust::distance(dev_initRays.begin(), thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU())));//��Ч�ķ���·������
		}

		if (stateHasTran) {//����͸��·��
			int numBlocks6 = (static_cast<int>(numEvaluateTranRay) * threadsPerBlock - 1) / threadsPerBlock;
			generateNewTransmitRayKernel CUDA_KERNEL(numBlocks6, threadsPerBlock)(thrust::raw_pointer_cast(dev_interReflSeries.data() + oldInterSize), newInterReflSize, oldInterSize,
				thrust::raw_pointer_cast(dev_initRays.data() + numValidReflRay), segmentsGPU);
			cudaerr = cudaDeviceSynchronize();
			if (cudaerr != cudaSuccess) {
				//LOG_ERROR << "PathTracingGPU: generateNewTransmitRayKernel " << cudaGetErrorString(cudaerr) << ENDL;
			}
			numValidTranRay = static_cast<int>(thrust::distance(dev_initRays.begin() + numValidReflRay, thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU())));
			
		}

		if (stateHasDiff && numInterDiff != 0) {//��������·��
			int numBlocks7 = (numInterDiff * diffractRayNum * threadsPerBlock - 1) / threadsPerBlock;
			generateNewDifftactRayKernel CUDA_KERNEL(numBlocks7, threadsPerBlock)(thrust::raw_pointer_cast(dev_interReflSeries.data() + oldInterSize + newInterReflSize), newInterDiffSize, oldInterSize + newInterReflSize,
				thrust::raw_pointer_cast(dev_initRays.data() + numValidReflRay + numValidTranRay), wedges);
			cudaerr = cudaDeviceSynchronize();
			if (cudaerr != cudaSuccess) {
				//LOG_ERROR << "PathTracingGPU: generateNewDifftactRayKernel " << cudaGetErrorString(cudaerr) << ENDL;
			}
			numValidDiffRay = static_cast<int>(thrust::distance(dev_initRays.begin() + numValidReflRay + numValidTranRay, thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU())));
		}

		auto iter = thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU());
		int dis = static_cast<int>(thrust::distance(dev_initRays.begin(), iter));

		numRays = numValidReflRay + numValidTranRay + numValidDiffRay;//����·����ֵ��dev_initRay���е���
		dev_initRays.resize(numRays);
		layer++;
		//LOG_INFO << "ending new ray" << ENDL;
	}
	end = std::chrono::high_resolution_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	LOG_INFO << "total time: " << duration.count() << ENDL;
	//���ڶ����н������ݣ��õ��ྶ����
	std::vector<PathNodeGPU> host_rxPathNode(validRxNodeSize);

	
	std::vector<PathNodeGPU> hostPathNodes(validRxNodeSize);																		/** @brief	�洢�ڱ��ص�·���ڵ�	*/
	thrust::copy(dev_rxPathNode.begin(), dev_rxPathNode.begin() + validRxNodeSize, hostPathNodes.begin());
	outPathNode.resize(validRxNodeSize);
	for (int i = 0; i < validRxNodeSize; ++i) {
		PathNodeGPU* newPathNode = new PathNodeGPU(hostPathNodes[i]);
		outPathNode[i] = newPathNode;
	}

	//�ͷ��Դ����ݣ����ྶ���ݵ�������
	//LOG_INFO << "start release GPU memory" << ENDL;
	dev_initRays.swap(dev_initRays);
	dev_interInit.swap(dev_interInit);
	dev_interDiffSeries.swap(dev_interDiffSeries);
	everySplitNum.swap(everySplitNum);
	dev_splitRays.swap(dev_splitRays);
	dev_InterSplit.swap(dev_InterSplit);
	dev_interReflSeries.swap(dev_interReflSeries);
	dev_rxPathNode.swap(dev_rxPathNode);
	//ClearDeviceVectorMemory(dev_initRays);//dev_initRays
	//ClearDeviceVectorMemory(dev_interInit);//dev_interInit
	//ClearDeviceVectorMemory(dev_interDiffSeries);//dev_interDiffSeries
	//ClearDeviceVectorMemory(everySplitNum);//everySplitNum
	//ClearDeviceVectorMemory(dev_splitRays);//dev_SplitRays
	//ClearDeviceVectorMemory(dev_InterSplit);//dev_interSplit
	//ClearDeviceVectorMemory(dev_interReflSeries);//dev_interReflSeries
	//ClearDeviceVectorMemory(dev_rxPathNode);//dev_rxPathNode
	//LOG_INFO << "GPU memory release complete" << ENDL;
}

//���Ǹ���GPU�㷨
//1-���Ӿ���͸��						-���
//2-����͸������߲�����				-���
//3-����͸��󲻽��з��䡢����			-���
//4-��������󲻽���͸��				-���
void PathTraceGPUOnlyTree(const std::vector<Ray2D>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, int diffractRayNum, Segment2DGPU* segmentsGPU, size_t numSegments, Wedge2DGPU* wedges, size_t numWedges, SignedDistanceFieldGPU* sdfGPU, std::vector<TreeNodeGPU*>& outTreeNodes)
{
	int threadsPerBlock = 128;

	std::chrono::high_resolution_clock::time_point start;
	std::chrono::high_resolution_clock::time_point end;
	std::chrono::milliseconds duration;
	//���pathNodeTree���д���

	//task-0 ת��ԭʼ����
	size_t numRays = rays.size();
	std::vector<Ray2DGPU> host_initRays(numRays);//ִ�����ߵ�ת��
	for (size_t i = 0; i < numRays; ++i) {
		host_initRays[i] = rays[i].Convert2GPU();
		host_initRays[i].m_isValid = true;
		host_initRays[i].m_nodeType = NODE_ROOT;
		host_initRays[i].m_limTotl = limitInfo.m_limitTotal;
		host_initRays[i].m_limRefl = limitInfo.m_limitReflect;
		host_initRays[i].m_limTran = limitInfo.m_limitTransmit;
		host_initRays[i].m_limDiff = limitInfo.m_limitDiffract;
		host_initRays[i].m_limScat = limitInfo.m_limitScatter;
	}

	//�漰������ת��


	start = std::chrono::high_resolution_clock::now();
	LOG_INFO << "running gpu code" << ENDL;

	cudaMemcpyToSymbol(D_CONST_LIM_TOTL, &limitInfo.m_limitTotal, sizeof(uint16_t));
	cudaMemcpyToSymbol(D_CONST_LIM_REFL, &limitInfo.m_limitReflect, sizeof(uint16_t));
	cudaMemcpyToSymbol(D_CONST_LIM_TRAN, &limitInfo.m_limitTransmit, sizeof(uint16_t));
	cudaMemcpyToSymbol(D_CONST_LIM_DIFF, &limitInfo.m_limitDiffract, sizeof(uint16_t));
	cudaMemcpyToSymbol(D_CONST_LIM_SCAT, &limitInfo.m_limitScatter, sizeof(uint16_t));
	cudaMemcpyToSymbol(D_CONST_RAYSPLITRADIUS, &raySplitRadius, sizeof(RtLbsType));
	cudaMemcpyToSymbol(_D_CONST_DIFFTACTRAYNUM, &diffractRayNum, sizeof(int));



	bool stateHasDiff = limitInfo.m_limitDiffract == 0 ? false : true;
	bool stateHasRefl = limitInfo.m_limitReflect == 0 ? false : true;
	bool stateHasTran = limitInfo.m_limitTransmit == 0 ? false : true;
	bool stateHasScat = limitInfo.m_limitScatter == 0 ? false : true;



	thrust::device_vector<Ray2DGPU> dev_initRays;//��ֹ��ѭ�����ж�dev_newRays�����ͷ�
	dev_initRays.reserve(numRays * 5);//Ԥ����5���ڴ�
	dev_initRays.resize(numRays);
	dev_initRays = host_initRays;


	thrust::device_vector<Intersection2DGPU> dev_interInit(numRays);//ԭʼ�����ཻ�������ѭ���н�ֹ�Դ����ݽ���ɾ������������

	thrust::device_vector<Intersection2DGPU> dev_interDiffSeries(numWedges * 1000);	/** @brief	��ǰ�����������Ѱ��������Ĭ����1000��ԭʼ���������	*/

	thrust::device_vector<int> everySplitNum(numRays); /** @brief	��¼ÿ�����ߵķ�����Ŀ	*/
	thrust::device_vector<Ray2DGPU> dev_splitRays;		/** @brief	���ѵ�����	*/

	thrust::device_vector<Intersection2DGPU> dev_InterSplit; /** @brief	�������ߵĽ�����Ϣ	*/

	thrust::device_vector<Intersection2DGPU> dev_interReflSeries; /** @brief	�ϲ��ķ���ϵ�н�����Ϣ,�������в���Ϣ��ԭʼ�������߽�����Ϣ+�������ߵĽ�����Ϣ��,��Ҫ���ڲ���rx	*/

	thrust::device_vector<TreeNodeGPU> dev_treeNodes;				/** @brief	�������ڵ�����	*/

	int validTreeNodeNum = 0;										/** @brief	��Ч���������ڵ�����	*/

	cudaError_t cudaerr;										/** @brief	cuda������	*/

	int layer = 0;
	
	while (dev_initRays.size() != 0) {
		//task-1 ---------------���м��������ཻ�����ڵ������(�������)----------------------------------------------------------------------------------------------

		numRays = dev_initRays.size();

		int numInterDiff = 0;
		if (stateHasDiff) {
			size_t numBlocks1 = (numRays + threadsPerBlock - 1) / threadsPerBlock;
			ResetIntersect2DGPU(dev_interDiffSeries);
			cudaMemcpyToSymbol(G_DIFFID, &C_ZERO, sizeof(int));//��ʼ��ԭ������

			for (int i = 0; i < numWedges; ++i) {//�������������������������
				diffractionFindKernel1D CUDA_KERNEL(numBlocks1, threadsPerBlock)(thrust::raw_pointer_cast(dev_initRays.data()), numRays,
					&wedges[i], sdfGPU, segmentsGPU, thrust::raw_pointer_cast(dev_interDiffSeries.data()));
				cudaerr = cudaDeviceSynchronize();
				if (cudaerr != cudaSuccess) {
					//LOG_ERROR << "PathTracingGPU: diffractionFindKernel1D " << cudaGetErrorString(cudaerr) << ENDL;
				}
			}
			auto iter_end = thrust::remove_if(dev_interDiffSeries.begin(), dev_interDiffSeries.end(), IsInValidIntersectGPU());
			numInterDiff = static_cast<int>(thrust::distance(dev_interDiffSeries.begin(), iter_end));//������Ч����ƫ��ֵ
			//LOG_INFO <<layer<<": " << numInterDiff << ENDL;
			//LOG_INFO << "end find diff" << ENDL;
		}



		//task-2----------------- ���м������߷���---------------------------------------------------------------------------------------------------------
		//2-1	Ԥ������Ѹ���
		dev_interInit.resize(numRays);//��չΪ��������
		ResetIntersect2DGPU(dev_interInit);///--dev_initIntersects���г�ʼ��

		//����ԭʼ���߽�����Ϣ�����߷����������������з������Ե�ԭʼ���ߣ��������߱��������ԵĽ�����Ϣ
		size_t numInterNoSplit = numRays;
		size_t numInterSplit = 0;
		size_t numRaySplit = 0;
		if (raySplitFlag == true) {
			size_t numBlocks2 = (numRays + threadsPerBlock - 1) / threadsPerBlock;
			everySplitNum.resize(numRays);
			thrust::fill(everySplitNum.begin(), everySplitNum.end(), 0);//����ÿ�����߷�������
			intersectAndCalculateRaySplitNumKernelLBS CUDA_KERNEL(numBlocks2, threadsPerBlock)(thrust::raw_pointer_cast(dev_initRays.data()),
				thrust::raw_pointer_cast(dev_interInit.data()), thrust::raw_pointer_cast(everySplitNum.data()), sdfGPU, segmentsGPU, numRays);
			cudaerr = cudaDeviceSynchronize();//�ȴ��̼߳������
			if (cudaerr != cudaSuccess) {
				//LOG_ERROR << "PathTracingGPU: intersectAndCalculateRaySplitNumKernel " << cudaGetErrorString(cudaerr) << ENDL;
			}

			//������Ч�Ľ�����Ϣ-����������������ʹ��
			auto iterEnd_validInter = thrust::remove_if(dev_interInit.begin(), dev_interInit.end(), IsInValidIntersectGPU());
			numInterNoSplit = static_cast<int>(thrust::distance(dev_interInit.begin(), iterEnd_validInter));//δ���ѵ�ԭʼ������Ϣ��Ŀ

			//������Ч������-������ʹ��
			auto iterEnd_validRay = thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU());
			numInterSplit = static_cast<int>(thrust::distance(dev_initRays.begin(), iterEnd_validRay));//�ѷ��ѵ�ԭʼ������Ϣ��Ŀ

			//��������Ŀ��Ϊ0��ǰ��
			auto iterEnd_validEverySplitNum = thrust::remove_if(everySplitNum.begin(), everySplitNum.end(), is_zero());//�����ѵ�������ԭʼ�ķ���������м���������ߣ��õ�splitRays

			//�������������Ŀ����
			numRaySplit = thrust::reduce(everySplitNum.begin(), iterEnd_validEverySplitNum, 0);

			if (numRaySplit != 0) {													//��Ԥ�������������Ϊ0���������ʵ���Ѽ���
				dev_splitRays.resize(numRaySplit);//�����ѵ����߽���resize
				ResetRay2DGPU(dev_splitRays);

				size_t numblocks_split = (numInterSplit + threadsPerBlock - 1) / threadsPerBlock;//ע�⣬�����ԭʼ�ɷ��������Ѿ�ǰ�ã�������ƥ��˼����д���
				raySplitKernel CUDA_KERNEL(numblocks_split, threadsPerBlock)(thrust::raw_pointer_cast(dev_initRays.data()), numInterSplit,
					thrust::raw_pointer_cast(dev_splitRays.data()), thrust::raw_pointer_cast(everySplitNum.data()), segmentsGPU);
				cudaerr = cudaDeviceSynchronize();
				if (cudaerr != cudaSuccess) {
					//LOG_ERROR << "PathTracingGPU: raySplitKernel " << cudaGetErrorString(cudaerr) << ENDL;
				}

				//������Ч�ķ�������
				auto interEnd_validSplitRay = thrust::remove_if(dev_splitRays.begin(), dev_splitRays.end(), IsInValidRayGPU());
				numRaySplit = static_cast<int>(thrust::distance(dev_splitRays.begin(), interEnd_validSplitRay));//���·������ߵ���Ŀ
				//task-3------------------ ���м�����������ཻ------------------------------------------------------------------------------------------------------
				dev_InterSplit.resize(numRaySplit);//�Է��ѵ����߽�����Ϣ����resize
				ResetIntersect2DGPU(dev_InterSplit);

				int numBlocks3 = (numRaySplit + threadsPerBlock - 1) / threadsPerBlock;
				intersectKernel CUDA_KERNEL(numBlocks3, threadsPerBlock)(thrust::raw_pointer_cast(dev_splitRays.data()), numRaySplit,
					thrust::raw_pointer_cast(dev_InterSplit.data()), sdfGPU, segmentsGPU);
				cudaerr = cudaDeviceSynchronize();
				if (cudaerr != cudaSuccess) {
					//LOG_ERROR << "PathTracingGPU: intersectKernel " << cudaGetErrorString(cudaerr) << ENDL;
				}
			}
		}


		//task-4------------------ �ϲ�����/δ�������ߵ�Inter ���--------------------------------------------------------------------------------------------
		size_t oldInterSize = dev_interReflSeries.size(); //��һ�㽻����Ϣ����
		size_t newInterReflSize = numInterNoSplit + numRaySplit;// ��ǰ��ķ���ϵ�н�����Ϣ����
		size_t newInterDiffSize = numInterDiff;   //��ǰ�������ϵ�еĽ�����Ϣ����

		dev_interReflSeries.resize(oldInterSize + newInterReflSize + newInterDiffSize);//�Է���ϵ������Ϣ����resize

		//��δ�������߽�����Ϣ�ͷ������߽�����Ϣ�ж�ȡ����
		thrust::copy(dev_interInit.begin(), dev_interInit.begin() + numInterNoSplit, dev_interReflSeries.begin() + oldInterSize);											//����ʼδ���ѽ��㸽��������ϵ�н���
		thrust::copy(dev_InterSplit.begin(), dev_InterSplit.begin() + numRaySplit, dev_interReflSeries.begin() + oldInterSize + numInterNoSplit);							//�����ѽ��㸽��������ϵ�н����
		thrust::copy(dev_interDiffSeries.begin(), dev_interDiffSeries.begin() + newInterDiffSize, dev_interReflSeries.begin() + oldInterSize + numInterNoSplit + numRaySplit);	//������ϵ���㸽��������ϵ�н���֮��

		size_t numValidReflRay = 0;
		size_t numValidTranRay = 0;
		size_t numValidDiffRay = 0;
		size_t numEvaluateReflRay = newInterReflSize;								/** @brief	Ԥ��������������	*/
		size_t numEvaluateTranRay = newInterReflSize;								/** @brief	Ԥ��͸����������	*/
		size_t numEvaluateDiffRay = numInterDiff * diffractRayNum;							/** @brief	Ԥ��������������	*/

		size_t evaluateRayNum = numEvaluateReflRay * static_cast<int>(stateHasRefl) +		/** @brief	Ԥ����������	*/
			numEvaluateTranRay * static_cast<int>(stateHasTran) +
			numEvaluateDiffRay * static_cast<int>(stateHasDiff);
		if (dev_initRays.capacity() < evaluateRayNum) {
			dev_initRays.reserve(static_cast<int>(evaluateRayNum * 1.2));//����1.2Ԥ���ڴ�
		}
		dev_initRays.resize(evaluateRayNum);
		ResetRay2DGPU(dev_initRays);//��ʼ����������ǰ��Ҫ��ԭʼ������Ч�Խ�������

		//���ӿ����������ڵ�����
		size_t oldTreeNodeSize = dev_treeNodes.size();								/** @brief	��һ��ε����ڵ������	*/
		size_t newTreeNodeSize = oldTreeNodeSize + evaluateRayNum;					/** @brief	�µ����ڵ������	*/

		dev_treeNodes.resize(newTreeNodeSize);

		//������·����Ϊ����-һ��Ϊ��������·����һ��Ϊ��Ч����·����������ֹ·������������ֹ·����ֹ������һ��������

		if (stateHasRefl) {//��������·��
			size_t numBlocks5 = (numEvaluateReflRay * threadsPerBlock - 1) / threadsPerBlock;
			generateNewReflectionRayWithNodeKernel CUDA_KERNEL(numBlocks5, threadsPerBlock)(thrust::raw_pointer_cast(dev_interReflSeries.data() + oldInterSize), newInterReflSize, oldInterSize,
				thrust::raw_pointer_cast(dev_initRays.data()), thrust::raw_pointer_cast(dev_treeNodes.data() + oldInterSize), segmentsGPU, layer);
			cudaerr = cudaDeviceSynchronize();
			if (cudaerr != cudaSuccess) {
				//LOG_ERROR << "PathTracingGPU: generateNewReflectionRayKernel " << cudaGetErrorString(cudaerr) << ENDL;
			}
			numValidReflRay = static_cast<int>(thrust::distance(dev_initRays.begin(), thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU())));//��Ч�ķ���·������
		}

		if (stateHasTran) {//����͸��·��
			size_t numBlocks6 = (numEvaluateTranRay * threadsPerBlock - 1) / threadsPerBlock;
			generateNewTransmitRayWithNodeKernel CUDA_KERNEL(numBlocks6, threadsPerBlock)(thrust::raw_pointer_cast(dev_interReflSeries.data() + oldInterSize), newInterReflSize, oldInterSize,
				thrust::raw_pointer_cast(dev_initRays.data() + numValidReflRay), thrust::raw_pointer_cast(dev_treeNodes.data() + oldInterSize + numValidReflRay), segmentsGPU, layer);
			cudaerr = cudaDeviceSynchronize();
			if (cudaerr != cudaSuccess) {
				LOG_ERROR << "PathTracingGPU: generateNewTransmitRayKernel " << cudaGetErrorString(cudaerr) << ENDL;
			}
			numValidTranRay = static_cast<int>(thrust::distance(dev_initRays.begin() + numValidReflRay, thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU())));

		}

		if (stateHasDiff && numInterDiff != 0) {//��������·��
			size_t numBlocks7 = (numInterDiff * diffractRayNum * threadsPerBlock - 1) / threadsPerBlock;
			generateNewDifftactRayWithNodeKernel CUDA_KERNEL(numBlocks7, threadsPerBlock)(thrust::raw_pointer_cast(dev_interReflSeries.data() + oldInterSize + newInterReflSize), newInterDiffSize, oldInterSize + newInterReflSize,
				thrust::raw_pointer_cast(dev_initRays.data() + numValidReflRay + numValidTranRay), thrust::raw_pointer_cast(dev_treeNodes.data() + oldInterSize + numValidReflRay + numValidTranRay), wedges, layer);
			cudaerr = cudaDeviceSynchronize();
			if (cudaerr != cudaSuccess) {
				//LOG_ERROR << "PathTracingGPU: generateNewDifftactRayKernel " << cudaGetErrorString(cudaerr) << ENDL;
			}
			numValidDiffRay = static_cast<int>(thrust::distance(dev_initRays.begin() + numValidReflRay + numValidTranRay, thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU())));
		}

		auto iter = thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU());
		int dis = static_cast<int>(thrust::distance(dev_initRays.begin(), iter));

		numRays = numValidReflRay + numValidTranRay + numValidDiffRay;//����·����ֵ��dev_initRay���е���
		dev_initRays.resize(numRays);

		validTreeNodeNum += static_cast<int>(thrust::distance(dev_treeNodes.begin() + oldTreeNodeSize, thrust::remove_if(dev_treeNodes.begin(), dev_treeNodes.end(), IsInValidTreeNodeGPU())));
		dev_treeNodes.resize(validTreeNodeNum);			//�ж������߾ͻ��ж��ٸ����ڵ�
		layer++;
	}
	end = std::chrono::high_resolution_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	//LOG_INFO << "total time: " << duration.count() << ENDL;

	//�����ڵ�������CPU��
	std::vector<TreeNodeGPU> hostTreeNodes(validTreeNodeNum);																		/** @brief	�洢�ڱ��ص�·���ڵ�	*/
	thrust::copy(dev_treeNodes.begin(), dev_treeNodes.begin() + validTreeNodeNum, hostTreeNodes.begin());
	outTreeNodes.resize(validTreeNodeNum);
	for (int i = 0; i < validTreeNodeNum; ++i) {
		TreeNodeGPU* newTreeNode = new TreeNodeGPU(hostTreeNodes[i]);
		outTreeNodes[i] = newTreeNode;
	}


	//�ͷ��Դ����ݣ����ྶ���ݵ�������
	//LOG_INFO << "start release GPU memory" << ENDL;
	dev_initRays.swap(dev_initRays);
	dev_interInit.swap(dev_interInit);
	dev_interDiffSeries.swap(dev_interDiffSeries);
	everySplitNum.swap(everySplitNum);
	dev_splitRays.swap(dev_splitRays);
	dev_InterSplit.swap(dev_InterSplit);
	dev_interReflSeries.swap(dev_interReflSeries);
	//LOG_INFO << "GPU memory release complete" << ENDL;
}

bool PathTraceGPULite(RayPathGPU& inpath, const Segment2DGPU* segments)
{
	//����˼���ǻ���ԭʼ���ߣ����͸���������ȣ�ͨ��ԭʼ���߷��ѷ����𲽱ƽ���ʵ��͸��rx�����λ��,ע�⣬���������ģʽΪ��������ģʽ
	//��������,������Ž�
	std::vector<Ray2DGPU> splitRays; //��Ҫ���ѵ�����
	const std::vector<PathNodeGPU*>& nodes = inpath.m_nodes;//·���ڵ���Ϣ

	PathNodeGPU* frontNode = nodes.back();//ע�⣬�����޸�Ϊ���򣬴ӹ���Դ�ڵ㴫����Ŀ���
	PathNodeGPU* backNode = nodes.front();
	const PathNodeGPU* secondNode = *prev(nodes.end());//������Ҫ��������ĵڶ����ڵ�Ϊ�����ڶ����ڵ�
	const Ray2DGPU& initRay = secondNode->m_inter.m_ray;
	//��⵽Ŀ�������߰뾶r
	RtLbsType t = backNode->m_inter.m_ray.m_tMax - backNode->m_inter.m_ray.m_tMin + (backNode->m_inter.m_intersect - backNode->m_inter.m_ray.m_Ori).Length();
	RtLbsType r = initRay.GetRayRadis(t);
	int splitNum = static_cast<int>(ceil(r / TRAN_EPSILON));
	double theta = initRay.m_theta * 2.0;
	double dtheta = theta / splitNum;
	double halftheta = dtheta / 2.0;
	double coshalftheta = cos(halftheta);
	Vector2D newDir = Rotate(initRay.m_Dir, -initRay.m_theta);//��ת��������Сֵ
	splitRays.resize(splitNum);
	//���ߵĳ�ʼ��
	newDir.Rotate(halftheta);//��һ������
	splitRays[0] = initRay;
	splitRays[0].m_Dir = newDir;
	splitRays[0].m_theta = halftheta;
	splitRays[0].m_costheta = coshalftheta;

	for (size_t i = 1; i < splitRays.size(); ++i) {//��������
		newDir.Rotate(dtheta);
		splitRays[i] = initRay;
		splitRays[i].m_Dir = newDir;
		splitRays[i].m_theta = halftheta;
		splitRays[i].m_costheta = coshalftheta;
	}

	std::vector<RayPathGPU> outpaths;//�����·��
	std::vector<Ray2DGPU> nextrays;//ĩβ�ڵ�������
	//���㲢��������
	for (Ray2DGPU& splitray : splitRays) {//������������
		RayPathGPU newPath;
		newPath.m_nodes.push_back(frontNode);
		Ray2DGPU iterateRay(splitray); //���ڵ���������
		auto it = prev(nodes.end(), 2);
		while (it != nodes.begin()) {//�ٳ���βԪ�أ������ڵ㲢��������׷��
			PathNodeGPU* node = *it;
			Intersection2DGPU inter;
			const Segment2DGPU& segment = segments[node->m_inter.m_segmentId];
			if (!segment.GetIntersect(iterateRay, &inter)) //��������ڵ���Ԫ���ཻ������׷��·��
				break;
			if (node->m_inter.m_type == NODE_REFL) {//����ڵ�
				Ray2DGPU reflectRay;
				if(!GenerateReflectRayGPU(node->m_inter, 0, segments, &reflectRay))//���������˷���·�����������������
					break;
				PathNodeGPU* newNode = new PathNodeGPU(true, node->m_layer, node->m_rxId, inter);
				newPath.m_nodes.push_back(newNode);
				iterateRay = reflectRay;
			}
			if (node->m_inter.m_type == NODE_TRANIN || node->m_inter.m_type == NODE_TRANOUT) {//͸��ڵ�
				Ray2DGPU transmitRay;
				if (!GenerateTransmitRayGPU(node->m_inter, 0, segments, &transmitRay))
					break;
				PathNodeGPU* newNode = new PathNodeGPU(true, node->m_layer, node->m_rxId, inter);
				newPath.m_nodes.push_back(newNode);
				iterateRay = transmitRay;
			}
			if (node->m_inter.m_type == NODE_ETRANIN || node->m_inter.m_type == NODE_ETRANOUT) {//͸��ڵ�
				Ray2DGPU transmitRay;
				if (!GenerateEmpiricalTransmitRayGPU(node->m_inter, 0, segments, &transmitRay))
					break;
				PathNodeGPU* newNode = new PathNodeGPU(true, node->m_layer, node->m_rxId, inter);
				newPath.m_nodes.push_back(newNode);
				iterateRay = transmitRay;
			}
			--it;
		}
		
		if (newPath.m_nodes.size() != (inpath.m_nodes.size() - 1))//�ٳ�ĩβԪ�غ����·����������ԭʼ����·������ͬ����������Ч·��
			continue;
		outpaths.push_back(newPath);
		nextrays.push_back(iterateRay);
	}
	if (outpaths.empty())//����ǰ·������Ϊ�գ�����·��������
		return false;

	//��������rx�����·��
	RayPathGPU targetPath;
	RtLbsType tmin = FLT_MAX;
	for (int i = 0; i < outpaths.size(); ++i) {
		RayPathGPU& curPath = outpaths[i];
		RtLbsType tcur = nextrays[i].GetSquaredDistanceToPoint(backNode->m_inter.m_intersect);
		if (tcur < tmin) {
			tmin = tcur;
			targetPath = curPath;
		}
	}
	targetPath.m_nodes.push_back(backNode);
	reverse(targetPath.m_nodes.begin(), targetPath.m_nodes.end());
	inpath = targetPath;
	return true;
}

