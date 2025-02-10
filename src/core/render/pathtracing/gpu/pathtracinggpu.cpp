#include "pathtracinggpu.h"
#include "radiowave/raypath/gpu/raypathgpu.h"
#include <fstream>
#include <chrono>

__constant__ uint16_t D_CONST_LIM_TOTL; /** @brief	设备端全局--总限制数	*/
__constant__ uint16_t D_CONST_LIM_REFL;	/** @brief	设备端全局--反射限制数	*/
__constant__ uint16_t D_CONST_LIM_TRAN; /** @brief	设备端全局--透射限制数	*/
__constant__ uint16_t D_CONST_LIM_DIFF; /** @brief	设备端全局--绕射限制数	*/
__constant__ uint16_t D_CONST_LIM_SCAT; /** @brief	设备端全局--散射限制数	*/
__constant__ RtLbsType D_CONST_RAYSPLITRADIUS;	/** @brief	设备端全局--射线分裂半径	*/
__constant__ int _D_CONST_DIFFTACTRAYNUM;		/** @brief	设备端全局--绕射射线数量	*/


__device__ int G_DIFFID = 0; /** @brief	用于绕射点寻找的原子锁操作索引	*/
__device__ int G_RXID = 0; /** @brief	用于接收点rx坐标判定的原子锁操作索引	*/
const int C_ZERO = 0;		/** @brief	用于重置每层索引编号	*/

__global__ void GPUDiffFinderKernel(Ray2DGPU* ray, Intersection2DGPU* intersect, int* everyIntersectNum, Wedge2DGPU* wedge, int wedgeCount, int size)
{
	int tid = blockIdx.x * blockDim.x + threadIdx.x;//获得线程编号
	//计算每一个线程的编号，然后在intersect内存中寻找到对应的内存位置，并写入数据
	if (tid < size) {
		int containWedgesNum = 0;
		for (int i = 0; i < wedgeCount; ++i) {
			if (/*当前节点在射线内*/true) {
				Intersection2DGPU newIntersect;//新创建的交点信息
				intersect[tid * 50 + containWedgesNum] = newIntersect;//这里的50代表每个射线最大可包含的绕射点
				containWedgesNum++;
			}
		}
		everyIntersectNum[tid] = containWedgesNum;
	}
}


/**
* @brief		绕射点寻找函数1维度
* @param[in]	
* @return		
* @author		胡硕
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
			intersects[idx].m_prevId = rays[rayIdx].m_prevInterId;//上一个交点信息编号传递
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
		//附加计算每条射线的分裂数目
		RtLbsType radius = ray[tid].GetRayRadis(intersect[tid].m_ft);
		intersect[tid].m_prevId = ray[tid].m_prevInterId;
		if (radius > D_CONST_RAYSPLITRADIUS) {
			everySplitNum[tid] = static_cast<int>(ceil(2 * radius / D_CONST_RAYSPLITRADIUS));
			intersect[tid].m_isValid = false;//不保留已分裂的交点信息，对后续无用
		}
		else {
			everySplitNum[tid] = 0;
			ray[tid].m_isValid = false;//不保留未分裂的射线，对分裂射线无用
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
			return;											//透射后不进行分裂操作
		}
		sdf->GetIntersect(ray[tid], &intersect[tid], segments);
		//附加计算每条射线的分裂数目
		RtLbsType radius = ray[tid].GetRayRadis(intersect[tid].m_ft);
		intersect[tid].m_prevId = ray[tid].m_prevInterId;
		if (radius > D_CONST_RAYSPLITRADIUS) {
			everySplitNum[tid] = static_cast<int>(ceil(2 * radius / D_CONST_RAYSPLITRADIUS));
			intersect[tid].m_isValid = false;//不保留已分裂的交点信息，对后续无用
		}
		else {
			everySplitNum[tid] = 0;
			ray[tid].m_isValid = false;//不保留未分裂的射线，对分裂射线无用
		}
	}
}

__global__ void raySplitKernel(Ray2DGPU* rays, size_t numRays, Ray2DGPU* newRays, int* everySplitNum, Segment2DGPU* segments)
{
	int rayIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (rayIdx < numRays) {
		if (everySplitNum[rayIdx] > 0) {
			int startIdx = 0;
			for (int i = 0; i < rayIdx; ++i) {//计算到当前射线分裂数量的索引值
				startIdx += everySplitNum[i];
			}
			GenerateSplittingRayGPU(rays[rayIdx], everySplitNum[rayIdx], &newRays[startIdx], segments);//找到地址之后直接在数组上进行赋值
		}
	}
}

__global__ void checkRxInsideKernel1D(Intersection2DGPU* allIntersects, Intersection2DGPU* intersects, int numIntersects, Point2D* rx, int rxId, int offset, int layer, PathNodeGPU* pathNode) {
	int intersectIdx = blockIdx.x * blockDim.x + threadIdx.x;
	if (intersectIdx < numIntersects) {
		Intersection2DGPU* intersect = &intersects[intersectIdx];
		if (intersect->IsCaptureRx(*rx)) {
			int idx = atomicAdd(&G_RXID, (layer + 1)) + offset;//在原子锁的基础上加上针对每个rx设计的地址偏移量，offset=rxId*300*(layer+1);,这里的layer+1是为了保留每个接收点的所有追溯路径数据
			pathNode[idx].m_inter = intersects[intersectIdx];
			pathNode[idx].m_isValid = true;
			pathNode[idx].m_layer = layer;
			pathNode[idx].m_rxId = rxId;
			Intersection2DGPU *curInter = &intersects[intersectIdx];//当前迭代的交点信息指针
			for (int i = 1; i < layer + 1; ++i) {//刨根问底追溯到所有数据,这里的i从1开始只是为了适配pathNode增加的编号
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
			if (ray.m_limRefl <= 0 || ray.m_limTotl <= 0)			// 若反射射线限制数达到限制或总限制数达到限制，则直接返回
				return;
			if (!intersect.m_propagationProperty.m_hasRelfection)	//若交点所在的传播属性中无反射属性，则直接返回
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
			if (ray.m_limRefl <= 0 || ray.m_limTotl <= 0)			// 若反射射线限制数达到限制或总限制数达到限制，则直接返回
				return;
			if (!intersect.m_propagationProperty.m_hasRelfection)	//若交点所在的传播属性中无反射属性，则直接返回
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
			if (intersect.m_propagationProperty.m_hasTransmission) {	//若交点所在的传播属性中无透射属性，则直接返回
				if (ray.m_limTran <= 0 || ray.m_limTotl <= 0) {			//若透射射线限制数达到限制或总限制数达到限制，则直接返回
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
			if (intersect.m_propagationProperty.m_hasTransmission) {	//若交点所在的传播属性中无透射属性，则直接返回
				if (ray.m_limTran <= 0 || ray.m_limTotl <= 0) {			//若透射射线限制数达到限制或总限制数达到限制，则直接返回
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
			
			if (ray.m_limDiff <= 0 || ray.m_limTotl <= 0)			//若绕射射线限制数达到限制或总限制数达到限制，则直接返回
				return;
			if (!intersect.m_propagationProperty.m_hasDiffraction)	//若交点所在的传播属性中无绕射属性，则直接返回
				return;
			
			int interId = intersectIdx + prevInterSize;
			GenerateDiffractRaysGPU(intersect, ray, wedge, interId, &newRays[intersectIdx * _D_CONST_DIFFTACTRAYNUM], _D_CONST_DIFFTACTRAYNUM);//当且仅当在满足限制数条件下才可进行产生新射线
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

			if (ray.m_limDiff <= 0 || ray.m_limTotl <= 0)			//若绕射射线限制数达到限制或总限制数达到限制，则直接返回
				return;
			if (!intersect.m_propagationProperty.m_hasDiffraction)	//若交点所在的传播属性中无绕射属性，则直接返回
				return;

			int interId = intersectIdx + prevInterSize;
			GenerateDiffractRaysGPU(intersect, ray, wedge, interId, &newRays[intersectIdx * _D_CONST_DIFFTACTRAYNUM], &newNodes[intersectIdx * _D_CONST_DIFFTACTRAYNUM], _D_CONST_DIFFTACTRAYNUM, layer);//当且仅当在满足限制数条件下才可进行产生新射线
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
		return;			//还未考虑，故不做修正
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


//全局函数-射线追踪GPU版本，采用GPU并行填满pathNodeTree,调试阶段，只采用固定单个Tx作为测试
void PathTraceGPU(const std::vector<Ray2D>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, int diffractRayNum, Point2D* dev_rxPositions, size_t numRxPositions, Segment2DGPU* segmentsGPU, size_t numSegments, Wedge2DGPU* wedges, size_t numWedges, SignedDistanceFieldGPU* sdfGPU, std::vector<PathNodeGPU*>& outPathNode) {
	int threadsPerBlock = 128;

	std::chrono::high_resolution_clock::time_point start;
	std::chrono::high_resolution_clock::time_point end;
	std::chrono::milliseconds duration;
	//针对pathNodeTree进行处理

	//task-0 转化原始射线
	size_t numRays = rays.size();
	std::vector<Ray2DGPU> host_initRays(numRays);//执行射线的转换
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

	//涉及场景的转换


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


	
	thrust::device_vector<Ray2DGPU> dev_initRays;//禁止在循环体中对dev_newRays进行释放
	dev_initRays.reserve(numRays * 5);//预分配5倍内存
	dev_initRays.resize(numRays);
	dev_initRays = host_initRays;


	thrust::device_vector<Intersection2DGPU> dev_interInit(numRays);//原始射线相交结果，在循环中禁止对此数据进行删除或缩减操作

	thrust::device_vector<Intersection2DGPU> dev_interDiffSeries(numWedges*1000);	/** @brief	当前层射线绕射点寻找总数，默认是1000倍原始绕射点数量	*/

	thrust::device_vector<int> everySplitNum(numRays); /** @brief	记录每条射线的分裂数目	*/
	thrust::device_vector<Ray2DGPU> dev_splitRays;		/** @brief	分裂的射线	*/

	thrust::device_vector<Intersection2DGPU> dev_InterSplit; /** @brief	分裂射线的交点信息	*/

	thrust::device_vector<Intersection2DGPU> dev_interReflSeries; /** @brief	合并的反射系列交点信息,包含所有层信息（原始保留射线交点信息+分裂射线的交点信息）,主要用于捕获rx	*/

	int maxSingleRxPathPerLayer = 1500; //每层每个rx最大500条路径
	int validRxNodeSize = 0;//叠加每一层有效的rxPathNode数量
	thrust::device_vector<PathNodeGPU> dev_rxPathNode;/** @brief	存储所有Rx交点捕获的变量*/
	
	cudaError_t cudaerr;										/** @brief	cuda错误处理	*/

	int layer = 0;
	while (dev_initRays.size() != 0) {
		//task-1 ---------------并行计算射线相交射线内的绕射点(修缮完成)----------------------------------------------------------------------------------------------

		numRays = dev_initRays.size();

		int numInterDiff = 0;
		if (stateHasDiff) {
			int numBlocks1 = (static_cast<int>(numRays) + threadsPerBlock - 1) / threadsPerBlock;
			ResetIntersect2DGPU(dev_interDiffSeries);
			cudaMemcpyToSymbol(G_DIFFID, &C_ZERO, sizeof(int));//初始化原子索引
			
			for (int i = 0; i < numWedges; ++i) {//若射线中无绕射生命，则放弃
				diffractionFindKernel1D CUDA_KERNEL(numBlocks1, threadsPerBlock)(thrust::raw_pointer_cast(dev_initRays.data()), static_cast<int>(numRays),
					&wedges[i], sdfGPU, segmentsGPU, thrust::raw_pointer_cast(dev_interDiffSeries.data()));
				cudaerr = cudaDeviceSynchronize();
				if (cudaerr != cudaSuccess) {
					//LOG_ERROR <<"PathTracingGPU: diffractionFindKernel1D " << cudaGetErrorString(cudaerr) << ENDL;	
				}
			}
			auto iter_end = thrust::remove_if(dev_interDiffSeries.begin(), dev_interDiffSeries.end(), IsInValidIntersectGPU());
			numInterDiff = static_cast<int>(thrust::distance(dev_interDiffSeries.begin(), iter_end));//计算有效数据偏离值
			//LOG_INFO <<layer<<": " << numInterDiff << ENDL;
			//LOG_INFO << "end find diff" << ENDL;
		}
		


		//task-2----------------- 并行计算射线分裂---------------------------------------------------------------------------------------------------------
		//2-1	预计算分裂个数
		dev_interInit.resize(numRays);//扩展为射线数量
		ResetIntersect2DGPU(dev_interInit);///--dev_initIntersects进行初始化

		//计算原始射线交点信息及射线分裂数量，保留具有分裂属性的原始射线，保留不具备分裂属性的交点信息
		size_t numInterNoSplit = numRays;
		size_t numInterSplit = 0;
		size_t numRaySplit = 0;
		if (raySplitFlag == true) {
			int numBlocks2 = (static_cast<int>(numRays) + threadsPerBlock - 1) / threadsPerBlock;
			everySplitNum.resize(numRays);
			thrust::fill(everySplitNum.begin(), everySplitNum.end(), 0);//重置每条射线分裂数量
			intersectAndCalculateRaySplitNumKernel CUDA_KERNEL(numBlocks2, threadsPerBlock)(thrust::raw_pointer_cast(dev_initRays.data()),
				thrust::raw_pointer_cast(dev_interInit.data()), thrust::raw_pointer_cast(everySplitNum.data()), sdfGPU, segmentsGPU, static_cast<int>(numRays));
			cudaerr = cudaDeviceSynchronize();//等待线程计算完成
			if (cudaerr != cudaSuccess) {
				//LOG_ERROR << "PathTracingGPU: intersectAndCalculateRaySplitNumKernel " << cudaGetErrorString(cudaerr) << ENDL;
			}

			//保留有效的交点信息-供后续捕获及新射线使用
			auto iterEnd_validInter = thrust::remove_if(dev_interInit.begin(), dev_interInit.end(), IsInValidIntersectGPU());
			numInterNoSplit = static_cast<int>(thrust::distance(dev_interInit.begin(), iterEnd_validInter));//未分裂的原始交点信息数目

			//保留有效的射线-供分裂使用
			auto iterEnd_validRay = thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU());
			numInterSplit = static_cast<int>(thrust::distance(dev_initRays.begin(), iterEnd_validRay));//已分裂的原始交点信息数目

			//将分裂数目不为0的前置
			auto iterEnd_validEverySplitNum = thrust::remove_if(everySplitNum.begin(), everySplitNum.end(), is_zero());//将分裂的射线与原始的分裂数组进行计算分裂射线，得到splitRays

			//计算分裂射线数目总量
			numRaySplit = thrust::reduce(everySplitNum.begin(), iterEnd_validEverySplitNum, 0);

			if (numRaySplit != 0) {													//若预计算分裂数量不为0，则进行真实分裂计算
				dev_splitRays.resize(numRaySplit);//将分裂的射线进行resize
				ResetRay2DGPU(dev_splitRays);

				size_t numblocks_split = (numInterSplit + threadsPerBlock - 1) / threadsPerBlock;//注意，这里的原始可分裂射线已经前置，请利用匹配思想进行处理
				raySplitKernel CUDA_KERNEL(numblocks_split, threadsPerBlock)(thrust::raw_pointer_cast(dev_initRays.data()), numInterSplit,
					thrust::raw_pointer_cast(dev_splitRays.data()), thrust::raw_pointer_cast(everySplitNum.data()), segmentsGPU);
				cudaerr = cudaDeviceSynchronize();
				if (cudaerr != cudaSuccess) {
					//LOG_ERROR << "PathTracingGPU: raySplitKernel " << cudaGetErrorString(cudaerr) << ENDL;
				}

				//保留有效的分裂射线
				auto interEnd_validSplitRay = thrust::remove_if(dev_splitRays.begin(), dev_splitRays.end(), IsInValidRayGPU());
				numRaySplit = static_cast<int>(thrust::distance(dev_splitRays.begin(), interEnd_validSplitRay));//更新分裂射线的数目
				//task-3------------------ 并行计算分裂射线相交------------------------------------------------------------------------------------------------------
				dev_InterSplit.resize(numRaySplit);//对分裂的射线交点信息进行resize
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
		


		

		//task-4------------------ 合并分裂/未分裂射线的Inter 结果--------------------------------------------------------------------------------------------
		size_t oldInterSize = dev_interReflSeries.size(); //上一层交点信息数量
		size_t newInterReflSize = numInterNoSplit + numRaySplit;// 当前层的反射系列交点信息数量
		size_t newInterDiffSize = numInterDiff;   //当前层的绕射系列的交点信息数量

		dev_interReflSeries.resize(oldInterSize + newInterReflSize + newInterDiffSize);//对反射系交点信息进行resize

		//从未分裂射线交点信息和分裂射线交点信息中读取数据
		thrust::copy(dev_interInit.begin(), dev_interInit.begin() + numInterNoSplit, dev_interReflSeries.begin() + oldInterSize);											//将初始未分裂交点附加至反射系列交点
		thrust::copy(dev_InterSplit.begin(), dev_InterSplit.begin() + numRaySplit, dev_interReflSeries.begin() + oldInterSize + numInterNoSplit);							//将分裂交点附加至反射系列交点后
		thrust::copy(dev_interDiffSeries.begin(), dev_interDiffSeries.begin() + newInterDiffSize, dev_interReflSeries.begin() + oldInterSize + numInterNoSplit + numRaySplit);	//将绕射系交点附加至反射系列交点之后

		
		//task-4------------------------------ 并行计算被rx捕获，输出结果------------------------------------------------------------------------------
		//4-1 执行rx截获检测
		//LOG_INFO << "start capture" << ENDL;
		int numBlocks4 = (static_cast<int>(newInterReflSize) + threadsPerBlock - 1) / threadsPerBlock;
		//ResetPathNodeGPU(dev_rxPathNode);//初始化rx捕获值
		int oldPathNodeSize = validRxNodeSize;
		int newPathNodeSize = static_cast<int>(numRxPositions) * maxSingleRxPathPerLayer * (layer + 1);//每层预估的最大容量
		int pathNodeSize = oldPathNodeSize + newPathNodeSize;
		dev_rxPathNode.resize(pathNodeSize);
		
		for (int i = 0; i < numRxPositions; ++i) {
			//原子锁模式
			cudaMemcpyToSymbol(G_RXID, &C_ZERO, sizeof(int));//初始化原子索引
			//maxLayer* rx_positions.size() * maxPathInLayer* maxPathLength
			int offset = i * maxSingleRxPathPerLayer * (layer + 1);
			checkRxInsideKernel1D CUDA_KERNEL(numBlocks4, threadsPerBlock)(thrust::raw_pointer_cast(dev_interReflSeries.data()), thrust::raw_pointer_cast(dev_interReflSeries.data() + oldInterSize), newInterReflSize,
				thrust::raw_pointer_cast(&dev_rxPositions[i]), i, offset, layer, thrust::raw_pointer_cast(dev_rxPathNode.data() + oldPathNodeSize));//只捕获反射系列部分
			cudaerr = cudaDeviceSynchronize();
			if (cudaerr != cudaSuccess) {
				//LOG_ERROR << "PathTracingGPU: checkRxInsideKernel1D " << cudaGetErrorString(cudaerr) << ENDL;
			}
			//返回每个rx的路径节点值（多径），层数一致，与迭代深度相同
		}
		auto iterEnd = thrust::remove_if(dev_rxPathNode.begin() + oldPathNodeSize, dev_rxPathNode.end(), IsInValidPathNodeGPU());
		validRxNodeSize += static_cast<int>(thrust::distance(dev_rxPathNode.begin() + oldPathNodeSize, iterEnd));



		size_t numValidReflRay = 0;
		size_t numValidTranRay = 0;
		size_t numValidDiffRay = 0;
		size_t numEvaluateReflRay = newInterReflSize;								/** @brief	预估反射射线数量	*/
		size_t numEvaluateTranRay = newInterReflSize;								/** @brief	预估透射射线数量	*/
		size_t numEvaluateDiffRay = numInterDiff * diffractRayNum;							/** @brief	预估绕射射线数量	*/

		size_t evaluateRayNum = numEvaluateReflRay * static_cast<int>(stateHasRefl) +		/** @brief	预估射线总数	*/
							 numEvaluateTranRay * static_cast<int>(stateHasTran) +
							 numEvaluateDiffRay * static_cast<int>(stateHasDiff);
		if (dev_initRays.capacity() < evaluateRayNum) {
			dev_initRays.reserve(static_cast<int>(evaluateRayNum * 1.2));//扩大1.2预估内存
		}
		dev_initRays.resize(evaluateRayNum);
		ResetRay2DGPU(dev_initRays);//开始运行新射线前需要对原始射线有效性进行重置

		
		//将反射路径分为两类-一类为正常反射路径，一类为无效反射路径（反射终止路径），反射终止路径禁止加入下一步迭代中

		if (stateHasRefl) {//产生反射路径
			int numBlocks5 = (static_cast<int>(numEvaluateReflRay) * threadsPerBlock - 1) / threadsPerBlock;
			generateNewReflectionRayKernel CUDA_KERNEL(numBlocks5, threadsPerBlock)(thrust::raw_pointer_cast(dev_interReflSeries.data() + oldInterSize), newInterReflSize, oldInterSize,
				thrust::raw_pointer_cast(dev_initRays.data()), segmentsGPU);
			cudaerr = cudaDeviceSynchronize();
			if (cudaerr != cudaSuccess) {
				//LOG_ERROR << "PathTracingGPU: generateNewReflectionRayKernel " << cudaGetErrorString(cudaerr) << ENDL;
			}
			numValidReflRay = static_cast<int>(thrust::distance(dev_initRays.begin(), thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU())));//有效的反射路径数量
		}

		if (stateHasTran) {//产生透射路径
			int numBlocks6 = (static_cast<int>(numEvaluateTranRay) * threadsPerBlock - 1) / threadsPerBlock;
			generateNewTransmitRayKernel CUDA_KERNEL(numBlocks6, threadsPerBlock)(thrust::raw_pointer_cast(dev_interReflSeries.data() + oldInterSize), newInterReflSize, oldInterSize,
				thrust::raw_pointer_cast(dev_initRays.data() + numValidReflRay), segmentsGPU);
			cudaerr = cudaDeviceSynchronize();
			if (cudaerr != cudaSuccess) {
				//LOG_ERROR << "PathTracingGPU: generateNewTransmitRayKernel " << cudaGetErrorString(cudaerr) << ENDL;
			}
			numValidTranRay = static_cast<int>(thrust::distance(dev_initRays.begin() + numValidReflRay, thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU())));
			
		}

		if (stateHasDiff && numInterDiff != 0) {//产生绕射路径
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

		numRays = numValidReflRay + numValidTranRay + numValidDiffRay;//将新路径赋值给dev_initRay进行迭代
		dev_initRays.resize(numRays);
		layer++;
		//LOG_INFO << "ending new ray" << ENDL;
	}
	end = std::chrono::high_resolution_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	LOG_INFO << "total time: " << duration.count() << ENDL;
	//基于多层表中解析数据，得到多径数据
	std::vector<PathNodeGPU> host_rxPathNode(validRxNodeSize);

	
	std::vector<PathNodeGPU> hostPathNodes(validRxNodeSize);																		/** @brief	存储在本地的路径节点	*/
	thrust::copy(dev_rxPathNode.begin(), dev_rxPathNode.begin() + validRxNodeSize, hostPathNodes.begin());
	outPathNode.resize(validRxNodeSize);
	for (int i = 0; i < validRxNodeSize; ++i) {
		PathNodeGPU* newPathNode = new PathNodeGPU(hostPathNodes[i]);
		outPathNode[i] = newPathNode;
	}

	//释放显存数据，将多径数据导出处理
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

//考虑更新GPU算法
//1-增加经验透射						-完成
//2-增加透射后射线不分裂				-完成
//3-增加透射后不进行反射、绕射			-完成
//4-增加绕射后不进行透射				-完成
void PathTraceGPUOnlyTree(const std::vector<Ray2D>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, int diffractRayNum, Segment2DGPU* segmentsGPU, size_t numSegments, Wedge2DGPU* wedges, size_t numWedges, SignedDistanceFieldGPU* sdfGPU, std::vector<TreeNodeGPU*>& outTreeNodes)
{
	int threadsPerBlock = 128;

	std::chrono::high_resolution_clock::time_point start;
	std::chrono::high_resolution_clock::time_point end;
	std::chrono::milliseconds duration;
	//针对pathNodeTree进行处理

	//task-0 转化原始射线
	size_t numRays = rays.size();
	std::vector<Ray2DGPU> host_initRays(numRays);//执行射线的转换
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

	//涉及场景的转换


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



	thrust::device_vector<Ray2DGPU> dev_initRays;//禁止在循环体中对dev_newRays进行释放
	dev_initRays.reserve(numRays * 5);//预分配5倍内存
	dev_initRays.resize(numRays);
	dev_initRays = host_initRays;


	thrust::device_vector<Intersection2DGPU> dev_interInit(numRays);//原始射线相交结果，在循环中禁止对此数据进行删除或缩减操作

	thrust::device_vector<Intersection2DGPU> dev_interDiffSeries(numWedges * 1000);	/** @brief	当前层射线绕射点寻找总数，默认是1000倍原始绕射点数量	*/

	thrust::device_vector<int> everySplitNum(numRays); /** @brief	记录每条射线的分裂数目	*/
	thrust::device_vector<Ray2DGPU> dev_splitRays;		/** @brief	分裂的射线	*/

	thrust::device_vector<Intersection2DGPU> dev_InterSplit; /** @brief	分裂射线的交点信息	*/

	thrust::device_vector<Intersection2DGPU> dev_interReflSeries; /** @brief	合并的反射系列交点信息,包含所有层信息（原始保留射线交点信息+分裂射线的交点信息）,主要用于捕获rx	*/

	thrust::device_vector<TreeNodeGPU> dev_treeNodes;				/** @brief	射线树节点数组	*/

	int validTreeNodeNum = 0;										/** @brief	有效的射线树节点数量	*/

	cudaError_t cudaerr;										/** @brief	cuda错误处理	*/

	int layer = 0;
	
	while (dev_initRays.size() != 0) {
		//task-1 ---------------并行计算射线相交射线内的绕射点(修缮完成)----------------------------------------------------------------------------------------------

		numRays = dev_initRays.size();

		int numInterDiff = 0;
		if (stateHasDiff) {
			size_t numBlocks1 = (numRays + threadsPerBlock - 1) / threadsPerBlock;
			ResetIntersect2DGPU(dev_interDiffSeries);
			cudaMemcpyToSymbol(G_DIFFID, &C_ZERO, sizeof(int));//初始化原子索引

			for (int i = 0; i < numWedges; ++i) {//若射线中无绕射生命，则放弃
				diffractionFindKernel1D CUDA_KERNEL(numBlocks1, threadsPerBlock)(thrust::raw_pointer_cast(dev_initRays.data()), numRays,
					&wedges[i], sdfGPU, segmentsGPU, thrust::raw_pointer_cast(dev_interDiffSeries.data()));
				cudaerr = cudaDeviceSynchronize();
				if (cudaerr != cudaSuccess) {
					//LOG_ERROR << "PathTracingGPU: diffractionFindKernel1D " << cudaGetErrorString(cudaerr) << ENDL;
				}
			}
			auto iter_end = thrust::remove_if(dev_interDiffSeries.begin(), dev_interDiffSeries.end(), IsInValidIntersectGPU());
			numInterDiff = static_cast<int>(thrust::distance(dev_interDiffSeries.begin(), iter_end));//计算有效数据偏离值
			//LOG_INFO <<layer<<": " << numInterDiff << ENDL;
			//LOG_INFO << "end find diff" << ENDL;
		}



		//task-2----------------- 并行计算射线分裂---------------------------------------------------------------------------------------------------------
		//2-1	预计算分裂个数
		dev_interInit.resize(numRays);//扩展为射线数量
		ResetIntersect2DGPU(dev_interInit);///--dev_initIntersects进行初始化

		//计算原始射线交点信息及射线分裂数量，保留具有分裂属性的原始射线，保留不具备分裂属性的交点信息
		size_t numInterNoSplit = numRays;
		size_t numInterSplit = 0;
		size_t numRaySplit = 0;
		if (raySplitFlag == true) {
			size_t numBlocks2 = (numRays + threadsPerBlock - 1) / threadsPerBlock;
			everySplitNum.resize(numRays);
			thrust::fill(everySplitNum.begin(), everySplitNum.end(), 0);//重置每条射线分裂数量
			intersectAndCalculateRaySplitNumKernelLBS CUDA_KERNEL(numBlocks2, threadsPerBlock)(thrust::raw_pointer_cast(dev_initRays.data()),
				thrust::raw_pointer_cast(dev_interInit.data()), thrust::raw_pointer_cast(everySplitNum.data()), sdfGPU, segmentsGPU, numRays);
			cudaerr = cudaDeviceSynchronize();//等待线程计算完成
			if (cudaerr != cudaSuccess) {
				//LOG_ERROR << "PathTracingGPU: intersectAndCalculateRaySplitNumKernel " << cudaGetErrorString(cudaerr) << ENDL;
			}

			//保留有效的交点信息-供后续捕获及新射线使用
			auto iterEnd_validInter = thrust::remove_if(dev_interInit.begin(), dev_interInit.end(), IsInValidIntersectGPU());
			numInterNoSplit = static_cast<int>(thrust::distance(dev_interInit.begin(), iterEnd_validInter));//未分裂的原始交点信息数目

			//保留有效的射线-供分裂使用
			auto iterEnd_validRay = thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU());
			numInterSplit = static_cast<int>(thrust::distance(dev_initRays.begin(), iterEnd_validRay));//已分裂的原始交点信息数目

			//将分裂数目不为0的前置
			auto iterEnd_validEverySplitNum = thrust::remove_if(everySplitNum.begin(), everySplitNum.end(), is_zero());//将分裂的射线与原始的分裂数组进行计算分裂射线，得到splitRays

			//计算分裂射线数目总量
			numRaySplit = thrust::reduce(everySplitNum.begin(), iterEnd_validEverySplitNum, 0);

			if (numRaySplit != 0) {													//若预计算分裂数量不为0，则进行真实分裂计算
				dev_splitRays.resize(numRaySplit);//将分裂的射线进行resize
				ResetRay2DGPU(dev_splitRays);

				size_t numblocks_split = (numInterSplit + threadsPerBlock - 1) / threadsPerBlock;//注意，这里的原始可分裂射线已经前置，请利用匹配思想进行处理
				raySplitKernel CUDA_KERNEL(numblocks_split, threadsPerBlock)(thrust::raw_pointer_cast(dev_initRays.data()), numInterSplit,
					thrust::raw_pointer_cast(dev_splitRays.data()), thrust::raw_pointer_cast(everySplitNum.data()), segmentsGPU);
				cudaerr = cudaDeviceSynchronize();
				if (cudaerr != cudaSuccess) {
					//LOG_ERROR << "PathTracingGPU: raySplitKernel " << cudaGetErrorString(cudaerr) << ENDL;
				}

				//保留有效的分裂射线
				auto interEnd_validSplitRay = thrust::remove_if(dev_splitRays.begin(), dev_splitRays.end(), IsInValidRayGPU());
				numRaySplit = static_cast<int>(thrust::distance(dev_splitRays.begin(), interEnd_validSplitRay));//更新分裂射线的数目
				//task-3------------------ 并行计算分裂射线相交------------------------------------------------------------------------------------------------------
				dev_InterSplit.resize(numRaySplit);//对分裂的射线交点信息进行resize
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


		//task-4------------------ 合并分裂/未分裂射线的Inter 结果--------------------------------------------------------------------------------------------
		size_t oldInterSize = dev_interReflSeries.size(); //上一层交点信息数量
		size_t newInterReflSize = numInterNoSplit + numRaySplit;// 当前层的反射系列交点信息数量
		size_t newInterDiffSize = numInterDiff;   //当前层的绕射系列的交点信息数量

		dev_interReflSeries.resize(oldInterSize + newInterReflSize + newInterDiffSize);//对反射系交点信息进行resize

		//从未分裂射线交点信息和分裂射线交点信息中读取数据
		thrust::copy(dev_interInit.begin(), dev_interInit.begin() + numInterNoSplit, dev_interReflSeries.begin() + oldInterSize);											//将初始未分裂交点附加至反射系列交点
		thrust::copy(dev_InterSplit.begin(), dev_InterSplit.begin() + numRaySplit, dev_interReflSeries.begin() + oldInterSize + numInterNoSplit);							//将分裂交点附加至反射系列交点后
		thrust::copy(dev_interDiffSeries.begin(), dev_interDiffSeries.begin() + newInterDiffSize, dev_interReflSeries.begin() + oldInterSize + numInterNoSplit + numRaySplit);	//将绕射系交点附加至反射系列交点之后

		size_t numValidReflRay = 0;
		size_t numValidTranRay = 0;
		size_t numValidDiffRay = 0;
		size_t numEvaluateReflRay = newInterReflSize;								/** @brief	预估反射射线数量	*/
		size_t numEvaluateTranRay = newInterReflSize;								/** @brief	预估透射射线数量	*/
		size_t numEvaluateDiffRay = numInterDiff * diffractRayNum;							/** @brief	预估绕射射线数量	*/

		size_t evaluateRayNum = numEvaluateReflRay * static_cast<int>(stateHasRefl) +		/** @brief	预估射线总数	*/
			numEvaluateTranRay * static_cast<int>(stateHasTran) +
			numEvaluateDiffRay * static_cast<int>(stateHasDiff);
		if (dev_initRays.capacity() < evaluateRayNum) {
			dev_initRays.reserve(static_cast<int>(evaluateRayNum * 1.2));//扩大1.2预估内存
		}
		dev_initRays.resize(evaluateRayNum);
		ResetRay2DGPU(dev_initRays);//开始运行新射线前需要对原始射线有效性进行重置

		//增加考虑射线树节点数据
		size_t oldTreeNodeSize = dev_treeNodes.size();								/** @brief	上一层次的树节点的数量	*/
		size_t newTreeNodeSize = oldTreeNodeSize + evaluateRayNum;					/** @brief	新的树节点的数量	*/

		dev_treeNodes.resize(newTreeNodeSize);

		//将反射路径分为两类-一类为正常反射路径，一类为无效反射路径（反射终止路径），反射终止路径禁止加入下一步迭代中

		if (stateHasRefl) {//产生反射路径
			size_t numBlocks5 = (numEvaluateReflRay * threadsPerBlock - 1) / threadsPerBlock;
			generateNewReflectionRayWithNodeKernel CUDA_KERNEL(numBlocks5, threadsPerBlock)(thrust::raw_pointer_cast(dev_interReflSeries.data() + oldInterSize), newInterReflSize, oldInterSize,
				thrust::raw_pointer_cast(dev_initRays.data()), thrust::raw_pointer_cast(dev_treeNodes.data() + oldInterSize), segmentsGPU, layer);
			cudaerr = cudaDeviceSynchronize();
			if (cudaerr != cudaSuccess) {
				//LOG_ERROR << "PathTracingGPU: generateNewReflectionRayKernel " << cudaGetErrorString(cudaerr) << ENDL;
			}
			numValidReflRay = static_cast<int>(thrust::distance(dev_initRays.begin(), thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU())));//有效的反射路径数量
		}

		if (stateHasTran) {//产生透射路径
			size_t numBlocks6 = (numEvaluateTranRay * threadsPerBlock - 1) / threadsPerBlock;
			generateNewTransmitRayWithNodeKernel CUDA_KERNEL(numBlocks6, threadsPerBlock)(thrust::raw_pointer_cast(dev_interReflSeries.data() + oldInterSize), newInterReflSize, oldInterSize,
				thrust::raw_pointer_cast(dev_initRays.data() + numValidReflRay), thrust::raw_pointer_cast(dev_treeNodes.data() + oldInterSize + numValidReflRay), segmentsGPU, layer);
			cudaerr = cudaDeviceSynchronize();
			if (cudaerr != cudaSuccess) {
				LOG_ERROR << "PathTracingGPU: generateNewTransmitRayKernel " << cudaGetErrorString(cudaerr) << ENDL;
			}
			numValidTranRay = static_cast<int>(thrust::distance(dev_initRays.begin() + numValidReflRay, thrust::remove_if(dev_initRays.begin(), dev_initRays.end(), IsInValidRayGPU())));

		}

		if (stateHasDiff && numInterDiff != 0) {//产生绕射路径
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

		numRays = numValidReflRay + numValidTranRay + numValidDiffRay;//将新路径赋值给dev_initRay进行迭代
		dev_initRays.resize(numRays);

		validTreeNodeNum += static_cast<int>(thrust::distance(dev_treeNodes.begin() + oldTreeNodeSize, thrust::remove_if(dev_treeNodes.begin(), dev_treeNodes.end(), IsInValidTreeNodeGPU())));
		dev_treeNodes.resize(validTreeNodeNum);			//有多少射线就会有多少个树节点
		layer++;
	}
	end = std::chrono::high_resolution_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	//LOG_INFO << "total time: " << duration.count() << ENDL;

	//拷贝节点数据至CPU中
	std::vector<TreeNodeGPU> hostTreeNodes(validTreeNodeNum);																		/** @brief	存储在本地的路径节点	*/
	thrust::copy(dev_treeNodes.begin(), dev_treeNodes.begin() + validTreeNodeNum, hostTreeNodes.begin());
	outTreeNodes.resize(validTreeNodeNum);
	for (int i = 0; i < validTreeNodeNum; ++i) {
		TreeNodeGPU* newTreeNode = new TreeNodeGPU(hostTreeNodes[i]);
		outTreeNodes[i] = newTreeNode;
	}


	//释放显存数据，将多径数据导出处理
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
	//主体思想是基于原始射线，结合透射修正精度，通过原始射线分裂方法逐步逼近真实的透射rx坐标点位置,注意，这里的修正模式为正向修正模式
	//构造射线,计算半张角
	std::vector<Ray2DGPU> splitRays; //需要分裂的射线
	const std::vector<PathNodeGPU*>& nodes = inpath.m_nodes;//路径节点信息

	PathNodeGPU* frontNode = nodes.back();//注意，这里修改为正序，从广义源节点传播至目标点
	PathNodeGPU* backNode = nodes.front();
	const PathNodeGPU* secondNode = *prev(nodes.end());//由于需要倒序，这里的第二个节点为倒数第二个节点
	const Ray2DGPU& initRay = secondNode->m_inter.m_ray;
	//求解到目标点的射线半径r
	RtLbsType t = backNode->m_inter.m_ray.m_tMax - backNode->m_inter.m_ray.m_tMin + (backNode->m_inter.m_intersect - backNode->m_inter.m_ray.m_Ori).Length();
	RtLbsType r = initRay.GetRayRadis(t);
	int splitNum = static_cast<int>(ceil(r / TRAN_EPSILON));
	double theta = initRay.m_theta * 2.0;
	double dtheta = theta / splitNum;
	double halftheta = dtheta / 2.0;
	double coshalftheta = cos(halftheta);
	Vector2D newDir = Rotate(initRay.m_Dir, -initRay.m_theta);//旋转到方向最小值
	splitRays.resize(splitNum);
	//射线的初始化
	newDir.Rotate(halftheta);//第一条射线
	splitRays[0] = initRay;
	splitRays[0].m_Dir = newDir;
	splitRays[0].m_theta = halftheta;
	splitRays[0].m_costheta = coshalftheta;

	for (size_t i = 1; i < splitRays.size(); ++i) {//后续射线
		newDir.Rotate(dtheta);
		splitRays[i] = initRay;
		splitRays[i].m_Dir = newDir;
		splitRays[i].m_theta = halftheta;
		splitRays[i].m_costheta = coshalftheta;
	}

	std::vector<RayPathGPU> outpaths;//输出的路径
	std::vector<Ray2DGPU> nextrays;//末尾节点后的射线
	//计算并跟踪射线
	for (Ray2DGPU& splitray : splitRays) {//遍历所有射线
		RayPathGPU newPath;
		newPath.m_nodes.push_back(frontNode);
		Ray2DGPU iterateRay(splitray); //用于迭代的射线
		auto it = prev(nodes.end(), 2);
		while (it != nodes.begin()) {//刨除首尾元素，遍历节点并进行射线追踪
			PathNodeGPU* node = *it;
			Intersection2DGPU inter;
			const Segment2DGPU& segment = segments[node->m_inter.m_segmentId];
			if (!segment.GetIntersect(iterateRay, &inter)) //若射线与节点面元不相交，放弃追踪路线
				break;
			if (node->m_inter.m_type == NODE_REFL) {//反射节点
				Ray2DGPU reflectRay;
				if(!GenerateReflectRayGPU(node->m_inter, 0, segments, &reflectRay))//若产生不了反射路径，则放弃该条射线
					break;
				PathNodeGPU* newNode = new PathNodeGPU(true, node->m_layer, node->m_rxId, inter);
				newPath.m_nodes.push_back(newNode);
				iterateRay = reflectRay;
			}
			if (node->m_inter.m_type == NODE_TRANIN || node->m_inter.m_type == NODE_TRANOUT) {//透射节点
				Ray2DGPU transmitRay;
				if (!GenerateTransmitRayGPU(node->m_inter, 0, segments, &transmitRay))
					break;
				PathNodeGPU* newNode = new PathNodeGPU(true, node->m_layer, node->m_rxId, inter);
				newPath.m_nodes.push_back(newNode);
				iterateRay = transmitRay;
			}
			if (node->m_inter.m_type == NODE_ETRANIN || node->m_inter.m_type == NODE_ETRANOUT) {//透射节点
				Ray2DGPU transmitRay;
				if (!GenerateEmpiricalTransmitRayGPU(node->m_inter, 0, segments, &transmitRay))
					break;
				PathNodeGPU* newNode = new PathNodeGPU(true, node->m_layer, node->m_rxId, inter);
				newPath.m_nodes.push_back(newNode);
				iterateRay = transmitRay;
			}
			--it;
		}
		
		if (newPath.m_nodes.size() != (inpath.m_nodes.size() - 1))//刨除末尾元素后的新路径若数量与原始射线路径不相同，则舍弃无效路径
			continue;
		outpaths.push_back(newPath);
		nextrays.push_back(iterateRay);
	}
	if (outpaths.empty())//若当前路径集合为空，表明路径不存在
		return false;

	//搜索距离rx最近的路径
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

