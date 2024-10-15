#ifndef RTLBS_RAYPATHSETTERGPU
#define RTLBS_RAYPATHSETTERGPU

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "core/render/pathtracing/gpu/pathbuildergpu.h"

__global__ void CheckTargetInsideKernel_RayPathSetter(CPUConverterPathNode* allNodes, int numNodes, Point2D* target, int* nodeIds);


__global__ void FindPathNodeKernel_RayPathSetter(CPUConverterPathNode* allNodes, int numNodes, int* nodeIds, int numId, int* pathNodeIds, int numPathId);


//ʵʱ��������׷��·����Ϣ��GPU���̣߳�- LBS�����õ�
void DirectlySetResultPath_GPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRaidus, std::vector<GSPairCluster>& clusters);

#endif
