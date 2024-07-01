#ifndef RTLBS_PATHBUILDERGPU
#define RTLBS_PATHBUILDERGPU

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "managers/logmanager.h"
#include "tree/raytreenode.h"
#include "geometry/scene.h"
#include "result/raytracingresult.h"
#include "tree/gpu/cpuconverterpathnode.h"
#include "physical/treenodegenerator.h"
#include "tree/raypath.h"
#include "tree/raypath3d.h"
#include "localization/gspaircluster.h"

__global__ void CheckTargetInsideKernel(CPUConverterPathNode* allNodes, int numNodes, Point2D* target, int* nodeIds);


__global__ void FindPathNodeKernel(CPUConverterPathNode* allNodes, int numNodes, int* nodeIds, int numId, int* pathNodeIds, int numPathId);

//实时计算射线追踪路径信息（GPU多线程）- LBS方法用到
void DirectlySetResultPath_GPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRaidus, std::vector<GSPairCluster>& clusters);

#endif
