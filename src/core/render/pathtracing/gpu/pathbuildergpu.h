#ifndef RTLBS_PATHBUILDERGPU
#define RTLBS_PATHBUILDERGPU

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "managers/logmanager.h"
#include "radiowave/raypath/raytreenode.h"
#include "scene/scene.h"
#include "result/raytracingresult.h"
#include "radiowave/raypath/gpu/cpuconverterpathnode.h"
#include "core/render/pathtracing/treenodegenerator.h"
#include "radiowave/raypath/raypath.h"
#include "radiowave/raypath/raypath3d.h"
#include "localization/gspaircluster.h"
#include "result/result.h"
#include "core/raypathrectifier/raypathrectifier.h"


__global__ void CheckTargetInsideKernel_PathBuilder(CPUConverterPathNode* allNodes, int numNodes, Point2D* target, int* nodeIds);


__global__ void FindPathNodeKernel_PathBuilder(CPUConverterPathNode* allNodes, int numNodes, int* nodeIds, int numId, int* pathNodeIds, int numPathId);



//实时计算射线追踪路径信息（GPU多线程-CPU单线程）-RT方法
void PathBuilder_CPUGPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, Result& result);

#endif
