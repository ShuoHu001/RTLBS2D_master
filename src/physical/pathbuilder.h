#ifndef RTLBS_PATHBUILDER
#define RTLBS_PATHBUILDER

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/point2d.h"
#include "geometry/point3d.h"
#include "equipment/transmitter.h"
#include "equipment/receiver.h"
#include "geometry/scene.h"
#include "tree/raytreenode.h"
#include "tree/pathnode.h"
#include "tree/pathnode3d.h"
#include "tree/raypath.h"
#include "tree/raypath3d.h"
#include "tree/terraindiffractionpath.h"
#include "managers/logmanager.h"
#include "parallel/threadpool.h"
#include "tree/gpu/pathnodegpu.h"
#include "tree/gpu/raypathgpu.h"
#include "physical/treenodegenerator.h"
#include "parallel/threadpool.h"
#include "localization/gspaircluster.h"


class Result;
class RaytracingResult;

//CPU单核路径收集
void PathBuilder_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, Result& result);

//CPU多核路径收集
void PathBuilder_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, int16_t threadNum, Result& result);

//GPU多核路径收集
void PathBuilder_GPUMultiThread(const std::vector<std::vector<PathNodeGPU*>>& gpuPathNodes, const Scene* scene, Result& result);

//调试-输出多径信息
void PathBuilder_DEBUG(const std::vector<RayTreeNode*>& vroots, const Scene* scene);

//实时计算射线追踪路径信息（CPU单线程）-LBS方法用到
void DirectlySetResultPath_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, GSPairCluster* cluster);

//实时计算射线追踪路径信息 （CPU多线程）-LBS方法用到
void DirectlySetResultPath_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, uint16_t threadNum, std::vector<GSPairCluster>& clusters);





#endif
