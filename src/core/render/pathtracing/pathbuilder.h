#ifndef RTLBS_PATHBUILDER
#define RTLBS_PATHBUILDER

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "math/point2d.h"
#include "math/point3d.h"
#include "equipment/transceiver/transmitter.h"
#include "equipment/transceiver/receiver.h"
#include "scene/scene.h"
#include "radiowave/raypath/raytreenode.h"
#include "radiowave/raypath/pathnode.h"
#include "radiowave/raypath/pathnode3d.h"
#include "radiowave/raypath/raypath.h"
#include "radiowave/raypath/raypath3d.h"
#include "radiowave/raypath/terraindiffractionpath.h"
#include "managers/logmanager.h"
#include "parallel/threadpool.h"
#include "radiowave/raypath/gpu/pathnodegpu.h"
#include "radiowave/raypath/gpu/raypathgpu.h"
#include "core/render/pathtracing/treenodegenerator.h"
#include "parallel/threadpool.h"
#include "core/raypathrectifier/raypathrectifier.h"



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




//输出单个tx对所有rx坐标点的多径
void GenerateMultiPathofRxSingleTxGPU(const std::vector<PathNodeGPU*>& nodes, const Point2D& txPosition, const std::vector<Point2D>& rxPositions, const Segment2DGPU* segments, const Scene* scene, std::vector<std::vector<RayPathGPU*>>& outRayPath);
//基于GPU运算的一维nodes节点解算出每个rx上的坐标值
void GenerateMultiPathofRxGPU(std::vector<std::vector<PathNodeGPU*>> nodes, std::vector<Point2D> txPositions, std::vector<Point2D> rxPositions, Segment2DGPU* segments, const Scene* scene, std::vector<std::vector<std::vector<RayPathGPU*>>>& outRayPaths);



#endif
