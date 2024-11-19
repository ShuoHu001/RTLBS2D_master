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

//CPU����·���ռ�
void PathBuilder_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, Result& result);

//CPU���·���ռ�
void PathBuilder_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, int16_t threadNum, Result& result);

//GPU���·���ռ�
void PathBuilder_GPUMultiThread(const std::vector<std::vector<PathNodeGPU*>>& gpuPathNodes, const Scene* scene, Result& result);

//����-����ྶ��Ϣ
void PathBuilder_DEBUG(const std::vector<RayTreeNode*>& vroots, const Scene* scene);




//�������tx������rx�����Ķྶ
void GenerateMultiPathofRxSingleTxGPU(const std::vector<PathNodeGPU*>& nodes, const Point2D& txPosition, const std::vector<Point2D>& rxPositions, const Segment2DGPU* segments, const Scene* scene, std::vector<std::vector<RayPathGPU*>>& outRayPath);
//����GPU�����һάnodes�ڵ�����ÿ��rx�ϵ�����ֵ
void GenerateMultiPathofRxGPU(std::vector<std::vector<PathNodeGPU*>> nodes, std::vector<Point2D> txPositions, std::vector<Point2D> rxPositions, Segment2DGPU* segments, const Scene* scene, std::vector<std::vector<std::vector<RayPathGPU*>>>& outRayPaths);



#endif
